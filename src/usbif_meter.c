// SPDX-License-Identifier: MIT
//
// Band levels for a spectrum meter, computed where the sound card's bytes
// already pass: the C pump task (usbif_i2s.c). Spike code (the audio meter
// spike, 2026-09-25).
//
// Python never sees a sample. It asks for N log-spaced bands between two
// frequencies with uac_pump_meter(), and reads the latest levels at its own
// frame rate with uac_pump_levels(). The pump feeds this file every block it
// reads, and about 60 times a second runs the analysis inline.
//
// The method is two FFTs, because one can't be both quick and sharp at the
// bottom of a 20 Hz..20 kHz log scale:
//
// - 1024 points at the host rate (21 ms, 47 Hz bins) for every band centred
//   above ~400 Hz;
// - 512 points on a copy low-passed at 700 Hz (6th-order Butterworth) and
//   decimated by 16 (3 kHz, 171 ms, 5.9 Hz bins) for the bands below.
//
// A band's level is the power of the bins it covers, in dB relative to a
// full-scale sine. A band narrower than a bin takes the interpolated bin
// power scaled by its width, so pink noise reads flat across the whole scale.
// The meter is an eye-candy source, not an instrument: no calibration beyond
// that, and levels go out as one byte each in half-dB steps.

#include "py/mpconfig.h"

#if MICROPY_HW_ENABLE_USBDEV
#include "tusb.h"
#endif

#if defined(CFG_TUD_AUDIO) && CFG_TUD_AUDIO

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_private/esp_clk.h"

#include "usbif_meter.h"

#define HI_N (1024)
#define LO_N (512)
#define LO_DEC (16)
#define METER_HZ (60)
#define LP_HZ (700.0f)
#define SPLIT_HZ (400.0f)
#define NSEC (3)

typedef struct {
    float b0, b1, b2, a1, a2;
    float z1, z2;
} biquad_t;

// One band: which FFT, and either a bin range to sum or a point to
// interpolate at (k1 == 0) with a width scale.
typedef struct {
    uint8_t lo;       // 1: the decimated FFT
    uint16_t k0, k1;  // sum bins k0..k1-1; k1 == 0 means interpolate
    float pos;        // interpolation point in bins
    float scale;      // band width over bin width, for interpolated bands
} band_t;

static struct {
    // Requested from Python; picked up by the pump at its next block.
    volatile bool want_config;
    uint8_t want_bands;
    float want_lo_hz, want_hi_hz;

    bool enabled;
    uint8_t nbands;
    float lo_hz, hi_hz;
    uint32_t rate;

    float *hi_ring, *lo_ring;
    uint32_t hi_pos, lo_pos;
    float *win_hi, *win_lo;
    float *re, *im, *pw;
    float *tw_cos, *tw_sin;
    float norm_hi, norm_lo;
    biquad_t lp[NSEC];
    uint32_t dec_phase;
    uint32_t hop, hop_count;
    float peak, sumsq;
    uint32_t nsum;
    band_t band[USBIF_METER_MAX_BANDS];

    // Published: a sequence lock (odd while writing) and the levels.
    volatile uint32_t seq;
    uint8_t levels[USBIF_METER_MAX_BANDS];
    uint8_t peak_b, rms_b;

    // Cost, in CPU cycles, split into the per-block feed and the analysis.
    uint64_t feed_cycles, fft_cycles;
    uint32_t analyses, max_fft_cycles;
    int64_t t0;
} m;

static void *meter_alloc(size_t n) {
    void *p = heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!p) {
        p = malloc(n);
    }
    return p;
}

// Buffers are taken once, from the caller's context (never the pump's), and
// kept: the pump may be mid-block when Python turns the meter off.
bool usbif_meter_alloc(void) {
    if (m.hi_ring) {
        return true;
    }
    float *hr = meter_alloc(HI_N * sizeof(float));
    float *lr = meter_alloc(LO_N * sizeof(float));
    float *wh = meter_alloc(HI_N * sizeof(float));
    float *wl = meter_alloc(LO_N * sizeof(float));
    float *re = meter_alloc(HI_N * sizeof(float));
    float *im = meter_alloc(HI_N * sizeof(float));
    float *tc = meter_alloc(HI_N / 2 * sizeof(float));
    float *ts = meter_alloc(HI_N / 2 * sizeof(float));
    float *pw = meter_alloc(HI_N / 2 * sizeof(float));
    if (!hr || !lr || !wh || !wl || !re || !im || !tc || !ts || !pw) {
        free(hr), free(lr), free(wh), free(wl), free(re), free(im), free(tc), free(ts), free(pw);
        return false;
    }
    memset(hr, 0, HI_N * sizeof(float));
    memset(lr, 0, LO_N * sizeof(float));
    float s2 = 0;
    for (int i = 0; i < HI_N; i++) {
        wh[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / HI_N);
        s2 += wh[i] * wh[i];
    }
    m.norm_hi = 2.0f / (HI_N * s2);
    s2 = 0;
    for (int i = 0; i < LO_N; i++) {
        wl[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / LO_N);
        s2 += wl[i] * wl[i];
    }
    m.norm_lo = 2.0f / (LO_N * s2);
    for (int i = 0; i < HI_N / 2; i++) {
        tc[i] = cosf(2.0f * (float)M_PI * i / HI_N);
        ts[i] = -sinf(2.0f * (float)M_PI * i / HI_N);
    }
    m.lo_ring = lr, m.win_hi = wh, m.win_lo = wl, m.re = re, m.im = im;
    m.tw_cos = tc, m.tw_sin = ts, m.pw = pw;
    m.hi_ring = hr;  // last: its presence says the rest is ready
    return true;
}

void usbif_meter_configure(int bands, float lo_hz, float hi_hz) {
    if (bands > USBIF_METER_MAX_BANDS) {
        bands = USBIF_METER_MAX_BANDS;
    }
    m.want_bands = (uint8_t)(bands < 0 ? 0 : bands);
    m.want_lo_hz = lo_hz;
    m.want_hi_hz = hi_hz;
    m.want_config = true;
}

// Butterworth low-pass sections (RBJ biquads at the Butterworth Qs).
static void meter_design(uint32_t rate) {
    static const float q[NSEC] = { 0.51764f, 0.70711f, 1.93185f };
    const float w0 = 2.0f * (float)M_PI * LP_HZ / (float)rate;
    const float c = cosf(w0), s = sinf(w0);
    for (int i = 0; i < NSEC; i++) {
        const float alpha = s / (2.0f * q[i]);
        const float a0 = 1.0f + alpha;
        biquad_t *b = &m.lp[i];
        b->b0 = (1.0f - c) / 2.0f / a0;
        b->b1 = (1.0f - c) / a0;
        b->b2 = b->b0;
        b->a1 = -2.0f * c / a0;
        b->a2 = (1.0f - alpha) / a0;
        b->z1 = b->z2 = 0;
    }
    m.hop = rate / METER_HZ;
    m.hop_count = 0;
    m.dec_phase = 0;

    const float r = powf(m.hi_hz / m.lo_hz, 1.0f / m.nbands);
    const float lo_rate = (float)rate / LO_DEC;
    for (int i = 0; i < m.nbands; i++) {
        const float f0 = m.lo_hz * powf(r, (float)i);
        const float f1 = f0 * r;
        const float fc = sqrtf(f0 * f1);
        band_t *b = &m.band[i];
        b->lo = (fc < SPLIT_HZ);
        const float w = b->lo ? lo_rate / LO_N : (float)rate / HI_N;
        const int nbins = b->lo ? LO_N / 2 : HI_N / 2;
        int k0 = (int)ceilf(f0 / w);
        int k1 = (int)ceilf(f1 / w);
        if (k1 > nbins) {
            k1 = nbins;
        }
        if (k1 - k0 >= 1 && k0 >= 1) {
            b->k0 = (uint16_t)k0;
            b->k1 = (uint16_t)k1;
        } else {
            b->k0 = 0;
            b->k1 = 0;
            b->pos = fc / w;
            if (b->pos > nbins - 2) {
                b->pos = (float)(nbins - 2);
            }
            b->scale = (f1 - f0) / w;
        }
    }
}

// In-place iterative radix-2 FFT on m.re/m.im, n a power of two <= HI_N / 2
// (it runs at half the real length; see meter_real_power).
static void IRAM_ATTR meter_fft(int n) {
    float *re = m.re, *im = m.im;
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            float t = re[i];
            re[i] = re[j];
            re[j] = t;
            t = im[i];
            im[i] = im[j];
            im[j] = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        const int half = len >> 1;
        const int stride = HI_N / len;
        for (int i = 0; i < n; i += len) {
            for (int k = 0; k < half; k++) {
                const float wr = m.tw_cos[k * stride];
                const float wi = m.tw_sin[k * stride];
                const int a = i + k, b = a + half;
                const float xr = re[b] * wr - im[b] * wi;
                const float xi = re[b] * wi + im[b] * wr;
                re[b] = re[a] - xr;
                im[b] = im[a] - xi;
                re[a] += xr;
                im[a] += xi;
            }
        }
    }
}

// A real FFT of length n from the half-length complex one: the window put
// even samples in re and odd ones in im, meter_fft(n / 2) ran, and this
// untangles the two and leaves |X[k]|^2 in m.pw for k < n / 2.
static void IRAM_ATTR meter_real_power(int n) {
    const float *re = m.re, *im = m.im;
    float *pw = m.pw;
    const int h = n / 2;
    const int stride = HI_N / n;
    pw[0] = 0;
    for (int k = 1; k < h; k++) {
        const float ar = re[k], ai = im[k];
        const float br = re[h - k], bi = im[h - k];
        const float er = 0.5f * (ar + br), ei = 0.5f * (ai - bi);
        const float xr = 0.5f * (ai + bi), xi = -0.5f * (ar - br);
        const float c = m.tw_cos[k * stride], s = m.tw_sin[k * stride];
        const float r = er + c * xr - s * xi;
        const float i = ei + c * xi + s * xr;
        pw[k] = r * r + i * i;
    }
}

static inline uint8_t meter_db_byte(float power) {
    // Half-dB steps, 0 = -100 dB or quieter, 200 = 0 dB.
    if (power <= 1e-10f) {
        return 0;
    }
    float v = 2.0f * (10.0f * log10f(power) + 100.0f);
    if (v < 0) {
        return 0;
    }
    if (v > 255) {
        return 255;
    }
    return (uint8_t)(v + 0.5f);
}

static float IRAM_ATTR band_power(const band_t *b, float norm) {
    const float *pw = m.pw;
    if (b->k1) {
        float s = 0;
        for (int k = b->k0; k < b->k1; k++) {
            s += pw[k];
        }
        return s * norm;
    }
    const int k = (int)b->pos;
    const float f = b->pos - k;
    return (pw[k] + (pw[k + 1] - pw[k]) * f) * b->scale * norm;
}

static void IRAM_ATTR meter_window(const float *ring, uint32_t pos, const float *win, int n) {
    float *re = m.re, *im = m.im;
    const uint32_t mask = (uint32_t)n - 1;
    for (int i = 0; i < n / 2; i++) {
        re[i] = ring[(pos + 2 * i) & mask] * win[2 * i];
        im[i] = ring[(pos + 2 * i + 1) & mask] * win[2 * i + 1];
    }
}

static uint8_t out_levels[USBIF_METER_MAX_BANDS];

static void IRAM_ATTR meter_analyse(void) {
    uint8_t *out = out_levels;
    // The fast FFT for the upper bands.
    meter_window(m.hi_ring, m.hi_pos, m.win_hi, HI_N);
    meter_fft(HI_N / 2);
    meter_real_power(HI_N);
    for (int i = 0; i < m.nbands; i++) {
        if (!m.band[i].lo) {
            out[i] = meter_db_byte(band_power(&m.band[i], m.norm_hi) * 2.0f);
        }
    }
    // The slow, sharp one for the bottom. Its window is 171 ms long, so
    // every other hop is plenty.
    if (m.band[0].lo && (m.analyses & 1) == 0) {
        meter_window(m.lo_ring, m.lo_pos, m.win_lo, LO_N);
        meter_fft(LO_N / 2);
        meter_real_power(LO_N);
        for (int i = 0; i < m.nbands; i++) {
            if (m.band[i].lo) {
                out[i] = meter_db_byte(band_power(&m.band[i], m.norm_lo) * 2.0f);
            }
        }
    }
    const uint8_t pk = meter_db_byte(m.peak * m.peak);
    const uint8_t rms = meter_db_byte(m.nsum ? m.sumsq / m.nsum : 0);
    m.peak = 0;
    m.sumsq = 0;
    m.nsum = 0;

    m.seq++;
    __sync_synchronize();
    memcpy(m.levels, out, m.nbands);
    m.peak_b = pk;
    m.rms_b = rms;
    __sync_synchronize();
    m.seq++;
}

// Called by the pump with each block it read, before it converts the block in
// place: interleaved int16 frames at the host's rate.
void IRAM_ATTR usbif_meter_feed(const int16_t *frames, uint32_t n, uint32_t channels, uint32_t rate) {
    if (m.want_config) {
        m.want_config = false;
        m.nbands = m.want_bands;
        m.lo_hz = m.want_lo_hz;
        m.hi_hz = m.want_hi_hz;
        m.enabled = (m.nbands > 0) && m.hi_ring;
        m.rate = 0;  // redesign below
        m.feed_cycles = m.fft_cycles = 0;
        m.analyses = m.max_fft_cycles = 0;
        m.t0 = esp_timer_get_time();
    }
    if (!m.enabled || n == 0 || rate == 0) {
        return;
    }
    const uint32_t c0 = esp_cpu_get_cycle_count();
    if (rate != m.rate) {
        m.rate = rate;
        meter_design(rate);
    }
    const float k = 1.0f / 32768.0f;
    float pk = m.peak, ss = m.sumsq;
    uint32_t hp = m.hi_pos, lp = m.lo_pos, ph = m.dec_phase;
    float *hr = m.hi_ring, *lr = m.lo_ring;
    // The three low-pass sections, held in registers for the block.
    biquad_t q0 = m.lp[0], q1 = m.lp[1], q2 = m.lp[2];
    for (uint32_t f = 0; f < n; f++) {
        float x;
        if (channels == 2) {
            x = ((float)frames[2 * f] + (float)frames[2 * f + 1]) * (0.5f * k);
        } else {
            x = (float)frames[f] * k;
        }
        const float ax = fabsf(x);
        if (ax > pk) {
            pk = ax;
        }
        ss += x * x;
        hr[hp] = x;
        hp = (hp + 1) & (HI_N - 1);
        // Low-pass, then keep one sample in LO_DEC.
        float o = q0.b0 * x + q0.z1;
        q0.z1 = q0.b1 * x - q0.a1 * o + q0.z2;
        q0.z2 = q0.b2 * x - q0.a2 * o;
        float y = o;
        o = q1.b0 * y + q1.z1;
        q1.z1 = q1.b1 * y - q1.a1 * o + q1.z2;
        q1.z2 = q1.b2 * y - q1.a2 * o;
        y = o;
        o = q2.b0 * y + q2.z1;
        q2.z1 = q2.b1 * y - q2.a1 * o + q2.z2;
        q2.z2 = q2.b2 * y - q2.a2 * o;
        if (++ph == LO_DEC) {
            ph = 0;
            lr[lp] = o;
            lp = (lp + 1) & (LO_N - 1);
        }
    }
    m.lp[0] = q0;
    m.lp[1] = q1;
    m.lp[2] = q2;
    m.peak = pk;
    m.sumsq = ss;
    m.nsum += n;
    m.hi_pos = hp;
    m.lo_pos = lp;
    m.dec_phase = ph;
    m.hop_count += n;
    const uint32_t c1 = esp_cpu_get_cycle_count();
    m.feed_cycles += (uint32_t)(c1 - c0);
    if (m.hop_count >= m.hop) {
        m.hop_count -= m.hop;
        meter_analyse();
        const uint32_t d = esp_cpu_get_cycle_count() - c1;
        m.fft_cycles += d;
        m.analyses++;
        if (d > m.max_fft_cycles) {
            m.max_fft_cycles = d;
        }
    }
}

// Copy the latest levels out under the sequence lock. Returns the sequence
// number (it advances about 60 times a second while audio flows) or 0 if the
// meter has never published.
uint32_t usbif_meter_read(uint8_t *levels, uint32_t max, uint8_t *peak, uint8_t *rms, uint32_t *nbands) {
    uint32_t s;
    for (int tries = 0; tries < 4; tries++) {
        s = m.seq;
        if (s & 1) {
            continue;
        }
        __sync_synchronize();
        uint32_t n = m.nbands < max ? m.nbands : max;
        memcpy(levels, m.levels, n);
        *peak = m.peak_b;
        *rms = m.rms_b;
        *nbands = n;
        __sync_synchronize();
        if (m.seq == s) {
            return s / 2;
        }
    }
    *nbands = 0;
    return 0;
}

void usbif_meter_stats(usbif_meter_stats_t *st) {
    const uint32_t mhz = (uint32_t)(esp_clk_cpu_freq() / 1000000);
    st->enabled = m.enabled;
    st->bands = m.nbands;
    st->analyses = m.analyses;
    st->feed_us = m.feed_cycles / mhz;
    st->fft_us = m.fft_cycles / mhz;
    st->max_fft_us = m.max_fft_cycles / mhz;
    st->elapsed_us = m.enabled ? esp_timer_get_time() - m.t0 : 0;
    st->cpu_mhz = mhz;
}

#endif // CFG_TUD_AUDIO
