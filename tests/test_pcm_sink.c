// SPDX-License-Identifier: MIT
//
// Host-side tests for the pcm_c_sink_t lifetime gate (usbif#43).
//
// The property that matters is the one a unit test cannot see by calling
// functions in order: a producer on another core writing while the owner
// closes the stream and frees the ring. So the race test runs real threads.
// A writer hammers the sink; the main thread opens a ring on fresh heap
// storage, drains it for a moment, closes, and frees it, twenty thousand
// times. Inside the gate a hook widens the window by sleeping, so a close()
// that did not wait would be caught freeing under a writer on most laps
// rather than once a week.
//
// Two ways a violation is seen. The hook checks, on the way in and on the way
// out, that the storage it is about to touch (or just touched) has not been
// released: the main thread clears `storage_live` only after close() returns,
// so the writer finding it clear while inside is close() having returned
// early. And the build CI runs under AddressSanitizer turns an actual write
// into freed memory into a crash.
//
// Build with -DUSBIF_PCM_SINK_PLANT_NO_WAIT and close() skips its wait. That
// build MUST fail this test; CI runs it and requires the failure, so the race
// test is known to be able to fail.
#define _POSIX_C_SOURCE 200809L

#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static volatile int storage_live;
static volatile unsigned inside_violations;
static volatile unsigned hook_calls;

static void inside_hook(void) {
    if (!__atomic_load_n(&storage_live, __ATOMIC_SEQ_CST)) {
        __atomic_fetch_add(&inside_violations, 1, __ATOMIC_RELAXED);
    }
    // Every few calls, sit inside the gate long enough for the main thread
    // to reach close() and, if it does not wait, free.
    if ((__atomic_fetch_add(&hook_calls, 1, __ATOMIC_RELAXED) & 7) == 0) {
        struct timespec ts = {0, 20000};
        nanosleep(&ts, NULL);
    }
    if (!__atomic_load_n(&storage_live, __ATOMIC_SEQ_CST)) {
        __atomic_fetch_add(&inside_violations, 1, __ATOMIC_RELAXED);
    }
}

#define USBIF_PCM_SINK_INSIDE_HOOK() inside_hook()
#include "shared/usbif_pcm_sink.c"
#include "pcm_c_sink.h"

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
} while (0)

static void yield_wait(void) {
    sched_yield();
}

// --- the published struct, wired as usbif_host_uac.c wires it -------------

static usbif_byte_ring_t ring;
static usbif_pcm_sink_t gate;

static int sink_write(void *ctx, const uint8_t *data, size_t len) {
    return usbif_pcm_sink_write(&gate, (uint32_t)(uintptr_t)ctx, data, len);
}

static int sink_space(void *ctx) {
    return usbif_pcm_sink_space(&gate, (uint32_t)(uintptr_t)ctx);
}

static pcm_c_sink_t make_sink(uint32_t gen) {
    pcm_c_sink_t s = {
        .magic = PCM_C_SINK_MAGIC, .version = PCM_C_SINK_VERSION,
        .ctx = (void *)(uintptr_t)gen, .write = sink_write, .space = sink_space,
        .rate = 48000, .channels = 2, .bits = 16, .frame_bytes = 4,
    };
    return s;
}

// --- 1. sequential semantics ------------------------------------------------

static void test_sequential(void) {
    static uint8_t store[64];
    usbif_byte_ring_init(&ring, store, sizeof(store));
    usbif_pcm_sink_init(&gate, &ring, yield_wait);
    storage_live = 1;

    CHECK(usbif_pcm_sink_current(&gate) == 0);
    CHECK(usbif_pcm_sink_write(&gate, 1, store, 4) == -1);   // never opened

    uint32_t g1 = usbif_pcm_sink_open(&gate, 4);
    CHECK(g1 != 0);
    CHECK(usbif_pcm_sink_current(&gate) == g1);
    pcm_c_sink_t s1 = make_sink(g1);

    const uint8_t a[70] = {0};
    CHECK(s1.space(s1.ctx) == 64);
    CHECK(s1.write(s1.ctx, a, 6) == 4);                       // whole frames only
    CHECK(s1.write(s1.ctx, a, 3) == 0);                       // less than a frame
    CHECK(s1.space(s1.ctx) == 60);
    CHECK(s1.write(s1.ctx, a, 70) == 60);                     // cut to what fits
    CHECK(s1.write(s1.ctx, a, 4) == 0);                       // full: 0, not -1
    CHECK(s1.space(s1.ctx) == 0);
    CHECK(usbif_pcm_sink_write(&gate, 0, a, 4) == -1);        // 0 is never a stream

    usbif_pcm_sink_close(&gate);
    CHECK(usbif_pcm_sink_current(&gate) == 0);
    CHECK(s1.write(s1.ctx, a, 4) == -1);
    CHECK(s1.space(s1.ctx) == -1);
    usbif_pcm_sink_close(&gate);                              // idempotent

    // A new stream does not revive the old sink.
    usbif_byte_ring_init(&ring, store, sizeof(store));
    uint32_t g2 = usbif_pcm_sink_open(&gate, 4);
    CHECK(g2 != g1 && g2 != 0);
    pcm_c_sink_t s2 = make_sink(g2);
    CHECK(s1.write(s1.ctx, a, 4) == -1);
    CHECK(s1.space(s1.ctx) == -1);
    CHECK(s2.write(s2.ctx, a, 4) == 4);
    usbif_pcm_sink_close(&gate);

    // Generations skip 0 when they wrap.
    gate.last_gen = UINT32_MAX;
    CHECK(usbif_pcm_sink_open(&gate, 4) == 1);
    usbif_pcm_sink_close(&gate);
}

// --- 2. the published struct round-trips through a byte buffer --------------

static void test_copy(void) {
    pcm_c_sink_t s = make_sink(7), got;
    uint8_t blob[sizeof(pcm_c_sink_t) + 1];
    memcpy(blob + 1, &s, sizeof(s));                          // deliberately unaligned
    CHECK(pcm_c_sink_copy(&got, blob + 1, sizeof(s)));
    CHECK(got.write == sink_write && got.ctx == s.ctx && got.frame_bytes == 4);
    CHECK(!pcm_c_sink_copy(&got, blob + 1, sizeof(s) - 1));   // short
    CHECK(!pcm_c_sink_copy(&got, NULL, sizeof(s)));
    s.magic ^= 1;
    memcpy(blob + 1, &s, sizeof(s));
    CHECK(!pcm_c_sink_copy(&got, blob + 1, sizeof(s)));       // wrong magic
    s = make_sink(7);
    s.version = PCM_C_SINK_VERSION + 1;
    memcpy(blob + 1, &s, sizeof(s));
    CHECK(!pcm_c_sink_copy(&got, blob + 1, sizeof(s)));       // newer version
}

// --- 3. write racing close ----------------------------------------------------

static volatile uint32_t published_gen;   // the sink the writer should use
static volatile uint32_t closed_upto;     // highest generation close() finished
static volatile int stop;
static volatile unsigned late_success;    // a write that began after close and took bytes
static volatile unsigned bytes_taken;

static void *writer(void *arg) {
    (void)arg;
    uint8_t frames[256];
    memset(frames, 0x5A, sizeof(frames));
    while (!__atomic_load_n(&stop, __ATOMIC_ACQUIRE)) {
        uint32_t gen = __atomic_load_n(&published_gen, __ATOMIC_ACQUIRE);
        if (gen == 0) {
            continue;
        }
        // Read before the call: if close() had already finished for this
        // generation when the write began, the write must return -1.
        uint32_t closed = __atomic_load_n(&closed_upto, __ATOMIC_ACQUIRE);
        pcm_c_sink_t s = make_sink(gen);
        int n = s.write(s.ctx, frames, sizeof(frames));
        if (n > 0) {
            __atomic_fetch_add(&bytes_taken, (unsigned)n, __ATOMIC_RELAXED);
            if (closed >= gen) {
                __atomic_fetch_add(&late_success, 1, __ATOMIC_RELAXED);
            }
        }
        (void)s.space(s.ctx);
    }
    return NULL;
}

static void test_race(void) {
    usbif_pcm_sink_init(&gate, &ring, yield_wait);
    usbif_byte_ring_init(&ring, NULL, 0);
    storage_live = 0;
    inside_violations = 0;

    pthread_t t;
    pthread_create(&t, NULL, writer, NULL);

    uint8_t drain[512];
    const int laps = 20000;
    for (int lap = 0; lap < laps; lap++) {
        const uint32_t size = 1024 + 4 * (uint32_t)(lap % 97);
        uint8_t *mem = malloc(size);
        usbif_byte_ring_init(&ring, mem, size);
        __atomic_store_n(&storage_live, 1, __ATOMIC_SEQ_CST);
        uint32_t gen = usbif_pcm_sink_open(&gate, 4);
        __atomic_store_n(&published_gen, gen, __ATOMIC_RELEASE);

        // The consumer's part, briefly: the transfer callbacks, here.
        for (int i = 0; i < (lap % 5); i++) {
            usbif_byte_ring_pop(&ring, drain, sizeof(drain));
            sched_yield();
        }

        usbif_pcm_sink_close(&gate);
        __atomic_store_n(&closed_upto, gen, __ATOMIC_RELEASE);
        // close() has returned: the guarantee says the storage is ours.
        __atomic_store_n(&storage_live, 0, __ATOMIC_SEQ_CST);
        usbif_byte_ring_init(&ring, NULL, 0);
        memset(mem, 0xDD, size);
        free(mem);
    }

    __atomic_store_n(&stop, 1, __ATOMIC_RELEASE);
    pthread_join(t, NULL);

    printf("race: %d laps, %u bytes written, %u hook calls, "
        "%u writes inside after free, %u late writes accepted\n",
        laps, bytes_taken, hook_calls, inside_violations, late_success);
    CHECK(bytes_taken > 0);             // the writer really did get in
    CHECK(inside_violations == 0);
    CHECK(late_success == 0);
}

// --- 4. two producers never corrupt the ring --------------------------------
//
// The interpreter and a usermod's task can both hold a way to write. With the
// try-lock, frames arrive whole and each producer's own sequence in order;
// without it, two pushes race on the ring's head and both are wrong.

static volatile int producers_stop;

static void *producer(void *arg) {
    const uint8_t id = (uint8_t)(uintptr_t)arg;
    uint8_t seq = 0;
    uint8_t frame[4];
    while (!__atomic_load_n(&producers_stop, __ATOMIC_ACQUIRE)) {
        frame[0] = id;
        frame[1] = seq;
        frame[2] = (uint8_t)~id;
        frame[3] = (uint8_t)~seq;
        int n = usbif_pcm_sink_write(&gate, usbif_pcm_sink_current(&gate), frame, 4);
        if (n == 4) {
            seq++;
        }
    }
    return NULL;
}

static void test_two_producers(void) {
    static uint8_t store[4096];
    usbif_byte_ring_init(&ring, store, sizeof(store));
    usbif_pcm_sink_init(&gate, &ring, yield_wait);
    __atomic_store_n(&storage_live, 1, __ATOMIC_SEQ_CST);
    usbif_pcm_sink_open(&gate, 4);

    pthread_t a, b;
    pthread_create(&a, NULL, producer, (void *)(uintptr_t)1);
    pthread_create(&b, NULL, producer, (void *)(uintptr_t)2);

    uint8_t next[3] = {0, 0, 0};
    unsigned frames = 0, bad = 0;
    uint8_t buf[256];
    while (frames < 200000) {
        uint32_t got = usbif_byte_ring_pop(&ring, buf, sizeof(buf));
        if (got % 4) {
            bad++;
        }
        for (uint32_t i = 0; i + 4 <= got; i += 4) {
            const uint8_t *f = buf + i;
            if ((f[0] != 1 && f[0] != 2) || f[2] != (uint8_t)~f[0]
                || f[3] != (uint8_t)~f[1] || f[1] != next[f[0]]) {
                bad++;
            } else {
                next[f[0]]++;
            }
            frames++;
        }
    }
    __atomic_store_n(&producers_stop, 1, __ATOMIC_RELEASE);
    pthread_join(a, NULL);
    pthread_join(b, NULL);
    usbif_pcm_sink_close(&gate);
    printf("two producers: %u frames, %u bad\n", frames, bad);
    CHECK(bad == 0);
}

int main(void) {
    test_sequential();
    test_copy();
    test_two_producers();
    test_race();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("pcm sink: all passed\n");
    return 0;
}
