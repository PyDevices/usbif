"""Count MIDI messages arriving on a Windows MIDI input, with nothing but winmm.

    python midi_in_count.py [substring-of-port-name] [seconds]

Lists the inputs, opens the first whose name contains the substring (or the
first input at all), and prints how many short messages arrived and the last
few, so a device's MIDI IN endpoint can be proven from the PC side without
installing anything.
"""
import ctypes, ctypes.wintypes as wt, sys, time

winmm = ctypes.WinDLL("winmm")
MIM_DATA = 0x3C3
CALLBACK_FUNCTION = 0x30000

class MIDIINCAPSW(ctypes.Structure):
    _fields_ = [("wMid", wt.WORD), ("wPid", wt.WORD), ("vDriverVersion", wt.UINT),
                ("szPname", wt.WCHAR * 32), ("dwSupport", wt.DWORD)]

def inputs():
    n = winmm.midiInGetNumDevs()
    out = []
    for i in range(n):
        caps = MIDIINCAPSW()
        winmm.midiInGetDevCapsW(i, ctypes.byref(caps), ctypes.sizeof(caps))
        out.append(caps.szPname)
    return out

PROC = ctypes.WINFUNCTYPE(None, wt.HANDLE, wt.UINT, ctypes.POINTER(wt.DWORD), ctypes.POINTER(wt.DWORD), ctypes.POINTER(wt.DWORD))
count = 0
last = []
@PROC
def on_msg(h, msg, inst, p1, p2):
    global count
    if msg == MIM_DATA:
        count += 1
        v = ctypes.cast(p1, ctypes.c_void_p).value or 0
        last.append("%02x %02x %02x" % (v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF))
        del last[:-4]

want = sys.argv[1] if len(sys.argv) > 1 else ""
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 5
names = inputs()
print("MIDI inputs:", names)
idx = next((i for i, n in enumerate(names) if want.lower() in n.lower()), None)
if idx is None:
    print("no input matching %r" % want); sys.exit(2)
h = wt.HANDLE()
rc = winmm.midiInOpen(ctypes.byref(h), idx, on_msg, 0, CALLBACK_FUNCTION)
if rc:
    print("midiInOpen failed", rc); sys.exit(1)
winmm.midiInStart(h)
print("listening on %r for %.0f s" % (names[idx], secs)); sys.stdout.flush()
time.sleep(secs)
winmm.midiInStop(h); winmm.midiInClose(h)
print("messages: %d  last: %s" % (count, last))
