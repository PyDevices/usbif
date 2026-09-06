"""Enumerate USB devices and watch attach / detach.

Uses the portable host API (``usbif.auto.host``), so the same script runs on
a board with the native module and on a desktop where the OS owns the bus.
Capabilities are discovered, never assumed -- an empty set is a valid answer.

    mpftp run -d COM49 examples/host_enum.py
    python examples/host_enum.py          # desktop

Plug and unplug devices while it runs; each attach and detach is printed.
On an S3 host that means a powered hub or OTG adapter (no VBUS switching on
the Waveshare touch boards). P4 high-speed host is blocked (usbif#3).
"""

import sys
import time

import events
import usbif
from usbif import auto


def main(seconds=60):
    host = auto.host()
    caps = host.capabilities()
    print("backend capabilities:", sorted(caps) if caps else "(none)")
    host.start()

    print("currently attached:")
    for info in host.devices():
        print(" ", usbif.describe(info))
    if not host.devices():
        print("  (none yet -- plug something in)")

    print("watching attach/detach for %d s ..." % seconds)
    deadline = time.ticks_add(time.ticks_ms(), seconds * 1000) if hasattr(time, "ticks_ms") \
        else None
    end = time.time() + seconds
    try:
        while True:
            if deadline is not None:
                if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                    break
            elif time.time() >= end:
                break
            for event in host.poll():
                kind = "attach" if event.type == events.USBATTACH else "detach"
                if event.type not in (events.USBATTACH, events.USBDETACH):
                    kind = str(event.type)
                print("%s: %s" % (kind, usbif.describe(event.device)))
            if host.overflowed:
                print("warning: event buffer overflowed -- poll more often")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("stopped")
    finally:
        host.stop()


if __name__ == "__main__":
    secs = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    main(secs)
