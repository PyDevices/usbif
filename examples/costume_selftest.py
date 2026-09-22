# Structural self-test of every USB costume this firmware can wear.
#
# For each combination of functions, assemble the descriptor and validate
# it: descriptors tile wTotalLength exactly, interface numbers run dense
# from zero, bNumInterfaces agrees with the content, every sibling
# reference the assembler rewrote lands on an interface that exists, and
# the device class agrees with whether any interface association survived.
#
# A host finds these faults by refusing the device -- days later, with no
# explanation. This finds them in microseconds and names which one.
#
# Note it does NOT re-enumerate: validation runs on the assembled bytes,
# so the whole matrix is checked in a fraction of a second without a host
# in the loop. Run it after any change to the descriptor assembler.

import usbif.auto

FAULTS = {
    -1: "wTotalLength outside the buffer",
    -2: "zero-length descriptor",
    -3: "descriptor runs past wTotalLength",
    -4: "short interface descriptor",
    -5: "interface number out of range",
    -6: "short interface association",
    -7: "association spans past the range",
    -8: "CDC call-management points at no interface",
    -9: "CDC union points at no interface",
    -10: "audio control header points at no interface",
    -11: "descriptors do not tile wTotalLength",
    -12: "configuration has no interfaces",
    -13: "gap in interface numbering",
    -14: "bNumInterfaces disagrees with content",
    -15: "device class disagrees with the associations",
    -16: "a class that requires an interface association lost it",
}


def main():
    dev = usbif.auto.device()
    # The names this firmware was built with, in the portable spelling. Asking
    # the board rather than listing them here is what keeps the matrix honest
    # when a firmware gains or loses a function -- a hand-written list would
    # quietly stop covering the new one.
    built = sorted(dev.functions_available())
    restore = sorted(dev.functions())
    checked = 0
    failed = 0
    try:
        for combo in range(1, 1 << len(built)):
            names = [n for i, n in enumerate(built) if combo & (1 << i)]
            dev.functions(*names)
            code = dev.desc_check()
            checked += 1
            if code != 0:
                failed += 1
                print("FAIL {:<24} {}".format(
                    "+".join(names),
                    FAULTS.get(code, "unknown fault {}".format(code))))
    finally:
        dev.functions(*restore)
    print("costume self-test: {} checked, {} failed".format(checked, failed))
    return failed == 0


main()
