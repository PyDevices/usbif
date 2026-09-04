# MicroPython manifest for usbif.
#
# Deliberately empty, and not for want of Python to freeze.
#
# usbif ships both halves of itself: the native module built from ``src/``
# and the portable API in ``lib/usbif`` that fronts it. Freezing the Python
# half here would put it in every interpreter the parent workspace builds,
# and a frozen copy shadows a mip-installed one -- so what is running would
# stop being what was published. That is the same trap ``pydevices``' own
# manifest refuses to walk into, and audioif's with it.
#
# So a board gets the C half from its firmware build and the Python half
# from mip, like any other package. This file exists so the repository can
# be pointed at by a manifest include without a special case.
