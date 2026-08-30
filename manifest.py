# MicroPython manifest for usbif.
#
# The native module needs no frozen Python: the portable API it backs ships
# with the `pydevices` distribution as `lib/usbif`, so a board gets the C half
# from its firmware build and the Python half from mip like any other package.
# This file exists so the repository can be pointed at by a manifest include
# without a special case.
