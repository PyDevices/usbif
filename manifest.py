# MicroPython manifest for usbif.
#
# usbif ships both halves of itself: the native module built from ``src/``
# and the portable API in ``lib/usbif`` that fronts it. They are written
# against each other and version together -- ``usbif.uvc`` exists to
# configure ``usbif_host_uvc.c`` and nothing else -- so they ship together
# too. A board that has the C half in its firmware has the Python half with
# it, with no install step.
#
# The cost, stated plainly because it is real: ``.frozen`` precedes ``lib``
# on ``sys.path``, so a frozen copy wins over anything mip installs, and the
# Python half becomes reflash-only. That is the right trade *here* and the
# wrong one for ``pydevices``, whose libraries are shared and independently
# versioned -- which is why its own manifest refuses to freeze and says so.
# For a native module's own API, a mismatched pair is a worse failure than a
# non-upgradeable one.
#
# Iterating on the Python half without reflashing: ``.`` precedes ``.frozen``,
# so a copy in the board's filesystem *root* shadows the frozen one. Put it
# in ``/``, not in ``/lib`` -- ``/lib`` loses to ``.frozen`` and the edit
# will appear to do nothing.
#
# Note this does not make usbif self-contained: ``usbif/__init__.py`` imports
# ``events``, which lives in pydevices and is installed by mip.
package("usbif", base_path="lib", opt=3)
