#!/usr/bin/env bash
# Build ESP-IDF's own usb_host_lib example for the ESP32-P4 panel, as one
# image that mpftp flashes at 0x2000 in place of MicroPython. It is the
# discriminator usbif#3 names: when usbif's host on the P4 sees no device,
# this says whether IDF's host stack does either, with usbif out of the picture.
#
#   spikes/p4_idf_host_example.sh [--fs] [OUT.bin]
#
# --fs puts the host on the P4's full-speed controller (peripheral_map BIT1)
#      instead of the high-speed one (BIT0: what usbif installs on, and the
#      one wired to the panel's Type-C connector). The full-speed PHY shares
#      pins with USB-Serial-JTAG and reaches header P2 only through
#      unpopulated resistors, so --fs is for the header experiment, not the
#      connector.
# OUT  where the merged image goes (default: ./usb_host_lib_p4_at_0x2000.bin).
#
# Flash it, then read the console at 115200 on the board's UART port:
#   mpftp firmware flash --artifact OUT.bin --family esp32p4 -d COM4
# An attach prints "CLASS: Opening device at address 1" and the descriptors;
# silence after "CLASS: Registering Client" means the root port saw no
# pull-up. Flash MicroPython back with --erase afterwards: the example's
# partition table is not MicroPython's, and mpftp refuses to flash over it
# without one.
#
# What it needs: an ESP-IDF checkout (IDF_PATH, else ../esp-idf beside this
# repository) with its tools installed. The 2026-09-23 run (IDF 5.5.4, Touch-LCD-4B,
# powered hub, no VBUS injected) saw nothing on either controller.
set -euo pipefail

controller=hs
out=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --fs) controller=fs; shift ;;
        -h|--help) sed -n '2,27p' "$0"; exit 0 ;;
        *) out="$1"; shift ;;
    esac
done
out=${out:-$PWD/usb_host_lib_p4_at_0x2000.bin}
case "$out" in /*) ;; *) out="$PWD/$out" ;; esac

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
idf_dir=${IDF_PATH:-$repo_dir/../esp-idf}
example="$idf_dir/examples/peripherals/usb/host/usb_host_lib"
if [[ ! -f "$example/main/usb_host_lib_main.c" ]]; then
    echo "error: no usb_host_lib example at $example; set IDF_PATH" >&2
    exit 1
fi

work=$(mktemp -d "${TMPDIR:-/tmp}/usb_host_lib.XXXXXX")
trap 'rm -rf "$work"' EXIT
cp -r "$example/." "$work/"

# The example's own defaults, plus: hub support (a hub is how a board that
# sources no VBUS reaches anything, and it is off upstream), and the chip
# revision floor MicroPython's PRE_REV3 variant sets, so the bootloader runs
# on the rev 1.x silicon the bench board carries (a rev 3 chip runs it too).
cat >> "$work/sdkconfig.defaults" <<'CFG'
CONFIG_USB_HOST_HUBS_SUPPORTED=y
CONFIG_ESP32P4_REV_MIN_300=n
CONFIG_ESP32P4_REV_MIN_0=y
CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y
CFG

# The example installs on BIT1. IDF's usb_host maps port index 1 to the
# full-speed controller and index 0 to the high-speed one (hal/esp32p4
# usb_dwc_ll.h: USB_DWC_LL_GET_HW), the reverse of TinyUSB's numbering.
if [[ $controller == hs ]]; then
    sed -i 's/\.peripheral_map = BIT1,/.peripheral_map = BIT0,/' "$work/main/usb_host_lib_main.c"
fi
grep -q "peripheral_map = BIT" "$work/main/usb_host_lib_main.c" || {
    echo "error: the example no longer sets peripheral_map the way this script expects" >&2
    exit 1
}

# shellcheck disable=SC1091
source "$idf_dir/export.sh" >/dev/null 2>&1
(cd "$work" && idf.py set-target esp32p4 >/dev/null && idf.py build >/dev/null)

# One image at 0x2000 (the P4's bootloader offset), which is what mpftp's
# --artifact/--family esp32p4 path writes.
(cd "$work/build" && esptool.py --chip esp32p4 merge_bin -o "$out" --target-offset 0x2000 \
    $(grep -oE "0x[0-9a-f]+ [^ ]+" flash_args | tr '\n' ' ') >/dev/null)
echo "Built $out (host on the ${controller^^} controller)"
