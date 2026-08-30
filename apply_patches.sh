#!/usr/bin/env bash
# Apply usbif's MicroPython patches to a MicroPython tree.
#
#   ./apply_patches.sh --status [MP_DIR]
#   ./apply_patches.sh --apply  [MP_DIR]
#   ./apply_patches.sh --revert [MP_DIR]
#
# MP_DIR defaults to the sibling cmods/micropython checkout.
#
# The patches are small and additive: two hooks that let a user C module extend
# MicroPython's built-in USB configuration, and one that lets it vary that
# configuration at runtime. Each carries its purpose, provenance and version
# range in its own header. They live here rather than in the MicroPython
# checkout because a pinned upstream tree carrying undocumented local edits is
# precisely the failure this organization's patch-queue discipline exists to
# prevent -- the tree should be pristine, and every downstream change should be
# a file with a reason attached.
set -euo pipefail

MODE="${1:---status}"
MP_DIR="${2:-$(cd "$(dirname "$0")/.." && pwd)/cmods/micropython}"
PATCH_DIR="$(cd "$(dirname "$0")" && pwd)/patches"

if [ ! -d "$MP_DIR" ]; then
    echo "MicroPython tree not found: $MP_DIR" >&2
    exit 1
fi

patches=$(ls "$PATCH_DIR"/*.patch | sort)

case "$MODE" in
    --status)
        # The series is ordered and interdependent -- 0002 and 0003 both touch
        # mp_usbd.h -- so checking a middle patch in isolation reports a
        # conflict on a perfectly healthy tree. The last patch reversing
        # cleanly means the whole series is applied; the first applying cleanly
        # means none of it is. Anything else is genuinely worth looking at.
        last=$(echo "$patches" | tail -1)
        first=$(echo "$patches" | head -1)
        if git -C "$MP_DIR" apply --reverse --check "$last" 2>/dev/null; then
            echo "all applied ($(echo "$patches" | wc -l) patches)"
        elif git -C "$MP_DIR" apply --check "$first" 2>/dev/null; then
            echo "none applied"
        else
            echo "PARTIAL or conflicting -- inspect before building:"
            for p in $patches; do
                if git -C "$MP_DIR" apply --reverse --check "$p" 2>/dev/null; then
                    printf '  applied     %s\n' "$(basename "$p")"
                elif git -C "$MP_DIR" apply --check "$p" 2>/dev/null; then
                    printf '  not applied %s\n' "$(basename "$p")"
                else
                    printf '  unclear     %s  (may just be a later patch sharing a file)\n' "$(basename "$p")"
                fi
            done
        fi
        ;;

    --apply)
        for p in $patches; do
            if git -C "$MP_DIR" apply --reverse --check "$p" 2>/dev/null; then
                printf 'already applied %s\n' "$(basename "$p")"
            else
                git -C "$MP_DIR" apply "$p"
                printf 'applied         %s\n' "$(basename "$p")"
            fi
        done
        echo
        echo "Also required, per board: point the board header at usbif's config, e.g."
        echo '  #define MICROPY_HW_USB_EXT_TUSB_CONFIG "usbif_tusb_ext.h"'
        ;;
    --revert)
        for p in $(echo "$patches" | tac); do
            if git -C "$MP_DIR" apply --reverse --check "$p" 2>/dev/null; then
                git -C "$MP_DIR" apply --reverse "$p"
                printf 'reverted        %s\n' "$(basename "$p")"
            else
                printf 'not applied     %s\n' "$(basename "$p")"
            fi
        done
        ;;
    *)
        sed -n '2,8p' "$0" >&2
        exit 2
        ;;
esac
