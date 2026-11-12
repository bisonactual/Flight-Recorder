#!/bin/bash
# Builds Flexi-HAL A7 firmware with the flight recorder. These are Expatria's own builds
# with the recorder added, as UF2 files for their bootloader, the same as their releases.
#
# Usage: firmware/flexihal-a7/build.sh [variant ...]
#   no arguments  every variant we release (not the _nokeypad diagnostic builds)
#   list          show the variants, the diagnostic builds too
#   variant       e.g. f446re_flexi_cnc_printnc_eth_sd_fr
#
# Needs git, python3 and PlatformIO (pio) on PATH. The driver is checked out under
# firmware/build/ (or $FR_BUILD_DIR) and the UF2 files end up in firmware/flexihal-a7/out/.
# Extra arguments for pio (e.g. -j 3) can go in $PIO_ARGS.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
. "$here/driver.lock"
build=${FR_BUILD_DIR:-$root/firmware/build}/flexihal-a7
driver=$build/STM32F4xx
out=$here/out

all=$(sed -n 's/^\[env:\(.*\)\]$/\1/p' "$here/platformio.local.ini")
releases=$(grep -v '_nokeypad$' <<< "$all")
if [ "${1:-}" = list ]; then
    echo "$all"
    exit 0
fi

# 1. Expatria's driver at the pinned commit, with every submodule at the commit their tree
#    records except rgb (see driver.lock). --force undoes our patches from last time.
if [ ! -d "$driver/.git" ]; then
    mkdir -p "$build"
    git clone -q "$DRIVER_URL" "$driver"
fi
if ! git -C "$driver" cat-file -e "$DRIVER_COMMIT^{commit}" 2>/dev/null; then
    git -C "$driver" fetch -q origin
fi
git -C "$driver" checkout -q -f "$DRIVER_COMMIT"
for m in $(git -C "$driver" config -f .gitmodules --get-regexp '\.path$' | awk '{print $2}'); do
    [ "$m" = rgb ] && continue
    git -C "$driver" submodule update -q --init --force --recursive "$m"
done
git -C "$driver" submodule update -q --init rgb 2>/dev/null || true   # its recorded commit isn't published
if ! git -C "$driver/rgb" cat-file -e "$RGB_COMMIT^{commit}" 2>/dev/null; then
    git -C "$driver/rgb" fetch -q origin
fi
git -C "$driver/rgb" checkout -q -f "$RGB_COMMIT"

# 2. Our patches to the core (executing line, plugin start-up, Modbus counters, rapid levels),
#    the driver's I2C wait, the keypad's key read and the status light plugin.
git -C "$driver/grbl" apply "$root/core/executing-line.patch"
git -C "$driver/grbl" apply "$root/core/plugin-init.patch"
git -C "$driver/grbl" apply "$root/core/modbus-stats.patch"
git -C "$driver/grbl" apply "$root/core/rapid-levels.patch"
git -C "$driver" apply "$here/patches/i2c-wait-ready.patch"
git -C "$driver/keypad" apply "$here/patches/keypad-strobe-task.patch"
git -C "$driver/rgb" apply "$here/patches/rgb-bit-count.patch"

# 3. The plugin, as a PlatformIO library next to Expatria's.
rm -rf "$driver/flight_recorder"
cp -r "$root/plugin" "$driver/flight_recorder"

# 4. Our variants go in platformio.local.ini, which their platformio.ini already loads.
cp "$here/platformio.local.ini" "$driver/platformio.local.ini"

variants=("$@")
if [ ${#variants[@]} -eq 0 ]; then
    mapfile -t variants <<< "$releases"
fi

# 5. Build. Expatria's post-build script writes FLEXI_HAL_<variant>_<driver version>.uf2
#    into the driver folder, so collect them from there.
mkdir -p "$out"
rm -f "$driver"/FLEXI_HAL_*_FR_*.uf2
for v in "${variants[@]}"; do
    (cd "$driver" && pio run -e "$v" ${PIO_ARGS:-})
done
mv -f "$driver"/FLEXI_HAL_*_FR_*.uf2 "$out"/
ls -1 "$out"
