#!/bin/bash
# Build Flexi-HAL A7 firmware with the flight recorder: Expatria's builds, each with the
# recorder added, as UF2 files for their bootloader (same as their releases).
#
# Usage: firmware/flexihal-a7/build.sh [variant ...]
#   no arguments  every variant
#   list          show the variants
#   variant       e.g. f446re_flexi_cnc_printnc_eth_sd_fr
#
# Needs git, python3 and PlatformIO (pio) on PATH. The driver is checked out under
# firmware/build/ (or $FR_BUILD_DIR); UF2 files land in firmware/flexihal-a7/out/.
# Extra arguments for pio (e.g. -j 3) can go in $PIO_ARGS.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
. "$here/driver.lock"
build=${FR_BUILD_DIR:-$root/firmware/build}/flexihal-a7
driver=$build/STM32F4xx
out=$here/out

all=$(sed -n 's/^\[env:\(.*\)\]$/\1/p' "$here/platformio.local.ini")
if [ "${1:-}" = list ]; then
    echo "$all"
    exit 0
fi

# 1. Expatria's driver at the pinned commit; every submodule at the commit their tree
#    records, except rgb (see driver.lock). --force resets our patches from last time.
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

# 2. Patches: the core (executing line, plugin start-up, Modbus counters) and the status light plugin.
git -C "$driver/grbl" apply "$root/core/executing-line.patch"
git -C "$driver/grbl" apply "$root/core/plugin-init.patch"
git -C "$driver/grbl" apply "$root/core/modbus-stats.patch"
git -C "$driver/rgb" apply "$here/patches/rgb-bit-count.patch"

# 3. The plugin, as a PlatformIO library next to Expatria's.
rm -rf "$driver/flight_recorder"
cp -r "$root/plugin" "$driver/flight_recorder"

# 4. Our variants, in the platformio.local.ini their platformio.ini already loads.
cp "$here/platformio.local.ini" "$driver/platformio.local.ini"

variants=("$@")
if [ ${#variants[@]} -eq 0 ]; then
    mapfile -t variants <<< "$all"
fi

# 5. Build. Expatria's post-build script writes FLEXI_HAL_<variant>_<driver version>.uf2
#    into the driver folder; collect them.
mkdir -p "$out"
rm -f "$driver"/FLEXI_HAL_*_FR_*.uf2
for v in "${variants[@]}"; do
    (cd "$driver" && pio run -e "$v" ${PIO_ARGS:-})
done
mv -f "$driver"/FLEXI_HAL_*_FR_*.uf2 "$out"/
ls -1 "$out"
