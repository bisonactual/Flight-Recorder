#!/bin/bash
# Builds the simulator variants the SD card evidence uses (evidence/SD.md).
#
# Usage: tools/build_sim.sh <GRBLHAL-Sim checkout, branch sd-card> <output dir> [variant ...]
#   sd          the normal build: FRAM stand-in (FLIGHT.DAT) unless -F, the SD card with -d
#   sd-fwrite   SD writes through f_write + f_sync instead, for comparison
#   sd-nobatch  every event written at once, for comparison
#   sd-compact  FLIGHT.DAT's three parts packed together instead of 4 MiB apart, for comparison
#   sd-1mhz     1 MHz processor clock, for whole-job runs (evidence/README.md)
#   sd-short    short retract limit, for the focused hold-retract scenario
# Each lands in <output dir>/<variant>/grblHAL_flexihal_sim. Linux or WSL, with cmake and gcc.
set -euo pipefail

src=$(cd "$1" && pwd)
out=$2
shift 2
fr=$(cd "$(dirname "$0")/.." && pwd)
variants=("$@")
[ ${#variants[@]} -eq 0 ] && variants=(sd sd-fwrite sd-nobatch sd-compact sd-1mhz sd-short)

for v in "${variants[@]}"; do
    case $v in
        sd)         flags="" ;;
        sd-fwrite)  flags="-DFR_SD_FATFS_WRITES=1" ;;
        sd-nobatch) flags="-DFR_SD_BATCH_EVENTS=0" ;;
        sd-compact) flags="-DFR_SD_SPREAD=0" ;;
        sd-1mhz)    flags="-UF_CPU -DF_CPU=1000000" ;;
        sd-short)   flags="-DFR_RETRACT_LIMIT=15000u" ;;
        *) echo "unknown variant $v"; exit 1 ;;
    esac
    mkdir -p "$out/$v"
    cmake -S "$src" -B "$out/$v" -DCMAKE_BUILD_TYPE=Release -DFLIGHT_RECORDER_DIR="$fr" -DCMAKE_C_FLAGS="$flags" > "$out/$v/cmake.log"
    make -C "$out/$v" -j"$(nproc)" > "$out/$v/make.log"
    echo "$v: $out/$v/grblHAL_flexihal_sim"
done
