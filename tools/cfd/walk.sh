#!/bin/bash
# Walk one case up the flowing branch: tools/cfd/walk.sh <case> <iters> <Pa> <Pa> ...
# Runs `cfd.sh <case> solve <Pa> <iters>` for each suction in turn and stops after the first stage that ends stalled
# (eye flow below 0.3 L/s over its last 100 iterations), so the end of the flowing branch is bracketed without
# wasting stages past it.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
CASE=$1; N=$2; shift 2
for PA in "$@"; do
    "$ROOT/tools/cfd/cfd.sh" "$CASE" solve "$PA" "$N"
    T0=$(tail -1 "$ROOT/build/cfd/runs/$CASE/stages.txt" | awk '{print $1}')
    EYE=$(awk '!/^#/ {print $2}' "$ROOT/build/cfd/runs/$CASE/postProcessing/eyeFlow/$T0/surfaceFieldValue.dat" | tail -20 \
          | awk '{s+=$1; n++} END {printf "%.6f", s/n}')
    echo "walk $CASE: $PA Pa -> eye flow $EYE m3/s"
    if awk -v e="$EYE" 'BEGIN{exit !(e < 0.3e-3)}'; then echo "walk $CASE: stalled at $PA Pa, stop"; break; fi
done
