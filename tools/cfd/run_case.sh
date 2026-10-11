#!/bin/bash
# Runs INSIDE the OpenFOAM container (tools/cfd/cfd.sh starts it), in the case directory.
#   run_case.sh mesh                   blockMesh, features, parallel snappyHexMesh, MRF zone, flow zones, checkMesh
#   run_case.sh solve <Pa> <iters>     one stage of simpleFoam at suction <Pa>, continuing from the latest time
#   run_case.sh yplus                  y+ on the walls at the latest time
set -eo pipefail
source /usr/lib/openfoam/openfoam2512/etc/bashrc 2>/dev/null || true
NP=$(foamDictionary -entry numberOfSubdomains -value system/decomposeParDict)
RUN="mpirun -np $NP --use-hwthread-cpus --bind-to none"
log() { echo "[$(date +%H:%M:%S)] $*"; }

latest() {  # latest time in processor0 (0 if none)
    ls processor0 2>/dev/null | grep -E '^[0-9.e+-]+$' | sort -g | tail -1
}

case "$1" in
mesh)
    log blockMesh;            blockMesh > log.blockMesh 2>&1
    log surfaceFeatureExtract; surfaceFeatureExtract > log.surfaceFeatureExtract 2>&1
    log decomposePar;         decomposePar -force > log.decomposePar.mesh 2>&1
    log snappyHexMesh;        $RUN snappyHexMesh -parallel -overwrite > log.snappyHexMesh 2>&1
    log topoSet;              $RUN topoSet -parallel > log.topoSet 2>&1
    log checkMesh;            $RUN checkMesh -parallel -constant > log.checkMesh 2>&1 || true
    rm -rf processor*/0 && for d in processor*; do cp -r 0.orig $d/0; done
    : > constant/p0Table.steps
    grep -E "cells:|Max non-ortho|Max skewness|Mesh OK|Failed" log.checkMesh | head -12
    ;;
solve)
    PA=$2; N=$3
    T0=$(latest); T0=${T0:-0}; T1=$((T0 + N))
    P0=$(awk -v s="$PA" 'BEGIN{printf "%.6f", -s/1.2}')
    # p0 against iteration: a step at the start of this stage
    if [ "$T0" = "0" ]; then echo "(0 $P0)" >> constant/p0Table.steps; else echo "($((T0 + 1)) $P0)" >> constant/p0Table.steps; fi
    echo "($T1 $P0)" >> constant/p0Table.steps
    { echo "("; cat constant/p0Table.steps; echo ")"; } > constant/p0Table
    foamDictionary -entry endTime -set $T1 system/controlDict > /dev/null
    W=$(python3 -c "import math; print(math.gcd($T0, $N))" 2>/dev/null || awk -v a=$T0 -v b=$N 'function g(x,y){return y?g(y,x%y):x} BEGIN{print g(a,b)}')
    foamDictionary -entry writeInterval -set $W system/controlDict > /dev/null
    echo "$T0 $T1 $PA" >> stages.txt
    log "simpleFoam $T0 -> $T1 at $PA Pa"
    $RUN simpleFoam -parallel > log.simpleFoam.$T1 2>&1
    for d in processor*; do for t in $(ls $d | grep -E '^[0-9]+$'); do
        if [ "$t" -gt "$T0" ] && [ "$t" -lt "$T1" ]; then rm -rf "$d/$t"; fi; done; done
    tail -3 log.simpleFoam.$T1 | head -2
    ;;
yplus)
    $RUN simpleFoam -parallel -postProcess -func yPlus -latestTime > log.yPlus 2>&1
    grep -A12 "yPlus" log.yPlus | grep -E "patch|y\+" | head -20
    ;;
*)
    echo "usage: run_case.sh mesh | solve <Pa> <iters> | yplus"; exit 1;;
esac
