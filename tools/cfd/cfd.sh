#!/bin/bash
# Host-side wrapper: run tools/cfd/run_case.sh for a case under build/cfd/runs/ in the OpenFOAM container,
# with hard limits (WSL dies if memory runs out): 14 GB RAM, no swap, 16 CPUs.
#   tools/cfd/cfd.sh <case> mesh | solve <Pa> <iters> | yplus | shell <cmd...>
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
IMAGE=${CFD_IMAGE:-opencfd/openfoam-default:latest}   # OpenFOAM v2512 (OpenCFD), pulled 2026-10-03
CASE=$1; shift
[ -d "$ROOT/build/cfd/runs/$CASE" ] || { echo "no case build/cfd/runs/$CASE"; exit 1; }
if [ "$1" = shell ]; then shift; CMD="source /usr/lib/openfoam/openfoam2512/etc/bashrc; $*"; else CMD="/tools/run_case.sh $*"; fi
exec docker run --rm --name "cfd_${CASE}_$$" --memory=${CFD_MEM:-14g} --memory-swap=${CFD_MEM:-14g} --cpus=${CFD_CPUS:-16} \
    --shm-size=2g --user "$(id -u):$(id -g)" \
    -v "$ROOT/build/cfd:/work" -v "$ROOT/tools/cfd:/tools:ro" -w "/work/runs/$CASE" \
    --entrypoint bash "$IMAGE" -c "cd /work/runs/$CASE && $CMD"
