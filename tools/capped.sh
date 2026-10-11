#!/usr/bin/env bash
# Run a command with a hard memory cap so a runaway CAD/render job gets killed instead of
# exhausting WSL. Usage: tools/capped.sh [cmd...]   (MEM=6G by default)
exec systemd-run --user --scope -q -p MemoryMax="${MEM:-6G}" -p MemorySwapMax=0 "$@"
