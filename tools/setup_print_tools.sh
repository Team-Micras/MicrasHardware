#!/usr/bin/env bash
# Install the open-source slicing tools tools/slice.py uses, natively on Linux (WSL included):
#   PrusaSlicer (from the distribution: slices both printers' files, headless)
#   UVtools (self-contained release: turns PrusaSlicer's .sl1 into the Photon Mono 4's .pm4n)
set -euo pipefail
UVTOOLS_VERSION=7.0.0

if ! command -v prusa-slicer >/dev/null; then
    sudo apt-get install -y prusa-slicer
fi
if ! command -v UVtoolsCmd >/dev/null; then
    dest="$HOME/.local/opt/UVtools"
    tmp=$(mktemp -d)
    curl -fL -o "$tmp/uvtools.zip" \
        "https://github.com/sn4k3/UVtools/releases/download/v${UVTOOLS_VERSION}/UVtools_linux-x64_v${UVTOOLS_VERSION}.zip"
    rm -rf "$dest" && mkdir -p "$dest" "$HOME/.local/bin"
    unzip -q "$tmp/uvtools.zip" -d "$dest"
    chmod +x "$dest/UVtoolsCmd" "$dest/UVtools" 2>/dev/null || true
    ln -sf "$dest/UVtoolsCmd" "$HOME/.local/bin/UVtoolsCmd"
    rm -rf "$tmp"
fi
prusa-slicer --help | head -1
UVtoolsCmd --version
echo "ok (UVtoolsCmd is in ~/.local/bin: make sure it is on your PATH)"
