#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TOOLS_DIR="${PROJECT_ROOT}/tools"

if ! command -v cargo-appimage >/dev/null 2>&1; then
  echo "cargo-appimage not found. Install with: cargo install cargo-appimage" >&2
  exit 1
fi

if ! command -v appimagetool >/dev/null 2>&1; then
  if [[ -x "${TOOLS_DIR}/appimagetool.AppImage" ]]; then
    ln -sf "${TOOLS_DIR}/appimagetool.AppImage" "${TOOLS_DIR}/appimagetool"
    export PATH="${TOOLS_DIR}:${PATH}"
  else
    cat >&2 <<'EOF'
appimagetool not found.
Install one of these:
  1) system-wide: sudo apt install appimagetool
  2) local file:  ./tools/appimagetool.AppImage  (must be executable)

Example local setup:
  mkdir -p tools
  wget -O tools/appimagetool.AppImage \
    https://github.com/AppImage/AppImageKit/releases/latest/download/appimagetool-x86_64.AppImage
  chmod +x tools/appimagetool.AppImage
EOF
    exit 1
  fi
fi

cargo appimage --release
