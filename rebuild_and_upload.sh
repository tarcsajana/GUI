#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
DEFAULT_IDF_EXPORT_SH="$HOME/.espressif/v6.1/esp-idf/export.sh"

IDF_EXPORT_SH="${IDF_EXPORT_SH:-$DEFAULT_IDF_EXPORT_SH}"
GITHUB_REPO="${GITHUB_REPO:-tarcsajana/GUI}"
RELEASE_TAG="${RELEASE_TAG:-OTA}"
ASSET_PATH="${ASSET_PATH:-build/lvgl_porting.bin}"

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "Error: required command '$1' was not found." >&2
        exit 1
    fi
}

if [[ -f "$IDF_EXPORT_SH" ]]; then
    # shellcheck disable=SC1090
    source "$IDF_EXPORT_SH"
elif ! command -v idf.py >/dev/null 2>&1; then
    echo "Error: ESP-IDF environment is not active and export script was not found at '$IDF_EXPORT_SH'." >&2
    echo "Set IDF_EXPORT_SH to your export.sh path or start this from an ESP-IDF shell." >&2
    exit 1
fi

require_command idf.py
require_command gh

cd "$PROJECT_DIR"

echo "Reconfiguring project..."
idf.py reconfigure

echo "Building firmware..."
idf.py build

if [[ ! -f "$ASSET_PATH" ]]; then
    echo "Error: build artifact '$ASSET_PATH' was not produced." >&2
    exit 1
fi

echo "Uploading '$ASSET_PATH' to GitHub release '$RELEASE_TAG' in '$GITHUB_REPO'..."
gh release upload "$RELEASE_TAG" "$ASSET_PATH" --repo "$GITHUB_REPO" --clobber

echo "Verifying uploaded asset..."
gh release view "$RELEASE_TAG" --repo "$GITHUB_REPO"

echo "Done. OTA asset updated from '$ASSET_PATH'."