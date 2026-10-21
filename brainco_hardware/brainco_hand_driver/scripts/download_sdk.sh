#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENDOR_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)/vendor"
SDK_VERSION="v2.0.8"
BASE_URL="https://app.brainco.cn/universal/bc-stark-sdk/libs/${SDK_VERSION}"
VERSION_FILE="${VENDOR_DIR}/VERSION"
PLATFORM_FILE="${VENDOR_DIR}/.sdk-platform"
OS_TYPE=$(uname -s)
ARCH=$(uname -m)

case "${OS_TYPE}:${ARCH}" in
  Linux:x86_64) PLATFORM="linux"; LIB_PLATFORM="linux"; LIB_NAME="libbc_stark_sdk.so"; ELF_ARCH="x86-64" ;;
  Linux:aarch64|Linux:arm64) PLATFORM="linux-arm64"; LIB_PLATFORM="linux"; LIB_NAME="libbc_stark_sdk.so"; ELF_ARCH="ARM aarch64" ;;
  Darwin:arm64|Darwin:x86_64) PLATFORM="mac"; LIB_PLATFORM="mac"; LIB_NAME="libbc_stark_sdk.dylib" ;;
  MINGW*:x86_64|MSYS_NT*:x86_64) PLATFORM="win-cpp"; LIB_PLATFORM="win"; LIB_NAME="bc_stark_sdk.dll" ;;
  *) echo "Unsupported platform: ${OS_TYPE}/${ARCH}" >&2; exit 1 ;;
esac

LIB_PATH="${VENDOR_DIR}/dist/shared/${LIB_PLATFORM}/${LIB_NAME}"
HEADER_PATH="${VENDOR_DIR}/dist/include/stark-sdk.h"
PLATFORM_KEY="${PLATFORM}:${ARCH}"

valid_library() {
  [ -s "$1" ] || return 1
  if [ "$OS_TYPE" = Linux ]; then
    file -b "$1" | grep -Fq "${ELF_ARCH}"
  fi
}

if [ -f "$VERSION_FILE" ] && [ "$(cat "$VERSION_FILE")" = "$SDK_VERSION" ] &&
   [ -f "$PLATFORM_FILE" ] && [ "$(cat "$PLATFORM_FILE")" = "$PLATFORM_KEY" ] &&
   [ -s "$HEADER_PATH" ] && [ -s "${VENDOR_DIR}/LICENSE" ] && valid_library "$LIB_PATH"; then
  echo "[BrainCo Stark SDK] ${SDK_VERSION} for ${PLATFORM_KEY} is already installed"
  exit 0
fi

# Validate the archive before replacing the installed SDK or cache metadata.
mkdir -p "$VENDOR_DIR"
TEMP_DIR=$(mktemp -d "${VENDOR_DIR}/.sdk-download.XXXXXX")
trap 'rm -rf "$TEMP_DIR"' EXIT
DOWNLOAD_URL="${BASE_URL}/${PLATFORM}.zip"
echo "[BrainCo Stark SDK] Downloading ${DOWNLOAD_URL}"
if command -v curl >/dev/null 2>&1; then
  curl -fL --retry 2 "$DOWNLOAD_URL" -o "${TEMP_DIR}/sdk.zip"
elif command -v wget >/dev/null 2>&1; then
  wget -O "${TEMP_DIR}/sdk.zip" "$DOWNLOAD_URL"
else
  echo "Install curl or wget to download the SDK" >&2
  exit 1
fi
unzip -q "${TEMP_DIR}/sdk.zip" -d "${TEMP_DIR}/unpacked"
PACKAGE_DIR="${TEMP_DIR}/unpacked"
PACKAGE_LIB=$(find "$PACKAGE_DIR/dist/shared" -type f -name "$LIB_NAME")
if [ -z "$PACKAGE_LIB" ] || ! valid_library "$PACKAGE_LIB" ||
   [ ! -s "$PACKAGE_DIR/dist/include/stark-sdk.h" ] || [ ! -s "$PACKAGE_DIR/LICENSE" ]; then
  echo "SDK archive is incomplete or has the wrong architecture" >&2
  exit 1
fi

mkdir -p "$(dirname "$LIB_PATH")" "$(dirname "$HEADER_PATH")"
# Invalidate the cache before changing any installed file.
rm -f "$PLATFORM_FILE"
cp "$PACKAGE_LIB" "$LIB_PATH"
cp "$PACKAGE_DIR/dist/include/stark-sdk.h" "$HEADER_PATH"
cp "$PACKAGE_DIR/LICENSE" "${VENDOR_DIR}/LICENSE"
printf '%s\n' "$SDK_VERSION" > "$VERSION_FILE"
printf '%s\n' "$PLATFORM_KEY" > "$PLATFORM_FILE"
echo "[BrainCo Stark SDK] Installed ${SDK_VERSION} for ${PLATFORM_KEY}"
