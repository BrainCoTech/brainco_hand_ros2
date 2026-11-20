#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENDOR_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)/vendor"
SDK_VERSION="v2.0.9"
BASE_URL="https://app.brainco.cn/universal/bc-stark-sdk/libs/${SDK_VERSION}"
VERSION_FILE="${VENDOR_DIR}/VERSION"
PLATFORM_FILE="${VENDOR_DIR}/.sdk-platform"
OS_TYPE=$(uname -s)
ARCH=$(uname -m)

case "${OS_TYPE}:${ARCH}" in
  Linux:x86_64)
    PLATFORM="linux"; LIB_PLATFORM="linux"; LIB_NAME="libbc_stark_sdk.so"; ELF_ARCH="x86-64" ;;
  Linux:aarch64|Linux:arm64)
    PLATFORM="linux-arm64"; LIB_PLATFORM="linux"; LIB_NAME="libbc_stark_sdk.so"; ELF_ARCH="ARM aarch64" ;;
  Darwin:arm64|Darwin:x86_64)
    PLATFORM="mac"; LIB_PLATFORM="mac"; LIB_NAME="libbc_stark_sdk.dylib" ;;
  MINGW*:x86_64|MSYS_NT*:x86_64)
    PLATFORM="win-cpp"; LIB_PLATFORM="win"; LIB_NAME="bc_stark_sdk.dll" ;;
  *) echo "Unsupported platform: ${OS_TYPE}/${ARCH}" >&2; exit 1 ;;
esac

LIB_PATH="${VENDOR_DIR}/dist/shared/${LIB_PLATFORM}/${LIB_NAME}"
HEADER_PATH="${VENDOR_DIR}/dist/include/stark-sdk.h"
PLATFORM_KEY="${PLATFORM}:${ARCH}"

if [ "$OS_TYPE" = Linux ] && ! command -v file >/dev/null 2>&1; then
  echo "Install 'file' to validate the SDK library architecture" >&2
  exit 1
fi

valid_library() {
  [ -s "$1" ] || return 1
  if [ "$OS_TYPE" = Linux ]; then
    local description
    description=$(file -b "$1")
    [[ "$description" == *"ELF 64-bit"* && "$description" == *"shared object"* &&
       "$description" == *"${ELF_ARCH}"* ]]
  fi
}

# A version marker alone does not prove that the SDK is installed for this machine.
if [ -f "$VERSION_FILE" ] && [ "$(cat "$VERSION_FILE")" = "$SDK_VERSION" ] &&
   [ -f "$PLATFORM_FILE" ] && [ "$(cat "$PLATFORM_FILE")" = "$PLATFORM_KEY" ] &&
   [ -s "$HEADER_PATH" ] && [ -s "${VENDOR_DIR}/LICENSE" ] && valid_library "$LIB_PATH"; then
  echo "[BrainCo Stark SDK] ${SDK_VERSION} for ${PLATFORM_KEY} is already installed"
  exit 0
fi

for dependency in unzip find mktemp; do
  if ! command -v "$dependency" >/dev/null 2>&1; then
    echo "Install '${dependency}' to download and extract the SDK" >&2
    exit 1
  fi
done

if command -v curl >/dev/null 2>&1; then
  downloader="curl"
elif command -v wget >/dev/null 2>&1; then
  downloader="wget"
else
  echo "Install curl or wget to download the SDK" >&2
  exit 1
fi

# Extract and validate before changing the installed SDK or its cache metadata.
mkdir -p "$VENDOR_DIR"
TEMP_DIR=$(mktemp -d "${VENDOR_DIR}/.sdk-download.XXXXXX")
trap 'rm -rf "$TEMP_DIR"' EXIT
DOWNLOAD_URL="${BASE_URL}/${PLATFORM}.zip"
echo "[BrainCo Stark SDK] Downloading ${DOWNLOAD_URL}"
if [ "$downloader" = curl ]; then
  curl -fL --connect-timeout 15 --max-time 180 --retry 2 --retry-max-time 240 \
    "$DOWNLOAD_URL" -o "${TEMP_DIR}/sdk.zip"
else
  wget --timeout=30 --tries=3 -O "${TEMP_DIR}/sdk.zip" "$DOWNLOAD_URL"
fi
unzip -q "${TEMP_DIR}/sdk.zip" -d "${TEMP_DIR}/unpacked"
PACKAGE_DIR="${TEMP_DIR}/unpacked"
if [ ! -d "$PACKAGE_DIR/dist/shared" ] ||
   [ ! -s "$PACKAGE_DIR/dist/include/stark-sdk.h" ] || [ ! -s "$PACKAGE_DIR/LICENSE" ]; then
  echo "SDK archive is incomplete (library directory, header or LICENSE missing)" >&2
  exit 1
fi

PACKAGE_LIBS=()
while IFS= read -r -d '' candidate; do
  PACKAGE_LIBS+=("$candidate")
done < <(find "$PACKAGE_DIR/dist/shared" -type f -name "$LIB_NAME" -print0)
if [ "${#PACKAGE_LIBS[@]}" -ne 1 ] || ! valid_library "${PACKAGE_LIBS[0]}"; then
  echo "SDK archive must contain exactly one nonempty ${LIB_NAME} for ${OS_TYPE}/${ARCH}" >&2
  exit 1
fi

mkdir -p "$(dirname "$LIB_PATH")" "$(dirname "$HEADER_PATH")"
# If installation is interrupted, a missing platform marker forces a fresh install.
rm -f "$PLATFORM_FILE"
cp "${PACKAGE_LIBS[0]}" "$LIB_PATH"
cp "$PACKAGE_DIR/dist/include/stark-sdk.h" "$HEADER_PATH"
cp "$PACKAGE_DIR/LICENSE" "${VENDOR_DIR}/LICENSE"
printf '%s\n' "$SDK_VERSION" > "$VERSION_FILE"
printf '%s\n' "$PLATFORM_KEY" > "$PLATFORM_FILE"
echo "[BrainCo Stark SDK] Installed ${SDK_VERSION} for ${PLATFORM_KEY}"
