#!/bin/bash
set -e # Exit on error

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Vendor directory should be in package root
PACKAGE_ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENDOR_DIR="${PACKAGE_ROOT_DIR}/vendor"
VERSION_FILE="${VENDOR_DIR}/VERSION"
SDK_DIR="${VENDOR_DIR}/brainco_stark_sdk"

# Configuration
SDK_VERSION="v1.0.0"
BASE_URL="https://app.brainco.cn/universal/bc-stark-sdk/libs/${SDK_VERSION}"

# Colorful echo functions
echo_y() { echo -e "\033[1;33m$*\033[0m"; } # Yellow
echo_g() { echo -e "\033[0;32m$*\033[0m"; } # Green
echo_r() { echo -e "\033[0;31m$*\033[0m"; } # Red

# Check if version is already installed
if [ -f "$VERSION_FILE" ] && grep -F --quiet "$SDK_VERSION" "$VERSION_FILE"; then
  echo_g "[BrainCo Stark SDK] (${SDK_VERSION}) is already installed"
  cat "$VERSION_FILE"
  exit 0
fi

# Determine platform and library name
OS_TYPE=$(uname -s)
ARCH=$(uname -m)
echo_y "OS type: $OS_TYPE, ARCH: $ARCH"

case "$OS_TYPE" in
"Linux")
  if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    PLATFORM="linux-arm64"
  else
    PLATFORM="linux"
  fi
  LIB_NAME="libbc_stark_sdk.so"
  ;;
"Darwin")
  PLATFORM="mac"
  LIB_NAME="libbc_stark_sdk.dylib"
  ;;
"MSYS_NT"*|"MINGW"*)
  PLATFORM="win"
  LIB_NAME="bc_stark_sdk.dll"
  ;;
*)
  echo_r "Unsupported platform: $OS_TYPE"
  exit 1
  ;;
esac

# Create vendor directory
mkdir -p "$VENDOR_DIR"

# Download URL
DOWNLOAD_URL="${BASE_URL}/${PLATFORM}.zip"
ZIP_NAME="brainco_stark_sdk_${SDK_VERSION}_${PLATFORM}.zip"

echo_y "[BrainCo Stark SDK] Downloading (${SDK_VERSION}) for ${PLATFORM}..."

# Check if wget or curl is available
if command -v wget >/dev/null 2>&1; then
  DOWNLOADER="wget -q --show-progress -O"
elif command -v curl >/dev/null 2>&1; then
  DOWNLOADER="curl -L -# -o"
else
  echo_r "Error: Neither wget nor curl is installed. Please install one and try again."
  exit 1
fi

# Download
$DOWNLOADER "${SCRIPT_DIR}/${ZIP_NAME}" "$DOWNLOAD_URL" || {
  echo_r "Error: Failed to download ${ZIP_NAME}"
  exit 1
}

# Extract
echo_y "[BrainCo Stark SDK] Extracting ${ZIP_NAME}..."
unzip -o -q "${SCRIPT_DIR}/${ZIP_NAME}" -d "$VENDOR_DIR" || {
  echo_r "Error: Failed to unzip ${ZIP_NAME}"
  exit 1
}

# Clean up
rm -f "${SCRIPT_DIR}/${ZIP_NAME}"
rm -rf "${VENDOR_DIR}/__MACOSX"

# Write version file
echo "$SDK_VERSION" > "$VERSION_FILE"
echo_g "[BrainCo Stark SDK] (${SDK_VERSION}) installed successfully!"
echo "Location: ${VENDOR_DIR}"
echo "Include: ${VENDOR_DIR}/dist/include"
echo "Library: ${VENDOR_DIR}/dist/shared"
