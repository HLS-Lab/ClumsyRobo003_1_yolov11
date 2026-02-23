#!/bin/bash
# =============================================================================
# build_arm64.sh - Cross-compile Docker image for Raspberry Pi 4 (ARM64)
# =============================================================================
# Uses Docker Buildx with QEMU emulation to build a linux/arm64 image
# on an x86_64 host. Exports the image as a .tar file for offline transfer.
#
# WARNING: Cross-compilation via QEMU is slow (~30-60 minutes for pip installs).
# =============================================================================

set -e

IMAGE_NAME="yolo_rpi"
IMAGE_TAG="arm64"
BUILDER_NAME="rpi4_builder"
OUTPUT_FILE="yolo_rpi_arm64.tar"

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "============================================="
echo " Cross-compiling for ARM64 (Raspberry Pi 4)"
echo " Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "============================================="

# -------------------------------------------------------------------------
# Step 1: Ensure QEMU emulators are registered
# -------------------------------------------------------------------------
echo "[1/4] Setting up QEMU emulation for ARM64..."
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes 2>/dev/null || true

# -------------------------------------------------------------------------
# Step 2: Create buildx builder (if it doesn't exist)
# -------------------------------------------------------------------------
echo "[2/4] Configuring Docker Buildx builder..."
if ! docker buildx inspect ${BUILDER_NAME} > /dev/null 2>&1; then
    echo "  Creating new builder: ${BUILDER_NAME}"
    docker buildx create --name ${BUILDER_NAME} --use
else
    echo "  Using existing builder: ${BUILDER_NAME}"
    docker buildx use ${BUILDER_NAME}
fi
docker buildx inspect --bootstrap

# -------------------------------------------------------------------------
# Step 3: Build the ARM64 image
# -------------------------------------------------------------------------
echo "[3/4] Building ARM64 image (this will take 30-60 minutes)..."
docker buildx build \
    --platform linux/arm64 \
    -t ${IMAGE_NAME}:${IMAGE_TAG} \
    -f "${SCRIPT_DIR}/Dockerfile" \
    --output type=docker \
    "${SCRIPT_DIR}"

# -------------------------------------------------------------------------
# Step 4: Export to .tar file
# -------------------------------------------------------------------------
echo "[4/4] Exporting image to ${OUTPUT_FILE}..."
docker save ${IMAGE_NAME}:${IMAGE_TAG} -o "${SCRIPT_DIR}/${OUTPUT_FILE}"

FILE_SIZE=$(du -h "${SCRIPT_DIR}/${OUTPUT_FILE}" | cut -f1)

echo ""
echo "============================================="
echo " Build Complete!"
echo " Image:  ${IMAGE_NAME}:${IMAGE_TAG}"
echo " File:   ${OUTPUT_FILE} (${FILE_SIZE})"
echo "============================================="
echo ""
echo " Transfer to RPi4:"
echo "   scp ${OUTPUT_FILE} pi@<RPI4_IP>:~/"
echo ""
echo " Load on RPi4:"
echo "   docker load -i ~/${OUTPUT_FILE}"
echo ""
echo " Run on RPi4:"
echo "   ./run_rpi4.sh"
