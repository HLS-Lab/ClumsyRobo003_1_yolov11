#!/bin/bash
# =============================================================================
# run_rpi4.sh - Run the Docker container on Raspberry Pi 4 (headless)
# =============================================================================
# Features:
#   - USB camera device mapping (/dev/video0)
#   - Host network for ROS 2 DDS discovery (enables remote viewing from PC)
#   - No X11 forwarding (headless operation to save resources)
#   - OMP_NUM_THREADS=4 (RPi4 has 4 cores)
#   - Project directory mounted for live development
# =============================================================================

set -e

IMAGE_NAME="yolo_rpi"
IMAGE_TAG="arm64"
CONTAINER_NAME="yolo_rpi4"

# Get the directory where this script is located (project root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "============================================="
echo " Starting RPi4 Container (Headless)"
echo " Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "============================================="

# Check if container already exists and remove it
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Removing existing container: ${CONTAINER_NAME}"
    docker rm -f ${CONTAINER_NAME}
fi

# Detect available video device
VIDEO_DEVICE="/dev/video0"
if [ ! -e "${VIDEO_DEVICE}" ]; then
    echo "WARNING: ${VIDEO_DEVICE} not found. Trying /dev/video1..."
    VIDEO_DEVICE="/dev/video1"
fi
if [ ! -e "${VIDEO_DEVICE}" ]; then
    echo "ERROR: No video device found. Connect a USB camera and retry."
    exit 1
fi
echo "Using camera: ${VIDEO_DEVICE}"

# Run container in headless mode
docker run -it \
    --name ${CONTAINER_NAME} \
    --device=${VIDEO_DEVICE}:${VIDEO_DEVICE} \
    --network=host \
    --privileged \
    -e OMP_NUM_THREADS=4 \
    -e OPENBLAS_NUM_THREADS=4 \
    -v "${SCRIPT_DIR}:/ros2_ws/src/yolo_rpi:rw" \
    -w /ros2_ws \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    /bin/bash

echo ""
echo "Container exited."
