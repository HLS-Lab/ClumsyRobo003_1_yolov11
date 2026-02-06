#!/bin/bash
# =============================================================================
# build_docker.sh - Build the Docker image for RPi4 YOLOv11 development
# =============================================================================

set -e  # Exit on error

IMAGE_NAME="yolo_rpi"
IMAGE_TAG="v1"

echo "============================================="
echo " Building Docker Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "============================================="

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Build the Docker image
docker build \
    -t ${IMAGE_NAME}:${IMAGE_TAG} \
    -f "${SCRIPT_DIR}/Dockerfile" \
    "${SCRIPT_DIR}"

echo ""
echo "============================================="
echo " Build Complete!"
echo " Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "============================================="
echo ""
echo "To run the container, use: ./run_dev.sh"
