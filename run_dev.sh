#!/bin/bash
# =============================================================================
# run_dev.sh - Run the Docker container for x86 PC development
# =============================================================================
# Features:
#   - X11 forwarding for GUI applications (rqt, rqt_image_view, etc.)
#   - USB camera device mapping (/dev/video0)
#   - Host network for ROS 2 DDS discovery
#   - Project directory mounted for live development
# =============================================================================

set -e  # Exit on error

IMAGE_NAME="yolo_rpi"
IMAGE_TAG="v1"
CONTAINER_NAME="yolo_dev"

# Get the directory where this script is located (project root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "============================================="
echo " Starting Development Container"
echo " Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "============================================="

# Check if container already exists and remove it
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Removing existing container: ${CONTAINER_NAME}"
    docker rm -f ${CONTAINER_NAME}
fi

# Run the container with development settings
# - X11 Forwarding: -e DISPLAY, -v /tmp/.X11-unix (for GUI apps like rqt)
# - Camera: --device=/dev/video0
# - Network: --network=host (for ROS 2 DDS discovery)
# - Privileges: --privileged (for hardware access)
# - Volume: Mount project for live development
docker run -it \
    --name ${CONTAINER_NAME} \
    -e DISPLAY=${DISPLAY:-:0} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device=/dev/video0:/dev/video0 \
    --network=host \
    --privileged \
    -v "${SCRIPT_DIR}:/ros2_ws/src/yolo_rpi:rw" \
    -w /ros2_ws \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    /bin/bash

echo ""
echo "Container exited."
