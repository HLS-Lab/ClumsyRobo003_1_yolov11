# =============================================================================
# Dockerfile: RPi4 YOLOv11 + ROS 2 Jazzy Development Environment
# =============================================================================
# Base Image: osrf/ros:jazzy-desktop (Multi-arch: amd64, arm64)
# Target: Raspberry Pi 4 (arm64) and x86 PC (amd64) development
# =============================================================================

FROM osrf/ros:jazzy-desktop

# -----------------------------------------------------------------------------
# Environment Variables
# -----------------------------------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Workspace path
ENV ROS2_WS=/ros2_ws

# Prevent Python from writing .pyc files and enable unbuffered output
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# -----------------------------------------------------------------------------
# System Dependencies (apt)
# -----------------------------------------------------------------------------
# Update package list and install dependencies in a single layer to reduce image size
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools and utilities
    git \
    wget \
    curl \
    nano \
    # Python tools
    python3-pip \
    python3-venv \
    # ROS 2 camera and image processing packages
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-vision-msgs \
    # Additional ROS 2 packages for visualization and debugging
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-image-view \
    # OpenCV dependencies (required for cv_bridge and ultralytics)
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    # Video4Linux utilities for camera debugging
    v4l-utils \
    # Clean up apt cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Python Dependencies (pip) - CPU ONLY
# -----------------------------------------------------------------------------
# CRITICAL: No GPU on target (RPi4). Use CPU-only PyTorch.
#   - amd64: explicit CPU wheel index (prevents ~4GB CUDA download)
#   - arm64: standard PyPI (ARM64 wheels are CPU-only by default)

ARG TARGETARCH
RUN if [ "$TARGETARCH" = "arm64" ]; then \
    pip3 install --no-cache-dir --break-system-packages --ignore-installed \
    torch torchvision torchaudio; \
    else \
    pip3 install --no-cache-dir --break-system-packages --ignore-installed \
    torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/cpu; \
    fi

# Remove apt-installed packages that conflict with pip (missing RECORD files)
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed scipy

# Install ultralytics (will see CPU torch is already installed and skip it)
# NOTE: Do NOT use --ignore-installed here, otherwise pip re-downloads CUDA torch
RUN pip3 install --no-cache-dir --break-system-packages \
    ultralytics \
    setuptools \
    wheel

# -----------------------------------------------------------------------------
# ROS 2 Workspace Setup
# -----------------------------------------------------------------------------
# Create the workspace directory structure
RUN mkdir -p ${ROS2_WS}/src

# Set the workspace as the working directory
WORKDIR ${ROS2_WS}

# -----------------------------------------------------------------------------
# Environment Configuration
# -----------------------------------------------------------------------------
# Add ROS 2 environment sourcing to bashrc for interactive shells
RUN echo "" >> ~/.bashrc && \
    echo "# ROS 2 Jazzy Environment Setup" >> ~/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Workspace setup (if built)" >> ~/.bashrc && \
    echo "if [ -f ${ROS2_WS}/install/setup.bash ]; then" >> ~/.bashrc && \
    echo "    source ${ROS2_WS}/install/setup.bash" >> ~/.bashrc && \
    echo "fi" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Helpful aliases" >> ~/.bashrc && \
    echo "alias cb='cd ${ROS2_WS} && colcon build --symlink-install'" >> ~/.bashrc && \
    echo "alias cs='cd ${ROS2_WS}/src'" >> ~/.bashrc

# -----------------------------------------------------------------------------
# Default Command
# -----------------------------------------------------------------------------
# Start an interactive bash shell
CMD ["/bin/bash"]
