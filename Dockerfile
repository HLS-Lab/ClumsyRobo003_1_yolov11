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
# Python Dependencies (pip)
# -----------------------------------------------------------------------------
# Note: Using --break-system-packages because this is a single-purpose container.
# Container-level isolation already exists, and this avoids complexity with
# ROS 2 environment sourcing when using virtual environments.
# --ignore-installed is needed because numpy is pre-installed via apt and cannot
# be uninstalled by pip (RECORD file not found).

# CRITICAL: Install CPU-only PyTorch FIRST to avoid downloading CUDA dependencies.
# The RPi4 has no NVIDIA GPU, so we use the CPU-only wheel from pytorch.org.
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu

# Install ultralytics WITHOUT torch (already installed above)
# --ignore-installed needed for scipy conflict with system package
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
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
