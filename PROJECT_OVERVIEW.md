# Project Overview: RPi4 YOLOv11 Object Detection

## 1. Goal

Implement a real-time object detection system using YOLOv11 on a Raspberry Pi 4 (8GB RAM).
The system must be developed and verified on an x86 PC using Docker, then deployed to the RPi4 (ARM64).

## 2. Target Environment

- **Device:** Raspberry Pi 4 Model B (8GB RAM)
- **OS:** Ubuntu 24.04 LTS (Noble Numbat) Server/Desktop
- **ROS Version:** ROS 2 Jazzy Jalisco
- **Containerization:** Docker (Multi-arch build required: linux/amd64, linux/arm64)

## 3. Host Environment (Development)

- **Device:** x86_64 PC (Windows)
- **Hardware:** USB Webcam connected (/dev/video0)
- **Tool:** Docker with Buildx support

## 4. Key Components

1.  **Object Detection:** Ultralytics YOLOv11 (Model: `yolo11n.pt` for performance).
2.  **Middleware:** ROS 2 Jazzy.
3.  **Camera Input:** `v4l2_camera` or `usb_cam` package.
4.  **Reference:** https://github.com/mgonzs13/yolo_ros.git (Use logic, but refactor for ROS 2 Jazzy compliance).

## 5. Constraints

- **NO CUDA:** The RPi4 has no NVIDIA GPU. Inference must run on CPU (optimize for NEON/SIMD if possible) or via TFLite/ONNX Runtime.
- **Python Version:** 3.12 (Strict compliance required, avoid `distutils` dependency errors).
- **Visualization:** Detection results must be viewable via `rqt_image_view` on the Host PC.
