# Project Rules (AGENTS.md)

You act as a Senior Embedded AI Engineer. Follow these rules strictly.

## 1. Docker First Policy

- Do NOT install dependencies on the host machine directly.
- All development environments must be defined in `Dockerfile`.
- When testing, always assume execution inside the container.

## 2. Architecture Awareness

- We are cross-compiling. Always check if a library is compatible with `linux/arm64`.
- For `Dockerfile`, use multi-stage builds or explicit platform arguments if necessary.
- Do not include NVIDIA/CUDA dependencies (`nvidia-container-toolkit`, `cuda-12-x`, `*cublas*`, `*nvidia*`) as the target is Raspberry Pi 4.

## 3. Code Quality & ROS 2 Standards

- Use `rclpy` for Python nodes.
- Structure the ROS 2 package properly: `package.xml`, `setup.py` (or `pyproject.toml`), and `resource` folders.
- Use explicit type hinting in Python code.
- Handle exceptions for Camera I/O and Model Inference to prevent node crashes.

## 4. Reference Handling

- The reference repo (`mgonzs13/yolo_ros`) might be outdated.
- **DO NOT** blindly clone and build it if it fails.
- Instead, read its source code to understand the logic, then **re-implement** a minimal, clean node compatible with ROS 2 Jazzy and Ultralytics 8.x+.

## 5. Interaction Protocol

- Before executing a complex shell command, briefly explain what it does.
- If an error occurs, analyze the log _before_ trying a random fix.
- Do not assume the user has a display connected to the RPi4; output should be essentially network-based (ROS Topics).
