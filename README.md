# ClumsyRobo003_1_yolov11

YOLOv11 object detection node for **ROS 2 Jazzy**, targeting the **Raspberry Pi 4** (CPU-only).

Runs inside a Docker container based on `osrf/ros:jazzy-desktop`, with CPU-only PyTorch and Ultralytics pre-installed.

> 📖 **한국어 문서는 [README_KOR.md](README_KOR.md)를 참고하세요.**

## Project Structure

```
├── Dockerfile              # ROS 2 Jazzy + YOLOv11 CPU-only environment
├── build_docker.sh         # Build x86 Docker image
├── build_arm64.sh          # Cross-compile ARM64 image for RPi4
├── run_dev.sh              # Start dev container (Linux, with camera/X11)
├── run_rpi4.sh             # Start headless container on RPi4
└── yolo_rpi_core/          # ROS 2 Python package
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── config/
    │   ├── yolo_params.yaml
    │   └── tracking_params.yaml    # Tracking & Actuator config
    ├── launch/
    │   ├── yolo.launch.py          # YOLO node only
    │   ├── yolo_vision.launch.py   # Camera + YOLO + viewer
    │   ├── yolo_tracking.launch.py # YOLO + Tracker + Dummy Actuator
    │   └── yolo_headless.launch.py # Camera + YOLO (RPi4 headless)
    └── yolo_rpi_core/
        ├── __init__.py
        ├── yolo_node.py
        ├── tracker_node.py         # Object tracking (Centroid/ByteTrack)
        ├── bytetrack_tracker.py    # ByteTrack algorithm (CPU-only)
        ├── base_actuator.py        # Abstract base class for control
        └── dummy_actuator_node.py  # Simulation of pan-tilt actuator
```

## Prerequisites

- **Docker** (Desktop or Engine) — [Install Guide](https://docs.docker.com/get-docker/)
- ~8 GB of free disk space

## Quick Start

### 1. Build the Docker Image

This step creates a ready-to-use environment with ROS 2, PyTorch (CPU), and YOLOv11.
It takes about **5–10 minutes** the first time (downloading base image + dependencies).

```bash
# Linux / macOS
./build_docker.sh

# Windows (PowerShell) — run from the project root folder
docker build -t yolo_rpi:v1 -f Dockerfile .
```

### 2. Start the Container

This opens an **interactive shell** inside the Docker container where all ROS 2 and YOLO tools are ready.

```bash
# Windows (PowerShell) — test-only mode (no camera)
docker run -it --rm --name yolo_dev `
  -v "${PWD}/yolo_rpi_core:/ros2_ws/src/yolo_rpi_core" `
  yolo_rpi:v1

# Linux — full mode with USB camera and GUI support
./run_dev.sh
```

> 💡 **What does `-v` do?** It "mounts" your project folder into the container, so any code changes you make on your PC are instantly visible inside the container — no need to rebuild!

### 3. Build the ROS 2 Package (inside the container)

Once inside the container (you'll see a prompt like `root@abc123:/ros2_ws#`), run:

```bash
cd /ros2_ws
colcon build --packages-select yolo_rpi_core --symlink-install
source install/setup.bash
```

> 💡 **What is `colcon build`?** It's the ROS 2 build tool. `--symlink-install` means your code changes take effect immediately without rebuilding. `source install/setup.bash` tells ROS 2 where to find our newly built package.

---

## Running Tests

### Test 1: YOLO Model Inference (Easiest — no camera needed)

**What this does:** Downloads a sample photo of a bus, runs YOLOv11 on it, and prints what objects were detected (people, buses, etc.). This confirms the AI model works on your machine.

**Run this inside the container:**

```bash
python3 -c "
from ultralytics import YOLO
model = YOLO('yolo11n.pt')
results = model('https://ultralytics.com/images/bus.jpg', conf=0.5)
print(f'Detected {len(results[0].boxes)} objects')
for box in results[0].boxes:
    cls = int(box.cls[0].item())
    conf = float(box.conf[0].item())
    print(f'  - {model.names[cls]}: {conf:.2f}')
"
```

**What to expect:**

```
Detected 4 objects
  - person: 0.88
  - person: 0.87
  - bus: 0.86
  - person: 0.82
```

> ✅ If you see detected objects with confidence scores — **success!** The YOLO model is working on CPU.
>
> ⚠️ The first run downloads the model file (~5 MB). This is normal.

---

### Test 2: PC Webcam Tracking Pipeline (No ROS 2 needed)

**What this does:** Tests the YOLO model and ByteTrack tracking algorithm directly using your PC's webcam. It works on Windows, Linux, and macOS natively without needing to start the Docker container or ROS 2. It automatically attempts to use connected USB cameras first before falling back to the built-in webcam.

**Run this on your host machine (with `uv` installed):**

```bash
uv run python test_webcam.py
```

**What to expect:**
A live camera window will open showing bounding boxes, tracking IDs, and velocity arrows for detected objects. Press 'q' to exit.

---

### Test 3: Object Tracking Integration (Internal Pipeline)

**What this does:** Simulates an object moving across a 640x480 frame and verifies the tracking pipeline (Tracker Node → Actuator Node). It tests ID assignment, error calculation, and state transitions without any hardware.

```bash
# Run with Centroid Tracker (default)
python3 src/yolo_rpi_core/test/test_tracking.py --tracker-type centroid

# Run with ByteTrack (Kalman Filter)
python3 src/yolo_rpi_core/test/test_tracking.py --tracker-type bytetrack
```

---

### Test 4: Full Vision Pipeline (Advanced — Linux with USB camera)

**What this does:** Runs the complete vision pipeline — camera captures images → YOLO detects objects → results are displayed in a live window with bounding boxes.

```bash
ros2 launch yolo_rpi_core yolo_vision.launch.py
```

---

### Test 5: Full Tracking Pipeline (Advanced — Linux with USB camera)

**What this does:** Runs the complete vision-control loop — Camera captures images → YOLO detects objects → Tracker Node selects target → Dummy Actuator logs movement commands.

```bash
ros2 launch yolo_rpi_core yolo_tracking.launch.py
```

---

## Object Tracking

This project supports two tracking algorithms (selected via `tracker_type` parameter):

1. **Centroid Tracker**: Simple and extremely fast. Best for single-object tracking with minimal occlusion.
2. **ByteTrack**: Uses **Kalman Filters** and IoU matching with a 2-stage association. Handles occlusion much better by reusing low-confidence detections. CPU-only and lightweight.

This starts three nodes at once:

1. `v4l2_camera` — reads frames from your USB camera
2. `yolo_node` — runs YOLO detection on each frame
3. `image_view` — shows the live video with bounding boxes

> ✅ If a window pops up showing your camera feed with colored bounding boxes around detected objects — **success!**

---

## RPi4 Deployment

### Step 1: Cross-compile the ARM64 image (on Host PC)

This builds the same Docker image but for the RPi4's ARM processor. It uses QEMU to emulate ARM64 on your x86 PC.

```bash
./build_arm64.sh
# Produces: yolo_rpi_arm64.tar (~3-4 GB)
# ⏱️ Takes 30-60 minutes — this is normal (QEMU emulation is slow)
```

### Step 2: Transfer to RPi4

Copy the image file to your Raspberry Pi over your local network:

```bash
scp yolo_rpi_arm64.tar pi@<RPI4_IP>:~/
```

> 💡 Replace `<RPI4_IP>` with your RPi4's IP address (e.g., `192.168.0.10`).

### Step 3: Load and run on RPi4

```bash
# On the RPi4:
docker load -i ~/yolo_rpi_arm64.tar    # Load the image into Docker
./run_rpi4.sh                           # Start the container (headless)

# Inside the container:
cd /ros2_ws
colcon build --packages-select yolo_rpi_core --symlink-install
source install/setup.bash
ros2 launch yolo_rpi_core yolo_headless.launch.py
```

### Step 4: Remote visualization from Host PC

Since both the RPi4 and your PC use `--network=host`, ROS 2 automatically discovers topics across the local network.

```bash
# On the Host PC (must be on the same Wi-Fi/LAN):
source /opt/ros/jazzy/setup.bash
ros2 topic list                                  # Should show RPi4's topics
ros2 run rqt_image_view rqt_image_view           # View /yolo/debug_image
```

> 💡 Both machines must use the same `ROS_DOMAIN_ID` (default is `0`, so this usually works out of the box).

---

## ROS 2 Topics

| Topic                   | Type                               | Description                 |
| ----------------------- | ---------------------------------- | --------------------------- |
| `/image_raw`            | `sensor_msgs/msg/Image`            | Input camera images         |
| `/yolo/detections`      | `vision_msgs/msg/Detection2DArray` | Raw YOLO detections         |
| `/tracking/command`     | `std_msgs/msg/String` (JSON)       | Target errors & velocity    |
| `/tracking/status`      | `std_msgs/msg/String` (JSON)       | Actuator state feedback     |
| `/yolo/debug_image`     | `sensor_msgs/msg/Image`            | Visualized YOLO results     |
| `/tracking/debug_image` | `sensor_msgs/msg/Image`            | Visualized Tracking results |

## Parameters

### YOLO Node

| Name             | Type   | Default      | Description          |
| ---------------- | ------ | ------------ | -------------------- |
| `model_path`     | string | `yolo11n.pt` | Path to YOLO weights |
| `device`         | string | `cpu`        | Inference device     |
| `conf_threshold` | float  | `0.5`        | Confidence threshold |

### Tracker Node

| Name                    | Type   | Default    | Description                           |
| ----------------------- | ------ | ---------- | ------------------------------------- |
| `tracker_type`          | string | `centroid` | Algorithm (`centroid` or `bytetrack`) |
| `tracking_target_class` | string | `person`   | Target class name to follow           |
| `max_disappeared`       | int    | `30`       | Frames to wait for lost target        |
| `max_distance`          | float  | `80.0`     | Range for Centroid matching (px)      |

---

## Architecture

For a detailed dive into the software design (OOP patterns, State Machines, JSON Schemas), please refer to **[ARCHITECTURE.md](ARCHITECTURE.md)**.
