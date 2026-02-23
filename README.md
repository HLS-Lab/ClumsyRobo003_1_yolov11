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
    │   └── yolo_params.yaml
    ├── launch/
    │   ├── yolo.launch.py          # YOLO node only
    │   ├── yolo_vision.launch.py   # Camera + YOLO + viewer (full pipeline)
    │   └── yolo_headless.launch.py # Camera + YOLO (RPi4 headless)
    └── yolo_rpi_core/
        ├── __init__.py
        └── yolo_node.py
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

### Test 2: ROS 2 Node Startup (Intermediate — two terminals needed)

**What this does:** Starts the YOLO detection node as a proper ROS 2 service. This is how the robot will actually use it — waiting for camera images and publishing detections.

**Terminal 1** — Start the YOLO node:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run yolo_rpi_core yolo_node --ros-args -p model_path:=yolo11n.pt
```

You should see:

```
[INFO] [yolo_node]: Loading YOLO model: yolo11n.pt
[INFO] [yolo_node]: Device: cpu
[INFO] [yolo_node]: YoloNode initialized and ready!
```

**Terminal 2** — Open a **new terminal window** and connect to the same container:

```bash
docker exec -it yolo_dev bash
```

Then verify the node is running:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 node list          # Should print: /yolo_node
ros2 topic list         # Should include: /image_raw, /yolo/detections, /yolo/debug_image
```

> ✅ If `ros2 node list` shows `/yolo_node` — **success!** The ROS 2 node is alive and waiting for camera images.

---

### Test 3: Full Vision Pipeline (Advanced — Linux with USB camera)

**What this does:** Runs the complete pipeline — camera captures images → YOLO detects objects → results are displayed in a live window with bounding boxes drawn on the video feed.

**Requirements:** Linux host with USB camera + X11 display (see `run_dev.sh`).

```bash
ros2 launch yolo_rpi_core yolo_vision.launch.py
```

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

| Topic               | Type                               | Description               |
| ------------------- | ---------------------------------- | ------------------------- |
| `/image_raw`        | `sensor_msgs/msg/Image`            | Input camera images       |
| `/yolo/detections`  | `vision_msgs/msg/Detection2DArray` | Detection results         |
| `/yolo/debug_image` | `sensor_msgs/msg/Image`            | Image with bounding boxes |

## Parameters

| Parameter        | Type   | Default      | Description          |
| ---------------- | ------ | ------------ | -------------------- |
| `model_path`     | string | `yolo11n.pt` | Path to YOLO weights |
| `device`         | string | `cpu`        | Inference device     |
| `conf_threshold` | float  | `0.5`        | Confidence threshold |
