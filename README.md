# ClumsyRobo003_1_yolov11

YOLOv11 object detection node for **ROS 2 Jazzy**, targeting the **Raspberry Pi 4** (CPU-only).

Runs inside a Docker container based on `osrf/ros:jazzy-desktop`, with CPU-only PyTorch and Ultralytics pre-installed.

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
    │   ├── yolo_vision.launch.py   # Camera + YOLO + viewer
    │   └── yolo_headless.launch.py # Camera + YOLO (RPi4 headless)
    └── yolo_rpi_core/
        ├── __init__.py
        └── yolo_node.py
```

## Prerequisites

- **Docker** (Desktop or Engine)
- ~8 GB of disk space

## Quick Start

### 1. Build the Docker Image

```bash
# Linux / macOS
./build_docker.sh

# Windows (PowerShell)
docker build -t yolo_rpi:v1 -f Dockerfile .
```

### 2. Start the Container

```bash
# Windows (PowerShell) — no camera, test-only
docker run -it --rm --name yolo_dev `
  -v "${PWD}/yolo_rpi_core:/ros2_ws/src/yolo_rpi_core" `
  yolo_rpi:v1

# Linux — with camera and GUI
./run_dev.sh
```

### 3. Build the ROS 2 Package (inside the container)

```bash
cd /ros2_ws
colcon build --packages-select yolo_rpi_core --symlink-install
source install/setup.bash
```

## Running Tests

### Test 1: YOLO Model Inference (no camera needed)

Verifies the model loads and runs detection on a sample image:

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

Expected output:

```
Detected 4 objects
  - person: 0.88
  - person: 0.87
  - bus: 0.86
  - person: 0.82
```

### Test 2: ROS 2 Node Startup (two terminals)

**Terminal 1** — Start the YOLO node:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run yolo_rpi_core yolo_node --ros-args -p model_path:=yolo11n.pt
```

**Terminal 2** — Open a second shell (`docker exec -it yolo_dev bash`):

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# Verify node and topics
ros2 node list          # → /yolo_node
ros2 topic list         # → /image_raw, /yolo/detections, /yolo/debug_image
```

### Test 3: Full Vision Pipeline (Linux with camera)

```bash
ros2 launch yolo_rpi_core yolo_vision.launch.py
```

Starts `v4l2_camera` + `yolo_node` + `image_view` together.

## RPi4 Deployment

### Step 1: Cross-compile the ARM64 image (on Host PC)

```bash
./build_arm64.sh
# Produces: yolo_rpi_arm64.tar (~3-4 GB)
# Takes 30-60 minutes due to QEMU emulation
```

### Step 2: Transfer to RPi4

```bash
scp yolo_rpi_arm64.tar pi@<RPI4_IP>:~/
```

### Step 3: Load and run on RPi4

```bash
# On the RPi4:
docker load -i ~/yolo_rpi_arm64.tar
./run_rpi4.sh

# Inside the container:
cd /ros2_ws
colcon build --packages-select yolo_rpi_core --symlink-install
source install/setup.bash
ros2 launch yolo_rpi_core yolo_headless.launch.py
```

### Step 4: Remote visualization from Host PC

Since both RPi4 and PC use `--network=host`, ROS 2 DDS discovers topics across the network automatically.

```bash
# On the Host PC (same Wi-Fi network):
source /opt/ros/jazzy/setup.bash
ros2 topic list                    # Should show RPi4's topics
ros2 run rqt_image_view rqt_image_view  # Subscribe to /yolo/debug_image
```

> **Note:** Both machines must use the same `ROS_DOMAIN_ID` (default: 0).

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
