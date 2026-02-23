# Architecture

## System Overview

A Dockerized ROS 2 Jazzy pipeline that captures USB camera frames,
runs YOLOv11 object detection on CPU, and publishes results over DDS.

```
┌──────────────────────────────────────────────────────────────────┐
│  Docker Container  (osrf/ros:jazzy-desktop)                      │
│                                                                  │
│  ┌──────────────┐    /image_raw    ┌───────────┐                 │
│  │ v4l2_camera   │ ──────────────► │ yolo_node │                 │
│  │ (640×480)     │   Image         │           │                 │
│  └──────────────┘                  │  YOLO     │                 │
│                                    │  CPU      │                 │
│        /dev/video0                 │  Infer    │                 │
│        (USB Cam)                   └─────┬─────┘                 │
│                                     ┌────┴────┐                  │
│                          /yolo/     │         │    /yolo/         │
│                       detections    │         │  debug_image      │
│                    Detection2DArray │         │    Image          │
│                                     ▼         ▼                  │
│                              [Downstream]  [image_view /         │
│                               Nodes         rqt_image_view]      │
└──────────────────────────────────────────────────────────────────┘
```

## Node: `yolo_node`

**Source:** `yolo_rpi_core/yolo_rpi_core/yolo_node.py`
**Class:** `YoloNode(rclpy.node.Node)`

### Subscriptions

| Topic        | Type                    | QoS                  | Purpose       |
| ------------ | ----------------------- | -------------------- | ------------- |
| `/image_raw` | `sensor_msgs/msg/Image` | BEST_EFFORT, depth=1 | Camera frames |

### Publications

| Topic               | Type                               | QoS                | Purpose                             |
| ------------------- | ---------------------------------- | ------------------ | ----------------------------------- |
| `/yolo/detections`  | `vision_msgs/msg/Detection2DArray` | RELIABLE, depth=10 | Structured detection results        |
| `/yolo/debug_image` | `sensor_msgs/msg/Image`            | RELIABLE, depth=10 | Annotated frame with bounding boxes |

### Parameters

| Name             | Type   | Default      | Description                    |
| ---------------- | ------ | ------------ | ------------------------------ |
| `model_path`     | string | `yolo11n.pt` | YOLO weight file path          |
| `device`         | string | `cpu`        | Inference device               |
| `conf_threshold` | float  | `0.5`        | Detection confidence threshold |

### Processing Pipeline

```
image_callback(msg)
  ├─ cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
  ├─ model(cv_image, conf=conf_threshold)
  ├─ _create_detection_array(result, header)
  │   └─ box.xyxy → center/size → Detection2D
  │       └─ class_id + confidence → ObjectHypothesisWithPose
  ├─ detection_pub.publish(detection_array)
  ├─ result.plot() → debug_cv_image
  └─ debug_image_pub.publish(debug_msg)
```

## Launch Files

| File                      | Nodes Started                              | Use Case                  |
| ------------------------- | ------------------------------------------ | ------------------------- |
| `yolo.launch.py`          | `yolo_node`                                | Node-only (manual camera) |
| `yolo_vision.launch.py`   | `v4l2_camera` + `yolo_node` + `image_view` | PC development with GUI   |
| `yolo_headless.launch.py` | `v4l2_camera` + `yolo_node`                | RPi4 headless deployment  |

## Docker Architecture

### Base Image

`osrf/ros:jazzy-desktop` — multi-arch (`amd64`, `arm64`)

### Key Dependencies

| Layer            | Packages                                                                           |
| ---------------- | ---------------------------------------------------------------------------------- |
| **ROS 2 (apt)**  | `v4l2-camera`, `cv-bridge`, `vision-msgs`, `image-transport-plugins`, `image-view` |
| **Python (pip)** | `torch` (CPU-only), `torchvision`, `ultralytics`, `scipy`                          |

### Multi-arch Build Strategy

```dockerfile
ARG TARGETARCH
# amd64 → --index-url .../whl/cpu  (explicit CPU wheels, avoids CUDA)
# arm64 → standard PyPI             (ARM64 has no CUDA, already CPU-only)
```

### Volume Mounts

| Host Path          | Container Path                | Purpose                |
| ------------------ | ----------------------------- | ---------------------- |
| `./yolo_rpi_core/` | `/ros2_ws/src/yolo_rpi_core/` | Live code editing      |
| `/dev/video0`      | `/dev/video0`                 | USB camera device      |
| `/tmp/.X11-unix`   | `/tmp/.X11-unix`              | X11 display (dev only) |

## Deployment Modes

### Development (x86 Host PC)

```
Host PC ──► run_dev.sh ──► Docker (amd64)
                              ├─ v4l2_camera
                              ├─ yolo_node
                              └─ image_view (X11)
```

### Production (Raspberry Pi 4)

```
Host PC ──► build_arm64.sh ──► yolo_rpi_arm64.tar
                                    │
                               scp to RPi4
                                    │
RPi4 ──► docker load ──► run_rpi4.sh ──► Docker (arm64, headless)
                                            ├─ v4l2_camera
                                            └─ yolo_node
                                                  │
                                            DDS over Wi-Fi
                                                  │
Host PC ◄── rqt_image_view ◄── /yolo/debug_image ─┘
```

### RPi4 Runtime Environment

| Setting                | Value | Reason                          |
| ---------------------- | ----- | ------------------------------- |
| `OMP_NUM_THREADS`      | 4     | Match RPi4 quad-core Cortex-A72 |
| `OPENBLAS_NUM_THREADS` | 4     | Linear algebra thread control   |
| `--network=host`       | —     | DDS multicast discovery         |
| `--privileged`         | —     | `/dev/video*` access            |

## Object Tracking Pipeline

### System Diagram (Detection + Tracking)

```
                    /image_raw                /yolo/detections
  v4l2_camera ──────────────► yolo_node ────────────────────────┐
                                  │                             │
                          /yolo/debug_image              ┌──────▼──────┐
                                  │                      │ tracker_node │
                                  ▼                      │  (centroid   │
                            [image_view]                 │   tracking)  │
                                                         └──────┬──────┘
                                                                │
                                                  /tracking/command
                                              (std_msgs/String JSON)
                                                                │
                                                     ┌──────────▼──────────┐
                                                     │ dummy_actuator_node │
                                                     │  (BaseActuator →    │
                                                     │   DummyActuator)    │
                                                     └──────────┬──────────┘
                                                                │
                                                  /tracking/status
                                              (std_msgs/String JSON)
```

## Node: `tracker_node`

**Source:** `yolo_rpi_core/yolo_rpi_core/tracker_node.py`
**Classes:** `CentroidTracker` (pure Python) + `TrackerNode(rclpy.node.Node)`

### Algorithm: Centroid Tracking

Lightweight CPU-friendly tracker that matches detections across frames by
minimizing Euclidean distance between bounding-box centers. No re-ID network
required (suitable for RPi4).

### Subscriptions

| Topic              | Type                               | QoS                  | Purpose              |
| ------------------ | ---------------------------------- | -------------------- | -------------------- |
| `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | RELIABLE, depth=10   | YOLO detection input |
| `/image_raw`       | `sensor_msgs/msg/Image`            | BEST_EFFORT, depth=1 | Debug image source   |

### Publications

| Topic                   | Type                    | QoS                | Purpose                            |
| ----------------------- | ----------------------- | ------------------ | ---------------------------------- |
| `/tracking/command`     | `std_msgs/msg/String`   | RELIABLE, depth=10 | JSON tracking command for actuator |
| `/tracking/debug_image` | `sensor_msgs/msg/Image` | RELIABLE, depth=10 | Frame with tracking overlay        |

### Tracking Command JSON Schema

```json
{
  "target_id": 0,
  "error_x": -0.375,
  "error_y": -0.375,
  "velocity_x": 10.0,
  "velocity_y": 5.0,
  "bbox_width": 80.0,
  "bbox_height": 100.0,
  "class_name": "person",
  "confidence": 0.85,
  "timestamp": 1234567890.123456
}
```

### Parameters

| Name                    | Type   | Default  | Description                              |
| ----------------------- | ------ | -------- | ---------------------------------------- |
| `tracking_target_class` | string | `person` | YOLO class to track (`all` = any class)  |
| `image_width`           | int    | `640`    | Camera frame width (pixels)              |
| `image_height`          | int    | `480`    | Camera frame height (pixels)             |
| `max_disappeared`       | int    | `30`     | Frames before deregistering lost object  |
| `max_distance`          | float  | `80.0`   | Max pixel distance for centroid matching |

## Node: `dummy_actuator_node`

**Source:** `yolo_rpi_core/yolo_rpi_core/dummy_actuator_node.py`
**Inherits:** `BaseActuatorNode` → `DummyActuatorNode`

### OOP Inheritance Hierarchy

```
rclpy.node.Node
    └── BaseActuatorNode (ABC)    ← base_actuator.py
            ├── DummyActuatorNode     ← dummy_actuator_node.py (logs only)
            ├── ServoActuatorNode     ← (future: PWM servo control)
            └── StepperActuatorNode   ← (future: stepper motor control)
```

### Subscriptions

| Topic               | Type                  | QoS                | Purpose          |
| ------------------- | --------------------- | ------------------ | ---------------- |
| `/tracking/command` | `std_msgs/msg/String` | RELIABLE, depth=10 | Tracking command |

### Publications

| Topic              | Type                  | QoS                | Purpose         |
| ------------------ | --------------------- | ------------------ | --------------- |
| `/tracking/status` | `std_msgs/msg/String` | RELIABLE, depth=10 | Actuator status |

### Actuator State Machine

```
IDLE ──────► TRACKING  (first valid command)
TRACKING ──► IDLE      (target lost / stop)
TRACKING ──► ERROR     (hardware failure)
ERROR ─────► IDLE      (recovery / reset)
```

## Launch Files

| File                      | Nodes Started                                                        | Use Case                  |
| ------------------------- | -------------------------------------------------------------------- | ------------------------- |
| `yolo.launch.py`          | `yolo_node`                                                          | Node-only (manual camera) |
| `yolo_vision.launch.py`   | `v4l2_camera` + `yolo_node` + `image_view`                           | PC development with GUI   |
| `yolo_headless.launch.py` | `v4l2_camera` + `yolo_node`                                          | RPi4 headless deployment  |
| `yolo_tracking.launch.py` | `v4l2_camera` + `yolo_node` + `tracker_node` + `dummy_actuator_node` | Full tracking pipeline    |

## File Map

```
.
├── Dockerfile                  # Multi-arch container definition
├── build_docker.sh             # Build x86 image
├── build_arm64.sh              # Cross-compile ARM64 image (QEMU + buildx)
├── run_dev.sh                  # x86 dev container (camera + X11)
├── run_rpi4.sh                 # RPi4 headless container
└── yolo_rpi_core/
    ├── package.xml             # ROS 2 package manifest (ament_python)
    ├── setup.py                # Python package config + entry points
    ├── setup.cfg               # Console script install paths
    ├── config/
    │   ├── yolo_params.yaml    # Default YOLO node parameters
    │   └── tracking_params.yaml # Tracker + actuator parameters
    ├── launch/
    │   ├── yolo.launch.py
    │   ├── yolo_vision.launch.py
    │   ├── yolo_headless.launch.py
    │   └── yolo_tracking.launch.py   # Full tracking pipeline
    ├── resource/
    │   └── yolo_rpi_core       # ament resource index marker
    ├── test/
    │   └── test_tracking.py    # Integration test (init point based)
    └── yolo_rpi_core/
        ├── __init__.py
        ├── yolo_node.py        # YoloNode implementation (234 lines)
        ├── tracker_node.py     # CentroidTracker + TrackerNode
        ├── base_actuator.py    # Abstract BaseActuatorNode (ABC)
        └── dummy_actuator_node.py  # DummyActuatorNode (log-only)
```
