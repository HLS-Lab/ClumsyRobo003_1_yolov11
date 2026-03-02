# ClumsyRobo003_1_yolov11

**ROS 2 Jazzy** 기반 YOLOv11 객체 탐지 노드 — **Raspberry Pi 4** (CPU 전용) 대상

`osrf/ros:jazzy-desktop` 기반 Docker 컨테이너 안에서 CPU 전용 PyTorch와 Ultralytics가 사전 설치된 환경에서 동작합니다.

> 📖 **English documentation: [README.md](README.md)**

## 프로젝트 구조

```
├── Dockerfile              # ROS 2 Jazzy + YOLOv11 CPU 전용 환경
├── build_docker.sh         # x86 Docker 이미지 빌드
├── build_arm64.sh          # RPi4용 ARM64 이미지 크로스 컴파일
├── run_dev.sh              # 개발용 컨테이너 실행 (Linux, 카메라/X11 포함)
├── run_rpi4.sh             # RPi4에서 헤드리스 컨테이너 실행
└── yolo_rpi_core/          # ROS 2 Python 패키지
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── config/
    │   ├── yolo_params.yaml
    │   └── tracking_params.yaml    # 추적 및 액추에이터 설정
    ├── launch/
    │   ├── yolo.launch.py          # YOLO 노드만 실행
    │   ├── yolo_vision.launch.py   # 카메라 + YOLO + 뷰어
    │   ├── yolo_tracking.launch.py # YOLO + 추적기 + 더미 액추에이터
    │   └── yolo_headless.launch.py # 카메라 + YOLO (RPi4 헤드리스)
    └── yolo_rpi_core/
        ├── __init__.py
        ├── yolo_node.py
        ├── tracker_node.py              # 객체 추적 (Centroid/ByteTrack)
        ├── bytetrack_tracker.py         # ByteTrack 알고리즘 (CPU 전용)
        ├── base_actuator.py             # 제어용 추상 베이스 클래스
        ├── dummy_actuator_node.py       # 팬-틸트 액추에이터 시뮬레이션
        ├── serial_motor_controller.py   # UART/JSON 어댑터 (control_v1 프로토콜)
        └── serial_motor_actuator_node.py # 실제 모터 액추에이터 (USB-시리얼)
```

## 사전 요구사항

- **Docker** (Desktop 또는 Engine) — [설치 가이드](https://docs.docker.com/get-docker/)
- 여유 디스크 공간 약 8 GB

## 빠른 시작

### 1. Docker 이미지 빌드

ROS 2, PyTorch (CPU), YOLOv11이 포함된 환경을 생성합니다.
처음 실행 시 기본 이미지와 종속 패키지를 다운로드하므로 **5~10분** 정도 소요됩니다.

```bash
# Linux / macOS
./build_docker.sh

# Windows (PowerShell) — 프로젝트 루트 폴더에서 실행
docker build -t yolo_rpi:v1 -f Dockerfile .
```

### 2. 컨테이너 시작

Docker 컨테이너 내부에 **대화형 셸**을 열어, 모든 ROS 2 및 YOLO 도구를 바로 사용할 수 있습니다.

```bash
# Windows (PowerShell) — 테스트 전용 (카메라 없음)
docker run -it --rm --name yolo_dev `
  -v "${PWD}/yolo_rpi_core:/ros2_ws/src/yolo_rpi_core" `
  yolo_rpi:v1

# Linux — USB 카메라 및 GUI 지원 포함
./run_dev.sh
```

> 💡 **`-v` 옵션이란?** 프로젝트 폴더를 컨테이너 내부에 "마운트"합니다. PC에서 코드를 수정하면 컨테이너 안에서도 즉시 반영됩니다 — 다시 빌드할 필요가 없습니다!

### 3. ROS 2 패키지 빌드 (컨테이너 내부에서)

컨테이너 안에 들어가면 `root@abc123:/ros2_ws#` 같은 프롬프트가 보입니다. 다음 명령을 실행하세요:

```bash
cd /ros2_ws
colcon build --packages-select yolo_rpi_core --symlink-install
source install/setup.bash
```

> 💡 **`colcon build`란?** ROS 2 빌드 도구입니다. `--symlink-install`을 사용하면 코드 변경 사항이 다시 빌드하지 않아도 즉시 반영됩니다. `source install/setup.bash`는 ROS 2에게 새로 빌드한 패키지의 위치를 알려줍니다.

---

## 테스트 실행

### 테스트 1: YOLO 모델 추론 (가장 쉬움 — 카메라 필요 없음)

**이 테스트가 하는 일:** 버스 사진 샘플을 다운로드하고, YOLOv11으로 분석하여 감지된 객체(사람, 버스 등)를 출력합니다. AI 모델이 정상적으로 작동하는지 확인하는 가장 간단한 방법입니다.

**컨테이너 내부에서 실행:**

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

**예상 출력:**

```
Detected 4 objects
  - person: 0.88
  - person: 0.87
  - bus: 0.86
  - person: 0.82
```

> ✅ 신뢰도 점수와 함께 탐지된 객체가 보이면 — **성공!** CPU에서 YOLO 모델이 정상 작동합니다.
>
> ⚠️ 처음 실행 시 모델 파일(~5 MB)을 다운로드합니다. 이것은 정상입니다.

---

### 테스트 2: PC 웹캠 추적 파이프라인 (ROS 2 불필요)

**이 테스트가 하는 일:** PC의 웹캠을 사용하여 YOLO 모델과 ByteTrack 추적 알고리즘을 직접 테스트합니다. Docker 컨테이너나 ROS 2를 실행할 필요 없이 Windows, Linux, macOS에서 기본적으로 작동합니다. 연결된 외부 USB 카메라를 먼저 사용하려고 시도하며 실패할 경우 내장 웹캠으로 대체합니다.

**호스트 PC에서 실행 (`uv` 설치 필요):**

```bash
uv run python test_webcam.py
```

**예상 출력:**
탐지된 객체의 바운딩 박스, 추적 ID, 속도 화살표가 그려진 실시간 카메라 창이 열립니다. 종료하려면 'q' 키를 누르세요.

---

### 테스트 3: 객체 추적 통합 (내부 파이프라인)

**이 테스트가 하는 일:** 640x480 프레임에서 움직이는 객체를 시뮬레이션하고 추적 파이프라인(추적 노드 → 액추에이터 노드)을 검증합니다. 하드웨어 없이 ID 할당, 오차 계산, 상태 전이를 테스트합니다.

```bash
# Centroid Tracker로 실행 (기본값)
python3 src/yolo_rpi_core/test/test_tracking.py --tracker-type centroid

# ByteTrack으로 실행 (Kalman Filter)
python3 src/yolo_rpi_core/test/test_tracking.py --tracker-type bytetrack
```

---

### 테스트 4: 전체 비전 파이프라인 (고급 — USB 카메라가 있는 Linux 환경)

**이 테스트가 하는 일:** 전체 비전 파이프라인을 실행합니다 — 카메라가 이미지 캡처 → YOLO가 객체 탐지 → 실시간 화면에 바운딩 박스를 표시합니다.

```bash
ros2 launch yolo_rpi_core yolo_vision.launch.py
```

---

### 테스트 5: 전체 추적 파이프라인 (고급 — USB 카메라가 있는 Linux 환경)

**이 테스트가 하는 일:** 전체 비전 제어 루프를 실행합니다 — 카메라 이미지 캡처 → YOLO 객체 탐지 → 추적 노드가 타겟 선택 → 더미 액추에이터가 제어 명령을 로그로 기록합니다.

```bash
ros2 launch yolo_rpi_core yolo_tracking.launch.py
```

---

### 테스트 6: 시리얼 모터 액추에이터 — 시뮬레이션 모드 (하드웨어 불필요)

**이 테스트가 하는 일:** `SerialMotorActuatorNode`를 **시뮬레이션 모드**로 실행합니다 (USB-시리얼 장치 불필요). 사람이 감지되면 두 모터 모두 **90° (중심)** 명령을 내립니다. IDLE → TRACKING 상태 머신과 모터 상태 메시지를 검증합니다.

```bash
# 먼저 빌드 및 소스 적용 (컨테이너 내부)
colcon build --packages-select yolo_rpi_core && source install/setup.bash

# 하드웨어 없이 테스트 실행
python3 src/yolo_rpi_core/test/test_serial_motor_actuator.py
```

**실제 모터로 실행** (`/dev/ttyUSB0`에 하드웨어 연결 시):

```bash
ros2 launch yolo_rpi_core yolo_tracking.launch.py \
    use_real_motor:=true \
    simulation_mode:=false \
    serial_port:=/dev/ttyUSB0
```

---

## 객체 추적 (Object Tracking)

본 프로젝트는 두 가지 추적 알고리즘을 지원합니다 (`tracker_type` 파라미터로 선택):

1. **Centroid Tracker**: 구현이 단순하고 매우 빠릅니다. 가림(occlusion)이 적은 단일 객체 추적에 적합합니다.
2. **ByteTrack**: **Kalman Filter**와 IoU 매칭, 2단계 연관(association) 방식을 사용합니다. 낮은 신뢰도의 탐지 결과도 활용하여 객체가 가려지는 상황을 훨씬 잘 처리합니다. CPU 전용으로 경량화되어 있습니다.

이 명령은 세 개의 노드를 동시에 시작합니다:

1. `v4l2_camera` — USB 카메라에서 영상 프레임 읽기
2. `yolo_node` — 각 프레임에 대해 YOLO 탐지 실행
3. `image_view` — 바운딩 박스가 그려진 실시간 영상 표시

> ✅ 카메라 화면에 탐지된 객체 주위로 색상 바운딩 박스가 그려진 창이 뜨면 — **성공!**

---

## RPi4 배포

### 단계 1: ARM64 이미지 크로스 컴파일 (호스트 PC에서)

RPi4의 ARM 프로세서용으로 같은 Docker 이미지를 빌드합니다. QEMU를 사용하여 x86 PC에서 ARM64를 에뮬레이션합니다.

```bash
./build_arm64.sh
# 생성 파일: yolo_rpi_arm64.tar (~3-4 GB)
# ⏱️ 30~60분 소요 — QEMU 에뮬레이션이라 느린 것은 정상입니다
```

### 단계 2: RPi4로 전송

로컬 네트워크를 통해 이미지 파일을 Raspberry Pi로 복사합니다:

```bash
scp yolo_rpi_arm64.tar pi@<RPI4_IP>:~/
```

> 💡 `<RPI4_IP>`를 RPi4의 IP 주소로 변경하세요 (예: `192.168.0.10`).

### 단계 3: RPi4에서 로드 및 실행

```bash
# RPi4에서:
docker load -i ~/yolo_rpi_arm64.tar    # Docker에 이미지 로드
./run_rpi4.sh                           # 컨테이너 시작 (헤드리스)

# 컨테이너 내부:
cd /ros2_ws
colcon build --packages-select yolo_rpi_core --symlink-install
source install/setup.bash
ros2 launch yolo_rpi_core yolo_headless.launch.py
```

### 단계 4: 호스트 PC에서 원격 시각화

RPi4와 PC 모두 `--network=host`를 사용하므로, ROS 2가 로컬 네트워크에서 자동으로 토픽을 탐색합니다.

```bash
# 호스트 PC에서 (같은 Wi-Fi/LAN에 연결되어 있어야 함):
source /opt/ros/jazzy/setup.bash
ros2 topic list                                  # RPi4의 토픽이 보여야 합니다
ros2 run rqt_image_view rqt_image_view           # /yolo/debug_image 구독
```

> 💡 두 기기 모두 같은 `ROS_DOMAIN_ID`를 사용해야 합니다 (기본값은 `0`이므로 보통 별도 설정 없이 작동합니다).

---

## ROS 2 토픽

| 토픽                    | 타입                               | 설명                     |
| ----------------------- | ---------------------------------- | ------------------------ |
| `/image_raw`            | `sensor_msgs/msg/Image`            | 입력 카메라 이미지       |
| `/yolo/detections`      | `vision_msgs/msg/Detection2DArray` | 원본 YOLO 탐지 결과      |
| `/tracking/command`     | `std_msgs/msg/String` (JSON)       | 제어용 타겟 오차 및 속도 |
| `/tracking/status`      | `std_msgs/msg/String` (JSON)       | 액추에이터 상태 피드백   |
| `/yolo/debug_image`     | `sensor_msgs/msg/Image`            | 시각화된 YOLO 탐지 결과  |
| `/tracking/debug_image` | `sensor_msgs/msg/Image`            | 시각화된 객체 추적 결과  |

## 파라미터

### YOLO 노드

| 파라미터         | 타입   | 기본값       | 설명                  |
| ---------------- | ------ | ------------ | --------------------- |
| `model_path`     | string | `yolo11n.pt` | YOLO 가중치 파일 경로 |
| `device`         | string | `cpu`        | 추론 디바이스         |
| `conf_threshold` | float  | `0.5`        | 신뢰도 임계값         |

### 추적 노드 (Tracker Node)

| 파라미터                | 타입   | 기본값     | 설명                                   |
| ----------------------- | ------ | ---------- | -------------------------------------- |
| `tracker_type`          | string | `centroid` | 알고리즘 (`centroid` 또는 `bytetrack`) |
| `tracking_target_class` | string | `person`   | 추적할 대상 클래스 이름                |
| `max_disappeared`       | int    | `30`       | 대상을 놓쳤을 때 유지할 프레임 수      |
| `max_distance`          | float  | `80.0`     | Centroid 매칭 범위 (픽셀)              |

### 시리얼 모터 액추에이터 노드 (Serial Motor Actuator Node)

| 파라미터          | 타입   | 기본값         | 설명                                                              |
| ----------------- | ------ | -------------- | ----------------------------------------------------------------- |
| `serial_port`     | string | `/dev/ttyUSB0` | USB-시리얼 장치 경로                                              |
| `serial_baud`     | int    | `115200`       | 전송 속도 (모터 컨트롤러 펌웨어와 일치해야 함)                    |
| `simulation_mode` | bool   | `true`         | `true` = 시리얼 없이 로그만 출력; `false` = 실제 하드웨어         |
| `center_angle`    | float  | `90.0`         | 모터 중심 위치 (도)                                               |
| `pan_range_deg`   | float  | `0.0`          | 팬 게인: `error_x` 단위당 이동 각도. `0` = 데모 모드 (항상 90°)   |
| `tilt_range_deg`  | float  | `0.0`          | 틸트 게인: `error_y` 단위당 이동 각도. `0` = 데모 모드 (항상 90°) |
| `min_angle`       | float  | `0.0`          | 하드웨어 최소 각도 한계 (도)                                      |
| `max_angle`       | float  | `180.0`        | 하드웨어 최대 각도 한계 (도)                                      |

> 💡 **데모 모드** (기본값): `pan_range_deg: 0.0` → 사람이 감지되면 두 모터 모두 **90° (중심)**으로 이동합니다.  
> 비례 팬-틸트 추적을 활성화하려면 `pan_range_deg: 45.0`, `tilt_range_deg: 30.0`으로 설정하세요.

---

## 아키텍처 (Architecture)

소프트웨어 설계(OOP 패턴, 상태 머신, JSON 스키마 등)에 대한 자세한 내용은 **[ARCHITECTURE.md](ARCHITECTURE.md)**를 참고하세요.
