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
    │   └── yolo_params.yaml
    ├── launch/
    │   ├── yolo.launch.py          # YOLO 노드만 실행
    │   ├── yolo_vision.launch.py   # 카메라 + YOLO + 뷰어 (전체 파이프라인)
    │   └── yolo_headless.launch.py # 카메라 + YOLO (RPi4 헤드리스)
    └── yolo_rpi_core/
        ├── __init__.py
        └── yolo_node.py
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

### 테스트 2: ROS 2 노드 시작 (중급 — 두 개의 터미널 필요)

**이 테스트가 하는 일:** YOLO 탐지 노드를 ROS 2 서비스로 실행합니다. 실제 로봇에서 이렇게 사용됩니다 — 카메라 이미지를 기다렸다가 탐지 결과를 발행합니다.

**터미널 1** — YOLO 노드 시작:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run yolo_rpi_core yolo_node --ros-args -p model_path:=yolo11n.pt
```

다음과 같이 출력되어야 합니다:

```
[INFO] [yolo_node]: Loading YOLO model: yolo11n.pt
[INFO] [yolo_node]: Device: cpu
[INFO] [yolo_node]: YoloNode initialized and ready!
```

**터미널 2** — **새 터미널 창**을 열고 같은 컨테이너에 접속:

```bash
docker exec -it yolo_dev bash
```

노드가 실행 중인지 확인합니다:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 node list          # 출력: /yolo_node
ros2 topic list         # 포함: /image_raw, /yolo/detections, /yolo/debug_image
```

> ✅ `ros2 node list`에 `/yolo_node`가 보이면 — **성공!** ROS 2 노드가 활성화되어 카메라 이미지를 기다리고 있습니다.

---

### 테스트 3: 전체 비전 파이프라인 (고급 — USB 카메라가 있는 Linux 환경)

**이 테스트가 하는 일:** 전체 파이프라인을 실행합니다 — 카메라가 이미지 캡처 → YOLO가 객체 탐지 → 실시간 화면에 바운딩 박스가 그려진 영상이 표시됩니다.

**필수 조건:** USB 카메라와 X11 디스플레이가 있는 Linux 호스트 (`run_dev.sh` 참고).

```bash
ros2 launch yolo_rpi_core yolo_vision.launch.py
```

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

| 토픽                | 타입                               | 설명                        |
| ------------------- | ---------------------------------- | --------------------------- |
| `/image_raw`        | `sensor_msgs/msg/Image`            | 입력 카메라 이미지          |
| `/yolo/detections`  | `vision_msgs/msg/Detection2DArray` | 탐지 결과                   |
| `/yolo/debug_image` | `sensor_msgs/msg/Image`            | 바운딩 박스가 그려진 이미지 |

## 파라미터

| 파라미터         | 타입   | 기본값       | 설명                  |
| ---------------- | ------ | ------------ | --------------------- |
| `model_path`     | string | `yolo11n.pt` | YOLO 가중치 파일 경로 |
| `device`         | string | `cpu`        | 추론 디바이스         |
| `conf_threshold` | float  | `0.5`        | 신뢰도 임계값         |
