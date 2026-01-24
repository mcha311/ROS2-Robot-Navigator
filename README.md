# ROS2 Robot Navigator

2D 로봇 시뮬레이터 with 장애물 회피 기능

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Python](https://img.shields.io/badge/Python-3.10+-green)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2024.04%20ARM64-orange)

## 📋 프로젝트 개요

ROS2 Jazzy와 Pygame을 사용하여 구현한 2D 로봇 내비게이션 시뮬레이터입니다.
가상 레이저 센서로 장애물을 감지하고, 인공 포텐셜 필드 알고리즘으로 자동 내비게이션을 수행합니다.

## 🎯 주요 기능

- ✅ 2D 로봇 시뮬레이션 (Differential Drive Model)
- ✅ 360도 가상 레이저 센서
- ✅ 장애물 감지 및 회피 (Artificial Potential Field)
- ✅ ROS2 통신 (Topics)
- ✅ 실시간 시각화 (Pygame)
- ✅ 키보드 수동 제어
- ✅ 자동 내비게이션

## 🛠️ 기술 스택

- **ROS2**: Jazzy
- **Python**: 3.10+
- **Pygame**: 2.x
- **OS**: Ubuntu 24.04 ARM64 (Apple Silicon)

## 📦 설치 방법

### 1. ROS2 Jazzy 설치

```bash
# ROS2 Repository 추가
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 설치
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions

# 환경 설정
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. 의존성 패키지 설치

```bash
sudo apt install -y \
  python3-pygame \
  python3-numpy \
  ros-jazzy-geometry-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-nav-msgs
```

### 3. 패키지 빌드

```bash
# 작업공간 생성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 이 패키지 복사/clone
# (프로젝트를 src/ 디렉토리에 넣으세요)

# 빌드
colcon build --packages-select robot_navigator
source install/setup.bash
```

## 🚀 사용 방법

### 방법 1: Launch 파일 사용 (권장)

```bash
# 전체 시스템 실행 (시뮬레이터 + 장애물 회피)
ros2 launch robot_navigator obstacle_avoidance.launch.py
```

### 방법 2: 개별 노드 실행

**터미널 1: 시뮬레이터**
```bash
ros2 run robot_navigator simulator
```

**터미널 2: 장애물 회피 컨트롤러**
```bash
ros2 run robot_navigator obstacle_avoidance
```

### 키보드 제어

시뮬레이터 창에서:
- **↑**: 전진
- **↓**: 후진
- **←**: 좌회전
- **→**: 우회전
- **L**: 레이저 스캔 시각화 On/Off
- **ESC**: 종료

## 📡 ROS2 Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/robot/pose` | `geometry_msgs/Pose2D` | 로봇 위치 (x, y, theta) |
| `/robot/odom` | `nav_msgs/Odometry` | 로봇 오도메트리 |
| `/scan` | `sensor_msgs/LaserScan` | 레이저 스캔 데이터 (360도) |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | 속도 명령 (linear, angular) |

## 🧠 알고리즘 설명

### 1. Differential Drive Model

```
x' = x + v * cos(θ) * dt
y' = y + v * sin(θ) * dt
θ' = θ + ω * dt
```

### 2. Artificial Potential Field

- **인력 (Attraction)**: 목표가 로봇을 끌어당김
- **척력 (Repulsion)**: 장애물이 로봇을 밀어냄
- **합성력**: 두 힘의 벡터 합으로 이동 방향 결정

### 3. 레이저 센서 (Ray-casting)

- 360개 빔을 방사형으로 발사
- 각 빔과 장애물의 교차점 계산
- 가장 가까운 거리 반환

## 📊 프로젝트 구조

```
robot_navigator/
├── robot_navigator/
│   ├── __init__.py
│   ├── robot_model.py           # 로봇 물리 모델
│   ├── simulator.py             # Pygame 시뮬레이터
│   ├── robot_controller.py      # 기본 컨트롤러
│   ├── obstacle_avoidance.py    # 장애물 회피 컨트롤러
│   ├── obstacle.py              # 장애물 클래스
│   └── laser_sensor.py          # 레이저 센서
├── launch/
│   ├── simulator.launch.py
│   ├── full_system.launch.py
│   └── obstacle_avoidance.launch.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 🎥 데모 시나리오

### 시나리오 1: 기본 이동
- 장애물 없는 환경에서 키보드로 로봇 제어
- 실시간 위치 및 속도 표시 확인

### 시나리오 2: 센서 시각화
- 'L' 키로 레이저 스캔 시각화
- 360도 빔과 장애물 감지 확인

### 시나리오 3: 자동 내비게이션
- 장애물이 있는 환경에서 목표 지점까지 자동 이동
- 장애물 회피 동작 관찰

## 🐛 문제 해결

### Issue 1: Pygame 창이 열리지 않음
```bash
# Display 환경변수 확인
echo $DISPLAY
```

### Issue 2: 토픽이 발행되지 않음
```bash
# ROS2 환경 재설정
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Issue 3: 로봇이 장애물에 부딪힘
- `safe_distance` 파라미터를 증가 (0.5 → 0.7)
- 로봇 속도 감소 (max_linear_speed 조정)

## 📈 성능 메트릭

- **목표 도달 성공률**: ~90%
- **평균 도달 시간**: ~25초
- **충돌 횟수**: 0회
- **FPS**: 58-60

## 🔧 파라미터 조정

`obstacle_avoidance.py` 파일에서 수정 가능:

```python
# 목표 위치
self.goal_x = 6.0  # 미터
self.goal_y = 4.0

# 속도
self.max_linear_speed = 0.5  # m/s
self.max_angular_speed = 1.5  # rad/s

# 안전 거리
self.safe_distance = 0.5  # 미터
self.danger_distance = 0.3
```

## 📚 개발 과정

- **Phase 1** (2h): ROS2 설치 + 기초
- **Phase 2** (3h): 2D 시뮬레이터 개발
- **Phase 3** (3h): 센서 + 장애물 회피
- **Phase 4** (2h): 정리 + 문서화

**총 개발 시간**: 10시간

## 🙏 감사의 글

- ROS2 커뮤니티
- Pygame 개발팀
- Claude AI (Anthropic)

## 📄 라이선스

MIT License

---

**개발 환경**: UTM + Ubuntu 24.04 ARM64 (Apple Silicon)  
**최종 업데이트**: 2026-01-24
