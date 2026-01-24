# Quick Start Guide

## ⚡ 5분 만에 시작하기

### 1단계: 빌드 (최초 1회)

```bash
cd ~/ros2_ws
colcon build --packages-select robot_navigator
source install/setup.bash
```

### 2단계: 실행!

```bash
ros2 launch robot_navigator obstacle_avoidance.launch.py
```

끝! 🎉

---

## 🎮 조작법

**시뮬레이터 창에서**:
- `↑` : 전진
- `↓` : 후진
- `←` : 좌회전
- `→` : 우회전
- `L` : 레이저 표시 On/Off
- `ESC` : 종료

---

## 🔍 확인 사항

### ✅ 정상 작동 시:
- Pygame 창이 열림
- 파란 로봇 + 회색 장애물들
- 빨간 레이저 빔 (L키로 토글)
- 로봇이 자동으로 목표로 이동

### ❌ 문제 발생 시:

**문제 1**: "Package not found"
```bash
cd ~/ros2_ws
colcon build --packages-select robot_navigator
source install/setup.bash
```

**문제 2**: Pygame 창 안 열림
```bash
echo $DISPLAY  # 출력 있어야 함
```

**문제 3**: 토픽 안 보임
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 📝 다른 실행 방법

### 방법 1: 시뮬레이터만 실행
```bash
ros2 run robot_navigator simulator
# 화살표 키로 수동 제어
```

### 방법 2: 개별 실행
```bash
# 터미널 1
ros2 run robot_navigator simulator

# 터미널 2
ros2 run robot_navigator obstacle_avoidance
```

---

**더 자세한 내용은 README.md 참고!**
