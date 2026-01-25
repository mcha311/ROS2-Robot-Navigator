# ROS2 Robot Navigator

2D 로봇 시뮬레이터 장애물 회피 기능

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


**개발 환경**: UTM + Ubuntu 24.04 ARM64 (Apple Silicon) 
