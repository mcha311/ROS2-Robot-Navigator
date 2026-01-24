#!/bin/bash

# ROS2 Robot Navigator 자동 테스트 스크립트

echo "======================================"
echo "ROS2 Robot Navigator Test Suite"
echo "======================================"
echo ""

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 작업 디렉토리
cd ~/ros2_ws
source install/setup.bash

# Test 1: 패키지 확인
echo -e "${YELLOW}[Test 1]${NC} Checking package..."
if ros2 pkg list | grep -q "robot_navigator"; then
    echo -e "${GREEN}✓ Package found${NC}"
else
    echo -e "${RED}✗ Package not found${NC}"
    exit 1
fi

# Test 2: Executables 확인
echo -e "${YELLOW}[Test 2]${NC} Checking executables..."
EXECUTABLES=("simulator" "robot_controller" "obstacle_avoidance")
for exec in "${EXECUTABLES[@]}"; do
    if ros2 pkg executables robot_navigator | grep -q "$exec"; then
        echo -e "${GREEN}✓ $exec found${NC}"
    else
        echo -e "${RED}✗ $exec not found${NC}"
        exit 1
    fi
done

# Test 3: Launch 파일 확인
echo -e "${YELLOW}[Test 3]${NC} Checking launch files..."
LAUNCH_DIR="$HOME/ros2_ws/src/robot_navigator/launch"
if [ -d "$LAUNCH_DIR" ]; then
    echo -e "${GREEN}✓ Launch directory exists${NC}"
    ls -1 $LAUNCH_DIR/*.py
else
    echo -e "${RED}✗ Launch directory not found${NC}"
fi

# Test 4: Python 모듈 import 테스트
echo -e "${YELLOW}[Test 4]${NC} Testing Python imports..."
python3 << PYEOF
try:
    from robot_navigator.robot_model import Robot
    from robot_navigator.obstacle import ObstacleManager
    from robot_navigator.laser_sensor import LaserSensor
    print('${GREEN}✓ All imports successful${NC}')
except ImportError as e:
    print('${RED}✗ Import failed:', e, '${NC}')
    exit(1)
PYEOF

# Test 5: ROS2 환경 확인
echo -e "${YELLOW}[Test 5]${NC} Checking ROS2 environment..."
if [ "$ROS_DISTRO" = "jazzy" ]; then
    echo -e "${GREEN}✓ ROS_DISTRO = jazzy${NC}"
else
    echo -e "${RED}✗ ROS_DISTRO = $ROS_DISTRO (expected: jazzy)${NC}"
fi

echo ""
echo "======================================"
echo -e "${GREEN}All tests completed!${NC}"
echo "======================================"
