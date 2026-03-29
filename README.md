# ROS2 Robot Navigator

2D autonomous robot navigation simulator with obstacle avoidance, built on ROS2 Jazzy and Pygame. Serves as the robot simulation backend for [Servi Fleet Manager](https://github.com/mcha311/servi-fleet-manager).

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://python.org)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2024.04%20ARM64-orange)](https://ubuntu.com)

---

## What It Does

Simulates a differential drive robot navigating a 2D environment:

- **360° virtual LiDAR sensor** — ray-casting obstacle detection
- **Artificial Potential Field (APF)** — real-time autonomous obstacle avoidance
- **Differential drive model** — realistic robot kinematics
- **ROS2 topic publishing** — publishes `/odom`, `/battery`, `/scan` compatible with Servi Fleet Manager
- **Manual teleop** — keyboard control override
- **Pygame visualization** — real-time 2D rendering

---

## Why This Stack?

| Technology | Why |
|------------|-----|
| **ROS2 Jazzy** | Industry-standard robot middleware. DDS-based pub/sub enables plug-and-play integration with any ROS2-compatible system (including Servi Fleet Manager's bridge node) |
| **Artificial Potential Field** | Computationally lightweight path planning. Goals generate attractive forces, obstacles generate repulsive forces — the robot follows the gradient in real time |
| **Differential Drive Model** | Matches real-world service robots (Servi, TurtleBot). Two independently controlled wheels; angular velocity achieved by speed differential |
| **Pygame** | Low-latency 2D rendering. Visualizes robot pose, sensor rays, and obstacles in real time without GPU overhead |

---

## How It Works

### Artificial Potential Field Algorithm

```
F_total = F_attractive + F_repulsive

F_attractive = k_att × (goal_position - robot_position)
  → pulls robot toward the goal

F_repulsive = k_rep × (1/distance - 1/d_threshold) × (1/distance²) × direction_away
  → pushes robot away from obstacles within sensor range
  → zero force beyond d_threshold

robot_velocity = F_total (clamped to max speed)
```

When the robot is far from obstacles, only attraction acts and it moves directly toward the goal. As it approaches obstacles, repulsion kicks in and curves the path around them.

### Ray-Casting LiDAR Sensor

```
For each of 360 angles:
  Cast a ray from robot center
  Step along the ray until:
    - Hit an obstacle → record distance
    - Exceed max_range → record max_range

Result: 360 distance measurements (LaserScan message)
```

This data is published as a ROS2 `sensor_msgs/LaserScan` topic, identical in format to a real LiDAR sensor. The Servi Fleet Manager bridge node subscribes to this topic and downsamples to 36 points for web transmission.

### Differential Drive Kinematics

```
linear_vel  = (v_right + v_left) / 2
angular_vel = (v_right - v_left) / wheel_base

new_x   = x + linear_vel × cos(θ) × dt
new_y   = y + linear_vel × sin(θ) × dt
new_θ   = θ + angular_vel × dt
```

The robot turns by making one wheel faster than the other — no steering mechanism required.

---

## ROS2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/robot/robot_01/odom` | `nav_msgs/Odometry` | Publish | Robot pose (x, y, θ) and velocity |
| `/robot/robot_01/scan` | `sensor_msgs/LaserScan` | Publish | 360° LiDAR distances |
| `/robot/robot_01/battery` | `sensor_msgs/BatteryState` | Publish | Battery percentage (drains over time) |
| `/robot/robot_01/goal_pose` | `geometry_msgs/PoseStamped` | Subscribe | Navigation goal from Servi Fleet Manager |
| `/robot/robot_01/cmd_vel` | `geometry_msgs/Twist` | Subscribe | Manual velocity commands |

These topics are compatible with Servi Fleet Manager's bridge node out of the box.

---

## Integration with Servi Fleet Manager

This navigator is the robot-side component of the Servi Fleet Manager system:

```
ros2-robot-navigator (this repo)
    │  publishes ROS2 topics (DDS / CycloneDDS)
    ▼
servi_bridge bridge_node
    │  converts DDS → WebSocket JSON
    ▼
FastAPI backend (servi-fleet-manager)
    │  broadcasts to browsers
    ▼
React dashboard (live map + alerts)
```

To run the full integration:

```bash
# Terminal 1 — Start Servi Fleet Manager backend (on Mac)
cd servi-fleet-manager
docker-compose -f docker-compose.yml -f docker-compose.ros2.yml up

# Terminal 2 — Start the bridge node (on UTM Ubuntu)
cd ~/servi_ws
source install/setup.bash
export FASTAPI_WS_URL=ws://192.168.64.1:8000/ws/ros2
ros2 run servi_bridge bridge_node

# Terminal 3 — Start the navigator (on UTM Ubuntu)
cd ~/ros2_ws
source install/setup.bash
ros2 run robot_navigator navigator_node
```

---

## Quick Start (Standalone)

**Prerequisites:** Ubuntu 24.04, ROS2 Jazzy, Python 3.10+

```bash
# Install dependencies
sudo apt install ros-jazzy-desktop python3-pygame

# Clone and build
git clone https://github.com/mcha311/ROS2-Robot-Navigator
cd ROS2-Robot-Navigator
mkdir -p ~/ros2_ws/src
cp -r . ~/ros2_ws/src/robot_navigator
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Run
ros2 run robot_navigator navigator_node
```

**Keyboard controls:**

| Key | Action |
|-----|--------|
| `W` / `S` | Forward / Backward |
| `A` / `D` | Turn left / right |
| `Space` | Emergency stop |
| `G` | Set random navigation goal (auto mode) |
| `M` | Toggle manual / autonomous mode |

---

## Docker (Alternative)

```bash
docker-compose up
```

Note: Pygame requires display access. Set `DISPLAY` environment variable or use X11 forwarding.

---

## Development Environment

Built on Apple Silicon Mac using UTM virtual machine:
- Host: macOS (Apple Silicon M-series)
- VM: UTM + Ubuntu 24.04 ARM64
- ROS2: Jazzy Jalisco (native ARM64)

No ROS2 Docker image was used — native installation ensures full DDS multicast support required for Servi Fleet Manager integration.

---

## Project Structure

```
ROS2-Robot-Navigator/
├── robot_navigator/
│   ├── navigator_node.py    # Main ROS2 node
│   ├── robot.py             # Differential drive model
│   ├── sensor.py            # Ray-casting LiDAR
│   ├── apf.py               # Artificial Potential Field
│   └── visualizer.py        # Pygame renderer
├── launch/
│   └── navigator.launch.py  # ROS2 launch file
├── scripts/                 # Utility scripts
├── Dockerfile
├── docker-compose.yml
├── package.xml
└── setup.py
```

---

[@mcha311](https://github.com/mcha311)
