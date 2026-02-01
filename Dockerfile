# ROS2 Robot Navigator Docker Image
FROM ros:jazzy

# 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# 작업 디렉토리
WORKDIR /root/ros2_ws

# 필수 패키지 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-pygame \
    python3-numpy \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# 작업공간 생성
RUN mkdir -p /root/ros2_ws/src

# 프로젝트 복사
COPY . /root/ros2_ws/src/robot_navigator

# ROS2 패키지 빌드
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /root/ros2_ws && \
    colcon build --packages-select robot_navigator"

# 환경 설정 스크립트
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# 기본 명령어
CMD ["/bin/bash"]
