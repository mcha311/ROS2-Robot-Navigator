#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber
        self.pose_sub = self.create_subscription(
            Pose2D, 'robot/pose', self.pose_callback, 10)
        
        # 현재 위치
        self.current_pose = None
        
        # 목표 위치
        self.goal_x = 5.0  # 미터
        self.goal_y = 3.0
        
        # 제어 파라미터
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.goal_tolerance = 0.1  # 미터
        
        # 제어 타이머
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Robot Controller Started!')
        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y})')
        
    def pose_callback(self, msg):
        """위치 정보 콜백"""
        self.current_pose = msg
        
    def control_loop(self):
        """간단한 제어 루프"""
        if self.current_pose is None:
            return
            
        # 목표까지의 거리와 각도 계산
        dx = self.goal_x - self.current_pose.x
        dy = self.goal_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # 각도 차이
        angle_diff = target_angle - self.current_pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # 속도 명령 생성
        cmd = Twist()
        
        if distance > self.goal_tolerance:
            # 목표에 도달하지 않음
            if abs(angle_diff) > 0.1:
                # 먼저 방향 전환
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            else:
                # 전진
                cmd.linear.x = min(self.linear_speed, distance)
                cmd.angular.z = angle_diff
        else:
            # 목표 도달!
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Goal Reached!', once=True)
            
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
