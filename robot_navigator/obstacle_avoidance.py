#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class ObstacleAvoidanceController(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_controller')
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber
        self.pose_sub = self.create_subscription(
            Pose2D, 'robot/pose', self.pose_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        
        # 현재 상태
        self.current_pose = None
        self.laser_data = None
        
        # 목표 위치
        self.goal_x = 6.0  # 미터
        self.goal_y = 4.0
        self.goal_tolerance = 0.15
        
        # 제어 파라미터
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.5  # rad/s
        
        # 장애물 회피 파라미터
        self.safe_distance = 0.5  # 안전 거리 (미터)
        self.danger_distance = 0.3  # 위험 거리
        
        # 상태
        self.state = 'MOVE_TO_GOAL'
        
        # 제어 타이머
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Obstacle Avoidance Controller Started!')
        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y})')
        
    def pose_callback(self, msg):
        """위치 정보 콜백"""
        self.current_pose = msg
        
    def laser_callback(self, msg):
        """레이저 스캔 콜백"""
        self.laser_data = msg
        
    def get_obstacle_vectors(self):
        """장애물로부터의 회피 벡터 계산"""
        if self.laser_data is None:
            return 0.0, 0.0
            
        repulsion_x = 0.0
        repulsion_y = 0.0
        
        num_ranges = len(self.laser_data.ranges)
        
        for i, distance in enumerate(self.laser_data.ranges):
            if distance < self.safe_distance and distance > 0.01:
                # 장애물이 안전 거리 내에 있음
                angle = (self.laser_data.angle_min + 
                        i * self.laser_data.angle_increment)
                
                # 위험도 계산 (가까울수록 큰 힘)
                if distance < self.danger_distance:
                    force = 1.0
                else:
                    force = (self.safe_distance - distance) / \
                           (self.safe_distance - self.danger_distance)
                
                # 반발력 벡터 (장애물 반대 방향)
                repulsion_x -= force * math.cos(angle)
                repulsion_y -= force * math.sin(angle)
        
        return repulsion_x, repulsion_y
    
    def control_loop(self):
        """메인 제어 루프 - 인공 포텐셜 필드 방식"""
        if self.current_pose is None or self.laser_data is None:
            return
            
        # 목표까지의 벡터
        dx_goal = self.goal_x - self.current_pose.x
        dy_goal = self.goal_y - self.current_pose.y
        distance_to_goal = math.sqrt(dx_goal**2 + dy_goal**2)
        
        cmd = Twist()
        
        if distance_to_goal < self.goal_tolerance:
            # 목표 도달
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.state = 'GOAL_REACHED'
            self.get_logger().info('Goal Reached!', throttle_duration_sec=1.0)
        else:
            # 목표로의 인력 벡터 (정규화)
            attraction_x = dx_goal / distance_to_goal
            attraction_y = dy_goal / distance_to_goal
            
            # 장애물로부터의 척력 벡터
            repulsion_x, repulsion_y = self.get_obstacle_vectors()
            
            # 합성 벡터 (인공 포텐셜 필드)
            total_x = attraction_x + repulsion_x
            total_y = attraction_y + repulsion_y
            
            # 목표 방향
            target_angle = math.atan2(total_y, total_x)
            
            # 현재 방향과의 차이
            angle_diff = target_angle - self.current_pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # 속도 명령 생성
            if abs(angle_diff) > 0.3:
                # 큰 각도 차이 -> 회전 우선
                cmd.linear.x = 0.1
                cmd.angular.z = max(-self.max_angular_speed, 
                                   min(self.max_angular_speed, 2.0 * angle_diff))
            else:
                # 전진 + 미세 조정
                cmd.linear.x = min(self.max_linear_speed, distance_to_goal * 0.5)
                cmd.angular.z = angle_diff
                
            # 전방 장애물 긴급 감지
            front_ranges = self.laser_data.ranges[
                len(self.laser_data.ranges)//2 - 30:
                len(self.laser_data.ranges)//2 + 30]
            
            min_front_distance = min([r for r in front_ranges if r > 0.01])
            
            if min_front_distance < self.danger_distance:
                # 긴급 정지!
                cmd.linear.x = 0.0
                cmd.angular.z = self.max_angular_speed
                self.state = 'AVOID_OBSTACLE'
            else:
                self.state = 'MOVE_TO_GOAL'
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = ObstacleAvoidanceController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
