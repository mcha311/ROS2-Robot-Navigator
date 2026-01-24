#!/usr/bin/env python3
import pygame
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from robot_navigator.robot_model import Robot
from robot_navigator.obstacle import ObstacleManager
from robot_navigator.laser_sensor import LaserSensor

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        
        # Pygame 초기화
        pygame.init()
        self.width = 800
        self.height = 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('ROS2 Robot Navigator - Phase 3')
        self.clock = pygame.time.Clock()
        self.fps = 60
        
        # 색상 정의
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.RED = (255, 0, 0)
        self.BLUE = (0, 0, 255)
        self.GREEN = (0, 255, 0)
        self.GRAY = (128, 128, 128)
        
        # 로봇 생성
        self.robot = Robot(x=self.width//2, y=self.height//2, theta=0)
        
        # 장애물 관리자
        self.obstacle_manager = ObstacleManager(self.width, self.height)
        self.obstacle_manager.add_wall_obstacles()
        self.obstacle_manager.add_random_obstacles(num_circles=5, num_rects=3)
        
        # 레이저 센서
        self.laser_sensor = LaserSensor(num_beams=360, max_range=200.0)
        
        # ROS2 Publisher
        self.pose_pub = self.create_publisher(Pose2D, 'robot/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, 'robot/odom', 10)
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # ROS2 Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # 타이머 (ROS2 메시지 발행용)
        self.create_timer(0.1, self.publish_state)  # 10Hz
        
        # 센서 시각화 옵션
        self.show_laser = True
        
        self.get_logger().info('Robot Simulator Started with Sensors!')
        self.get_logger().info('Press L to toggle laser visualization')
        
    def cmd_vel_callback(self, msg):
        """cmd_vel 토픽 콜백"""
        linear = msg.linear.x * 100  # m/s -> 픽셀/s
        angular = msg.angular.z
        self.robot.set_velocity(linear, angular)
        
    def publish_state(self):
        """로봇 상태 및 센서 데이터 발행"""
        # Pose2D 메시지
        pose_msg = Pose2D()
        pose_msg.x = self.robot.x / 100.0
        pose_msg.y = self.robot.y / 100.0
        pose_msg.theta = self.robot.theta
        self.pose_pub.publish(pose_msg)
        
        # Odometry 메시지
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = pose_msg.x
        odom_msg.pose.pose.position.y = pose_msg.y
        odom_msg.pose.pose.position.z = 0.0
        
        odom_msg.pose.pose.orientation.z = math.sin(pose_msg.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(pose_msg.theta / 2.0)
        
        linear_vel, angular_vel = self.robot.get_velocity()
        odom_msg.twist.twist.linear.x = linear_vel / 100.0
        odom_msg.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(odom_msg)
        
        # 레이저 스캔
        ranges = self.laser_sensor.scan(
            self.robot.x, self.robot.y, self.robot.theta,
            self.obstacle_manager.get_obstacles())
        
        laser_msg = self.laser_sensor.get_laser_scan_msg(
            self.get_clock().now().to_msg())
        self.laser_pub.publish(laser_msg)
        
    def draw_robot(self):
        """로봇 그리기"""
        x, y, theta = self.robot.get_pose()
        
        # 로봇 몸체 (원)
        pygame.draw.circle(self.screen, self.BLUE, 
                          (int(x), int(y)), self.robot.radius)
        
        # 방향 표시 (선)
        end_x = x + self.robot.radius * math.cos(theta)
        end_y = y + self.robot.radius * math.sin(theta)
        pygame.draw.line(self.screen, self.RED, 
                        (int(x), int(y)), (int(end_x), int(end_y)), 3)
        
    def draw_laser_scan(self):
        """레이저 스캔 시각화"""
        if not self.show_laser:
            return
            
        x, y, theta = self.robot.get_pose()
        
        for i, range_val in enumerate(self.laser_sensor.ranges):
            if range_val >= self.laser_sensor.max_range:
                continue
                
            # 빔 각도
            beam_angle = (self.laser_sensor.angle_min + 
                         i * self.laser_sensor.angle_increment)
            global_angle = theta + beam_angle
            
            # 끝점 계산
            end_x = x + range_val * math.cos(global_angle)
            end_y = y + range_val * math.sin(global_angle)
            
            # 그리기
            pygame.draw.line(self.screen, self.RED, 
                           (int(x), int(y)), (int(end_x), int(end_y)), 1)
            pygame.draw.circle(self.screen, self.GREEN, 
                             (int(end_x), int(end_y)), 2)
        
    def draw_info(self):
        """정보 표시"""
        font = pygame.font.Font(None, 24)
        x, y, theta = self.robot.get_pose()
        linear, angular = self.robot.get_velocity()
        
        info_texts = [
            f'Position: ({x/100:.2f}m, {y/100:.2f}m)',
            f'Angle: {math.degrees(theta):.1f}°',
            f'Linear Vel: {linear/100:.2f} m/s',
            f'Angular Vel: {angular:.2f} rad/s',
            '',
            'Controls:',
            'Arrow Keys: Manual Control',
            'L: Toggle Laser Viz',
            'ESC: Quit'
        ]
        
        y_offset = 10
        for text in info_texts:
            surface = font.render(text, True, self.BLACK)
            self.screen.blit(surface, (10, y_offset))
            y_offset += 25
            
    def handle_keyboard(self):
        """키보드 수동 제어"""
        keys = pygame.key.get_pressed()
        
        linear = 0.0
        angular = 0.0
        
        if keys[pygame.K_UP]:
            linear = 50.0
        if keys[pygame.K_DOWN]:
            linear = -50.0
        if keys[pygame.K_LEFT]:
            angular = 1.0
        if keys[pygame.K_RIGHT]:
            angular = -1.0
            
        if linear != 0 or angular != 0:
            self.robot.set_velocity(linear, angular)
        
    def run(self):
        """메인 루프"""
        running = True
        
        while running and rclpy.ok():
            dt = self.clock.tick(self.fps) / 1000.0
            
            # 이벤트 처리
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    if event.key == pygame.K_l:
                        self.show_laser = not self.show_laser
                        self.get_logger().info(f'Laser viz: {self.show_laser}')
                        
            # 키보드 제어
            self.handle_keyboard()
            
            # 로봇 업데이트
            self.robot.update(dt)
            
            # ROS2 콜백 처리
            rclpy.spin_once(self, timeout_sec=0)
            
            # 화면 그리기
            self.screen.fill(self.WHITE)
            self.obstacle_manager.draw_all(self.screen)
            if self.show_laser:
                self.draw_laser_scan()
            self.draw_robot()
            self.draw_info()
            pygame.display.flip()
            
        pygame.quit()
        
def main(args=None):
    rclpy.init(args=args)
    simulator = RobotSimulator()
    
    try:
        simulator.run()
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
