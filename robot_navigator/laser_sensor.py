import math
import numpy as np

class LaserSensor:
    def __init__(self, num_beams=360, max_range=200.0, angle_min=-math.pi, 
                 angle_max=math.pi):
        """
        2D 레이저 스캐너 시뮬레이션
        num_beams: 레이저 빔 개수
        max_range: 최대 감지 거리 (픽셀)
        angle_min, angle_max: 스캔 각도 범위
        """
        self.num_beams = num_beams
        self.max_range = max_range
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = (angle_max - angle_min) / num_beams
        
        # 센서 데이터
        self.ranges = [max_range] * num_beams
        self.intensities = [0.0] * num_beams
        
    def scan(self, robot_x, robot_y, robot_theta, obstacles):
        """
        장애물 스캔
        robot_x, robot_y: 로봇 위치
        robot_theta: 로봇 방향
        obstacles: 장애물 리스트
        """
        self.ranges = []
        
        for i in range(self.num_beams):
            # 현재 빔의 각도 (로봇 좌표계)
            beam_angle = self.angle_min + i * self.angle_increment
            # 글로벌 좌표계로 변환
            global_angle = robot_theta + beam_angle
            
            # 빔 방향 벡터
            dx = math.cos(global_angle)
            dy = math.sin(global_angle)
            
            # 레이캐스팅
            min_distance = self.max_range
            
            for obstacle in obstacles:
                distance = self._ray_obstacle_intersection(
                    robot_x, robot_y, dx, dy, obstacle)
                if distance is not None and distance < min_distance:
                    min_distance = distance
                    
            self.ranges.append(min_distance)
            
        return self.ranges
    
    def _ray_obstacle_intersection(self, robot_x, robot_y, dx, dy, obstacle):
        """
        광선과 장애물의 교차점 계산
        """
        if hasattr(obstacle, 'radius'):
            # 원형 장애물
            return self._ray_circle_intersection(
                robot_x, robot_y, dx, dy, 
                obstacle.x, obstacle.y, obstacle.radius)
        else:
            # 사각형 장애물
            return self._ray_rectangle_intersection(
                robot_x, robot_y, dx, dy, obstacle)
    
    def _ray_circle_intersection(self, ox, oy, dx, dy, cx, cy, r):
        """광선-원 교차 계산"""
        # 로봇에서 원 중심으로의 벡터
        fx = cx - ox
        fy = cy - oy
        
        # 투영 길이
        proj = fx * dx + fy * dy
        
        if proj < 0:
            return None
            
        # 가장 가까운 점
        closest_x = ox + proj * dx
        closest_y = oy + proj * dy
        
        # 원 중심까지 거리
        dist_to_center = math.sqrt((closest_x - cx)**2 + (closest_y - cy)**2)
        
        if dist_to_center > r:
            return None
            
        # 교차점까지 거리
        offset = math.sqrt(r**2 - dist_to_center**2)
        distance = proj - offset
        
        if distance < 0 or distance > self.max_range:
            return None
            
        return distance
    
    def _ray_rectangle_intersection(self, ox, oy, dx, dy, rect):
        """광선-사각형 교차 계산 (AABB)"""
        min_x = rect.x
        min_y = rect.y
        max_x = rect.x + rect.width
        max_y = rect.y + rect.height
        
        # 각 면과의 교차 체크
        distances = []
        
        # 좌측 면
        if dx != 0:
            t = (min_x - ox) / dx
            y = oy + t * dy
            if t > 0 and min_y <= y <= max_y and t <= self.max_range:
                distances.append(t)
                
        # 우측 면
        if dx != 0:
            t = (max_x - ox) / dx
            y = oy + t * dy
            if t > 0 and min_y <= y <= max_y and t <= self.max_range:
                distances.append(t)
                
        # 상단 면
        if dy != 0:
            t = (min_y - oy) / dy
            x = ox + t * dx
            if t > 0 and min_x <= x <= max_x and t <= self.max_range:
                distances.append(t)
                
        # 하단 면
        if dy != 0:
            t = (max_y - oy) / dy
            x = ox + t * dx
            if t > 0 and min_x <= x <= max_x and t <= self.max_range:
                distances.append(t)
        
        if distances:
            return min(distances)
        return None
    
    def get_laser_scan_msg(self, current_time):
        """ROS2 LaserScan 메시지 생성"""
        from sensor_msgs.msg import LaserScan
        
        msg = LaserScan()
        msg.header.stamp = current_time
        msg.header.frame_id = 'laser_frame'
        
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.0
        msg.range_max = self.max_range / 100.0  # 픽셀 -> 미터
        
        # 거리를 미터로 변환
        msg.ranges = [r / 100.0 for r in self.ranges]
        msg.intensities = []
        
        return msg
