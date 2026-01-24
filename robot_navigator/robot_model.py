import math
import numpy as np

class Robot:
    def __init__(self, x=400, y=300, theta=0.0):
        """
        2D 로봇 모델
        x, y: 위치 (픽셀)
        theta: 방향 (라디안)
        """
        self.x = x
        self.y = y
        self.theta = theta
        
        # 로봇 물리 파라미터
        self.radius = 15  # 로봇 반지름 (픽셀)
        self.max_linear_velocity = 100.0  # 픽셀/초
        self.max_angular_velocity = 2.0  # 라디안/초
        
        # 현재 속도
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # 센서 데이터 (나중에 사용)
        self.sensor_readings = []
        
    def update(self, dt):
        """
        로봇 위치 업데이트 (Differential Drive 모델)
        dt: 시간 간격 (초)
        """
        # 각속도 적용
        self.theta += self.angular_velocity * dt
        
        # 각도 정규화 (-pi ~ pi)
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # 선속도 적용
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt
        
    def set_velocity(self, linear, angular):
        """
        속도 명령 설정
        linear: 선속도 (-max ~ max)
        angular: 각속도 (-max ~ max)
        """
        self.linear_velocity = max(-self.max_linear_velocity, 
                                   min(self.max_linear_velocity, linear))
        self.angular_velocity = max(-self.max_angular_velocity, 
                                    min(self.max_angular_velocity, angular))
        
    def get_pose(self):
        """현재 위치 및 방향 반환"""
        return self.x, self.y, self.theta
    
    def get_velocity(self):
        """현재 속도 반환"""
        return self.linear_velocity, self.angular_velocity
