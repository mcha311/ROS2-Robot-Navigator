import pygame
import random

class Obstacle:
    def __init__(self, x, y, radius):
        """
        원형 장애물
        x, y: 중심 좌표
        radius: 반지름
        """
        self.x = x
        self.y = y
        self.radius = radius
        self.color = (100, 100, 100)  # 회색
        
    def draw(self, screen):
        """장애물 그리기"""
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)
        
    def is_collision(self, robot_x, robot_y, robot_radius):
        """로봇과 충돌 감지"""
        distance = ((self.x - robot_x)**2 + (self.y - robot_y)**2)**0.5
        return distance < (self.radius + robot_radius)


class RectangleObstacle:
    def __init__(self, x, y, width, height):
        """
        사각형 장애물
        x, y: 좌상단 좌표
        width, height: 크기
        """
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = (80, 80, 80)
        
    def draw(self, screen):
        """장애물 그리기"""
        pygame.draw.rect(screen, self.color, 
                        (int(self.x), int(self.y), self.width, self.height))
        
    def is_collision(self, robot_x, robot_y, robot_radius):
        """로봇과 충돌 감지 (간단한 AABB)"""
        closest_x = max(self.x, min(robot_x, self.x + self.width))
        closest_y = max(self.y, min(robot_y, self.y + self.height))
        
        distance = ((closest_x - robot_x)**2 + (closest_y - robot_y)**2)**0.5
        return distance < robot_radius


class ObstacleManager:
    def __init__(self, width, height):
        """장애물 관리자"""
        self.obstacles = []
        self.width = width
        self.height = height
        
    def add_random_obstacles(self, num_circles=5, num_rects=3):
        """랜덤 장애물 생성"""
        # 원형 장애물
        for _ in range(num_circles):
            x = random.randint(100, self.width - 100)
            y = random.randint(100, self.height - 100)
            radius = random.randint(20, 40)
            self.obstacles.append(Obstacle(x, y, radius))
            
        # 사각형 장애물
        for _ in range(num_rects):
            x = random.randint(50, self.width - 150)
            y = random.randint(50, self.height - 150)
            width = random.randint(40, 100)
            height = random.randint(40, 100)
            self.obstacles.append(RectangleObstacle(x, y, width, height))
            
    def add_wall_obstacles(self):
        """벽 추가"""
        wall_thickness = 10
        # 상단 벽
        self.obstacles.append(RectangleObstacle(0, 0, self.width, wall_thickness))
        # 하단 벽
        self.obstacles.append(RectangleObstacle(0, self.height - wall_thickness, 
                                               self.width, wall_thickness))
        # 좌측 벽
        self.obstacles.append(RectangleObstacle(0, 0, wall_thickness, self.height))
        # 우측 벽
        self.obstacles.append(RectangleObstacle(self.width - wall_thickness, 0, 
                                               wall_thickness, self.height))
    
    def draw_all(self, screen):
        """모든 장애물 그리기"""
        for obstacle in self.obstacles:
            obstacle.draw(screen)
            
    def check_collision(self, robot_x, robot_y, robot_radius):
        """충돌 체크"""
        for obstacle in self.obstacles:
            if obstacle.is_collision(robot_x, robot_y, robot_radius):
                return True
        return False
    
    def get_obstacles(self):
        """장애물 리스트 반환"""
        return self.obstacles
