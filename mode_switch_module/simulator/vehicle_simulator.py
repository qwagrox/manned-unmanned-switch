"""
车辆模拟器
模拟拖拉机的运动学和动力学
"""

import numpy as np
from typing import Tuple


class VehicleSimulator:
    """车辆模拟器
    
    使用简化的自行车模型模拟拖拉机运动
    """
    
    def __init__(self, wheelbase: float = 2.5, max_steering_angle: float = 0.5,
                 max_speed: float = 3.0, max_acceleration: float = 1.0):
        """初始化车辆模拟器
        
        Args:
            wheelbase: 轴距（米）
            max_steering_angle: 最大转向角（弧度）
            max_speed: 最大速度（米/秒）
            max_acceleration: 最大加速度（米/秒²）
        """
        self.wheelbase = wheelbase
        self.max_steering_angle = max_steering_angle
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        
        # 车辆状态
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0
        
    def reset(self, x: float, y: float, heading: float):
        """重置车辆状态
        
        Args:
            x: 初始X坐标
            y: 初始Y坐标
            heading: 初始航向角（弧度）
        """
        self.x = x
        self.y = y
        self.heading = heading
        self.speed = 0.0
        self.steering_angle = 0.0
        
    def update(self, target_speed: float, target_steering: float, dt: float):
        """更新车辆状态
        
        Args:
            target_speed: 目标速度（米/秒）
            target_steering: 目标转向角（弧度）
            dt: 时间步长（秒）
        """
        # 限制目标值
        target_speed = np.clip(target_speed, 0.0, self.max_speed)
        target_steering = np.clip(target_steering, -self.max_steering_angle, self.max_steering_angle)
        
        # 速度控制（一阶延迟）
        speed_error = target_speed - self.speed
        acceleration = np.clip(speed_error * 2.0, -self.max_acceleration, self.max_acceleration)
        self.speed += acceleration * dt
        self.speed = np.clip(self.speed, 0.0, self.max_speed)
        
        # 转向控制（一阶延迟）
        steering_error = target_steering - self.steering_angle
        steering_rate = np.clip(steering_error * 3.0, -1.0, 1.0)  # 最大转向速度1 rad/s
        self.steering_angle += steering_rate * dt
        self.steering_angle = np.clip(self.steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        # 自行车模型运动学
        if abs(self.steering_angle) < 0.001:
            # 直线运动
            self.x += self.speed * np.cos(self.heading) * dt
            self.y += self.speed * np.sin(self.heading) * dt
        else:
            # 圆弧运动
            turning_radius = self.wheelbase / np.tan(self.steering_angle)
            angular_velocity = self.speed / turning_radius
            
            # 更新航向
            self.heading += angular_velocity * dt
            self.heading = np.arctan2(np.sin(self.heading), np.cos(self.heading))
            
            # 更新位置（圆弧运动）
            dx = self.speed * np.cos(self.heading) * dt
            dy = self.speed * np.sin(self.heading) * dt
            self.x += dx
            self.y += dy
    
    def get_state(self) -> Tuple[float, float, float, float, float]:
        """获取车辆状态
        
        Returns:
            (x, y, heading, speed, heading_rate)
        """
        # 计算航向角速度
        if abs(self.steering_angle) < 0.001:
            heading_rate = 0.0
        else:
            turning_radius = self.wheelbase / np.tan(self.steering_angle)
            heading_rate = self.speed / turning_radius
        
        return self.x, self.y, self.heading, self.speed, heading_rate
    
    def apply_manual_control(self, speed_delta: float, steering_delta: float):
        """应用手动控制（模拟驾驶员操作）
        
        Args:
            speed_delta: 速度变化量（米/秒）
            steering_delta: 转向角变化量（弧度）
        """
        self.speed = np.clip(self.speed + speed_delta, 0.0, self.max_speed)
        self.steering_angle = np.clip(
            self.steering_angle + steering_delta,
            -self.max_steering_angle,
            self.max_steering_angle
        )

