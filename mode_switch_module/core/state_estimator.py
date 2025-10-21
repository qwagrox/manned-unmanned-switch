"""
状态估计器模块
负责融合多传感器数据，提供高精度的车辆状态估计
"""

import numpy as np
from typing import Dict, Optional
from .data_structures import VehicleState, DrivingMode


class StateEstimator:
    """状态估计器
    
    使用简化的卡尔曼滤波融合GPS、IMU和轮速计数据
    """
    
    def __init__(self):
        """初始化状态估计器"""
        # 状态向量: [x, y, vx, vy, heading, heading_rate]
        self.state = np.zeros(6)
        
        # 状态协方差矩阵
        self.P = np.eye(6) * 10.0
        
        # 过程噪声协方差
        self.Q = np.diag([0.1, 0.1, 0.5, 0.5, 0.01, 0.01])
        
        # 观测噪声协方差
        self.R_gps = np.diag([0.02, 0.02])  # GPS位置噪声（RTK级别）
        self.R_speed = 0.1  # 速度噪声
        self.R_heading = 0.05  # 航向噪声
        
        self.last_update_time = 0.0
        self.current_mode = DrivingMode.MANUAL
        
    def predict(self, dt: float):
        """预测步骤
        
        Args:
            dt: 时间间隔（秒）
        """
        if dt <= 0:
            return
        
        # 状态转移矩阵
        F = np.eye(6)
        F[0, 2] = dt  # x += vx * dt
        F[1, 3] = dt  # y += vy * dt
        F[4, 5] = dt  # heading += heading_rate * dt
        
        # 预测状态
        self.state = F @ self.state
        
        # 预测协方差
        self.P = F @ self.P @ F.T + self.Q
        
    def update_gps(self, x: float, y: float, quality: float = 1.0):
        """更新GPS观测
        
        Args:
            x: GPS X坐标
            y: GPS Y坐标
            quality: GPS质量 (0-1)
        """
        # 观测矩阵
        H = np.zeros((2, 6))
        H[0, 0] = 1  # 观测x
        H[1, 1] = 1  # 观测y
        
        # 根据质量调整观测噪声
        R = self.R_gps * (2.0 - quality)
        
        # 观测值
        z = np.array([x, y])
        
        # 卡尔曼增益
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 更新状态
        innovation = z - H @ self.state
        self.state = self.state + K @ innovation
        
        # 更新协方差
        self.P = (np.eye(6) - K @ H) @ self.P
        
    def update_imu(self, heading: float, heading_rate: float):
        """更新IMU观测
        
        Args:
            heading: 航向角（弧度）
            heading_rate: 航向角速度（弧度/秒）
        """
        # 更新航向
        H_heading = np.zeros((1, 6))
        H_heading[0, 4] = 1
        
        z_heading = np.array([heading])
        S = H_heading @ self.P @ H_heading.T + self.R_heading
        K = self.P @ H_heading.T / S
        
        innovation = z_heading - H_heading @ self.state
        # 处理角度环绕
        innovation = np.arctan2(np.sin(innovation), np.cos(innovation))
        
        self.state = self.state + K * innovation
        self.P = (np.eye(6) - K @ H_heading) @ self.P
        
        # 直接更新航向角速度（简化处理）
        self.state[5] = heading_rate
        
    def update_speed(self, speed: float):
        """更新速度观测
        
        Args:
            speed: 速度（米/秒）
        """
        # 从速度和航向计算vx, vy
        heading = self.state[4]
        vx = speed * np.cos(heading)
        vy = speed * np.sin(heading)
        
        # 简化更新（直接赋值）
        alpha = 0.7  # 滤波系数
        self.state[2] = alpha * vx + (1 - alpha) * self.state[2]
        self.state[3] = alpha * vy + (1 - alpha) * self.state[3]
        
    def get_vehicle_state(self, timestamp: float, mode: DrivingMode) -> VehicleState:
        """获取当前车辆状态
        
        Args:
            timestamp: 时间戳
            mode: 当前驾驶模式
            
        Returns:
            车辆状态对象
        """
        speed = np.sqrt(self.state[2]**2 + self.state[3]**2)
        
        # 计算定位质量（基于协方差）
        position_uncertainty = np.sqrt(self.P[0, 0] + self.P[1, 1])
        quality = np.clip(1.0 - position_uncertainty / 5.0, 0.0, 1.0)
        
        return VehicleState(
            timestamp=timestamp,
            x=self.state[0],
            y=self.state[1],
            heading=self.state[4],
            speed=speed,
            mode=mode,
            positioning_quality=quality
        )
    
    def reset(self, x: float, y: float, heading: float):
        """重置状态估计器
        
        Args:
            x: 初始X坐标
            y: 初始Y坐标
            heading: 初始航向角
        """
        self.state = np.array([x, y, 0, 0, heading, 0])
        self.P = np.eye(6) * 10.0

