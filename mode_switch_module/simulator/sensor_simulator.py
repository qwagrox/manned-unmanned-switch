"""
传感器模拟器
模拟GPS、IMU、轮速计等传感器数据
"""

import numpy as np
from typing import Tuple


class SensorSimulator:
    """传感器模拟器
    
    模拟真实传感器的输出，包括噪声和延迟
    """
    
    def __init__(self, gps_noise_std: float = 0.02, imu_noise_std: float = 0.01,
                 speed_noise_std: float = 0.1):
        """初始化传感器模拟器
        
        Args:
            gps_noise_std: GPS噪声标准差（米）
            imu_noise_std: IMU噪声标准差（弧度）
            speed_noise_std: 速度噪声标准差（米/秒）
        """
        self.gps_noise_std = gps_noise_std
        self.imu_noise_std = imu_noise_std
        self.speed_noise_std = speed_noise_std
        
        # RTK质量模拟（偶尔会降低）
        self.gps_quality = 1.0
        self.quality_degradation_prob = 0.01  # 1%概率质量下降
        
    def simulate_gps(self, true_x: float, true_y: float) -> Tuple[float, float, float]:
        """模拟GPS测量
        
        Args:
            true_x: 真实X坐标
            true_y: 真实Y坐标
            
        Returns:
            (测量X, 测量Y, GPS质量)
        """
        # 添加高斯噪声
        noise_x = np.random.normal(0, self.gps_noise_std)
        noise_y = np.random.normal(0, self.gps_noise_std)
        
        measured_x = true_x + noise_x
        measured_y = true_y + noise_y
        
        # 模拟质量变化
        if np.random.random() < self.quality_degradation_prob:
            self.gps_quality = max(0.5, self.gps_quality - 0.1)
        else:
            self.gps_quality = min(1.0, self.gps_quality + 0.05)
        
        return measured_x, measured_y, self.gps_quality
    
    def simulate_imu(self, true_heading: float, true_heading_rate: float) -> Tuple[float, float]:
        """模拟IMU测量
        
        Args:
            true_heading: 真实航向角（弧度）
            true_heading_rate: 真实航向角速度（弧度/秒）
            
        Returns:
            (测量航向角, 测量航向角速度)
        """
        # 添加高斯噪声
        noise_heading = np.random.normal(0, self.imu_noise_std)
        noise_rate = np.random.normal(0, self.imu_noise_std * 0.5)
        
        measured_heading = true_heading + noise_heading
        measured_rate = true_heading_rate + noise_rate
        
        # 归一化航向角到[-pi, pi]
        measured_heading = np.arctan2(np.sin(measured_heading), np.cos(measured_heading))
        
        return measured_heading, measured_rate
    
    def simulate_wheel_speed(self, true_speed: float) -> float:
        """模拟轮速计测量
        
        Args:
            true_speed: 真实速度（米/秒）
            
        Returns:
            测量速度
        """
        # 添加高斯噪声
        noise = np.random.normal(0, self.speed_noise_std)
        measured_speed = max(0.0, true_speed + noise)  # 速度不能为负
        
        return measured_speed
    
    def reset(self):
        """重置传感器状态"""
        self.gps_quality = 1.0

