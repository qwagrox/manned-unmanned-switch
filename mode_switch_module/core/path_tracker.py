"""
路径跟踪器模块
负责路径匹配、偏离检测和控制指令生成
"""

import numpy as np
from typing import Optional, Tuple
from .data_structures import (
    Path, PathPoint, VehicleState, DeviationInfo, 
    DeviationLevel, DrivingMode
)


class PathTracker:
    """路径跟踪器
    
    实现路径匹配、偏离检测和Pure Pursuit控制
    """
    
    def __init__(self, work_width: float = 3.2, lookahead_distance: float = 5.0):
        """初始化路径跟踪器
        
        Args:
            work_width: 作业宽度（米）
            lookahead_distance: 前视距离（米），用于Pure Pursuit控制
        """
        self.work_width = work_width
        self.lookahead_distance = lookahead_distance
        self.current_path: Optional[Path] = None
        self.current_path_index = 0
        self.total_distance_traveled = 0.0
        
    def set_path(self, path: Path):
        """设置当前跟踪路径
        
        Args:
            path: 路径对象
        """
        self.current_path = path
        self.current_path_index = 0
        print(f"路径跟踪器: 设置新路径, 包含 {len(path.points)} 个点")
        
    def update(self, vehicle_state: VehicleState) -> Optional[DeviationInfo]:
        """更新路径跟踪状态
        
        Args:
            vehicle_state: 当前车辆状态
            
        Returns:
            偏离信息，如果没有路径则返回None
        """
        if self.current_path is None or len(self.current_path.points) == 0:
            return None
        
        # 查找最近的路径点
        closest_index = self._find_closest_point(vehicle_state.x, vehicle_state.y)
        self.current_path_index = closest_index
        
        # 计算偏离信息
        deviation_info = self._calculate_deviation(vehicle_state, closest_index)
        
        return deviation_info
    
    def _find_closest_point(self, x: float, y: float) -> int:
        """查找最近的路径点
        
        Args:
            x: X坐标
            y: Y坐标
            
        Returns:
            最近路径点的索引
        """
        if self.current_path is None:
            return 0
        
        # 从当前索引附近开始搜索（利用时间连续性）
        search_start = max(0, self.current_path_index - 10)
        search_end = min(len(self.current_path.points), self.current_path_index + 50)
        
        min_dist = float('inf')
        min_index = self.current_path_index
        
        for i in range(search_start, search_end):
            point = self.current_path.points[i]
            dist = point.distance_to(x, y)
            if dist < min_dist:
                min_dist = dist
                min_index = i
        
        return min_index
    
    def _calculate_deviation(self, vehicle_state: VehicleState, path_index: int) -> DeviationInfo:
        """计算偏离信息
        
        Args:
            vehicle_state: 车辆状态
            path_index: 路径点索引
            
        Returns:
            偏离信息
        """
        if self.current_path is None or path_index >= len(self.current_path.points):
            return DeviationInfo(
                lateral_deviation=0.0,
                heading_deviation=0.0,
                speed_deviation=0.0,
                deviation_level=DeviationLevel.MINIMAL,
                closest_path_index=0,
                distance_along_path=0.0
            )
        
        path_point = self.current_path.points[path_index]
        
        # 计算横向偏差（垂直于路径方向的距离）
        dx = vehicle_state.x - path_point.x
        dy = vehicle_state.y - path_point.y
        
        # 路径方向向量
        path_heading = path_point.heading
        path_dir_x = np.cos(path_heading)
        path_dir_y = np.sin(path_heading)
        
        # 横向偏差（叉积）
        lateral_deviation = dx * (-path_dir_y) + dy * path_dir_x
        
        # 航向偏差
        heading_deviation = vehicle_state.heading - path_point.heading
        # 归一化到[-pi, pi]
        heading_deviation = np.arctan2(np.sin(heading_deviation), np.cos(heading_deviation))
        
        # 速度偏差
        speed_deviation = vehicle_state.speed - path_point.speed
        
        # 判断偏离等级
        deviation_ratio = abs(lateral_deviation) / self.work_width
        if deviation_ratio < 0.5:
            level = DeviationLevel.MINIMAL
        elif deviation_ratio < 1.5:
            level = DeviationLevel.SLIGHT
        elif deviation_ratio < 3.0:
            level = DeviationLevel.MODERATE
        else:
            level = DeviationLevel.SEVERE
        
        # 计算沿路径的距离
        distance_along_path = sum(
            self.current_path.points[i].distance_to(
                self.current_path.points[i+1].x,
                self.current_path.points[i+1].y
            )
            for i in range(path_index)
        )
        
        return DeviationInfo(
            lateral_deviation=lateral_deviation,
            heading_deviation=heading_deviation,
            speed_deviation=speed_deviation,
            deviation_level=level,
            closest_path_index=path_index,
            distance_along_path=distance_along_path
        )
    
    def compute_control(self, vehicle_state: VehicleState) -> Tuple[float, float]:
        """计算控制指令（Pure Pursuit算法）
        
        Args:
            vehicle_state: 车辆状态
            
        Returns:
            (目标航向角, 目标速度)
        """
        if self.current_path is None or len(self.current_path.points) == 0:
            return vehicle_state.heading, 0.0
        
        # 查找前视点
        lookahead_point = self._find_lookahead_point(vehicle_state)
        if lookahead_point is None:
            # 如果没有找到前视点，说明接近路径终点
            if self.current_path_index < len(self.current_path.points):
                lookahead_point = self.current_path.points[-1]
            else:
                return vehicle_state.heading, 0.0
        
        # 计算目标航向
        dx = lookahead_point.x - vehicle_state.x
        dy = lookahead_point.y - vehicle_state.y
        target_heading = np.arctan2(dy, dx)
        
        # 目标速度
        target_speed = lookahead_point.speed
        
        return target_heading, target_speed
    
    def _find_lookahead_point(self, vehicle_state: VehicleState) -> Optional[PathPoint]:
        """查找前视点
        
        Args:
            vehicle_state: 车辆状态
            
        Returns:
            前视点，如果没有则返回None
        """
        if self.current_path is None:
            return None
        
        # 从当前索引开始向前搜索
        for i in range(self.current_path_index, len(self.current_path.points)):
            point = self.current_path.points[i]
            dist = point.distance_to(vehicle_state.x, vehicle_state.y)
            
            if dist >= self.lookahead_distance:
                return point
        
        # 如果没有找到，返回最后一个点
        if len(self.current_path.points) > 0:
            return self.current_path.points[-1]
        
        return None
    
    def get_progress(self) -> float:
        """获取路径执行进度
        
        Returns:
            进度百分比 (0-1)
        """
        if self.current_path is None or len(self.current_path.points) == 0:
            return 0.0
        
        return self.current_path_index / len(self.current_path.points)
    
    def is_path_completed(self, vehicle_state: VehicleState, threshold: float = 2.0) -> bool:
        """判断路径是否完成
        
        Args:
            vehicle_state: 车辆状态
            threshold: 距离阈值（米）
            
        Returns:
            是否完成
        """
        if self.current_path is None or len(self.current_path.points) == 0:
            return True
        
        # 检查是否接近最后一个点
        last_point = self.current_path.points[-1]
        dist = last_point.distance_to(vehicle_state.x, vehicle_state.y)
        
        return dist < threshold and self.current_path_index >= len(self.current_path.points) - 5

