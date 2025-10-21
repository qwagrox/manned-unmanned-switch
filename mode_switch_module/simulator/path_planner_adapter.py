"""
路径规划器适配器
与multi_layer_planner_v3集成
"""

import numpy as np
from typing import List, Tuple

try:
    from multi_layer_planner_v3 import MultiLayerPlanner
    PLANNER_AVAILABLE = True
except ImportError:
    print("警告: 无法导入multi_layer_planner_v3，将使用简化的路径生成")
    PLANNER_AVAILABLE = False

from ..core.data_structures import Path, PathPoint


class PathPlannerAdapter:
    """路径规划器适配器
    
    封装multi_layer_planner_v3，提供统一接口
    """
    
    def __init__(self, work_width: float = 3.2):
        """初始化路径规划器适配器
        
        Args:
            work_width: 作业宽度（米）
        """
        self.work_width = work_width
        self.planner_available = PLANNER_AVAILABLE
        
    def generate_coverage_path(self, field_bounds: Tuple[float, float, float, float],
                               start_x: float = None, start_y: float = None) -> Path:
        """生成覆盖路径
        
        Args:
            field_bounds: 田地边界 (min_x, min_y, max_x, max_y)
            start_x: 起始X坐标（可选）
            start_y: 起始Y坐标（可选）
            
        Returns:
            路径对象
        """
        if self.planner_available:
            return self._generate_with_planner(field_bounds, start_x, start_y)
        else:
            return self._generate_simple_path(field_bounds, start_x, start_y)
    
    def _generate_with_planner(self, field_bounds: Tuple[float, float, float, float],
                               start_x: float = None, start_y: float = None) -> Path:
        """使用multi_layer_planner_v3生成路径
        
        Args:
            field_bounds: 田地边界
            start_x: 起始X坐标
            start_y: 起始Y坐标
            
        Returns:
            路径对象
        """
        # 这里应该调用实际的multi_layer_planner_v3
        # 由于该规划器需要特定的输入格式，这里提供简化实现
        return self._generate_simple_path(field_bounds, start_x, start_y)
    
    def _generate_simple_path(self, field_bounds: Tuple[float, float, float, float],
                             start_x: float = None, start_y: float = None) -> Path:
        """生成简化的往复式覆盖路径
        
        Args:
            field_bounds: 田地边界 (min_x, min_y, max_x, max_y)
            start_x: 起始X坐标
            start_y: 起始Y坐标
            
        Returns:
            路径对象
        """
        min_x, min_y, max_x, max_y = field_bounds
        
        # 如果没有指定起点，使用左下角
        if start_x is None:
            start_x = min_x + self.work_width / 2
        if start_y is None:
            start_y = min_y + self.work_width / 2
        
        points = []
        
        # 生成往复式路径
        y = start_y
        direction = 1  # 1: 向右, -1: 向左
        path_id = 0
        
        while y < max_y - self.work_width / 2:
            if direction == 1:
                # 向右行驶
                x_start = min_x + self.work_width / 2
                x_end = max_x - self.work_width / 2
                heading = 0.0
            else:
                # 向左行驶
                x_start = max_x - self.work_width / 2
                x_end = min_x + self.work_width / 2
                heading = np.pi
            
            # 生成直线段
            num_points = int(abs(x_end - x_start) / 0.5) + 1
            for i in range(num_points):
                t = i / (num_points - 1) if num_points > 1 else 0
                x = x_start + t * (x_end - x_start)
                
                points.append(PathPoint(
                    x=x,
                    y=y,
                    heading=heading,
                    speed=2.0,  # 作业速度 2 m/s
                    segment_type="straight"
                ))
            
            # 转弯
            y += self.work_width
            if y < max_y - self.work_width / 2:
                # 添加转弯点
                num_turn_points = 10
                for i in range(1, num_turn_points + 1):
                    t = i / num_turn_points
                    turn_y = y - self.work_width * (1 - t)
                    
                    # 转弯时保持在边界
                    turn_x = x_end
                    turn_heading = heading + direction * np.pi * t
                    
                    points.append(PathPoint(
                        x=turn_x,
                        y=turn_y,
                        heading=turn_heading,
                        speed=1.0,  # 转弯速度降低
                        curvature=1.0 / (self.work_width / 2),
                        segment_type="turn"
                    ))
                
                direction *= -1  # 改变方向
        
        # 创建路径对象
        path = Path(
            points=points,
            path_id="coverage_path_001",
            path_type="coverage"
        )
        
        return path
    
    def generate_alignment_path(self, current_x: float, current_y: float,
                               current_heading: float, target_x: float,
                               target_y: float, target_heading: float) -> Path:
        """生成对齐路径（从当前位置平滑过渡到目标路径）
        
        Args:
            current_x: 当前X坐标
            current_y: 当前Y坐标
            current_heading: 当前航向角
            target_x: 目标X坐标
            target_y: 目标Y坐标
            target_heading: 目标航向角
            
        Returns:
            对齐路径
        """
        points = []
        
        # 使用三次贝塞尔曲线生成平滑路径
        num_points = 20
        
        # 控制点
        control_length = 5.0  # 控制点距离
        p0 = np.array([current_x, current_y])
        p1 = p0 + control_length * np.array([np.cos(current_heading), np.sin(current_heading)])
        p3 = np.array([target_x, target_y])
        p2 = p3 - control_length * np.array([np.cos(target_heading), np.sin(target_heading)])
        
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # 三次贝塞尔曲线
            point = (1-t)**3 * p0 + 3*(1-t)**2*t * p1 + 3*(1-t)*t**2 * p2 + t**3 * p3
            
            # 计算切线方向（航向）
            tangent = 3*(1-t)**2 * (p1-p0) + 6*(1-t)*t * (p2-p1) + 3*t**2 * (p3-p2)
            heading = np.arctan2(tangent[1], tangent[0])
            
            # 速度从当前速度过渡到目标速度
            speed = 1.5  # 对齐时使用中等速度
            
            points.append(PathPoint(
                x=point[0],
                y=point[1],
                heading=heading,
                speed=speed,
                segment_type="alignment"
            ))
        
        path = Path(
            points=points,
            path_id="alignment_path",
            path_type="alignment"
        )
        
        return path
    
    def generate_approach_path(self, current_x: float, current_y: float,
                              target_x: float, target_y: float,
                              target_heading: float) -> Path:
        """生成接近路径（从当前位置到作业起点）
        
        Args:
            current_x: 当前X坐标
            current_y: 当前Y坐标
            target_x: 目标X坐标
            target_y: 目标Y坐标
            target_heading: 目标航向角
            
        Returns:
            接近路径
        """
        points = []
        
        # 简单的直线接近
        distance = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        num_points = max(int(distance / 0.5), 2)
        
        for i in range(num_points):
            t = i / (num_points - 1)
            x = current_x + t * (target_x - current_x)
            y = current_y + t * (target_y - current_y)
            
            # 航向逐渐过渡到目标航向
            heading = np.arctan2(target_y - current_y, target_x - current_x)
            if i == num_points - 1:
                heading = target_heading
            
            points.append(PathPoint(
                x=x,
                y=y,
                heading=heading,
                speed=1.5,
                segment_type="approach"
            ))
        
        path = Path(
            points=points,
            path_id="approach_path",
            path_type="approach"
        )
        
        return path

