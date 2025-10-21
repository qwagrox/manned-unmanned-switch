"""
数据结构定义模块
定义了模式切换系统中使用的所有核心数据结构
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Tuple, Optional
import numpy as np
from datetime import datetime


class DrivingMode(Enum):
    """驾驶模式枚举"""
    MANUAL = "manual"  # 有人驾驶
    AUTO = "auto"  # 无人驾驶
    TRANSITION = "transition"  # 过渡状态
    PAUSED = "paused"  # 暂停
    ERROR = "error"  # 错误状态


class CoverageStatus(Enum):
    """覆盖状态枚举"""
    UNCOVERED = 0  # 未覆盖
    MANUAL_COVERED = 1  # 有人模式覆盖
    AUTO_COVERED = 2  # 无人模式覆盖
    BOTH_COVERED = 3  # 两种模式都覆盖过（重复覆盖）


class DeviationLevel(Enum):
    """偏离等级枚举"""
    MINIMAL = "minimal"  # 极小偏离 (<0.5个作业宽度)
    SLIGHT = "slight"  # 轻微偏离 (0.5-1.5个作业宽度)
    MODERATE = "moderate"  # 中度偏离 (1.5-3.0个作业宽度)
    SEVERE = "severe"  # 严重偏离 (>3.0个作业宽度)


class RecoveryStrategy(Enum):
    """恢复策略枚举"""
    DIRECT_RESUME = "direct_resume"  # 直接恢复
    SMOOTH_ALIGNMENT = "smooth_alignment"  # 平滑对齐
    LOCAL_REPLAN = "local_replan"  # 局部重规划
    GLOBAL_REPLAN = "global_replan"  # 全局重规划


@dataclass
class VehicleState:
    """车辆状态"""
    timestamp: float  # 时间戳（秒）
    x: float  # X坐标（米）
    y: float  # Y坐标（米）
    heading: float  # 航向角（弧度）
    speed: float  # 速度（米/秒）
    mode: DrivingMode  # 当前驾驶模式
    positioning_quality: float = 1.0  # 定位质量 (0-1)
    lateral_acceleration: float = 0.0  # 侧向加速度（米/秒²）
    longitudinal_acceleration: float = 0.0  # 纵向加速度（米/秒²）
    
    def to_dict(self):
        """转换为字典"""
        return {
            'timestamp': self.timestamp,
            'x': self.x,
            'y': self.y,
            'heading': self.heading,
            'speed': self.speed,
            'mode': self.mode.value,
            'positioning_quality': self.positioning_quality,
            'lateral_acceleration': self.lateral_acceleration,
            'longitudinal_acceleration': self.longitudinal_acceleration
        }


@dataclass
class PathPoint:
    """路径点"""
    x: float  # X坐标（米）
    y: float  # Y坐标（米）
    heading: float  # 期望航向角（弧度）
    speed: float  # 期望速度（米/秒）
    curvature: float = 0.0  # 曲率（1/米）
    segment_type: str = "straight"  # 路径段类型：straight, arc, turn
    
    def distance_to(self, x: float, y: float) -> float:
        """计算到指定点的距离"""
        return np.sqrt((self.x - x) ** 2 + (self.y - y) ** 2)


@dataclass
class Path:
    """路径"""
    points: List[PathPoint]  # 路径点列表
    path_id: str = ""  # 路径ID
    path_type: str = "coverage"  # 路径类型：coverage, approach, alignment
    total_length: float = 0.0  # 总长度（米）
    created_at: float = 0.0  # 创建时间戳
    
    def __post_init__(self):
        """初始化后计算总长度"""
        if self.total_length == 0.0 and len(self.points) > 1:
            self.total_length = sum(
                self.points[i].distance_to(self.points[i+1].x, self.points[i+1].y)
                for i in range(len(self.points) - 1)
            )
    
    def get_closest_point_index(self, x: float, y: float) -> int:
        """获取最近路径点的索引"""
        if not self.points:
            return -1
        distances = [p.distance_to(x, y) for p in self.points]
        return int(np.argmin(distances))
    
    def get_segment_at_index(self, index: int) -> Optional[Tuple[PathPoint, PathPoint]]:
        """获取指定索引处的路径段"""
        if index < 0 or index >= len(self.points) - 1:
            return None
        return (self.points[index], self.points[index + 1])


@dataclass
class DeviationInfo:
    """偏离信息"""
    lateral_deviation: float  # 横向偏差（米）
    heading_deviation: float  # 航向偏差（弧度）
    speed_deviation: float  # 速度偏差（米/秒）
    deviation_level: DeviationLevel  # 偏离等级
    closest_path_index: int  # 最近路径点索引
    distance_along_path: float  # 沿路径的距离（米）
    
    def to_dict(self):
        """转换为字典"""
        return {
            'lateral_deviation': self.lateral_deviation,
            'heading_deviation': self.heading_deviation,
            'speed_deviation': self.speed_deviation,
            'deviation_level': self.deviation_level.value,
            'closest_path_index': self.closest_path_index,
            'distance_along_path': self.distance_along_path
        }


@dataclass
class CoverageCell:
    """覆盖地图单元格"""
    status: CoverageStatus = CoverageStatus.UNCOVERED
    coverage_count: int = 0  # 覆盖次数
    last_covered_time: float = 0.0  # 最后覆盖时间
    covered_by_mode: Optional[DrivingMode] = None  # 覆盖模式


@dataclass
class CoverageMap:
    """覆盖地图"""
    origin_x: float  # 地图原点X坐标（米）
    origin_y: float  # 地图原点Y坐标（米）
    resolution: float  # 分辨率（米/格）
    width: int  # 宽度（格数）
    height: int  # 高度（格数）
    cells: np.ndarray = field(default_factory=lambda: np.array([]))  # 单元格数组
    
    def __post_init__(self):
        """初始化单元格数组"""
        if self.cells.size == 0:
            # 创建结构化数组存储覆盖信息
            dtype = [
                ('status', 'i1'),  # CoverageStatus值
                ('count', 'i2'),  # 覆盖次数
                ('time', 'f4'),  # 最后覆盖时间
                ('mode', 'i1')  # 覆盖模式
            ]
            self.cells = np.zeros((self.height, self.width), dtype=dtype)
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """世界坐标转网格坐标"""
        # 确保x和y是标量
        if isinstance(x, np.ndarray):
            x = float(x.flat[0])
        elif hasattr(x, '__len__') and not isinstance(x, str):
            x = float(x[0])
        else:
            x = float(x)
            
        if isinstance(y, np.ndarray):
            y = float(y.flat[0])
        elif hasattr(y, '__len__') and not isinstance(y, str):
            y = float(y[0])
        else:
            y = float(y)
            
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """网格坐标转世界坐标"""
        x = self.origin_x + (grid_x + 0.5) * self.resolution
        y = self.origin_y + (grid_y + 0.5) * self.resolution
        return (x, y)
    
    def is_valid_grid(self, grid_x: int, grid_y: int) -> bool:
        """检查网格坐标是否有效"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def mark_covered(self, x: float, y: float, width: float, mode: DrivingMode, timestamp: float):
        """标记区域为已覆盖"""
        # 计算覆盖矩形的网格范围
        half_width = width / 2.0
        x_min, y_min = self.world_to_grid(x - half_width, y - half_width)
        x_max, y_max = self.world_to_grid(x + half_width, y + half_width)
        
        # 标记所有覆盖的网格
        for gx in range(max(0, x_min), min(self.width, x_max + 1)):
            for gy in range(max(0, y_min), min(self.height, y_max + 1)):
                cell = self.cells[gy, gx]
                
                # 更新覆盖状态
                if cell['status'] == CoverageStatus.UNCOVERED.value:
                    if mode == DrivingMode.MANUAL:
                        cell['status'] = CoverageStatus.MANUAL_COVERED.value
                    else:
                        cell['status'] = CoverageStatus.AUTO_COVERED.value
                elif cell['status'] == CoverageStatus.MANUAL_COVERED.value and mode == DrivingMode.AUTO:
                    cell['status'] = CoverageStatus.BOTH_COVERED.value
                elif cell['status'] == CoverageStatus.AUTO_COVERED.value and mode == DrivingMode.MANUAL:
                    cell['status'] = CoverageStatus.BOTH_COVERED.value
                
                cell['count'] += 1
                cell['time'] = timestamp
                # 存储模式的整数值
                if mode == DrivingMode.MANUAL:
                    cell['mode'] = 1
                elif mode == DrivingMode.AUTO:
                    cell['mode'] = 2
                else:
                    cell['mode'] = 0
    
    def get_coverage_rate(self) -> float:
        """计算覆盖率"""
        covered = np.sum(self.cells['status'] > 0)
        total = self.width * self.height
        return covered / total if total > 0 else 0.0
    
    def get_overlap_rate(self) -> float:
        """计算重复覆盖率"""
        both_covered = np.sum(self.cells['status'] == CoverageStatus.BOTH_COVERED.value)
        covered = np.sum(self.cells['status'] > 0)
        return both_covered / covered if covered > 0 else 0.0


@dataclass
class ModeTransitionEvent:
    """模式切换事件"""
    timestamp: float  # 时间戳
    from_mode: DrivingMode  # 切换前模式
    to_mode: DrivingMode  # 切换后模式
    position: Tuple[float, float]  # 切换位置
    reason: str  # 切换原因
    success: bool = True  # 是否成功
    
    def to_dict(self):
        """转换为字典"""
        return {
            'timestamp': self.timestamp,
            'from_mode': self.from_mode.value,
            'to_mode': self.to_mode.value,
            'position': self.position,
            'reason': self.reason,
            'success': self.success
        }


@dataclass
class WorkStatistics:
    """作业统计"""
    total_distance: float = 0.0  # 总行驶距离（米）
    auto_distance: float = 0.0  # 无人模式距离（米）
    manual_distance: float = 0.0  # 有人模式距离（米）
    total_time: float = 0.0  # 总作业时间（秒）
    auto_time: float = 0.0  # 无人模式时间（秒）
    manual_time: float = 0.0  # 有人模式时间（秒）
    mode_switches: int = 0  # 模式切换次数
    coverage_rate: float = 0.0  # 覆盖率
    overlap_rate: float = 0.0  # 重复覆盖率
    
    def to_dict(self):
        """转换为字典"""
        return {
            'total_distance': self.total_distance,
            'auto_distance': self.auto_distance,
            'manual_distance': self.manual_distance,
            'total_time': self.total_time,
            'auto_time': self.auto_time,
            'manual_time': self.manual_time,
            'mode_switches': self.mode_switches,
            'coverage_rate': self.coverage_rate,
            'overlap_rate': self.overlap_rate,
            'auto_ratio': self.auto_distance / self.total_distance if self.total_distance > 0 else 0.0
        }

