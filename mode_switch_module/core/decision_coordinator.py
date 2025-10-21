"""
决策协调器模块
负责综合各模块信息，做出智能决策
"""

from typing import Optional, Tuple
from .data_structures import (
    VehicleState, DeviationInfo, DeviationLevel, 
    RecoveryStrategy, DrivingMode, Path
)


class DecisionCoordinator:
    """决策协调器
    
    实现基于规则的决策引擎，协调各模块的工作
    """
    
    def __init__(self, work_width: float = 3.2):
        """初始化决策协调器
        
        Args:
            work_width: 作业宽度（米）
        """
        self.work_width = work_width
        
        # 决策阈值
        self.minimal_deviation_threshold = 0.5 * work_width
        self.slight_deviation_threshold = 1.5 * work_width
        self.moderate_deviation_threshold = 3.0 * work_width
        
        self.replan_trigger_count = 0
        self.last_replan_time = 0.0
        self.replan_cooldown = 5.0  # 重规划冷却时间（秒）
        
    def decide_recovery_strategy(self, deviation_info: DeviationInfo, 
                                 vehicle_state: VehicleState) -> RecoveryStrategy:
        """决定恢复策略
        
        Args:
            deviation_info: 偏离信息
            vehicle_state: 车辆状态
            
        Returns:
            恢复策略
        """
        # 根据偏离等级决定策略
        if deviation_info.deviation_level == DeviationLevel.MINIMAL:
            return RecoveryStrategy.DIRECT_RESUME
        
        elif deviation_info.deviation_level == DeviationLevel.SLIGHT:
            return RecoveryStrategy.SMOOTH_ALIGNMENT
        
        elif deviation_info.deviation_level == DeviationLevel.MODERATE:
            return RecoveryStrategy.LOCAL_REPLAN
        
        else:  # SEVERE
            return RecoveryStrategy.GLOBAL_REPLAN
    
    def should_trigger_replan(self, deviation_info: DeviationInfo, 
                             vehicle_state: VehicleState,
                             coverage_rate: float) -> bool:
        """判断是否应该触发重规划
        
        Args:
            deviation_info: 偏离信息
            vehicle_state: 车辆状态
            coverage_rate: 当前覆盖率
            
        Returns:
            是否触发重规划
        """
        # 检查冷却时间
        if vehicle_state.timestamp - self.last_replan_time < self.replan_cooldown:
            return False
        
        # 严重偏离时触发重规划
        if deviation_info.deviation_level == DeviationLevel.SEVERE:
            return True
        
        # 中度偏离且覆盖率较低时触发重规划
        if deviation_info.deviation_level == DeviationLevel.MODERATE and coverage_rate < 0.5:
            return True
        
        return False
    
    def should_skip_covered_segment(self, segment_coverage_rate: float) -> bool:
        """判断是否应该跳过已覆盖的路径段
        
        Args:
            segment_coverage_rate: 路径段的覆盖率
            
        Returns:
            是否跳过
        """
        # 如果路径段覆盖率超过80%，建议跳过
        return segment_coverage_rate > 0.8
    
    def should_compensate_uncovered(self, uncovered_area: float, 
                                   total_area: float) -> bool:
        """判断是否应该进行补偿作业
        
        Args:
            uncovered_area: 未覆盖面积（平方米）
            total_area: 总面积（平方米）
            
        Returns:
            是否需要补偿
        """
        # 如果未覆盖面积超过5平方米且占比超过2%，需要补偿
        if uncovered_area < 5.0:
            return False
        
        uncovered_ratio = uncovered_area / total_area if total_area > 0 else 0.0
        return uncovered_ratio > 0.02
    
    def evaluate_mode_switch_safety(self, vehicle_state: VehicleState, 
                                    target_mode: DrivingMode) -> Tuple[bool, str]:
        """评估模式切换的安全性
        
        Args:
            vehicle_state: 车辆状态
            target_mode: 目标模式
            
        Returns:
            (是否安全, 原因说明)
        """
        if target_mode == DrivingMode.AUTO:
            # 切换到无人模式的安全检查
            # 处理numpy数组
            quality = vehicle_state.positioning_quality
            if isinstance(quality, np.ndarray):
                quality = float(quality.flat[0])
            else:
                quality = float(quality)
                
            speed = vehicle_state.speed
            if isinstance(speed, np.ndarray):
                speed = float(speed.flat[0])
            else:
                speed = float(speed)
                
            lateral_acc = vehicle_state.lateral_acceleration
            if isinstance(lateral_acc, np.ndarray):
                lateral_acc = float(lateral_acc.flat[0])
            else:
                lateral_acc = float(lateral_acc)
            
            if quality < 0.7:
                return False, "定位质量不足"
            
            if speed > 5.0:
                return False, "速度过高"
            
            if abs(lateral_acc) > 2.0:
                return False, "侧向加速度过大"
        
        return True, "安全"
    
    def recommend_action(self, vehicle_state: VehicleState, 
                        deviation_info: Optional[DeviationInfo],
                        coverage_rate: float) -> str:
        """推荐操作建议
        
        Args:
            vehicle_state: 车辆状态
            deviation_info: 偏离信息
            coverage_rate: 覆盖率
            
        Returns:
            操作建议文本
        """
        if vehicle_state.mode == DrivingMode.MANUAL:
            if coverage_rate > 0.95:
                return "作业接近完成，建议检查未覆盖区域"
            else:
                return "有人模式作业中，可随时切换到无人模式"
        
        elif vehicle_state.mode == DrivingMode.AUTO:
            if deviation_info is None:
                return "无人模式作业正常"
            
            if deviation_info.deviation_level == DeviationLevel.MINIMAL:
                return "路径跟踪良好"
            elif deviation_info.deviation_level == DeviationLevel.SLIGHT:
                return f"轻微偏离 ({deviation_info.lateral_deviation:.2f}m)，系统正在调整"
            elif deviation_info.deviation_level == DeviationLevel.MODERATE:
                return f"中度偏离 ({deviation_info.lateral_deviation:.2f}m)，建议检查"
            else:
                return f"严重偏离 ({deviation_info.lateral_deviation:.2f}m)，建议切换到有人模式"
        
        return "系统状态正常"
    
    def compute_priority_score(self, uncovered_position: Tuple[float, float],
                              vehicle_position: Tuple[float, float],
                              uncovered_area: float) -> float:
        """计算未覆盖区域的优先级分数
        
        Args:
            uncovered_position: 未覆盖区域位置
            vehicle_position: 车辆位置
            uncovered_area: 未覆盖面积
            
        Returns:
            优先级分数（越高越优先）
        """
        # 计算距离
        dx = uncovered_position[0] - vehicle_position[0]
        dy = uncovered_position[1] - vehicle_position[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        # 距离越近，面积越大，优先级越高
        # 使用加权公式
        distance_score = 1.0 / (1.0 + distance / 10.0)  # 距离权重
        area_score = min(uncovered_area / 10.0, 1.0)  # 面积权重
        
        priority = 0.6 * distance_score + 0.4 * area_score
        
        return priority


import numpy as np

