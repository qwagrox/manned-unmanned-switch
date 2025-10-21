"""
模式切换主控制器
整合所有核心模块，提供统一的接口
"""

import time
from typing import Optional, Tuple, Dict, Any
from .data_structures import (
    VehicleState, DrivingMode, Path, DeviationInfo,
    RecoveryStrategy, WorkStatistics
)
from .state_estimator import StateEstimator
from .mode_manager import ModeManager
from .coverage_manager import CoverageManager
from .path_tracker import PathTracker
from .decision_coordinator import DecisionCoordinator


class ModeSwitchController:
    """模式切换主控制器
    
    整合所有核心模块，提供统一的控制接口
    """
    
    def __init__(self, field_bounds: Tuple[float, float, float, float],
                 work_width: float = 3.2, resolution: float = 0.5):
        """初始化模式切换控制器
        
        Args:
            field_bounds: 田地边界 (min_x, min_y, max_x, max_y)
            work_width: 作业宽度（米）
            resolution: 覆盖地图分辨率（米）
        """
        self.work_width = work_width
        self.field_bounds = field_bounds
        
        # 初始化各个模块
        self.state_estimator = StateEstimator()
        self.mode_manager = ModeManager()
        self.coverage_manager = CoverageManager(field_bounds, resolution, work_width)
        self.path_tracker = PathTracker(work_width)
        self.decision_coordinator = DecisionCoordinator(work_width)
        
        # 当前状态
        self.current_vehicle_state: Optional[VehicleState] = None
        self.current_deviation: Optional[DeviationInfo] = None
        
        # 统计信息
        self.statistics = WorkStatistics()
        self.last_update_time = 0.0
        
        # 注册模式切换回调
        self.mode_manager.register_callback(self._on_mode_change)
        
        print("模式切换控制器初始化完成")
        
    def initialize(self, x: float, y: float, heading: float):
        """初始化系统状态
        
        Args:
            x: 初始X坐标
            y: 初始Y坐标
            heading: 初始航向角
        """
        self.state_estimator.reset(x, y, heading)
        self.last_update_time = time.time()
        # 创建初始车辆状态
        self.current_vehicle_state = self.state_estimator.get_vehicle_state(
            self.last_update_time, self.mode_manager.get_mode()
        )
        print(f"系统初始化: 位置=({x:.2f}, {y:.2f}), 航向={heading:.2f}rad")
        
    def update_sensors(self, gps_x: float, gps_y: float, gps_quality: float,
                      imu_heading: float, imu_heading_rate: float,
                      wheel_speed: float, timestamp: float):
        """更新传感器数据
        
        Args:
            gps_x: GPS X坐标
            gps_y: GPS Y坐标
            gps_quality: GPS质量 (0-1)
            imu_heading: IMU航向角（弧度）
            imu_heading_rate: IMU航向角速度（弧度/秒）
            wheel_speed: 轮速（米/秒）
            timestamp: 时间戳（秒）
        """
        # 计算时间间隔
        dt = timestamp - self.last_update_time if self.last_update_time > 0 else 0.02
        
        # 状态估计器预测
        self.state_estimator.predict(dt)
        
        # 更新各传感器数据
        self.state_estimator.update_gps(gps_x, gps_y, gps_quality)
        self.state_estimator.update_imu(imu_heading, imu_heading_rate)
        self.state_estimator.update_speed(wheel_speed)
        
        # 获取融合后的车辆状态
        self.current_vehicle_state = self.state_estimator.get_vehicle_state(
            timestamp, self.mode_manager.get_mode()
        )
        
        # 更新覆盖地图
        self.coverage_manager.update(self.current_vehicle_state)
        
        # 更新路径跟踪
        if self.path_tracker.current_path is not None:
            self.current_deviation = self.path_tracker.update(self.current_vehicle_state)
        
        # 更新统计信息
        self._update_statistics(dt)
        
        self.last_update_time = timestamp
        
    def set_path(self, path: Path):
        """设置作业路径
        
        Args:
            path: 路径对象
        """
        self.path_tracker.set_path(path)
        print(f"设置作业路径: {len(path.points)} 个点, 总长度 {path.total_length:.2f}m")
        
    def request_mode_change(self, target_mode: DrivingMode, reason: str = "") -> bool:
        """请求模式切换
        
        Args:
            target_mode: 目标模式
            reason: 切换原因
            
        Returns:
            是否接受切换请求
        """
        if self.current_vehicle_state is None:
            print("错误: 车辆状态未初始化")
            return False
        
        # 安全性评估
        is_safe, safety_reason = self.decision_coordinator.evaluate_mode_switch_safety(
            self.current_vehicle_state, target_mode
        )
        
        if not is_safe:
            print(f"模式切换被拒绝: {safety_reason}")
            return False
        
        # 请求切换
        success = self.mode_manager.request_mode_change(
            target_mode, self.current_vehicle_state, reason
        )
        
        if success:
            # 如果切换成功，立即完成切换（简化处理）
            self.mode_manager.complete_mode_change(target_mode, self.current_vehicle_state)
        
        return success
    
    def get_recovery_strategy(self) -> Optional[RecoveryStrategy]:
        """获取当前推荐的恢复策略
        
        Returns:
            恢复策略，如果不需要恢复则返回None
        """
        if self.current_deviation is None or self.current_vehicle_state is None:
            return None
        
        return self.decision_coordinator.decide_recovery_strategy(
            self.current_deviation, self.current_vehicle_state
        )
    
    def compute_control_command(self) -> Tuple[float, float]:
        """计算控制指令（仅在无人模式下有效）
        
        Returns:
            (目标航向角, 目标速度)
        """
        if self.current_vehicle_state is None:
            return 0.0, 0.0
        
        if self.mode_manager.get_mode() != DrivingMode.AUTO:
            return self.current_vehicle_state.heading, 0.0
        
        return self.path_tracker.compute_control(self.current_vehicle_state)
    
    def get_status(self) -> Dict[str, Any]:
        """获取系统状态
        
        Returns:
            状态字典
        """
        if self.current_vehicle_state is None:
            return {'error': '系统未初始化'}
        
        status = {
            'timestamp': self.current_vehicle_state.timestamp,
            'mode': self.mode_manager.get_mode().value,
            'position': (self.current_vehicle_state.x, self.current_vehicle_state.y),
            'heading': self.current_vehicle_state.heading,
            'speed': self.current_vehicle_state.speed,
            'positioning_quality': self.current_vehicle_state.positioning_quality,
            'coverage_rate': self.coverage_manager.get_coverage_rate(),
            'overlap_rate': self.coverage_manager.get_overlap_rate(),
            'path_progress': self.path_tracker.get_progress() if self.path_tracker.current_path else 0.0
        }
        
        if self.current_deviation is not None:
            status['deviation'] = {
                'lateral': self.current_deviation.lateral_deviation,
                'heading': self.current_deviation.heading_deviation,
                'level': self.current_deviation.deviation_level.value
            }
        
        # 添加决策建议
        status['recommendation'] = self.decision_coordinator.recommend_action(
            self.current_vehicle_state,
            self.current_deviation,
            self.coverage_manager.get_coverage_rate()
        )
        
        return status
    
    def get_statistics(self) -> Dict[str, Any]:
        """获取作业统计信息
        
        Returns:
            统计信息字典
        """
        stats = self.statistics.to_dict()
        
        # 添加覆盖统计
        coverage_stats = self.coverage_manager.get_statistics()
        stats.update(coverage_stats)
        
        # 添加模式切换统计
        mode_stats = self.mode_manager.get_statistics()
        stats.update(mode_stats)
        
        return stats
    
    def _on_mode_change(self, from_mode: DrivingMode, to_mode: DrivingMode, 
                       vehicle_state: VehicleState):
        """模式切换回调函数
        
        Args:
            from_mode: 切换前模式
            to_mode: 切换后模式
            vehicle_state: 车辆状态
        """
        self.statistics.mode_switches += 1
        
        # 更新覆盖率统计
        self.statistics.coverage_rate = self.coverage_manager.get_coverage_rate()
        self.statistics.overlap_rate = self.coverage_manager.get_overlap_rate()
        
        print(f"模式切换回调: {from_mode.value} -> {to_mode.value}")
        
    def _update_statistics(self, dt: float):
        """更新统计信息
        
        Args:
            dt: 时间间隔（秒）
        """
        if self.current_vehicle_state is None:
            return
        
        # 更新时间
        self.statistics.total_time += dt
        
        # 更新距离
        distance = self.current_vehicle_state.speed * dt
        self.statistics.total_distance += distance
        
        # 根据模式更新
        if self.current_vehicle_state.mode == DrivingMode.AUTO:
            self.statistics.auto_time += dt
            self.statistics.auto_distance += distance
        elif self.current_vehicle_state.mode == DrivingMode.MANUAL:
            self.statistics.manual_time += dt
            self.statistics.manual_distance += distance
        
        # 更新覆盖率
        self.statistics.coverage_rate = self.coverage_manager.get_coverage_rate()
        self.statistics.overlap_rate = self.coverage_manager.get_overlap_rate()
    
    def get_coverage_grid(self):
        """获取覆盖网格（用于可视化）"""
        return self.coverage_manager.get_coverage_grid()
    
    def is_work_completed(self) -> bool:
        """判断作业是否完成"""
        coverage_rate = self.coverage_manager.get_coverage_rate()
        return coverage_rate > 0.95

