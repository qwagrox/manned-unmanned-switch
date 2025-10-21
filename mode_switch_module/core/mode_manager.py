"""
模式管理器模块
负责管理驾驶模式和模式切换
"""

import time
from typing import List, Optional, Callable
from .data_structures import DrivingMode, VehicleState, ModeTransitionEvent


class ModeManager:
    """模式管理器
    
    实现状态机，管理有人/无人模式的切换
    """
    
    def __init__(self):
        """初始化模式管理器"""
        self.current_mode = DrivingMode.MANUAL
        self.previous_mode = DrivingMode.MANUAL
        self.transition_start_time = 0.0
        self.transition_timeout = 5.0  # 切换超时时间（秒）
        
        # 切换历史记录
        self.transition_history: List[ModeTransitionEvent] = []
        
        # 切换回调函数
        self.on_mode_change_callbacks: List[Callable] = []
        
        # 切换条件检查器
        self.condition_checkers = {
            DrivingMode.AUTO: self._check_auto_conditions,
            DrivingMode.MANUAL: self._check_manual_conditions
        }
        
    def request_mode_change(self, target_mode: DrivingMode, vehicle_state: VehicleState, 
                           reason: str = "") -> bool:
        """请求模式切换
        
        Args:
            target_mode: 目标模式
            vehicle_state: 当前车辆状态
            reason: 切换原因
            
        Returns:
            是否接受切换请求
        """
        # 如果已经是目标模式，直接返回
        if self.current_mode == target_mode:
            return True
        
        # 如果正在切换中，拒绝新的切换请求
        if self.current_mode == DrivingMode.TRANSITION:
            return False
        
        # 检查切换条件
        if target_mode in self.condition_checkers:
            can_switch, error_msg = self.condition_checkers[target_mode](vehicle_state)
            if not can_switch:
                print(f"模式切换被拒绝: {error_msg}")
                return False
        
        # 进入过渡状态
        self.previous_mode = self.current_mode
        self.current_mode = DrivingMode.TRANSITION
        self.transition_start_time = vehicle_state.timestamp
        
        # 记录切换事件
        event = ModeTransitionEvent(
            timestamp=vehicle_state.timestamp,
            from_mode=self.previous_mode,
            to_mode=target_mode,
            position=(vehicle_state.x, vehicle_state.y),
            reason=reason,
            success=True
        )
        self.transition_history.append(event)
        
        print(f"模式切换: {self.previous_mode.value} -> {target_mode.value}, 原因: {reason}")
        
        return True
    
    def complete_mode_change(self, target_mode: DrivingMode, vehicle_state: VehicleState) -> bool:
        """完成模式切换
        
        Args:
            target_mode: 目标模式
            vehicle_state: 当前车辆状态
            
        Returns:
            是否成功完成切换
        """
        if self.current_mode != DrivingMode.TRANSITION:
            return False
        
        # 检查超时
        elapsed = vehicle_state.timestamp - self.transition_start_time
        if elapsed > self.transition_timeout:
            print(f"模式切换超时，回滚到 {self.previous_mode.value}")
            self.current_mode = self.previous_mode
            return False
        
        # 完成切换
        self.current_mode = target_mode
        
        # 触发回调
        for callback in self.on_mode_change_callbacks:
            callback(self.previous_mode, target_mode, vehicle_state)
        
        print(f"模式切换完成: {target_mode.value}")
        return True
    
    def force_mode_change(self, target_mode: DrivingMode, vehicle_state: VehicleState, 
                         reason: str = "强制切换"):
        """强制模式切换（用于紧急情况）
        
        Args:
            target_mode: 目标模式
            vehicle_state: 当前车辆状态
            reason: 切换原因
        """
        old_mode = self.current_mode
        self.current_mode = target_mode
        
        # 记录切换事件
        event = ModeTransitionEvent(
            timestamp=vehicle_state.timestamp,
            from_mode=old_mode,
            to_mode=target_mode,
            position=(vehicle_state.x, vehicle_state.y),
            reason=reason,
            success=True
        )
        self.transition_history.append(event)
        
        # 触发回调
        for callback in self.on_mode_change_callbacks:
            callback(old_mode, target_mode, vehicle_state)
        
        print(f"强制模式切换: {old_mode.value} -> {target_mode.value}, 原因: {reason}")
    
    def _check_auto_conditions(self, vehicle_state: VehicleState) -> tuple:
        """检查切换到无人模式的条件
        
        Args:
            vehicle_state: 车辆状态
            
        Returns:
            (是否满足条件, 错误消息)
        """
        # 处理numpy数组
        import numpy as np
        
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
        
        # 检查定位质量
        if quality < 0.7:
            return False, f"定位质量不足 ({quality:.2f} < 0.7)"
        
        # 检查速度（不能在高速时切换）
        if speed > 5.0:
            return False, f"速度过高 ({speed:.2f} m/s > 5.0 m/s)"
        
        return True, ""
    
    def _check_manual_conditions(self, vehicle_state: VehicleState) -> tuple:
        """检查切换到有人模式的条件
        
        Args:
            vehicle_state: 车辆状态
            
        Returns:
            (是否满足条件, 错误消息)
        """
        # 切换到有人模式通常没有限制
        return True, ""
    
    def register_callback(self, callback: Callable):
        """注册模式切换回调函数
        
        Args:
            callback: 回调函数，签名为 callback(from_mode, to_mode, vehicle_state)
        """
        self.on_mode_change_callbacks.append(callback)
    
    def get_mode(self) -> DrivingMode:
        """获取当前模式"""
        return self.current_mode
    
    def get_transition_history(self) -> List[ModeTransitionEvent]:
        """获取切换历史"""
        return self.transition_history
    
    def get_statistics(self) -> dict:
        """获取模式切换统计信息"""
        total_switches = len(self.transition_history)
        
        # 统计各种切换类型
        auto_to_manual = sum(1 for e in self.transition_history 
                            if e.from_mode == DrivingMode.AUTO and e.to_mode == DrivingMode.MANUAL)
        manual_to_auto = sum(1 for e in self.transition_history 
                            if e.from_mode == DrivingMode.MANUAL and e.to_mode == DrivingMode.AUTO)
        
        return {
            'total_switches': total_switches,
            'auto_to_manual': auto_to_manual,
            'manual_to_auto': manual_to_auto,
            'current_mode': self.current_mode.value
        }

