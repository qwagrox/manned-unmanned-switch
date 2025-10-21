"""
场景模拟器
模拟各种有人/无人模式切换场景
"""

import time
import numpy as np
from typing import Dict, Any, List
from .sensor_simulator import SensorSimulator
from .vehicle_simulator import VehicleSimulator
from .path_planner_adapter import PathPlannerAdapter
from ..core import ModeSwitchController, DrivingMode


class ScenarioSimulator:
    """场景模拟器
    
    整合所有模拟器，提供完整的场景模拟
    """
    
    def __init__(self, field_bounds: tuple, work_width: float = 3.2):
        """初始化场景模拟器
        
        Args:
            field_bounds: 田地边界 (min_x, min_y, max_x, max_y)
            work_width: 作业宽度（米）
        """
        self.field_bounds = field_bounds
        self.work_width = work_width
        
        # 初始化各个模拟器
        self.sensor_sim = SensorSimulator()
        self.vehicle_sim = VehicleSimulator()
        self.path_planner = PathPlannerAdapter(work_width)
        
        # 初始化控制器
        self.controller = ModeSwitchController(field_bounds, work_width)
        
        # 模拟参数
        self.dt = 0.05  # 时间步长（秒），对应20Hz
        self.current_time = 0.0
        
        # 场景状态
        self.scenario_events: List[Dict[str, Any]] = []
        
    def reset(self, start_x: float = None, start_y: float = None, start_heading: float = 0.0):
        """重置模拟器
        
        Args:
            start_x: 起始X坐标
            start_y: 起始Y坐标
            start_heading: 起始航向角
        """
        min_x, min_y, max_x, max_y = self.field_bounds
        
        if start_x is None:
            start_x = min_x + self.work_width
        if start_y is None:
            start_y = min_y + self.work_width
        
        # 重置各模拟器
        self.sensor_sim.reset()
        self.vehicle_sim.reset(start_x, start_y, start_heading)
        
        # 重置控制器
        self.controller.initialize(start_x, start_y, start_heading)
        
        self.current_time = 0.0
        self.scenario_events = []
        
        print(f"场景模拟器重置: 起点=({start_x:.2f}, {start_y:.2f}), 航向={start_heading:.2f}rad")
        
    def generate_and_set_path(self):
        """生成并设置作业路径"""
        path = self.path_planner.generate_coverage_path(self.field_bounds)
        self.controller.set_path(path)
        return path
    
    def step(self, manual_control: Dict[str, float] = None) -> Dict[str, Any]:
        """执行一个模拟步骤
        
        Args:
            manual_control: 手动控制输入 {'speed_delta': float, 'steering_delta': float}
            
        Returns:
            当前状态信息
        """
        # 获取车辆真实状态
        true_x, true_y, true_heading, true_speed, true_heading_rate = self.vehicle_sim.get_state()
        
        # 模拟传感器测量
        gps_x, gps_y, gps_quality = self.sensor_sim.simulate_gps(true_x, true_y)
        imu_heading, imu_heading_rate = self.sensor_sim.simulate_imu(true_heading, true_heading_rate)
        wheel_speed = self.sensor_sim.simulate_wheel_speed(true_speed)
        
        # 更新控制器
        self.controller.update_sensors(
            gps_x, gps_y, gps_quality,
            imu_heading, imu_heading_rate,
            wheel_speed, self.current_time
        )
        
        # 获取当前模式
        current_mode = self.controller.mode_manager.get_mode()
        
        # 根据模式决定控制输入
        if current_mode == DrivingMode.AUTO:
            # 无人模式：使用控制器计算的控制指令
            target_heading, target_speed = self.controller.compute_control_command()
            
            # 将目标航向转换为转向角（简化）
            heading_error = target_heading - true_heading
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
            target_steering = np.clip(heading_error * 0.5, -0.3, 0.3)
            
            self.vehicle_sim.update(target_speed, target_steering, self.dt)
            
        elif current_mode == DrivingMode.MANUAL:
            # 有人模式：使用手动控制输入
            if manual_control is not None:
                self.vehicle_sim.apply_manual_control(
                    manual_control.get('speed_delta', 0.0),
                    manual_control.get('steering_delta', 0.0)
                )
            
            # 保持当前状态或缓慢减速
            self.vehicle_sim.update(true_speed * 0.95, 0.0, self.dt)
        
        # 更新时间
        self.current_time += self.dt
        
        # 获取状态信息
        status = self.controller.get_status()
        status['true_position'] = (true_x, true_y)
        status['true_heading'] = true_heading
        status['true_speed'] = true_speed
        status['simulation_time'] = self.current_time
        
        return status
    
    def run_scenario(self, duration: float, events: List[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """运行完整场景
        
        Args:
            duration: 模拟时长（秒）
            events: 场景事件列表，每个事件包含 {'time': float, 'action': str, 'params': dict}
            
        Returns:
            状态历史列表
        """
        if events is None:
            events = []
        
        # 按时间排序事件
        events = sorted(events, key=lambda e: e['time'])
        event_index = 0
        
        history = []
        
        num_steps = int(duration / self.dt)
        
        for step in range(num_steps):
            # 检查是否有事件需要触发
            while event_index < len(events) and events[event_index]['time'] <= self.current_time:
                event = events[event_index]
                self._handle_event(event)
                event_index += 1
            
            # 执行一步模拟
            status = self.step()
            history.append(status)
            
            # 每秒打印一次进度
            if step % int(1.0 / self.dt) == 0:
                mode = status['mode']
                pos = status['position']
                coverage = status['coverage_rate']
                print(f"[{self.current_time:.1f}s] 模式={mode}, 位置=({pos[0]:.1f}, {pos[1]:.1f}), 覆盖率={coverage:.1%}")
        
        return history
    
    def _handle_event(self, event: Dict[str, Any]):
        """处理场景事件
        
        Args:
            event: 事件字典
        """
        action = event['action']
        params = event.get('params', {})
        
        if action == 'switch_to_manual':
            reason = params.get('reason', '驾驶员接管')
            self.controller.request_mode_change(DrivingMode.MANUAL, reason)
            print(f"[事件 {self.current_time:.1f}s] 切换到有人模式: {reason}")
            
        elif action == 'switch_to_auto':
            reason = params.get('reason', '恢复无人作业')
            self.controller.request_mode_change(DrivingMode.AUTO, reason)
            print(f"[事件 {self.current_time:.1f}s] 切换到无人模式: {reason}")
            
        elif action == 'manual_deviation':
            # 模拟驾驶员偏离路径
            deviation_x = params.get('deviation_x', 0.0)
            deviation_y = params.get('deviation_y', 0.0)
            
            x, y, heading, speed, _ = self.vehicle_sim.get_state()
            self.vehicle_sim.x += deviation_x
            self.vehicle_sim.y += deviation_y
            print(f"[事件 {self.current_time:.1f}s] 手动偏离: ({deviation_x:.1f}, {deviation_y:.1f})m")
            
        elif action == 'set_speed':
            target_speed = params.get('speed', 2.0)
            self.vehicle_sim.speed = target_speed
            print(f"[事件 {self.current_time:.1f}s] 设置速度: {target_speed:.1f} m/s")
    
    def get_statistics(self) -> Dict[str, Any]:
        """获取模拟统计信息"""
        return self.controller.get_statistics()
    
    def get_coverage_grid(self):
        """获取覆盖网格"""
        return self.controller.get_coverage_grid()

