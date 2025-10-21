"""
可视化工具
用于实时显示和记录模拟过程
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
from typing import List, Dict, Any


class Visualizer:
    """可视化工具
    
    实时显示车辆位置、路径、覆盖地图等
    """
    
    def __init__(self, field_bounds: tuple, work_width: float = 3.2):
        """初始化可视化工具
        
        Args:
            field_bounds: 田地边界 (min_x, min_y, max_x, max_y)
            work_width: 作业宽度（米）
        """
        self.field_bounds = field_bounds
        self.work_width = work_width
        
        # 创建图形
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.suptitle('模式切换系统可视化', fontsize=16)
        
        # 子图1: 路径和车辆位置
        self.ax_path = self.axes[0, 0]
        self.ax_path.set_title('路径跟踪')
        self.ax_path.set_xlabel('X (m)')
        self.ax_path.set_ylabel('Y (m)')
        self.ax_path.set_aspect('equal')
        self.ax_path.grid(True, alpha=0.3)
        
        # 子图2: 覆盖地图
        self.ax_coverage = self.axes[0, 1]
        self.ax_coverage.set_title('覆盖地图')
        self.ax_coverage.set_xlabel('X (m)')
        self.ax_coverage.set_ylabel('Y (m)')
        self.ax_coverage.set_aspect('equal')
        
        # 子图3: 偏离曲线
        self.ax_deviation = self.axes[1, 0]
        self.ax_deviation.set_title('横向偏离')
        self.ax_deviation.set_xlabel('时间 (s)')
        self.ax_deviation.set_ylabel('偏离 (m)')
        self.ax_deviation.grid(True, alpha=0.3)
        
        # 子图4: 统计信息
        self.ax_stats = self.axes[1, 1]
        self.ax_stats.set_title('作业统计')
        self.ax_stats.axis('off')
        
        # 绘图元素
        self.path_line = None
        self.vehicle_marker = None
        self.vehicle_arrow = None
        self.coverage_image = None
        self.deviation_line = None
        self.stats_text = None
        
        # 数据缓存
        self.history_time = []
        self.history_deviation = []
        
    def plot_field_boundary(self):
        """绘制田地边界"""
        min_x, min_y, max_x, max_y = self.field_bounds
        
        rect = patches.Rectangle(
            (min_x, min_y), max_x - min_x, max_y - min_y,
            linewidth=2, edgecolor='black', facecolor='none'
        )
        self.ax_path.add_patch(rect)
        
        # 设置坐标轴范围
        margin = 5
        self.ax_path.set_xlim(min_x - margin, max_x + margin)
        self.ax_path.set_ylim(min_y - margin, max_y + margin)
        
    def plot_path(self, path):
        """绘制路径
        
        Args:
            path: 路径对象
        """
        if path is None or len(path.points) == 0:
            return
        
        x_coords = [p.x for p in path.points]
        y_coords = [p.y for p in path.points]
        
        if self.path_line is None:
            self.path_line, = self.ax_path.plot(
                x_coords, y_coords, 'b-', linewidth=1, alpha=0.5, label='规划路径'
            )
        else:
            self.path_line.set_data(x_coords, y_coords)
        
        self.ax_path.legend()
        
    def update_vehicle(self, x: float, y: float, heading: float, mode: str):
        """更新车辆位置
        
        Args:
            x: X坐标
            y: Y坐标
            heading: 航向角
            mode: 驾驶模式
        """
        # 根据模式选择颜色
        color = 'green' if mode == 'auto' else 'orange'
        
        # 更新车辆标记
        if self.vehicle_marker is None:
            self.vehicle_marker, = self.ax_path.plot(
                [x], [y], 'o', color=color, markersize=10, label='车辆'
            )
        else:
            self.vehicle_marker.set_data([x], [y])
            self.vehicle_marker.set_color(color)
        
        # 更新车辆方向箭头
        arrow_length = 2.0
        dx = arrow_length * np.cos(heading)
        dy = arrow_length * np.sin(heading)
        
        if self.vehicle_arrow is not None:
            self.vehicle_arrow.remove()
        
        self.vehicle_arrow = self.ax_path.arrow(
            x, y, dx, dy,
            head_width=1.0, head_length=0.5, fc=color, ec=color, alpha=0.7
        )
        
    def update_coverage(self, coverage_grid, resolution: float, origin_x: float, origin_y: float):
        """更新覆盖地图
        
        Args:
            coverage_grid: 覆盖网格
            resolution: 分辨率
            origin_x: 原点X坐标
            origin_y: 原点Y坐标
        """
        if coverage_grid is None:
            return
        
        # 创建颜色映射
        # 0: 白色（未覆盖）, 1: 蓝色（有人覆盖）, 2: 绿色（无人覆盖）, 3: 红色（重复覆盖）
        colors = np.zeros((*coverage_grid.shape, 3))
        colors[coverage_grid == 0] = [1, 1, 1]  # 白色
        colors[coverage_grid == 1] = [0.8, 0.8, 1]  # 浅蓝色
        colors[coverage_grid == 2] = [0.8, 1, 0.8]  # 浅绿色
        colors[coverage_grid == 3] = [1, 0.8, 0.8]  # 浅红色
        
        if self.coverage_image is None:
            self.coverage_image = self.ax_coverage.imshow(
                colors, origin='lower',
                extent=[origin_x, origin_x + coverage_grid.shape[1] * resolution,
                       origin_y, origin_y + coverage_grid.shape[0] * resolution],
                interpolation='nearest'
            )
        else:
            self.coverage_image.set_data(colors)
        
    def update_deviation(self, time: float, deviation: float):
        """更新偏离曲线
        
        Args:
            time: 时间
            deviation: 横向偏离
        """
        self.history_time.append(time)
        self.history_deviation.append(deviation)
        
        # 只保留最近100秒的数据
        if len(self.history_time) > 2000:
            self.history_time = self.history_time[-2000:]
            self.history_deviation = self.history_deviation[-2000:]
        
        if self.deviation_line is None:
            self.deviation_line, = self.ax_deviation.plot(
                self.history_time, self.history_deviation, 'b-', linewidth=1
            )
            # 添加参考线
            self.ax_deviation.axhline(y=0, color='k', linestyle='--', alpha=0.3)
            self.ax_deviation.axhline(y=self.work_width/2, color='r', linestyle='--', alpha=0.3, label='作业宽度/2')
            self.ax_deviation.axhline(y=-self.work_width/2, color='r', linestyle='--', alpha=0.3)
            self.ax_deviation.legend()
        else:
            self.deviation_line.set_data(self.history_time, self.history_deviation)
            
            # 自动调整X轴范围
            if len(self.history_time) > 0:
                self.ax_deviation.set_xlim(max(0, self.history_time[-1] - 50), self.history_time[-1] + 5)
        
    def update_statistics(self, stats: Dict[str, Any]):
        """更新统计信息
        
        Args:
            stats: 统计信息字典
        """
        # 清除之前的文本
        self.ax_stats.clear()
        self.ax_stats.axis('off')
        
        # 格式化统计信息
        text_lines = [
            f"总距离: {stats.get('total_distance', 0):.1f} m",
            f"无人距离: {stats.get('auto_distance', 0):.1f} m ({stats.get('auto_ratio', 0)*100:.1f}%)",
            f"有人距离: {stats.get('manual_distance', 0):.1f} m",
            f"",
            f"总时间: {stats.get('total_time', 0):.1f} s",
            f"无人时间: {stats.get('auto_time', 0):.1f} s",
            f"有人时间: {stats.get('manual_time', 0):.1f} s",
            f"",
            f"覆盖率: {stats.get('coverage_rate', 0)*100:.1f}%",
            f"重复率: {stats.get('overlap_rate', 0)*100:.1f}%",
            f"模式切换: {stats.get('mode_switches', 0)} 次",
        ]
        
        text = '\n'.join(text_lines)
        self.ax_stats.text(0.1, 0.9, text, transform=self.ax_stats.transAxes,
                          fontsize=12, verticalalignment='top', family='monospace')
        
    def update(self, status: Dict[str, Any], coverage_grid=None, 
              coverage_resolution: float = 0.5, coverage_origin: tuple = (0, 0)):
        """更新所有可视化
        
        Args:
            status: 状态字典
            coverage_grid: 覆盖网格
            coverage_resolution: 覆盖地图分辨率
            coverage_origin: 覆盖地图原点
        """
        # 更新车辆位置
        pos = status.get('position', (0, 0))
        heading = status.get('heading', 0)
        mode = status.get('mode', 'manual')
        self.update_vehicle(pos[0], pos[1], heading, mode)
        
        # 更新偏离曲线
        if 'deviation' in status:
            deviation = status['deviation'].get('lateral', 0)
            time = status.get('simulation_time', 0)
            self.update_deviation(time, deviation)
        
        # 更新覆盖地图
        if coverage_grid is not None:
            self.update_coverage(coverage_grid, coverage_resolution, 
                               coverage_origin[0], coverage_origin[1])
        
        # 刷新显示
        plt.pause(0.001)
        
    def save_figure(self, filename: str):
        """保存图形
        
        Args:
            filename: 文件名
        """
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"图形已保存到: {filename}")
        
    def show(self):
        """显示图形"""
        plt.show()

