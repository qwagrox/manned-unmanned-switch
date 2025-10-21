"""
覆盖管理器模块
负责维护和查询覆盖地图
"""

import numpy as np
from typing import List, Tuple, Optional
from .data_structures import CoverageMap, CoverageStatus, VehicleState, DrivingMode


class CoverageManager:
    """覆盖管理器
    
    维护高精度的覆盖地图，实时跟踪作业覆盖情况
    """
    
    def __init__(self, field_bounds: Tuple[float, float, float, float], 
                 resolution: float = 0.5, work_width: float = 3.2):
        """初始化覆盖管理器
        
        Args:
            field_bounds: 田地边界 (min_x, min_y, max_x, max_y)
            resolution: 网格分辨率（米）
            work_width: 作业宽度（米）
        """
        self.work_width = work_width
        
        min_x, min_y, max_x, max_y = field_bounds
        
        # 计算地图尺寸
        width = int(np.ceil((max_x - min_x) / resolution))
        height = int(np.ceil((max_y - min_y) / resolution))
        
        # 创建覆盖地图
        self.coverage_map = CoverageMap(
            origin_x=min_x,
            origin_y=min_y,
            resolution=resolution,
            width=width,
            height=height
        )
        
        print(f"覆盖地图初始化: {width}x{height} 格, 分辨率 {resolution}m")
        
    def update(self, vehicle_state: VehicleState):
        """更新覆盖地图
        
        Args:
            vehicle_state: 当前车辆状态
        """
        # 只在有人或无人模式下更新覆盖
        if vehicle_state.mode not in [DrivingMode.MANUAL, DrivingMode.AUTO]:
            return
        
        # 标记当前位置为已覆盖
        self.coverage_map.mark_covered(
            vehicle_state.x,
            vehicle_state.y,
            self.work_width,
            vehicle_state.mode,
            vehicle_state.timestamp
        )
    
    def get_coverage_rate(self) -> float:
        """获取覆盖率"""
        return self.coverage_map.get_coverage_rate()
    
    def get_overlap_rate(self) -> float:
        """获取重复覆盖率"""
        return self.coverage_map.get_overlap_rate()
    
    def is_covered(self, x: float, y: float) -> bool:
        """检查指定位置是否已覆盖
        
        Args:
            x: X坐标
            y: Y坐标
            
        Returns:
            是否已覆盖
        """
        gx, gy = self.coverage_map.world_to_grid(x, y)
        if not self.coverage_map.is_valid_grid(gx, gy):
            return False
        
        status = self.coverage_map.cells[gy, gx]['status']
        return status > CoverageStatus.UNCOVERED.value
    
    def get_coverage_status(self, x: float, y: float) -> CoverageStatus:
        """获取指定位置的覆盖状态
        
        Args:
            x: X坐标
            y: Y坐标
            
        Returns:
            覆盖状态
        """
        gx, gy = self.coverage_map.world_to_grid(x, y)
        if not self.coverage_map.is_valid_grid(gx, gy):
            return CoverageStatus.UNCOVERED
        
        status_value = self.coverage_map.cells[gy, gx]['status']
        return CoverageStatus(status_value)
    
    def find_uncovered_regions(self) -> List[Tuple[float, float, float]]:
        """查找未覆盖区域
        
        Returns:
            未覆盖区域列表，每个元素为 (center_x, center_y, area)
        """
        uncovered_cells = np.where(self.coverage_map.cells['status'] == CoverageStatus.UNCOVERED.value)
        
        if len(uncovered_cells[0]) == 0:
            return []
        
        # 简化处理：将所有未覆盖格子作为独立区域返回
        regions = []
        for i in range(len(uncovered_cells[0])):
            gy, gx = uncovered_cells[0][i], uncovered_cells[1][i]
            x, y = self.coverage_map.grid_to_world(gx, gy)
            area = self.coverage_map.resolution ** 2
            regions.append((x, y, area))
        
        return regions
    
    def find_overlap_regions(self) -> List[Tuple[float, float, float]]:
        """查找重复覆盖区域
        
        Returns:
            重复覆盖区域列表，每个元素为 (center_x, center_y, area)
        """
        overlap_cells = np.where(self.coverage_map.cells['status'] == CoverageStatus.BOTH_COVERED.value)
        
        if len(overlap_cells[0]) == 0:
            return []
        
        regions = []
        for i in range(len(overlap_cells[0])):
            gy, gx = overlap_cells[0][i], overlap_cells[1][i]
            x, y = self.coverage_map.grid_to_world(gx, gy)
            area = self.coverage_map.resolution ** 2
            regions.append((x, y, area))
        
        return regions
    
    def find_nearest_uncovered(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """查找最近的未覆盖点
        
        Args:
            x: 起始X坐标
            y: 起始Y坐标
            
        Returns:
            最近未覆盖点的坐标，如果没有则返回None
        """
        gx, gy = self.coverage_map.world_to_grid(x, y)
        
        # 使用BFS查找最近的未覆盖格子
        from collections import deque
        
        visited = set()
        queue = deque([(gx, gy, 0)])
        visited.add((gx, gy))
        
        while queue:
            cx, cy, dist = queue.popleft()
            
            # 检查是否未覆盖
            if self.coverage_map.is_valid_grid(cx, cy):
                if self.coverage_map.cells[cy, cx]['status'] == CoverageStatus.UNCOVERED.value:
                    return self.coverage_map.grid_to_world(cx, cy)
            
            # 扩展邻居
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) not in visited and self.coverage_map.is_valid_grid(nx, ny):
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))
            
            # 限制搜索范围
            if dist > 100:
                break
        
        return None
    
    def get_statistics(self) -> dict:
        """获取覆盖统计信息"""
        total_cells = self.coverage_map.width * self.coverage_map.height
        uncovered = np.sum(self.coverage_map.cells['status'] == CoverageStatus.UNCOVERED.value)
        manual_covered = np.sum(self.coverage_map.cells['status'] == CoverageStatus.MANUAL_COVERED.value)
        auto_covered = np.sum(self.coverage_map.cells['status'] == CoverageStatus.AUTO_COVERED.value)
        both_covered = np.sum(self.coverage_map.cells['status'] == CoverageStatus.BOTH_COVERED.value)
        
        coverage_rate = (total_cells - uncovered) / total_cells if total_cells > 0 else 0.0
        overlap_rate = both_covered / (total_cells - uncovered) if (total_cells - uncovered) > 0 else 0.0
        
        return {
            'total_cells': total_cells,
            'uncovered_cells': int(uncovered),
            'manual_covered_cells': int(manual_covered),
            'auto_covered_cells': int(auto_covered),
            'both_covered_cells': int(both_covered),
            'coverage_rate': coverage_rate,
            'overlap_rate': overlap_rate
        }
    
    def get_coverage_grid(self) -> np.ndarray:
        """获取覆盖网格（用于可视化）
        
        Returns:
            覆盖状态网格
        """
        return self.coverage_map.cells['status'].copy()

