"""
模式切换核心模块
"""

from .data_structures import (
    DrivingMode, CoverageStatus, DeviationLevel, RecoveryStrategy,
    VehicleState, PathPoint, Path, DeviationInfo, CoverageMap,
    ModeTransitionEvent, WorkStatistics
)

from .state_estimator import StateEstimator
from .mode_manager import ModeManager
from .coverage_manager import CoverageManager
from .path_tracker import PathTracker
from .decision_coordinator import DecisionCoordinator
from .mode_switch_controller import ModeSwitchController

__all__ = [
    # 枚举类型
    'DrivingMode',
    'CoverageStatus',
    'DeviationLevel',
    'RecoveryStrategy',
    
    # 数据结构
    'VehicleState',
    'PathPoint',
    'Path',
    'DeviationInfo',
    'CoverageMap',
    'ModeTransitionEvent',
    'WorkStatistics',
    
    # 核心模块
    'StateEstimator',
    'ModeManager',
    'CoverageManager',
    'PathTracker',
    'DecisionCoordinator',
    'ModeSwitchController',
]

