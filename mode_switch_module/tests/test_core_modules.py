"""
核心模块单元测试
"""

import unittest
import numpy as np

from core import (
    DrivingMode, DeviationLevel, VehicleState, Path, PathPoint,
    StateEstimator, ModeManager, CoverageManager, PathTracker,
    DecisionCoordinator, ModeSwitchController
)


class TestDataStructures(unittest.TestCase):
    """测试数据结构"""
    
    def test_vehicle_state(self):
        """测试车辆状态"""
        state = VehicleState(
            timestamp=1.0,
            x=10.0,
            y=20.0,
            heading=0.5,
            speed=2.0,
            mode=DrivingMode.AUTO
        )
        
        self.assertEqual(state.x, 10.0)
        self.assertEqual(state.y, 20.0)
        self.assertEqual(state.mode, DrivingMode.AUTO)
        
    def test_path(self):
        """测试路径"""
        points = [
            PathPoint(x=0, y=0, heading=0, speed=2.0),
            PathPoint(x=10, y=0, heading=0, speed=2.0),
            PathPoint(x=10, y=10, heading=1.57, speed=2.0),
        ]
        
        path = Path(points=points)
        
        self.assertEqual(len(path.points), 3)
        self.assertGreater(path.total_length, 0)
        
        # 测试最近点查找
        closest_idx = path.get_closest_point_index(5, 0)
        self.assertIsNotNone(closest_idx)


class TestStateEstimator(unittest.TestCase):
    """测试状态估计器"""
    
    def setUp(self):
        """初始化"""
        self.estimator = StateEstimator()
        self.estimator.reset(0, 0, 0)
        
    def test_initialization(self):
        """测试初始化"""
        self.assertEqual(self.estimator.state[0], 0.0)
        self.assertEqual(self.estimator.state[1], 0.0)
        
    def test_gps_update(self):
        """测试GPS更新"""
        self.estimator.update_gps(10.0, 20.0, 1.0)
        
        # 状态应该接近GPS测量值
        self.assertAlmostEqual(self.estimator.state[0], 10.0, delta=1.0)
        self.assertAlmostEqual(self.estimator.state[1], 20.0, delta=1.0)
        
    def test_predict(self):
        """测试预测"""
        self.estimator.state[2] = 1.0  # vx = 1 m/s
        self.estimator.state[3] = 0.0  # vy = 0
        
        self.estimator.predict(1.0)  # 预测1秒
        
        # X坐标应该增加约1米
        self.assertAlmostEqual(self.estimator.state[0], 1.0, delta=0.1)


class TestModeManager(unittest.TestCase):
    """测试模式管理器"""
    
    def setUp(self):
        """初始化"""
        self.manager = ModeManager()
        
    def test_initial_mode(self):
        """测试初始模式"""
        self.assertEqual(self.manager.get_mode(), DrivingMode.MANUAL)
        
    def test_mode_change_request(self):
        """测试模式切换请求"""
        vehicle_state = VehicleState(
            timestamp=1.0, x=0, y=0, heading=0, speed=1.0,
            mode=DrivingMode.MANUAL, positioning_quality=0.9
        )
        
        # 请求切换到无人模式
        success = self.manager.request_mode_change(
            DrivingMode.AUTO, vehicle_state, "测试切换"
        )
        
        self.assertTrue(success)
        self.assertEqual(self.manager.get_mode(), DrivingMode.TRANSITION)
        
        # 完成切换
        self.manager.complete_mode_change(DrivingMode.AUTO, vehicle_state)
        self.assertEqual(self.manager.get_mode(), DrivingMode.AUTO)
        
    def test_mode_change_rejection(self):
        """测试模式切换拒绝"""
        vehicle_state = VehicleState(
            timestamp=1.0, x=0, y=0, heading=0, speed=10.0,  # 速度过高
            mode=DrivingMode.MANUAL, positioning_quality=0.5  # 定位质量不足
        )
        
        # 请求切换应该被拒绝
        success = self.manager.request_mode_change(
            DrivingMode.AUTO, vehicle_state, "测试切换"
        )
        
        self.assertFalse(success)


class TestCoverageManager(unittest.TestCase):
    """测试覆盖管理器"""
    
    def setUp(self):
        """初始化"""
        field_bounds = (0, 0, 100, 60)
        self.manager = CoverageManager(field_bounds, resolution=0.5, work_width=3.2)
        
    def test_initialization(self):
        """测试初始化"""
        self.assertIsNotNone(self.manager.coverage_map)
        self.assertEqual(self.manager.get_coverage_rate(), 0.0)
        
    def test_coverage_update(self):
        """测试覆盖更新"""
        vehicle_state = VehicleState(
            timestamp=1.0, x=10, y=10, heading=0, speed=2.0,
            mode=DrivingMode.AUTO
        )
        
        # 更新覆盖
        self.manager.update(vehicle_state)
        
        # 覆盖率应该大于0
        self.assertGreater(self.manager.get_coverage_rate(), 0.0)
        
        # 检查该位置是否已覆盖
        self.assertTrue(self.manager.is_covered(10, 10))


class TestPathTracker(unittest.TestCase):
    """测试路径跟踪器"""
    
    def setUp(self):
        """初始化"""
        self.tracker = PathTracker(work_width=3.2)
        
        # 创建简单路径
        points = [
            PathPoint(x=i*1.0, y=0, heading=0, speed=2.0)
            for i in range(100)
        ]
        self.path = Path(points=points)
        self.tracker.set_path(self.path)
        
    def test_path_tracking(self):
        """测试路径跟踪"""
        vehicle_state = VehicleState(
            timestamp=1.0, x=10, y=0.5, heading=0, speed=2.0,
            mode=DrivingMode.AUTO
        )
        
        deviation = self.tracker.update(vehicle_state)
        
        self.assertIsNotNone(deviation)
        self.assertAlmostEqual(deviation.lateral_deviation, 0.5, delta=0.1)
        self.assertEqual(deviation.deviation_level, DeviationLevel.MINIMAL)
        
    def test_control_computation(self):
        """测试控制计算"""
        vehicle_state = VehicleState(
            timestamp=1.0, x=10, y=0, heading=0, speed=2.0,
            mode=DrivingMode.AUTO
        )
        
        target_heading, target_speed = self.tracker.compute_control(vehicle_state)
        
        self.assertIsNotNone(target_heading)
        self.assertIsNotNone(target_speed)
        self.assertGreater(target_speed, 0)


class TestDecisionCoordinator(unittest.TestCase):
    """测试决策协调器"""
    
    def setUp(self):
        """初始化"""
        self.coordinator = DecisionCoordinator(work_width=3.2)
        
    def test_recovery_strategy(self):
        """测试恢复策略决策"""
        from core.data_structures import DeviationInfo, RecoveryStrategy
        
        # 测试轻微偏离
        deviation = DeviationInfo(
            lateral_deviation=1.0,
            heading_deviation=0.1,
            speed_deviation=0.0,
            deviation_level=DeviationLevel.SLIGHT,
            closest_path_index=10,
            distance_along_path=10.0
        )
        
        vehicle_state = VehicleState(
            timestamp=1.0, x=10, y=1, heading=0, speed=2.0,
            mode=DrivingMode.AUTO
        )
        
        strategy = self.coordinator.decide_recovery_strategy(deviation, vehicle_state)
        self.assertEqual(strategy, RecoveryStrategy.SMOOTH_ALIGNMENT)


class TestModeSwitchController(unittest.TestCase):
    """测试模式切换控制器"""
    
    def setUp(self):
        """初始化"""
        field_bounds = (0, 0, 100, 60)
        self.controller = ModeSwitchController(field_bounds, work_width=3.2)
        self.controller.initialize(10, 10, 0)
        
    def test_initialization(self):
        """测试初始化"""
        self.assertIsNotNone(self.controller.current_vehicle_state)
        
    def test_sensor_update(self):
        """测试传感器更新"""
        self.controller.update_sensors(
            gps_x=10.0, gps_y=10.0, gps_quality=1.0,
            imu_heading=0.0, imu_heading_rate=0.0,
            wheel_speed=2.0, timestamp=1.0
        )
        
        status = self.controller.get_status()
        self.assertIsNotNone(status)
        self.assertIn('mode', status)
        self.assertIn('position', status)
        
    def test_mode_change(self):
        """测试模式切换"""
        # 先更新传感器
        self.controller.update_sensors(
            gps_x=10.0, gps_y=10.0, gps_quality=1.0,
            imu_heading=0.0, imu_heading_rate=0.0,
            wheel_speed=1.0, timestamp=1.0
        )
        
        # 请求切换到无人模式
        success = self.controller.request_mode_change(DrivingMode.AUTO, "测试")
        self.assertTrue(success)
        
        # 检查模式
        self.assertEqual(self.controller.mode_manager.get_mode(), DrivingMode.AUTO)


def run_tests():
    """运行所有测试"""
    print("=" * 60)
    print("运行单元测试")
    print("=" * 60)
    
    # 创建测试套件
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # 添加所有测试
    suite.addTests(loader.loadTestsFromTestCase(TestDataStructures))
    suite.addTests(loader.loadTestsFromTestCase(TestStateEstimator))
    suite.addTests(loader.loadTestsFromTestCase(TestModeManager))
    suite.addTests(loader.loadTestsFromTestCase(TestCoverageManager))
    suite.addTests(loader.loadTestsFromTestCase(TestPathTracker))
    suite.addTests(loader.loadTestsFromTestCase(TestDecisionCoordinator))
    suite.addTests(loader.loadTestsFromTestCase(TestModeSwitchController))
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 打印总结
    print("\n" + "=" * 60)
    print(f"测试完成: 运行 {result.testsRun} 个测试")
    print(f"成功: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"失败: {len(result.failures)}")
    print(f"错误: {len(result.errors)}")
    print("=" * 60)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)

