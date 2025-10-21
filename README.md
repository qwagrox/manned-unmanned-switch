# 前装无人驾驶拖拉机模式切换系统（最先进的智能切换）

- **版本**: 1.0.0
- **作者**: [tangyong@stmail.ujs.edu.cn](mailto:tangyong@stmail.ujs.edu.cn)，目前就读于江苏大学农机控制理论与工程博士
- **日期**: 2025/10/21 

## 系统简介

本系统是一个高质量、可落地的有人/无人驾驶模式切换系统，专为前装无人驾驶拖拉机设计。系统能够智能处理驾驶员在有人和无人模式之间的任意切换，保证作业的连续性、完整性和安全性。

### 核心特性

- **状态感知**: 实时融合GPS/RTK、IMU、轮速计等传感器数据，提供高精度车辆状态估计
- **智能决策**: 基于规则的决策引擎，自动评估偏离程度并选择最优恢复策略
- **覆盖管理**: 高精度覆盖地图（不单单是高精地图）实时跟踪作业进度，避免重复和遗漏
- **路径跟踪**: Pure Pursuit控制算法（丘沃智能无人驾驶控制模块）实现平滑的路径跟踪
- **模式切换**: 完善的模式切换协议，确保切换过程的安全性和平滑性
- **可视化**: 实时可视化工具（集成到驾驶舱的天际屏），直观展示车辆状态、路径、覆盖情况，包括所有需要展示给机手这台无人拖拉机的数据

## 系统架构

```
mode_switch_module/
├── core/                      # 核心功能模块
│   ├── data_structures.py     # 数据结构定义
│   ├── state_estimator.py     # 状态估计器
│   ├── mode_manager.py         # 模式管理器
│   ├── coverage_manager.py     # 覆盖管理器
│   ├── path_tracker.py         # 路径跟踪器
│   ├── decision_coordinator.py # 决策协调器
│   └── mode_switch_controller.py # 主控制器
├── simulator/                  # 模拟器模块
│   ├── sensor_simulator.py     # 传感器模拟器
│   ├── vehicle_simulator.py    # 车辆模拟器
│   ├── path_planner_adapter.py # 路径规划器适配器
│   ├── scenario_simulator.py   # 场景模拟器
│   └── visualizer.py           # 可视化工具
├── examples/                   # 示例程序
│   ├── example_01_basic_switching.py      # 基本模式切换
│   └── example_02_deviation_recovery.py   # 路径偏离恢复
├── tests/                      # 单元测试
│   └── test_core_modules.py    # 核心模块测试
└── docs/                       # 文档目录
```

## 快速开始

### 环境要求

- Python 3.8+
- NumPy
- Matplotlib (临时用于可视化)

### 安装依赖

```bash
pip install numpy matplotlib
```

### 基本使用

```python
from core import ModeSwitchController, DrivingMode

# 定义田地边界 (min_x, min_y, max_x, max_y)
field_bounds = (0, 0, 100, 60)

# 创建控制器
controller = ModeSwitchController(field_bounds, work_width=3.2)

# 初始化系统
controller.initialize(x=10.0, y=10.0, heading=0.0)

# 更新传感器数据
controller.update_sensors(
    gps_x=10.0, gps_y=10.0, gps_quality=0.95,
    imu_heading=0.0, imu_heading_rate=0.0,
    wheel_speed=2.0, timestamp=1.0
)

# 请求模式切换（真实场景是驾驶舱的类似手扶箱按钮切换）
success = controller.request_mode_change(DrivingMode.AUTO, "开始无人作业")

# 获取控制指令（无人模式下）
target_heading, target_speed = controller.compute_control_command()

# 获取系统状态
status = controller.get_status()
print(f"当前模式: {status['mode']}")
print(f"覆盖率: {status['coverage_rate']:.1%}")
```

### 运行示例

#### 示例1: 基本模式切换

```bash
cd mode_switch_module
python3 examples/example_01_basic_switching.py
```

该示例演示无人作业过程中多次切换到有人模式，然后恢复的场景。

#### 示例2: 路径偏离恢复

```bash
python3 examples/example_02_deviation_recovery.py
```

该示例演示驾驶员在有人模式下偏离路径不同程度，系统采用不同恢复策略的场景。

### 运行测试

```bash
python3 tests/test_core_modules.py
```

所有16个单元测试应该全部通过。

## 核心模块详解

### 1. 状态估计器 (StateEstimator)

融合多传感器数据，提供高精度车辆状态估计。

**主要功能**:
- 卡尔曼滤波融合GPS、IMU、轮速计数据
- 实时输出位置、航向、速度和定位质量
- 处理传感器噪声和短暂信号丢失

### 2. 模式管理器 (ModeManager)

管理驾驶模式和模式切换。

**主要功能**:
- 实现状态机，管理5种工作模式（有人、无人、过渡、暂停、错误）
- 切换条件检查（定位质量、速度、加速度等）
- 切换历史记录和统计

### 3. 覆盖管理器 (CoverageManager)

维护高精度覆盖地图。

**主要功能**:
- 0.5米分辨率的实时覆盖地图
- 区分有人覆盖、无人覆盖和重复覆盖
- 覆盖率和重复率统计
- 查找未覆盖和重复覆盖区域

### 4. 路径跟踪器 (PathTracker)

路径匹配和偏离检测。

**主要功能**:
- 快速路径匹配算法
- 偏离等级分类（极小/轻微/中度/严重）
- Pure Pursuit控制算法
- 路径执行进度跟踪

### 5. 决策协调器 (DecisionCoordinator)

综合各模块信息，做出智能决策。

**主要功能**:
- 恢复策略决策（直接恢复/平滑对齐/局部重规划/全局重规划）
- 模式切换安全性评估
- 操作建议生成
- 未覆盖区域优先级计算

### 6. 主控制器 (ModeSwitchController)

整合所有模块，提供统一接口。

**主要功能**:
- 协调各模块工作
- 传感器数据更新
- 模式切换请求处理
- 控制指令计算
- 统计信息收集

## 模拟器使用

### 场景模拟器

```python
from simulator import ScenarioSimulator
from simulator.visualizer import Visualizer

# 创建模拟器
simulator = ScenarioSimulator(field_bounds=(0, 0, 100, 60), work_width=3.2)
simulator.reset(start_x=5, start_y=5, start_heading=0.0)

# 生成路径
path = simulator.generate_and_set_path()

# 定义场景事件
events = [
    {'time': 0.0, 'action': 'switch_to_auto', 'params': {'reason': '开始无人作业'}},
    {'time': 30.0, 'action': 'switch_to_manual', 'params': {'reason': '驾驶员接管'}},
    {'time': 40.0, 'action': 'switch_to_auto', 'params': {'reason': '恢复无人作业'}},
]

# 运行模拟
history = simulator.run_scenario(duration=120.0, events=events)

# 获取统计
stats = simulator.get_statistics()
print(f"覆盖率: {stats['coverage_rate']:.1%}")
```

### 可视化

```python
# 创建可视化工具
visualizer = Visualizer(field_bounds, work_width)
visualizer.plot_field_boundary()
visualizer.plot_path(path)

# 在模拟循环中更新
for status in history:
    coverage_grid = simulator.get_coverage_grid()
    visualizer.update(status, coverage_grid)
    visualizer.update_statistics(stats)

# 保存图形
visualizer.save_figure('result.png')
visualizer.show()
```

## 与现有路径规划器集成

本模块通过`PathPlannerAdapter`与`multi_layer_planner_v3.py`集成。

```python
from simulator import PathPlannerAdapter

# 创建适配器
planner = PathPlannerAdapter(work_width=3.2)

# 生成覆盖路径
path = planner.generate_coverage_path(field_bounds)

# 生成对齐路径
alignment_path = planner.generate_alignment_path(
    current_x=10, current_y=10, current_heading=0,
    target_x=20, target_y=10, target_heading=0
)

# 生成接近路径
approach_path = planner.generate_approach_path(
    current_x=5, current_y=5,
    target_x=10, target_y=10, target_heading=0
)
```

## 关键场景处理

### 场景1: 路径内切换

驾驶员在无人作业中切换到有人模式，手动驾驶后再切回无人模式。

**处理策略**:
- 极小偏离(<0.5个作业宽度): 直接恢复
- 轻微偏离(0.5-1.5个作业宽度): 平滑对齐
- 中度偏离(1.5-3.0个作业宽度): 局部重规划
- 严重偏离(>3.0个作业宽度): 全局重规划

### 场景2: 作业开始

驾驶员在田地任意位置启动无人作业。

**处理策略**:
- 评估当前位置是否适合作为起点
- 如不适合，搜索最优起点
- 规划接近路径
- 显示完整路径供驾驶员确认

### 场景3: 紧急接管

驾驶员因紧急情况主动接管控制权。

**处理策略**:
- 快速检测接管信号(<100ms)
- 立即停止控制指令
- 保存作业状态
- 切换到有人模式并记录轨迹

### 场景4: 覆盖冲突

有人模式下的作业与无人模式规划的路径产生冲突。

**处理策略**:
- 实时检测重复覆盖和遗漏覆盖
- 动态调整路径，跳过已覆盖区域
- 作业结束前规划补偿路径
- 确保覆盖完整性

## 性能指标

- **定位精度**: 厘米级（RTK-GPS）
- **更新频率**: 20Hz
- **覆盖地图分辨率**: 0.5米
- **路径匹配延迟**: <10ms
- **模式切换延迟**: <2秒
- **覆盖精度**: >95%
- **重复覆盖率**: <10%

## 扩展和定制

### 添加新的传感器

在`StateEstimator`中添加新的更新方法:

```python
def update_custom_sensor(self, measurement):
    # 实现传感器融合逻辑
    pass
```

### 自定义决策规则

在`DecisionCoordinator`中添加新的决策规则:

```python
def custom_decision_rule(self, vehicle_state, context):
    # 实现自定义决策逻辑
    return decision
```

### 集成实际硬件

替换`SensorSimulator`和`VehicleSimulator`为实际硬件接口:

```python
# 替代模拟器
from your_hardware import GPSInterface, IMUInterface, VehicleInterface

# 使用实际传感器数据
gps_data = gps_interface.read()
imu_data = imu_interface.read()

controller.update_sensors(
    gps_x=gps_data.x,
    gps_y=gps_data.y,
    gps_quality=gps_data.quality,
    imu_heading=imu_data.heading,
    imu_heading_rate=imu_data.heading_rate,
    wheel_speed=vehicle_interface.get_speed(),
    timestamp=time.time()
)
```

## 常见问题

### Q: 如何调整作业宽度？

A: 在创建控制器时指定`work_width`参数:

```python
controller = ModeSwitchController(field_bounds, work_width=4.0)
```

### Q: 如何提高覆盖地图精度？

A: 减小`resolution`参数（但会增加内存使用）:

```python
controller = ModeSwitchController(field_bounds, work_width=3.2, resolution=0.25)
```

### Q: 如何处理GPS信号丢失？

A: 状态估计器会自动使用IMU和轮速计进行短时预测。长时间丢失会触发定位质量下降，系统会拒绝切换到无人模式。（相比纯无人，这是优势）

### Q: 如何集成到ROS系统？

A: 创建ROS节点封装控制器，订阅传感器话题，发布控制指令:

```python
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist

class ModeSwitchNode:
    def __init__(self):
        self.controller = ModeSwitchController(...)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
```