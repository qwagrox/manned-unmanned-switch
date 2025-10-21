# 快速入门指南

本指南将帮助您在5分钟内开始使用模式切换模块。

## 第一步：安装依赖

```bash
pip install numpy matplotlib
```

## 第二步：运行第一个示例

```bash
cd mode_switch_module
python3 examples/example_01_basic_switching.py
```

您将看到一个可视化窗口，显示拖拉机在田地中作业，并在有人/无人模式之间切换。

## 第三步：理解示例代码

打开 `examples/example_01_basic_switching.py`，核心代码如下：

```python
# 1. 创建模拟器
simulator = ScenarioSimulator(field_bounds=(0, 0, 100, 60), work_width=3.2)

# 2. 初始化
simulator.reset(start_x=5, start_y=5, start_heading=0.0)

# 3. 生成路径
path = simulator.generate_and_set_path()

# 4. 定义场景事件
events = [
    {'time': 0.0, 'action': 'switch_to_auto', 'params': {'reason': '开始无人作业'}},
    {'time': 30.0, 'action': 'switch_to_manual', 'params': {'reason': '驾驶员接管'}},
    {'time': 40.0, 'action': 'switch_to_auto', 'params': {'reason': '恢复无人作业'}},
]

# 5. 运行模拟
history = simulator.run_scenario(duration=120.0, events=events)
```

## 第四步：集成到您的系统

### 4.1 创建控制器

```python
from core import ModeSwitchController, DrivingMode

# 定义您的田地边界
field_bounds = (0, 0, 100, 60)  # (min_x, min_y, max_x, max_y)

# 创建控制器
controller = ModeSwitchController(
    field_bounds=field_bounds,
    work_width=3.2,  # 您的作业宽度
    resolution=0.5   # 覆盖地图分辨率
)

# 初始化
controller.initialize(x=10.0, y=10.0, heading=0.0)
```

### 4.2 主循环中更新传感器

```python
# 在您的主控制循环中（建议20Hz）
while True:
    # 读取传感器数据
    gps_data = your_gps.read()
    imu_data = your_imu.read()
    speed_data = your_vehicle.get_speed()
    
    # 更新控制器
    controller.update_sensors(
        gps_x=gps_data.x,
        gps_y=gps_data.y,
        gps_quality=gps_data.quality,
        imu_heading=imu_data.heading,
        imu_heading_rate=imu_data.heading_rate,
        wheel_speed=speed_data,
        timestamp=time.time()
    )
    
    # 获取系统状态
    status = controller.get_status()
    print(f"模式: {status['mode']}, 覆盖率: {status['coverage_rate']:.1%}")
    
    # 如果是无人模式，获取控制指令
    if status['mode'] == 'auto':
        target_heading, target_speed = controller.compute_control_command()
        your_vehicle.set_control(target_heading, target_speed)
    
    time.sleep(0.05)  # 20Hz
```

### 4.3 处理模式切换请求

```python
# 驾驶员按下"无人模式"按钮
if driver_pressed_auto_button:
    success = controller.request_mode_change(DrivingMode.AUTO, "驾驶员请求")
    if success:
        print("切换到无人模式")
    else:
        print("切换被拒绝，请检查条件")
        status = controller.get_status()
        print(f"建议: {status.get('recommendation', '无')}")

# 驾驶员按下"有人模式"按钮（紧急接管）
if driver_pressed_manual_button:
    controller.request_mode_change(DrivingMode.MANUAL, "驾驶员接管")
    print("切换到有人模式")
```

### 4.4 设置作业路径

```python
from simulator import PathPlannerAdapter

# 使用路径规划器生成路径
planner = PathPlannerAdapter(work_width=3.2)
path = planner.generate_coverage_path(field_bounds)

# 设置到控制器
controller.set_path(path)
```

## 第五步：测试不同场景

### 场景1：基本模式切换

运行 `example_01_basic_switching.py`，观察无人作业中多次切换到有人模式的过程。

### 场景2：路径偏离恢复

运行 `example_02_deviation_recovery.py`，观察不同偏离程度下的恢复策略：

- **轻微偏离（1米）**: 系统采用平滑对齐，车辆缓慢回到路径
- **中度偏离（5米）**: 系统采用局部重规划，生成新的对齐路径
- **严重偏离（14米）**: 系统采用全局重规划，重新规划整个作业路径

## 第六步：自定义参数

### 调整作业宽度

```python
controller = ModeSwitchController(field_bounds, work_width=4.0)
```

### 调整覆盖地图精度

```python
controller = ModeSwitchController(field_bounds, resolution=0.25)  # 更高精度
```

### 调整路径跟踪参数

```python
from core import PathTracker

tracker = PathTracker(
    work_width=3.2,
    lookahead_distance=8.0  # 增加前视距离，路径更平滑但响应更慢
)
```

### 调整偏离阈值

在 `DecisionCoordinator` 中修改偏离等级的阈值：

```python
# 在 core/decision_coordinator.py 中
def decide_recovery_strategy(self, deviation_info, vehicle_state):
    lateral_dev = abs(deviation_info.lateral_deviation)
    
    # 自定义阈值
    if lateral_dev < 0.3 * self.work_width:  # 原来是 0.5
        return RecoveryStrategy.DIRECT_RESUME
    # ...
```

## 常见问题

### Q1: 模拟器运行很慢

A: 减少可视化更新频率：

```python
# 每1秒更新一次可视化，而不是每0.5秒
if step % int(1.0 / simulator.dt) == 0:
    visualizer.update(status, coverage_grid)
```

### Q2: 切换到无人模式被拒绝

A: 检查拒绝原因：

```python
success = controller.request_mode_change(DrivingMode.AUTO, "测试")
if not success:
    status = controller.get_status()
    print(f"拒绝原因: {status.get('recommendation', '未知')}")
```

常见原因：
- 定位质量不足（<0.7）
- 速度过高（>5.0 m/s）
- 未设置路径

### Q3: 如何查看详细日志

A: 所有关键操作都会打印到控制台。如需更详细的日志，可以在各模块中添加：

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

### Q4: 如何保存模拟结果

A: 使用可视化工具保存图形：

```python
visualizer.save_figure('result.png')
```

或保存历史数据：

```python
import json

with open('history.json', 'w') as f:
    json.dump(history, f, indent=2)
```

