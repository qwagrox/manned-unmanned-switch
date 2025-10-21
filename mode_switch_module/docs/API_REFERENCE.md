# API 参考文档

## 核心模块 API

### ModeSwitchController

主控制器类，整合所有核心功能模块。

#### 构造函数

```python
ModeSwitchController(field_bounds, work_width=3.2, resolution=0.5)
```

**参数**:
- `field_bounds` (tuple): 田地边界 `(min_x, min_y, max_x, max_y)`
- `work_width` (float): 作业宽度（米），默认3.2
- `resolution` (float): 覆盖地图分辨率（米），默认0.5

#### 方法

##### initialize

```python
initialize(x, y, heading)
```

初始化系统状态。

**参数**:
- `x` (float): 初始X坐标（米）
- `y` (float): 初始Y坐标（米）
- `heading` (float): 初始航向角（弧度）

##### update_sensors

```python
update_sensors(gps_x, gps_y, gps_quality, imu_heading, imu_heading_rate, wheel_speed, timestamp)
```

更新传感器数据。

**参数**:
- `gps_x` (float): GPS X坐标（米）
- `gps_y` (float): GPS Y坐标（米）
- `gps_quality` (float): GPS质量 (0-1)
- `imu_heading` (float): IMU航向角（弧度）
- `imu_heading_rate` (float): IMU航向角速度（弧度/秒）
- `wheel_speed` (float): 轮速（米/秒）
- `timestamp` (float): 时间戳（秒）

##### set_path

```python
set_path(path)
```

设置作业路径。

**参数**:
- `path` (Path): 路径对象

##### request_mode_change

```python
request_mode_change(target_mode, reason="") -> bool
```

请求模式切换。

**参数**:
- `target_mode` (DrivingMode): 目标模式
- `reason` (str): 切换原因

**返回**:
- `bool`: 是否接受切换请求

##### compute_control_command

```python
compute_control_command() -> Tuple[float, float]
```

计算控制指令（仅在无人模式下有效）。

**返回**:
- `tuple`: (目标航向角, 目标速度)

##### get_status

```python
get_status() -> Dict[str, Any]
```

获取系统状态。

**返回**:
- `dict`: 状态字典，包含以下键:
  - `timestamp`: 时间戳
  - `mode`: 当前模式
  - `position`: (x, y) 位置
  - `heading`: 航向角
  - `speed`: 速度
  - `positioning_quality`: 定位质量
  - `coverage_rate`: 覆盖率
  - `overlap_rate`: 重复覆盖率
  - `path_progress`: 路径进度
  - `deviation`: 偏离信息（如果有）
  - `recommendation`: 操作建议

##### get_statistics

```python
get_statistics() -> Dict[str, Any]
```

获取作业统计信息。

**返回**:
- `dict`: 统计信息字典

##### is_work_completed

```python
is_work_completed() -> bool
```

判断作业是否完成。

**返回**:
- `bool`: 是否完成（覆盖率>95%）

---

### StateEstimator

状态估计器类，融合多传感器数据。

#### 构造函数

```python
StateEstimator()
```

#### 方法

##### reset

```python
reset(x, y, heading)
```

重置状态估计器。

**参数**:
- `x` (float): 初始X坐标
- `y` (float): 初始Y坐标
- `heading` (float): 初始航向角

##### predict

```python
predict(dt)
```

预测步骤。

**参数**:
- `dt` (float): 时间间隔（秒）

##### update_gps

```python
update_gps(x, y, quality=1.0)
```

更新GPS观测。

**参数**:
- `x` (float): GPS X坐标
- `y` (float): GPS Y坐标
- `quality` (float): GPS质量 (0-1)

##### update_imu

```python
update_imu(heading, heading_rate)
```

更新IMU观测。

**参数**:
- `heading` (float): 航向角（弧度）
- `heading_rate` (float): 航向角速度（弧度/秒）

##### update_speed

```python
update_speed(speed)
```

更新速度观测。

**参数**:
- `speed` (float): 速度（米/秒）

##### get_vehicle_state

```python
get_vehicle_state(timestamp, mode) -> VehicleState
```

获取当前车辆状态。

**参数**:
- `timestamp` (float): 时间戳
- `mode` (DrivingMode): 当前驾驶模式

**返回**:
- `VehicleState`: 车辆状态对象

---

### ModeManager

模式管理器类，管理驾驶模式和模式切换。

#### 构造函数

```python
ModeManager()
```

#### 方法

##### request_mode_change

```python
request_mode_change(target_mode, vehicle_state, reason="") -> bool
```

请求模式切换。

**参数**:
- `target_mode` (DrivingMode): 目标模式
- `vehicle_state` (VehicleState): 当前车辆状态
- `reason` (str): 切换原因

**返回**:
- `bool`: 是否接受切换请求

##### complete_mode_change

```python
complete_mode_change(target_mode, vehicle_state) -> bool
```

完成模式切换。

**参数**:
- `target_mode` (DrivingMode): 目标模式
- `vehicle_state` (VehicleState): 当前车辆状态

**返回**:
- `bool`: 是否成功完成切换

##### force_mode_change

```python
force_mode_change(target_mode, vehicle_state, reason="强制切换")
```

强制模式切换（用于紧急情况）。

**参数**:
- `target_mode` (DrivingMode): 目标模式
- `vehicle_state` (VehicleState): 当前车辆状态
- `reason` (str): 切换原因

##### register_callback

```python
register_callback(callback)
```

注册模式切换回调函数。

**参数**:
- `callback` (Callable): 回调函数，签名为 `callback(from_mode, to_mode, vehicle_state)`

##### get_mode

```python
get_mode() -> DrivingMode
```

获取当前模式。

**返回**:
- `DrivingMode`: 当前模式

##### get_transition_history

```python
get_transition_history() -> List[ModeTransitionEvent]
```

获取切换历史。

**返回**:
- `list`: 切换事件列表

---

### CoverageManager

覆盖管理器类，维护高精度覆盖地图。

#### 构造函数

```python
CoverageManager(field_bounds, resolution=0.5, work_width=3.2)
```

**参数**:
- `field_bounds` (tuple): 田地边界 `(min_x, min_y, max_x, max_y)`
- `resolution` (float): 网格分辨率（米）
- `work_width` (float): 作业宽度（米）

#### 方法

##### update

```python
update(vehicle_state)
```

更新覆盖地图。

**参数**:
- `vehicle_state` (VehicleState): 当前车辆状态

##### get_coverage_rate

```python
get_coverage_rate() -> float
```

获取覆盖率。

**返回**:
- `float`: 覆盖率 (0-1)

##### get_overlap_rate

```python
get_overlap_rate() -> float
```

获取重复覆盖率。

**返回**:
- `float`: 重复覆盖率 (0-1)

##### is_covered

```python
is_covered(x, y) -> bool
```

检查指定位置是否已覆盖。

**参数**:
- `x` (float): X坐标
- `y` (float): Y坐标

**返回**:
- `bool`: 是否已覆盖

##### find_uncovered_regions

```python
find_uncovered_regions() -> List[Tuple[float, float, float]]
```

查找未覆盖区域。

**返回**:
- `list`: 未覆盖区域列表，每个元素为 `(center_x, center_y, area)`

##### find_nearest_uncovered

```python
find_nearest_uncovered(x, y) -> Optional[Tuple[float, float]]
```

查找最近的未覆盖点。

**参数**:
- `x` (float): 起始X坐标
- `y` (float): 起始Y坐标

**返回**:
- `tuple` or `None`: 最近未覆盖点的坐标

---

### PathTracker

路径跟踪器类，实现路径匹配和控制。

#### 构造函数

```python
PathTracker(work_width=3.2, lookahead_distance=5.0)
```

**参数**:
- `work_width` (float): 作业宽度（米）
- `lookahead_distance` (float): 前视距离（米）

#### 方法

##### set_path

```python
set_path(path)
```

设置当前跟踪路径。

**参数**:
- `path` (Path): 路径对象

##### update

```python
update(vehicle_state) -> Optional[DeviationInfo]
```

更新路径跟踪状态。

**参数**:
- `vehicle_state` (VehicleState): 当前车辆状态

**返回**:
- `DeviationInfo` or `None`: 偏离信息

##### compute_control

```python
compute_control(vehicle_state) -> Tuple[float, float]
```

计算控制指令（Pure Pursuit算法）。

**参数**:
- `vehicle_state` (VehicleState): 车辆状态

**返回**:
- `tuple`: (目标航向角, 目标速度)

##### get_progress

```python
get_progress() -> float
```

获取路径执行进度。

**返回**:
- `float`: 进度百分比 (0-1)

##### is_path_completed

```python
is_path_completed(vehicle_state, threshold=2.0) -> bool
```

判断路径是否完成。

**参数**:
- `vehicle_state` (VehicleState): 车辆状态
- `threshold` (float): 距离阈值（米）

**返回**:
- `bool`: 是否完成

---

### DecisionCoordinator

决策协调器类，实现智能决策。

#### 构造函数

```python
DecisionCoordinator(work_width=3.2)
```

**参数**:
- `work_width` (float): 作业宽度（米）

#### 方法

##### decide_recovery_strategy

```python
decide_recovery_strategy(deviation_info, vehicle_state) -> RecoveryStrategy
```

决定恢复策略。

**参数**:
- `deviation_info` (DeviationInfo): 偏离信息
- `vehicle_state` (VehicleState): 车辆状态

**返回**:
- `RecoveryStrategy`: 恢复策略

##### should_trigger_replan

```python
should_trigger_replan(deviation_info, vehicle_state, coverage_rate) -> bool
```

判断是否应该触发重规划。

**参数**:
- `deviation_info` (DeviationInfo): 偏离信息
- `vehicle_state` (VehicleState): 车辆状态
- `coverage_rate` (float): 当前覆盖率

**返回**:
- `bool`: 是否触发重规划

##### evaluate_mode_switch_safety

```python
evaluate_mode_switch_safety(vehicle_state, target_mode) -> Tuple[bool, str]
```

评估模式切换的安全性。

**参数**:
- `vehicle_state` (VehicleState): 车辆状态
- `target_mode` (DrivingMode): 目标模式

**返回**:
- `tuple`: (是否安全, 原因说明)

##### recommend_action

```python
recommend_action(vehicle_state, deviation_info, coverage_rate) -> str
```

推荐操作建议。

**参数**:
- `vehicle_state` (VehicleState): 车辆状态
- `deviation_info` (DeviationInfo): 偏离信息
- `coverage_rate` (float): 覆盖率

**返回**:
- `str`: 操作建议文本

---

## 数据结构

### DrivingMode (枚举)

驾驶模式枚举。

**值**:
- `MANUAL`: 有人驾驶
- `AUTO`: 无人驾驶
- `TRANSITION`: 过渡状态
- `PAUSED`: 暂停
- `ERROR`: 错误状态

### DeviationLevel (枚举)

偏离等级枚举。

**值**:
- `MINIMAL`: 极小偏离 (<0.5个作业宽度)
- `SLIGHT`: 轻微偏离 (0.5-1.5个作业宽度)
- `MODERATE`: 中度偏离 (1.5-3.0个作业宽度)
- `SEVERE`: 严重偏离 (>3.0个作业宽度)

### RecoveryStrategy (枚举)

恢复策略枚举。

**值**:
- `DIRECT_RESUME`: 直接恢复
- `SMOOTH_ALIGNMENT`: 平滑对齐
- `LOCAL_REPLAN`: 局部重规划
- `GLOBAL_REPLAN`: 全局重规划

### VehicleState (数据类)

车辆状态。

**字段**:
- `timestamp` (float): 时间戳（秒）
- `x` (float): X坐标（米）
- `y` (float): Y坐标（米）
- `heading` (float): 航向角（弧度）
- `speed` (float): 速度（米/秒）
- `mode` (DrivingMode): 当前驾驶模式
- `positioning_quality` (float): 定位质量 (0-1)
- `lateral_acceleration` (float): 侧向加速度（米/秒²）
- `longitudinal_acceleration` (float): 纵向加速度（米/秒²）

### PathPoint (数据类)

路径点。

**字段**:
- `x` (float): X坐标（米）
- `y` (float): Y坐标（米）
- `heading` (float): 期望航向角（弧度）
- `speed` (float): 期望速度（米/秒）
- `curvature` (float): 曲率（1/米）
- `segment_type` (str): 路径段类型

### Path (数据类)

路径。

**字段**:
- `points` (List[PathPoint]): 路径点列表
- `path_id` (str): 路径ID
- `path_type` (str): 路径类型
- `total_length` (float): 总长度（米）
- `created_at` (float): 创建时间戳

### DeviationInfo (数据类)

偏离信息。

**字段**:
- `lateral_deviation` (float): 横向偏差（米）
- `heading_deviation` (float): 航向偏差（弧度）
- `speed_deviation` (float): 速度偏差（米/秒）
- `deviation_level` (DeviationLevel): 偏离等级
- `closest_path_index` (int): 最近路径点索引
- `distance_along_path` (float): 沿路径的距离（米）

---

## 模拟器 API

### ScenarioSimulator

场景模拟器类。

#### 构造函数

```python
ScenarioSimulator(field_bounds, work_width=3.2)
```

**参数**:
- `field_bounds` (tuple): 田地边界
- `work_width` (float): 作业宽度

#### 方法

##### reset

```python
reset(start_x=None, start_y=None, start_heading=0.0)
```

重置模拟器。

##### generate_and_set_path

```python
generate_and_set_path() -> Path
```

生成并设置作业路径。

##### step

```python
step(manual_control=None) -> Dict[str, Any]
```

执行一个模拟步骤。

**参数**:
- `manual_control` (dict): 手动控制输入

**返回**:
- `dict`: 当前状态信息

##### run_scenario

```python
run_scenario(duration, events=None) -> List[Dict[str, Any]]
```

运行完整场景。

**参数**:
- `duration` (float): 模拟时长（秒）
- `events` (list): 场景事件列表

**返回**:
- `list`: 状态历史列表

---

### Visualizer

可视化工具类。

#### 构造函数

```python
Visualizer(field_bounds, work_width=3.2)
```

#### 方法

##### plot_field_boundary

```python
plot_field_boundary()
```

绘制田地边界。

##### plot_path

```python
plot_path(path)
```

绘制路径。

##### update

```python
update(status, coverage_grid=None, coverage_resolution=0.5, coverage_origin=(0, 0))
```

更新所有可视化。

##### save_figure

```python
save_figure(filename)
```

保存图形。

##### show

```python
show()
```

显示图形。

