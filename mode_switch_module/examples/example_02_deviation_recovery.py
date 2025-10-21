"""
示例2: 路径偏离恢复
演示驾驶员在有人模式下偏离路径，然后恢复无人模式的场景
"""

from simulator import ScenarioSimulator
from simulator.visualizer import Visualizer


def main():
    """路径偏离恢复示例"""
    print("=" * 60)
    print("示例2: 路径偏离恢复")
    print("=" * 60)
    
    # 定义田地边界
    field_bounds = (0, 0, 100, 60)
    work_width = 3.2
    
    # 创建模拟器
    simulator = ScenarioSimulator(field_bounds, work_width)
    simulator.reset(start_x=5, start_y=5, start_heading=0.0)
    
    # 生成路径
    path = simulator.generate_and_set_path()
    print(f"生成路径: {len(path.points)} 个点")
    
    # 创建可视化
    visualizer = Visualizer(field_bounds, work_width)
    visualizer.plot_field_boundary()
    visualizer.plot_path(path)
    
    # 定义场景事件：模拟不同程度的偏离
    events = [
        # 启动无人作业
        {'time': 0.0, 'action': 'switch_to_auto', 'params': {'reason': '开始无人作业'}},
        
        # 场景1: 轻微偏离（15秒）
        {'time': 15.0, 'action': 'switch_to_manual', 'params': {'reason': '测试轻微偏离'}},
        {'time': 15.5, 'action': 'manual_deviation', 'params': {'deviation_x': 0.0, 'deviation_y': 1.0}},  # 偏离1米
        {'time': 20.0, 'action': 'switch_to_auto', 'params': {'reason': '恢复无人作业'}},
        
        # 场景2: 中度偏离（40秒）
        {'time': 40.0, 'action': 'switch_to_manual', 'params': {'reason': '测试中度偏离'}},
        {'time': 40.5, 'action': 'manual_deviation', 'params': {'deviation_x': 0.0, 'deviation_y': 5.0}},  # 偏离5米
        {'time': 45.0, 'action': 'switch_to_auto', 'params': {'reason': '恢复无人作业'}},
        
        # 场景3: 严重偏离（65秒）
        {'time': 65.0, 'action': 'switch_to_manual', 'params': {'reason': '测试严重偏离'}},
        {'time': 65.5, 'action': 'manual_deviation', 'params': {'deviation_x': 10.0, 'deviation_y': 10.0}},  # 偏离14米
        {'time': 70.0, 'action': 'switch_to_auto', 'params': {'reason': '恢复无人作业'}},
    ]
    
    print("\n开始模拟...")
    print("场景: 测试不同程度的路径偏离和恢复策略")
    print("  - 15s: 轻微偏离 (1m)")
    print("  - 40s: 中度偏离 (5m)")
    print("  - 65s: 严重偏离 (14m)")
    
    # 运行模拟
    duration = 100.0
    history = []
    
    num_steps = int(duration / simulator.dt)
    event_index = 0
    events = sorted(events, key=lambda e: e['time'])
    
    for step in range(num_steps):
        # 处理事件
        while event_index < len(events) and events[event_index]['time'] <= simulator.current_time:
            simulator._handle_event(events[event_index])
            event_index += 1
        
        # 执行一步
        status = simulator.step()
        history.append(status)
        
        # 每0.5秒更新一次可视化
        if step % int(0.5 / simulator.dt) == 0:
            coverage_grid = simulator.get_coverage_grid()
            visualizer.update(
                status, 
                coverage_grid, 
                simulator.controller.coverage_manager.coverage_map.resolution,
                (simulator.controller.coverage_manager.coverage_map.origin_x,
                 simulator.controller.coverage_manager.coverage_map.origin_y)
            )
            
            stats = simulator.get_statistics()
            visualizer.update_statistics(stats)
            
        # 打印偏离信息
        if 'deviation' in status and step % int(1.0 / simulator.dt) == 0:
            deviation = status['deviation']
            if abs(deviation['lateral']) > 0.5:
                print(f"[{simulator.current_time:.1f}s] 检测到偏离: {deviation['lateral']:.2f}m, "
                      f"等级: {deviation['level']}")
    
    # 打印最终统计
    print("\n" + "=" * 60)
    print("模拟完成！最终统计:")
    print("=" * 60)
    stats = simulator.get_statistics()
    print(f"总距离: {stats['total_distance']:.1f} m")
    print(f"覆盖率: {stats['coverage_rate']*100:.1f}%")
    print(f"重复覆盖率: {stats['overlap_rate']*100:.1f}%")
    print(f"模式切换次数: {stats['mode_switches']}")
    
    # 保存图形
    visualizer.save_figure('/home/ubuntu/mode_switch_module/examples/example_02_result.png')
    
    print("\n按任意键关闭...")
    visualizer.show()


if __name__ == '__main__':
    main()

