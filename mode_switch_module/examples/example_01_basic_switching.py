"""
示例1: 基本模式切换
演示最简单的有人/无人模式切换场景
"""

from simulator import ScenarioSimulator
from simulator.visualizer import Visualizer


def main():
    """基本模式切换示例"""
    print("=" * 60)
    print("示例1: 基本模式切换")
    print("=" * 60)
    
    # 定义田地边界（100m x 60m的矩形田地）
    field_bounds = (0, 0, 100, 60)
    work_width = 3.2
    
    # 创建模拟器
    simulator = ScenarioSimulator(field_bounds, work_width)
    
    # 重置到起始位置
    simulator.reset(start_x=5, start_y=5, start_heading=0.0)
    
    # 生成覆盖路径
    path = simulator.generate_and_set_path()
    print(f"生成路径: {len(path.points)} 个点")
    
    # 创建可视化工具
    visualizer = Visualizer(field_bounds, work_width)
    visualizer.plot_field_boundary()
    visualizer.plot_path(path)
    
    # 定义场景事件
    events = [
        # 0秒: 启动无人作业
        {'time': 0.0, 'action': 'switch_to_auto', 'params': {'reason': '开始无人作业'}},
        
        # 30秒: 切换到有人模式
        {'time': 30.0, 'action': 'switch_to_manual', 'params': {'reason': '驾驶员接管'}},
        
        # 40秒: 切换回无人模式
        {'time': 40.0, 'action': 'switch_to_auto', 'params': {'reason': '恢复无人作业'}},
        
        # 70秒: 再次切换到有人模式
        {'time': 70.0, 'action': 'switch_to_manual', 'params': {'reason': '驾驶员接管'}},
        
        # 80秒: 最后切换回无人模式
        {'time': 80.0, 'action': 'switch_to_auto', 'params': {'reason': '恢复无人作业'}},
    ]
    
    print("\n开始模拟...")
    print("场景: 无人作业过程中多次切换到有人模式，然后恢复")
    
    # 运行模拟（120秒）
    duration = 120.0
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
            
            # 更新统计信息
            stats = simulator.get_statistics()
            visualizer.update_statistics(stats)
    
    # 打印最终统计
    print("\n" + "=" * 60)
    print("模拟完成！最终统计:")
    print("=" * 60)
    stats = simulator.get_statistics()
    print(f"总距离: {stats['total_distance']:.1f} m")
    print(f"无人模式距离: {stats['auto_distance']:.1f} m ({stats['auto_distance']/stats['total_distance']*100:.1f}%)")
    print(f"有人模式距离: {stats['manual_distance']:.1f} m ({stats['manual_distance']/stats['total_distance']*100:.1f}%)")
    print(f"覆盖率: {stats['coverage_rate']*100:.1f}%")
    print(f"重复覆盖率: {stats['overlap_rate']*100:.1f}%")
    print(f"模式切换次数: {stats['mode_switches']}")
    
    # 保存图形
    visualizer.save_figure('/home/ubuntu/mode_switch_module/examples/example_01_result.png')
    
    print("\n按任意键关闭...")
    visualizer.show()


if __name__ == '__main__':
    main()

