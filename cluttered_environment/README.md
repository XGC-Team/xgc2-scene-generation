# Cluttered Environment Generator / 杂乱环境生成器

ROS package for generating simulation scenarios with static and dynamic convex obstacles.

用于生成包含静态和动态凸障碍物的仿真场景的ROS功能包。

## Features / 功能特性

- **Multiple Geometry Types** / **多种几何体类型**
  - Sphere / 球体
  - Cylinder / 圆柱
  - Cube / 立方体
  - H-Polytope / H凸包

- **Velocity Obstacles (VO)** / **速度障碍物**
  - Subscribe to ROS topics for dynamic velocity control
  - 订阅ROS话题进行动态速度控制

- **TF-based Coordinate Systems** / **基于TF的坐标系**
  - Each obstacle has its own frame
  - 每个障碍物拥有独立坐标系

- **Scale Support** / **缩放支持**
  - Independent scaling along X, Y, Z axes in local frame
  - 在局部坐标系中沿X、Y、Z轴独立缩放

- **H-Polytope Library** / **H凸包库**
  - Reusable template definitions for complex geometries
  - 可复用的复杂几何体模板定义

## Installation / 安装

```bash
cd ~/catkin_ws/src/ros_ws
# (Copy this package here / 将此包复制到这里)
cd ~/catkin_ws
catkin build cluttered_environment
source devel/setup.bash
```

## Quick Start / 快速开始

### 1. Launch with default scenario / 使用默认场景启动
```bash
roslaunch cluttered_environment scenario.launch
```

### 2. Launch with custom scenario / 使用自定义场景启动
```bash
roslaunch cluttered_environment scenario.launch config_file:=/path/to/your/scenario.yaml
```

### 3. Launch simple test scenario / 启动简单测试场景
```bash
roslaunch cluttered_environment scenario_simple.launch
```

## Configuration / 配置说明

See `config/scenario_default.yaml` for a complete example.

参见 `config/scenario_default.yaml` 获取完整示例。

### Obstacle Scale / 障碍物缩放

The `scale` parameter applies scaling in the obstacle's **local coordinate system**:

`scale` 参数在障碍物的**局部坐标系**中应用缩放：

```yaml
pose:
  position: [0, 0, 1]
  orientation: [0, 0, 0, 1]
  scale: [2.0, 1.0, 0.5]  # X: 2x, Y: 1x, Z: 0.5x
```

**Examples / 示例：**
- Sphere with `scale: [2.0, 1.0, 0.5]` → Ellipsoid (椭球体)
- Cube with `scale: [1.0, 1.0, 2.0]` → Tall cuboid (高立方体)
- Cylinder with `scale: [2.0, 2.0, 1.0]` → Wide short cylinder (宽扁圆柱)

### Velocity Obstacles / 速度障碍物

Enable velocity control from external topics:

启用外部话题的速度控制：

```yaml
velocity_obstacle:
  enabled: true
  topic: "/tracked_objects/1/twist"
```

The obstacle will subscribe to `geometry_msgs/Twist` messages. Initial velocity is zero until the first message is received.

障碍物将订阅 `geometry_msgs/Twist` 消息。初始速度为零，直到接收到第一条消息。

### H-Polytope Templates / H凸包模板

Reuse predefined templates from the library:

复用库中的预定义模板：

```yaml
geometry:
  template: "cube_2m"  # Reference library template / 引用库模板
```

Or define inline:

或内联定义：

```yaml
geometry:
  h_matrix:
    - [1.0, 0.0, 0.0, -1.0]
    - [-1.0, 0.0, 0.0, -1.0]
    # ... more half-planes
  epsilon: 1.0e-6
```

## Package Status / 功能包状态

**Current Version / 当前版本**: v0.1.0 (Basic Framework / 基础框架)

- [x] Package structure / 包结构
- [x] YAML configuration loading / YAML配置加载
- [x] Launch files / 启动文件
- [x] Scale support / 缩放支持
- [ ] Obstacle geometry generation / 障碍物几何体生成
- [ ] TF broadcasting / TF广播
- [ ] RViz visualization / RViz可视化
- [ ] Velocity obstacle subscription / 速度障碍物订阅

## Configuration Files / 配置文件

- `config/scenario_default.yaml` - Complete example with all obstacle types
  完整示例，包含所有障碍物类型

- `config/scenario_simple.yaml` - Minimal setup for testing
  简单测试配置

- `config/h_polytope_library.yaml` - Reusable H-polytope templates
  可复用的H凸包模板库

## Dependencies / 依赖

- ROS (tested on Melodic/Noetic)
- Eigen3
- yaml-cpp
- tf2
- Boost

## Example Output / 示例输出

After launching, you should see:

启动后，您应该看到：

```
=== Scenario Configuration ===
World frame: world
Publish rate: 10.0 Hz
TF publish rate: 50.0 Hz

=== Obstacles (7 total) ===

[Obstacle 0: static_sphere_1]
  Type: sphere, Motion: static
  Frame: world -> obstacle_0
  Position: [2.00, 3.00, 1.00]
  Orientation (XYZW): [0.000, 0.000, 0.000, 1.000]
  Scale (XYZ): [1.00, 1.00, 1.00]
  Geometry:
    - Radius: 0.50 m
    - Resolution: 20
  Velocity Obstacle: NO
...
```

## Troubleshooting / 故障排除

### Cannot find package / 找不到包

Make sure you have sourced the workspace:

确保您已加载工作空间：

```bash
source ~/catkin_ws/devel/setup.bash
```

### YAML parsing error / YAML解析错误

Check your YAML file syntax, especially:

检查YAML文件语法，特别是：

- Indentation (use spaces, not tabs) / 缩进（使用空格，不要使用制表符）
- List format: `[x, y, z]` / 列表格式
- Boolean values: `true` or `false` / 布尔值

### Package path not resolved / 包路径无法解析

Make sure the package is built:

确保包已构建：

```bash
catkin build cluttered_environment
```

## Future Work / 未来工作

- [ ] Implement obstacle mesh generation / 实现障碍物网格生成
- [ ] Add TF frame broadcasting / 添加TF坐标系广播
- [ ] Implement RViz visualization / 实现RViz可视化
- [ ] Add velocity obstacle subscribers / 添加速度障碍物订阅器
- [ ] Support for more geometry types / 支持更多几何体类型
- [ ] Interactive obstacle manipulation / 交互式障碍物操作

## License / 许可证

MIT

## Author / 作者

DMPC Lab

## Version History / 版本历史

- **v0.1.0** (2025) - Initial framework with YAML loading
  初始框架，支持YAML加载
