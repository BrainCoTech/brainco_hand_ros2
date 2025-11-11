<div align="right">

[简体中文](README_CN.md)|[English](README.md)

</div>

<div align="center">

# BrainCo Gazebo Revo2 灵巧手仿真演示包

</div>

## 功能包概述

brainco_gazebo 功能包提供了 BrainCo Revo2 灵巧手的完整 Gazebo 仿真环境。基于 ROS2 和 Ignition Gazebo 6 构建，为灵巧手操作研究和开发提供专业级的仿真体验。

### 主要特性

- **灵巧手仿真**: BrainCo Revo2 灵巧手（左右手）的完整仿真
- **Gazebo物理仿真**: 基于 Ignition Gazebo 6 的高保真物理仿真
- **ROS2控制集成**: 完整的 ros2_control 支持，包含轨迹控制器
- **MoveIt集成**: 完整的 Gazebo-MoveIt 整合，支持运动规划与执行
- **RViz可视化**: 内置启动文件，支持 RViz 可视化
- **多手型支持**: 支持左右手配置切换

## 系统环境

- Ubuntu 22.04
- ROS 2 (Humble)
- Ignition Gazebo 6

## 安装配置

### 1. 构建工作空间

```bash
# 创建工作空间（如果不存在）
mkdir -p ~/brainco_ws/src
cd ~/brainco_ws/src

# 克隆或复制功能包
# 确保 revo2_description 可用
# 将 brainco_gazebo 复制到 src/

# 依赖安装
rosdep install --ignore-src --from-paths . -y -r

# 构建工作空间
cd ~/brainco_ws
colcon build --packages-select revo2_description brainco_gazebo brainco_moveit_config --symlink-install
source install/setup.bash
```

### 2. 验证安装

```bash
# 检查功能包是否可用
ros2 pkg list | grep -E "(revo2_description|brainco_gazebo|brainco_moveit_config)"

```

## 快速开始

### 基础启动

```bash
# 加载工作空间
source ~/brainco_ws/install/setup.bash

# 启动单手仿真
ros2 launch brainco_gazebo revo2_hand_gazebo.launch.py hand_type:=left
ros2 launch brainco_gazebo revo2_hand_gazebo.launch.py hand_type:=right

# 启动双手仿真
ros2 launch brainco_gazebo dual_revo2_hand_gazebo.launch.py
```

### 参数化启动

```bash
# 不启动RViz（提升性能）
ros2 launch brainco_gazebo revo2_hand_gazebo.launch.py use_rviz:=false

# 使用自定义世界文件
ros2 launch brainco_gazebo revo2_hand_gazebo.launch.py world:=your_world
```

## Gazebo-MoveIt 整合

### 功能概述

本包提供了整合的 Gazebo-MoveIt 启动文件，结合了物理仿真与运动规划能力。这些启动文件支持高级运动规划、路径优化和碰撞避免功能。

### 启动 Gazebo-MoveIt 系统

```bash
# 启动单手模式（带 MoveIt）
ros2 launch brainco_gazebo revo2_hand_gazebo_moveit.launch.py hand_type:=left
ros2 launch brainco_gazebo revo2_hand_gazebo_moveit.launch.py hand_type:=right

# 启动双手模式（带 MoveIt）
ros2 launch brainco_gazebo dual_revo2_hand_gazebo_moveit.launch.py
```

### MoveIt 启动参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `hand_type` | `right` | 手的类型 (left/right，仅单手模式) |
| `world` | `empty_world` | Gazebo 世界文件 |
| `use_rviz` | `true` | 是否启动带 MoveIt 插件的 RViz |
| `db` | `false` | 是否启动 MoveIt 仓库数据库 |
| `publish_monitored_planning_scene` | `true` | 是否发布监控的规划场景 |

### 使用 MoveIt 进行运动规划

启动 Gazebo-MoveIt 系统后，可以使用 MoveIt 的 RViz 插件规划和执行轨迹：

1. **交互式规划**: 使用 RViz 中的交互式标记设置目标姿态
2. **生成轨迹**: 点击 "Plan" 生成运动规划
3. **执行轨迹**: 点击 "Execute" 在 Gazebo 中执行轨迹

### 架构说明：Gazebo vs MoveIt 模式

本包提供两种启动配置：

#### 1. 纯 Gazebo 模式
- 启动文件: `revo2_hand_gazebo.launch.py`, `dual_revo2_hand_gazebo.launch.py`
- 硬件接口: `gz_ros2_control` with GazeboSimSystem
- 应用场景: 基础仿真和手动控制

#### 2. Gazebo-MoveIt 整合模式
- 启动文件: `revo2_hand_gazebo_moveit.launch.py`, `dual_revo2_hand_gazebo_moveit.launch.py`
- 硬件接口: 与纯 Gazebo 模式相同
- 额外组件: MoveIt move_group、运动规划管道
- 应用场景: 高级运动规划和自主控制

**主要区别：**
- 两种模式使用相同的 Gazebo 仿真和控制器
- MoveIt 模式在此基础上增加了运动规划能力

## 启动参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `hand_type` | `right` | 手类型：'left' 或 'right' |
| `model` | `revo2_hand` | 机器人模型名称 |
| `world` | `empty_world` | Gazebo世界文件名（无需.sdf后缀） |
| `use_rviz` | `true` | 是否启动RViz可视化 |

## 控制接口

### 控制器概览

| 控制器名称 | 类型 | 管理关节 | 功能 |
|-----------|------|----------|------|
| `joint_state_broadcaster` | JointStateBroadcaster | 所有关节 | 关节状态发布 |
| `revo2_hand_controller` | JointTrajectoryController | 灵巧手6个主关节 | 灵巧手姿态控制 |

### 灵巧手控制示例

#### 张开手掌
```bash
ros2 topic pub --once /right_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "right_thumb_metacarpal_joint",
      "right_thumb_proximal_joint",
      "right_index_proximal_joint",
      "right_middle_proximal_joint",
      "right_ring_proximal_joint",
      "right_pinky_proximal_joint"
    ],
    points: [{
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 1}
    }]
  }'
```

#### 握拳动作
```bash
ros2 topic pub --once /right_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "right_thumb_metacarpal_joint",
      "right_thumb_proximal_joint",
      "right_index_proximal_joint",
      "right_middle_proximal_joint",
      "right_ring_proximal_joint",
      "right_pinky_proximal_joint"
    ],
    points: [{
      positions: [0.3, 0.8, 1.2, 1.2, 1.2, 1.2],
      time_from_start: {sec: 1}
    }]
  }'
```

#### OK手势
```bash
ros2 topic pub --once /left_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "left_thumb_metacarpal_joint",
      "left_thumb_proximal_joint",
      "left_index_proximal_joint",
      "left_middle_proximal_joint",
      "left_ring_proximal_joint",
      "left_pinky_proximal_joint"
    ],
    points: [{
      positions: [1.2, 0.7, 0.6, 0.0, 0.0, 0.0],
      time_from_start: {sec: 2}
    }]
  }'

ros2 topic pub --once /right_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "right_thumb_metacarpal_joint",
      "right_thumb_proximal_joint",
      "right_index_proximal_joint",
      "right_middle_proximal_joint",
      "right_ring_proximal_joint",
      "right_pinky_proximal_joint"
    ],
    points: [{
      positions: [1.2, 0.7, 0.6, 0.0, 0.0, 0.0],
      time_from_start: {sec: 2}
    }]
  }'
```

#### Action轨迹控制

```bash
ros2 action send_goal /left_revo2_hand_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [
      'left_thumb_metacarpal_joint',
      'left_thumb_proximal_joint',
      'left_index_proximal_joint',
      'left_middle_proximal_joint',
      'left_ring_proximal_joint',
      'left_pinky_proximal_joint'
    ],
    points: [
      {  # 张开手掌
        positions: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 1, nanosec: 0}
      },
      {  # 握拳
        positions: [0.3, 0.8, 1.2, 1.2, 1.2, 1.2],
        time_from_start: {sec: 2, nanosec: 500000000}
      },
      {  # 伸出食指
        positions: [0.3, 0.8, 0.0, 1.2, 1.2, 1.2],
        time_from_start: {sec: 4, nanosec: 0}
      },
      {  # 伸出食指和中指
        positions: [0.3, 0.8, 0.0, 0.0, 1.2, 1.2],
        time_from_start: {sec: 5, nanosec: 500000000}
      },
      {  # OK手势
        positions: [1.2, 0.7, 0.6, 0.0, 0.0, 0.0],
        time_from_start: {sec: 7, nanosec: 0}
      }
    ]
  }
}"


ros2 action send_goal /right_revo2_hand_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [
      'right_thumb_metacarpal_joint',
      'right_thumb_proximal_joint',
      'right_index_proximal_joint',
      'right_middle_proximal_joint',
      'right_ring_proximal_joint',
      'right_pinky_proximal_joint'
    ],
    points: [
      {  # 张开手掌
        positions: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 1, nanosec: 0}
      },
      {  # 握拳
        positions: [0.3, 0.8, 1.2, 1.2, 1.2, 1.2],
        time_from_start: {sec: 2, nanosec: 500000000}
      },
      {  # 伸出食指
        positions: [0.3, 0.8, 0.0, 1.2, 1.2, 1.2],
        time_from_start: {sec: 4, nanosec: 0}
      },
      {  # 伸出食指和中指
        positions: [0.3, 0.8, 0.0, 0.0, 1.2, 1.2],
        time_from_start: {sec: 5, nanosec: 500000000}
      },
      {  # OK手势
        positions: [1.2, 0.7, 0.6, 0.0, 0.0, 0.0],
        time_from_start: {sec: 7, nanosec: 0}
      }
    ]
  }
}"
```

## 状态监控

### 系统状态命令

```bash
# 列出所有运行的节点
ros2 node list

# 列出所有控制器
ros2 control list_controllers

# 检查硬件组件
ros2 control list_hardware_components

# 检查硬件接口
ros2 control list_hardware_interfaces

# 列出所有话题
ros2 topic list

# 列出所有动作
ros2 action list

# 监控关节状态
ros2 topic echo /joint_states
```

### 预期的输出

#### 核心节点列表（双手配置）
```bash
/controller_manager
/gz_ros2_control
/joint_state_broadcaster
/left_revo2_hand_controller
/move_group
/right_revo2_hand_controller
/robot_state_publisher
/ros_gz_bridge
/rviz
```

#### 控制器状态
```bash
joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster          active
right_revo2_hand_controller joint_trajectory_controller/JointTrajectoryController  active
left_revo2_hand_controller  joint_trajectory_controller/JointTrajectoryController  active
```

#### 硬件组件（双手配置）
```bash
Hardware Component 1
	name: GazeboSimSystem
	type:
	plugin name:
	state: id=3 label=active
	command interfaces
		left_thumb_metacarpal_joint/position [available] [claimed]
		left_thumb_proximal_joint/position [available] [claimed]
		left_index_proximal_joint/position [available] [claimed]
		left_middle_proximal_joint/position [available] [claimed]
		left_ring_proximal_joint/position [available] [claimed]
		left_pinky_proximal_joint/position [available] [claimed]
		left_thumb_distal_joint_mimic/position [available] [unclaimed]
		left_index_distal_joint_mimic/position [available] [unclaimed]
		left_middle_distal_joint_mimic/position [available] [unclaimed]
		left_ring_distal_joint_mimic/position [available] [unclaimed]
		left_pinky_distal_joint_mimic/position [available] [unclaimed]
		right_thumb_metacarpal_joint/position [available] [claimed]
		right_thumb_proximal_joint/position [available] [claimed]
		right_index_proximal_joint/position [available] [claimed]
		right_middle_proximal_joint/position [available] [claimed]
		right_ring_proximal_joint/position [available] [claimed]
		right_pinky_proximal_joint/position [available] [claimed]
		right_thumb_distal_joint_mimic/position [available] [unclaimed]
		right_index_distal_joint_mimic/position [available] [unclaimed]
		right_middle_distal_joint_mimic/position [available] [unclaimed]
		right_ring_distal_joint_mimic/position [available] [unclaimed]
		right_pinky_distal_joint_mimic/position [available] [unclaimed]
```

#### 硬件接口（双手配置）
```bash
command interfaces
	left_index_distal_joint_mimic/position [available] [unclaimed]
	left_index_proximal_joint/position [available] [claimed]
	left_middle_distal_joint_mimic/position [available] [unclaimed]
	left_middle_proximal_joint/position [available] [claimed]
	left_pinky_distal_joint_mimic/position [available] [unclaimed]
	left_pinky_proximal_joint/position [available] [claimed]
	left_ring_distal_joint_mimic/position [available] [unclaimed]
	left_ring_proximal_joint/position [available] [claimed]
	left_thumb_distal_joint_mimic/position [available] [unclaimed]
	left_thumb_metacarpal_joint/position [available] [claimed]
	left_thumb_proximal_joint/position [available] [claimed]
	right_index_distal_joint_mimic/position [available] [unclaimed]
	right_index_proximal_joint/position [available] [claimed]
	right_middle_distal_joint_mimic/position [available] [unclaimed]
	right_middle_proximal_joint/position [available] [claimed]
	right_pinky_distal_joint_mimic/position [available] [unclaimed]
	right_pinky_proximal_joint/position [available] [claimed]
	right_ring_distal_joint_mimic/position [available] [unclaimed]
	right_ring_proximal_joint/position [available] [claimed]
	right_thumb_distal_joint_mimic/position [available] [unclaimed]
	right_thumb_metacarpal_joint/position [available] [claimed]
	right_thumb_proximal_joint/position [available] [claimed]
state interfaces
	left_index_distal_joint_mimic/effort
	left_index_distal_joint_mimic/position
	left_index_distal_joint_mimic/velocity
	left_index_proximal_joint/position
	left_index_proximal_joint/velocity
	left_middle_distal_joint_mimic/effort
	left_middle_distal_joint_mimic/position
	left_middle_distal_joint_mimic/velocity
	left_middle_proximal_joint/position
	left_middle_proximal_joint/velocity
	left_pinky_distal_joint_mimic/effort
	left_pinky_distal_joint_mimic/position
	left_pinky_distal_joint_mimic/velocity
	left_pinky_proximal_joint/position
	left_pinky_proximal_joint/velocity
	left_ring_distal_joint_mimic/effort
	left_ring_distal_joint_mimic/position
	left_ring_distal_joint_mimic/velocity
	left_ring_proximal_joint/position
	left_ring_proximal_joint/velocity
	left_thumb_distal_joint_mimic/effort
	left_thumb_distal_joint_mimic/position
	left_thumb_distal_joint_mimic/velocity
	left_thumb_metacarpal_joint/position
	left_thumb_metacarpal_joint/velocity
	left_thumb_proximal_joint/position
	left_thumb_proximal_joint/velocity
	right_index_distal_joint_mimic/effort
	right_index_distal_joint_mimic/position
	right_index_distal_joint_mimic/velocity
	right_index_proximal_joint/position
	right_index_proximal_joint/velocity
	right_middle_distal_joint_mimic/effort
	right_middle_distal_joint_mimic/position
	right_middle_distal_joint_mimic/velocity
	right_middle_proximal_joint/position
	right_middle_proximal_joint/velocity
	right_pinky_distal_joint_mimic/effort
	right_pinky_distal_joint_mimic/position
	right_pinky_distal_joint_mimic/velocity
	right_pinky_proximal_joint/position
	right_pinky_proximal_joint/velocity
	right_ring_distal_joint_mimic/effort
	right_ring_distal_joint_mimic/position
	right_ring_distal_joint_mimic/velocity
	right_ring_proximal_joint/position
	right_ring_proximal_joint/velocity
	right_thumb_distal_joint_mimic/effort
	right_thumb_distal_joint_mimic/position
	right_thumb_distal_joint_mimic/velocity
	right_thumb_metacarpal_joint/position
	right_thumb_metacarpal_joint/velocity
	right_thumb_proximal_joint/position
	right_thumb_proximal_joint/velocity
```

#### 核心话题列表（双手配置）
```bash
/clock
/joint_states
/left_revo2_hand_controller/joint_trajectory
/planning_scene
/right_revo2_hand_controller/joint_trajectory
/robot_description
/tf
/tf_static
/trajectory_execution_event
```

#### 核心动作列表（双手配置）
```bash
/execute_trajectory
/left_revo2_hand_controller/follow_joint_trajectory
/right_revo2_hand_controller/follow_joint_trajectory
```

### 性能监控

```bash
# 检查控制频率
ros2 topic hz /joint_states

# 检查控制器更新频率
ros2 topic hz /revo2_hand_controller/controller_state

# 监控系统资源
htop
```

## 功能包结构

```
brainco_gazebo/
├── launch/                                         # ROS 启动文件
│   ├── revo2_hand_gazebo.launch.py                 # 单手 Gazebo 启动
│   ├── dual_revo2_hand_gazebo.launch.py            # 双手 Gazebo 启动
│   ├── revo2_hand_gazebo_moveit.launch.py          # 单手 Gazebo+MoveIt 启动
│   └── dual_revo2_hand_gazebo_moveit.launch.py     # 双手 Gazebo+MoveIt 启动
├── config/                                         # 配置文件
│   ├── revo2_gazebo_description.urdf.xacro         # 单手机器人 Gazebo 描述文件
│   ├── dual_revo2_gazebo_description.urdf.xacro    # 双手机器人 Gazebo 描述文件
│   ├── revo2_left_controllers.yaml                 # 左手控制器配置
│   ├── revo2_right_controllers.yaml                # 右手控制器配置
│   └── dual_revo2_controllers.yaml                 # 双手控制器配置
├── worlds/                                         # Gazebo 世界文件
│   └── empty_world.sdf                            # 默认世界
├── rviz/                                           # RViz 配置文件
│   ├── revo2_left_hand.rviz                        # 左手 RViz 配置
│   ├── revo2_right_hand.rviz                       # 右手 RViz 配置
│   └── dual_revo2_hand.rviz                        # 双手 RViz 配置
├── CMakeLists.txt                                 # 构建配置
├── package.xml                                    # 功能包描述
├── CHANGELOG.rst                                  # 版本变更日志
├── LICENSE                                        # 许可证文件
├── README.md                                      # 英文文档
└── README_CN.md                                   # 中文文档
```

## API参考

### Action接口

- **灵巧手控制**: `/revo2_hand_controller/follow_joint_trajectory`

### Topic接口

- **关节状态**: `/joint_states` (sensor_msgs/JointState)
- **轨迹命令**: `/revo2_hand_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)
- **控制器状态**: `/revo2_hand_controller/controller_state`

### Service接口

- **控制器管理**: `/controller_manager/list_controllers`
- **硬件接口**: `/controller_manager/list_hardware_interfaces`

## 许可证

本项目采用Apache License 2.0许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。
