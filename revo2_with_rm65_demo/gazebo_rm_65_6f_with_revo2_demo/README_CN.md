<div align="right">

[简体中文](README_CN.md)|[English](README.md)

</div>

<div align="center">

# BrainCo Revo2 + RM65-6F 机械臂 Gazebo 仿真演示包

</div>

## 功能包概述

gazebo_rm_65_6f_with_revo2_demo 功能包提供了睿尔曼RM65-6F机械臂集成BrainCo Revo2灵巧手的完整Gazebo仿真环境。基于ROS2和Ignition Gazebo 6构建，为机器人研究和开发提供专业级的仿真体验。

### 主要特性

- **集成机器人系统**: RM65-6F机械臂与Revo2灵巧手的完整仿真
- **Gazebo物理仿真**: 基于Ignition Gazebo 6的高保真物理仿真
- **ROS2控制集成**: 完整的ros2_control支持，包含轨迹控制器
- **RViz可视化**: 内置启动文件，支持RViz可视化
- **多控制器支持**: 机械臂和灵巧手系统独立控制

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
# 将 gazebo_rm_65_6f_with_revo2_demo 复制到 src/

# 依赖安装
rosdep install --ignore-src --from-paths . -y -r

# 构建工作空间
cd ~/brainco_ws
colcon build --packages-select revo2_description gazebo_rm_65_6f_with_revo2_demo --symlink-install
source install/setup.bash
```

### 2. 验证安装

```bash
# 检查功能包是否可用
ros2 pkg list | grep -E "(revo2_description|gazebo_rm_65_6f_with_revo2_demo)"

```

## 快速开始

### 基础启动

```bash
# 加载工作空间
source ~/brainco_ws/install/setup.bash

# 启动完整仿真
ros2 launch gazebo_rm_65_6f_with_revo2_demo gazebo_rm_65_6f_with_revo2.launch.py
```

### 参数化启动

```bash
# 不启动RViz（提升性能）
ros2 launch gazebo_rm_65_6f_with_revo2_demo gazebo_rm_65_6f_with_revo2.launch.py use_rviz:=false

# 使用自定义世界文件
ros2 launch gazebo_rm_65_6f_with_revo2_demo gazebo_rm_65_6f_with_revo2.launch.py world:=your_world
```

## 启动参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `model` | `rm_65_revo2` | 机器人模型名称 |
| `world` | `empty_world` | Gazebo世界文件名（无需.sdf后缀） |
| `use_rviz` | `true` | 是否启动RViz可视化 |

## 控制接口

### 控制器概览

| 控制器名称 | 类型 | 管理关节 | 功能 |
|-----------|------|----------|------|
| `joint_state_broadcaster` | JointStateBroadcaster | 所有关节 | 关节状态发布 |
| `rm_group_controller` | JointTrajectoryController | joint1~joint6 | 机械臂运动控制 |
| `revo2_hand_controller` | JointTrajectoryController | 灵巧手6个主关节 | 灵巧手姿态控制 |

### 灵巧手控制示例

#### 张开手掌
```bash
ros2 topic pub --once /revo2_hand_controller/joint_trajectory \
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
      positions: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0], 
      time_from_start: {sec: 1}
    }]
  }'
```

#### 握拳动作
```bash
ros2 topic pub --once /revo2_hand_controller/joint_trajectory \
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
ros2 topic pub --once /revo2_hand_controller/joint_trajectory \
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

### 机械臂控制示例

#### 位置控制
```bash
# 移动到预设位置
ros2 action send_goal /rm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], 
      points: [
        {positions: [0.0, -0.5, 1.0, 0.0, 1.0, 0.0], time_from_start: {sec: 2}}
      ]
    }
  }"

# 回到初始位置
ros2 action send_goal /rm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], 
      points: [
        {positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 4}}
      ]
    }
  }"
```

## 状态监控

### 系统状态命令

```bash
# 列出所有控制器
ros2 control list_controllers

# 检查硬件组件
ros2 control list_hardware_components

# 监控关节状态
ros2 topic echo /joint_states

# 监控控制器状态
ros2 topic echo /revo2_hand_controller/controller_state

# 列出所有话题
ros2 topic list

# 列出所有服务
ros2 service list

# 列出所有动作
ros2 action list
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
gazebo_rm_65_6f_with_revo2_demo/
├── launch/                                    # ROS启动文件
│   └── gazebo_rm_65_6f_with_revo2.launch.py  # 主启动文件
├── config/                                    # 配置文件
│   ├── gazebo_65_6f_revo2_description_gz.urdf.xacro  # 机器人描述
│   └── revo2_controllers.yaml                # 控制器配置
├── urdf/                                      # URDF/Xacro文件
│   ├── rm_65_gazebo.urdf.xacro              # RM65机械臂URDF
│   └── rm_65_with_revo2_right.xacro         # 集成模型
├── worlds/                                    # Gazebo世界文件
│   └── empty_world.sdf                       # 默认世界
├── rviz/                                      # RViz配置
│   └── rm_65_revo2.rviz                      # 默认RViz配置
├── meshes/                                    # 3D模型文件
│   └── rm_65_arm/                            # RM65机械臂模型
├── CMakeLists.txt                            # 构建配置
├── package.xml                               # 功能包描述
├── CHANGELOG.rst                             # 版本变更日志
├── LICENSE                                   # 许可证文件
├── README.md                                 # 英文文档
└── README_CN.md                              # 中文文档
```

## API参考

### Action接口

- **灵巧手控制**: `/revo2_hand_controller/follow_joint_trajectory`
- **机械臂控制**: `/rm_group_controller/follow_joint_trajectory`

### Topic接口

- **关节状态**: `/joint_states` (sensor_msgs/JointState)
- **轨迹命令**: `/revo2_hand_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)
- **控制器状态**: `/revo2_hand_controller/controller_state`

### Service接口

- **控制器管理**: `/controller_manager/list_controllers`
- **硬件接口**: `/controller_manager/list_hardware_interfaces`

## 许可证

本项目采用Apache License 2.0许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。
