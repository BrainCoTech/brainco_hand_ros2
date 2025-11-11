# BrainCo Hand EtherCAT 驱动包

[English](README.md) | [简体中文](README_CN.md)

## 概述

BrainCo Hand EtherCAT 驱动包为 BrainCo Revo2 灵巧手提供基于 EtherCAT 通信协议的 ROS2 硬件接口。该功能包实现了 ros2_control 硬件接口，支持通过 EtherCAT 主站对 6 个手指关节进行实时控制和状态监控。

## 特性

- **EtherCAT 通信**：与 Revo2 灵巧手硬件的实时 EtherCAT 通信
- **ros2_control 集成**：完整的 ros2_control 硬件接口实现
- **双手支持**：支持左手和右手配置
- **MoveIt 集成**：可选的 MoveIt 集成用于运动规划
- **实时控制**：高频控制循环，实现精确的手指操控
- **状态反馈**：所有关节的位置、速度反馈
- **轨迹控制**：关节轨迹控制器支持，实现平滑运动执行

## 环境系统

- Ubuntu 22.04
- ROS 2 (Humble)
- EtherCAT 主站 (IgH EtherCAT Master)
- Python 3.8+

## 安装和设置

### 1. 依赖项

确保已安装以下软件包：

```bash
# ROS2 依赖项
sudo apt install ros-humble-controller-manager ros-humble-joint-trajectory-controller
sudo apt install ros-humble-joint-state-broadcaster ros-humble-robot-state-publisher

# Python 依赖项
pip3 install rclpy trajectory_msgs sensor_msgs control_msgs

# EtherCAT 主站（如果尚未安装）
# 请按照系统的 EtherCAT 主站安装指南进行安装
```

### 2. 构建工作空间

```bash
# 进入工作空间
cd ~/brainco_ws

# 安装依赖项
rosdep install --ignore-src --from-paths src -y -r

# 构建功能包
colcon build --packages-select brainco_hand_ethercat_driver stark_ethercat_interface stark_ethercat_driver --symlink-install

# Source 工作空间
source install/setup.bash
```

### 3. 验证安装

```bash
# 检查功能包是否可用
ros2 pkg list | grep brainco_hand_ethercat_driver

# 检查 EtherCAT 主站状态
sudo systemctl status ethercat
```

## 硬件连接

### EtherCAT 设置

在使用驱动之前，请确保您的 EtherCAT 设置正确：

```bash
# 检查网络接口
ip link show

# 检查 EtherCAT 主站状态
sudo systemctl status ethercat

# 检查 EtherCAT 从站设备
sudo ethercat slaves

# 检查设备 SDO/PDO 信息
sudo ethercat sdos
sudo ethercat pdos
```

## 快速开始

### 启动右手系统

```bash
ros2 launch brainco_hand_ethercat_driver revo2_system.launch.py hand_type:=right
```

### 启动左手系统

```bash
ros2 launch brainco_hand_ethercat_driver revo2_system.launch.py hand_type:=left
```

```bash
# 过滤域信息输出
ros2 launch brainco_hand_ethercat_driver revo2_system.launch.py 2>&1 | grep -v "\[ros2_control_node-1\] Domain"
```

### 启动 MoveIt 集成

```bash
# 右手带 MoveIt
ros2 launch brainco_hand_ethercat_driver revo2_real_moveit.launch.py hand_type:=right

# 左手带 MoveIt
ros2 launch brainco_hand_ethercat_driver revo2_real_moveit.launch.py hand_type:=left
```

### Launch 参数

#### 基础系统参数（revo2_system.launch.py）

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `hand_type` | `right` | 手型选择：`left` 或 `right` |
| `prefix` | `""` | 关节名称前缀，用于多机器人设置 |
| `ctrl_param_duration_ms` | `20` | 运动时间参数，单位毫秒（1ms = 最快速度） |
| `robot_controller` | 自动生成 | 控制器名称（根据 hand_type 自动生成） |

#### MoveIt 集成参数（revo2_real_moveit.launch.py）

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `hand_type` | `right` | 手型选择：`left` 或 `right` |
| `ctrl_param_duration_ms` | `10` | 运动时间参数，单位毫秒（1ms = 最快速度） |
| `use_rviz` | `true` | 是否启动 RViz 可视化 |
| `publish_monitored_planning_scene` | `true` | 是否发布监控的规划场景 |

## 控制接口

### 使用 Topic 接口

右手轨迹控制示例：

```bash
ros2 topic pub --once /right_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "right_thumb_proximal_joint",
      "right_thumb_metacarpal_joint",
      "right_index_proximal_joint",
      "right_middle_proximal_joint",
      "right_ring_proximal_joint",
      "right_pinky_proximal_joint"
    ],
    points: [{
      positions: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
      time_from_start: {sec: 2}
    }]
  }'
```

左手轨迹控制示例：

```bash
ros2 topic pub --once /left_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "left_thumb_proximal_joint",
      "left_thumb_metacarpal_joint",
      "left_index_proximal_joint",
      "left_middle_proximal_joint",
      "left_ring_proximal_joint",
      "left_pinky_proximal_joint"
    ],
    points: [{
      positions: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
      time_from_start: {sec: 2}
    }]
  }'
```

### 使用 Action 接口

右手 Action 控制示例：

```bash
ros2 action send_goal /right_revo2_hand_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{
    trajectory: {
      joint_names: [
        "right_thumb_proximal_joint",
        "right_thumb_metacarpal_joint",
        "right_index_proximal_joint",
        "right_middle_proximal_joint",
        "right_ring_proximal_joint",
        "right_pinky_proximal_joint"
      ],
      points: [
        {
          positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 1}
        },
        {
          positions: [0.5, 0.0, 1.4, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 0.0, 0.0],
          time_from_start: {sec: 3}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 1.4, 0.0],
          time_from_start: {sec: 4}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 1.4, 1.4],
          time_from_start: {sec: 5}
        }
      ]
    }
  }'
```

左手 Action 控制示例：

```bash
ros2 action send_goal /left_revo2_hand_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{
    trajectory: {
      joint_names: [
        "left_thumb_proximal_joint",
        "left_thumb_metacarpal_joint",
        "left_index_proximal_joint",
        "left_middle_proximal_joint",
        "left_ring_proximal_joint",
        "left_pinky_proximal_joint"
      ],
      points: [
        {
          positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 1}
        },
        {
          positions: [0.5, 0.0, 1.4, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 0.0, 0.0],
          time_from_start: {sec: 3}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 1.4, 0.0],
          time_from_start: {sec: 4}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 1.4, 1.4],
          time_from_start: {sec: 5}
        }
      ]
    }
  }'
```


### 归零位置（Home Position）

返回归零位置：

```bash
# 右手
ros2 topic pub --once /right_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "right_thumb_proximal_joint",
      "right_thumb_metacarpal_joint",
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

# 左手
ros2 topic pub --once /left_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "left_thumb_proximal_joint",
      "left_thumb_metacarpal_joint",
      "left_index_proximal_joint",
      "left_middle_proximal_joint",
      "left_ring_proximal_joint",
      "left_pinky_proximal_joint"
    ],
    points: [{
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 1}
    }]
  }'
```

## 测试脚本

功能包包含一个交互式测试脚本：

```bash
# 进入脚本目录
cd <package_path>/scripts/

# 交互式菜单
python3 test_revo2_ethercat.py --menu

# 直接测试命令
python3 test_revo2_ethercat.py --test all_fingers --amplitude 0.8
python3 test_revo2_ethercat.py --test individual
python3 test_revo2_ethercat.py --test home
```

## 监控和调试

### 检查系统状态

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

#### 核心节点列表（右手配置）
```bash
/controller_manager
/joint_state_broadcaster
/right_revo2_hand_controller
/robot_state_publisher
```

#### 控制器状态
```bash
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster          active
right_revo2_hand_controller   joint_trajectory_controller/JointTrajectoryController  active
```

#### 硬件组件（右手配置）
```bash
Hardware Component 1
	name: Revo2RightEthercatSystem
	type: system
	plugin name: stark_ethercat_driver/EthercatDriver
	state: id=3 label=active
	command interfaces
		right_thumb_proximal_joint/position [available] [claimed]
		right_thumb_metacarpal_joint/position [available] [claimed]
		right_index_proximal_joint/position [available] [claimed]
		right_middle_proximal_joint/position [available] [claimed]
		right_ring_proximal_joint/position [available] [claimed]
		right_pinky_proximal_joint/position [available] [claimed]
```

#### 硬件接口（右手配置）
```bash
command interfaces
	right_index_proximal_joint/position [available] [claimed]
	right_middle_proximal_joint/position [available] [claimed]
	right_pinky_proximal_joint/position [available] [claimed]
	right_ring_proximal_joint/position [available] [claimed]
	right_thumb_metacarpal_joint/position [available] [claimed]
	right_thumb_proximal_joint/position [available] [claimed]
state interfaces
	right_index_proximal_joint/effort
	right_index_proximal_joint/position
	right_index_proximal_joint/velocity
	right_middle_proximal_joint/effort
	right_middle_proximal_joint/position
	right_middle_proximal_joint/velocity
	right_pinky_proximal_joint/effort
	right_pinky_proximal_joint/position
	right_pinky_proximal_joint/velocity
	right_ring_proximal_joint/effort
	right_ring_proximal_joint/position
	right_ring_proximal_joint/velocity
	right_thumb_metacarpal_joint/effort
	right_thumb_metacarpal_joint/position
	right_thumb_metacarpal_joint/velocity
	right_thumb_proximal_joint/effort
	right_thumb_proximal_joint/position
	right_thumb_proximal_joint/velocity
```

#### 核心话题列表（右手配置）
```bash
/joint_states
/right_revo2_hand_controller/joint_trajectory
/robot_description
/tf
/tf_static
```

#### 核心动作列表（右手配置）
```bash
/right_revo2_hand_controller/follow_joint_trajectory
```

## 关节映射

### 右手关节

| 关节名称 | 描述 | 角度范围（弧度） | 最大角度（度） |
|---------|------|----------------|--------------|
| `right_thumb_proximal_joint` | 拇指近端关节 | 0 ~ 1.03 | 59 |
| `right_thumb_metacarpal_joint` | 拇指掌骨关节 | 0 ~ 1.57 | 90 |
| `right_index_proximal_joint` | 食指近端关节 | 0 ~ 1.41 | 81 |
| `right_middle_proximal_joint` | 中指近端关节 | 0 ~ 1.41 | 81 |
| `right_ring_proximal_joint` | 无名指近端关节 | 0 ~ 1.41 | 81 |
| `right_pinky_proximal_joint` | 小指近端关节 | 0 ~ 1.41 | 81 |

### 左手关节

| 关节名称 | 描述 | 角度范围（弧度） | 最大角度（度） |
|---------|------|----------------|--------------|
| `left_thumb_proximal_joint` | 拇指近端关节 | 0 ~ 1.03 | 59 |
| `left_thumb_metacarpal_joint` | 拇指掌骨关节 | 0 ~ 1.57 | 90 |
| `left_index_proximal_joint` | 食指近端关节 | 0 ~ 1.41 | 81 |
| `left_middle_proximal_joint` | 中指近端关节 | 0 ~ 1.41 | 81 |
| `left_ring_proximal_joint` | 无名指近端关节 | 0 ~ 1.41 | 81 |
| `left_pinky_proximal_joint` | 小指近端关节 | 0 ~ 1.41 | 81 |

## 功能包结构

```
brainco_hand_ethercat_driver/
├── launch/                                      # Launch 文件
│   ├── revo2_system.launch.py                    # 主系统启动文件
│   └── revo2_real_moveit.launch.py               # MoveIt 集成启动文件
├── config/                                      # 配置文件
├── include/                                     # 头文件
│   └── revo2_ethercat_plugins/                  # 插件头文件
├── src/                                         # 源文件
│   └── revo2_joints_system_slave.cpp            # 硬件接口实现
├── scripts/                                     # 工具脚本
│   └── test_revo2_ethercat.py                   # 测试脚本
├── CMakeLists.txt                               # 构建配置
├── package.xml                                  # 功能包描述
├── revo2_plugins.xml                            # 插件描述
└── README.md                                    # 英文说明
```

## 许可证

本项目采用 Apache License 2.0 许可证 - 详见 [LICENSE](LICENSE) 文件。

## 联系方式

如有问题或建议，请联系开发团队。