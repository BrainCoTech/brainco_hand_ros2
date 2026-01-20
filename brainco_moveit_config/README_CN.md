# BrainCo MoveIt Config

[English](README.md) | [简体中文](README_CN.md)

## 概述

BrainCo MoveIt Config 功能包为 BrainCo Revo2 灵巧手提供完整的 MoveIt 配置。支持三种配置：

- **单左手**：左手 Revo2 灵巧手的 MoveIt 配置
- **单右手**：右手 Revo2 灵巧手的 MoveIt 配置
- **双手**：左右手 Revo2 灵巧手的 MoveIt 配置

所有配置都使用 `mock_components/GenericSystem` 作为 FakeSystem 后端，允许您在没有真实硬件的情况下使用 MoveIt 进行测试和开发。

## 特性

- **FakeSystem 支持**：使用模拟硬件接口进行无硬件测试
- **完整的 MoveIt 集成**：完整的 MoveIt 规划和执行流程
- **RViz 可视化**：内置 RViz 配置用于可视化
- **多种配置**：支持左手、右手和双手配置
- **预定义姿态**：手部张开、半握和完全握拳姿态
- **ros2_control 集成**：完整的 ros2_control 支持与轨迹控制器

## 环境系统

- Ubuntu 22.04
- ROS 2 (Humble)
- MoveIt 2
- revo2_description 功能包

## 安装和设置

### 1. 依赖项

确保已安装以下软件包：

```bash
sudo apt install ros-humble-moveit ros-humble-moveit-visual-tools
sudo apt install ros-humble-controller-manager ros-humble-joint-trajectory-controller
```

### 2. 构建工作空间

**重要提示**：
- `brainco_moveit_config` 包本身可以独立编译，用于 FakeSystem 模式（无需硬件）
- 如果要使用真机功能（Modbus/CAN FD/EtherCAT），需要先编译相应的硬件驱动包
- CAN FD 和 EtherCAT 功能默认不编译，需要编译时启用

**方式一：只编译 MoveIt 配置包（用于 FakeSystem 模式）**

```bash
# 进入工作空间
cd ~/brainco_ws

# 安装依赖项
rosdep install --ignore-src --from-paths src -y -r

# 只编译 MoveIt 配置包（用于 FakeSystem 模式）
colcon build --packages-select brainco_moveit_config --symlink-install

# Source 工作空间
source install/setup.bash
```

**方式二：编译所有包（包括真机驱动，推荐）**

使用项目提供的编译脚本：

```bash
# 进入工作空间根目录
cd ~/brainco_ws

# 默认编译（不启用 CAN FD 和 EtherCAT，仅支持 Modbus）
./build.sh

# 启用 CAN FD 支持
./build.sh --canfd

# 启用 EtherCAT 支持
./build.sh --ethercat

# 同时启用 CAN FD 和 EtherCAT
./build.sh --canfd --ethercat
```

或使用 colcon 命令：

```bash
# 默认编译（不启用 CAN FD 和 EtherCAT，仅支持 Modbus）
colcon build --symlink-install --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# 启用 CAN FD 支持
colcon build --symlink-install --cmake-args -DENABLE_CANFD=ON --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# 启用 EtherCAT 支持
colcon build --symlink-install

# 同时启用 CAN FD 和 EtherCAT
colcon build --symlink-install --cmake-args -DENABLE_CANFD=ON
```

**编译选项说明**：
- **默认行为**：`ENABLE_CANFD=OFF`（禁用 CAN FD 支持），EtherCAT 相关包默认不编译，仅支持 Modbus 模式
- **启用 CAN FD**：使用 `-DENABLE_CANFD=ON`，需要 ZLG USB-CAN FD 库文件
- **启用 EtherCAT**：EtherCAT 是独立的 ROS2 包，默认不编译。如需启用，请在工作空间根目录直接编译（不添加 `--packages-ignore`），或使用项目根目录的 `build.sh --ethercat` 脚本
- **FakeSystem 模式**：不需要编译硬件驱动包，可以直接使用 MoveIt 进行测试和开发

### 3. 验证安装

```bash
# 检查功能包是否可用
ros2 pkg list | grep brainco_moveit_config
```

## 快速开始

### 启动单左手

```bash
ros2 launch brainco_moveit_config revo2_left_moveit.launch.py
```

### 启动单右手

```bash
ros2 launch brainco_moveit_config revo2_right_moveit.launch.py
```

### 启动双手

```bash
ros2 launch brainco_moveit_config dual_revo2_moveit.launch.py
```

## 启动 MoveIt 集成（真机）

**重要提示**：
- 使用真机功能前，请确保已编译相应的硬件驱动包
- 如果未启用 CAN FD 支持，使用 CAN FD 模式会报错
- 如果禁用了 EtherCAT 支持，使用 EtherCAT 模式会报错（找不到相关包）
- Modbus 模式是默认支持的，无需额外编译选项

### 单手系统（Modbus/CAN FD 模式）

```bash
# 右手带 MoveIt（Modbus 模式）
ros2 launch brainco_moveit_config revo2_real_moveit.launch.py hand_type:=right

# 右手带 MoveIt（CAN FD 模式）
ros2 launch brainco_moveit_config revo2_real_moveit.launch.py hand_type:=right protocol:=canfd

# 左手带 MoveIt（Modbus 模式）
ros2 launch brainco_moveit_config revo2_real_moveit.launch.py hand_type:=left

# 左手带 MoveIt（CAN FD 模式）
ros2 launch brainco_moveit_config revo2_real_moveit.launch.py hand_type:=left protocol:=canfd
```

### 双手系统（MoveIt）

```bash
# 双手系统 Modbus 模式（默认）
ros2 launch brainco_moveit_config dual_revo2_real_moveit.launch.py

# 双手系统 CAN FD 模式
ros2 launch brainco_moveit_config dual_revo2_real_moveit.launch.py protocol:=canfd
```

### EtherCAT 模式

```bash
# 右手带 MoveIt（EtherCAT 模式）
ros2 launch brainco_moveit_config revo2_ethercat_real_moveit.launch.py hand_type:=right

# 左手带 MoveIt（EtherCAT 模式）
ros2 launch brainco_moveit_config revo2_ethercat_real_moveit.launch.py hand_type:=left
```

### Launch 参数

所有 launch 文件都支持以下参数：

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `use_rviz` | `true` | 是否启动 RViz 可视化 |
| `publish_frequency` | `15.0` | TF 发布频率 (Hz) |
| `allow_trajectory_execution` | `true` | 是否允许轨迹执行 |
| `publish_monitored_planning_scene` | `true` | 是否发布监控的规划场景 |

带参数示例：

```bash
ros2 launch brainco_moveit_config revo2_right_moveit.launch.py use_rviz:=true
```

## 控制接口

### 使用 Action 接口

右手示例：

```bash
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
      {
        positions: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 1, nanosec: 0}
      }
    ]
  }
}"
```

左手示例：

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
      {
        positions: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 1, nanosec: 0}
      }
    ]
  }
}"
```

## 预定义手部姿态

每个配置都包含以下预定义姿态：

### 单手配置

- `hand_open`: 完全张开
- `hand_half_close`: 半握
- `hand_close`: 完全握拳

### 双手配置

- `left_hand_open` / `right_hand_open`: 完全张开
- `left_hand_half_close` / `right_hand_half_close`: 半握
- `left_hand_close` / `right_hand_close`: 完全握拳

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

#### 核心节点列表（双手配置）
```bash
/controller_manager
/joint_state_broadcaster
/left_revo2_hand_controller
/move_group
/right_revo2_hand_controller
/robot_state_publisher
/rviz
```

#### 控制器状态
```bash
left_revo2_hand_controller  joint_trajectory_controller/JointTrajectoryController  active
right_revo2_hand_controller joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster          active
```

#### 硬件组件（双手配置）
```bash
Hardware Component 1
	name: FakeSystem
	type: system
	plugin name: mock_components/GenericSystem
	state: id=3 label=active
	command interfaces
		left_thumb_metacarpal_joint/position [available] [claimed]
		left_thumb_proximal_joint/position [available] [claimed]
		left_index_proximal_joint/position [available] [claimed]
		left_middle_proximal_joint/position [available] [claimed]
		left_ring_proximal_joint/position [available] [claimed]
		left_pinky_proximal_joint/position [available] [claimed]
		right_thumb_metacarpal_joint/position [available] [claimed]
		right_thumb_proximal_joint/position [available] [claimed]
		right_index_proximal_joint/position [available] [claimed]
		right_middle_proximal_joint/position [available] [claimed]
		right_ring_proximal_joint/position [available] [claimed]
		right_pinky_proximal_joint/position [available] [claimed]
```

#### 硬件接口（双手配置）
```bash
command interfaces
	left_index_proximal_joint/position [available] [claimed]
	left_middle_proximal_joint/position [available] [claimed]
	left_pinky_proximal_joint/position [available] [claimed]
	left_ring_proximal_joint/position [available] [claimed]
	left_thumb_metacarpal_joint/position [available] [claimed]
	left_thumb_proximal_joint/position [available] [claimed]
	right_index_proximal_joint/position [available] [claimed]
	right_middle_proximal_joint/position [available] [claimed]
	right_pinky_proximal_joint/position [available] [claimed]
	right_ring_proximal_joint/position [available] [claimed]
	right_thumb_metacarpal_joint/position [available] [claimed]
	right_thumb_proximal_joint/position [available] [claimed]
state interfaces
	left_index_proximal_joint/position
	left_index_proximal_joint/velocity
	left_middle_proximal_joint/position
	left_middle_proximal_joint/velocity
	left_pinky_proximal_joint/position
	left_pinky_proximal_joint/velocity
	left_ring_proximal_joint/position
	left_ring_proximal_joint/velocity
	left_thumb_metacarpal_joint/position
	left_thumb_metacarpal_joint/velocity
	left_thumb_proximal_joint/position
	left_thumb_proximal_joint/velocity
	right_index_proximal_joint/position
	right_index_proximal_joint/velocity
	right_middle_proximal_joint/position
	right_middle_proximal_joint/velocity
	right_pinky_proximal_joint/position
	right_pinky_proximal_joint/velocity
	right_ring_proximal_joint/position
	right_ring_proximal_joint/velocity
	right_thumb_metacarpal_joint/position
	right_thumb_metacarpal_joint/velocity
	right_thumb_proximal_joint/position
	right_thumb_proximal_joint/velocity
```

#### 核心话题列表（双手配置）
```bash
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

## 功能包结构

```
brainco_moveit_config/
├── launch/                                      # Launch 文件
│   ├── revo2_left_moveit.launch.py              # 左手启动文件（FakeSystem）
│   ├── revo2_right_moveit.launch.py             # 右手启动文件（FakeSystem）
│   ├── dual_revo2_moveit.launch.py              # 双手启动文件（FakeSystem）
│   ├── revo2_real_moveit.launch.py              # 单手真机启动文件（Modbus/CAN FD）
│   ├── dual_revo2_real_moveit.launch.py         # 双手真机启动文件（Modbus/CAN FD）
│   └── revo2_ethercat_real_moveit.launch.py     # 单手真机启动文件（EtherCAT）
├── config/                                      # 配置文件
│   ├── revo2_left.urdf.xacro                    # 左手 URDF
│   ├── revo2_left.ros2_control.xacro            # 左手 ros2_control
│   ├── revo2_left.srdf                          # 左手 SRDF
│   ├── revo2_left_controllers.yaml              # 左手控制器配置
│   ├── revo2_left_initial_positions.yaml        # 左手初始位置
│   ├── revo2_left_joint_limits.yaml             # 左手关节限制
│   ├── revo2_left_kinematics.yaml               # 左手运动学配置
│   ├── revo2_left_moveit_controllers.yaml       # 左手 MoveIt 控制器
│   ├── revo2_right.urdf.xacro                   # 右手 URDF
│   ├── revo2_right.ros2_control.xacro           # 右手 ros2_control
│   ├── revo2_right.srdf                         # 右手 SRDF
│   ├── revo2_right_controllers.yaml             # 右手控制器配置
│   ├── revo2_right_initial_positions.yaml       # 右手初始位置
│   ├── revo2_right_joint_limits.yaml            # 右手关节限制
│   ├── revo2_right_kinematics.yaml              # 右手运动学配置
│   ├── revo2_right_moveit_controllers.yaml      # 右手 MoveIt 控制器
│   ├── dual_revo2.urdf.xacro                    # 双手 URDF
│   ├── dual_revo2.ros2_control.xacro            # 双手 ros2_control
│   ├── dual_revo2.srdf                          # 双手 SRDF
│   ├── dual_revo2_controllers.yaml              # 双手控制器配置
│   ├── dual_revo2_initial_positions.yaml        # 双手初始位置
│   ├── dual_revo2_joint_limits.yaml             # 双手关节限制
│   ├── dual_revo2_kinematics.yaml               # 双手运动学配置
│   ├── dual_revo2_moveit_controllers.yaml       # 双手 MoveIt 控制器
│   ├── moveit.rviz                              # RViz 配置
│   └── pilz_cartesian_limits.yaml               # Pilz 规划器限制
├── CMakeLists.txt                               # 构建配置
├── package.xml                                  # 功能包描述
├── README.md                                    # 英文说明
└── README_CN.md                                 # 中文说明
```

## 相关功能包

- **revo2_description**: Revo2 灵巧手的 URDF 模型
- **brainco_gazebo**: Revo2 灵巧手的 Gazebo 仿真

## 许可证

本项目采用 Apache License 2.0 许可证 - 详见 [LICENSE](LICENSE) 文件。
