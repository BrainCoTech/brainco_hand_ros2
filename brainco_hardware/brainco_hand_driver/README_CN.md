# BrainCo Hand Driver 驱动包

[English](README.md) | [简体中文](README_CN.md)

## 概述

BrainCo Hand Driver 驱动包为 BrainCo Revo2 灵巧手提供基于 Modbus/CAN FD 通信协议的 ROS2 硬件接口。该功能包实现了 ros2_control 硬件接口，支持通过串口或 CAN FD 对 6 个手指关节进行实时控制和状态监控。

**通信协议支持：**
- **Modbus**（默认）：通过串口通信，兼容性好，部署简单
- **CAN FD**：支持同时控制多只手，需 ZLG CAN FD 硬件设备

## 特性

- **通信协议**：支持 Modbus 和 CAN FD 双协议
- **ros2_control 集成**：完整的 ros2_control 硬件接口实现
- **双手支持**：支持左手和右手配置，以及双手同时控制
- **实时控制**：高频控制循环，实现精确的手指操控
- **状态反馈**：所有关节的位置、速度反馈
- **轨迹控制**：关节轨迹控制器支持，实现平滑运动执行
- **自动检测**：支持 Modbus 自动检测串口和从站 ID（仅单手模式）

## 环境系统

- Ubuntu 22.04
- ROS 2 (Humble)
- Python 3.8+

### Modbus 模式（默认）
- Modbus 串口设备（如 /dev/ttyUSB0）

### CAN FD 模式（可选）
- ZLG USB-CAN FD 设备（如 USBCANFD-200U）
- ZLG CAN FD 驱动库（已随包内置）
- CAN FD 总线连接

## 安装和设置

### 1. 依赖项

确保已安装以下软件包：

```bash
# ROS2 依赖项
sudo apt install ros-humble-controller-manager ros-humble-joint-trajectory-controller
sudo apt install ros-humble-joint-state-broadcaster ros-humble-robot-state-publisher

# Python 依赖项
pip3 install rclpy trajectory_msgs sensor_msgs control_msgs
```

### 2. BrainCo Stark SDK

BrainCo Stark SDK 在 `vendor/` 目录中，可直接使用。如需更新 SDK 版本，可运行：

```bash
cd brainco_hardware/brainco_hand_driver
./scripts/download_sdk.sh
```

### 3. 构建工作空间

```bash
# 进入工作空间
cd ~/brainco_ws

# 安装依赖项
rosdep install --ignore-src --from-paths src -y -r

# 方式一：全部编译（默认禁用 CAN FD 支持，仅支持 Modbus）
colcon build --symlink-install

# 方式二：全部编译并启用 CAN FD 支持
colcon build --symlink-install --cmake-args -DENABLE_CANFD=ON

# 方式三：全部编译（Release 模式，启用 CAN FD 支持）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_CANFD=ON

# 方式四：全部编译（Release 模式，禁用 CAN FD 支持）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source 工作空间
source install/setup.bash
```

**编译选项说明：**
- **默认行为**：`ENABLE_CANFD=OFF`（禁用 CAN FD 支持），仅支持 Modbus 模式，不需要 ZLG 库文件
- **启用 CAN FD**：使用 `-DENABLE_CANFD=ON`，需要 ZLG USB-CAN FD 库文件
- **构建模式**：可选的 `-DCMAKE_BUILD_TYPE=Release` 用于 Release 模式构建

**单独编译 brainco_hand_driver 包：**

如果只想编译 `brainco_hand_driver` 包：

```bash
# 单独编译（默认禁用 CAN FD）
colcon build --packages-select brainco_hand_driver --symlink-install

# 单独编译并启用 CAN FD
colcon build --packages-select brainco_hand_driver --cmake-args -DENABLE_CANFD=ON --symlink-install

# 单独编译（Release 模式，启用 CAN FD）
colcon build --packages-select brainco_hand_driver --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_CANFD=ON --symlink-install
```

### 4. 验证安装

```bash
# 检查功能包是否可用
ros2 pkg list | grep brainco_hand_driver

# 检查串口设备权限
ls -l /dev/ttyUSB*
# 如果权限不足，将用户添加到 dialout 组：
sudo usermod -a -G dialout $USER
# 然后重新登录
```

## 硬件连接

### Modbus 串口设置

在使用驱动之前，请确保您的 Modbus 串口设置正确：

```bash
# 检查串口设备
ls -l /dev/ttyUSB*

# 检查串口权限
groups | grep dialout

# 如果没有权限，添加用户到 dialout 组
sudo usermod -a -G dialout $USER
# 重新登录后生效

# 测试串口连接（可选）
sudo apt install minicom
sudo minicom -D /dev/ttyUSB0
```

## 快速开始

### 启动右手系统

```bash
# Modbus 模式（默认，使用协议配置文件中的端口和从站 ID）
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right

# CAN FD 模式（需要 ZLG USB-CAN FD 设备）
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right protocol:=canfd
```

### 启动左手系统

```bash
# Modbus 模式（默认，使用协议配置文件中的端口和从站 ID）
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=left

# CAN FD 模式（需要 ZLG USB-CAN FD 设备）
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=left protocol:=canfd
```

### 启动双手系统（同时控制）

双手系统支持同时控制左右两只手。有两种方式：

#### 方式一：使用 Modbus 模式（单个节点）

使用专门的双手启动文件，在单个节点中同时控制左右手：

```bash
# 使用默认配置（左手：/dev/ttyUSB0，右手：/dev/ttyUSB1）
ros2 launch brainco_hand_driver dual_revo2_system.launch.py

# 自定义协议配置文件
ros2 launch brainco_hand_driver dual_revo2_system.launch.py \
    left_protocol_config_file:=/path/to/protocol_modbus_left.yaml \
    right_protocol_config_file:=/path/to/protocol_modbus.yaml
```

**注意：** 
- 默认配置中，左手使用 `/dev/ttyUSB0`（slave_id: 126），右手使用 `/dev/ttyUSB1`（slave_id: 127）
- 如果您的设备连接在不同的串口，请修改 `protocol_modbus_left.yaml` 和 `protocol_modbus.yaml` 中的 `port` 参数
- 确保左右手使用不同的 `slave_id`（例如左手 126，右手 127）

#### 方式二：使用 CAN FD 模式（推荐用于多手控制）

CAN FD 模式可以在单个节点中同时控制多只手，适合需要控制多只手的场景：

```bash
# 双手系统 CAN FD 模式
ros2 launch brainco_moveit_config dual_revo2_real_moveit.launch.py protocol:=canfd
```

#### 方式三：启动两个独立的节点（Modbus 模式）

如果需要分别控制左右手，也可以启动两个独立的节点：

**终端 1 - 启动左手：**
```bash
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=left
```

**终端 2 - 启动右手：**
```bash
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right
```

**注意：** 使用此方式时，需要确保左右手连接到不同的串口（例如 `/dev/ttyUSB0` 和 `/dev/ttyUSB1`），并在各自的协议配置文件中指定正确的端口。

### Launch 参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `hand_type` | `right` | 手型选择：`left` 或 `right` |
| `protocol` | `modbus` | 通信协议：`modbus` 或 `canfd` |
| `protocol_config_file` | `""` | 协议配置文件（YAML），留空则使用默认配置 |

使用 Modbus 模式时，协议配置文件中的 `port` 和 `slave_id` 需要根据实际硬件连接情况进行修改。

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

### 双手同时控制示例

当双手系统启动后，可以同时控制左右手：

**同时控制双手（Topic 方式）：**

```bash
# 终端 1 - 控制左手
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

# 终端 2 - 同时控制右手
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

**同时控制双手（Action 方式）：**

```bash
# 终端 1 - 控制左手
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
      points: [{
        positions: [0.5, 0.0, 1.4, 1.4, 1.4, 1.4],
        time_from_start: {sec: 3}
      }]
    }
  }'

# 终端 2 - 同时控制右手
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
      points: [{
        positions: [0.5, 0.0, 1.4, 1.4, 1.4, 1.4],
        time_from_start: {sec: 3}
      }]
    }
  }'
```

**监控双手状态：**

```bash
# 查看所有关节状态（包含左右手）
ros2 topic echo /joint_states

# 列出所有控制器
ros2 control list_controllers
# 应该看到：
# left_revo2_hand_controller   joint_trajectory_controller/JointTrajectoryController  active
# right_revo2_hand_controller  joint_trajectory_controller/JointTrajectoryController  active
# joint_state_broadcaster       joint_state_broadcaster/JointStateBroadcaster          active
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

#### 核心节点列表（双手配置）
```bash
/controller_manager
/joint_state_broadcaster
/left_revo2_hand_controller
/right_revo2_hand_controller
/robot_state_publisher
```

#### 控制器状态（右手配置）
```bash
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster          active
right_revo2_hand_controller   joint_trajectory_controller/JointTrajectoryController  active
```

#### 控制器状态（双手配置）
```bash
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster          active
left_revo2_hand_controller   joint_trajectory_controller/JointTrajectoryController   active
right_revo2_hand_controller  joint_trajectory_controller/JointTrajectoryController  active
```

#### 硬件组件（右手配置）
```bash
Hardware Component 1
	name: Revo2RightSystem
	type: system
	plugin name: brainco_hand_driver/BraincoHandHardware
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

#### 核心话题列表（右手配置）
```bash
/joint_states
/right_revo2_hand_controller/joint_trajectory
/robot_description
/tf
/tf_static
```

#### 核心话题列表（双手配置）
```bash
/joint_states
/left_revo2_hand_controller/joint_trajectory
/right_revo2_hand_controller/joint_trajectory
/robot_description
/tf
/tf_static
```

#### 核心动作列表（右手配置）
```bash
/right_revo2_hand_controller/follow_joint_trajectory
```

#### 核心动作列表（双手配置）
```bash
/left_revo2_hand_controller/follow_joint_trajectory
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
brainco_hand_driver/
├── launch/                                      # Launch 文件
│   └── revo2_system.launch.py                    # 主系统启动文件
├── config/                                      # 配置文件
│   ├── protocol_*.yaml                          # 协议配置文件（Modbus/CAN FD）
│   ├── xxx.urdf.xacro                           # URDF XACRO 文件
│   ├── xxx.ros2_control.xacro                   # ros2_control 配置
│   ├── xxx_controllers.yaml                     # 控制器配置
│   ├── xxx_initial_positions.yaml               # 初始位置配置
│   └── ...                                      # 其他配置文件
├── include/                                     # 头文件
│   └── brainco_hand_driver/                     # 驱动头文件
│       ├── brainco_hand_hardware.hpp            # 硬件接口头文件
│       └── logger_macros.hpp                    # 日志宏定义
├── src/                                         # 源文件
│   └── brainco_hand_hardware.cpp                # 硬件接口实现
├── scripts/                                     # 工具脚本
│   └── download_sdk.sh                          # SDK 下载脚本
├── vendor/                                      # BrainCo Stark SDK
│   └── dist/                                    # SDK 分发文件
├── CMakeLists.txt                               # 构建配置
├── package.xml                                  # 功能包描述
├── brainco_hand_driver_plugins.xml              # 插件描述
└── README.md                                    # 英文说明
```

## 许可证

本项目采用 Apache License 2.0 许可证 - 详见 [LICENSE](LICENSE) 文件。

## 联系方式

如有问题或建议，请联系开发团队。
