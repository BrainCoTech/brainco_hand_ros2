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
- **自动检测**：支持 Modbus 自动检测串口和从站 ID（支持单手和双手模式）

## 环境系统

- Ubuntu 22.04
- ROS 2 (Humble)
- Python 3.8+

### Modbus 模式（默认）
- Modbus 串口设备（如 `/dev/ttyUSB0`）

### CAN FD 模式（可选）
- ZLG USB-CAN FD 设备（如 USBCANFD-200U）
- ZLG CAN FD 驱动库（已随包内置）

## 安装和构建


### BrainCo Stark SDK

SDK 位于 `vendor/` 目录，可直接使用。如需更新：

```bash
cd brainco_hardware/brainco_hand_driver
./scripts/download_sdk.sh
```

### 构建工作空间

**推荐：使用编译脚本**

```bash
cd ~/brainco_ws

# 默认编译（仅 Modbus）
./build.sh

# 启用 CAN FD 支持
./build.sh --canfd

# 启用 EtherCAT 支持
./build.sh --ethercat

# Release 模式，启用所有功能
./build.sh --release --canfd --ethercat
```

**使用 colcon 命令**

```bash
cd ~/brainco_ws


# 默认禁用 CAN FD 
colcon build --packages-up-to brainco_hand_driver --symlink-install 

# 启用 CAN FD 支持
colcon build --packages-up-to brainco_hand_driver --symlink-install --cmake-args -DENABLE_CANFD=ON


# Source 工作空间
source install/setup.bash
```

**编译选项说明：**
- 默认：`ENABLE_CANFD=OFF`，仅支持 Modbus
- 启用 CAN FD：使用 `-DENABLE_CANFD=ON`

**注意：** `brainco_hand_driver` 依赖于 `revo2_description` 包。

### 验证安装

```bash
# 检查功能包
ros2 pkg list | grep brainco_hand_driver

# 检查串口权限
ls -l /dev/ttyUSB*
# 如权限不足，添加用户到 dialout 组：
sudo usermod -a -G dialout $USER
# 然后重新登录
```

## 硬件连接

### Modbus 串口设置

```bash
# 检查串口设备
ls -l /dev/ttyUSB*

# 检查串口权限
groups | grep dialout

# 添加用户到 dialout 组（如需要）
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### Modbus 自动检测

驱动支持自动检测串口和从站 ID，适用于：
- 串口设备路径不固定的场景
- 双手配置场景，自动识别左右手设备
- 设备热插拔后端口号变化

**启用自动检测：**

编辑配置文件 `config/protocol_modbus_right.yaml`：

```yaml
hardware:
  protocol: modbus
  slave_id: 127  # 右手通常使用 127，左手使用 126
  auto_detect: true
  auto_detect_quick: true  # 快速检测模式（推荐）
  auto_detect_port: ""  # 端口提示，留空检测所有端口，或指定如 "/dev/ttyUSB"
  # 当 auto_detect 为 true 时，以下参数会被忽略
  port: /dev/ttyUSB0
  baudrate: 460800
```

**参数说明：**

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `auto_detect` | bool | `false` | 是否启用自动检测 |
| `auto_detect_quick` | bool | `true` | 快速检测模式。`false` 会检测所有可能的 slave_id（1-247），速度较慢 |
| `auto_detect_port` | string | `""` | 端口提示。留空检测所有端口，或指定如 `"/dev/ttyUSB"` 只检测 USB 端口 |

**双手配置：**

在双手配置中，每个硬件实例独立进行自动检测，通过 `slave_id` 区分左右手设备：

- 左手配置 (`config/protocol_modbus_left.yaml`)：`slave_id: 126`
- 右手配置 (`config/protocol_modbus_right.yaml`)：`slave_id: 127`


## 快速开始

### 启动右手系统

```bash
# Modbus 模式（默认）
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right

# CAN FD 模式
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right protocol:=canfd
```

### 启动左手系统

```bash
# Modbus 模式
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=left

# CAN FD 模式
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=left protocol:=canfd
```

### 启动双手系统

```bash
# 使用默认配置（自动检测） Modbus 模式
ros2 launch brainco_hand_driver dual_revo2_system.launch.py

# 自定义协议配置文件
ros2 launch brainco_hand_driver dual_revo2_system.launch.py \
    left_protocol_config_file:=/path/to/protocol_modbus_left.yaml \
    right_protocol_config_file:=/path/to/protocol_modbus_right.yaml

# 如果启用了自动检测（`auto_detect: true`），驱动会自动扫描并找到左右手设备。确保左右手使用不同的 `slave_id`（左手 126，右手 127）。

# CAN FD 模式
ros2 launch brainco_moveit_config dual_revo2_real_moveit.launch.py protocol:=canfd
```

### Launch 参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `hand_type` | `right` | 手型选择：`left` 或 `right` |
| `protocol` | `modbus` | 通信协议：`modbus` 或 `canfd` |
| `protocol_config_file` | `""` | 协议配置文件（YAML），留空则使用默认配置 |

## 控制接口

### Topic 接口

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

# 监控关节状态
ros2 topic echo /joint_states
```

### 预期的输出

**核心节点列表（右手配置）：**
```
/controller_manager
/joint_state_broadcaster
/right_revo2_hand_controller
/robot_state_publisher
```

**控制器状态（右手配置）：**
```
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster          active
right_revo2_hand_controller   joint_trajectory_controller/JointTrajectoryController  active
```

**硬件组件（右手配置）：**
```
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

**核心话题列表（右手配置）：**
```
/joint_states
/right_revo2_hand_controller/joint_trajectory
/robot_description
/tf
/tf_static
```

**核心动作列表（右手配置）：**
```
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

**注意：** 左手关节名称将 `right_` 替换为 `left_`，角度范围相同。

## 功能包结构

```
brainco_hand_driver/
├── launch/                                      # Launch 文件
│   ├── revo2_system.launch.py                    # 单手机器人启动文件
│   └── dual_revo2_system.launch.py               # 双手机器人启动文件
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
