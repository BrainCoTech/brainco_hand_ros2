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

**方式一：使用编译脚本（推荐，简洁方便）**

项目提供了便捷的编译脚本 `build.sh`，可以更方便地控制编译选项：

```bash
# 进入工作空间根目录
cd ~/brainco_ws

# 默认编译（不启用 CAN FD 和 EtherCAT）
./build.sh

# 启用 CAN FD 支持
./build.sh --canfd

# 启用 EtherCAT 支持
./build.sh --ethercat

# 同时启用 CAN FD 和 EtherCAT
./build.sh --canfd --ethercat

# Release 模式编译
./build.sh --release

# Release 模式，启用所有功能
./build.sh --release --canfd --ethercat

# 查看帮助信息
./build.sh --help
```

**方式二：使用 colcon 命令（高级用户）**

如果需要更精细的控制，可以直接使用 colcon 命令：

```bash
# 进入工作空间
cd ~/brainco_ws

# 安装依赖项
rosdep install --ignore-src --from-paths src -y -r

# 方式一：全部编译（默认禁用 CAN FD 和 EtherCAT，仅支持 Modbus）
# 注意：在工作空间根目录编译时需要 --packages-ignore 来跳过 EtherCAT
colcon build --symlink-install --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# 方式二：全部编译并启用 CAN FD 支持（禁用 EtherCAT）
colcon build --symlink-install --cmake-args -DENABLE_CANFD=ON --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# 方式三：全部编译（Release 模式，启用 CAN FD 支持，禁用 EtherCAT）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_CANFD=ON --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# 方式四：全部编译（Release 模式，禁用 CAN FD 和 EtherCAT）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# Source 工作空间
source install/setup.bash
```

**编译选项说明：**
- **默认行为**：`ENABLE_CANFD=OFF`（禁用 CAN FD 支持），EtherCAT 相关包默认不编译，仅支持 Modbus 模式
- **启用 CAN FD**：使用 `-DENABLE_CANFD=ON`，需要 ZLG USB-CAN FD 库文件
- **启用 EtherCAT**：EtherCAT 是独立的 ROS2 包，默认不编译。如需启用，请在工作空间根目录直接编译（不添加 `--packages-ignore`），或使用项目根目录的 `build.sh --ethercat` 脚本
- **构建模式**：可选的 `-DCMAKE_BUILD_TYPE=Release` 用于 Release 模式构建

**单独编译 brainco_hand_driver 包：**

如果只想编译 `brainco_hand_driver` 包，有三种方式：

**方式 A：使用 build.sh 脚本（推荐）**

```bash
# 在工作空间根目录
cd ~/brainco_ws

# 使用 build.sh 脚本会自动处理依赖关系
# 默认编译（不启用 CAN FD 和 EtherCAT）
./build.sh

# 启用 CAN FD 支持
./build.sh --canfd
```

**方式 B：在工作空间根目录使用 --packages-up-to（推荐用于单独编译）**

```bash
# 在工作空间根目录
cd ~/brainco_ws

# 使用 --packages-up-to 会自动编译依赖包（如 revo2_description）
# 单独编译（默认禁用 CAN FD）
colcon build --packages-up-to brainco_hand_driver --symlink-install --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# 单独编译并启用 CAN FD
colcon build --packages-up-to brainco_hand_driver --cmake-args -DENABLE_CANFD=ON --symlink-install --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# 单独编译（Release 模式，启用 CAN FD）
colcon build --packages-up-to brainco_hand_driver --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_CANFD=ON --symlink-install --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver
```

**方式 C：在包目录下直接编译（仅用于快速测试，不推荐）**

```bash
# 进入包目录
cd ~/brainco_ws/src/brainco_hand_ros2/brainco_hardware/brainco_hand_driver

# 编译当前包（默认禁用 CAN FD）
# 注意：在包目录下编译时，只编译当前包，不需要 --packages-ignore
# 但需要确保依赖包（如 revo2_description）已经编译过
colcon build --symlink-install

# 编译当前包并启用 CAN FD
colcon build --symlink-install --cmake-args -DENABLE_CANFD=ON
```

**重要提示：**
- `brainco_hand_driver` 依赖于 `revo2_description` 包
- 如果使用 `--packages-select`，需要先编译依赖包，否则会报错：`Failed to find revo2_description`
- 推荐使用 `--packages-up-to` 或 `build.sh` 脚本，它们会自动处理依赖关系

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

### Modbus 自动检测功能

驱动支持自动检测串口和从站 ID，特别适用于：
- 串口设备路径不固定的场景（如 `/dev/ttyUSB0`, `/dev/ttyUSB1` 等）
- 双手配置场景，需要自动识别左右手设备
- 设备热插拔后端口号可能变化

#### 启用自动检测

编辑配置文件 `config/protocol_modbus_right.yaml`（右手）或 `config/protocol_modbus_left.yaml`（左手）：

```yaml
hardware:
  protocol: modbus
  # slave_id: 当 auto_detect 为 true 时，此值作为提示，实际会使用检测到的设备的 slave_id
  # 如果 auto_detect 为 false，则使用此值作为目标 slave_id
  # 注意：左手通常使用 slave_id 126，右手使用 127
  slave_id: 127
  # 启用自动检测：自动扫描所有可用串口，找到 Revo2 设备
  auto_detect: true
  # 快速检测模式：true 为快速模式（推荐），false 为完整检测（更慢但更准确）
  auto_detect_quick: true
  # 自动检测端口提示：如果指定，只在此端口范围内检测（例如 "/dev/ttyUSB" 会检测所有 ttyUSB*）
  # 留空则检测所有可用串口
  # 对于双手配置，建议为左手和右手指定不同的端口提示，避免冲突
  # 例如：左手使用 "/dev/ttyUSB0"，右手使用 "/dev/ttyUSB2"
  auto_detect_port: ""
  # 当 auto_detect 为 true 时，以下参数会被忽略，使用自动检测到的值
  # 当 auto_detect 为 false 时，使用指定的 port 和 baudrate
  port: /dev/ttyUSB0
  baudrate: 460800
```

#### 自动检测参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `auto_detect` | bool | `false` | 是否启用自动检测 |
| `auto_detect_quick` | bool | `true` | 快速检测模式（推荐）。`false` 会检测所有可能的 slave_id（1-247），速度较慢 |
| `auto_detect_port` | string | `""` | 端口提示。留空检测所有端口，或指定提示如 `"/dev/ttyUSB"` 只检测 USB 端口 |

#### 双手配置中的自动检测

在双手配置中，每个硬件实例（左手/右手）独立进行自动检测，通过 `slave_id` 来区分左右手设备：

**左手配置** (`config/protocol_modbus_left.yaml`)：
```yaml
hardware:
  protocol: modbus
  slave_id: 126  # 左手通常使用 126
  auto_detect: true
  auto_detect_quick: true
  auto_detect_port: ""  # 或指定端口提示，如 "/dev/ttyUSB0"
```

**右手配置** (`config/protocol_modbus_right.yaml`)：
```yaml
hardware:
  protocol: modbus
  slave_id: 127  # 右手通常使用 127
  auto_detect: true
  auto_detect_quick: true
  auto_detect_port: ""  # 或指定端口提示，如 "/dev/ttyUSB2"
```

**最佳实践**：
- 如果两个设备都连接在同一台机器上，建议使用 `auto_detect_port` 提示来限制检测范围，避免冲突
- 确保两个设备的 `slave_id` 不同（左手 126，右手 127）
- 如果检测到的 `slave_id` 与请求的不匹配，驱动会记录警告但继续使用检测到的设备

#### 自动检测工作原理

1. **启动检测**：当 `auto_detect: true` 时，驱动会调用 SDK 的自动检测功能
2. **扫描串口**：扫描所有可用串口（或根据 `auto_detect_port` 提示限制范围）
3. **识别设备**：尝试连接每个串口，识别 Revo2 设备
4. **匹配 slave_id**：
   - 如果检测到的 `slave_id` 与配置的匹配，直接使用
   - 如果不匹配，会给出警告但继续使用检测到的设备
5. **建立连接**：使用检测到的端口和参数建立连接

#### 故障排查

**问题：自动检测失败**

可能原因：
1. 设备未连接或未上电
2. USB 线缆连接不良
3. 设备权限不足
4. 设备被其他进程占用

解决方法：
```bash
# 检查设备是否存在
ls -l /dev/ttyUSB*

# 检查设备权限
sudo chmod 666 /dev/ttyUSB*

# 检查是否有其他进程占用
lsof /dev/ttyUSB*

# 如果权限不足，将用户添加到 dialout 组
sudo usermod -a -G dialout $USER
# 然后重新登录
```

**问题：检测到的 slave_id 不匹配**

可能原因：
1. 设备的 slave_id 配置不正确
2. 连接的是错误的手（左手/右手）

解决方法：
1. 检查设备的实际 slave_id
2. 确认配置文件中的 `slave_id` 是否正确
3. 如果检测到的 slave_id 是正确的，可以忽略警告

**问题：双手配置时检测到错误的设备**

解决方法：
1. 使用 `auto_detect_port` 提示来限制检测范围
2. 确保两个设备的 `slave_id` 不同（左手 126，右手 127）
3. 如果仍然有问题，可以临时禁用自动检测，手动指定端口

#### 禁用自动检测（手动配置）

如果自动检测有问题，可以回退到手动配置：

```yaml
hardware:
  protocol: modbus
  slave_id: 127
  auto_detect: false  # 禁用自动检测
  port: /dev/ttyUSB1  # 手动指定端口
  baudrate: 460800
```

更多详细信息，请参考 [AUTO_DETECT_GUIDE.md](AUTO_DETECT_GUIDE.md) 和 [AUTO_DETECT_USB_PORTS.md](AUTO_DETECT_USB_PORTS.md)。

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
    right_protocol_config_file:=/path/to/protocol_modbus_right.yaml
```

**注意：** 
- 默认配置中，如果启用了自动检测（`auto_detect: true`），驱动会自动扫描并找到左右手设备
- 如果未启用自动检测，默认配置中左手使用 `/dev/ttyUSB0`（slave_id: 126），右手使用 `/dev/ttyUSB1`（slave_id: 127）
- 如果您的设备连接在不同的串口，可以：
  1. **推荐**：启用自动检测（`auto_detect: true`），驱动会自动找到设备
  2. 或者：修改 `protocol_modbus_left.yaml` 和 `protocol_modbus_right.yaml` 中的 `port` 参数
- 确保左右手使用不同的 `slave_id`（例如左手 126，右手 127），自动检测会通过 `slave_id` 来区分左右手设备

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

**注意：** 
- 如果启用了自动检测（`auto_detect: true`），驱动会自动找到设备，无需手动指定端口
- 如果未启用自动检测，需要确保左右手连接到不同的串口（例如 `/dev/ttyUSB0` 和 `/dev/ttyUSB1`），并在各自的协议配置文件中指定正确的端口

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
