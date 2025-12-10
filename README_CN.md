# BrainCo Hand ROS2 项目

[English](README.md) | [简体中文](README_CN.md)


## 项目概述

BrainCo Hand ROS2 是一个完整的 ROS 2 软件包集合，为 BrainCo Revo2 灵巧手提供从硬件驱动到仿真、运动规划的全套解决方案。本项目基于 ROS 2 Humble 开发，支持单手和双手配置，提供硬件控制、Gazebo 仿真、MoveIt 运动规划等功能。

### 主要特性

- **完整的硬件驱动支持**：支持 Modbus 、CAN FD 和 Ethercat 通信协议
- **ros2_control 集成**：完整的 ros2_control 硬件接口实现
- **Gazebo 仿真环境**：基于 Ignition Gazebo 6 的高保真物理仿真
- **MoveIt 运动规划**：完整的 MoveIt 2 集成，支持高级运动规划
- **实时控制**：高频控制循环，实现精确的手指操控
- **双手支持**：支持左手、右手以及双手同时控制
- **机械臂集成**：提供 RM65 机械臂与 Revo2 灵巧手的集成演示

## 系统要求

### 基础环境

- **操作系统**：Ubuntu 22.04
- **ROS 版本**：ROS 2 Humble
- **Python 版本**：Python 3.8+

### 硬件要求（硬件控制模式）

#### Modbus 模式（默认）
- Modbus 串口设备（如 `/dev/ttyUSB0`）
- 串口权限配置（用户需在 `dialout` 组中）

#### CAN FD 模式（可选）
- ZLG USB-CAN FD 设备（如 USBCANFD-200U）
- CAN FD 总线连接

#### EtherCAT 模式（可选）
- EtherCAT 主站（IgH EtherCAT Master）
- EtherCAT 主站服务配置（`ethercat` 系统服务）

### 仿真环境要求

- **Gazebo**：Ignition Gazebo 6（用于仿真功能包）
- **MoveIt 2**：用于运动规划功能包

## 项目结构

本项目包含以下主要功能包：

```
brainco_hand_ros2/
├── revo2_description/                    # Revo2 灵巧手 URDF 描述包
├── brainco_hardware/                     # 硬件驱动包
│   ├── brainco_hand_driver/            # Revo2 灵巧手硬件驱动
│   └── stark_ethercat/brainco_hand_ethercat_driver/                  # EtherCAT 驱动
├── brainco_gazebo/                      # Gazebo 仿真包
├── brainco_moveit_config/               # MoveIt 配置包
└── revo2_with_rm65_demo/                # RM65 机械臂集成演示
    ├── gazebo_rm_65_6f_with_revo2_demo/ # RM65+Revo2 Gazebo 仿真
    └── rm65_with_revo2_right_moveit_config/ # RM65+Revo2 MoveIt 配置
```

## 功能包说明

### 1. revo2_description

**功能**：Revo2 灵巧手的 URDF 模型描述包，提供机器人模型的 3D 可视化。

**主要特性**：
- 左右手 URDF 模型
- RViz 可视化支持
- 完整的关节和链接定义

**详细文档**：请参阅 [revo2_description/README_CN.md](revo2_description/README_CN.md)

### 2. brainco_hardware

**功能**：Revo2 灵巧手的硬件驱动包，提供基于 ros2_control 的硬件接口。

**主要特性**：
- 支持 Modbus 、CAN FD 和 Ethercat 通信协议
- 完整的 ros2_control 硬件接口实现
- 支持单手和双手配置
- MoveIt 集成支持
- 实时关节位置和速度反馈

**详细文档**：请参阅 [brainco_hardware/brainco_hand_driver/README_CN.md](brainco_hardware/brainco_hand_driver/README_CN.md)

**Ethercat 详细文档**：请参阅 [brainco_hardware/stark_ethercat/brainco_hand_ethercat_driver/README_CN.md](brainco_hardware/stark_ethercat/brainco_hand_ethercat_driver/README_CN.md)

### 3. brainco_gazebo

**功能**：Revo2 灵巧手的 Gazebo 仿真包，提供完整的物理仿真环境。

**主要特性**：
- 基于 Ignition Gazebo 6 的物理仿真
- 支持单手和双手仿真
- Gazebo-MoveIt 整合
- RViz 可视化集成

**详细文档**：请参阅 [brainco_gazebo/README_CN.md](brainco_gazebo/README_CN.md)

### 4. brainco_moveit_config

**功能**：Revo2 灵巧手的 MoveIt 配置包，提供运动规划功能。

**主要特性**：
- 支持左手、右手和双手配置
- 完整的 MoveIt 2 集成
- FakeSystem 支持

**详细文档**：请参阅 [brainco_moveit_config/README_CN.md](brainco_moveit_config/README_CN.md)

### 5. revo2_with_rm65_demo

**功能**：RM65 机械臂与 Revo2 灵巧手的集成演示包。

**包含子包**：

#### 5.1 gazebo_rm_65_6f_with_revo2_demo

**功能**：RM65 机械臂与 Revo2 灵巧手的 Gazebo 仿真。

**详细文档**：请参阅 [revo2_with_rm65_demo/gazebo_rm_65_6f_with_revo2_demo/README_CN.md](revo2_with_rm65_demo/gazebo_rm_65_6f_with_revo2_demo/README_CN.md)

#### 5.2 rm65_with_revo2_right_moveit_config

**功能**：RM65 机械臂与 Revo2 灵巧右手的 MoveIt 配置。

**详细文档**：请参阅 [revo2_with_rm65_demo/rm65_with_revo2_right_moveit_config/README_CN.md](revo2_with_rm65_demo/rm65_with_revo2_right_moveit_config/README_CN.md)

## 快速开始

### 1. 环境准备

```bash
# 检查 ROS 2 环境
echo $ROS_DISTRO  # 应该输出: humble

# 创建工作空间（如果还没有）
mkdir -p <workspace>/src
cd <workspace>
```

**注意**：请将 `<workspace>` 替换为您的工作空间路径，例如 `~/brainco_ws`。

### 2. 克隆仓库

```bash
# 进入工作空间 src 目录
cd <workspace>/src

# 克隆主仓库
git clone https://github.com/BrainCoTech/brainco_hand_ros2.git
cd brainco_hand_ros2

# 使用 vcs 工具克隆依赖仓库
vcs import . < brainco_hand.repos --recursive --skip-existing
```

**说明**：如果您的系统没有安装 `vcs` 工具，请先安装：
```bash
sudo apt-get install python3-vcstool
```

### 3. 安装依赖并构建

```bash
# 返回工作空间根目录
cd <workspace>

# 更新软件包列表
sudo apt-get update

# 更新 rosdep 数据库
rosdep update

# 安装所有依赖项
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# 构建所有功能包
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error

# 激活工作空间
source install/setup.bash
```

## 使用场景

### 场景 1：硬件控制（真实灵巧手）

如果您有真实的 Revo2 灵巧手硬件，可以使用 `brainco_hand_driver` 进行控制：

```bash
# 启动右手系统（Modbus 模式）
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right

# 启动右手系统（带 MoveIt）
ros2 launch brainco_hand_driver revo2_real_moveit.launch.py hand_type:=right

# 启动双手系统
ros2 launch brainco_hand_driver dual_revo2_real_moveit.launch.py
```

**详细说明**：请参阅 [brainco_hardware/brainco_hand_driver/README_CN.md](brainco_hardware/brainco_hand_driver/README_CN.md)

### 场景 2：Gazebo 仿真

如果您想在没有硬件的情况下进行开发和测试，可以使用 Gazebo 仿真：

```bash
# 启动单手仿真
ros2 launch brainco_gazebo revo2_hand_gazebo.launch.py hand_type:=right

# 启动双手仿真
ros2 launch brainco_gazebo dual_revo2_hand_gazebo.launch.py

# 启动带 MoveIt 的仿真
ros2 launch brainco_gazebo revo2_hand_gazebo_moveit.launch.py hand_type:=right
```

**详细说明**：请参阅 [brainco_gazebo/README_CN.md](brainco_gazebo/README_CN.md)

### 场景 3：MoveIt 运动规划（无硬件）

如果您想使用 MoveIt 进行运动规划，但不需要硬件或仿真，可以使用 FakeSystem：

```bash
# 启动右手 MoveIt（FakeSystem）
ros2 launch brainco_moveit_config revo2_right_moveit.launch.py

# 启动双手 MoveIt（FakeSystem）
ros2 launch brainco_moveit_config dual_revo2_moveit.launch.py
```

**详细说明**：请参阅 [brainco_moveit_config/README_CN.md](brainco_moveit_config/README_CN.md)

### 场景 4：机械臂集成演示（FakeSystem）

如果您想测试 RM65 机械臂与 Revo2 灵巧手的集成：

```bash
# 启动 Gazebo 仿真
ros2 launch gazebo_rm_65_6f_with_revo2_demo gazebo_rm_65_6f_with_revo2.launch.py

# 启动 MoveIt 配置（FakeSystem）
ros2 launch rm65_with_revo2_right_moveit_config rm65_with_revo2_right_moveit.launch.py
```

**详细说明**：
- [revo2_with_rm65_demo/gazebo_rm_65_6f_with_revo2_demo/README_CN.md](revo2_with_rm65_demo/gazebo_rm_65_6f_with_revo2_demo/README_CN.md)
- [revo2_with_rm65_demo/rm65_with_revo2_right_moveit_config/README_CN.md](revo2_with_rm65_demo/rm65_with_revo2_right_moveit_config/README_CN.md)

## 双手 MoveIt 仿真示意

![示意](doc/dual_hand_gazebo_moveit.gif)

## 控制接口

### Topic 接口

所有功能包都提供标准的 ROS 2 Topic 接口：

- **关节状态**：`/joint_states` (sensor_msgs/JointState)
- **轨迹命令**：`/xxx_revo2_hand_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)

### Action 接口

- **轨迹执行**：`/xxx_revo2_hand_controller/follow_joint_trajectory` (control_msgs/action/FollowJointTrajectory)

### 控制示例

#### 右手张开手掌

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
```

#### 右手握拳

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
      positions: [0.8, 0.1, 1.4, 1.4, 1.4, 1.4],
      time_from_start: {sec: 2}
    }]
  }'
```

## 监控和调试

### 系统状态检查

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

## 联系方式

如有问题或建议，请联系开发团队。
