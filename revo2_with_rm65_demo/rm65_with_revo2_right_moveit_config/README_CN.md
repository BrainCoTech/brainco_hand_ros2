<div align="right">

[简体中文](README_CN.md)|[English](README.md)

</div>

<div align="center">

# RM65 搭配 Revo2 右手 MoveIt 配置包

</div>

## 目录
* 1.[功能包概述](#功能包概述)
* 2.[系统要求](#系统要求)
* 3.[安装配置](#安装配置)
* 4.[快速开始](#快速开始)
* 5.[运动规划](#运动规划)
* 6.[控制接口](#控制接口)
* 7.[配置说明](#配置说明)

## 功能包概述

rm65_with_revo2_right_moveit_config 功能包为配备 BrainCo Revo2 灵巧右手的 RM65 机械臂提供完整的 MoveIt 2 配置。该功能包支持运动规划、操作控制和可视化，使用户能够通过 MoveIt 2 控制集成 RM65-Revo2 系统。

### 主要特性

- **集成机器人配置**: RM65 机械臂与 Revo2 灵巧右手的完整 MoveIt 配置
- **运动规划**: 基于 MoveIt 2 的完整运动规划功能
- **RViz 集成**: 内置 RViz 配置，支持可视化和交互式规划
- **ROS2 控制支持**: 与 ros2_control 兼容，支持硬件接口管理
- **多种规划算法**: 支持多种运动规划器（OMPL、Pilz 等）

## 系统环境

- Ubuntu 22.04
- ROS 2 (Humble/Iron)
- MoveIt 2
- 依赖包：`gazebo_rm_65_6f_with_revo2_demo`、`revo2_description`

## 安装配置

### 1. 工作空间设置

```bash
# 创建 ROS 2 工作空间（如果不存在）
mkdir -p ~/brainco_ws/src
cd ~/brainco_ws/src

# 克隆或复制所需功能包
# - rm65_with_revo2_right_moveit_config
# - gazebo_rm_65_6f_with_revo2_demo
# - revo2_description

# 安装依赖
rosdep install --ignore-src --from-paths . -y -r

# 构建工作空间
cd ~/brainco_ws
colcon build --packages-select rm65_with_revo2_right_moveit_config gazebo_rm_65_6f_with_revo2_demo revo2_description --symlink-install
source install/setup.bash
```

### 2. 验证安装

```bash
# 检查功能包是否可用
ros2 pkg list | grep -E "(rm65_with_revo2_right_moveit_config|gazebo_rm_65_6f_with_revo2_demo|revo2_description)"
```

## 快速开始

### 启动 MoveIt 与 RViz

```bash
# 加载工作空间
source ~/brainco_ws/install/setup.bash

# 启动带有 RViz 界面的 MoveIt
ros2 launch rm65_with_revo2_right_moveit_config rm65_with_revo2_right_moveit.launch.py
```

## 启动参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `use_sim_time` | `false` | 使用仿真时间 |
| `pipeline` | `ompl` | 运动规划管道（ompl/pilz） |

## 运动规划

### 规划组

| 规划组 | 描述 | 关节数 |
|--------|------|--------|
| `rm65_arm` | RM65 机械臂 | 6 个关节（肩部到腕部） |
| `revo2_right_hand` | Revo2 灵巧右手 | 6 个关节（拇指、手指） |
| `rm65_with_revo2_right` | 组合臂和手 | 全部 12 个关节 |

## 配置说明

### SRDF 配置
- **文件**: `config/rm_65_revo2_right.srdf`
- **用途**: 语义机器人描述，包含规划组、链路关系和碰撞设置

### 运动学配置
- **文件**: `config/kinematics.yaml`
- **用途**: 运动规划的运动学求解器设置

### 关节限位
- **文件**: `config/joint_limits.yaml`
- **用途**: 关节速度和加速度限位

### MoveIt 控制器
- **文件**: `config/moveit_controllers.yaml`
- **用途**: 轨迹执行的控制器配置

## 功能包结构

```
rm65_with_revo2_right_moveit_config/
├── launch/                                    # ROS 启动文件
│   ├── demo.launch.py                        # 完整演示启动
│   ├── move_group.launch.py                  # MoveIt 运动组启动
│   ├── moveit_rviz.launch.py                 # RViz 界面启动
│   └── rm65_with_revo2_right_moveit.launch.py # 主 MoveIt 启动
├── config/                                    # MoveIt 配置文件
│   ├── rm_65_revo2_right.srdf                # 语义机器人描述
│   ├── rm_65_revo2_right.urdf.xacro          # URDF 描述
│   ├── kinematics.yaml                       # 运动学配置
│   ├── joint_limits.yaml                     # 关节限位
│   ├── moveit_controllers.yaml               # 控制器配置
│   ├── ros2_controllers.yaml                 # ROS2 控制配置
│   ├── moveit.rviz                          # RViz 配置
│   └── pilz_cartesian_limits.yaml           # Pilz 规划器的笛卡尔限位
├── CMakeLists.txt                           # 构建配置
├── package.xml                              # 功能包描述
├── CHANGELOG.rst                            # 版本变更日志
├── LICENSE                                  # 许可证文件
├── README.md                                # 英文文档
└── README_CN.md                             # 中文文档
```

## 许可证

本项目采用 Apache License 2.0 许可证 - 详见 [LICENSE](LICENSE) 文件。
