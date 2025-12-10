<div align="right">

[English](README.md)|[简体中文](README_CN.md)

</div>

# RM65 with Revo2 Right Hand MoveIt Config

## Overview

The RM65 with Revo2 Right Hand MoveIt Config package provides MoveIt 2 configuration files and launch scripts for controlling a BrainCo RM65 robotic arm equipped with a Revo2 dexterous right hand. This package enables motion planning, manipulation, and control of the integrated RM65-Revo2 system using MoveIt 2.

## Features

- **Integrated Robot Configuration**: Complete MoveIt configuration for RM65 arm with Revo2 right hand
- **Motion Planning**: Full motion planning capabilities using MoveIt 2
- **RViz Integration**: Built-in RViz configuration for visualization and interactive planning
- **ROS2 Control Support**: Compatible with ros2_control for hardware interface management
- **Multiple Planning Algorithms**: Support for various motion planners (OMPL, Pilz, etc.)

## Environment

- Ubuntu 22.04
- ROS 2 (Humble/Iron)
- MoveIt 2
- Required packages: `gazebo_rm_65_6f_with_revo2_demo`, `revo2_description`

## Installation and Setup

### 1. Workspace Setup

```bash
# Create ROS 2 workspace (if not exists)
mkdir -p ~/brainco_ws/src
cd ~/brainco_ws/src

# Clone or copy required packages
# - rm65_with_revo2_right_moveit_config
# - gazebo_rm_65_6f_with_revo2_demo
# - revo2_description

# Install dependencies
rosdep install --ignore-src --from-paths . -y -r

# Build the workspace
cd ~/brainco_ws
colcon build --packages-select rm65_with_revo2_right_moveit_config gazebo_rm_65_6f_with_revo2_demo revo2_description --symlink-install
source install/setup.bash
```

### 2. Verify Installation

```bash
# Check if packages are available
ros2 pkg list | grep -E "(rm65_with_revo2_right_moveit_config|gazebo_rm_65_6f_with_revo2_demo|revo2_description)"
```

## Quick Start

### Launch MoveIt with RViz

```bash
# Source the workspace
source ~/brainco_ws/install/setup.bash

# Launch MoveIt with RViz interface
ros2 launch rm65_with_revo2_right_moveit_config rm65_with_revo2_right_moveit.launch.py
```

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation time |
| `pipeline` | `ompl` | Motion planning pipeline (ompl/pilz) |

## Motion Planning

### Planning Groups

| Planning Group | Description | Joints |
|----------------|-------------|--------|
| `rm65_arm` | RM65 robotic arm | 6 joints (shoulder to wrist) |
| `revo2_right_hand` | Revo2 dexterous right hand | 6 joints (thumb, fingers) |
| `rm65_with_revo2_right` | Combined arm and hand | All 12 joints |

## Control Interface

### Controllers

| Controller | Type | Description |
|------------|------|-------------|
| `rm65_controller` | JointTrajectoryController | RM65 arm trajectory control |
| `revo2_right_controller` | JointTrajectoryController | Revo2 right hand trajectory control |

## Configuration Files

### SRDF Configuration
- **File**: `config/rm_65_revo2_right.srdf`
- **Purpose**: Semantic robot description with planning groups, link relationships, and collision settings

### Kinematics Configuration
- **File**: `config/kinematics.yaml`
- **Purpose**: Kinematics solver settings for motion planning

### Joint Limits
- **File**: `config/joint_limits.yaml`
- **Purpose**: Joint velocity and acceleration limits

### MoveIt Controllers
- **File**: `config/moveit_controllers.yaml`
- **Purpose**: Controller configuration for trajectory execution

## Package Structure

```
rm65_with_revo2_right_moveit_config/
├── launch/                                    # ROS launch files
│   ├── demo.launch.py                        # Complete demo launch
│   ├── move_group.launch.py                  # MoveIt move group launch
│   ├── moveit_rviz.launch.py                 # RViz interface launch
│   └── rm65_with_revo2_right_moveit.launch.py # Main MoveIt launch
├── config/                                    # MoveIt configuration files
│   ├── rm_65_revo2_right.srdf                # Semantic robot description
│   ├── rm_65_revo2_right.urdf.xacro          # URDF description
│   ├── kinematics.yaml                       # Kinematics configuration
│   ├── joint_limits.yaml                     # Joint limits
│   ├── moveit_controllers.yaml               # Controller configuration
│   ├── ros2_controllers.yaml                 # ROS2 control configuration
│   ├── moveit.rviz                          # RViz configuration
│   └── pilz_cartesian_limits.yaml           # Cartesian limits for Pilz planner
├── CMakeLists.txt                           # Build configuration
├── package.xml                              # Package description
├── CHANGELOG.rst                            # Version changelog
├── LICENSE                                  # License file
├── README.md                                # English documentation
└── README_CN.md                             # Chinese documentation
```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
