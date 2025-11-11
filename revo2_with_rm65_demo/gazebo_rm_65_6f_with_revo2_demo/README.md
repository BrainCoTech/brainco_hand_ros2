<div align="right">

[English](README.md)|[简体中文](README_CN.md)

</div>

# Gazebo RM65-6F with Revo2 Demo

## Overview

The Gazebo RM65-6F with Revo2 Demo package provides a complete Gazebo simulation environment integrating the RM65-6F robotic arm with BrainCo Revo2 dexterous hands. Built on ROS2 and Ignition Gazebo 6, it offers professional-grade robot simulation capabilities for research and development.

## Features

- **Integrated Robot System**: Complete simulation of RM65-6F robotic arm with Revo2 dexterous hands
- **Gazebo Simulation**: High-fidelity physics simulation using Ignition Gazebo 6
- **ROS2 Control Integration**: Full ros2_control support with trajectory controllers
- **RViz Visualization**: Built-in launch files for visualizing robot models in RViz
- **Multi-Controller Support**: Independent control of arm and hand systems

## Environment

- Ubuntu 22.04
- ROS 2 (Humble)
- Ignition Gazebo 6

## Installation and Setup

### 1. Build the Workspace

```bash
# Create workspace (if not exists)
mkdir -p ~/brainco_ws/src
cd ~/brainco_ws/src

# Clone or copy the packages
# Make sure revo2_description is available
# Copy gazebo_rm_65_6f_with_revo2_demo to src/

# Required Packages
rosdep install --ignore-src --from-paths . -y -r

# Build the workspace
cd ~/brainco_ws
colcon build --packages-select revo2_description gazebo_rm_65_6f_with_revo2_demo --symlink-install
source install/setup.bash
```

### 2. Verify Installation

```bash
# Check if packages are available
ros2 pkg list | grep -E "(revo2_description|gazebo_rm_65_6f_with_revo2_demo)"

```

## Quick Start

### Basic Launch

```bash
# Source the workspace
source ~/brainco_ws/install/setup.bash

# Launch the complete simulation
ros2 launch gazebo_rm_65_6f_with_revo2_demo gazebo_rm_65_6f_with_revo2.launch.py
```

### Launch with Parameters

```bash
# Launch without RViz (for performance)
ros2 launch gazebo_rm_65_6f_with_revo2_demo gazebo_rm_65_6f_with_revo2.launch.py use_rviz:=false

# Launch with custom world
ros2 launch gazebo_rm_65_6f_with_revo2_demo gazebo_rm_65_6f_with_revo2.launch.py world:=your_world
```

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model` | `rm_65_revo2` | Robot model name |
| `world` | `empty_world` | Gazebo world file (without .sdf extension) |
| `use_rviz` | `true` | Whether to launch RViz visualization |

## Control Interface

### Controller Overview

| Controller Name | Type | Managed Joints | Function |
|----------------|------|----------------|----------|
| `joint_state_broadcaster` | JointStateBroadcaster | All joints | Joint state publishing |
| `rm_group_controller` | JointTrajectoryController | joint1~joint6 | Arm motion control |
| `revo2_hand_controller` | JointTrajectoryController | Hand 6 main joints | Hand pose control |

### Hand Control Examples

#### Open Hand
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

#### Fist Gesture
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

#### OK Gesture
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

### Arm Control Examples

#### Basic Position Control
```bash
# Move to preset position
ros2 action send_goal /rm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], 
      points: [
        {positions: [0.0, -0.5, 1.0, 0.0, 0.5, 0.0], time_from_start: {sec: 2}}
      ]
    }
  }"

# Return to initial position
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

## Monitoring and Debugging

### System Status Commands

```bash
# List all controllers
ros2 control list_controllers

# Check hardware components
ros2 control list_hardware_components

# Monitor joint states
ros2 topic echo /joint_states

# Monitor controller states
ros2 topic echo /revo2_hand_controller/controller_state

# List all topics
ros2 topic list

# List all services
ros2 service list

# List all actions
ros2 action list
```

### Performance Monitoring

```bash
# Check control frequency
ros2 topic hz /joint_states

# Check controller update frequency
ros2 topic hz /revo2_hand_controller/controller_state

# Monitor system resources
htop
```

## Package Structure

```
gazebo_rm_65_6f_with_revo2_demo/
├── launch/                                    # ROS launch files
│   └── gazebo_rm_65_6f_with_revo2.launch.py  # Main launch file
├── config/                                    # Configuration files
│   ├── gazebo_65_6f_revo2_description_gz.urdf.xacro  # Robot description
│   └── revo2_controllers.yaml                # Controller configuration
├── urdf/                                      # URDF/Xacro files
│   ├── rm_65_gazebo.urdf.xacro              # RM65 arm URDF
│   └── rm_65_with_revo2_right.xacro         # Integrated model
├── worlds/                                    # Gazebo world files
│   └── empty_world.sdf                       # Default world
├── rviz/                                      # RViz configurations
│   └── rm_65_revo2.rviz                      # Default RViz config
├── meshes/                                    # 3D mesh files
│   └── rm_65_arm/                            # RM65 arm meshes
├── CMakeLists.txt                            # Build configuration
├── package.xml                               # Package description
├── CHANGELOG.rst                             # Version changelog
├── LICENSE                                   # License file
├── README.md                                 # English documentation
└── README_CN.md                              # Chinese documentation
```

## API Reference

### Action Interfaces

- **Hand Control**: `/revo2_hand_controller/follow_joint_trajectory`
- **Arm Control**: `/rm_group_controller/follow_joint_trajectory`

### Topic Interfaces

- **Joint States**: `/joint_states` (sensor_msgs/JointState)
- **Trajectory Commands**: `/revo2_hand_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)
- **Controller States**: `/revo2_hand_controller/controller_state`

### Service Interfaces

- **Controller Management**: `/controller_manager/list_controllers`
- **Hardware Interfaces**: `/controller_manager/list_hardware_interfaces`

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.