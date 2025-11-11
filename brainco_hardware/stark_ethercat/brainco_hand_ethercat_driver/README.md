# BrainCo Hand EtherCAT Driver

[English](README.md) | [简体中文](README_CN.md)

## Overview

The BrainCo Hand EtherCAT Driver package provides a ROS2 hardware interface for BrainCo Revo2 dexterous hands using EtherCAT communication protocol. This package implements a ros2_control hardware interface that enables real-time control and monitoring of 6 finger joints through EtherCAT master.

## Features

- **EtherCAT Communication**: Real-time EtherCAT communication with Revo2 hand hardware
- **ros2_control Integration**: Full ros2_control hardware interface implementation
- **Dual Hand Support**: Support for both left and right hand configurations
- **MoveIt Integration**: Optional MoveIt integration for motion planning
- **Real-time Control**: High-frequency control loop for precise finger manipulation
- **State Feedback**: Position, velocity, and effort feedback from all joints
- **Trajectory Control**: Joint trajectory controller support for smooth motion execution

## Environment

- Ubuntu 22.04
- ROS 2 (Humble)
- EtherCAT Master (IgH EtherCAT Master)
- Python 3.8+

## Installation and Setup

### 1. Dependencies

Make sure you have the following packages installed:

```bash
# ROS2 dependencies
sudo apt install ros-humble-controller-manager ros-humble-joint-trajectory-controller
sudo apt install ros-humble-joint-state-broadcaster ros-humble-robot-state-publisher

# Python dependencies
pip3 install rclpy trajectory_msgs sensor_msgs control_msgs

# EtherCAT Master (if not already installed)
# Follow EtherCAT Master installation guide for your system
```

### 2. Build the Workspace

```bash
# Navigate to workspace
cd ~/brainco_ws

# Install dependencies
rosdep install --ignore-src --from-paths src -y -r

# Build the package
colcon build --packages-select brainco_hand_ethercat_driver stark_ethercat_interface stark_ethercat_driver --symlink-install

# Source the workspace
source install/setup.bash
```

### 3. Verify Installation

```bash
# Check if package is available
ros2 pkg list | grep brainco_hand_ethercat_driver

# Check EtherCAT master status
sudo systemctl status ethercat
```

## Hardware Connection

### EtherCAT Setup

Before using the driver, ensure your EtherCAT setup is correct:

```bash
# Check network interface
ip link show

# Check EtherCAT master status
sudo systemctl status ethercat

# Check EtherCAT slave devices
sudo ethercat slaves

# Check device SDO/PDO information
sudo ethercat sdos
sudo ethercat pdos
```

## Quick Start

### Launch Right Hand System

```bash
ros2 launch brainco_hand_ethercat_driver revo2_system.launch.py hand_type:=right
```

### Launch Left Hand System

```bash
ros2 launch brainco_hand_ethercat_driver revo2_system.launch.py hand_type:=left
```

### Launch Parameters

All launch files support the following parameters:

#### Basic System Parameters (revo2_system.launch.py)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hand_type` | `right` | Hand type: `left` or `right` |
| `prefix` | `""` | Prefix for joint names, useful for multi-robot setup |
| `ctrl_param_duration_ms` | `20` | Motion time parameter in milliseconds (1ms = fastest speed) |
| `robot_controller` | Auto-generated | Controller name (auto-generated based on hand_type) |

#### MoveIt Integration Parameters (revo2_real_moveit.launch.py)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hand_type` | `right` | Hand type: `left` or `right` |
| `ctrl_param_duration_ms` | `10` | Motion time parameter in milliseconds (1ms = fastest speed) |
| `use_rviz` | `true` | Whether to launch RViz visualization |
| `publish_monitored_planning_scene` | `true` | Whether to publish monitored planning scene |

## Control Interface

### Using Topic Interface

For right hand trajectory control:

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

For left hand trajectory control:

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

### Using Action Interface

For right hand action control:

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

For left hand action control:

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

### Home Position (Zero Position)

To return to home position:

```bash
# Right hand
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

# Left hand
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

## Test Scripts

The package includes a test script for interactive testing:

```bash
# Navigate to scripts directory
cd <package_path>/scripts/

# Interactive menu
python3 test_revo2_ethercat.py --menu

# Direct test commands
python3 test_revo2_ethercat.py --test all_fingers --amplitude 0.8
python3 test_revo2_ethercat.py --test individual
python3 test_revo2_ethercat.py --test home
```

## Monitoring and Debugging

### Check System Status

```bash
# List all running nodes
ros2 node list

# List all controllers
ros2 control list_controllers

# Check hardware components
ros2 control list_hardware_components

# Check hardware interfaces
ros2 control list_hardware_interfaces

# List all topics
ros2 topic list

# List all actions
ros2 action list

# Monitor joint states
ros2 topic echo /joint_states
```

### Expected Output

#### Core Node List (Right Hand Configuration)
```bash
/controller_manager
/joint_state_broadcaster
/right_revo2_hand_controller
/robot_state_publisher
```

#### Controller Status
```bash
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster          active
right_revo2_hand_controller   joint_trajectory_controller/JointTrajectoryController  active
```

#### Hardware Components (Right Hand Configuration)
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

#### Hardware Interfaces (Right Hand Configuration)
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

#### Core Topic List (Right Hand Configuration)
```bash
/joint_states
/right_revo2_hand_controller/joint_trajectory
/robot_description
/tf
/tf_static
```

#### Core Action List (Right Hand Configuration)
```bash
/right_revo2_hand_controller/follow_joint_trajectory
```

## Joint Mapping

### Right Hand Joints

| Joint Name | Description | Angle Range (rad) | Max Angle (deg) |
|------------|-------------|-------------------|-----------------|
| `right_thumb_proximal_joint` | Thumb proximal joint | 0 ~ 1.03 | 59 |
| `right_thumb_metacarpal_joint` | Thumb metacarpal joint | 0 ~ 1.57 | 90 |
| `right_index_proximal_joint` | Index finger proximal joint | 0 ~ 1.41 | 81 |
| `right_middle_proximal_joint` | Middle finger proximal joint | 0 ~ 1.41 | 81 |
| `right_ring_proximal_joint` | Ring finger proximal joint | 0 ~ 1.41 | 81 |
| `right_pinky_proximal_joint` | Pinky finger proximal joint | 0 ~ 1.41 | 81 |

### Left Hand Joints

| Joint Name | Description | Angle Range (rad) | Max Angle (deg) |
|------------|-------------|-------------------|-----------------|
| `left_thumb_proximal_joint` | Thumb proximal joint | 0 ~ 1.03 | 59 |
| `left_thumb_metacarpal_joint` | Thumb metacarpal joint | 0 ~ 1.57 | 90 |
| `left_index_proximal_joint` | Index finger proximal joint | 0 ~ 1.41 | 81 |
| `left_middle_proximal_joint` | Middle finger proximal joint | 0 ~ 1.41 | 81 |
| `left_ring_proximal_joint` | Ring finger proximal joint | 0 ~ 1.41 | 81 |
| `left_pinky_proximal_joint` | Pinky finger proximal joint | 0 ~ 1.41 | 81 |

## Package Structure

```
brainco_hand_ethercat_driver/
├── launch/                                      # Launch files
│   ├── revo2_system.launch.py                    # Main system launch
│   └── revo2_real_moveit.launch.py               # MoveIt integration launch
├── config/                                      # Configuration files
│   ├── revo2_left.urdf.xacro                    # Left hand URDF
│   ├── revo2_left.ros2_control.xacro            # Left hand ros2_control
│   ├── revo2_left.srdf                          # Left hand SRDF
│   ├── revo2_left_controllers.yaml              # Left hand controllers
│   ├── revo2_left_initial_positions.yaml        # Left hand initial positions
│   ├── revo2_left_joint_limits.yaml             # Left hand joint limits
│   ├── revo2_left_kinematics.yaml               # Left hand kinematics
│   ├── revo2_left_moveit_controllers.yaml       # Left hand MoveIt controllers
│   ├── revo2_right.urdf.xacro                   # Right hand URDF
│   ├── revo2_right.ros2_control.xacro           # Right hand ros2_control
│   ├── revo2_right.srdf                         # Right hand SRDF
│   ├── revo2_right_controllers.yaml             # Right hand controllers
│   ├── revo2_right_initial_positions.yaml       # Right hand initial positions
│   ├── revo2_right_joint_limits.yaml            # Right hand joint limits
│   ├── revo2_right_kinematics.yaml              # Right hand kinematics
│   ├── revo2_right_moveit_controllers.yaml     # Right hand MoveIt controllers
│   ├── moveit.rviz                              # RViz configuration
│   └── pilz_cartesian_limits.yaml                # Pilz planner limits
├── include/                                     # Header files
│   └── revo2_ethercat_plugins/                  # Plugin headers
│       ├── execute_command.hpp
│       ├── logger_macros.hpp
│       └── revo2_joints_system_slave.hpp
├── src/                                         # Source files
│   └── revo2_joints_system_slave.cpp            # Hardware interface implementation
├── scripts/                                     # Utility scripts
│   └── test_revo2_ethercat.py                   # Test script
├── CMakeLists.txt                               # Build configuration
├── package.xml                                  # Package description
├── revo2_plugins.xml                            # Plugin description
└── README.md                                    # This file
```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Contact

If you have any questions or suggestions, please contact the development team.