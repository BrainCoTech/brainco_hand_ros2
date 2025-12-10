# BrainCo MoveIt Config

[English](README.md) | [简体中文](README_CN.md)

## Overview

The BrainCo MoveIt Config package provides complete MoveIt configurations for BrainCo Revo2 dexterous hands. It supports three configurations:

- **Single Left Hand**: MoveIt configuration for left Revo2 hand
- **Single Right Hand**: MoveIt configuration for right Revo2 hand  
- **Dual Hands**: MoveIt configuration for both left and right Revo2 hands

All configurations use `mock_components/GenericSystem` as the FakeSystem backend, allowing you to test and develop with MoveIt without real hardware.

## Features

- **FakeSystem Support**: Use mock hardware interface for testing without real robots
- **Complete MoveIt Integration**: Full MoveIt planning and execution pipeline
- **RViz Visualization**: Built-in RViz configurations for visualization
- **Multiple Configurations**: Support for left, right, and dual hand setups
- **Predefined Poses**: Hand open, half close, and full close poses
- **ros2_control Integration**: Full ros2_control support with trajectory controllers

## Environment

- Ubuntu 22.04
- ROS 2 (Humble)
- MoveIt 2
- revo2_description package

## Installation and Setup

### 1. Dependencies

Make sure you have the following packages installed:

```bash
sudo apt install ros-humble-moveit ros-humble-moveit-visual-tools
sudo apt install ros-humble-controller-manager ros-humble-joint-trajectory-controller
```

### 2. Build the Workspace

```bash
# Navigate to workspace
cd ~/brainco_ws

# Install dependencies
rosdep install --ignore-src --from-paths src -y -r

# Build the package
colcon build --packages-select brainco_moveit_config --symlink-install

# Source the workspace
source install/setup.bash
```

### 3. Verify Installation

```bash
# Check if package is available
ros2 pkg list | grep brainco_moveit_config
```

## Quick Start

### Launch Single Left Hand

```bash
ros2 launch brainco_moveit_config revo2_left_moveit.launch.py
```

### Launch Single Right Hand

```bash
ros2 launch brainco_moveit_config revo2_right_moveit.launch.py
```

### Launch Dual Hands

```bash
ros2 launch brainco_moveit_config dual_revo2_moveit.launch.py
```

### Launch Parameters

All launch files support the following parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_rviz` | `true` | Whether to launch RViz visualization |
| `publish_frequency` | `15.0` | TF publish frequency (Hz) |
| `allow_trajectory_execution` | `true` | Whether to allow trajectory execution |
| `publish_monitored_planning_scene` | `true` | Whether to publish monitored planning scene |

Example with parameters:

```bash
ros2 launch brainco_moveit_config revo2_right_moveit.launch.py use_rviz:=true
```

## Control Interface

### Using Action Interface

For right hand:

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

For left hand:

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

## Predefined Hand Poses

Each configuration includes the following predefined poses:

### Single Hand Configurations

- `hand_open`: Fully open hand
- `hand_half_close`: Half closed hand
- `hand_close`: Fully closed hand (fist)

### Dual Hand Configuration

- `left_hand_open` / `right_hand_open`: Fully open hands
- `left_hand_half_close` / `right_hand_half_close`: Half closed hands
- `left_hand_close` / `right_hand_close`: Fully closed hands

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

#### Core Node List (Dual Hands Configuration)
```bash
/controller_manager
/joint_state_broadcaster
/left_revo2_hand_controller
/move_group
/right_revo2_hand_controller
/robot_state_publisher
/rviz
```

#### Controller Status
```bash
left_revo2_hand_controller  joint_trajectory_controller/JointTrajectoryController  active
right_revo2_hand_controller joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster          active
```

#### Hardware Components (Dual Hands Configuration)
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

#### Hardware Interfaces (Dual Hands Configuration)
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

#### Core Topic List (Dual Hands Configuration)
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

#### Core Action List (Dual Hands Configuration)
```bash
/execute_trajectory
/left_revo2_hand_controller/follow_joint_trajectory
/right_revo2_hand_controller/follow_joint_trajectory
```

## Package Structure

```
brainco_moveit_config/
├── launch/                                      # Launch files
│   ├── revo2_left_moveit.launch.py              # Left hand launch
│   ├── revo2_right_moveit.launch.py             # Right hand launch
│   └── dual_revo2_moveit.launch.py              # Dual hands launch
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
│   ├── revo2_right_moveit_controllers.yaml      # Right hand MoveIt controllers
│   ├── dual_revo2.urdf.xacro                    # Dual hands URDF
│   ├── dual_revo2.ros2_control.xacro            # Dual hands ros2_control
│   ├── dual_revo2.srdf                          # Dual hands SRDF
│   ├── dual_revo2_controllers.yaml              # Dual hands controllers
│   ├── dual_revo2_initial_positions.yaml        # Dual hands initial positions
│   ├── dual_revo2_joint_limits.yaml             # Dual hands joint limits
│   ├── dual_revo2_kinematics.yaml               # Dual hands kinematics
│   ├── dual_revo2_moveit_controllers.yaml       # Dual hands MoveIt controllers
│   ├── moveit.rviz                              # RViz configuration
│   └── pilz_cartesian_limits.yaml               # Pilz planner limits
├── CMakeLists.txt                               # Build configuration
├── package.xml                                  # Package description
└── README.md                                    # This file
```

## Related Packages

- **revo2_description**: URDF models for Revo2 hands
- **brainco_gazebo**: Gazebo simulation for Revo2 hands

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
