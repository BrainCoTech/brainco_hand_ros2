<div align="right">

[English](README.md)|[简体中文](README_CN.md)

</div>

# BrainCo Gazebo Revo2 Hand Demo

## Overview

The BrainCo Gazebo Revo2 Hand Demo package provides a complete Gazebo simulation environment for BrainCo Revo2 dexterous hands. Built on ROS2 and Ignition Gazebo 6, it offers professional-grade robot simulation capabilities for research and development of dexterous manipulation.

## Features

- **Dexterous Hand Simulation**: Complete simulation of BrainCo Revo2 dexterous hands (left/right)
- **Gazebo Physics Simulation**: High-fidelity physics simulation using Ignition Gazebo 6
- **ROS2 Control Integration**: Full ros2_control support with trajectory controllers
- **MoveIt Integration**: Complete Gazebo-MoveIt integration for motion planning and execution
- **RViz Visualization**: Built-in launch files for visualizing hand models in RViz
- **Multi-Hand Support**: Support for both left and right hand configurations

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
# Copy brainco_gazebo to src/

# Required Packages
rosdep install --ignore-src --from-paths . -y -r

# Build the workspace
cd ~/brainco_ws
colcon build --packages-select revo2_description brainco_gazebo brainco_moveit_config --symlink-install
source install/setup.bash
```

### 2. Verify Installation

```bash
# Check if packages are available
ros2 pkg list | grep -E "(revo2_description|brainco_gazebo|brainco_moveit_config)"

```

## Quick Start

### Basic Launch

```bash
# Source the workspace
source ~/brainco_ws/install/setup.bash

# Launch single hand simulation
ros2 launch brainco_gazebo revo2_hand_gazebo.launch.py hand_type:=left
ros2 launch brainco_gazebo revo2_hand_gazebo.launch.py hand_type:=right

# Launch dual hand simulation
ros2 launch brainco_gazebo dual_revo2_hand_gazebo.launch.py
```

### Launch with Parameters

```bash
# Launch without RViz (for performance)
ros2 launch brainco_gazebo revo2_hand_gazebo.launch.py use_rviz:=false

# Launch with custom world
ros2 launch brainco_gazebo revo2_hand_gazebo.launch.py world:=your_world
```

## Gazebo-MoveIt Integration

### Overview

The package provides integrated Gazebo-MoveIt launch files that combine physics simulation with motion planning capabilities. These launch files enable advanced motion planning, path optimization, and collision avoidance for the Revo2 hands.

### Launch Gazebo-MoveIt System

```bash
# Launch single hand with MoveIt
ros2 launch brainco_gazebo revo2_hand_gazebo_moveit.launch.py hand_type:=left
ros2 launch brainco_gazebo revo2_hand_gazebo_moveit.launch.py hand_type:=right

# Launch dual hands with MoveIt
ros2 launch brainco_gazebo dual_revo2_hand_gazebo_moveit.launch.py
```

### MoveIt Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hand_type` | `right` | Hand type: 'left' or 'right' (single hand only) |
| `world` | `empty_world` | Gazebo world file |
| `use_rviz` | `true` | Whether to launch RViz with MoveIt plugin |
| `db` | `false` | Whether to start MoveIt warehouse database |
| `publish_monitored_planning_scene` | `true` | Publish monitored planning scene |

### Using MoveIt for Motion Planning

Once the Gazebo-MoveIt system is launched, you can use MoveIt's RViz plugin to plan and execute trajectories:

1. **Interactive Planning**: Use the interactive markers in RViz to set goal poses
2. **Plan Trajectory**: Click "Plan" to generate a motion plan
3. **Execute**: Click "Execute" to send the trajectory to the simulated hand in Gazebo

### Architecture: Gazebo vs MoveIt Launch

The package provides two types of launch configurations:

#### 1. Gazebo-Only Mode
- Launch files: `revo2_hand_gazebo.launch.py`, `dual_revo2_hand_gazebo.launch.py`
- Hardware interface: `gz_ros2_control` with GazeboSimSystem
- Use case: Basic simulation and manual control

#### 2. Gazebo-MoveIt Integration Mode
- Launch files: `revo2_hand_gazebo_moveit.launch.py`, `dual_revo2_hand_gazebo_moveit.launch.py`
- Hardware interface: Same as Gazebo-only
- Additional components: MoveIt move_group, motion planning pipelines
- Use case: Advanced motion planning and autonomous control

**Key Differences:**
- Both modes use the same Gazebo simulation and controllers
- MoveIt mode adds motion planning capabilities on top

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hand_type` | `right` | Hand type: 'left' or 'right' |
| `model` | `revo2_hand` | Robot model name |
| `world` | `empty_world` | Gazebo world file (without .sdf extension) |
| `use_rviz` | `true` | Whether to launch RViz visualization |

## Control Interface

### Controller Overview

| Controller Name | Type | Managed Joints | Function |
|----------------|------|----------------|----------|
| `joint_state_broadcaster` | JointStateBroadcaster | All joints | Joint state publishing |
| `revo2_hand_controller` | JointTrajectoryController | Hand 6 main joints | Hand pose control |

### Hand Control Examples

#### Open Hand
```bash
ros2 topic pub --once /right_revo2_hand_controller/joint_trajectory \
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
ros2 topic pub --once /right_revo2_hand_controller/joint_trajectory \
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
ros2 topic pub --once /left_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "left_thumb_metacarpal_joint",
      "left_thumb_proximal_joint",
      "left_index_proximal_joint",
      "left_middle_proximal_joint",
      "left_ring_proximal_joint",
      "left_pinky_proximal_joint"
    ],
    points: [{
      positions: [1.2, 0.7, 0.6, 0.0, 0.0, 0.0],
      time_from_start: {sec: 2}
    }]
  }'

ros2 topic pub --once /right_revo2_hand_controller/joint_trajectory \
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

#### Action trajectory control

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
      {  # Open palm
        positions: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 1, nanosec: 0}
      },
      {  # Fist
        positions: [0.3, 0.8, 1.2, 1.2, 1.2, 1.2],
        time_from_start: {sec: 2, nanosec: 500000000}
      },
      {  # Extend index finger
        positions: [0.3, 0.8, 0.0, 1.2, 1.2, 1.2],
        time_from_start: {sec: 4, nanosec: 0}
      },
      {  # Extend index and middle fingers
        positions: [0.3, 0.8, 0.0, 0.0, 1.2, 1.2],
        time_from_start: {sec: 5, nanosec: 500000000}
      },
      {  # OK gesture
        positions: [1.2, 0.7, 0.6, 0.0, 0.0, 0.0],
        time_from_start: {sec: 7, nanosec: 0}
      }
    ]
  }
}"


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
      {  # Open palm
        positions: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 1, nanosec: 0}
      },
      {  # Fist
        positions: [0.3, 0.8, 1.2, 1.2, 1.2, 1.2],
        time_from_start: {sec: 2, nanosec: 500000000}
      },
      {  # Extend index finger
        positions: [0.3, 0.8, 0.0, 1.2, 1.2, 1.2],
        time_from_start: {sec: 4, nanosec: 0}
      },
      {  # Extend index and middle fingers
        positions: [0.3, 0.8, 0.0, 0.0, 1.2, 1.2],
        time_from_start: {sec: 5, nanosec: 500000000}
      },
      {  # OK gesture
        positions: [1.2, 0.7, 0.6, 0.0, 0.0, 0.0],
        time_from_start: {sec: 7, nanosec: 0}
      }
    ]
  }
}"
```

## Monitoring and Debugging

### System Status Commands

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
/gz_ros2_control
/joint_state_broadcaster
/left_revo2_hand_controller
/move_group
/right_revo2_hand_controller
/robot_state_publisher
/ros_gz_bridge
/rviz
```

#### Controller Status
```bash
joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster          active
right_revo2_hand_controller joint_trajectory_controller/JointTrajectoryController  active
left_revo2_hand_controller  joint_trajectory_controller/JointTrajectoryController  active
```

#### Hardware Components (Dual Hands Configuration)
```bash
Hardware Component 1
	name: GazeboSimSystem
	type:
	plugin name:
	state: id=3 label=active
	command interfaces
		left_thumb_metacarpal_joint/position [available] [claimed]
		left_thumb_proximal_joint/position [available] [claimed]
		left_index_proximal_joint/position [available] [claimed]
		left_middle_proximal_joint/position [available] [claimed]
		left_ring_proximal_joint/position [available] [claimed]
		left_pinky_proximal_joint/position [available] [claimed]
		left_thumb_distal_joint_mimic/position [available] [unclaimed]
		left_index_distal_joint_mimic/position [available] [unclaimed]
		left_middle_distal_joint_mimic/position [available] [unclaimed]
		left_ring_distal_joint_mimic/position [available] [unclaimed]
		left_pinky_distal_joint_mimic/position [available] [unclaimed]
		right_thumb_metacarpal_joint/position [available] [claimed]
		right_thumb_proximal_joint/position [available] [claimed]
		right_index_proximal_joint/position [available] [claimed]
		right_middle_proximal_joint/position [available] [claimed]
		right_ring_proximal_joint/position [available] [claimed]
		right_pinky_proximal_joint/position [available] [claimed]
		right_thumb_distal_joint_mimic/position [available] [unclaimed]
		right_index_distal_joint_mimic/position [available] [unclaimed]
		right_middle_distal_joint_mimic/position [available] [unclaimed]
		right_ring_distal_joint_mimic/position [available] [unclaimed]
		right_pinky_distal_joint_mimic/position [available] [unclaimed]
```

#### Hardware Interfaces (Dual Hands Configuration)
```bash
command interfaces
	left_index_distal_joint_mimic/position [available] [unclaimed]
	left_index_proximal_joint/position [available] [claimed]
	left_middle_distal_joint_mimic/position [available] [unclaimed]
	left_middle_proximal_joint/position [available] [claimed]
	left_pinky_distal_joint_mimic/position [available] [unclaimed]
	left_pinky_proximal_joint/position [available] [claimed]
	left_ring_distal_joint_mimic/position [available] [unclaimed]
	left_ring_proximal_joint/position [available] [claimed]
	left_thumb_distal_joint_mimic/position [available] [unclaimed]
	left_thumb_metacarpal_joint/position [available] [claimed]
	left_thumb_proximal_joint/position [available] [claimed]
	right_index_distal_joint_mimic/position [available] [unclaimed]
	right_index_proximal_joint/position [available] [claimed]
	right_middle_distal_joint_mimic/position [available] [unclaimed]
	right_middle_proximal_joint/position [available] [claimed]
	right_pinky_distal_joint_mimic/position [available] [unclaimed]
	right_pinky_proximal_joint/position [available] [claimed]
	right_ring_distal_joint_mimic/position [available] [unclaimed]
	right_ring_proximal_joint/position [available] [claimed]
	right_thumb_distal_joint_mimic/position [available] [unclaimed]
	right_thumb_metacarpal_joint/position [available] [claimed]
	right_thumb_proximal_joint/position [available] [claimed]
state interfaces
	left_index_distal_joint_mimic/effort
	left_index_distal_joint_mimic/position
	left_index_distal_joint_mimic/velocity
	left_index_proximal_joint/position
	left_index_proximal_joint/velocity
	left_middle_distal_joint_mimic/effort
	left_middle_distal_joint_mimic/position
	left_middle_distal_joint_mimic/velocity
	left_middle_proximal_joint/position
	left_middle_proximal_joint/velocity
	left_pinky_distal_joint_mimic/effort
	left_pinky_distal_joint_mimic/position
	left_pinky_distal_joint_mimic/velocity
	left_pinky_proximal_joint/position
	left_pinky_proximal_joint/velocity
	left_ring_distal_joint_mimic/effort
	left_ring_distal_joint_mimic/position
	left_ring_distal_joint_mimic/velocity
	left_ring_proximal_joint/position
	left_ring_proximal_joint/velocity
	left_thumb_distal_joint_mimic/effort
	left_thumb_distal_joint_mimic/position
	left_thumb_distal_joint_mimic/velocity
	left_thumb_metacarpal_joint/position
	left_thumb_metacarpal_joint/velocity
	left_thumb_proximal_joint/position
	left_thumb_proximal_joint/velocity
	right_index_distal_joint_mimic/effort
	right_index_distal_joint_mimic/position
	right_index_distal_joint_mimic/velocity
	right_index_proximal_joint/position
	right_index_proximal_joint/velocity
	right_middle_distal_joint_mimic/effort
	right_middle_distal_joint_mimic/position
	right_middle_distal_joint_mimic/velocity
	right_middle_proximal_joint/position
	right_middle_proximal_joint/velocity
	right_pinky_distal_joint_mimic/effort
	right_pinky_distal_joint_mimic/position
	right_pinky_distal_joint_mimic/velocity
	right_pinky_proximal_joint/position
	right_pinky_proximal_joint/velocity
	right_ring_distal_joint_mimic/effort
	right_ring_distal_joint_mimic/position
	right_ring_distal_joint_mimic/velocity
	right_ring_proximal_joint/position
	right_ring_proximal_joint/velocity
	right_thumb_distal_joint_mimic/effort
	right_thumb_distal_joint_mimic/position
	right_thumb_distal_joint_mimic/velocity
	right_thumb_metacarpal_joint/position
	right_thumb_metacarpal_joint/velocity
	right_thumb_proximal_joint/position
	right_thumb_proximal_joint/velocity
```

#### Core Topic List (Dual Hands Configuration)
```bash
/clock
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
brainco_gazebo/
├── launch/                                         # ROS launch files
│   ├── revo2_hand_gazebo.launch.py                 # Single hand Gazebo launch
│   ├── dual_revo2_hand_gazebo.launch.py            # Dual hands Gazebo launch
│   ├── revo2_hand_gazebo_moveit.launch.py          # Single hand Gazebo+MoveIt launch
│   └── dual_revo2_hand_gazebo_moveit.launch.py     # Dual hands Gazebo+MoveIt launch
├── config/                                         # Configuration files
│   ├── revo2_gazebo_description.urdf.xacro         # Single hand robot description for Gazebo
│   ├── dual_revo2_gazebo_description.urdf.xacro    # Dual hands robot description for Gazebo
│   ├── revo2_left_controllers.yaml                 # Left hand controller configuration
│   ├── revo2_right_controllers.yaml                # Right hand controller configuration
│   └── dual_revo2_controllers.yaml                 # Dual hands controller configuration
├── worlds/                                         # Gazebo world files
│   └── empty_world.sdf                            # Default world
├── rviz/                                           # RViz configurations
│   ├── revo2_left_hand.rviz                        # Left hand RViz config
│   ├── revo2_right_hand.rviz                       # Right hand RViz config
│   └── dual_revo2_hand.rviz                        # Dual hands RViz config
├── CMakeLists.txt                                 # Build configuration
├── package.xml                                    # Package description
├── CHANGELOG.rst                                  # Version changelog
├── LICENSE                                        # License file
├── README.md                                      # English documentation
└── README_CN.md                                   # Chinese documentation
```

## API Reference

### Action Interfaces

- **Hand Control**: `/revo2_hand_controller/follow_joint_trajectory`

### Topic Interfaces

- **Joint States**: `/joint_states` (sensor_msgs/JointState)
- **Trajectory Commands**: `/revo2_hand_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)
- **Controller States**: `/revo2_hand_controller/controller_state`

### Service Interfaces

- **Controller Management**: `/controller_manager/list_controllers`
- **Hardware Interfaces**: `/controller_manager/list_hardware_interfaces`

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.