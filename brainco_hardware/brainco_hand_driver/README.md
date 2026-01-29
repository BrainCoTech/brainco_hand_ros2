# BrainCo Hand Driver

[English](README.md) | [简体中文](README_CN.md)

## Overview

The BrainCo Hand Driver package provides a ROS2 hardware interface for BrainCo Revo2 dexterous hands using Modbus/CAN FD communication protocols. This package implements a ros2_control hardware interface that enables real-time control and monitoring of 6 finger joints through serial port or CAN FD.

**Communication Protocol Support:**
- **Modbus** (Default): Serial communication with good compatibility and simple deployment
- **CAN FD**: High-speed bus communication, supports controlling multiple hands simultaneously, requires ZLG CAN FD hardware

## Features

- **Communication Protocol**: Support for Modbus and CAN FD dual protocols
- **ros2_control Integration**: Full ros2_control hardware interface implementation
- **Dual Hand Support**: Support for both left and right hand configurations, as well as simultaneous dual-hand control
- **Real-time Control**: High-frequency control loop for precise finger manipulation
- **State Feedback**: Position and velocity feedback from all joints
- **Trajectory Control**: Joint trajectory controller support for smooth motion execution
- **Auto Detection**: Support for Modbus automatic port and slave ID detection (supports both single and dual hand modes)

## Environment

- Ubuntu 22.04
- ROS 2 (Humble)
- Python 3.8+

### Modbus Mode (Default)
- Modbus serial device (e.g., /dev/ttyUSB0)

### CAN FD Mode (Optional)
- ZLG USB-CAN FD device (e.g., USBCANFD-200U)
- ZLG CAN FD driver library (included with package)
- CAN FD bus connection

## Installation and Setup

### BrainCo Stark SDK

The SDK is located in the `vendor/` directory and can be used directly. To update:

```bash
cd brainco_hardware/brainco_hand_driver
./scripts/download_sdk.sh
```

### Build the Workspace

**Recommended: Using Build Script**

```bash
cd ~/brainco_ws

# Default build (Modbus only)
./build.sh

# Enable CAN FD support
./build.sh --canfd

# Enable EtherCAT support
./build.sh --ethercat

# Release mode, enable all features
./build.sh --release --canfd --ethercat
```

**Using colcon Commands**

```bash
cd ~/brainco_ws

# Default build (CAN FD disabled)
colcon build --packages-up-to brainco_hand_driver --symlink-install 

# Enable CAN FD support
colcon build --packages-up-to brainco_hand_driver --symlink-install --cmake-args -DENABLE_CANFD=ON

# Source the workspace
source install/setup.bash
```

**Build Options:**
- Default: `ENABLE_CANFD=OFF`, Modbus only
- Enable CAN FD: Use `-DENABLE_CANFD=ON`

**Note:** `brainco_hand_driver` depends on `revo2_description` package.

### Verify Installation

```bash
# Check if package is available
ros2 pkg list | grep brainco_hand_driver

# Check serial port permissions
ls -l /dev/ttyUSB*
# If permissions are insufficient, add user to dialout group:
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

## Hardware Connection

### Modbus Serial Port Setup

Before using the driver, ensure your Modbus serial port setup is correct:

```bash
# Check serial port devices
ls -l /dev/ttyUSB*

# Check serial port permissions
groups | grep dialout

# If no permission, add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in to take effect

# Test serial port connection (optional)
sudo apt install minicom
sudo minicom -D /dev/ttyUSB0
```

### Modbus Auto Detection

The driver supports automatic detection of serial ports and slave IDs, useful for:
- Scenarios where serial port device paths are not fixed
- Dual hand configurations, automatically identifying left/right hand devices
- Device hot-plugging where port numbers may change

**Enable Auto Detection:**

Edit the configuration file `config/protocol_modbus_right.yaml`:

```yaml
hardware:
  protocol: modbus
  slave_id: 127  # Right hand typically uses 127, left hand uses 126
  auto_detect: true
  auto_detect_quick: true  # Quick detection mode (recommended)
  auto_detect_port: ""  # Port hint, leave empty to detect all ports, or specify like "/dev/ttyUSB"
  # When auto_detect is true, the following parameters are ignored
  port: /dev/ttyUSB0
  baudrate: 460800
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `auto_detect` | bool | `false` | Whether to enable auto detection |
| `auto_detect_quick` | bool | `true` | Quick detection mode. `false` will detect all possible slave_ids (1-247), slower |
| `auto_detect_port` | string | `""` | Port hint. Leave empty to detect all ports, or specify like `"/dev/ttyUSB"` to only detect USB ports |

**Dual Hand Configuration:**

In dual hand configuration, each hardware instance performs auto detection independently, distinguishing left and right hand devices by `slave_id`:

- Left hand configuration (`config/protocol_modbus_left.yaml`): `slave_id: 126`
- Right hand configuration (`config/protocol_modbus_right.yaml`): `slave_id: 127`

## Quick Start

### Launch Right Hand System

```bash
# Modbus mode (default, uses port and slave ID from protocol configuration file)
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right

# CAN FD mode (requires ZLG USB-CAN FD device)
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right protocol:=canfd
```

### Launch Left Hand System

```bash
# Modbus mode (default, uses port and slave ID from protocol configuration file)
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=left

# CAN FD mode (requires ZLG USB-CAN FD device)
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=left protocol:=canfd
```

### Launch Dual Hand System

```bash
# Use default configuration (auto detection) Modbus mode
ros2 launch brainco_hand_driver dual_revo2_system.launch.py

# Custom protocol configuration files
ros2 launch brainco_hand_driver dual_revo2_system.launch.py \
    left_protocol_config_file:=/path/to/protocol_modbus_left.yaml \
    right_protocol_config_file:=/path/to/protocol_modbus_right.yaml

# If auto detection is enabled (`auto_detect: true`), the driver will automatically scan and find left/right hand devices. Ensure left and right hands use different `slave_id` (left hand 126, right hand 127).

# CAN FD mode
ros2 launch brainco_moveit_config dual_revo2_real_moveit.launch.py protocol:=canfd
```

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hand_type` | `right` | Hand type: `left` or `right` |
| `protocol` | `modbus` | Communication protocol: `modbus` or `canfd` |
| `protocol_config_file` | `""` | Protocol configuration file (YAML), leave empty to use default configuration |

When using Modbus mode, the `port` and `slave_id` in protocol configuration file need to be modified according to actual hardware connection.

## Control Interface

### Topic Interface

Right hand trajectory control example:

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

Left hand trajectory control example:

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

Right hand Action control example:

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

Left hand Action control example:

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

# Monitor joint states
ros2 topic echo /joint_states
```

### Expected Output

**Core Node List (Right Hand Configuration):**
```
/controller_manager
/joint_state_broadcaster
/right_revo2_hand_controller
/robot_state_publisher
```

**Controller Status (Right Hand Configuration):**
```
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster          active
right_revo2_hand_controller   joint_trajectory_controller/JointTrajectoryController  active
```

**Hardware Components (Right Hand Configuration):**
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

**Core Topic List (Right Hand Configuration):**
```
/joint_states
/right_revo2_hand_controller/joint_trajectory
/robot_description
/tf
/tf_static
```

**Core Action List (Right Hand Configuration):**
```
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

**Note:** Left hand joint names replace `right_` with `left_`, with the same angle ranges.

## Package Structure

```
brainco_hand_driver/
├── launch/                                      # Launch files
│   ├── revo2_system.launch.py                    # Single-hand robot launch file
│   └── dual_revo2_system.launch.py               # Dual-hand robot launch file
├── config/                                      # Configuration files
│   ├── protocol_*.yaml                          # Protocol configuration files (Modbus/CAN FD)
│   ├── xxx.urdf.xacro                           # URDF XACRO files
│   ├── xxx.ros2_control.xacro                   # ros2_control configuration
│   ├── xxx_controllers.yaml                     # Controller configuration
│   ├── xxx_initial_positions.yaml               # Initial positions configuration
│   └── ...                                      # Other configuration files
├── include/                                     # Header files
│   └── brainco_hand_driver/                     # Driver header files
│       ├── brainco_hand_hardware.hpp            # Hardware interface header
│       └── logger_macros.hpp                    # Logger macros
├── src/                                         # Source files
│   └── brainco_hand_hardware.cpp                # Hardware interface implementation
├── scripts/                                     # Utility scripts
│   └── download_sdk.sh                          # SDK download script
├── vendor/                                      # BrainCo Stark SDK
│   └── dist/                                    # SDK distribution files
├── CMakeLists.txt                               # Build configuration
├── package.xml                                  # Package description
├── brainco_hand_driver_plugins.xml              # Plugin description
└── README.md                                    # English documentation
```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Contact

If you have any questions or suggestions, please contact the development team.
