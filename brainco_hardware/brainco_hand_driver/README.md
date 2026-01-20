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

### 1. Dependencies

Make sure you have the following packages installed:

```bash
# ROS2 dependencies
sudo apt install ros-humble-controller-manager ros-humble-joint-trajectory-controller
sudo apt install ros-humble-joint-state-broadcaster ros-humble-robot-state-publisher

# Python dependencies
pip3 install rclpy trajectory_msgs sensor_msgs control_msgs
```

### 2. BrainCo Stark SDK

The BrainCo Stark SDK is included in the `vendor/` directory and can be used directly. To update the SDK version, you can run:

```bash
cd brainco_hardware/brainco_hand_driver
./scripts/download_sdk.sh
```

### 3. Build the Workspace

**Method 1: Using Build Script (Recommended, Simple and Convenient)**

The project provides a convenient build script `build.sh` for easier control of build options:

```bash
# Navigate to workspace root
cd ~/brainco_ws

# Default build (CAN FD and EtherCAT disabled)
./build.sh

# Enable CAN FD support
./build.sh --canfd

# Enable EtherCAT support
./build.sh --ethercat

# Enable both CAN FD and EtherCAT
./build.sh --canfd --ethercat

# Release mode build
./build.sh --release

# Release mode with all features enabled
./build.sh --release --canfd --ethercat

# Show help information
./build.sh --help
```

**Method 2: Using colcon Commands (Advanced Users)**

For more fine-grained control, you can use colcon commands directly:

```bash
# Navigate to workspace
cd ~/brainco_ws

# Install dependencies
rosdep install --ignore-src --from-paths src -y -r

# Method 1: Full build (CAN FD and EtherCAT disabled by default, Modbus only)
# Note: When building from workspace root, use --packages-ignore to skip EtherCAT
colcon build --symlink-install --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# Method 2: Full build with CAN FD support (EtherCAT disabled)
colcon build --symlink-install --cmake-args -DENABLE_CANFD=ON --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# Method 3: Full build (Release mode, CAN FD enabled, EtherCAT disabled)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_CANFD=ON --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# Method 4: Full build (Release mode, CAN FD and EtherCAT disabled)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# Source the workspace
source install/setup.bash
```

**Build Options:**
- **Default behavior**: `ENABLE_CANFD=OFF` (CAN FD support disabled), EtherCAT packages not built by default, Modbus mode only
- **Enable CAN FD**: Use `-DENABLE_CANFD=ON`, requires ZLG USB-CAN FD library files
- **Enable EtherCAT**: EtherCAT is a separate ROS2 package, not built by default. To enable, build from workspace root without `--packages-ignore`, or use `build.sh --ethercat` script
- **Build mode**: Optional `-DCMAKE_BUILD_TYPE=Release` for Release mode builds

**Building brainco_hand_driver Package Only:**

If you only want to build the `brainco_hand_driver` package, there are three methods:

**Method A: Using build.sh Script (Recommended)**

```bash
# From workspace root
cd ~/brainco_ws

# build.sh script automatically handles dependencies
# Default build (CAN FD and EtherCAT disabled)
./build.sh

# Enable CAN FD support
./build.sh --canfd
```

**Method B: Using --packages-up-to from Workspace Root (Recommended for Single Package)**

```bash
# From workspace root
cd ~/brainco_ws

# --packages-up-to automatically builds dependencies (like revo2_description)
# Single package build (CAN FD disabled by default)
colcon build --packages-up-to brainco_hand_driver --symlink-install --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# Single package build with CAN FD enabled
colcon build --packages-up-to brainco_hand_driver --cmake-args -DENABLE_CANFD=ON --symlink-install --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver

# Single package build (Release mode, CAN FD enabled)
colcon build --packages-up-to brainco_hand_driver --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_CANFD=ON --symlink-install --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver
```

**Method C: Building from Package Directory (Quick Testing Only, Not Recommended)**

```bash
# Navigate to package directory
cd ~/brainco_ws/src/brainco_hand_ros2/brainco_hardware/brainco_hand_driver

# Build current package (CAN FD disabled by default)
# Note: When building from package directory, only current package is built, no --packages-ignore needed
# But dependencies (like revo2_description) must be built first
colcon build --symlink-install

# Build current package with CAN FD enabled
colcon build --symlink-install --cmake-args -DENABLE_CANFD=ON
```

**Important Notes:**
- `brainco_hand_driver` depends on `revo2_description` package
- If using `--packages-select`, you need to build dependencies first, otherwise error: `Failed to find revo2_description`
- Recommended to use `--packages-up-to` or `build.sh` script, which automatically handle dependencies

### 4. Verify Installation

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

### Modbus Auto Detection Feature

The driver supports automatic detection of serial ports and slave IDs, especially useful for:
- Scenarios where serial port device paths are not fixed (e.g., `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.)
- Dual hand configurations that need automatic left/right hand device identification
- Device hot-plugging where port numbers may change

#### Enable Auto Detection

Edit the configuration file `config/protocol_modbus_right.yaml` (right hand) or `config/protocol_modbus_left.yaml` (left hand):

```yaml
hardware:
  protocol: modbus
  # slave_id: When auto_detect is true, this value is used as a hint, actual slave_id from detected device will be used
  # If auto_detect is false, this value is used as target slave_id
  # Note: Left hand typically uses slave_id 126, right hand uses 127
  slave_id: 127
  # Enable auto detection: automatically scan all available serial ports to find Revo2 device
  auto_detect: true
  # Quick detection mode: true for quick mode (recommended), false for full detection (slower but more accurate)
  auto_detect_quick: true
  # Auto detection port hint: if specified, only detect within this port range (e.g., "/dev/ttyUSB" will detect all ttyUSB*)
  # Leave empty to detect all available serial ports
  # For dual hand configuration, recommend specifying different port hints for left and right hands to avoid conflicts
  # Example: left hand uses "/dev/ttyUSB0", right hand uses "/dev/ttyUSB2"
  auto_detect_port: ""
  # When auto_detect is true, the following parameters are ignored, using auto-detected values
  # When auto_detect is false, use specified port and baudrate
  port: /dev/ttyUSB0
  baudrate: 460800
```

#### Auto Detection Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `auto_detect` | bool | `false` | Whether to enable auto detection |
| `auto_detect_quick` | bool | `true` | Quick detection mode (recommended). `false` will detect all possible slave_ids (1-247), slower |
| `auto_detect_port` | string | `""` | Port hint. Empty detects all ports, or specify hint like `"/dev/ttyUSB"` to only detect USB ports |

#### Auto Detection in Dual Hand Configuration

In dual hand configuration, each hardware instance (left/right hand) performs auto detection independently, distinguishing left and right hand devices by `slave_id`:

**Left Hand Configuration** (`config/protocol_modbus_left.yaml`):
```yaml
hardware:
  protocol: modbus
  slave_id: 126  # Left hand typically uses 126
  auto_detect: true
  auto_detect_quick: true
  auto_detect_port: ""  # Or specify port hint, e.g., "/dev/ttyUSB0"
```

**Right Hand Configuration** (`config/protocol_modbus_right.yaml`):
```yaml
hardware:
  protocol: modbus
  slave_id: 127  # Right hand typically uses 127
  auto_detect: true
  auto_detect_quick: true
  auto_detect_port: ""  # Or specify port hint, e.g., "/dev/ttyUSB2"
```

**Best Practices**:
- If both devices are connected to the same machine, recommend using `auto_detect_port` hint to limit detection range and avoid conflicts
- Ensure both devices have different `slave_id` (left hand 126, right hand 127)
- If detected `slave_id` doesn't match requested, driver will log a warning but continue using the detected device

#### How Auto Detection Works

1. **Start Detection**: When `auto_detect: true`, driver calls SDK's auto detection function
2. **Scan Serial Ports**: Scan all available serial ports (or limit range based on `auto_detect_port` hint)
3. **Identify Device**: Attempt to connect to each serial port to identify Revo2 device
4. **Match slave_id**:
   - If detected `slave_id` matches configuration, use directly
   - If not matched, warning is given but detected device is still used
5. **Establish Connection**: Establish connection using detected port and parameters

#### Troubleshooting

**Problem: Auto detection fails**

Possible causes:
1. Device not connected or not powered
2. Poor USB cable connection
3. Insufficient device permissions
4. Device occupied by another process

Solutions:
```bash
# Check if device exists
ls -l /dev/ttyUSB*

# Check device permissions
sudo chmod 666 /dev/ttyUSB*

# Check if another process is using the device
lsof /dev/ttyUSB*

# If permissions are insufficient, add user to dialout group
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

**Problem: Detected slave_id doesn't match**

Possible causes:
1. Device's slave_id configuration is incorrect
2. Wrong hand connected (left/right)

Solutions:
1. Check device's actual slave_id
2. Confirm if `slave_id` in configuration file is correct
3. If detected slave_id is correct, the warning can be ignored

**Problem: Wrong device detected in dual hand configuration**

Solutions:
1. Use `auto_detect_port` hint to limit detection range
2. Ensure both devices have different `slave_id` (left hand 126, right hand 127)
3. If still having issues, temporarily disable auto detection and manually specify ports

#### Disable Auto Detection (Manual Configuration)

If auto detection has issues, you can fall back to manual configuration:

```yaml
hardware:
  protocol: modbus
  slave_id: 127
  auto_detect: false  # Disable auto detection
  port: /dev/ttyUSB1  # Manually specify port
  baudrate: 460800
```

For more details, see [AUTO_DETECT_GUIDE.md](AUTO_DETECT_GUIDE.md) and [AUTO_DETECT_USB_PORTS.md](AUTO_DETECT_USB_PORTS.md).

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

### Launch Dual Hand System (Simultaneous Control)

The dual hand system supports controlling both left and right hands simultaneously. There are three methods:

#### Method 1: Using Modbus Mode (Single Node)

Use the dedicated dual hand launch file to control both hands in a single node:

```bash
# Use default configuration (left hand: /dev/ttyUSB0, right hand: /dev/ttyUSB1)
ros2 launch brainco_hand_driver dual_revo2_system.launch.py

# Custom protocol configuration files
ros2 launch brainco_hand_driver dual_revo2_system.launch.py \
    left_protocol_config_file:=/path/to/protocol_modbus_left.yaml \
    right_protocol_config_file:=/path/to/protocol_modbus_right.yaml
```

**Notes:**
- In default configuration, if auto detection is enabled (`auto_detect: true`), driver will automatically scan and find left/right hand devices
- If auto detection is not enabled, default configuration uses left hand on `/dev/ttyUSB0` (slave_id: 126), right hand on `/dev/ttyUSB1` (slave_id: 127)
- If your devices are connected to different serial ports, you can:
  1. **Recommended**: Enable auto detection (`auto_detect: true`), driver will automatically find devices
  2. Or: Modify `port` parameter in `protocol_modbus_left.yaml` and `protocol_modbus_right.yaml`
- Ensure left and right hands use different `slave_id` (e.g., left hand 126, right hand 127), auto detection distinguishes left/right hand devices by `slave_id`

#### Method 2: Using CAN FD Mode (Recommended for Multi-hand Control)

CAN FD mode can control multiple hands in a single node, suitable for scenarios requiring control of multiple hands:

```bash
# Dual hand system CAN FD mode
ros2 launch brainco_moveit_config dual_revo2_real_moveit.launch.py protocol:=canfd
```

#### Method 3: Launch Two Independent Nodes (Modbus Mode)

If you need to control left and right hands separately, you can launch two independent nodes:

**Terminal 1 - Launch Left Hand:**
```bash
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=left
```

**Terminal 2 - Launch Right Hand:**
```bash
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right
```

**Notes:**
- If auto detection is enabled (`auto_detect: true`), driver will automatically find devices, no need to manually specify ports
- If auto detection is not enabled, ensure left and right hands are connected to different serial ports (e.g., `/dev/ttyUSB0` and `/dev/ttyUSB1`), and specify correct ports in their respective protocol configuration files

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hand_type` | `right` | Hand type: `left` or `right` |
| `protocol` | `modbus` | Communication protocol: `modbus` or `canfd` |
| `protocol_config_file` | `""` | Protocol configuration file (YAML), leave empty to use default configuration |

When using Modbus mode, the `port` and `slave_id` in protocol configuration file need to be modified according to actual hardware connection.

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

### Dual Hand Simultaneous Control Examples

After launching the dual hand system, you can control both left and right hands simultaneously:

**Simultaneous Control (Topic Method):**

```bash
# Terminal 1 - Control left hand
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

# Terminal 2 - Simultaneously control right hand
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

**Simultaneous Control (Action Method):**

```bash
# Terminal 1 - Control left hand
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

# Terminal 2 - Simultaneously control right hand
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

**Monitor Dual Hand Status:**

```bash
# View all joint states (includes both left and right hands)
ros2 topic echo /joint_states

# List all controllers
ros2 control list_controllers
# Should see:
# left_revo2_hand_controller   joint_trajectory_controller/JointTrajectoryController  active
# right_revo2_hand_controller  joint_trajectory_controller/JointTrajectoryController  active
# joint_state_broadcaster       joint_state_broadcaster/JointStateBroadcaster          active
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

#### Core Node List (Dual Hand Configuration)
```bash
/controller_manager
/joint_state_broadcaster
/left_revo2_hand_controller
/right_revo2_hand_controller
/robot_state_publisher
```

#### Controller Status (Right Hand Configuration)
```bash
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster          active
right_revo2_hand_controller   joint_trajectory_controller/JointTrajectoryController  active
```

#### Controller Status (Dual Hand Configuration)
```bash
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster          active
left_revo2_hand_controller   joint_trajectory_controller/JointTrajectoryController   active
right_revo2_hand_controller  joint_trajectory_controller/JointTrajectoryController  active
```

#### Hardware Components (Right Hand Configuration)
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

#### Core Topic List (Right Hand Configuration)
```bash
/joint_states
/right_revo2_hand_controller/joint_trajectory
/robot_description
/tf
/tf_static
```

#### Core Topic List (Dual Hand Configuration)
```bash
/joint_states
/left_revo2_hand_controller/joint_trajectory
/right_revo2_hand_controller/joint_trajectory
/robot_description
/tf
/tf_static
```

#### Core Action List (Right Hand Configuration)
```bash
/right_revo2_hand_controller/follow_joint_trajectory
```

#### Core Action List (Dual Hand Configuration)
```bash
/left_revo2_hand_controller/follow_joint_trajectory
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
brainco_hand_driver/
├── launch/                                      # Launch files
│   └── revo2_system.launch.py                    # Main system launch
├── config/                                      # Configuration files
│   ├── protocol_*.yaml                          # Protocol configuration files (Modbus/CAN FD)
│   ├── xxx.urdf.xacro                           # URDF XACRO files
│   ├── xxx.ros2_control.xacro                   # ros2_control configuration
│   ├── xxx_controllers.yaml                     # Controller configuration
│   ├── xxx_initial_positions.yaml               # Initial positions configuration
│   └── ...                                      # Other configuration files
├── include/                                     # Header files
│   └── brainco_hand_driver/                     # Driver headers
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
└── README.md                                    # This file
```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Contact

If you have any questions or suggestions, please contact the development team.
