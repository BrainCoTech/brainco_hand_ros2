# BrainCo Hand Driver

[English](README.md) | [简体中文](README_CN.md)

## Overview

`revo2_driver` is a ROS 2 `ros2_control` hardware interface package for BrainCo Revo2 dexterous hands.
It supports:

- single-hand and dual-hand launches
- Modbus serial communication
- optional CAN FD communication
- position and velocity command interfaces
- joint state feedback and fingertip tactile feedback

Each hand exposes 6 controllable joints:

- `thumb_proximal_joint`
- `thumb_metacarpal_joint`
- `index_proximal_joint`
- `middle_proximal_joint`
- `ring_proximal_joint`
- `pinky_proximal_joint`

Tactile state interfaces are exported on 5 fingertip joints:

- `tactile_normal_force`
- `tactile_tangential_force`
- `tactile_tangential_direction`
- `tactile_self_proximity`
- `tactile_status`

## Current Package Behavior

The current implementation is centered around `ros2_control` forward controllers, not a trajectory controller.

- `revo2_joint_state` is loaded and activated by default
- `joint_forward_pos_controller` is loaded and activated by default
- `joint_forward_vel_controller` is loaded but kept inactive by default
- default controller manager update rate is `500 Hz`
- default launch namespace is `revo2_left` or `revo2_right`

If you are updating older integration code, note these important differences:

- default `description_package` is `revoarm_description`
- the package still relies on `revo2_description` mesh assets through `revoarm_description`
- the README examples that use `joint_trajectory_controller` are outdated and should not be used for this package

## Environment

- Ubuntu 22.04
- ROS 2 Humble
- `ros2_control`
- `controller_manager`
- `joint_state_broadcaster`
- `forward_command_controller`
- `robot_state_publisher`
- `xacro`
- `revoarm_description`
- `revo2_description`

## Build

Build from your ROS 2 workspace root:

```bash
cd revoarm_hardware/Revoarm_ws
source /opt/ros/humble/setup.bash

# Modbus only
colcon build --packages-up-to revo2_driver --symlink-install

# Enable CAN FD support
colcon build --packages-up-to revo2_driver --symlink-install --cmake-args -DENABLE_CANFD=ON

source install/setup.bash
```

### BrainCo Stark SDK

The Stark SDK is vendored under `vendor/` and can be used directly. The current vendored version is
`v2.0.9`, and `scripts/download_sdk.sh` is pinned to download the same SDK version.
If you need to refresh it:

```bash
cd revoarm_hardware/Revoarm_ws/src/brainco_drivers/revo2_driver
./scripts/download_sdk.sh
```

The script validates the platform, library architecture, header and SDK license before
installing the release. Keep the SDK header and shared library from the same release,
then rebuild `revo2_driver` and source the updated workspace. The SDK license is
included in `vendor/LICENSE` and installed under `share/revo2_driver/licenses/`.

### CAN FD Build Note

CAN FD support is compiled out unless you build with:

```bash
--cmake-args -DENABLE_CANFD=ON
```

If you launch with `protocol:=canfd` without rebuilding with `ENABLE_CANFD=ON`, hardware initialization will fail.

## Configuration Files

Main configuration files in this package:

| File | Purpose |
|---|---|
| `config/protocol_modbus_left.yaml` | Left-hand Modbus parameters |
| `config/protocol_modbus_right.yaml` | Right-hand Modbus parameters |
| `config/protocol_canfd_left.yaml` | Left-hand CAN FD parameters |
| `config/protocol_canfd_right.yaml` | Right-hand CAN FD parameters |
| `config/revo2_initial_positions.yaml` | Initial joint positions |
| `config/revo2_controllers.yaml` | Controller manager and forward controller template |

### Modbus Parameters

Important Modbus fields:

- `slave_id`: left hand defaults to `126`, right hand defaults to `127`
- `port`: serial device path such as `/dev/ttyUSB0`
- `baudrate`: default `460800`
- `auto_detect`: scan serial ports and detect a Revo2 device automatically
- `auto_detect_quick`: quick scan mode
- `auto_detect_port`: optional port prefix hint such as `/dev/ttyUSB`

When `auto_detect: true`:

- `port` and `baudrate` are ignored
- the detected device `slave_id` is used at runtime
- the configured `slave_id` is treated as a hint for left/right hand matching

### Scaling Parameters

The hardware interface converts between ROS-side radians and device-side raw values through:

- `position_command_scale`
- `position_state_scale`
- `velocity_state_scale`
- `velocity_command_scale`
- `velocity_device_min`
- `velocity_device_max`
- `position_device_min`
- `position_device_max`
- `velocity_percentage`

`joint_forward_vel_controller` commands use ROS joint velocity units (`rad/s`). The hardware
interface multiplies them by `velocity_command_scale` before calling the SDK speed API.
In normalized mode, the standard Revo2 scale is `572.9577951308232`, so `1.0 rad/s` is written
as about `573` SDK normalized speed units before clamping to `[-1000, 1000]`.

`velocity_percentage` is clamped to the valid runtime range `0` to `100`.

When `finger_unit_mode: physical` is selected, the base scaling parameters are still used unless
physical-mode overrides are configured. The optional override names are:

- `physical_position_command_scale`
- `physical_position_state_scale`
- `physical_velocity_state_scale`
- `physical_velocity_command_scale`
- `physical_position_device_min`
- `physical_position_device_max`
- `physical_velocity_device_min`
- `physical_velocity_device_max`

If only `physical_position_state_scale` or `physical_velocity_state_scale` is provided, the matching
command scale is derived as its inverse. A practical first calibration for degree-based physical
values is `57.29577951308232` for command scale and `0.017453292519943295` for state scale, then tune
`physical_position_device_max` against the real hand's observed range. For speed-mode teleoperation,
physical mode should also clamp SDK speed commands to the hand's degree-per-second range; the
right-hand Modbus profile currently uses `-150` to `150` deg/s.

### CAN FD Parameters

Important CAN FD fields:

- `can_device_type`
- `can_card_index`
- `can_channel_index`
- `can_clock_hz`
- `can_rx_wait_time`
- `can_rx_buffer_size`
- `can_master_id`
- arbitration timing fields
- data timing fields

For dual-hand CAN FD setups, do not point both hands at the same physical CAN device/channel unless your deployment is designed for that explicitly. In practice, use distinct channel or device settings per hand.

## Launch

### Single Hand

Right hand, Modbus:

```bash
ros2 launch revo2_driver revo2_system.launch.py hand_side:=right
```

Left hand, Modbus:

```bash
ros2 launch revo2_driver revo2_system.launch.py hand_side:=left
```

Right hand, CAN FD:

```bash
ros2 launch revo2_driver revo2_system.launch.py hand_side:=right protocol:=canfd
```

Left hand, CAN FD:

```bash
ros2 launch revo2_driver revo2_system.launch.py hand_side:=left protocol:=canfd
```

### Dual Hand

Both hands with default Modbus configs:

```bash
ros2 launch revo2_driver dual_revo2_system.launch.py
```

Left Modbus + right CAN FD:

```bash
ros2 launch revo2_driver dual_revo2_system.launch.py \
  left_protocol:=modbus \
  right_protocol:=canfd
```

Override protocol config files:

```bash
ros2 launch revo2_driver dual_revo2_system.launch.py \
  left_protocol_config_file:=protocol_modbus_left.yaml \
  right_protocol_config_file:=protocol_modbus_right.yaml
```

Relative protocol config names are resolved from `revo2_driver/config/`.

### Simulation

Use mock hardware instead of real hardware:

```bash
ros2 launch revo2_driver revo2_system.launch.py hand_side:=right if_sim:=true
```

## Launch Arguments

### `revo2_system.launch.py`

| Argument | Default | Description |
|---|---|---|
| `description_package` | `revoarm_description` | Package that provides `urdf/system/revo2.single.system.xacro` |
| `hand_side` | `right` | `left` or `right` |
| `protocol` | `modbus` | `modbus` or `canfd` |
| `protocol_config_file` | `""` | Override protocol YAML |
| `initial_positions_file` | `""` | Override initial positions YAML |
| `controllers_file` | `""` | Override controller template YAML |
| `use_namespace` | `true` | Use `revo2_left` or `revo2_right` namespace |
| `if_sim` | `false` | Use mock hardware |
| `launch_rsp` | `true` | Launch `robot_state_publisher` |

### `dual_revo2_system.launch.py`

| Argument | Default | Description |
|---|---|---|
| `description_package` | `revoarm_description` | Description package |
| `left_protocol` | `modbus` | Left-hand protocol |
| `right_protocol` | `modbus` | Right-hand protocol |
| `left_protocol_config_file` | `""` | Left protocol YAML override |
| `right_protocol_config_file` | `""` | Right protocol YAML override |
| `initial_positions_file` | `""` | Shared initial positions YAML override |
| `controllers_file` | `""` | Controller template override |
| `use_namespace` | `true` | Strongly recommended for dual-hand launch |
| `if_sim` | `false` | Use mock hardware |
| `launch_rsp` | `true` | Launch per-hand `robot_state_publisher` |

## Runtime Interfaces

### Controllers

For a right-hand launch with default namespace:

- controller manager: `/revo2_right/controller_manager`
- active controller: `/revo2_right/joint_forward_pos_controller`
- inactive controller: `/revo2_right/joint_forward_vel_controller`
- joint state broadcaster: `/revo2_right/revo2_joint_state`

For a left-hand launch with default namespace, replace `revo2_right` with `revo2_left`.

### Position Command Topic

The default active controller accepts `std_msgs/msg/Float64MultiArray`:

```bash
ros2 topic pub --once /revo2_right/joint_forward_pos_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.30, 0.10, 0.60, 0.60, 0.60, 0.60]}"
```

Joint order:

1. `thumb_proximal_joint`
2. `thumb_metacarpal_joint`
3. `index_proximal_joint`
4. `middle_proximal_joint`
5. `ring_proximal_joint`
6. `pinky_proximal_joint`

### Velocity Command Topic

The velocity forward controller is loaded inactive. Switch controllers first:

```bash
ros2 control switch_controllers \
  --controller-manager /revo2_right/controller_manager \
  --deactivate joint_forward_pos_controller \
  --activate joint_forward_vel_controller
```

Then publish velocity commands:

```bash
ros2 topic pub --once /revo2_right/joint_forward_vel_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.17, 0.17, 0.35, 0.35, 0.35, 0.35]}"
```

### State Topics

Joint states:

```bash
ros2 topic echo /revo2_right/revo2_joint_state/joint_states
```

Dynamic joint states, including tactile interfaces:

```bash
ros2 topic echo /revo2_right/revo2_joint_state/dynamic_joint_states
```

Tactile data is reported for:

- `thumb_proximal_joint`
- `index_proximal_joint`
- `middle_proximal_joint`
- `ring_proximal_joint`
- `pinky_proximal_joint`

`thumb_metacarpal_joint` does not export tactile interfaces.

## Troubleshooting

### Serial Port Permission

```bash
ls -l /dev/ttyUSB*
groups | grep dialout
sudo usermod -a -G dialout $USER
```

After adding the group, re-login before retrying.

### Modbus Auto Detect for Dual Hand

Recommended defaults:

- left hand: `slave_id: 126`
- right hand: `slave_id: 127`

If both hands are connected through USB serial adapters, you can either:

- keep manual `port` settings
- enable `auto_detect: true` and keep different `slave_id` hints
- narrow each side with `auto_detect_port`

### Common Checks

```bash
ros2 control list_controllers -c /revo2_right/controller_manager
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 topic list | grep revo2_right
```

If `protocol:=canfd` fails at startup, verify both:

- the package was built with `-DENABLE_CANFD=ON`
- the configured CAN device and channel are valid for the current machine

## PID integral term

The six joints have independent integral state. Existing parameter names remain
under `pd_velocity`:

```yaml
pd_velocity:
  velocity_kp: 6.7
  velocity_ki: 0.0
  velocity_kd: 0.8
  integral_max: 0.2
```

`velocity_ki` (1/s²) defaults to zero to preserve the existing PD behavior.
Each cycle applies `I = clamp(I + Ki * position_error * dt, -integral_max, integral_max)`.
`integral_max` bounds the I contribution in rad/s; zero also disables integration.
I is added after the existing joint PD scaling and before feedforward and
four-finger extension scaling. The latter can amplify its final contribution;
the total output still passes through the existing velocity limits.

Integral state is cleared in the error deadband, on error reversal, for outward
integral at configured position boundaries, on target timeout, on controller
deactivation/reactivation, and on invalid position feedback. Conditional
integration prevents further windup into controller velocity, thumb brake and
per-cycle slew saturation. Errors inside the deadband are not integrated.

These parameters are read during configuration: reload the controller or restart
the driver after changing YAML; live `ros2 param set` does not update the control
calculation. Start with a small Ki and unloaded, small-angle targets.
Anti-windup only covers limits known to the controller, not SDK/firmware saturation
or blocked motion. Match the controller velocity/position limits to the hardware.
This change does not add hardware position guards to pure speed mode or change
the existing stale-position behavior when switching back to position control.

## SDK v2.0.9 services

Set `sdk_file_logging:=true/false` at launch (empty uses the protocol YAML).
The driver exposes `touch/enable` and `touch/calibrate` in its ROS namespace,
using `revo2_driver/srv/TouchSensorCommand`. Success means transport success,
not sensor readiness or completed calibration. The request selects sensors with
`sensor_mask` (all five fingers: 31). Calibrate only with unloaded fingertips.

## Trajectory control

Startup always activates `joint_forward_pos_controller` and the joint state
broadcaster. Velocity, PID and trajectory controllers are loaded inactive.
To use the B-compatible `left_revo2_hand_controller` or
`right_revo2_hand_controller` and its `FollowJointTrajectory` action, switch
controllers after startup. For the right hand:

```bash
ros2 control switch_controllers -c /revo2_right/controller_manager \
  --deactivate joint_forward_pos_controller \
  --activate right_revo2_hand_controller --strict
```

The right-hand action is
`/revo2_right/right_revo2_hand_controller/follow_joint_trajectory`.
Goals must name all six joints and use increasing `time_from_start` values.
