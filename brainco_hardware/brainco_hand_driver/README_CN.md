# BrainCo Hand Driver 驱动包

[English](README.md) | [简体中文](README_CN.md)

## 概述

`revo2_driver` 是面向 BrainCo Revo2 灵巧手的 ROS 2 `ros2_control` 硬件接口包，当前支持：

- 单手和双手启动
- Modbus 串口通信
- 可选的 CAN FD 通信
- 位置和速度命令接口
- 关节状态反馈与指尖触觉反馈

每只手当前暴露 6 个可控关节：

- `thumb_proximal_joint`
- `thumb_metacarpal_joint`
- `index_proximal_joint`
- `middle_proximal_joint`
- `ring_proximal_joint`
- `pinky_proximal_joint`

其中 5 个指尖关节额外暴露触觉状态接口：

- `tactile_normal_force`
- `tactile_tangential_force`
- `tactile_tangential_direction`
- `tactile_self_proximity`
- `tactile_status`

其中触觉量纲约定如下：

- `tactile_normal_force` 与 `tactile_tangential_force` 已在驱动内做 `/100` 缩放，单位为 `N`
- `tactile_tangential_direction` 已在驱动内从角度（deg）转换为弧度（rad）

## 当前包的实际行为

- `revo2_joint_state` 和 `joint_forward_pos_controller` 默认加载并激活。
- 速度、PID 和轨迹控制器默认加载为 inactive，可在启动后切换。
- 每只手默认以 `500 Hz` 更新，使用 `revo2_left` 或 `revo2_right` 命名空间。

默认使用驱动自身的 `urdf/revo2.single.system.xacro`，从 `revo2_description`
加载左右手模型。模型依赖的版本固定在根目录 `brainco_hand.repos` 中。

## 环境依赖

- Ubuntu 22.04
- ROS 2 Humble
- `ros2_control`
- `controller_manager`
- `joint_state_broadcaster`
- `forward_command_controller`
- `robot_state_publisher`
- `xacro`
- `revo2_description`

## 构建

请在 ROS 2 工作空间根目录构建：

```bash
cd ~/brainco_ws
source /opt/ros/humble/setup.bash

bash src/brainco_hand_ros2/brainco_hardware/brainco_hand_driver/scripts/download_sdk.sh

# 仅启用 Modbus
colcon build --packages-up-to revo2_driver --symlink-install

# 启用 CAN FD
colcon build --packages-up-to revo2_driver --symlink-install --cmake-args -DENABLE_CANFD=ON

source install/setup.bash
```

### BrainCo Stark SDK

`scripts/download_sdk.sh` 将固定版本 `v2.0.9` 的 Stark SDK 安装到 `vendor/`。
可在工作空间根目录执行：

```bash
bash src/brainco_hand_ros2/brainco_hardware/brainco_hand_driver/scripts/download_sdk.sh
```

脚本会检查平台、动态库架构、头文件和 SDK 许可证后再安装。头文件和动态库必须
来自同一发布版本；更新后需要重新编译 `revo2_driver`，并重新 source 工作空间。
SDK 许可证位于 `vendor/LICENSE`，安装后位于 `share/revo2_driver/licenses/`。

### CAN FD 编译说明

只有在构建时显式开启：

```bash
--cmake-args -DENABLE_CANFD=ON
```

运行时才可以正常使用 `protocol:=canfd`。
如果未开启该选项，CAN FD 协议会在硬件初始化阶段失败。

## 配置文件

本包内的主要配置文件如下：

| 文件 | 作用 |
|---|---|
| `config/protocol_modbus_left.yaml` | 左手 Modbus 参数 |
| `config/protocol_modbus_right.yaml` | 右手 Modbus 参数 |
| `config/protocol_canfd_left.yaml` | 左手 CAN FD 参数 |
| `config/protocol_canfd_right.yaml` | 右手 CAN FD 参数 |
| `config/revo2_initial_positions.yaml` | 初始关节位置 |
| `config/revo2_controllers.yaml` | 控制器模板与 controller manager 配置 |

### Modbus 关键参数

重点参数包括：

- `slave_id`：左手默认 `126`，右手默认 `127`
- `port`：串口设备路径，例如 `/dev/ttyUSB0`
- `baudrate`：默认 `460800`
- `auto_detect`：自动扫描串口并检测 Revo2 设备
- `auto_detect_quick`：快速扫描模式
- `auto_detect_port`：串口前缀提示，例如 `/dev/ttyUSB`

当 `auto_detect: true` 时：

- `port` 和 `baudrate` 会被忽略
- 运行时会使用检测到的设备 `slave_id`
- 配置文件中的 `slave_id` 只作为左右手匹配提示

### 缩放参数

驱动会通过以下参数在 ROS 侧弧度值和设备原始值之间转换：

- `position_command_scale`
- `position_state_scale`
- `velocity_state_scale`
- `velocity_command_scale`
- `velocity_device_min`
- `velocity_device_max`
- `position_device_min`
- `position_device_max`
- `velocity_percentage`

`joint_forward_vel_controller` 的命令使用 ROS 关节速度单位（`rad/s`）。硬件接口会先乘以
`velocity_command_scale`，再调用 SDK speed API。在 normalized mode 下，标准 Revo2 scale 是
`572.9577951308232`，所以 `1.0 rad/s` 会先写成约 `573` 个 SDK normalized speed unit，
再限制到 `[-1000, 1000]`。

`velocity_percentage` 在驱动内部会被限制到 `0` 到 `100` 的有效范围。

当配置为 `finger_unit_mode: physical` 时，如果没有额外配置 physical 专用参数，驱动仍然会沿用
上面的基础缩放参数。可选的 physical 专用覆盖参数如下：

- `physical_position_command_scale`
- `physical_position_state_scale`
- `physical_velocity_state_scale`
- `physical_velocity_command_scale`
- `physical_position_device_min`
- `physical_position_device_max`
- `physical_velocity_device_min`
- `physical_velocity_device_max`

如果只配置了 `physical_position_state_scale` 或 `physical_velocity_state_scale`，对应的 command
scale 会自动用倒数推导。按角度值做第一次标定时，可以先用 `57.29577951308232` 作为 command scale、
`0.017453292519943295` 作为 state scale，然后根据真机实际行程调整 `physical_position_device_max`。
如果使用 speed-mode 遥操作，physical mode 下还需要把 SDK 速度命令限制在手的角速度范围内；
当前右手 Modbus 配置使用 `-150` 到 `150` deg/s。

### CAN FD 关键参数

重点参数包括：

- `can_device_type`
- `can_card_index`
- `can_channel_index`
- `can_clock_hz`
- `can_rx_wait_time`
- `can_rx_buffer_size`
- `can_master_id`
- 仲裁段时序参数
- 数据段时序参数

如果是双手 CAN FD 部署，不要在未明确设计的情况下让左右手同时指向同一个物理 CAN 设备和通道。更稳妥的做法是为左右手分配不同的设备或通道参数。

## 启动方式

### 单手启动

右手 Modbus：

```bash
ros2 launch revo2_driver revo2_system.launch.py hand_side:=right
```

左手 Modbus：

```bash
ros2 launch revo2_driver revo2_system.launch.py hand_side:=left
```

右手 CAN FD：

```bash
ros2 launch revo2_driver revo2_system.launch.py hand_side:=right protocol:=canfd
```

左手 CAN FD：

```bash
ros2 launch revo2_driver revo2_system.launch.py hand_side:=left protocol:=canfd
```

### 双手启动

使用默认 Modbus 配置同时启动左右手：

```bash
ros2 launch revo2_driver dual_revo2_system.launch.py
```

左手 Modbus，右手 CAN FD：

```bash
ros2 launch revo2_driver dual_revo2_system.launch.py \
  left_protocol:=modbus \
  right_protocol:=canfd
```

自定义协议配置文件：

```bash
ros2 launch revo2_driver dual_revo2_system.launch.py \
  left_protocol_config_file:=protocol_modbus_left.yaml \
  right_protocol_config_file:=protocol_modbus_right.yaml
```

相对协议配置文件名会从 `revo2_driver/config/` 目录解析。

### 仿真模式

使用 mock hardware 启动：

```bash
ros2 launch revo2_driver revo2_system.launch.py hand_side:=right if_sim:=true
```

## Launch 参数

### `revo2_system.launch.py`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `description_package` | `revo2_description` | 默认由驱动组装公开模型；自定义包需提供 `urdf/revo2.single.system.xacro` |
| `hand_side` | `right` | `left` 或 `right` |
| `protocol` | `modbus` | `modbus` 或 `canfd` |
| `protocol_config_file` | `""` | 协议 YAML 覆盖路径 |
| `initial_positions_file` | `""` | 初始位置 YAML 覆盖路径 |
| `controllers_file` | `""` | 控制器模板 YAML 覆盖路径 |
| `use_namespace` | `true` | 是否使用 `revo2_left` / `revo2_right` 命名空间 |
| `if_sim` | `false` | 是否使用 mock hardware |
| `launch_rsp` | `true` | 是否启动 `robot_state_publisher` |

### `dual_revo2_system.launch.py`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `description_package` | `revo2_description` | 默认由驱动组装公开模型；自定义包需提供 `urdf/revo2.single.system.xacro` |
| `left_protocol` | `modbus` | 左手协议 |
| `right_protocol` | `modbus` | 右手协议 |
| `left_protocol_config_file` | `""` | 左手协议 YAML 覆盖路径 |
| `right_protocol_config_file` | `""` | 右手协议 YAML 覆盖路径 |
| `initial_positions_file` | `""` | 双手共享初始位置 YAML 覆盖路径 |
| `controllers_file` | `""` | 控制器模板覆盖路径 |
| `use_namespace` | `true` | 双手启动时强烈建议保持开启 |
| `if_sim` | `false` | 是否使用 mock hardware |
| `launch_rsp` | `true` | 是否分别启动左右手 `robot_state_publisher` |

## 运行时接口

### 控制器

以右手默认命名空间为例：

- controller manager：`/revo2_right/controller_manager`
- 默认激活控制器：`/revo2_right/joint_forward_pos_controller`
- 默认未激活控制器：`/revo2_right/joint_forward_vel_controller`
- 关节状态广播器：`/revo2_right/revo2_joint_state`

左手把 `revo2_right` 替换为 `revo2_left` 即可。

### 位置控制命令

默认激活的是位置前向控制器，命令类型为 `std_msgs/msg/Float64MultiArray`：

```bash
ros2 topic pub --once /revo2_right/joint_forward_pos_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.30, 0.10, 0.60, 0.60, 0.60, 0.60]}"
```

关节顺序为：

1. `thumb_proximal_joint`
2. `thumb_metacarpal_joint`
3. `index_proximal_joint`
4. `middle_proximal_joint`
5. `ring_proximal_joint`
6. `pinky_proximal_joint`

### 速度控制命令

速度前向控制器默认是 inactive，需要先切换控制器：

```bash
ros2 control switch_controllers \
  --controller-manager /revo2_right/controller_manager \
  --deactivate joint_forward_pos_controller \
  --activate joint_forward_vel_controller
```

然后发布速度命令：

```bash
ros2 topic pub --once /revo2_right/joint_forward_vel_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.17, 0.17, 0.35, 0.35, 0.35, 0.35]}"
```

### 状态话题

关节状态：

```bash
ros2 topic echo /revo2_right/revo2_joint_state/joint_states
```

包含触觉扩展接口的动态关节状态：

```bash
ros2 topic echo /revo2_right/revo2_joint_state/dynamic_joint_states
```

当前触觉数据会出现在以下关节上：

- `thumb_proximal_joint`
- `index_proximal_joint`
- `middle_proximal_joint`
- `ring_proximal_joint`
- `pinky_proximal_joint`

`thumb_metacarpal_joint` 当前不导出触觉接口。

## 排障建议

### 串口权限

```bash
ls -l /dev/ttyUSB*
groups | grep dialout
sudo usermod -a -G dialout $USER
```

添加用户组后请重新登录再测试。

### 双手 Modbus 自动检测

推荐默认值：

- 左手：`slave_id: 126`
- 右手：`slave_id: 127`

如果左右手都通过 USB 转串口连接，可以选择：

- 保持手动 `port`
- 开启 `auto_detect: true` 并保留不同的 `slave_id` 提示
- 用 `auto_detect_port` 缩小各自扫描范围

### 常用检查命令

```bash
ros2 control list_controllers -c /revo2_right/controller_manager
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 topic list | grep revo2_right
```

如果 `protocol:=canfd` 启动失败，请优先检查：

- 是否已经用 `-DENABLE_CANFD=ON` 重新编译
- 当前机器上的 CAN 设备和通道参数是否有效

## PID 积分项

`revo2_pid_controller` 支持独立六关节积分，参数仍放在 `pd_velocity` 下，兼容原配置：

```yaml
pd_velocity:
  velocity_kp: 6.7
  velocity_ki: 0.0
  velocity_kd: 0.8
  integral_max: 0.2
```

`velocity_ki` 的单位为 1/s²，默认 0，关闭积分并保留原 PD 行为。
每周期按 `I = clamp(I + Ki * 位置误差 * dt, -integral_max, integral_max)` 累积。
`integral_max` 单位为 rad/s，限制 I 项自身的贡献；设为 0 也会关闭积分。
I 项加在原关节 PD 缩放之后、前馈和四指伸展方向缩放之前，因此四指伸展缩放
仍可能放大它的最终贡献，最终总速度继续受 `velocity_max` 等限制。

积分在误差进入现有死区、误差反向、当前位置边界的向外积分、目标超时、控制器
失活/重新激活或位置反馈无效时清零。控制器速度限幅、拇指减速或每周期速度变化
限制发生饱和时，不累计继续推向饱和方向的积分。死区内不积分，所以积分不会自动
消除死区内的小误差。

参数在控制器配置阶段读取；修改 YAML 后重新加载控制器或重启驱动，不支持仅通过
`ros2 param set` 实时修改控制计算。首次启用时从小的 Ki、小幅空载目标开始调试。
抗积分饱和只覆盖控制器已知的输出限制，不能检测 SDK/固件内部限速、接触阻挡或
设备实际边界；应同时保证 `velocity_max` 与协议速度范围一致，目标限位与设备一致。
这项改动没有增加纯速度模式的硬件位置边界保护，也没有修复切回位置控制的旧目标问题。

## SDK v2.0.9 新能力

支持启动参数 `sdk_file_logging:=true/false`（留空使用协议 YAML），以及同一命名空间下的
`touch/enable`、`touch/calibrate` 服务，类型均为 `revo2_driver/srv/TouchSensorCommand`。
请求通过 `sensor_mask` 选择传感器（五指全部为 31）；校准时指尖应无接触、无负载。
服务成功仅表示通信操作成功，不代表物理校准完成或数据精度已验证。

## 轨迹控制模式

启动时固定激活 `joint_forward_pos_controller` 和关节状态广播器。
速度、PID、轨迹控制器只加载，不激活。需要轨迹控制时，启动后手动切换到
`left_revo2_hand_controller` 或 `right_revo2_hand_controller`。右手切换命令：

```bash
ros2 control switch_controllers -c /revo2_right/controller_manager \
  --deactivate joint_forward_pos_controller \
  --activate right_revo2_hand_controller --strict
```

右手 action 为 `/revo2_right/right_revo2_hand_controller/follow_joint_trajectory`，
目标必须包含全部六个关节，且 `time_from_start` 严格递增。
