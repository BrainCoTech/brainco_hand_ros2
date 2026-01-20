# 串口自动检测使用指南

## 概述

本驱动支持自动检测和选择串口功能，特别适用于：
- 串口设备路径不固定的场景（如 `/dev/ttyUSB0`, `/dev/ttyUSB1` 等）
- 双手配置场景，需要自动识别左右手设备

## 功能特性

1. **自动检测串口**：自动扫描所有可用串口，找到 Revo2 设备
2. **智能匹配**：通过 `slave_id` 自动匹配左右手设备
3. **详细日志**：提供详细的检测和连接日志，便于排查问题
4. **错误提示**：连接失败时提供友好的错误信息和解决建议

## 配置方法

### 单 hand 配置

编辑配置文件 `config/protocol_modbus_right.yaml`：

```yaml
hardware:
  protocol: modbus
  slave_id: 127  # 右手通常使用 127，左手使用 126
  auto_detect: true  # 启用自动检测
  auto_detect_quick: true  # 快速检测模式（推荐）
  auto_detect_port: ""  # 留空检测所有串口，或指定提示如 "/dev/ttyUSB"
  # 当 auto_detect 为 true 时，以下参数会被忽略，使用检测到的值
  port: /dev/ttyUSB1
  baudrate: 460800
```

### 双手配置

#### 左手配置 (`config/protocol_modbus_left.yaml`)

```yaml
hardware:
  protocol: modbus
  slave_id: 126  # 左手通常使用 126
  auto_detect: true
  auto_detect_quick: true
  auto_detect_port: ""  # 留空或指定提示
```

#### 右手配置 (`config/protocol_modbus_right.yaml`)

```yaml
hardware:
  protocol: modbus
  slave_id: 127  # 右手通常使用 127
  auto_detect: true
  auto_detect_quick: true
  auto_detect_port: ""  # 留空或指定提示
```

## 工作原理

### 自动检测流程

1. **启动检测**：当 `auto_detect: true` 时，驱动会调用 SDK 的自动检测功能
2. **扫描串口**：扫描所有可用串口（或根据 `auto_detect_port` 提示限制范围）
3. **识别设备**：尝试连接每个串口，识别 Revo2 设备
4. **匹配 slave_id**：
   - 如果检测到的 `slave_id` 与配置的匹配，直接使用
   - 如果不匹配，会给出警告但继续使用检测到的设备
5. **建立连接**：使用检测到的端口和参数建立连接

### 双手场景处理

在双手配置中：
- 每个硬件实例（左手/右手）独立进行自动检测
- 通过 `slave_id` 来区分左右手设备
- 如果检测到的 `slave_id` 与请求的不匹配，会记录警告

**注意**：如果两个设备都连接在同一台机器上，自动检测可能会找到第一个设备。建议：
1. 使用 `auto_detect_port` 提示来限制检测范围
2. 或者确保设备的 `slave_id` 正确配置（左手 126，右手 127）

## 配置参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `auto_detect` | 是否启用自动检测 | `false` |
| `auto_detect_quick` | 快速检测模式（推荐） | `true` |
| `auto_detect_port` | 检测端口提示，留空检测所有串口 | `""` |
| `slave_id` | 设备从站ID，左手通常126，右手通常127 | `126` 或 `127` |

### auto_detect_port 使用示例

```yaml
# 检测所有 ttyUSB* 设备
auto_detect_port: "/dev/ttyUSB"

# 检测所有 ttyACM* 设备
auto_detect_port: "/dev/ttyACM"

# 留空检测所有串口
auto_detect_port: ""
```

## 故障排查

### 问题：自动检测失败

**可能原因**：
1. 设备未连接或未上电
2. USB 线缆连接不良
3. 设备权限不足
4. 设备被其他进程占用

**解决方法**：
```bash
# 检查设备是否存在
ls -l /dev/ttyUSB*

# 检查设备权限
sudo chmod 666 /dev/ttyUSB*

# 检查是否有其他进程占用
lsof /dev/ttyUSB*
```

### 问题：检测到的 slave_id 不匹配

**可能原因**：
1. 设备的 slave_id 配置不正确
2. 连接的是错误的手（左手/右手）

**解决方法**：
1. 检查设备的实际 slave_id
2. 确认配置文件中的 `slave_id` 是否正确
3. 如果检测到的 slave_id 是正确的，可以忽略警告

### 问题：双手配置时检测到错误的设备

**解决方法**：
1. 使用 `auto_detect_port` 提示来限制检测范围
2. 确保两个设备的 `slave_id` 不同（左手 126，右手 127）
3. 如果仍然有问题，可以临时禁用自动检测，手动指定端口

## 日志示例

### 成功检测示例

```
[2025-12-17 19:42:01.250 INFO] Starting auto-detection for Modbus Revo2 device (requested slave_id: 127, quick: true)
[2025-12-17 19:42:01.251 INFO] Auto-detection successful: port=/dev/ttyUSB1 baudrate=460800 detected_slave_id=127
[2025-12-17 19:42:01.252 INFO] Detected device slave_id (127) matches requested slave_id - correct hand detected!
[2025-12-17 19:42:01.253 INFO] Modbus connection established: port=/dev/ttyUSB1 baudrate=460800 slave_id=127
```

### 检测失败示例

```
[2025-12-17 19:42:01.250 INFO] Starting auto-detection for Modbus Revo2 device (requested slave_id: 127, quick: true)
[2025-12-17 19:42:01.251 ERROR] Auto-detection failed: No Revo2 device found. Please check:
  1. Device is connected and powered on
  2. USB cable is properly connected
  3. Device permissions (try: sudo chmod 666 /dev/ttyUSB*)
  4. No other process is using the device
  5. For dual-hand setup, ensure both devices are connected
```

### slave_id 不匹配警告示例

```
[2025-12-17 19:42:01.250 INFO] Starting auto-detection for Modbus Revo2 device (requested slave_id: 127, quick: true)
[2025-12-17 19:42:01.251 INFO] Auto-detection successful: port=/dev/ttyUSB0 baudrate=460800 detected_slave_id=126
[2025-12-17 19:42:01.252 WARN] Detected device slave_id (126) does not match requested slave_id (127). This may indicate:
  1. The device connected is not the expected hand (left/right)
  2. Device slave_id has been changed
  Using detected slave_id: 126
```

## 最佳实践

1. **生产环境**：建议启用 `auto_detect: true`，避免因端口变化导致的问题
2. **开发调试**：可以临时禁用自动检测，手动指定端口进行调试
3. **双手配置**：确保两个设备的 `slave_id` 不同，并正确配置在对应的配置文件中
4. **权限设置**：建议将用户添加到 `dialout` 组，避免权限问题：
   ```bash
   sudo usermod -a -G dialout $USER
   ```

## 回退到手动配置

如果自动检测有问题，可以回退到手动配置：

```yaml
hardware:
  protocol: modbus
  slave_id: 127
  auto_detect: false  # 禁用自动检测
  port: /dev/ttyUSB1  # 手动指定端口
  baudrate: 460800
```

