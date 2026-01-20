# USB 端口自动检测功能文档

## 概述

本驱动实现了智能的 USB 串口自动检测功能，能够自动扫描所有可用的串口设备，找到 Revo2 灵巧手设备，并自动选择可用的端口。这对于以下场景特别有用：

- 设备端口不固定（如 `/dev/ttyUSB0`, `/dev/ttyUSB1`, `/dev/ttyUSB2` 等）
- 双手配置场景，需要自动识别和分配不同的端口
- 设备热插拔后端口号可能变化

## 功能特性

### 1. 智能端口扫描
- 自动扫描所有可用的 USB 串口（`/dev/ttyUSB*`, `/dev/ttyACM*` 等）
- 支持扫描 0-15 号端口
- 自动跳过不存在的端口

### 2. 端口占用检测
- 自动检测端口是否被其他进程占用
- 如果端口被占用，自动尝试下一个可用端口
- 特别适用于双手配置场景

### 3. 双重检测机制
- **快速检测**：首先使用 SDK 的自动检测功能（快速、可靠）
- **完整扫描**：如果快速检测失败或端口被占用，手动扫描所有端口

### 4. 设备匹配
- 通过 `slave_id` 自动匹配左右手设备
- 提供详细的检测日志和匹配信息

## 配置方法

### 基本配置

在配置文件中启用自动检测：

```yaml
hardware:
  protocol: modbus
  slave_id: 127  # 右手通常使用 127，左手使用 126
  auto_detect: true  # 启用自动检测
  auto_detect_quick: true  # 快速检测模式（推荐）
  auto_detect_port: ""  # 留空检测所有端口
  # 当 auto_detect 为 true 时，以下参数会被忽略
  port: /dev/ttyUSB0
  baudrate: 460800
```

### 配置参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `auto_detect` | bool | `false` | 是否启用自动检测 |
| `auto_detect_quick` | bool | `true` | 快速检测模式（推荐）。`false` 会检测所有可能的 slave_id（1-247），速度较慢 |
| `auto_detect_port` | string | `""` | 端口提示。留空检测所有端口，或指定提示如 `"/dev/ttyUSB"` 只检测 USB 端口 |

### 端口提示使用示例

```yaml
# 只检测 USB 端口
auto_detect_port: "/dev/ttyUSB"

# 只检测 ACM 端口
auto_detect_port: "/dev/ttyACM"

# 检测特定端口（如果该端口被占用，会继续扫描其他端口）
auto_detect_port: "/dev/ttyUSB0"

# 留空检测所有类型的串口
auto_detect_port: ""
```

## 工作原理

### 检测流程

```
1. 启动自动检测
   ↓
2. 尝试 SDK 快速检测
   ├─ 成功 → 尝试打开端口
   │   ├─ 成功 → 使用该设备 ✓
   │   └─ 失败（被占用）→ 继续步骤 3
   └─ 失败 → 继续步骤 3
   ↓
3. 扫描所有可用串口
   ├─ /dev/ttyUSB0 → 检测设备 → 尝试打开
   │   ├─ 成功 → 使用该设备 ✓
   │   └─ 失败（被占用）→ 下一个端口
   ├─ /dev/ttyUSB1 → 检测设备 → 尝试打开
   │   ├─ 成功 → 使用该设备 ✓
   │   └─ 失败（被占用）→ 下一个端口
   ├─ /dev/ttyUSB2 → ...
   └─ ...
   ↓
4. 找到可用设备或所有端口检测完毕
```

### 端口扫描范围

系统会自动扫描以下端口模式：

- `/dev/ttyUSB0` 到 `/dev/ttyUSB15`
- `/dev/ttyACM0` 到 `/dev/ttyACM15`
- `/dev/ttyS0` 到 `/dev/ttyS15`

如果指定了 `auto_detect_port` 提示，则只扫描匹配的端口。

### 端口占用处理

当检测到端口被占用时（常见于双手配置场景）：

1. **记录警告日志**：提示端口被占用
2. **继续扫描**：自动尝试下一个可用端口
3. **找到可用设备**：使用第一个可用的设备

## 使用场景

### 场景 1：单 hand 配置

```yaml
hardware:
  protocol: modbus
  slave_id: 127
  auto_detect: true
  auto_detect_quick: true
  auto_detect_port: ""
```

**特点**：
- 自动找到第一个可用的 Revo2 设备
- 无需关心设备连接在哪个端口

### 场景 2：双手配置

#### 左手配置 (`protocol_modbus_left.yaml`)

```yaml
hardware:
  protocol: modbus
  slave_id: 126
  auto_detect: true
  auto_detect_quick: true
  auto_detect_port: ""
```

#### 右手配置 (`protocol_modbus_right.yaml`)

```yaml
hardware:
  protocol: modbus
  slave_id: 127
  auto_detect: true
  auto_detect_quick: true
  auto_detect_port: ""
```

**工作原理**：
1. 右手先启动，检测并占用 `/dev/ttyUSB0`
2. 左手启动时，检测到 `/dev/ttyUSB0` 被占用
3. 自动继续扫描，找到 `/dev/ttyUSB2` 并使用它
4. 两个设备成功连接

### 场景 3：限制检测范围

如果知道设备大致在哪些端口，可以限制检测范围：

```yaml
hardware:
  protocol: modbus
  slave_id: 127
  auto_detect: true
  auto_detect_port: "/dev/ttyUSB"  # 只检测 USB 端口，不检测 ACM
```

## 日志示例

### 成功检测示例

```
[2025-12-17 19:56:19.738 INFO] Starting auto-detection for Modbus Revo2 device (requested slave_id: 127, quick: true)
[2025-12-17 19:56:19.738 INFO] Found 2 serial port(s), scanning for Revo2 devices...
[2025-12-17 19:56:19.746 INFO] SDK auto-detection found available device on /dev/ttyUSB0
[2025-12-17 19:56:19.751 INFO] Auto-detection successful: port=/dev/ttyUSB0 baudrate=460800 detected_slave_id=126
[2025-12-17 19:56:19.751 WARN] Detected device slave_id (126) does not match requested slave_id (127)
[2025-12-17 19:56:19.751 INFO] Modbus connection established: port=/dev/ttyUSB0 baudrate=460800 slave_id=126
```

### 端口被占用，继续扫描示例

```
[2025-12-17 19:56:19.762 INFO] Starting auto-detection for Modbus Revo2 device (requested slave_id: 126, quick: true)
[2025-12-17 19:56:19.762 INFO] Found 2 serial port(s), scanning for Revo2 devices...
[2025-12-17 19:56:19.769 WARN] SDK detected device on /dev/ttyUSB0 but port is in use. Scanning other ports...
[2025-12-17 19:56:19.769 INFO] Trying port: /dev/ttyUSB1
[2025-12-17 19:56:19.770 INFO] Port /dev/ttyUSB1 is in use, trying next port...
[2025-12-17 19:56:19.771 INFO] Trying port: /dev/ttyUSB2
[2025-12-17 19:56:19.775 INFO] Found available device on port: /dev/ttyUSB2
[2025-12-17 19:56:19.775 INFO] Auto-detection successful: port=/dev/ttyUSB2 baudrate=460800 detected_slave_id=126
[2025-12-17 19:56:19.775 INFO] Detected device slave_id (126) matches requested slave_id - correct hand detected!
[2025-12-17 19:56:19.776 INFO] Modbus connection established: port=/dev/ttyUSB2 baudrate=460800 slave_id=126
```

### 检测失败示例

```
[2025-12-17 19:56:19.738 INFO] Starting auto-detection for Modbus Revo2 device (requested slave_id: 127, quick: true)
[2025-12-17 19:56:19.738 INFO] Found 0 serial port(s), scanning for Revo2 devices...
[2025-12-17 19:56:19.750 ERROR] Auto-detection failed: No available Revo2 device found. Please check:
  1. Device is connected and powered on
  2. USB cable is properly connected
  3. Device permissions (try: sudo chmod 666 /dev/ttyUSB*)
  4. For dual-hand setup, ensure both devices are connected
  5. Some ports may be in use by other hardware instances
```

## 故障排查

### 问题 1：检测不到设备

**可能原因**：
- 设备未连接或未上电
- USB 线缆连接不良
- 设备权限不足

**解决方法**：
```bash
# 检查设备是否存在
ls -l /dev/ttyUSB*

# 检查设备权限
sudo chmod 666 /dev/ttyUSB*

# 将用户添加到 dialout 组（推荐）
sudo usermod -a -G dialout $USER
# 然后重新登录
```

### 问题 2：端口被占用

**现象**：日志显示 "port is in use"

**原因**：
- 另一个硬件实例正在使用该端口
- 其他程序正在使用该端口

**解决方法**：
- 系统会自动跳过被占用的端口，继续扫描其他端口
- 如果所有端口都被占用，检查是否有其他进程在使用设备：
  ```bash
  lsof /dev/ttyUSB*
  ```

### 问题 3：检测到错误的设备

**现象**：检测到的 `slave_id` 与请求的不匹配

**原因**：
- 设备的 `slave_id` 配置不正确
- 连接的是错误的手（左手/右手）

**解决方法**：
1. 检查设备的实际 `slave_id`
2. 确认配置文件中的 `slave_id` 是否正确
3. 如果检测到的 `slave_id` 是正确的，可以忽略警告

### 问题 4：检测速度慢

**原因**：
- `auto_detect_quick: false` 会检测所有可能的 slave_id（1-247）

**解决方法**：
- 使用 `auto_detect_quick: true`（推荐）
- 或者使用 `auto_detect_port` 限制检测范围

## 最佳实践

### 1. 生产环境

**推荐配置**：
```yaml
auto_detect: true
auto_detect_quick: true
auto_detect_port: ""  # 或指定端口类型如 "/dev/ttyUSB"
```

**优点**：
- 自动适应端口变化
- 启动速度快
- 支持热插拔

### 2. 开发调试

**可选配置**：
```yaml
auto_detect: false
port: /dev/ttyUSB0  # 手动指定端口
```

**优点**：
- 端口固定，便于调试
- 启动速度最快

### 3. 双手配置

**推荐配置**：
- 两个硬件实例都启用 `auto_detect: true`
- 系统会自动分配不同的端口
- 无需手动指定端口

## 技术细节

### 端口扫描实现

系统使用以下方法扫描端口：

1. **文件系统检查**：使用 `std::filesystem::exists()` 检查端口文件是否存在
2. **SDK 检测**：调用 `auto_detect_modbus_revo2()` 进行快速检测
3. **端口测试**：尝试打开端口，验证是否可用
4. **设备验证**：确认检测到的设备是 Revo2 设备

### 性能考虑

- **快速模式**（`auto_detect_quick: true`）：只检测常见的 slave_id，速度较快（推荐）
- **完整模式**（`auto_detect_quick: false`）：检测所有可能的 slave_id（1-247），速度较慢

### 端口占用检测

系统通过尝试打开端口来检测是否被占用：
- 如果 `modbus_open()` 成功，说明端口可用
- 如果失败，说明端口被占用或设备不存在
- 被占用的端口会被自动跳过

## 相关文档

- [自动检测使用指南](./AUTO_DETECT_GUIDE.md)
- [双手配置故障排查](./DUAL_HAND_TROUBLESHOOTING.md)
- [README](./README.md)

## 更新日志

### 2025-12-17
- 实现智能端口扫描功能
- 支持自动跳过被占用的端口
- 添加详细的检测日志
- 优化双手配置场景的处理

