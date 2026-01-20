# 双手配置故障排查指南

## 问题描述

在双手配置中，可能会出现以下问题：
- 第一个硬件实例（右手）成功检测并占用了 `/dev/ttyUSB0`
- 第二个硬件实例（左手）检测时，由于 `/dev/ttyUSB0` 已被占用，检测失败

## 解决方案

### 方案 1：手动指定端口（推荐）

如果自动检测在双手场景下有问题，可以手动指定不同的端口：

#### 左手配置 (`protocol_modbus_left.yaml`)

```yaml
hardware:
  protocol: modbus
  slave_id: 126
  auto_detect: false  # 禁用自动检测
  port: /dev/ttyUSB0  # 手动指定端口
  baudrate: 460800
```

#### 右手配置 (`protocol_modbus_right.yaml`)

```yaml
hardware:
  protocol: modbus
  slave_id: 127
  auto_detect: false  # 禁用自动检测
  port: /dev/ttyUSB2  # 手动指定不同的端口
  baudrate: 460800
```

**优点**：
- 最可靠的方法
- 避免端口冲突
- 启动速度快

**缺点**：
- 需要手动确定每个设备对应的端口

### 方案 2：使用端口提示

使用 `auto_detect_port` 提示来限制检测范围：

#### 左手配置

```yaml
hardware:
  protocol: modbus
  slave_id: 126
  auto_detect: true
  auto_detect_port: "/dev/ttyUSB0"  # 限制检测范围
```

#### 右手配置

```yaml
hardware:
  protocol: modbus
  slave_id: 127
  auto_detect: true
  auto_detect_port: "/dev/ttyUSB2"  # 限制检测范围
```

### 方案 3：检查设备端口

在启动前，先检查设备对应的端口：

```bash
# 列出所有 USB 串口设备
ls -l /dev/ttyUSB*

# 查看设备信息
udevadm info /dev/ttyUSB0
udevadm info /dev/ttyUSB2
```

## 常见问题

### Q: 如何确定哪个端口对应哪个手？

A: 可以通过以下方法：

1. **逐个测试**：
   - 先只连接一个设备，运行单 hand 配置
   - 查看日志中的 `detected_slave_id`
   - 记录该设备对应的端口和 slave_id

2. **查看设备序列号**：
   - 在日志中查找 `Device SN` 信息
   - 记录每个设备的序列号和对应的端口

3. **使用 udev 规则**：
   - 创建 udev 规则，根据设备序列号创建固定的符号链接
   - 例如：`/dev/revo2_left` 和 `/dev/revo2_right`

### Q: 为什么自动检测在双手场景下会失败？

A: SDK 的自动检测函数在检测时可能会尝试打开端口。如果第一个硬件实例已经打开了某个端口，第二个实例就无法再打开同一个端口进行检测。

### Q: 如何创建 udev 规则来固定端口？

A: 创建文件 `/etc/udev/rules.d/99-revo2.rules`：

```bash
# 根据设备序列号创建固定符号链接
# 替换 YOUR_LEFT_SERIAL 和 YOUR_RIGHT_SERIAL 为实际序列号
SUBSYSTEM=="tty", ATTRS{serial}=="YOUR_LEFT_SERIAL", SYMLINK+="revo2_left"
SUBSYSTEM=="tty", ATTRS{serial}=="YOUR_RIGHT_SERIAL", SYMLINK+="revo2_right"
```

然后重新加载 udev 规则：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

在配置文件中使用：

```yaml
port: /dev/revo2_left  # 或 /dev/revo2_right
```

## 推荐配置

对于生产环境，推荐使用**方案 1（手动指定端口）**，因为：
- 最可靠
- 避免端口冲突
- 启动速度快
- 不依赖自动检测的时序

对于开发环境，可以使用**方案 2（端口提示）**，在设备端口相对固定的情况下。

## 日志分析

如果遇到问题，查看日志中的以下信息：

1. **检测到的端口**：
   ```
   Auto-detection successful: port=/dev/ttyUSB0 baudrate=460800 detected_slave_id=126
   ```

2. **端口打开失败**：
   ```
   Failed to open Modbus connection on /dev/ttyUSB0
   Device is already in use by another process (common in dual-hand setup)
   ```

3. **设备信息**：
   ```
   Device SN: "BCXRL1XXXB2500020"
   Device FW Version: "1.0.13.U"
   ```

通过这些信息，可以确定：
- 哪个设备在哪个端口
- 是否有端口冲突
- 设备的 slave_id 是否正确

