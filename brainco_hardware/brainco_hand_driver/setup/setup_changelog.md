# Revo2 setup 目录变更说明

日期：2026-05-22  
目的：与 `Revoarm_can/setup/` 对齐，去掉重复入口，串口 bootstrap 与 ROS 驱动编译解耦。

> **2026-06-15 更新：探测器改为 SDK-free。** 删除 C++ 探测工具
> （`src/detect_revo2_ports.cpp`、`CMakeLists.txt`、`build_detect_revo2_ports.sh`、
> `bin/`、`build/`），改用纯 Python `detect_revo2_ports.py`（pyserial 直发 Modbus RTU，
> slave 126/127、波特率 460800），不再依赖 Stark SDK，免去 SDK 升级后改源码重编。
> 依赖 `python3-serial`（已加入 `scripts/bootstrap_teleop.sh` 的 apt 列表）。
> 注：Stark SDK 仍由 ROS 驱动 `revo2_driver` 运行时使用，未删除。
> 下文 2026-05-22 的目录树/构建说明描述的是旧 C++ 方案，仅作历史保留。

---

## 1. 改了什么（摘要）

| 项 | 改前 | 改后 |
|----|------|------|
| setup 脚本位置 | `revo2_driver/scripts/` | `revo2_driver/setup/` |
| `scripts/` 目录 | bootstrap / discover / udev 等 + wrapper | **仅保留** `scripts/download_sdk.sh` |
| `detect_revo2_ports.cpp` | 在包根 `src/`，由 `revo2_driver` CMake 编 | 在 `setup/src/`，由 **`setup/CMakeLists.txt` 独立编** |
| 探测二进制 | `colcon build` 产物 | `setup/bin/detect_revo2_ports` |
| `bootstrap_revo2.sh` | 需 `--auto` 才自动认口 | **无参数默认 auto**；`--manual` 为交互 |
| SDK 下载 | 曾在 `scripts/` 与 `setup/` 重复 | 只在 **`scripts/download_sdk.sh`** |

---

## 2. 现行目录结构

```text
revo2_driver/
├── CMakeLists.txt              # 仅 ROS 驱动（brainco_hand_*.cpp）
├── scripts/
│   └── download_sdk.sh         # 编译前下载 Stark SDK → vendor/
└── setup/
    ├── CMakeLists.txt          # 仅 detect_revo2_ports
    ├── build_detect_revo2_ports.sh
    ├── bootstrap_revo2.sh      # 主入口（默认 auto）
    ├── discover_revo2_serial.sh
    ├── detect_revo2_ports_auto.sh
    ├── setup_revo2_udev_rules.sh
    ├── check_revo2_setup.sh
    ├── role_mapping_utils.sh
    ├── src/detect_revo2_ports.cpp
    ├── bin/detect_revo2_ports    # 编译产物（.gitignore）
    ├── build/                    # cmake 中间目录（.gitignore）
    ├── README.md
    ├── usage_guide.md
    └── setup_changelog.md          # 本文
```

---

## 3. 分步说明

### 3.1 脚本迁到 `setup/`

以下脚本从 `scripts/` 迁入 `setup/`，并**删除** `scripts/` 里的重复文件与转发 wrapper：

- `bootstrap_revo2.sh`
- `discover_revo2_serial.sh`
- `setup_revo2_udev_rules.sh`
- `check_revo2_setup.sh`
- `detect_revo2_ports_auto.sh`
- `role_mapping_utils.sh`

### 3.2 `download_sdk.sh` 留在 `scripts/`

SDK 下载是**编译前依赖**（驱动与探测工具都要用），不是 udev setup 流程，与 `revo3_driver/scripts/download_sdk.sh` 一致，故保留在 `scripts/`，不放入 `setup/`。

### 3.3 探测工具独立编译

- 源码：`setup/src/detect_revo2_ports.cpp`
- 构建：`bash setup/build_detect_revo2_ports.sh`（内部 `cmake` + `make`）
- 产物：`setup/bin/detect_revo2_ports`
- **不再**写入包根 `CMakeLists.txt`，**不再**要求 `colcon build` 才能 `--auto`

`detect_revo2_ports_auto.sh` 行为：

1. 若 `setup/bin/detect_revo2_ports` 不存在 → 自动调用 `build_detect_revo2_ports.sh`
2. 设置 `LD_LIBRARY_PATH` 指向 `vendor/dist/shared/linux`
3. 链接时使用 `$ORIGIN/../../vendor/...` RPATH，避免运行时找不到 `libbc_stark_sdk.so`

### 3.4 `bootstrap_revo2.sh` 默认 auto

| 命令 | 行为 |
|------|------|
| `bash bootstrap_revo2.sh` | 默认：SDK 扫 126/127 → udev → 权限 → check |
| `bash bootstrap_revo2.sh --auto` | 同上（兼容旧写法） |
| `bash bootstrap_revo2.sh --manual` | 交互：discover + 手输串口 |
| `bash bootstrap_revo2.sh /dev/ttyACM6 l /dev/ttyACM1 r` | 手动指定左右 |

---

## 4. 推荐使用流程

**首次（串口 udev，与 colcon 无关）：**

```bash
cd revo2_driver
bash scripts/download_sdk.sh
cd setup
bash bootstrap_revo2.sh
bash check_revo2_setup.sh
```

**日常 launch 双手：**

```bash
cd Revoarm_ws
colcon build --packages-select revo2_driver
source install/setup.bash
ros2 launch revo2_driver dual_revo2_system.launch.py
```

---

## 5. 与 CAN setup 的对应关系

| 子系统 | setup 目录 | 日常 init |
|--------|------------|-----------|
| CAN | `Revoarm_can/setup/` | `Revoarm_ws/can_init.sh` |
| Revo2 手 | `revo2_driver/setup/` | udev 稳定名后一般无需每次 bootstrap |

---

## 6. 仓库内相关文档

| 文档 | 内容 |
|------|------|
| [README.md](./README.md) | setup 快速开始与脚本表 |
| [usage_guide.md](./usage_guide.md) | 命令示例与注意事项 |
| [../doc/revo2_bimanual_modbus_serial_setup.md](../doc/revo2_bimanual_modbus_serial_setup.md) | 包级完整说明 |
| `doc/5.22/Revo2_setup目录重构方案.md` | 重构方案与阶段记录 |
| `doc/5.21/Revo2二代手Modbus串口配置.md` | 仓库级 Modbus 配置 |

---

## 7. 已知限制

- auto 需双手 USB 接通且 Modbus slave_id 为 **126（左）/ 127（右）**；否则用 `--manual` 或手动 `l`/`r`。
- `setup/bin/`、`setup/build/` 不提交 git（见 `setup/.gitignore`），新 clone 后首次 bootstrap 会自动编译探测工具。
- 真机全流程（bootstrap + launch）尚需在硬件上验收。
