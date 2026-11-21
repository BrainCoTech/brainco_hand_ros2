# Revo2 串口 setup（Modbus）

与 `Revoarm_can/setup/` 对齐：发现 → bootstrap → udev → 检查。

**中文说明**：[usage_guide.md](./usage_guide.md) · [setup_changelog.md](./setup_changelog.md)（2026-05-22 目录重构）
**包级文档**：[../doc/revo2_bimanual_modbus_serial_setup.md](../doc/revo2_bimanual_modbus_serial_setup.md)  
**仓库记录**：`doc/5.21/Revo2二代手Modbus串口配置.md`

---

## 快速开始

```bash
cd robots/revoarm_teleoperation/revoarm_hardware/Revoarm_ws/src/brainco_drivers/revo2_driver

cd setup
bash bootstrap_revo2.sh               # 默认 auto（SDK-free，pyserial 探测，无需编译）
bash check_revo2_setup.sh
```

日常 launch 跑手仍需 `colcon build revo2_driver` + Stark SDK（`scripts/download_sdk.sh`）；**串口 bootstrap 与 colcon / SDK 无关**（探测由纯 Python `detect_revo2_ports.py` 直发 Modbus 完成）。

---

## 脚本一览

| 脚本 | 用途 |
|------|------|
| `discover_revo2_serial.sh` | 列 tty + USB 拓扑 |
| `bootstrap_revo2.sh` | 一键 udev 绑定（默认 auto；`--manual` 交互） |
| `detect_revo2_ports_auto.sh` | 调 Python 探测器扫 126/127（bootstrap 默认内部调用） |
| `detect_revo2_ports.py` | SDK-free 探测器（pyserial 直发 Modbus RTU；依赖 `python3-serial`） |
| `setup_revo2_udev_rules.sh` | 写 udev（sudo） |
| `check_revo2_setup.sh` | 检查 `/dev/revo2_hand_*` |
