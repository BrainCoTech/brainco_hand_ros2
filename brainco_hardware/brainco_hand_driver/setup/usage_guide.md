# Revo2 脚本使用说明（Modbus 串口）

完整说明见上级目录：[../doc/revo2_bimanual_modbus_serial_setup.md](../doc/revo2_bimanual_modbus_serial_setup.md)

---

## 快速开始

```bash
cd src/brainco_drivers/revo2_driver/setup

bash bootstrap_revo2.sh    # 默认 auto（SDK-free，pyserial 探测）
bash check_revo2_setup.sh

# 启动双手（Revoarm_ws，需先 bash ../scripts/download_sdk.sh + colcon build revo2_driver）
cd <Revoarm_ws>
source install/setup.bash
ros2 launch revo2_driver dual_revo2_system.launch.py
```

---

## 脚本列表

| 脚本 | 用途 | 示例 |
|------|------|------|
| `bootstrap_revo2.sh` | 一键配置（默认 auto） | `bash bootstrap_revo2.sh` |
| `discover_revo2_serial.sh` | 列串口 | `./discover_revo2_serial.sh` |
| `detect_revo2_ports_auto.sh` | 调 Python 探测器找 126/127 | bootstrap 默认内部调用 |
| `detect_revo2_ports.py` | SDK-free 探测器（pyserial Modbus） | 由 auto 脚本调用 |
| `setup_revo2_udev_rules.sh` | 写 udev（sudo） | `sudo bash setup_revo2_udev_rules.sh /dev/ttyACM6 l /dev/ttyACM1 r` |
| `check_revo2_setup.sh` | 健康检查 | `bash check_revo2_setup.sh` |

`l` / `r` = 左 / 右。

---

## 注意

- 默认 auto 调用 SDK-free 的 `detect_revo2_ports.py`（需 `python3-serial`）；失败可用 `--manual`
- bootstrap 只写 udev，**不会**改 yaml；日常 launch 请确认 yaml 中：
  - `port: /dev/revo2_hand_left` / `/dev/revo2_hand_right`
  - `auto_detect: false`
- 若 `ls -l /dev/revo2_hand_*` 两个别名指向同一 tty，重跑 `bash bootstrap_revo2.sh`（udev 脚本会自动 fallback 到 exact 模式）
- 仅 Modbus；不要用 `left_protocol:=canfd`
- `bootstrap` **不会**启动 ROS；启动双手请用 `dual_revo2_system.launch.py`
