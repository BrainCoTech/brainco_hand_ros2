#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <revo2_ethercat_plugins/logger_macros.hpp>
#include <stark_ethercat_interface/ec_slave.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace revo2_ethercat_plugins
{

/**
 * @brief Revo2 6关节系统 EtherCAT 驱动
 *
 * EtherCAT 从站驱动，处理所有6个关节的控制与状态反馈
 */
class Revo2JointsSystemSlave : public stark_ethercat_interface::EcSlave
{
public:
  Revo2JointsSystemSlave();
  virtual ~Revo2JointsSystemSlave();

  // EcSlave 接口实现
  virtual void processData(size_t index, uint8_t * domain_address) override;
  virtual const ec_sync_info_t * syncs() override;
  virtual size_t syncSize() override;
  virtual const ec_pdo_entry_info_t * channels() override;
  virtual void domains(DomainMap & domains) const override;
  virtual int assign_activate_dc_sync() override;
  virtual bool initialized() override { return is_operational_; }
  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_parameters,
    std::vector<double> * state_interface, std::vector<double> * command_interface) override;

private:
  // 常量定义
  static constexpr size_t NUM_FINGER_MOTORS = 6;
  static constexpr uint8_t SINGLE_MODE_POSITION_TIME = 1;    // 位置+时间控制
  static constexpr uint16_t MULTI_MODE_POSITION_TIME = 1;    // 多指：位置+时间控制
  static constexpr uint16_t CTRL_PARAM_DURATION_MS = 100;  // 默认运动时间

  // 控制模式枚举
  enum RxPdoControlMode
  {
    MULTI_FINGER_MODE = 0,  // 多指同时控制（默认）
    SINGLE_FINGER_MODE = 1  // 单指轮询控制
  };

  // 手指ID枚举
  enum FingerID
  {
    THUMB_FLEX = 0,     // 拇指尖部
    THUMB_ABDUCT = 1,   // 拇指根部
    INDEX_FINGER = 2,   // 食指
    MIDDLE_FINGER = 3,  // 中指
    RING_FINGER = 4,    // 无名指
    PINKY_FINGER = 5    // 小指
  };

  // 关节名称映射
  const std::array<std::string, NUM_FINGER_MOTORS> joint_names_ = {
    "thumb_flex_joint", "thumb_abduct_joint", "index_joint",
    "middle_joint",     "ring_joint",         "pinky_joint"};

  // 配置参数
  uint16_t ctrl_param_duration_ms_;    // 默认运动时间
  uint32_t assign_activate_;             // DC同步配置
  RxPdoControlMode rxpdo_control_mode_;  // 控制模式：多指/单指

  // PDO 通道定义
  std::vector<ec_pdo_entry_info_t> channels_;
  std::vector<ec_pdo_info_t> rxpdo_;  // Master -> Slave (命令)
  std::vector<ec_pdo_info_t> txpdo_;  // Slave -> Master (状态)
  std::vector<ec_sync_info_t> syncs_;

  // 域映射
  DomainMap domain_map_;

  // 接口映射索引 (6个关节)
  struct JointInterface
  {
    int position_cmd;    // command_interface 中的位置索引
    int position_state;  // state_interface 中的位置索引
    int velocity_state;  // state_interface 中的速度索引
    int effort_state;    // state_interface 中的力矩索引
  };
  std::array<JointInterface, NUM_FINGER_MOTORS> joint_interfaces_;

  // 当前状态缓存
  std::array<double, NUM_FINGER_MOTORS> last_position_cmds_;
  std::array<double, NUM_FINGER_MOTORS> current_position_states_;
  std::array<double, NUM_FINGER_MOTORS> current_velocity_states_;
  std::array<double, NUM_FINGER_MOTORS> current_effort_states_;

  // 手指马达状态缓存 (6个马达)
  std::array<uint16_t, NUM_FINGER_MOTORS> finger_motor_status_;  // 马达状态值

  // 触觉数据缓存 (Touch设备专用，共5个手指)
  static constexpr size_t NUM_TOUCH_FINGERS = 5;
  std::array<uint16_t, NUM_TOUCH_FINGERS> touch_normal_force_;      // 法向力 (0.01N精度)
  std::array<uint16_t, NUM_TOUCH_FINGERS> touch_tangential_force_;  // 切向力 (0.01N精度)
  std::array<uint16_t, NUM_TOUCH_FINGERS> touch_direction_;         // 方向角 (度)
  std::array<uint32_t, NUM_TOUCH_FINGERS> touch_proximity_;         // 接近值
  std::array<uint16_t, NUM_TOUCH_FINGERS> touch_status_;            // 状态值
  // 日志节流缓存
  std::array<double, NUM_FINGER_MOTORS> last_logged_positions_;
  std::array<double, NUM_FINGER_MOTORS> last_logged_velocities_;
  std::array<double, NUM_FINGER_MOTORS> last_logged_efforts_;
  std::array<double, NUM_FINGER_MOTORS> last_logged_cmd_pos_;

  // 控制逻辑状态
  uint8_t active_finger_id_;  // 当前正在控制的手指ID
  bool finger_control_busy_;  // 是否有手指正在运动中

  bool is_physical_mode_ = false;
  bool is_touch_device_ = false;

  // 辅助函数
  void setupChannels();
  void setupSyncManagers();
  void setupDomainMapping();
  void setupInterfaceMapping();

  // 单手指控制更新
  void updateActiveFingerFromCommands();

  // 单位转换
  double revo2ToRadian(int16_t revo2_pos);
  int16_t radianToRevo2(double radian_pos);

  // 状态转换
  const char * getMotorStatusString(uint16_t status);

  // 单位转换 0.1deg(from device) to rad
  inline double degToRad(double deg) const { return deg * 0.1 * M_PI / 180.0; }
  inline double radToDeg(double rad) const { return rad * 180.0 / M_PI * 10.0; }

  // 数据类型转换
  inline void write_u8(uint8_t * data, uint8_t value) { EC_WRITE_U8(data, value); }

  inline void write_u16(uint8_t * data, uint16_t value) { EC_WRITE_U16(data, value); }

  inline void write_s16(uint8_t * data, int16_t value) { EC_WRITE_S16(data, value); }

  inline uint8_t read_u8(const uint8_t * data) { return EC_READ_U8(data); }

  inline uint16_t read_u16(const uint8_t * data) { return EC_READ_U16(data); }

  inline int16_t read_s16(const uint8_t * data) { return EC_READ_S16(data); }
};

}  // namespace revo2_ethercat_plugins
