#include "revo2_ethercat_plugins/revo2_joints_system_slave.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <stark_ethercat_interface/ec_sdo_manager.hpp>
#include "revo2_ethercat_plugins/execute_command.hpp"

// DEBUG_LOG
#define ENABLE_DEBUG_LOG_TxPDO 0

namespace revo2_ethercat_plugins
{

Revo2JointsSystemSlave::Revo2JointsSystemSlave()
: EcSlave(0x00BC0000, 0x00009252)  // Revo2 Vendor ID & Product ID
  ,
  ctrl_param_duration_ms_(CTRL_PARAM_DURATION_MS),
  assign_activate_(0x0000),
  rxpdo_control_mode_(MULTI_FINGER_MODE)  // 默认多指控制
  ,
  active_finger_id_(0),
  finger_control_busy_(false),
  is_physical_mode_(false),
  is_touch_device_(false)
{
  REVO2_LOG_INFO("Initializing Revo2 Joints System EtherCAT Slave Plugin");
  REVO2_LOG_INFO("build time: %s - %s", __DATE__, __TIME__);

  // 初始化状态缓存
  last_position_cmds_.fill(0.0);
  current_position_states_.fill(0.0);
  current_velocity_states_.fill(0.0);
  current_effort_states_.fill(0.0);

  // 日志节流辅助变量（首次打印与变化打印）
  last_logged_positions_.fill(std::numeric_limits<double>::quiet_NaN());
  last_logged_velocities_.fill(std::numeric_limits<double>::quiet_NaN());
  last_logged_efforts_.fill(std::numeric_limits<double>::quiet_NaN());
  last_logged_cmd_pos_.fill(std::numeric_limits<double>::quiet_NaN());

  // 初始化马达状态数据
  finger_motor_status_.fill(0);

  // 初始化触觉数据
  touch_normal_force_.fill(0);
  touch_tangential_force_.fill(0);
  touch_direction_.fill(0xFFFF);  // 0xFFFF表示方向无效
  touch_proximity_.fill(0);
  touch_status_.fill(0);
}

Revo2JointsSystemSlave::~Revo2JointsSystemSlave()
{
  REVO2_LOG_INFO("Revo2 Joints System EtherCAT Slave Plugin destroyed");
}

bool Revo2JointsSystemSlave::setupSlave(
  std::unordered_map<std::string, std::string> slave_parameters,
  std::vector<double> * state_interface, std::vector<double> * command_interface)
{
  REVO2_LOG_INFO("Setting up Revo2 Joints System EtherCAT Slave");

  // 调用基类
  EcSlave::setupSlave(slave_parameters, state_interface, command_interface);

  // 解析参数
  if (paramters_.find("ctrl_param_duration_ms") != paramters_.end())
  {
    ctrl_param_duration_ms_ =
      static_cast<uint16_t>(std::stoi(paramters_["ctrl_param_duration_ms"]));
  }
  if (paramters_.find("assign_activate") != paramters_.end())
  {
    assign_activate_ = static_cast<uint32_t>(std::stoul(paramters_["assign_activate"], nullptr, 0));
  }

  REVO2_LOG_INFO(
    "Default ctrl_param_duration_ms_: %u ms, Assign Activate: 0x%04X, Control Mode: %s", ctrl_param_duration_ms_,
    assign_activate_,
    (rxpdo_control_mode_ == MULTI_FINGER_MODE) ? "Multi-Finger" : "Single-Finger");

  // 首先进行设备检测，确定设备类型
  try
  {
    std::string command;
    std::string output;
    // 读取固件版本
    command = "ethercat upload -t string -p 0 0x8000 0x11";
    output = ExecuteCommand::run(command);
    REVO2_LOG_INFO("Run command: %s, CTRL FW version: %s", command.c_str(), output.c_str());
    command = "ethercat upload -t string -p 0 0x8000 0x13";
    output = ExecuteCommand::run(command);
    REVO2_LOG_INFO("Run command: %s, Wrist FW version: %s", command.c_str(), output.c_str());

    // 检测设备类型
    command = "ethercat upload -t string -p 0 0x1008 0x00";
    output = ExecuteCommand::run(command);
    REVO2_LOG_INFO("Run command: %s, Device name: %s", command.c_str(), output.c_str());
    // 去除前后空白和换行
    output.erase(0, output.find_first_not_of(" \t\r\n"));
    output.erase(output.find_last_not_of(" \t\r\n") + 1);
    is_touch_device_ = (output.find("Touch") != std::string::npos);
    REVO2_LOG_INFO("Device type detected: %s", is_touch_device_ ? "Touch" : "Pro");

    command = "ethercat upload -t raw -p 0 0x8000 0x05";
    output = ExecuteCommand::run(command);
    REVO2_LOG_INFO("Run command: %s, unit_mode: %s", command.c_str(), output.c_str());
    int value = 0;
    // 去除前后空白和换行
    output.erase(0, output.find_first_not_of(" \t\r\n"));
    output.erase(output.find_last_not_of(" \t\r\n") + 1);
    if (output.rfind("0x", 0) == 0 || output.rfind("0X", 0) == 0)
    {
      // 以0x开头，按16进制解析
      value = std::stoi(output, nullptr, 16);
    }
    else
    {
      // 否则按10进制解析
      value = std::stoi(output);
    }
    is_physical_mode_ = (value == 1);
    REVO2_LOG_INFO("is_physical_mode_: %d (raw value: %d)", is_physical_mode_, value);
  }
  catch (const std::exception & e)
  {
    REVO2_LOG_ERROR("Error during device detection: %s", e.what());
  }

  // 设备检测完成后，根据设备类型设置 PDO 通道
  setupChannels();

  // 设置同步管理器
  setupSyncManagers();

  // 设置域映射
  setupDomainMapping();

  // 设置ROS接口映射
  setupInterfaceMapping();

  // 配置 SDO（上电后写入期望配置：unit_mode=1 物理量模式）
  // XXX 重新上电生效
  sdo_config.clear();
  {
      using stark_ethercat_interface::SdoConfigEntry;
      SdoConfigEntry unit_mode;
      unit_mode.index = 0x8000;           // config_param
      unit_mode.sub_index = 0x05;         // unit_mode
      unit_mode.data_type = "uint8";
      unit_mode.data = 1;                 // 1 = 物理量
      sdo_config.push_back(unit_mode);

      SdoConfigEntry led_switch;
      led_switch.index = 0x8000;           // config_param
      led_switch.sub_index = 0x02;         // led_switch
      led_switch.data_type = "uint8";
      led_switch.data = 1;                 // 1 = 开
      sdo_config.push_back(led_switch);
  }

  REVO2_LOG_INFO("Revo2 Joints System Slave setup completed");
  return true;
}

void Revo2JointsSystemSlave::setupChannels()
{
  REVO2_LOG_DEBUG("Setting up PDO channels for Revo2 joints system");

  // 清空现有通道
  channels_.clear();
  rxpdo_.clear();
  txpdo_.clear();

  // RxPDO (Master -> Slave): 必须匹配从站固定PDO映射顺序
  // 注意：从站不支持PDO重配置，必须按照固定顺序配置

  // RxPDO条目0-2: 多指控制相关
  channels_.push_back({0x7000, 0x01, 16});  // mult_finger_ctrl_mode (uint16)
  channels_.push_back({0x7000, 0x02, 96});  // finger_param1[6] (6×int16) - 位置
  channels_.push_back({0x7000, 0x03, 96});  // finger_param2[6] (6×uint16) - 时间

  // RxPDO条目3-6: 单指控制相关
  channels_.push_back({0x7010, 0x01, 8});   // single_finger_ctrl_mode (uint8)
  channels_.push_back({0x7010, 0x02, 8});   // single_finger_id (uint8)
  channels_.push_back({0x7010, 0x03, 16});  // single_finger_param1 (int16) - 位置
  channels_.push_back({0x7010, 0x04, 16});  // single_finger_param2 (uint16) - 时间

  // TxPDO条目7-10: 所有手指状态 (96位数组)
  channels_.push_back({0x6000, 0x01, 96});  // finger_pos[6] (6×uint16)
  channels_.push_back({0x6000, 0x02, 96});  // finger_spd[6] (6×int16)
  channels_.push_back({0x6000, 0x03, 96});  // finger_cur[6] (6×int16)
  channels_.push_back({0x6000, 0x04, 96});  // finger_status[6] (6×uint16)

  // Touch设备额外的触觉数据 TxPDO条目11-15
  if (is_touch_device_)
  {
    channels_.push_back({0x6010, 0x01, 80});   // force_normal[5] (5×uint16)
    channels_.push_back({0x6010, 0x02, 80});   // force_tangential[5] (5×uint16)
    channels_.push_back({0x6010, 0x03, 80});   // force_direction[5] (5×uint16)
    channels_.push_back({0x6010, 0x04, 160});  // force_proximity[5] (5×uint32)
    channels_.push_back({0x6010, 0x05, 80});   // force_status[5] (5×uint16)
    REVO2_LOG_INFO("Added %zu touch channels for Touch device", 5);
  }
  else
  {
    REVO2_LOG_INFO("Standard device detected, no touch channels added");
  }

  REVO2_LOG_INFO(
    "Configured %zu channels for Revo2 joints system (%s)", channels_.size(),
    is_touch_device_ ? "Touch" : "Standard");
}

void Revo2JointsSystemSlave::setupSyncManagers()
{
  REVO2_LOG_DEBUG("Setting up sync managers for Revo2 joints system");

  syncs_.clear();

  // 构建 RxPDO (输出到从站) - 必须匹配从站的7个固定条目
  rxpdo_.clear();
  rxpdo_.push_back({0x1600, 7, &channels_[0]});  // 7个条目：多指控制(3) + 单指控制(4)

  // 构建 TxPDO (从站输入) - 标准设备4个条目，Touch设备9个条目
  txpdo_.clear();
  uint8_t txpdo_entries = is_touch_device_ ? 9 : 4;
  txpdo_.push_back({0x1A00, txpdo_entries, &channels_[7]});  // 状态数组通道：标准(4) + 触觉(5)

  // 同步管理器配置
  syncs_.push_back({0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE});
  syncs_.push_back({1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE});
  syncs_.push_back(
    {2, EC_DIR_OUTPUT, static_cast<uint8_t>(rxpdo_.size()), rxpdo_.data(), EC_WD_ENABLE});
  syncs_.push_back(
    {3, EC_DIR_INPUT, static_cast<uint8_t>(txpdo_.size()), txpdo_.data(), EC_WD_DISABLE});
  syncs_.push_back({0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE});  // 结束标记

  REVO2_LOG_INFO(
    "Configured sync managers for Revo2 joints system (%s, %d TxPDO entries)",
    is_touch_device_ ? "Touch" : "Standard", txpdo_entries);
}

void Revo2JointsSystemSlave::setupDomainMapping()
{
  REVO2_LOG_DEBUG("Setting up domain mapping for Revo2 joints system");

  // 域映射：所有通道都映射到域0
  std::vector<unsigned int> domain_channels;
  for (size_t i = 0; i < channels_.size(); ++i)
  {
    domain_channels.push_back(i);
  }
  domain_map_[0] = domain_channels;

  REVO2_LOG_DEBUG("Domain mapping configured for Revo2 joints system");
}

void Revo2JointsSystemSlave::setupInterfaceMapping()
{
  REVO2_LOG_DEBUG("Setting up interface mapping for Revo2 joints system");

  // 输出实际的接口大小信息
  size_t cmd_size = command_interface_ptr_ ? command_interface_ptr_->size() : 0;
  size_t state_size = state_interface_ptr_ ? state_interface_ptr_->size() : 0;
  REVO2_LOG_INFO("Interface sizes: command=%zu, state=%zu", cmd_size, state_size);

  // command interfaces [0-5] (6个关节的position), state interfaces [0-17] (6个关节×3个状态)

  for (size_t i = 0; i < NUM_FINGER_MOTORS; ++i)
  {
    // Commands are flattened per joint: [j0_pos, j1_pos, ...]
    joint_interfaces_[i].position_cmd = static_cast<int>(i);

    // States are flattened per joint in the order: [pos, vel, eff] for each joint
    const int base = static_cast<int>(i * 3);
    joint_interfaces_[i].position_state = base + 0;
    joint_interfaces_[i].velocity_state = base + 1;
    joint_interfaces_[i].effort_state = base + 2;

    REVO2_LOG_DEBUG(
      "Joint %s interfaces: cmd_pos=%d, state_pos=%d, state_vel=%d, state_eff=%d",
      joint_names_[i].c_str(), joint_interfaces_[i].position_cmd,
      joint_interfaces_[i].position_state, joint_interfaces_[i].velocity_state,
      joint_interfaces_[i].effort_state);
  }

  REVO2_LOG_INFO("Interface mapping configured for Revo2 joints system");
}

void Revo2JointsSystemSlave::processData(size_t index, uint8_t * domain_address)
{
  switch (index)
  {
    // RxPDO 通道 0: mult_finger_ctrl_mode (16位)
    case 0:
      if (rxpdo_control_mode_ == MULTI_FINGER_MODE)
      {
        // 多指控制：使用"位置+时间(CTRL_MODE_POS_TIME)"多指控制
        write_u16(domain_address, MULTI_MODE_POSITION_TIME);
      }
      else
      {
        // 单指控制：多指模式写0
        write_u16(domain_address, 0);
      }
      break;

    // RxPDO 通道 1: finger_param1 (96位) - 6×int16 目标位置/速度/电流等的param1
    case 1:
      if (rxpdo_control_mode_ == MULTI_FINGER_MODE)
      {
        // 多指控制：从 command_interface 读取6个关节的目标，并写入按从站定义的编码
        for (size_t i = 0; i < NUM_FINGER_MOTORS; ++i)
        {
          int cmd_idx = joint_interfaces_[i].position_cmd;
          int16_t raw_out = 0;
          double position_cmd_rad = 0.0;
          if (
            cmd_idx >= 0 && command_interface_ptr_ &&
            static_cast<size_t>(cmd_idx) < command_interface_ptr_->size())
          {
            position_cmd_rad = command_interface_ptr_->at(cmd_idx);
            // NaN/非有限命令回退：使用当前状态，避免初始0目标导致跳变
            if (!std::isfinite(position_cmd_rad))
            {
              int state_idx = joint_interfaces_[i].position_state;
              if (
                state_idx >= 0 && state_interface_ptr_ &&
                static_cast<size_t>(state_idx) < state_interface_ptr_->size())
              {
                position_cmd_rad = state_interface_ptr_->at(state_idx);
              }
              else
              {
                position_cmd_rad = current_position_states_[i];
              }
            }
            if (is_physical_mode_)
            {
              double deg = radToDeg(position_cmd_rad);
              raw_out = static_cast<int16_t>(std::lround(deg));
            }
            else
            {
              raw_out = radianToRevo2(position_cmd_rad);
            }
          }
          write_s16(domain_address + i * 2, raw_out);

          // 变化>0.1rad 时打印（节流）
          if (!std::isfinite(last_logged_cmd_pos_[i]))
          {
            last_logged_cmd_pos_[i] = position_cmd_rad;
          }
          else if (std::fabs(position_cmd_rad - last_logged_cmd_pos_[i]) > 0.1)
          {
            REVO2_LOG_DEBUG(
              "Cmd finger=%zu pos(rad)=%.4f -> raw_out=%d", i, position_cmd_rad, (raw_out));
            last_logged_cmd_pos_[i] = position_cmd_rad;
          }
        }
      }
      else
      {
        // 单指控制：多指参数写0
        memset(domain_address, 0, 12);
      }
      break;

    // RxPDO 通道 2: finger_param2 (96位) - 6×uint16 对于位置+时间，写入每指运动时间
    case 2:
      if (rxpdo_control_mode_ == MULTI_FINGER_MODE)
      {
        // 多指控制：写入每指运动时间
        for (size_t i = 0; i < NUM_FINGER_MOTORS; ++i)
        {
          write_u16(domain_address + i * 2, ctrl_param_duration_ms_);
        }
      }
      else
      {
        // 单指控制：多指参数写0
        memset(domain_address, 0, 12);
      }
      break;

    // RxPDO 通道 3: single_finger_ctrl_mode (8位)
    case 3:
      if (rxpdo_control_mode_ == SINGLE_FINGER_MODE)
      {
        // 单指控制：在写入模式之前，先更新当前手指电机ID
        updateActiveFingerFromCommands();
        write_u8(domain_address, SINGLE_MODE_POSITION_TIME);
      }
      else
      {
        // 多指控制：单指模式写0
        write_u8(domain_address, 0);
      }
      break;

    // RxPDO 通道 4: single_finger_id (8位)
    case 4:
      if (rxpdo_control_mode_ == SINGLE_FINGER_MODE)
      {
        write_u8(domain_address, active_finger_id_);
      }
      else
      {
        // 多指控制：单指ID写0
        write_u8(domain_address, 0);
      }
      break;

    // RxPDO 通道 5: single_finger_param1 (16位) - 位置
    case 5:
      if (rxpdo_control_mode_ == SINGLE_FINGER_MODE)
      {
        if (active_finger_id_ < NUM_FINGER_MOTORS)
        {
          int cmd_idx = joint_interfaces_[active_finger_id_].position_cmd;
          if (cmd_idx >= 0 && command_interface_ptr_)
          {
            double position_cmd_rad = command_interface_ptr_->at(cmd_idx);
            // NaN/非有限命令回退：使用当前状态
            if (!std::isfinite(position_cmd_rad))
            {
              int state_idx = joint_interfaces_[active_finger_id_].position_state;
              if (
                state_idx >= 0 && state_interface_ptr_ &&
                static_cast<size_t>(state_idx) < state_interface_ptr_->size())
              {
                position_cmd_rad = state_interface_ptr_->at(state_idx);
              }
              else
              {
                position_cmd_rad = current_position_states_[active_finger_id_];
              }
            }
            int16_t raw_out = 0;
            if (is_physical_mode_)
            {
              // 物理量模式：度
              double deg = radToDeg(position_cmd_rad);
              raw_out = static_cast<int16_t>(std::lround(deg));
            }
            else
            {
              // 千分比模式
              raw_out = radianToRevo2(position_cmd_rad);
            }
            write_s16(domain_address, raw_out);

            // 变化>0.1时打印
            if (!std::isfinite(last_logged_cmd_pos_[active_finger_id_]))
            {
              // 首次仅初始化
              last_logged_cmd_pos_[active_finger_id_] = position_cmd_rad;
            }
            else if (std::fabs(position_cmd_rad - last_logged_cmd_pos_[active_finger_id_]) > 0.1)
            {
              REVO2_LOG_DEBUG(
                "Single-Cmd finger=%u pos(rad)=%.4f -> raw_out=%d", active_finger_id_,
                position_cmd_rad, (raw_out));
              last_logged_cmd_pos_[active_finger_id_] = position_cmd_rad;
            }
          }
          else
          {
            write_s16(domain_address, 0);  // 默认
          }
        }
      }
      else
      {
        // 多指控制：单指参数写0
        write_s16(domain_address, 0);
      }
      break;

    // RxPDO 通道 6: single_finger_param2 (16位) - 时间
    case 6:
      if (rxpdo_control_mode_ == SINGLE_FINGER_MODE)
      {
        write_u16(domain_address, ctrl_param_duration_ms_);
      }
      else
      {
        // 多指控制：单指参数写0
        write_u16(domain_address, 0);
      }
      break;

    // TxPDO 通道 7: finger_pos[6] (6×uint16)
    case 7:
      for (size_t i = 0; i < NUM_FINGER_MOTORS; ++i)
      {
        uint16_t raw = read_u16(domain_address + i * 2);
        double radian_pos = 0.0;
        if (is_physical_mode_)
        {
          // 物理量模式：度
          radian_pos = degToRad(static_cast<double>(raw));
        }
        else
        {
          // 千分比模式
          radian_pos = revo2ToRadian(static_cast<int16_t>(raw));
        }
        current_position_states_[i] = radian_pos;
        int pos_idx = joint_interfaces_[i].position_state;
        if (pos_idx >= 0 && state_interface_ptr_)
        {
          state_interface_ptr_->at(pos_idx) = radian_pos;
        }

#if ENABLE_DEBUG_LOG_TxPDO
        if (
          !std::isfinite(last_logged_positions_[i]) ||
          std::fabs(radian_pos - last_logged_positions_[i]) > 1e-2)
        {
          REVO2_LOG_DEBUG("State finger=%zu pos_raw=%u pos(rad)=%.4f", i, raw, radian_pos);
          last_logged_positions_[i] = radian_pos;
        }
#endif
      }
      break;

    // TxPDO 通道 8: finger_spd[6] (6×int16)
    case 8:
      for (size_t i = 0; i < NUM_FINGER_MOTORS; ++i)
      {
        int16_t raw = read_s16(domain_address + i * 2);
        double radian_spd = 0.0;
        if (is_physical_mode_)
        {
          // 物理量模式：度
          radian_spd = degToRad(static_cast<double>(raw));
        }
        else
        {
          // 千分比模式
          radian_spd = revo2ToRadian(static_cast<int16_t>(raw));
        }
        current_velocity_states_[i] = radian_spd;

        int vel_idx = joint_interfaces_[i].velocity_state;
        if (vel_idx >= 0 && state_interface_ptr_)
        {
          state_interface_ptr_->at(vel_idx) = radian_spd;
        }

#if ENABLE_DEBUG_LOG_TxPDO
        if (
          !std::isfinite(last_logged_velocities_[i]) ||
          std::fabs(radian_spd - last_logged_velocities_[i]) > 1e-2)
        {
          REVO2_LOG_DEBUG("State finger=%zu spd_raw=%d spd(rad/s)=%.4f", i, raw, radian_spd);
          last_logged_velocities_[i] = radian_spd;
        }
#endif
      }
      break;

    // TxPDO 通道 9: finger_cur[6] (6×int16)
    case 9:
      for (size_t i = 0; i < NUM_FINGER_MOTORS; ++i)
      {
        int16_t raw = read_s16(domain_address + i * 2);
        double effort = 0.0;
        // 4指与拇指尖电机 额定力矩 60mN.m 额定电流 700mA
        if (is_physical_mode_)
        {
          // 物理量模式：mA
          effort = static_cast<double>(raw) * 60 / 700 * 0.001;  // 电流mA转换为力矩Nm
        }
        else
        {
          // 千分比模式
          effort = static_cast<double>(raw) * 700 / 1000;  // 电流mA 当做700mA满量程1000计算
          effort = effort * 60 / 700 * 0.001;              // XXX 电流mA转换为力矩Nm
        }
        current_effort_states_[i] = effort;

        int eff_idx = joint_interfaces_[i].effort_state;
        if (eff_idx >= 0 && state_interface_ptr_)
        {
          state_interface_ptr_->at(eff_idx) = effort;
        }

#if ENABLE_DEBUG_LOG_TxPDO
        if (
          !std::isfinite(last_logged_efforts_[i]) ||
          std::fabs(effort - last_logged_efforts_[i]) > 1e-2)
        {
          REVO2_LOG_DEBUG("State finger=%zu cur_raw=%d effort=%.4f", i, raw, effort);
          last_logged_efforts_[i] = effort;
        }
#endif
      }
      break;

    // TxPDO 通道 10: finger_status[6] (6×uint16)
    case 10:
      // 读取6个马达的状态数据
      for (size_t i = 0; i < NUM_FINGER_MOTORS; ++i)
      {
        finger_motor_status_[i] = read_u16(domain_address + i * 2);
      }

      // // 每秒打印一次状态数据
      // {
      //   static auto clock = rclcpp::Clock();
      //   RCLCPP_INFO_THROTTLE(
      //     rclcpp::get_logger("revo2_motor"), clock, 1000,
      //     "Motor Status - Thumb_Flex: %s | Thumb_Abduct: %s | Index: %s | "
      //     "Middle: %s | Ring: %s | Pinky: %s",
      //     getMotorStatusString(finger_motor_status_[0]),    // 拇指尖部
      //     getMotorStatusString(finger_motor_status_[1]),    // 拇指根部
      //     getMotorStatusString(finger_motor_status_[2]),    // 食指
      //     getMotorStatusString(finger_motor_status_[3]),    // 中指
      //     getMotorStatusString(finger_motor_status_[4]),    // 无名指
      //     getMotorStatusString(finger_motor_status_[5]));   // 小指
      // }
      break;

    // Touch版本专用的触觉数据通道 (11-15)
    case 11:  // force_normal[5] (5×uint16)
      if (is_touch_device_)
      {
        for (size_t i = 0; i < NUM_TOUCH_FINGERS; ++i)
        {
          touch_normal_force_[i] = read_u16(domain_address + i * 2);
        }
      }
      break;

    case 12:  // force_tangential[5] (5×uint16)
      if (is_touch_device_)
      {
        for (size_t i = 0; i < NUM_TOUCH_FINGERS; ++i)
        {
          touch_tangential_force_[i] = read_u16(domain_address + i * 2);
        }
      }
      break;

    case 13:  // force_direction[5] (5×uint16)
      if (is_touch_device_)
      {
        for (size_t i = 0; i < NUM_TOUCH_FINGERS; ++i)
        {
          touch_direction_[i] = read_u16(domain_address + i * 2);
        }
      }
      break;

    case 14:  // force_proximity[5] (5×uint32)
      if (is_touch_device_)
      {
        for (size_t i = 0; i < NUM_TOUCH_FINGERS; ++i)
        {
          // 读取32位数据并进行大小端转换
          // 读到两个16位寄存器: [high_word, low_word]
          uint16_t high_word = read_u16(domain_address + i * 4);
          uint16_t low_word = read_u16(domain_address + i * 4 + 2);
          // 大端转小端: 0x4433,0x2211 -> 0x11223344
          touch_proximity_[i] = (static_cast<uint32_t>(low_word) << 16) | high_word;
        }
      }
      break;

    case 15:  // force_status[5] (5×uint16)
      if (is_touch_device_)
      {
        for (size_t i = 0; i < NUM_TOUCH_FINGERS; ++i)
        {
          touch_status_[i] = read_u16(domain_address + i * 2);
        }

        // 法向力，切向力是 16 位的无符号数据。数值单位是 100 * N， 例如切向力 1000 表示 1000 / 100
        // N, 即 10 N。法向力，切向力的测量范围是 0 ~ 25 N。 切向力方向是 16
        // 位的无符号数据。单位是角度，数值范围为 0 ~ 359 度。靠近指尖的方向为 0
        // 度，按顺时针旋转最大到 359 度，当数值为 65535 (0xFFFF) 时，表示切向力方向无效。 接近值是
        // 32 位的无符号数据。 触觉状态寄存器为 16 位，分高低字节表示不同状态信息： 低 8
        // 位表示整体数据状态：0 表示数据正常，1 表示数据异常，2 表示与触觉传感器通信异常。 高 8
        // 位为数据异常志位： Bit0 表示原始值错误， Bit1 表示原始值长时间未更新， Bit2
        // 表示触发超时。

        // 每 秒打印一次触觉数据
        static auto clock = rclcpp::Clock();
        RCLCPP_INFO_THROTTLE(
          rclcpp::get_logger("revo2_touch"), clock, 30 * 1000,
          "Touch Data - Finger0: N=%.2fN T=%.2fN D=%d° P=%u S=0x%02X %02X | "
          "Finger1: N=%.2fN T=%.2fN D=%d° P=%u S=0x%02X %02X | "
          "Finger2: N=%.2fN T=%.2fN D=%d° P=%u S=0x%02X %02X | "
          "Finger3: N=%.2fN T=%.2fN D=%d° P=%u S=0x%02X %02X | "
          "Finger4: N=%.2fN T=%.2fN D=%d° P=%u S=0x%02X %02X",
          // Finger 0 (拇指)
          touch_normal_force_[0] * 0.01, touch_tangential_force_[0] * 0.01,
          touch_direction_[0] == 0xFFFF ? -1 : static_cast<int>(touch_direction_[0]),
          touch_proximity_[0], touch_status_[0] >> 8, touch_status_[0] & 0xFF,
          // Finger 1 (食指)
          touch_normal_force_[1] * 0.01, touch_tangential_force_[1] * 0.01,
          touch_direction_[1] == 0xFFFF ? -1 : static_cast<int>(touch_direction_[1]),
          touch_proximity_[1], touch_status_[1] >> 8, touch_status_[1] & 0xFF,
          // Finger 2 (中指)
          touch_normal_force_[2] * 0.01, touch_tangential_force_[2] * 0.01,
          touch_direction_[2] == 0xFFFF ? -1 : static_cast<int>(touch_direction_[2]),
          touch_proximity_[2], touch_status_[2] >> 8, touch_status_[2] & 0xFF,
          // Finger 3 (无名指)
          touch_normal_force_[3] * 0.01, touch_tangential_force_[3] * 0.01,
          touch_direction_[3] == 0xFFFF ? -1 : static_cast<int>(touch_direction_[3]),
          touch_proximity_[3], touch_status_[3] >> 8, touch_status_[3] & 0xFF,
          // Finger 4 (小指)
          touch_normal_force_[4] * 0.01, touch_tangential_force_[4] * 0.01,
          touch_direction_[4] == 0xFFFF ? -1 : static_cast<int>(touch_direction_[4]),
          touch_proximity_[4], touch_status_[4] >> 8, touch_status_[4] & 0xFF);
      }
      break;

    default:
      if (index >= 11 && index <= 15 && !is_touch_device_)
      {
        REVO2_LOG_WARN("Received touch data on non-touch device, channel index: %zu", index);
      }
      else
      {
        REVO2_LOG_WARN("Unknown PDO channel index: %zu", index);
      }
      break;
  }
}

// 检查哪个手指的命令发生了变化
void Revo2JointsSystemSlave::updateActiveFingerFromCommands()
{
  static size_t last_updated_finger = 0;
  static bool debug_logged = false;

  if (!debug_logged)
  {
    size_t cmd_size = command_interface_ptr_ ? command_interface_ptr_->size() : 0;
    REVO2_LOG_INFO("Command interface size: %zu", cmd_size);
    debug_logged = true;
  }

  // 轮询检查每个手指的命令是否有变化
  for (size_t i = 0; i < NUM_FINGER_MOTORS; ++i)
  {
    size_t finger_idx = (last_updated_finger + i) % NUM_FINGER_MOTORS;
    int cmd_idx = joint_interfaces_[finger_idx].position_cmd;

    if (
      cmd_idx >= 0 && command_interface_ptr_ &&
      static_cast<size_t>(cmd_idx) < command_interface_ptr_->size())
    {
      double position_cmd = command_interface_ptr_->at(cmd_idx);

      // XXX cmd分辨率使用0.001
      if (std::abs(position_cmd - last_position_cmds_[finger_idx]) > 1e-3)
      {
        // REVO2_LOG_DEBUG("Switching to finger %u, new position command from %.4f to %.4f",
        //     finger_idx, last_position_cmds_[finger_idx], position_cmd);
        active_finger_id_ = static_cast<uint8_t>(finger_idx);
        last_position_cmds_[finger_idx] = position_cmd;
        last_updated_finger = finger_idx;

        break;
      }
    }
  }
}

double Revo2JointsSystemSlave::revo2ToRadian(int16_t revo2_pos)
{
  // XXX 千分比与弧度线性映射
  return 0.001 + (static_cast<double>(revo2_pos - 1) * (1.57 - 0.001) / (1000 - 1));
}

int16_t Revo2JointsSystemSlave::radianToRevo2(double radian_pos)
{
  // XXX 弧度与千分比线性映射
  double scaled = 1 + ((radian_pos - 0.001) * (1000 - 1) / (1.57 - 0.001));
  return static_cast<int16_t>(std::round(std::clamp(scaled, 1.0, 1000.0)));
}

const char * Revo2JointsSystemSlave::getMotorStatusString(uint16_t status)
{
  switch (status)
  {
    case 0:
      return "IDLE";  // 马达空闲
    case 1:
      return "RUNNING";  // 马达运行
    case 2:
      return "STALLED";  // 马达堵转
    case 3:
      return "TURBO";  // Turbo模式
    default:
      return "UNKNOWN";  // 未知状态
  }
}

const ec_sync_info_t * Revo2JointsSystemSlave::syncs() { return syncs_.data(); }

size_t Revo2JointsSystemSlave::syncSize() { return syncs_.size(); }

const ec_pdo_entry_info_t * Revo2JointsSystemSlave::channels() { return channels_.data(); }

void Revo2JointsSystemSlave::domains(DomainMap & domains) const { domains = domain_map_; }

int Revo2JointsSystemSlave::assign_activate_dc_sync() { return static_cast<int>(assign_activate_); }

}  // namespace revo2_ethercat_plugins

// 导出插件
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  revo2_ethercat_plugins::Revo2JointsSystemSlave, stark_ethercat_interface::EcSlave)
