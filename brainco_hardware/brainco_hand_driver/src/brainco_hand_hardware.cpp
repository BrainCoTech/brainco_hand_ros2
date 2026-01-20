#include "brainco_hand_driver/brainco_hand_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "brainco_hand_driver/logger_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace brainco_hand_driver
{

#define ENABLE_DEBUG_LOG_READ 0
#define ENABLE_DEBUG_LOG_WRITE 0
namespace
{
constexpr std::size_t kFingerCount{6};
}

auto BraincoHandHardware::on_init(const hardware_interface::HardwareInfo & info)
  -> hardware_interface::CallbackReturn
{
  BRAINCO_HAND_LOG_INFO("on_init invoked");

  auto base_init = hardware_interface::SystemInterface::on_init(info);
  if (base_init != hardware_interface::CallbackReturn::SUCCESS)
  {
    BRAINCO_HAND_LOG_ERROR("SystemInterface::on_init failed");
    return base_init;
  }

  if (init_parameters(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (validate_joints() != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto joint_count = info_.joints.size();
  joint_names_.resize(joint_count);
  for (std::size_t i = 0; i < joint_count; ++i)
  {
    joint_names_[i] = info_.joints[i].name;
    BRAINCO_HAND_LOG_INFO("  Joint[%zu]=%s", i, joint_names_[i].c_str());
  }

  hw_positions_.assign(joint_count, 0.0);
  hw_velocities_.assign(joint_count, 0.0);
  hw_commands_.assign(joint_count, 0.0);
  hw_motor_states_.assign(joint_count, 0);

  resolved_connection_.reset();
  const char * protocol_label = config_.transport.protocol == Protocol::kCanfd ? "CANFD" : "MODBUS";
  BRAINCO_HAND_LOG_INFO(
    "BraincoHandApi prepared, protocol=%s, log_level=%s", protocol_label,
    brainco_log_level_to_string(config_.transport.log_level).c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::on_configure(const rclcpp_lifecycle::State & previous_state)
  -> hardware_interface::CallbackReturn
{
  (void)previous_state;
  BRAINCO_HAND_LOG_INFO("on_configure invoked");

  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_motor_states_.begin(), hw_motor_states_.end(), 0);

  if (!open_connection())
  {
    BRAINCO_HAND_LOG_ERROR(
      "Failed to open transport session during configuration. "
      "The hardware will not be activated. Please check the connection and try again.");
    // Return ERROR to prevent activation, but don't crash
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::on_cleanup(const rclcpp_lifecycle::State & previous_state)
  -> hardware_interface::CallbackReturn
{
  (void)previous_state;
  BRAINCO_HAND_LOG_INFO("on_cleanup invoked");
  close_connection();
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
  -> hardware_interface::CallbackReturn
{
  (void)previous_state;
  BRAINCO_HAND_LOG_INFO("on_activate invoked");

  if (!api_.is_open())
  {
    BRAINCO_HAND_LOG_ERROR("Cannot activate without valid transport session");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (config_.transport.ensure_physical_mode)
  {
    ensure_physical_unit_mode();
  }

  for (std::size_t i = 0; i < hw_positions_.size(); ++i)
  {
    hw_commands_[i] = hw_positions_[i];
  }

  is_active_ = true;
  BRAINCO_HAND_LOG_INFO("Hardware activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
  -> hardware_interface::CallbackReturn
{
  (void)previous_state;
  BRAINCO_HAND_LOG_INFO("on_deactivate invoked");
  is_active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::export_state_interfaces()
  -> std::vector<hardware_interface::StateInterface>
{
  BRAINCO_HAND_LOG_INFO("export_state_interfaces invoked");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints[i];
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

auto BraincoHandHardware::export_command_interfaces()
  -> std::vector<hardware_interface::CommandInterface>
{
  BRAINCO_HAND_LOG_INFO("export_command_interfaces invoked");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints[i];
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

auto BraincoHandHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> hardware_interface::return_type
{
  (void)time;
  (void)period;

  if (!api_.is_open())
  {
    BRAINCO_HAND_LOG_WARN("Skip read: API not connected");
    return hardware_interface::return_type::ERROR;
  }

  auto status = api_.get_motor_status(config_.transport.slave_id);
  if (!status)
  {
    BRAINCO_HAND_LOG_WARN("BraincoHandApi::get_motor_status returned no data");
    return hardware_interface::return_type::ERROR;
  }

  const auto & motor_status = *status;

  const auto joint_count = std::min<std::size_t>(hw_positions_.size(), kFingerCount);
  if (joint_count == 0)
  {
    BRAINCO_HAND_LOG_WARN("Skip read: no joints configured");
    return hardware_interface::return_type::OK;
  }
  for (std::size_t i = 0; i < joint_count; ++i)
  {
    const auto raw_position = static_cast<double>(motor_status.positions[i]);
    const auto raw_velocity = static_cast<double>(motor_status.speeds[i]);
    hw_positions_[i] = raw_position * config_.position_state_scale;
    hw_velocities_[i] = raw_velocity * config_.velocity_state_scale;
    hw_motor_states_[i] = motor_status.states[i];
  }

#if ENABLE_DEBUG_LOG_READ
  std::ostringstream read_stream;
  read_stream.setf(std::ios::fixed);
  read_stream.precision(2);
  read_stream << "read positions:";
  read_stream << "(device)raw:";
  for (std::size_t i = 0; i < joint_count; ++i)
  {
    read_stream << " " << motor_status.positions[i];
  }
  read_stream << " | (hw)radians:";
  for (std::size_t i = 0; i < joint_count; ++i)
  {
    read_stream << " " << hw_positions_[i];
  }
  BRAINCO_HAND_LOG_DEBUG("%s", read_stream.str().c_str());
#endif

  return hardware_interface::return_type::OK;
}

auto BraincoHandHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> hardware_interface::return_type
{
  (void)time;
  (void)period;

  if (!api_.is_open())
  {
    BRAINCO_HAND_LOG_WARN("Skip write: API not connected");
    return hardware_interface::return_type::ERROR;
  }

  if (!is_active_)
  {
    BRAINCO_HAND_LOG_WARN("Skip write: hardware not active");
    return hardware_interface::return_type::OK;
  }

  std::array<uint16_t, kFingerCount> goal_positions{};
  std::array<uint16_t, kFingerCount> goal_durations{};

  const auto joint_count = std::min<std::size_t>(hw_commands_.size(), kFingerCount);
  if (joint_count == 0)
  {
    BRAINCO_HAND_LOG_WARN("Skip write: no joints configured");
    return hardware_interface::return_type::OK;
  }
  for (std::size_t i = 0; i < joint_count; ++i)
  {
    const double desired_device = hw_commands_[i] * config_.position_command_scale;
    const double clamped =
      std::clamp(desired_device, config_.position_device_min, config_.position_device_max);
    goal_positions[i] = static_cast<uint16_t>(std::lround(clamped));
    goal_durations[i] = config_.ctrl_param_duration_ms;
  }

#if ENABLE_DEBUG_LOG_WRITE
  std::ostringstream write_stream;
  write_stream.setf(std::ios::fixed);
  write_stream.precision(2);
  write_stream << "write commands:";
  write_stream << "(hw)radians:";
  for (std::size_t i = 0; i < joint_count; ++i)
  {
    write_stream << " " << hw_commands_[i];
  }
  write_stream << " (ms=" << config_.ctrl_param_duration_ms << ")";
  write_stream << " | (device)raw:";
  for (std::size_t i = 0; i < joint_count; ++i)
  {
    write_stream << " " << goal_positions[i];
  }
  BRAINCO_HAND_LOG_DEBUG("%s", write_stream.str().c_str());
#endif

  if (!api_.set_finger_positions_and_durations(
        config_.transport.slave_id, goal_positions.data(), goal_durations.data(), joint_count))
  {
    BRAINCO_HAND_LOG_WARN(
      "Failed to send finger positions for slave %u", config_.transport.slave_id);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

auto BraincoHandHardware::init_parameters(const hardware_interface::HardwareInfo & info)
  -> hardware_interface::CallbackReturn
{
  (void)info;
  BRAINCO_HAND_LOG_INFO("Parsing hardware parameters");

  try
  {
    const auto protocol_value = get_parameter("protocol", "modbus");
    std::string protocol_lower = protocol_value;
    std::transform(
      protocol_lower.begin(), protocol_lower.end(), protocol_lower.begin(),
      [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

#if ENABLE_CANFD
    if (protocol_lower == "canfd" || protocol_lower == "can" || protocol_lower == "can-fd")
    {
      config_.transport.protocol = Protocol::kCanfd;
    }
    else
    {
      config_.transport.protocol = Protocol::kModbus;
    }
#else
    // CAN FD support is disabled at compile time
    if (protocol_lower == "canfd" || protocol_lower == "can" || protocol_lower == "can-fd")
    {
      BRAINCO_HAND_LOG_ERROR(
        "CAN FD protocol requested but CAN FD support is disabled at compile time. Please rebuild "
        "with ENABLE_CANFD=ON.");
      config_.transport.protocol = Protocol::kModbus;
    }
    else
    {
      config_.transport.protocol = Protocol::kModbus;
    }
#endif

    constexpr uint8_t kMaxSlaveId{247};
    const auto slave_id_value = std::stoul(get_parameter("slave_id", "126"));
    if (slave_id_value > static_cast<unsigned long>(kMaxSlaveId))
    {
      BRAINCO_HAND_LOG_WARN(
        "slave_id value %lu exceeds %u, will clamp to %u", slave_id_value,
        static_cast<unsigned>(kMaxSlaveId), static_cast<unsigned>(kMaxSlaveId));
      config_.transport.slave_id = kMaxSlaveId;
    }
    else
    {
      config_.transport.slave_id = static_cast<uint8_t>(slave_id_value);
    }

    config_.transport.log_level = parse_log_level(get_parameter("log_level", "info"));
    config_.transport.ensure_physical_mode =
      parse_bool(get_parameter("ensure_physical_mode", "true"), true);

    if (config_.transport.protocol == Protocol::kModbus)
    {
      config_.transport.modbus.port = get_parameter("port", "/dev/ttyUSB0");
      config_.transport.modbus.baudrate =
        static_cast<uint32_t>(std::stoul(get_parameter("baudrate", "460800")));
      config_.transport.modbus.auto_detect =
        parse_bool(get_parameter("auto_detect", "false"), false);
      config_.transport.modbus.auto_detect_quick =
        parse_bool(get_parameter("auto_detect_quick", "true"), true);
      config_.transport.modbus.auto_detect_port = get_parameter("auto_detect_port", "");
    }
#if ENABLE_CANFD
    else
    {
      config_.transport.canfd.device_type = static_cast<uint32_t>(std::stoul(
        get_parameter("can_device_type", std::to_string(config_.transport.canfd.device_type))));
      config_.transport.canfd.card_index = static_cast<uint32_t>(std::stoul(
        get_parameter("can_card_index", std::to_string(config_.transport.canfd.card_index))));
      config_.transport.canfd.channel_index = static_cast<uint32_t>(std::stoul(
        get_parameter("can_channel_index", std::to_string(config_.transport.canfd.channel_index))));
      config_.transport.canfd.clock_hz = static_cast<uint32_t>(std::stoul(
        get_parameter("can_clock_hz", std::to_string(config_.transport.canfd.clock_hz))));
      config_.transport.canfd.rx_wait_time = static_cast<uint32_t>(std::stoul(
        get_parameter("can_rx_wait_time", std::to_string(config_.transport.canfd.rx_wait_time))));
      config_.transport.canfd.rx_buffer_size = static_cast<uint32_t>(std::stoul(get_parameter(
        "can_rx_buffer_size", std::to_string(config_.transport.canfd.rx_buffer_size))));
      config_.transport.canfd.master_id = static_cast<uint8_t>(std::stoul(
        get_parameter("can_master_id", std::to_string(config_.transport.canfd.master_id))));

      config_.transport.canfd.arbitration.sjw = static_cast<uint8_t>(std::stoul(get_parameter(
        "can_arbitration_sjw", std::to_string(config_.transport.canfd.arbitration.sjw))));
      config_.transport.canfd.arbitration.brp = static_cast<uint16_t>(std::stoul(get_parameter(
        "can_arbitration_brp", std::to_string(config_.transport.canfd.arbitration.brp))));
      config_.transport.canfd.arbitration.tseg1 = static_cast<uint8_t>(std::stoul(get_parameter(
        "can_arbitration_tseg1", std::to_string(config_.transport.canfd.arbitration.tseg1))));
      config_.transport.canfd.arbitration.tseg2 = static_cast<uint8_t>(std::stoul(get_parameter(
        "can_arbitration_tseg2", std::to_string(config_.transport.canfd.arbitration.tseg2))));
      config_.transport.canfd.arbitration.smp = static_cast<uint8_t>(std::stoul(get_parameter(
        "can_arbitration_smp", std::to_string(config_.transport.canfd.arbitration.smp))));

      config_.transport.canfd.data.sjw = static_cast<uint8_t>(std::stoul(
        get_parameter("can_data_sjw", std::to_string(config_.transport.canfd.data.sjw))));
      config_.transport.canfd.data.brp = static_cast<uint16_t>(std::stoul(
        get_parameter("can_data_brp", std::to_string(config_.transport.canfd.data.brp))));
      config_.transport.canfd.data.tseg1 = static_cast<uint8_t>(std::stoul(
        get_parameter("can_data_tseg1", std::to_string(config_.transport.canfd.data.tseg1))));
      config_.transport.canfd.data.tseg2 = static_cast<uint8_t>(std::stoul(
        get_parameter("can_data_tseg2", std::to_string(config_.transport.canfd.data.tseg2))));
      config_.transport.canfd.data.smp = static_cast<uint8_t>(std::stoul(
        get_parameter("can_data_smp", std::to_string(config_.transport.canfd.data.smp))));
    }
#endif

    const auto duration_value = std::stoul(get_parameter("ctrl_param_duration_ms", "10"));
    config_.ctrl_param_duration_ms = static_cast<uint16_t>(
      std::min<unsigned long>(duration_value, std::numeric_limits<uint16_t>::max()));
    config_.position_command_scale = std::stod(get_parameter("position_command_scale", "1.0"));
    config_.position_state_scale = std::stod(get_parameter("position_state_scale", "1.0"));
    config_.velocity_state_scale = std::stod(get_parameter("velocity_state_scale", "1.0"));
    config_.position_device_min = std::stod(get_parameter("position_device_min", "0.0"));
    config_.position_device_max = std::stod(get_parameter("position_device_max", "1000.0"));
  }
  catch (const std::exception & e)
  {
    BRAINCO_HAND_LOG_ERROR("Parameter parsing failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (config_.ctrl_param_duration_ms == 0)
  {
    config_.ctrl_param_duration_ms = DriverConfig::kDefaultDurationMs;
  }

  if (config_.position_device_min < 0.0)
  {
    config_.position_device_min = 0.0;
  }

  if (config_.position_device_min > config_.position_device_max)
  {
    std::swap(config_.position_device_min, config_.position_device_max);
  }

  if (config_.position_command_scale == 0.0)
  {
    config_.position_command_scale = 1.0;
  }

  if (config_.position_state_scale == 0.0)
  {
    config_.position_state_scale = 1.0;
  }

  if (config_.velocity_state_scale == 0.0)
  {
    config_.velocity_state_scale = 1.0;
  }

  const char * protocol_label = config_.transport.protocol == Protocol::kCanfd ? "CANFD" : "MODBUS";

  BRAINCO_HAND_LOG_INFO(
    "Driver config -> protocol=%s slave_id=%u duration=%u ensure_physical_mode=%s", protocol_label,
    config_.transport.slave_id, config_.ctrl_param_duration_ms,
    config_.transport.ensure_physical_mode ? "true" : "false");

  if (config_.transport.protocol == Protocol::kModbus)
  {
    BRAINCO_HAND_LOG_INFO(
      "Modbus -> port=%s baudrate=%u auto_detect=%s quick=%s hint=%s",
      config_.transport.modbus.port.c_str(), config_.transport.modbus.baudrate,
      config_.transport.modbus.auto_detect ? "true" : "false",
      config_.transport.modbus.auto_detect_quick ? "true" : "false",
      config_.transport.modbus.auto_detect_port.empty()
        ? "<auto>"
        : config_.transport.modbus.auto_detect_port.c_str());
  }
#if ENABLE_CANFD
  else
  {
    BRAINCO_HAND_LOG_INFO(
      "CANFD -> device_type=%u card=%u channel=%u clock=%u rx_wait=%u rx_buf=%u master_id=%u",
      config_.transport.canfd.device_type, config_.transport.canfd.card_index,
      config_.transport.canfd.channel_index, config_.transport.canfd.clock_hz,
      config_.transport.canfd.rx_wait_time, config_.transport.canfd.rx_buffer_size,
      config_.transport.canfd.master_id);
  }
#endif

  BRAINCO_HAND_LOG_INFO(
    "position_command_scale=%f position_state_scale=%f velocity_state_scale=%f "
    "position_device_min=%f position_device_max=%f",
    config_.position_command_scale, config_.position_state_scale, config_.velocity_state_scale,
    config_.position_device_min, config_.position_device_max);
  BRAINCO_HAND_LOG_INFO(
    "log_level=%s", brainco_log_level_to_string(config_.transport.log_level).c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::validate_joints() const -> hardware_interface::CallbackReturn
{
  if (info_.joints.empty())
  {
    BRAINCO_HAND_LOG_ERROR("No joints configured for brainco_hand_driver");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints)
  {
    const bool has_position_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) { return interface.name == hardware_interface::HW_IF_POSITION; });
    const bool has_velocity_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) { return interface.name == hardware_interface::HW_IF_VELOCITY; });
    const bool has_position_command = std::any_of(
      joint.command_interfaces.begin(), joint.command_interfaces.end(),
      [](const auto & interface) { return interface.name == hardware_interface::HW_IF_POSITION; });

    if (!has_position_state || !has_velocity_state || !has_position_command)
    {
      BRAINCO_HAND_LOG_ERROR(
        "Joint %s missing required interfaces (position state, velocity state, position command)",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::close_connection() -> void
{
  if (api_.is_open())
  {
    const char * protocol_label =
      config_.transport.protocol == Protocol::kCanfd ? "CANFD" : "Modbus";
    if (resolved_connection_)
    {
      BRAINCO_HAND_LOG_INFO(
        "Closing %s connection on %s", protocol_label, resolved_connection_->port.c_str());
    }
    else
    {
      BRAINCO_HAND_LOG_INFO("Closing %s transport session", protocol_label);
    }
    api_.close();
  }
  resolved_connection_.reset();
}

auto BraincoHandHardware::open_connection() -> bool
{
  close_connection();

  api_.configure(config_.transport);

  const char * protocol_label = config_.transport.protocol == Protocol::kCanfd ? "CANFD" : "Modbus";

  if (config_.transport.protocol == Protocol::kModbus)
  {
    if (config_.transport.modbus.auto_detect)
    {
      BRAINCO_HAND_LOG_INFO(
        "Attempting to auto-detect %s device (requested slave_id: %u)...", protocol_label,
        config_.transport.slave_id);
    }
    else
    {
      BRAINCO_HAND_LOG_INFO(
        "Attempting to connect to %s device on port %s (slave_id: %u)...", protocol_label,
        config_.transport.modbus.port.c_str(), config_.transport.slave_id);
    }
  }

  if (!api_.open())
  {
    BRAINCO_HAND_LOG_ERROR(
      "Failed to open %s transport session. Connection cannot be established.", protocol_label);
    return false;
  }

  resolved_connection_ = api_.resolved_connection();
  if (resolved_connection_)
  {
    if (config_.transport.protocol == Protocol::kModbus)
    {
      config_.transport.slave_id = resolved_connection_->slave_id;
    }
    BRAINCO_HAND_LOG_INFO(
      "Resolved transport -> port=%s baudrate=%u slave_id=%u", resolved_connection_->port.c_str(),
      resolved_connection_->baudrate, resolved_connection_->slave_id);
  }

  BraincoHandApi::DeviceInfoData device_info{};
  if (api_.fetch_device_info(config_.transport.slave_id, device_info))
  {
    BRAINCO_HAND_LOG_INFO(
      "Device info -> SKU=%u hardware_type=%u serial=%s firmware=%s", device_info.sku_type,
      device_info.hardware_type,
      device_info.serial_number.empty() ? "<unknown>" : device_info.serial_number.c_str(),
      device_info.firmware_version.empty() ? "<unknown>" : device_info.firmware_version.c_str());
  }
  else
  {
    BRAINCO_HAND_LOG_WARN("Failed to fetch device info for slave %u", config_.transport.slave_id);
  }

  return true;
}

auto BraincoHandHardware::ensure_physical_unit_mode() -> bool
{
  if (!api_.is_open())
  {
    BRAINCO_HAND_LOG_WARN("ensure_physical_unit_mode skipped: API not connected");
    return false;
  }

  if (!api_.ensure_physical_unit_mode(config_.transport.slave_id))
  {
    BRAINCO_HAND_LOG_WARN(
      "Failed to ensure physical unit mode for slave %u",
      static_cast<unsigned>(config_.transport.slave_id));
    return false;
  }

  BRAINCO_HAND_LOG_INFO("Finger unit mode confirmed physical");
  return true;
}

auto BraincoHandHardware::parse_log_level(const std::string & level_str) -> BraincoLogLevel
{
  std::string lowercase = level_str;
  std::transform(
    lowercase.begin(), lowercase.end(), lowercase.begin(),
    [](unsigned char character) { return static_cast<char>(std::tolower(character)); });

  if (lowercase == "trace")
  {
    return BraincoLogLevel::kTrace;
  }
  if (lowercase == "debug")
  {
    return BraincoLogLevel::kDebug;
  }
  if (lowercase == "info")
  {
    return BraincoLogLevel::kInfo;
  }
  if (lowercase == "warn" || lowercase == "warning")
  {
    return BraincoLogLevel::kWarn;
  }
  if (lowercase == "error")
  {
    return BraincoLogLevel::kError;
  }

  BRAINCO_HAND_LOG_WARN("Unknown log level '%s', fallback to INFO", level_str.c_str());
  return BraincoLogLevel::kInfo;
}

auto BraincoHandHardware::parse_bool(const std::string & value, bool default_value) -> bool
{
  if (value.empty())
  {
    return default_value;
  }

  std::string lowercase = value;
  std::transform(
    lowercase.begin(), lowercase.end(), lowercase.begin(),
    [](unsigned char character) { return static_cast<char>(std::tolower(character)); });

  if (lowercase == "true" || lowercase == "1" || lowercase == "yes" || lowercase == "on")
  {
    return true;
  }

  if (lowercase == "false" || lowercase == "0" || lowercase == "no" || lowercase == "off")
  {
    return false;
  }

  return default_value;
}

auto BraincoHandHardware::get_parameter(
  const std::string & key, const std::string & default_value) const -> std::string
{
  const auto parameter_iterator = info_.hardware_parameters.find(key);
  if (parameter_iterator != info_.hardware_parameters.end() && !parameter_iterator->second.empty())
  {
    return parameter_iterator->second;
  }
  return default_value;
}

}  // namespace brainco_hand_driver

PLUGINLIB_EXPORT_CLASS(
  brainco_hand_driver::BraincoHandHardware, hardware_interface::SystemInterface)
