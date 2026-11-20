#include "brainco_hand_driver/brainco_hand_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "brainco_hand_driver/logger_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace revo2_driver
{

#define ENABLE_DEBUG_LOG_READ 0
#define ENABLE_DEBUG_LOG_WRITE 0
namespace
{
constexpr std::size_t kFingerCount{6};
constexpr std::size_t kTouchFingerCount{5};
constexpr double kTactileForceScale{100};  // centi-Newton to Newton
constexpr double kDegToRad{M_PI / 180.0};
constexpr const char * kTouchNf1Interface{"tactile_normal_force"};
constexpr const char * kTouchTf1Interface{"tactile_tangential_force"};
constexpr const char * kTouchTd1Interface{"tactile_tangential_direction"};
constexpr const char * kTouchSp1Interface{"tactile_self_proximity"};
constexpr const char * kTouchStatusInterface{"tactile_status"};
// Motor status from SDK / SDK 电机状态接口
constexpr const char * kCurrentInterface{"current"};
constexpr const char * kMotorStateInterface{"motor_state"};
// UNKNOWN motor_state sentinel. When read() fails for longer than the tolerance
// window we stamp this into hw_motor_states_ so joint_state_broadcaster stops
// republishing a healthy last frame; hardware_monitor's classifyRevo2 treats
// 255 as a hard fault ("comms/state unavailable"), so the self-check trips.
constexpr double kMotorStateUnknownSentinel{255.0};

auto to_lower_copy(std::string value) -> std::string
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

auto parse_finger_unit_mode_setting(const std::string & value) -> FingerUnitModeSetting
{
  const auto mode = to_lower_copy(value);
  if (mode.empty() || mode == "keep" || mode == "keep_current" || mode == "none" ||
    mode == "disabled")
  {
    return FingerUnitModeSetting::kKeepCurrent;
  }
  if (mode == "normalized" || mode == "normalised" || mode == "normalize")
  {
    return FingerUnitModeSetting::kNormalized;
  }
  if (mode == "physical")
  {
    return FingerUnitModeSetting::kPhysical;
  }
  throw std::invalid_argument("finger_unit_mode must be normalized, physical, or keep_current");
}

auto finger_unit_mode_setting_to_string(FingerUnitModeSetting mode) -> const char *
{
  switch (mode)
  {
    case FingerUnitModeSetting::kKeepCurrent:
      return "keep_current";
    case FingerUnitModeSetting::kNormalized:
      return "normalized";
    case FingerUnitModeSetting::kPhysical:
      return "physical";
  }
  return "unknown";
}

auto ends_with(const std::string & value, const std::string & suffix) -> bool
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Revo2 teleoperation targets follow the legacy SDK command order:
// [thumb_proximal/flex, thumb_metacarpal/base, index, middle, ring, pinky].
// Fallback physical position limits (degrees) used only when the device cannot
// report its own range. These mirror the Revo2 firmware defaults documented in
// stark_get_finger_max_position (max 60 87 84 84 84 84, min all 0). The live
// per-finger range is read from the SDK on activation and preferred over these.
constexpr std::array<double, kFingerCount> kDefaultFingerMinDeg{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
constexpr std::array<double, kFingerCount> kDefaultFingerMaxDeg{60.0, 87.0, 84.0, 84.0, 84.0, 84.0};

// Ignore obviously bogus SDK readings (0 or absurdly large) and fall back.
constexpr double kMaxPlausibleFingerDeg{360.0};

// Minimum seconds between per-finger out-of-range clamp warnings.
constexpr double kRangeWarnIntervalSeconds{2.0};

// Deadband around the physical limits before an out-of-range clamp is warned.
// Teleop targets that sit exactly at full range (e.g. 60 deg commanded as the
// rounded 1.0472 rad, ~2.4e-5 above 60*pi/180) are still clamped but must not
// spam a warning; only a substantive overshoot (~0.6 deg) is reported.
constexpr double kRangeWarnToleranceRad{0.01};

auto sdk_motor_index_for_joint_name(const std::string & joint_name, std::size_t fallback)
  -> std::size_t
{
  if (ends_with(joint_name, "_thumb_proximal_joint"))
  {
    return 0;
  }
  if (ends_with(joint_name, "_thumb_metacarpal_joint"))
  {
    return 1;
  }
  if (ends_with(joint_name, "_index_proximal_joint"))
  {
    return 2;
  }
  if (ends_with(joint_name, "_middle_proximal_joint"))
  {
    return 3;
  }
  if (ends_with(joint_name, "_ring_proximal_joint"))
  {
    return 4;
  }
  if (ends_with(joint_name, "_pinky_proximal_joint"))
  {
    return 5;
  }
  return fallback;
}

auto parse_optional_uint16(
  const std::string & value, unsigned long min_value, unsigned long max_value)
  -> uint16_t
{
  if (value.empty())
  {
    return 0;
  }

  auto parsed = std::stoul(value);
  if (parsed == 0)
  {
    return 0;
  }
  parsed = std::clamp(parsed, min_value, max_value);
  return static_cast<uint16_t>(parsed);
}
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
  joint_to_sdk_motor_index_.resize(joint_count);
  joint_name_to_index_.clear();
  std::array<bool, kFingerCount> sdk_motor_index_seen{};
  for (std::size_t i = 0; i < joint_count; ++i)
  {
    joint_names_[i] = info_.joints[i].name;
    joint_name_to_index_[joint_names_[i]] = i;
    const auto sdk_index = sdk_motor_index_for_joint_name(joint_names_[i], i);
    if (sdk_index >= kFingerCount)
    {
      BRAINCO_HAND_LOG_ERROR(
        "Joint %s maps to invalid SDK motor index %zu", joint_names_[i].c_str(), sdk_index);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (sdk_motor_index_seen[sdk_index])
    {
      BRAINCO_HAND_LOG_ERROR(
        "Multiple ROS joints map to SDK motor index %zu; check Revo2 joint names", sdk_index);
      return hardware_interface::CallbackReturn::ERROR;
    }
    sdk_motor_index_seen[sdk_index] = true;
    joint_to_sdk_motor_index_[i] = sdk_index;
    BRAINCO_HAND_LOG_INFO(
      "  Joint[%zu]=%s -> SDK motor[%zu]", i, joint_names_[i].c_str(), sdk_index);
  }

  hw_positions_.assign(joint_count, 0.0);
  hw_velocities_.assign(joint_count, 10.0);
  hw_currents_.assign(joint_count, 0.0);
  hw_commands_.assign(joint_count, 0.0);
  hw_motor_states_.assign(joint_count, 0.0);
  hw_touch_nf1_.assign(joint_count, 0.0);
  hw_touch_tf1_.assign(joint_count, 0.0);
  hw_touch_td1_.assign(joint_count, 0.0);
  hw_touch_sp1_.assign(joint_count, 0.0);
  hw_touch_status_.assign(joint_count, 0.0);
  hw_duration_command_.assign(joint_count, static_cast<double>(config_.ctrl_param_duration_ms));
  hw_velocities_command_.assign(joint_count, 0.0);
  hw_speeds_command_.assign(joint_count, 0.0);
  hw_currents_command_.assign(joint_count, 0.0);
  hw_pwms_command_.assign(joint_count, 0.0);

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
  stop_sdk_services();
  const std::lock_guard<std::mutex> lock(io_mutex_);
  (void)previous_state;
  BRAINCO_HAND_LOG_INFO("on_configure invoked");
  is_active_ = false;
  command_mode_initialized_ = false;

  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  std::fill(hw_currents_.begin(), hw_currents_.end(), 0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_motor_states_.begin(), hw_motor_states_.end(), 0.0);
  std::fill(hw_touch_nf1_.begin(), hw_touch_nf1_.end(), 0.0);
  std::fill(hw_touch_tf1_.begin(), hw_touch_tf1_.end(), 0.0);
  std::fill(hw_touch_td1_.begin(), hw_touch_td1_.end(), 0.0);
  std::fill(hw_touch_sp1_.begin(), hw_touch_sp1_.end(), 0.0);
  std::fill(hw_touch_status_.begin(), hw_touch_status_.end(), 0.0);
  std::fill(hw_duration_command_.begin(), hw_duration_command_.end(), static_cast<double>(config_.ctrl_param_duration_ms));
  std::fill(hw_velocities_command_.begin(), hw_velocities_command_.end(), 0.0);
  std::fill(hw_speeds_command_.begin(), hw_speeds_command_.end(), 1.0);  // 非零的合理默认值
  std::fill(hw_currents_command_.begin(), hw_currents_command_.end(), 1.0);  // 非零的合理默认值
  std::fill(hw_pwms_command_.begin(), hw_pwms_command_.end(), 1.0);  // 非零的合理默认值

  if (!open_connection_with_retry())
  {
    BRAINCO_HAND_LOG_ERROR(
      "Failed to open transport session during configuration. "
      "The hardware will not be activated. Please check the connection and try again.");
    // Return ERROR to prevent activation, but don't crash
    return hardware_interface::CallbackReturn::ERROR;
  }

  try
  {
    start_sdk_services();
  }
  catch (const std::exception & exception)
  {
    BRAINCO_HAND_LOG_ERROR("Could not start SDK services: %s", exception.what());
    close_connection();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::on_cleanup(const rclcpp_lifecycle::State & previous_state)
  -> hardware_interface::CallbackReturn
{
  stop_sdk_services();
  const std::lock_guard<std::mutex> lock(io_mutex_);
  (void)previous_state;
  BRAINCO_HAND_LOG_INFO("on_cleanup invoked");
  is_active_ = false;
  command_mode_initialized_ = false;
  close_connection();
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::refresh_finger_position_range() -> bool
{
  // Seed with firmware-default fallbacks so a partial/failed read still leaves a
  // usable range on every finger.
  for (std::size_t i = 0; i < kFingerCount; ++i)
  {
    finger_lower_rad_[i] = kDefaultFingerMinDeg[i] * kDegToRad;
    finger_upper_rad_[i] = kDefaultFingerMaxDeg[i] * kDegToRad;
  }

  const auto range = api_.get_finger_position_range(config_.transport.slave_id);
  if (!range)
  {
    BRAINCO_HAND_LOG_WARN(
      "Could not read finger position range from device; using firmware-default limits "
      "(max 60/87/84/84/84/84 deg). Position commands will be clamped to these.");
    finger_range_valid_ = true;  // fallbacks are still a valid, safe range
    return true;
  }

  std::ostringstream summary;
  summary.setf(std::ios::fixed);
  summary.precision(1);
  summary << "Finger position range (deg) [min,max]:";
  for (std::size_t i = 0; i < kFingerCount; ++i)
  {
    double min_deg = range->min_deg[i];
    double max_deg = range->max_deg[i];
    // Reject implausible readings (0 max, negative span, absurd magnitude) and
    // keep the seeded fallback for that finger instead.
    const bool max_ok = std::isfinite(max_deg) && max_deg > 0.0 &&
      max_deg <= kMaxPlausibleFingerDeg;
    const bool min_ok = std::isfinite(min_deg) && min_deg >= 0.0 && min_deg < max_deg;
    if (max_ok)
    {
      finger_upper_rad_[i] = max_deg * kDegToRad;
      finger_lower_rad_[i] = (min_ok ? min_deg : 0.0) * kDegToRad;
    }
    else
    {
      min_deg = kDefaultFingerMinDeg[i];
      max_deg = kDefaultFingerMaxDeg[i];
    }
    summary << " " << i << ":[" << (finger_lower_rad_[i] / kDegToRad) << ","
            << (finger_upper_rad_[i] / kDegToRad) << "]";
  }
  finger_range_valid_ = true;
  BRAINCO_HAND_LOG_INFO("%s", summary.str().c_str());
  return true;
}

auto BraincoHandHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
  -> hardware_interface::CallbackReturn
{
  const std::lock_guard<std::mutex> lock(io_mutex_);
  (void)previous_state;
  BRAINCO_HAND_LOG_INFO("on_activate invoked");

  if (!api_.is_open())
  {
    BRAINCO_HAND_LOG_ERROR("Cannot activate without valid transport session");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (config_.transport.finger_unit_mode != FingerUnitModeSetting::kKeepCurrent)
  {
    ensure_finger_unit_mode();
  }
  apply_thumb_aux_settings();
  // Read the per-finger physical limits so write()/read() can map physical
  // radians onto the SDK's unified 0~1000 line for each finger individually.
  refresh_finger_position_range();
  last_range_warn_stamp_.fill(-1.0);

  for (std::size_t i = 0; i < hw_positions_.size(); ++i)
  {
    hw_commands_[i] = hw_positions_[i];
    hw_velocities_command_[i] = 0.0;
    hw_speeds_command_[i] = hw_velocities_[i];  // 设置合理的默认速度值
  }
  last_touch_read_time_ = {};
  consecutive_read_failures_ = 0;

  is_active_ = true;
  BRAINCO_HAND_LOG_INFO("Hardware activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
  -> hardware_interface::CallbackReturn
{
  const std::lock_guard<std::mutex> lock(io_mutex_);
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
  std::size_t total_interfaces = 0;
  for (const auto & joint : info_.joints)
  {
    total_interfaces += joint.state_interfaces.size();
  }
  state_interfaces.reserve(total_interfaces);

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints[i];
    for (const auto & state_interface : joint.state_interfaces)
    {
      const auto & interface_name = state_interface.name;
      if (interface_name == hardware_interface::HW_IF_POSITION)
      {
        state_interfaces.emplace_back(
          joint.name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
      }
      else if (interface_name == hardware_interface::HW_IF_VELOCITY)
      {
        state_interfaces.emplace_back(
          joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
      }
      else if (interface_name == kTouchNf1Interface)
      {
        state_interfaces.emplace_back(joint.name, kTouchNf1Interface, &hw_touch_nf1_[i]);
      }
      else if (interface_name == kCurrentInterface)
      {
        // 电流观测值 / motor current from SDK (raw unit).
        state_interfaces.emplace_back(joint.name, kCurrentInterface, &hw_currents_[i]);
      }
      else if (interface_name == kMotorStateInterface)
      {
        // 暴露 SDK 原始状态码，便于诊断分析。
        // Raw SDK motor state for diagnostics and replay analysis.
        state_interfaces.emplace_back(joint.name, kMotorStateInterface, &hw_motor_states_[i]);
      }
      else if (interface_name == kTouchTf1Interface)
      {
        state_interfaces.emplace_back(joint.name, kTouchTf1Interface, &hw_touch_tf1_[i]);
      }
      else if (interface_name == kTouchTd1Interface)
      {
        state_interfaces.emplace_back(joint.name, kTouchTd1Interface, &hw_touch_td1_[i]);
      }
      else if (interface_name == kTouchSp1Interface)
      {
        state_interfaces.emplace_back(joint.name, kTouchSp1Interface, &hw_touch_sp1_[i]);
      }
      else if (interface_name == kTouchStatusInterface)
      {
        state_interfaces.emplace_back(joint.name, kTouchStatusInterface, &hw_touch_status_[i]);
      }
    }
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
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_command_[i]));
  }

  return command_interfaces;
}

auto BraincoHandHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> hardware_interface::return_type
{
  const std::lock_guard<std::mutex> lock(io_mutex_);
  (void)period;

  // On a failed read cycle, hold the last good state briefly (a single dropped
  // frame is transient); once failures persist past the tolerance window, stamp
  // the UNKNOWN(255) sentinel into hw_motor_states_ so the broadcaster stops
  // advertising a healthy last frame and hardware_monitor trips ERROR. Return
  // ERROR either way so ros2_control also sees the failure.
  auto handle_read_failure = [this]() -> hardware_interface::return_type {
    if (consecutive_read_failures_ < std::numeric_limits<uint32_t>::max())
    {
      ++consecutive_read_failures_;
    }
    if (consecutive_read_failures_ >= std::max<uint32_t>(1, config_.read_failure_tolerance))
    {
      std::fill(
        hw_motor_states_.begin(), hw_motor_states_.end(), kMotorStateUnknownSentinel);
    }
    return hardware_interface::return_type::ERROR;
  };

  if (!api_.is_open())
  {
    BRAINCO_HAND_LOG_WARN("Skip read: API not connected");
    return handle_read_failure();
  }

  auto status = api_.get_motor_status(config_.transport.slave_id);
  if (!status)
  {
    BRAINCO_HAND_LOG_WARN("BraincoHandApi::get_motor_status returned no data");
    return handle_read_failure();
  }

  // Successful read: clear the failure debounce so the next drop starts fresh.
  consecutive_read_failures_ = 0;

  const auto & motor_status = *status;

  const auto joint_count = std::min<std::size_t>(hw_positions_.size(), kFingerCount);
  if (joint_count == 0)
  {
    BRAINCO_HAND_LOG_WARN("Skip read: no joints configured");
    return hardware_interface::return_type::OK;
  }
  for (std::size_t i = 0; i < joint_count; ++i)
  {
    const auto sdk_index =
      i < joint_to_sdk_motor_index_.size() ? joint_to_sdk_motor_index_[i] : i;
    const auto raw_position = static_cast<double>(motor_status.positions[sdk_index]);
    const auto raw_velocity = static_cast<double>(motor_status.speeds[sdk_index]);
    const auto raw_current = static_cast<double>(motor_status.currents[sdk_index]);
    // Reverse of write(): map the unified 0~1000 device position back onto the
    // per-finger physical range so the reported joint state is in radians.
    if (finger_range_valid_ && sdk_index < kFingerCount)
    {
      const double device_span = config_.position_device_max - config_.position_device_min;
      const double normalized =
        device_span > 0.0 ? (raw_position - config_.position_device_min) / device_span : 0.0;
      hw_positions_[i] = finger_lower_rad_[sdk_index] +
        normalized * (finger_upper_rad_[sdk_index] - finger_lower_rad_[sdk_index]);
    }
    else
    {
      hw_positions_[i] = raw_position * config_.position_state_scale;
    }
    hw_velocities_[i] = raw_velocity * config_.velocity_state_scale;
    // SDK currents/states are published to dynamic_joint_states for monitor usage.
    // 将 SDK 电流与状态发布到 dynamic_joint_states，供 Monitor 统一监测。
    hw_currents_[i] = raw_current;
    hw_motor_states_[i] = static_cast<double>(motor_status.states[sdk_index]);
  }

  if (config_.debug_write_commands)
  {
    const double now_seconds = time.seconds();
    const bool should_log = last_read_debug_stamp_ < 0.0 ||
      config_.debug_write_interval <= 0.0 ||
      (now_seconds - last_read_debug_stamp_) >= config_.debug_write_interval;
    if (should_log)
    {
      last_read_debug_stamp_ = now_seconds;
      std::ostringstream read_stream;
      read_stream.setf(std::ios::fixed);
      read_stream.precision(3);
      read_stream << "SDK read debug ros_position_deg=[";
      for (std::size_t i = 0; i < joint_count; ++i)
      {
        if (i > 0)
        {
          read_stream << ", ";
        }
        read_stream << hw_positions_[i] / kDegToRad;
      }
      read_stream << "] ros_velocity_deg_s=[";
      for (std::size_t i = 0; i < joint_count; ++i)
      {
        if (i > 0)
        {
          read_stream << ", ";
        }
        read_stream << hw_velocities_[i] / kDegToRad;
      }
      read_stream << "] sdk_positions=[";
      for (std::size_t i = 0; i < joint_count; ++i)
      {
        if (i > 0)
        {
          read_stream << ", ";
        }
        const auto sdk_index =
          i < joint_to_sdk_motor_index_.size() ? joint_to_sdk_motor_index_[i] : i;
        read_stream << motor_status.positions[sdk_index];
      }
      read_stream << "] sdk_speeds=[";
      for (std::size_t i = 0; i < joint_count; ++i)
      {
        if (i > 0)
        {
          read_stream << ", ";
        }
        const auto sdk_index =
          i < joint_to_sdk_motor_index_.size() ? joint_to_sdk_motor_index_[i] : i;
        read_stream << motor_status.speeds[sdk_index];
      }
      read_stream << "]";
      BRAINCO_HAND_LOG_INFO("%s", read_stream.str().c_str());
    }
  }

  bool should_read_touch_status = false;
  if (config_.read_touch_status && config_.touch_read_hz > 0.0)
  {
    const auto touch_now = std::chrono::steady_clock::now();
    if (last_touch_read_time_ == std::chrono::steady_clock::time_point{})
    {
      should_read_touch_status = true;
    }
    else
    {
      const double elapsed_since_touch_read =
        std::chrono::duration<double>(touch_now - last_touch_read_time_).count();
      should_read_touch_status = elapsed_since_touch_read >= (1.0 / config_.touch_read_hz);
    }
  }

  // Touch sensors are slow on Modbus, so read them at a lower synchronous rate.
  if (should_read_touch_status)
  {
    const auto touch_status = api_.get_touch_status(config_.transport.slave_id);
    last_touch_read_time_ = std::chrono::steady_clock::now();
    if (touch_status)
    {
      const bool is_right_hand =
        !joint_names_.empty() && joint_names_.front().rfind("right_", 0) == 0;
      const char * prefix = is_right_hand ? "right_" : "left_";
      const std::array<const char *, kTouchFingerCount> touch_joint_suffixes{
        "thumb_proximal_joint",
        "index_proximal_joint",
        "middle_proximal_joint",
        "ring_proximal_joint",
        "pinky_proximal_joint"};

      for (std::size_t i = 0; i < kTouchFingerCount; ++i)
      {
        const std::string joint_name = std::string(prefix) + touch_joint_suffixes[i];
        const auto iter = joint_name_to_index_.find(joint_name);
        if (iter == joint_name_to_index_.end())
        {
          continue;
        }

        const std::size_t joint_index = iter->second;
        const auto & item = touch_status->items[i];
        hw_touch_nf1_[joint_index] =
          static_cast<double>(item.tactile_normal_force) / kTactileForceScale;
        hw_touch_tf1_[joint_index] =
          static_cast<double>(item.tactile_tangential_force) / kTactileForceScale;
        hw_touch_td1_[joint_index] =
          static_cast<double>(item.tactile_tangential_direction) * kDegToRad;
        hw_touch_sp1_[joint_index] = static_cast<double>(item.tactile_self_proximity);
        hw_touch_status_[joint_index] = static_cast<double>(item.tactile_status);
      }
    }
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
  const std::lock_guard<std::mutex> lock(io_mutex_);
  (void)time;
  (void)period;

  if (!api_.is_open())
  {
    BRAINCO_HAND_LOG_WARN("Skip write: API not connected");
    return hardware_interface::return_type::ERROR;
  }

  if (!is_active_)
  {
    // BRAINCO_HAND_LOG_WARN("Skip write: hardware not active");
    return hardware_interface::return_type::OK;
  }

  if (!command_mode_initialized_)
  {
    // Hardware is activated before the command controller is spawned. Sending
    // the initial zero position here would open the hand before PID takes over.
    return hardware_interface::return_type::OK;
  }

  // 检查模式是否改变
  if (control_mode_ != last_control_mode_)
  {
    auto mode_to_string = [](ControlMode mode) -> const char* {
      switch (mode)
      {
        case ControlMode::kPOSDurationBased:
          return "DURATION-BASED";
        case ControlMode::kPOSVelocityBased:
          return "VELOCITY-BASED";
        case ControlMode::kSpeedBased:
          return "SPEED-BASED";
        case ControlMode::kCurrentBased:
          return "CURRENT-BASED";
        case ControlMode::kPWMBased:
          return "PWM-BASED";
        default:
          return "UNKNOWN";
      }
    };
    
    BRAINCO_HAND_LOG_INFO(
      "Control mode switching from %s to %s",
      mode_to_string(last_control_mode_),
      mode_to_string(control_mode_));
    
    last_control_mode_ = control_mode_;
  }

  std::array<uint16_t, kFingerCount> goal_positions{};
  std::array<uint16_t, kFingerCount> control_values_uint16{};
  std::array<int16_t, kFingerCount> control_values_int16{};  

  const auto joint_count = std::min<std::size_t>(hw_commands_.size(), kFingerCount);
  if (joint_count == 0)
  {
    BRAINCO_HAND_LOG_WARN("Skip write: no joints configured");
    return hardware_interface::return_type::OK;
  }
  for (std::size_t i = 0; i < joint_count; ++i)
  {
    const auto sdk_index =
      i < joint_to_sdk_motor_index_.size() ? joint_to_sdk_motor_index_[i] : i;
    if (sdk_index >= kFingerCount)
    {
      continue;
    }
    // Physical-quantity control: hw_commands_ is a joint angle in radians. Clamp
    // it to the per-finger range read from the SDK, then map that physical range
    // onto the device's unified 0~1000 line (0 -> lower, 1000 -> upper). The SDK
    // position API only accepts 0~1000, so this keeps commands physically
    // meaningful while never sending an out-of-range value that would fault.
    const double lower_rad = finger_lower_rad_[sdk_index];
    const double upper_rad = finger_upper_rad_[sdk_index];
    const double raw_rad = std::isfinite(hw_commands_[i]) ? hw_commands_[i] : lower_rad;
    const double commanded_rad = std::clamp(raw_rad, lower_rad, upper_rad);

    if (raw_rad < lower_rad - kRangeWarnToleranceRad ||
      raw_rad > upper_rad + kRangeWarnToleranceRad)
    {
      const double now_seconds = time.seconds();
      double & last_warn = last_range_warn_stamp_[sdk_index];
      if (last_warn < 0.0 || (now_seconds - last_warn) >= kRangeWarnIntervalSeconds)
      {
        last_warn = now_seconds;
        BRAINCO_HAND_LOG_WARN(
          "Position command %.4f rad (%.1f deg) for joint %s out of range "
          "[%.4f, %.4f] rad ([%.1f, %.1f] deg); clamping to %.4f rad.",
          raw_rad, raw_rad / kDegToRad, joint_names_[i].c_str(), lower_rad, upper_rad,
          lower_rad / kDegToRad, upper_rad / kDegToRad, commanded_rad);
      }
    }

    const double span_rad = upper_rad - lower_rad;
    const double normalized = span_rad > 0.0 ? (commanded_rad - lower_rad) / span_rad : 0.0;
    const double desired_device =
      config_.position_device_min +
      normalized * (config_.position_device_max - config_.position_device_min);
    const double clamped =
      std::clamp(desired_device, config_.position_device_min, config_.position_device_max);

    // Convert position to uint16_t using converter
    goal_positions[sdk_index] = CommandConverter::to_uint16(
      clamped, config_.position_device_min, config_.position_device_max);
    
    // Select and convert control value based on control mode
    switch (control_mode_)
    {
      case ControlMode::kPOSDurationBased:
        control_values_uint16[sdk_index] = CommandConverter::to_uint16(
          hw_duration_command_[i], 0.0, 65535.0);
        break;
      case ControlMode::kPOSVelocityBased:
      {
        double velocity_percentage = std::abs(hw_velocities_command_[i]);
        if (!std::isfinite(velocity_percentage) || velocity_percentage <= 0.0)
        {
          velocity_percentage = config_.velocity_percentage;
        }
        // Normalized SDK speed is 0-1000; physical mode uses deg/s (keep 0-100 as-is).
        if (config_.transport.finger_unit_mode == FingerUnitModeSetting::kNormalized)
        {
          const double sdk_speed = velocity_percentage / 100.0 * 1000.0;
          control_values_uint16[sdk_index] = CommandConverter::to_uint16(
            sdk_speed, 0.0, 1000.0);
        }
        else
        {
          control_values_uint16[sdk_index] = CommandConverter::to_uint16(
            velocity_percentage, 0.0, 100.0);
        }
        break;
      }
      case ControlMode::kSpeedBased:
      {
        const double sdk_speed = hw_velocities_command_[i] * config_.velocity_command_scale;
        control_values_int16[sdk_index] = CommandConverter::to_int16(
          sdk_speed, config_.velocity_device_min, config_.velocity_device_max);
        break;
      }
      case ControlMode::kCurrentBased:
        control_values_int16[sdk_index] = CommandConverter::to_int16(
          hw_currents_command_[i], -32768.0, 32767.0);
        break;
      case ControlMode::kPWMBased:
        control_values_int16[sdk_index] = CommandConverter::to_int16(
          hw_pwms_command_[i], -32768.0, 32767.0);
        break;
      default:
        BRAINCO_HAND_LOG_WARN(
          "Invalid control mode %d for joint %s", static_cast<int>(control_mode_),
          joint_names_[i].c_str());
        break;
    }
  }

  if (config_.debug_write_commands)
  {
    const double now_seconds = time.seconds();
    const bool should_log = last_write_debug_stamp_ < 0.0 ||
      config_.debug_write_interval <= 0.0 ||
      (now_seconds - last_write_debug_stamp_) >= config_.debug_write_interval;
    if (should_log)
    {
      last_write_debug_stamp_ = now_seconds;
      auto mode_to_string = [](ControlMode mode) -> const char * {
        switch (mode)
        {
          case ControlMode::kPOSDurationBased:
            return "position_duration";
          case ControlMode::kPOSVelocityBased:
            return "position_velocity";
          case ControlMode::kSpeedBased:
            return "speed";
          case ControlMode::kCurrentBased:
            return "current";
          case ControlMode::kPWMBased:
            return "pwm";
          default:
            return "unknown";
        }
      };

      std::ostringstream write_stream;
      write_stream.setf(std::ios::fixed);
      write_stream.precision(3);
      write_stream << "SDK write debug mode=" << mode_to_string(control_mode_);
      write_stream << " ros_position_rad=[";
      for (std::size_t i = 0; i < joint_count; ++i)
      {
        if (i > 0)
        {
          write_stream << ", ";
        }
        write_stream << hw_commands_[i];
      }
      write_stream << "] ros_velocity_rad_s=[";
      for (std::size_t i = 0; i < joint_count; ++i)
      {
        if (i > 0)
        {
          write_stream << ", ";
        }
        write_stream << hw_velocities_command_[i];
      }
      write_stream << "] ros_position_deg=[";
      for (std::size_t i = 0; i < joint_count; ++i)
      {
        if (i > 0)
        {
          write_stream << ", ";
        }
        write_stream << hw_commands_[i] / kDegToRad;
      }
      write_stream << "] ros_velocity_deg_s=[";
      for (std::size_t i = 0; i < joint_count; ++i)
      {
        if (i > 0)
        {
          write_stream << ", ";
        }
        write_stream << hw_velocities_command_[i] / kDegToRad;
      }
      write_stream << "] sdk_positions=[";
      for (std::size_t i = 0; i < kFingerCount; ++i)
      {
        if (i > 0)
        {
          write_stream << ", ";
        }
        write_stream << goal_positions[i];
      }
      write_stream << "] sdk_control=[";
      for (std::size_t i = 0; i < kFingerCount; ++i)
      {
        if (i > 0)
        {
          write_stream << ", ";
        }
        if (control_mode_ == ControlMode::kPOSDurationBased ||
          control_mode_ == ControlMode::kPOSVelocityBased)
        {
          write_stream << control_values_uint16[i];
        }
        else
        {
          write_stream << control_values_int16[i];
        }
      }
      write_stream << "]";
      BRAINCO_HAND_LOG_INFO("%s", write_stream.str().c_str());
    }
  }

  // Call appropriate API method based on control mode
  bool result = false;
  switch (control_mode_)
  {
    case ControlMode::kPOSDurationBased:
      result = api_.set_finger_positions_and_durations(
        config_.transport.slave_id, goal_positions.data(), control_values_uint16.data(), joint_count);
      break;
    case ControlMode::kPOSVelocityBased:
      result = api_.set_finger_positions_and_velocities(
        config_.transport.slave_id, goal_positions.data(), control_values_uint16.data(), joint_count); 
      break;
    case ControlMode::kSpeedBased:
      result = api_.set_finger_speeds(config_.transport.slave_id, control_values_int16.data(), joint_count);
      break;
    case ControlMode::kCurrentBased:
      result = api_.set_finger_currents(config_.transport.slave_id, control_values_int16.data(), joint_count);
      break;
    case ControlMode::kPWMBased:
      result = api_.set_finger_pwms(config_.transport.slave_id, control_values_int16.data(), joint_count);
      break;
    default:
      BRAINCO_HAND_LOG_WARN(
        "Failed to send finger command for slave %u with invalid control mode %d", 
        config_.transport.slave_id,
        static_cast<int>(control_mode_));
      return hardware_interface::return_type::ERROR;
  }

  if (!result)
  {
    auto mode_to_string = [](ControlMode mode) -> const char* {
      switch (mode)
      {
        case ControlMode::kPOSDurationBased:
          return "POSDURATION-BASED";
        case ControlMode::kPOSVelocityBased:
          return "POSVELOCITY-BASED";
        case ControlMode::kSpeedBased:
          return "SPEED-BASED";
        case ControlMode::kCurrentBased:
          return "CURRENT-BASED";
        case ControlMode::kPWMBased:
          return "PWM-BASED";
        default:
          return "UNKNOWN";
      }
    };
    
    BRAINCO_HAND_LOG_WARN(
      "Failed to send finger command for slave %u (mode=%s)", 
      config_.transport.slave_id,
      mode_to_string(control_mode_));
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
    config_.transport.sdk_file_logging =
      parse_bool(get_parameter("sdk_file_logging", "false"), false);
    config_.transport.ensure_physical_mode =
      parse_bool(get_parameter("ensure_physical_mode", "false"), false);
    const auto finger_unit_mode_param = get_parameter("finger_unit_mode", "");
    if (!finger_unit_mode_param.empty())
    {
      config_.transport.finger_unit_mode =
        parse_finger_unit_mode_setting(finger_unit_mode_param);
    }
    else if (config_.transport.ensure_physical_mode)
    {
      config_.transport.finger_unit_mode = FingerUnitModeSetting::kPhysical;
    }
    else
    {
      config_.transport.finger_unit_mode = FingerUnitModeSetting::kNormalized;
    }

    // Parse control mode (duration or velocity)
    const auto control_mode_str = get_parameter("control_mode", "velocity");
    std::string mode_lower = control_mode_str;
    std::transform(
      mode_lower.begin(), mode_lower.end(), mode_lower.begin(),
      [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    
    if (mode_lower == "velocity")
    {
      control_mode_ = ControlMode::kPOSVelocityBased;
      BRAINCO_HAND_LOG_INFO("Control mode set to VELOCITY-BASED");
    }
    else if (mode_lower == "speed")
    {
      control_mode_ = ControlMode::kSpeedBased;
      BRAINCO_HAND_LOG_INFO("Control mode set to SPEED-BASED");
    }
    else if (mode_lower == "current" || mode_lower == "effort")
    {
      control_mode_ = ControlMode::kCurrentBased;
      BRAINCO_HAND_LOG_INFO("Control mode set to CURRENT-BASED");
    }
    else if (mode_lower == "pwm")
    {
      control_mode_ = ControlMode::kPWMBased;
      BRAINCO_HAND_LOG_INFO("Control mode set to PWM-BASED");
    }
    else
    {
      control_mode_ = ControlMode::kPOSDurationBased;
      BRAINCO_HAND_LOG_INFO("Control mode set to DURATION-BASED (default)");
    }

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
    const auto velocity_command_scale_param = get_parameter("velocity_command_scale", "");
    if (!velocity_command_scale_param.empty())
    {
      config_.velocity_command_scale = std::stod(velocity_command_scale_param);
    }
    else
    {
      config_.velocity_command_scale = 0.0;
    }
    config_.velocity_percentage = std::stod(get_parameter("velocity_percentage", "100.0"));
    config_.position_device_min = std::stod(get_parameter("position_device_min", "0.0"));
    config_.position_device_max = std::stod(get_parameter("position_device_max", "1000.0"));
    config_.connection_retry_attempts = static_cast<uint32_t>(
      std::stoul(get_parameter("connection_retry_attempts", "20")));
    config_.connection_retry_interval_ms = static_cast<uint32_t>(
      std::stoul(get_parameter("connection_retry_interval_ms", "500")));
    config_.read_failure_tolerance = static_cast<uint32_t>(
      std::stoul(get_parameter("read_failure_tolerance", "5")));
    config_.velocity_device_min = std::stod(get_parameter("velocity_device_min", "-1000.0"));
    config_.velocity_device_max = std::stod(get_parameter("velocity_device_max", "1000.0"));
    config_.debug_write_commands =
      parse_bool(get_parameter("debug_write_commands", "false"), false);
    config_.debug_write_interval = std::stod(get_parameter("debug_write_interval", "0.5"));
    config_.read_touch_status = parse_bool(get_parameter("read_touch_status", "true"), true);
    config_.touch_read_hz = std::stod(get_parameter("touch_read_hz", "20.0"));
    if (config_.touch_read_hz < 0.0)
    {
      BRAINCO_HAND_LOG_WARN(
        "touch_read_hz=%f is invalid; clamping to 0.0", config_.touch_read_hz);
      config_.touch_read_hz = 0.0;
    }

    if (config_.transport.finger_unit_mode == FingerUnitModeSetting::kPhysical)
    {
      std::size_t physical_override_count = 0;
      auto apply_physical_double = [&](const char * parameter_name, double & target) -> bool {
        const auto parameter_value = get_parameter(parameter_name, "");
        if (parameter_value.empty())
        {
          return false;
        }
        target = std::stod(parameter_value);
        ++physical_override_count;
        BRAINCO_HAND_LOG_INFO(
          "Applied physical-mode scale override %s=%f", parameter_name, target);
        return true;
      };

      const bool position_command_overridden = apply_physical_double(
        "physical_position_command_scale", config_.position_command_scale);
      const bool position_state_overridden =
        apply_physical_double("physical_position_state_scale", config_.position_state_scale);
      const bool velocity_state_overridden =
        apply_physical_double("physical_velocity_state_scale", config_.velocity_state_scale);
      const bool velocity_command_overridden = apply_physical_double(
        "physical_velocity_command_scale", config_.velocity_command_scale);
      apply_physical_double("physical_position_device_min", config_.position_device_min);
      apply_physical_double("physical_position_device_max", config_.position_device_max);
      apply_physical_double("physical_velocity_device_min", config_.velocity_device_min);
      apply_physical_double("physical_velocity_device_max", config_.velocity_device_max);

      if (position_state_overridden && !position_command_overridden &&
        config_.position_state_scale != 0.0)
      {
        config_.position_command_scale = 1.0 / config_.position_state_scale;
        BRAINCO_HAND_LOG_INFO(
          "Derived physical_position_command_scale=%f from physical_position_state_scale",
          config_.position_command_scale);
      }
      if (velocity_state_overridden && !velocity_command_overridden &&
        config_.velocity_state_scale != 0.0)
      {
        config_.velocity_command_scale = 1.0 / config_.velocity_state_scale;
        BRAINCO_HAND_LOG_INFO(
          "Derived physical_velocity_command_scale=%f from physical_velocity_state_scale",
          config_.velocity_command_scale);
      }
      if (physical_override_count == 0)
      {
        BRAINCO_HAND_LOG_WARN(
          "finger_unit_mode=physical but no physical_* scale overrides were configured; "
          "using the base position/velocity scales");
      }
      if (!position_command_overridden && !position_state_overridden)
      {
        BRAINCO_HAND_LOG_WARN(
          "finger_unit_mode=physical but physical_position_* scales are unset; "
          "position writes still use the 0-1000 normalized scale (command_scale=%f device_max=%f). "
          "Firmware physical mode expects degrees (typically 0-100; per-joint max 60/87/84) "
          "and will reject values such as 600/840 with Modbus IllegalDataValue. "
          "Set physical_position_command_scale and physical_position_device_max, "
          "or switch finger_unit_mode to normalized.",
          config_.position_command_scale, config_.position_device_max);
      }
    }

    config_.thumb_aux_lock_current =
      parse_optional_uint16(get_parameter("thumb_aux_lock_current", ""), 100, 500);
    config_.thumb_aux_max_current =
      parse_optional_uint16(get_parameter("thumb_aux_max_current", ""), 1, 2000);
    config_.thumb_aux_protected_current =
      parse_optional_uint16(get_parameter("thumb_aux_protected_current", ""), 100, 1500);
    config_.thumb_aux_max_speed =
      parse_optional_uint16(get_parameter("thumb_aux_max_speed", ""), 1, 1000);
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

  if (config_.velocity_device_min > config_.velocity_device_max)
  {
    std::swap(config_.velocity_device_min, config_.velocity_device_max);
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

  if (config_.velocity_command_scale == 0.0)
  {
    config_.velocity_command_scale = 1.0 / config_.velocity_state_scale;
  }

  if (config_.velocity_percentage < 0.0)
  {
    config_.velocity_percentage = 0.0;
  }
  if (config_.velocity_percentage > 100.0)
  {
    config_.velocity_percentage = 100.0;
  }
  if (config_.connection_retry_attempts == 0)
  {
    config_.connection_retry_attempts = 1;
  }

  if (config_.read_failure_tolerance == 0)
  {
    config_.read_failure_tolerance = 1;
  }

  if (config_.debug_write_interval < 0.0)
  {
    config_.debug_write_interval = 0.0;
  }

  const char * protocol_label = config_.transport.protocol == Protocol::kCanfd ? "CANFD" : "MODBUS";

  BRAINCO_HAND_LOG_INFO(
    "Driver config -> protocol=%s slave_id=%u duration=%u finger_unit_mode=%s "
    "ensure_physical_mode=%s",
    protocol_label, config_.transport.slave_id, config_.ctrl_param_duration_ms,
    finger_unit_mode_setting_to_string(config_.transport.finger_unit_mode),
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
    "position_command_scale=%f position_state_scale=%f velocity_state_scale=%f velocity_command_scale=%f velocity_percentage=%f%% "
    "position_device_min=%f position_device_max=%f velocity_device_min=%f velocity_device_max=%f "
    "connection_retry_attempts=%u connection_retry_interval_ms=%u",
    config_.position_command_scale, config_.position_state_scale, config_.velocity_state_scale,
    config_.velocity_command_scale, config_.velocity_percentage, config_.position_device_min,
    config_.position_device_max, config_.velocity_device_min, config_.velocity_device_max,
    config_.connection_retry_attempts,
    config_.connection_retry_interval_ms);
  BRAINCO_HAND_LOG_INFO("read_failure_tolerance=%u", config_.read_failure_tolerance);
  BRAINCO_HAND_LOG_INFO(
    "thumb_aux settings -> lock_current=%u max_current=%u protected_current=%u max_speed=%u",
    static_cast<unsigned>(config_.thumb_aux_lock_current),
    static_cast<unsigned>(config_.thumb_aux_max_current),
    static_cast<unsigned>(config_.thumb_aux_protected_current),
    static_cast<unsigned>(config_.thumb_aux_max_speed));
  BRAINCO_HAND_LOG_INFO(
    "log_level=%s", brainco_log_level_to_string(config_.transport.log_level).c_str());
  BRAINCO_HAND_LOG_INFO(
    "debug_write_commands=%s debug_write_interval=%f",
    config_.debug_write_commands ? "true" : "false", config_.debug_write_interval);
  BRAINCO_HAND_LOG_INFO(
    "read_touch_status=%s touch_read_hz=%f",
    config_.read_touch_status ? "true" : "false", config_.touch_read_hz);

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::validate_joints() const -> hardware_interface::CallbackReturn
{
  if (info_.joints.empty())
  {
    BRAINCO_HAND_LOG_ERROR("No joints configured for revo2_driver");
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
  connected_hardware_type_.reset();
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
    connected_hardware_type_ = device_info.hardware_type;
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

auto BraincoHandHardware::open_connection_with_retry() -> bool
{
  const uint32_t attempts = std::max<uint32_t>(1, config_.connection_retry_attempts);
  for (uint32_t attempt = 1; attempt <= attempts; ++attempt)
  {
    if (open_connection())
    {
      if (attempt > 1)
      {
        BRAINCO_HAND_LOG_INFO(
          "Transport session opened after %u/%u attempt(s)", attempt, attempts);
      }
      return true;
    }

    if (attempt == attempts)
    {
      break;
    }

    BRAINCO_HAND_LOG_WARN(
      "Transport session open failed (%u/%u); retrying in %u ms",
      attempt, attempts, config_.connection_retry_interval_ms);
    std::this_thread::sleep_for(
      std::chrono::milliseconds(config_.connection_retry_interval_ms));
  }
  return false;
}

auto BraincoHandHardware::ensure_finger_unit_mode() -> bool
{
  if (!api_.is_open())
  {
    BRAINCO_HAND_LOG_WARN("ensure_finger_unit_mode skipped: API not connected");
    return false;
  }

  if (!api_.ensure_finger_unit_mode(
      config_.transport.slave_id, config_.transport.finger_unit_mode))
  {
    BRAINCO_HAND_LOG_WARN(
      "Failed to ensure %s finger unit mode for slave %u",
      finger_unit_mode_setting_to_string(config_.transport.finger_unit_mode),
      static_cast<unsigned>(config_.transport.slave_id));
    return false;
  }

  BRAINCO_HAND_LOG_INFO(
    "Finger unit mode confirmed %s",
    finger_unit_mode_setting_to_string(config_.transport.finger_unit_mode));
  return true;
}

auto BraincoHandHardware::apply_thumb_aux_settings() -> bool
{
  if (!api_.is_open())
  {
    BRAINCO_HAND_LOG_WARN("apply_thumb_aux_settings skipped: API not connected");
    return false;
  }

  bool success = true;
  const auto slave_id = config_.transport.slave_id;

  if (config_.thumb_aux_lock_current > 0)
  {
    success = api_.set_thumb_aux_lock_current(slave_id, config_.thumb_aux_lock_current) && success;
    BRAINCO_HAND_LOG_INFO(
      "ThumbAux lock current requested: %u mA",
      static_cast<unsigned>(config_.thumb_aux_lock_current));
  }
  if (config_.thumb_aux_max_current > 0)
  {
    success = api_.set_thumb_aux_max_current(slave_id, config_.thumb_aux_max_current) && success;
    BRAINCO_HAND_LOG_INFO(
      "ThumbAux max current requested: %u mA",
      static_cast<unsigned>(config_.thumb_aux_max_current));
  }
  if (config_.thumb_aux_protected_current > 0)
  {
    success =
      api_.set_thumb_aux_protected_current(slave_id, config_.thumb_aux_protected_current) &&
      success;
    BRAINCO_HAND_LOG_INFO(
      "ThumbAux protected current requested: %u mA",
      static_cast<unsigned>(config_.thumb_aux_protected_current));
  }
  if (config_.thumb_aux_max_speed > 0)
  {
    success = api_.set_thumb_aux_max_speed(slave_id, config_.thumb_aux_max_speed) && success;
    BRAINCO_HAND_LOG_INFO(
      "ThumbAux max speed requested: %u deg/s",
      static_cast<unsigned>(config_.thumb_aux_max_speed));
  }

  return success;
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

auto BraincoHandHardware::prepare_command_mode_switch(
  const std::vector<std::string> & /*command_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
  -> hardware_interface::return_type
{
  // A different component can still reject the switch. Do not change the mode
  // or enable SDK writes until controller_manager commits it in perform().
  return hardware_interface::return_type::OK;
}

auto BraincoHandHardware::perform_command_mode_switch(
  const std::vector<std::string> & command_interfaces,
  const std::vector<std::string> & stop_interfaces)
  -> hardware_interface::return_type
{
  const std::lock_guard<std::mutex> lock(io_mutex_);
  update_control_mode_from_interfaces(command_interfaces, stop_interfaces);
  
  return hardware_interface::return_type::OK;
}

auto BraincoHandHardware::update_control_mode_from_interfaces(
  const std::vector<std::string> & command_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/) -> void
{
  // Diagnostic: print all command interfaces reported by controller_manager
  {
    std::ostringstream iface_stream;
    iface_stream << "Reported command_interfaces (count=" << command_interfaces.size() << "):";
    for (const auto & iface : command_interfaces)
    {
      iface_stream << " '" << iface << "'";
    }
    BRAINCO_HAND_LOG_INFO("%s", iface_stream.str().c_str());
  }

  bool has_position_interface = false;
  bool has_velocity_interface = false;
  bool has_POSDuration_interface = false;      
  bool has_current_interface = false;   
  bool has_pwm_interface = false;      
  
  for (const auto & iface : command_interfaces)
  {
    // ros2_control passes every component's keys to this callback. Only this
    // hand's joints may select its control mode or release the startup guard.
    const auto separator = iface.rfind('/');
    if (separator == std::string::npos ||
      joint_name_to_index_.find(iface.substr(0, separator)) == joint_name_to_index_.end())
    {
      continue;
    }
    const auto interface_name = iface.substr(separator + 1);
    if (interface_name == "position")
    {
      has_position_interface = true;
    }
    if (interface_name == "velocity")
    {
      has_velocity_interface = true;
    }
    if (interface_name == "duration")
    {
      has_POSDuration_interface = true;
    }
    if (interface_name == "current")
    {
      has_current_interface = true;
    }
    if (interface_name == "pwm")
    {
      has_pwm_interface = true;
    }
  }
  
  ControlMode desired_mode = control_mode_;
  
  // 优先级：PWM > Current > (Position+Velocity) > Position > Speed
  // Revo2 的固件更稳定地接受“目标位置 + 正速度百分比”。
  // 只有单独启动 velocity interface 时才进入纯 speed 寄存器接口。
  if (has_pwm_interface)
  {
    desired_mode = ControlMode::kPWMBased;
  }
  else if (has_current_interface)
  {
    desired_mode = ControlMode::kCurrentBased;
  }
  else if (has_position_interface && has_velocity_interface)
  {
    desired_mode = ControlMode::kPOSVelocityBased;
  }
  else if (has_position_interface && has_POSDuration_interface)
  {
    desired_mode = ControlMode::kPOSDurationBased;
  }
  else if (has_position_interface)
  {
    desired_mode = ControlMode::kPOSVelocityBased;
  }
  else if (has_velocity_interface)
  {
    desired_mode = ControlMode::kSpeedBased;
  }
  else
  {
    return;
  }

  command_mode_initialized_ = true;
  
  if (desired_mode != control_mode_)
  {
    auto mode_to_string = [](ControlMode mode) -> const char* {
      switch (mode)
      {
        case ControlMode::kPOSDurationBased:
          return "DURATION-BASED";
        case ControlMode::kPOSVelocityBased:
          return "POSITION-BASED";
        case ControlMode::kSpeedBased:
          return "SPEED-BASED";
        case ControlMode::kCurrentBased:
          return "CURRENT-BASED";
        case ControlMode::kPWMBased:
          return "PWM-BASED";
        default:
          return "UNKNOWN";
      }
    };
    
    BRAINCO_HAND_LOG_INFO(
      "Prepare mode switch: changing from %s to %s based on interface activation",
      mode_to_string(control_mode_),
      mode_to_string(desired_mode));
    control_mode_ = desired_mode;
  }
}

}  // namespace revo2_driver

PLUGINLIB_EXPORT_CLASS(
  revo2_driver::BraincoHandHardware, hardware_interface::SystemInterface)
