#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "brainco_hand_driver/brainco_hand_api.hpp"
#include "brainco_hand_driver/command_converter.hpp"
#include "brainco_hand_driver/srv/touch_sensor_command.hpp"
#include "logger_macros.hpp"

namespace revo2_driver
{

enum class ControlMode : uint8_t
{
  kPOSDurationBased = 0,  // 按时长控制
  kPOSVelocityBased = 1,  // 按速度控制
  kSpeedBased = 2,        // 按速度控制（仅速度接口）
  kCurrentBased = 3,      // 按电流控制（仅电流接口）
  kPWMBased = 4,          // 按PWM控制（仅PWM接口）
};

class BraincoHandHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BraincoHandHardware)
  ~BraincoHandHardware() override;

  auto on_init(const hardware_interface::HardwareInfo & info)
    -> hardware_interface::CallbackReturn override;
  auto on_configure(const rclcpp_lifecycle::State & previous_state)
    -> hardware_interface::CallbackReturn override;
  auto on_cleanup(const rclcpp_lifecycle::State & previous_state)
    -> hardware_interface::CallbackReturn override;
  auto on_error(const rclcpp_lifecycle::State & previous_state)
    -> hardware_interface::CallbackReturn override;
  auto on_activate(const rclcpp_lifecycle::State & previous_state)
    -> hardware_interface::CallbackReturn override;
  auto on_deactivate(const rclcpp_lifecycle::State & previous_state)
    -> hardware_interface::CallbackReturn override;
  auto export_state_interfaces() -> std::vector<hardware_interface::StateInterface> override;
  auto export_command_interfaces() -> std::vector<hardware_interface::CommandInterface> override;
  auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> hardware_interface::return_type override;
  auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> hardware_interface::return_type override;
  auto prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces)
    -> hardware_interface::return_type override;
  auto perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces)
    -> hardware_interface::return_type override;

private:
  struct DriverConfig
  {
    static constexpr uint16_t kDefaultDurationMs{10};
    static constexpr double kDefaultPositionMin{0.0};
    static constexpr double kDefaultPositionMax{1000.0};
    static constexpr double kDefaultVelocityMin{-1000.0};
    static constexpr double kDefaultVelocityMax{1000.0};
    static constexpr double kDefaultVelocityPercentage{100.0};  // 100% = full velocity

    BraincoHandApi::DriverConfig transport{};
    uint16_t ctrl_param_duration_ms{kDefaultDurationMs};
    double position_command_scale{1.0};
    double position_state_scale{1.0};
    double velocity_state_scale{1.0};
    double velocity_command_scale{1.0};
    double velocity_device_min{kDefaultVelocityMin};
    double velocity_device_max{kDefaultVelocityMax};
    double velocity_percentage{kDefaultVelocityPercentage};  // 外部可修改的速度百分比 (0-100%)
    double position_device_min{kDefaultPositionMin};
    double position_device_max{kDefaultPositionMax};
    uint32_t connection_retry_attempts{20};
    uint32_t connection_retry_interval_ms{500};
    // Consecutive failed read() cycles tolerated before the driver declares the
    // hand disconnected. Below this count the last good state is held (a single
    // dropped frame is a transient, not a fault); at/above it the driver writes
    // an UNKNOWN(255) motor_state sentinel so the broadcaster stops advertising a
    // healthy last frame and hardware_monitor trips ERROR. Acts as a time window
    // = read_failure_tolerance × control-loop period.
    uint32_t read_failure_tolerance{5};
    bool debug_write_commands{false};
    double debug_write_interval{0.5};
    uint16_t thumb_aux_lock_current{0};
    uint16_t thumb_aux_max_current{0};
    uint16_t thumb_aux_protected_current{0};
    uint16_t thumb_aux_max_speed{0};
    bool read_touch_status{true};
    double touch_read_hz{20.0};
  };

  auto init_parameters(const hardware_interface::HardwareInfo & info)
    -> hardware_interface::CallbackReturn;
  auto close_connection() -> void;
  auto open_connection() -> bool;
  auto open_connection_with_retry() -> bool;
  auto ensure_finger_unit_mode() -> bool;
  auto apply_thumb_aux_settings() -> bool;
  void start_sdk_services();
  void stop_sdk_services();
  void handle_touch_command(
    bool calibrate,
    const std::shared_ptr<srv::TouchSensorCommand::Request> & request,
    const std::shared_ptr<srv::TouchSensorCommand::Response> & response);
  auto validate_joints() const -> hardware_interface::CallbackReturn;
  auto update_control_mode_from_interfaces(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) -> void;
  static auto parse_log_level(const std::string & level_str) -> BraincoLogLevel;
  static auto parse_bool(const std::string & value, bool default_value) -> bool;
  auto get_parameter(const std::string & key, const std::string & default_value) const
    -> std::string;

  // Services and the control loop share one SDK handle. Hold this mutex for a
  // whole operation, including lifecycle open/close, not just individual calls.
  std::mutex io_mutex_;
  DriverConfig config_{};
  std::optional<BraincoHandApi::ConnectionInfo> resolved_connection_;
  std::optional<uint8_t> connected_hardware_type_;
  BraincoHandApi api_{};
  bool is_active_{false};
  rclcpp::Node::SharedPtr sdk_service_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> sdk_service_executor_;
  std::thread sdk_service_thread_;
  std::atomic<bool> sdk_services_running_{false};
  rclcpp::Service<srv::TouchSensorCommand>::SharedPtr enable_touch_service_;
  rclcpp::Service<srv::TouchSensorCommand>::SharedPtr calibrate_touch_service_;
  double last_read_debug_stamp_{-1.0};
  double last_write_debug_stamp_{-1.0};
  // Number of consecutive read() cycles that have failed. Reset to 0 on any
  // successful read; used with config_.read_failure_tolerance to debounce a
  // single dropped frame before flagging the hand as disconnected.
  uint32_t consecutive_read_failures_{0};
  std::chrono::steady_clock::time_point last_touch_read_time_{};
  
  // Control mode configuration
  // Do not send initialized position commands before a command controller has
  // actually taken ownership. State broadcasters alone must not enable writes.
  bool command_mode_initialized_{false};
  ControlMode control_mode_{ControlMode::kPOSDurationBased};
  ControlMode last_control_mode_{ControlMode::kPOSDurationBased};  // Track last mode for transitions

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_currents_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_duration_command_;    // Duration values (double) - converted to uint16_t when sending
  std::vector<double> hw_velocities_command_;  // Velocity values (double) - converted to uint16_t when sending
  std::vector<double> hw_speeds_command_;      // Speed values (double) - converted to int16_t when sending
  std::vector<double> hw_currents_command_;    // Current values (double) - converted to int16_t when sending
  std::vector<double> hw_pwms_command_;        // PWM values (double) - converted to int16_t when sending
  std::vector<double> hw_motor_states_;
  std::vector<double> hw_touch_nf1_;
  std::vector<double> hw_touch_tf1_;
  std::vector<double> hw_touch_td1_;
  std::vector<double> hw_touch_sp1_;
  std::vector<double> hw_touch_status_;
  std::vector<std::string> joint_names_;
  std::vector<std::size_t> joint_to_sdk_motor_index_;
  std::unordered_map<std::string, std::size_t> joint_name_to_index_;

  // Per-finger physical position limits (radians), indexed by SDK motor index
  // [Thumb, ThumbAux, Index, Middle, Ring, Pinky]. Read from the device on
  // activation via stark_get_finger_min/max_position; commands are clamped to
  // this range and mapped onto the SDK's unified 0~1000 line before sending, so
  // the driver is driven in physical units yet never trips a Modbus range fault.
  std::array<double, BraincoHandApi::kFingerCount> finger_lower_rad_{};
  std::array<double, BraincoHandApi::kFingerCount> finger_upper_rad_{};
  bool finger_range_valid_{false};
  // Rate-limited out-of-range warning bookkeeping (per SDK motor index).
  std::array<double, BraincoHandApi::kFingerCount> last_range_warn_stamp_{};

  // Read the per-finger position range from the device and cache it in
  // finger_lower_rad_/finger_upper_rad_. Falls back to the YAML-configured
  // defaults when a value cannot be read. Returns false only when no usable
  // range could be established.
  auto refresh_finger_position_range() -> bool;
};

}  // namespace revo2_driver
