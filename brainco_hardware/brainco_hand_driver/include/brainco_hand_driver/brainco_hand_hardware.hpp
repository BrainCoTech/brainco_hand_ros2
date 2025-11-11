#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "brainco_hand_driver/brainco_hand_api.hpp"
#include "logger_macros.hpp"

namespace brainco_hand_driver
{

class BraincoHandHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BraincoHandHardware)

  auto on_init(const hardware_interface::HardwareInfo & info)
    -> hardware_interface::CallbackReturn override;
  auto on_configure(const rclcpp_lifecycle::State & previous_state)
    -> hardware_interface::CallbackReturn override;
  auto on_cleanup(const rclcpp_lifecycle::State & previous_state)
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

private:
  struct DriverConfig
  {
    static constexpr uint16_t kDefaultDurationMs{10};
    static constexpr double kDefaultPositionMin{0.0};
    static constexpr double kDefaultPositionMax{1000.0};

    BraincoHandApi::DriverConfig transport{};
    uint16_t ctrl_param_duration_ms{kDefaultDurationMs};
    double position_command_scale{1.0};
    double position_state_scale{1.0};
    double velocity_state_scale{1.0};
    double position_device_min{kDefaultPositionMin};
    double position_device_max{kDefaultPositionMax};
  };

  auto init_parameters(const hardware_interface::HardwareInfo & info)
    -> hardware_interface::CallbackReturn;
  auto close_connection() -> void;
  auto open_connection() -> bool;
  auto ensure_physical_unit_mode() -> bool;
  auto validate_joints() const -> hardware_interface::CallbackReturn;
  static auto parse_log_level(const std::string & level_str) -> BraincoLogLevel;
  static auto parse_bool(const std::string & value, bool default_value) -> bool;
  auto get_parameter(const std::string & key, const std::string & default_value) const
    -> std::string;

  DriverConfig config_{};
  std::optional<BraincoHandApi::ConnectionInfo> resolved_connection_;
  BraincoHandApi api_{};
  bool is_active_{false};

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<uint8_t> hw_motor_states_;
  std::vector<std::string> joint_names_;
};

}  // namespace brainco_hand_driver
