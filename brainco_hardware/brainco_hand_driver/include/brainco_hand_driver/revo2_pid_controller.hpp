#pragma once

#include <array>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace revo2_driver
{

class Revo2PidController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kJointCount = 6;
  using JointArray = std::array<double, kJointCount>;

  void on_target(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool parse_target(const sensor_msgs::msg::JointState & msg, JointArray & target) const;
  void zero_commands();
  void reset_runtime_state();
  void warn_throttled(const rclcpp::Time & time, const std::string & message);

  double clamp_target(std::size_t index, double value) const;
  double deadband_for_joint(std::size_t index) const;
  static double clamp(double value, double low, double high);

  std::vector<std::string> joints_;
  std::string target_topic_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;

  mutable std::mutex target_mutex_;
  JointArray target_position_{};
  rclcpp::Time target_time_{0, 0, RCL_ROS_TIME};
  bool target_ready_{false};

  JointArray filtered_target_{};
  JointArray last_error_{};
  JointArray filtered_derivative_{};
  JointArray filtered_target_velocity_{};
  JointArray integral_velocity_{};
  JointArray target_velocity_{};
  JointArray command_velocity_{};
  bool filter_initialized_{false};
  rclcpp::Time last_pd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_warn_time_{0, 0, RCL_ROS_TIME};

  double target_timeout_{0.3};
  double target_filter_alpha_{0.45};
  double target_filter_fast_alpha_{0.9};
  double target_filter_fast_threshold_{0.095993109};
  double target_velocity_ff_gain_{0.0};
  double target_velocity_ff_alpha_{0.4};
  double target_velocity_ff_max_{1.0};

  double velocity_kp_{2.4};
  double velocity_ki_{0.0};
  double integral_max_{0.2};
  double velocity_kd_{0.0};
  double derivative_alpha_{0.25};
  double velocity_deadband_{0.013962634};
  double velocity_max_{2.094395102};
  double velocity_slew_rate_{1.0};
  double velocity_zero_epsilon_{0.002};

  double thumb_velocity_deadband_{0.020943951};
  double thumb_velocity_min_{0.0};
  double thumb_velocity_kp_scale_{2.4};
  double thumb_velocity_brake_zone_{0.209439510};
  double thumb_velocity_brake_max_{0.628318531};

  double ring_velocity_deadband_{0.020943951};
  double ring_velocity_min_{0.0};
  double ring_velocity_kp_scale_{1.5};

  double four_finger_extension_velocity_scale_{1.5};
  std::vector<int64_t> four_finger_extension_joints_{2, 3, 4, 5};

  JointArray joint_lower_limits_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  JointArray joint_upper_limits_{{
    1.0472, 1.5184, 1.413716694, 1.413716694, 1.413716694, 1.413716694}};
  JointArray feedback_position_scales_{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  JointArray feedback_position_offsets_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

}  // namespace revo2_driver
