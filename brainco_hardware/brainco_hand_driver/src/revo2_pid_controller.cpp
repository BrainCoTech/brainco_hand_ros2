#include "revo2_driver/revo2_pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace revo2_driver
{

namespace
{

constexpr std::size_t kJointCount = 6;

template<typename T>
std::array<double, kJointCount> six_array_from_vector(
  const std::vector<T> & values,
  const std::array<double, kJointCount> & defaults,
  const std::string & name)
{
  if (values.empty()) {
    return defaults;
  }
  if (values.size() != kJointCount) {
    throw std::runtime_error(name + " must contain exactly 6 values.");
  }

  std::array<double, kJointCount> result{};
  for (std::size_t i = 0; i < kJointCount; ++i) {
    result[i] = static_cast<double>(values[i]);
  }
  return result;
}

std::string short_joint_name(const std::string & joint_name)
{
  const auto underscore = joint_name.find('_');
  if (underscore == std::string::npos || underscore + 1 >= joint_name.size()) {
    return joint_name;
  }
  return joint_name.substr(underscore + 1);
}

}  // namespace

controller_interface::CallbackReturn Revo2PidController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("target_topic", "~/target_joint_states");
    auto_declare<double>("target_timeout", target_timeout_);

    auto_declare<double>("target_filter.alpha", target_filter_alpha_);
    auto_declare<double>("target_filter.fast_alpha", target_filter_fast_alpha_);
    auto_declare<double>("target_filter.fast_threshold", target_filter_fast_threshold_);
    auto_declare<double>("target_velocity_feedforward.gain", target_velocity_ff_gain_);
    auto_declare<double>("target_velocity_feedforward.alpha", target_velocity_ff_alpha_);
    auto_declare<double>("target_velocity_feedforward.max", target_velocity_ff_max_);

    auto_declare<double>("pd_velocity.velocity_kp", velocity_kp_);
    auto_declare<double>("pd_velocity.velocity_ki", velocity_ki_);
    auto_declare<double>("pd_velocity.integral_max", integral_max_);
    auto_declare<double>("pd_velocity.velocity_kd", velocity_kd_);
    auto_declare<double>("pd_velocity.derivative_alpha", derivative_alpha_);
    auto_declare<double>("pd_velocity.velocity_deadband", velocity_deadband_);
    auto_declare<double>("pd_velocity.velocity_max", velocity_max_);
    auto_declare<double>("pd_velocity.velocity_slew_rate", velocity_slew_rate_);

    auto_declare<double>("pd_velocity.thumb_velocity_deadband", thumb_velocity_deadband_);
    auto_declare<double>("pd_velocity.thumb_velocity_min", thumb_velocity_min_);
    auto_declare<double>("pd_velocity.thumb_velocity_kp_scale", thumb_velocity_kp_scale_);
    auto_declare<double>("pd_velocity.thumb_velocity_brake_zone", thumb_velocity_brake_zone_);
    auto_declare<double>("pd_velocity.thumb_velocity_brake_max", thumb_velocity_brake_max_);

    auto_declare<double>("pd_velocity.ring_velocity_deadband", ring_velocity_deadband_);
    auto_declare<double>("pd_velocity.ring_velocity_min", ring_velocity_min_);
    auto_declare<double>("pd_velocity.ring_velocity_kp_scale", ring_velocity_kp_scale_);
    auto_declare<double>(
      "pd_velocity.four_finger_extension_velocity_scale",
      four_finger_extension_velocity_scale_);
    auto_declare<std::vector<int64_t>>(
      "pd_velocity.four_finger_extension_joints",
      four_finger_extension_joints_);

    auto_declare<std::vector<double>>("joint_lower_limits", {});
    auto_declare<std::vector<double>>("joint_upper_limits", {});
    auto_declare<std::vector<double>>("feedback.position_scales", {});
    auto_declare<std::vector<double>>("feedback.position_offsets", {});
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", exc.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Revo2PidController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::InterfaceConfiguration
Revo2PidController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::CallbackReturn Revo2PidController::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    if (joints_.size() != kJointCount) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "revo2_pid_controller requires exactly 6 joints, got %zu.",
        joints_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    target_topic_ = get_node()->get_parameter("target_topic").as_string();
    target_timeout_ = std::max(0.0, get_node()->get_parameter("target_timeout").as_double());

    target_filter_alpha_ = clamp(
      get_node()->get_parameter("target_filter.alpha").as_double(), 0.0, 1.0);
    target_filter_fast_alpha_ = clamp(
      get_node()->get_parameter("target_filter.fast_alpha").as_double(), 0.0, 1.0);
    target_filter_fast_alpha_ = std::max(target_filter_alpha_, target_filter_fast_alpha_);
    target_filter_fast_threshold_ = std::max(
      0.0, get_node()->get_parameter("target_filter.fast_threshold").as_double());
    target_velocity_ff_gain_ = std::max(
      0.0, get_node()->get_parameter("target_velocity_feedforward.gain").as_double());
    target_velocity_ff_alpha_ = clamp(
      get_node()->get_parameter("target_velocity_feedforward.alpha").as_double(), 0.0, 1.0);
    target_velocity_ff_max_ = std::max(
      0.0, get_node()->get_parameter("target_velocity_feedforward.max").as_double());

    velocity_kp_ = get_node()->get_parameter("pd_velocity.velocity_kp").as_double();
    velocity_ki_ = get_node()->get_parameter("pd_velocity.velocity_ki").as_double();
    integral_max_ = get_node()->get_parameter("pd_velocity.integral_max").as_double();
    if (!std::isfinite(velocity_ki_) || velocity_ki_ < 0.0 ||
      !std::isfinite(integral_max_) || integral_max_ < 0.0)
    {
      throw std::runtime_error("velocity_ki and integral_max must be finite and nonnegative.");
    }
    velocity_kd_ = get_node()->get_parameter("pd_velocity.velocity_kd").as_double();
    derivative_alpha_ = clamp(
      get_node()->get_parameter("pd_velocity.derivative_alpha").as_double(), 0.0, 1.0);
    velocity_deadband_ = std::max(
      0.0, get_node()->get_parameter("pd_velocity.velocity_deadband").as_double());
    velocity_max_ = std::max(
      0.0, get_node()->get_parameter("pd_velocity.velocity_max").as_double());
    velocity_slew_rate_ = std::max(
      0.0, get_node()->get_parameter("pd_velocity.velocity_slew_rate").as_double());
    velocity_zero_epsilon_ = std::min(0.002, std::max(1e-6, velocity_max_ * 1e-3));

    thumb_velocity_deadband_ = std::max(
      0.0, get_node()->get_parameter("pd_velocity.thumb_velocity_deadband").as_double());
    thumb_velocity_min_ = std::max(
      0.0, get_node()->get_parameter("pd_velocity.thumb_velocity_min").as_double());
    thumb_velocity_kp_scale_ = std::max(
      0.0, get_node()->get_parameter("pd_velocity.thumb_velocity_kp_scale").as_double());
    thumb_velocity_brake_zone_ = std::max(
      0.0, get_node()->get_parameter("pd_velocity.thumb_velocity_brake_zone").as_double());
    thumb_velocity_brake_max_ = std::max(
      0.0, get_node()->get_parameter("pd_velocity.thumb_velocity_brake_max").as_double());

    ring_velocity_deadband_ = std::max(
      0.0, get_node()->get_parameter("pd_velocity.ring_velocity_deadband").as_double());
    ring_velocity_min_ = std::max(
      0.0, get_node()->get_parameter("pd_velocity.ring_velocity_min").as_double());
    ring_velocity_kp_scale_ = std::max(
      0.0, get_node()->get_parameter("pd_velocity.ring_velocity_kp_scale").as_double());

    four_finger_extension_velocity_scale_ = std::max(
      0.0,
      get_node()->get_parameter(
        "pd_velocity.four_finger_extension_velocity_scale").as_double());
    four_finger_extension_joints_ =
      get_node()->get_parameter("pd_velocity.four_finger_extension_joints").as_integer_array();
    for (const auto index : four_finger_extension_joints_) {
      if (index < 0 || index >= static_cast<int64_t>(kJointCount)) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "pd_velocity.four_finger_extension_joints contains invalid index: %ld",
          index);
        return controller_interface::CallbackReturn::ERROR;
      }
    }

    joint_lower_limits_ = six_array_from_vector(
      get_node()->get_parameter("joint_lower_limits").as_double_array(),
      joint_lower_limits_,
      "joint_lower_limits");
    joint_upper_limits_ = six_array_from_vector(
      get_node()->get_parameter("joint_upper_limits").as_double_array(),
      joint_upper_limits_,
      "joint_upper_limits");
    feedback_position_scales_ = six_array_from_vector(
      get_node()->get_parameter("feedback.position_scales").as_double_array(),
      feedback_position_scales_,
      "feedback.position_scales");
    feedback_position_offsets_ = six_array_from_vector(
      get_node()->get_parameter("feedback.position_offsets").as_double_array(),
      feedback_position_offsets_,
      "feedback.position_offsets");

    target_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      target_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        on_target(msg);
      });
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure Revo2PidController: %s", exc.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Revo2PidController configured: target=%s, joints=%zu, command=velocity rad/s, target_velocity_ff gain=%.3f alpha=%.3f max=%.3f",
    target_topic_.c_str(),
    joints_.size(),
    target_velocity_ff_gain_,
    target_velocity_ff_alpha_,
    target_velocity_ff_max_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Revo2PidController::on_activate(
  const rclcpp_lifecycle::State &)
{
  reset_runtime_state();
  zero_commands();
  RCLCPP_INFO(
    get_node()->get_logger(), "Integral control: Ki=%.3f, I limit=%.3f rad/s",
    velocity_ki_, integral_max_);
  RCLCPP_INFO(get_node()->get_logger(), "Revo2PidController activated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Revo2PidController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  zero_commands();
  RCLCPP_INFO(get_node()->get_logger(), "Revo2PidController deactivated; velocity commands zeroed.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Revo2PidController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  JointArray target{};
  rclcpp::Time target_time{0, 0, RCL_ROS_TIME};
  bool target_ready = false;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target = target_position_;
    target_time = target_time_;
    target_ready = target_ready_;
  }

  if (!target_ready) {
    zero_commands();
    warn_throttled(time, "Waiting for Revo2 target JointState.");
    return controller_interface::return_type::OK;
  }

  if (target_timeout_ > 0.0 && (time - target_time).seconds() > target_timeout_) {
    zero_commands();
    warn_throttled(time, "Revo2 target timeout, velocity commands zeroed.");
    return controller_interface::return_type::OK;
  }

  JointArray actual_position{};
  for (std::size_t i = 0; i < kJointCount; ++i) {
    actual_position[i] =
      state_interfaces_[i].get_value() * feedback_position_scales_[i] +
      feedback_position_offsets_[i];
    if (!std::isfinite(actual_position[i])) {
      zero_commands();
      warn_throttled(time, "Non-finite Revo2 position feedback, velocity commands zeroed.");
      return controller_interface::return_type::ERROR;
    }
  }

  JointArray previous_filtered_target = filtered_target_;
  if (!filter_initialized_) {
    filtered_target_ = target;
    previous_filtered_target = filtered_target_;
    filtered_target_velocity_.fill(0.0);
    filter_initialized_ = true;
  } else {
    previous_filtered_target = filtered_target_;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const double delta = std::abs(target[i] - filtered_target_[i]);
      const double alpha =
        (target_filter_fast_threshold_ > 0.0 && delta >= target_filter_fast_threshold_) ?
        target_filter_fast_alpha_ :
        target_filter_alpha_;
      filtered_target_[i] = (1.0 - alpha) * filtered_target_[i] + alpha * target[i];
    }
  }

  double dt = period.seconds();
  if ((!std::isfinite(dt) || dt <= 1e-6) && last_pd_time_.nanoseconds() != 0) {
    dt = std::max((time - last_pd_time_).seconds(), 1e-3);
  }
  if (!std::isfinite(dt) || dt <= 1e-6) {
    dt = 0.01;
  }

  JointArray error{};
  for (std::size_t i = 0; i < kJointCount; ++i) {
    error[i] = filtered_target_[i] - actual_position[i];
    const double raw_derivative = (error[i] - last_error_[i]) / dt;
    filtered_derivative_[i] =
      (1.0 - derivative_alpha_) * filtered_derivative_[i] +
      derivative_alpha_ * raw_derivative;

    const double raw_target_velocity = (filtered_target_[i] - previous_filtered_target[i]) / dt;
    filtered_target_velocity_[i] =
      (1.0 - target_velocity_ff_alpha_) * filtered_target_velocity_[i] +
      target_velocity_ff_alpha_ * raw_target_velocity;

    target_velocity_[i] = velocity_kp_ * error[i] + velocity_kd_ * filtered_derivative_[i];
  }

  target_velocity_[0] *= thumb_velocity_kp_scale_;
  target_velocity_[1] *= thumb_velocity_kp_scale_;
  target_velocity_[4] *= ring_velocity_kp_scale_;

  JointArray previous_integral_velocity{};
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double deadband = deadband_for_joint(i);
    const bool at_deadband = std::abs(error[i]) <= deadband;
    const bool reversed_error = error[i] * last_error_[i] < 0.0;
    const bool outward_at_limit =
      (actual_position[i] <= joint_lower_limits_[i] && integral_velocity_[i] < 0.0) ||
      (actual_position[i] >= joint_upper_limits_[i] && integral_velocity_[i] > 0.0);
    if (velocity_ki_ == 0.0 || integral_max_ == 0.0 || at_deadband ||
      reversed_error || outward_at_limit)
    {
      integral_velocity_[i] = 0.0;
    }
    previous_integral_velocity[i] = integral_velocity_[i];
    const bool outward_error =
      (actual_position[i] <= joint_lower_limits_[i] && error[i] < 0.0) ||
      (actual_position[i] >= joint_upper_limits_[i] && error[i] > 0.0);
    if (velocity_ki_ > 0.0 && integral_max_ > 0.0 && !at_deadband && !outward_error) {
      // Store the I contribution in rad/s, independently for each joint.
      integral_velocity_[i] = clamp(
        integral_velocity_[i] + velocity_ki_ * error[i] * dt,
        -integral_max_, integral_max_);
    }
    target_velocity_[i] += integral_velocity_[i];
    if (std::abs(error[i]) <= deadband) {
      target_velocity_[i] = 0.0;
    }

    const double feedforward_velocity = target_velocity_ff_max_ > 0.0 ?
      clamp(
        target_velocity_ff_gain_ * filtered_target_velocity_[i],
        -target_velocity_ff_max_,
        target_velocity_ff_max_) :
      0.0;
    target_velocity_[i] += feedforward_velocity;
  }

  for (const auto joint_index : four_finger_extension_joints_) {
    const std::size_t i = static_cast<std::size_t>(joint_index);
    if (target_velocity_[i] < 0.0) {
      target_velocity_[i] *= four_finger_extension_velocity_scale_;
    }
  }

  const JointArray requested_velocity = target_velocity_;
  for (auto & velocity : target_velocity_) {
    velocity = clamp(velocity, -velocity_max_, velocity_max_);
  }

  if (thumb_velocity_min_ > 0.0) {
    const double thumb_min = std::min(thumb_velocity_min_, velocity_max_);
    for (std::size_t i = 0; i < 2; ++i) {
      if (std::abs(error[i]) > thumb_velocity_deadband_ &&
          std::abs(target_velocity_[i]) < thumb_min)
      {
        target_velocity_[i] = std::copysign(thumb_min, error[i]);
      }
    }
  }

  if (ring_velocity_min_ > 0.0 &&
      std::abs(error[4]) > ring_velocity_deadband_ &&
      std::abs(target_velocity_[4]) < ring_velocity_min_)
  {
    target_velocity_[4] = std::copysign(std::min(ring_velocity_min_, velocity_max_), error[4]);
  }

  if (thumb_velocity_brake_zone_ > 0.0 && thumb_velocity_brake_max_ > 0.0) {
    const double brake_limit = std::min(thumb_velocity_brake_max_, velocity_max_);
    for (std::size_t i = 0; i < 2; ++i) {
      const double error_abs = std::abs(error[i]);
      if (error_abs < thumb_velocity_brake_zone_) {
        const double brake_cap = brake_limit * error_abs / thumb_velocity_brake_zone_;
        target_velocity_[i] = clamp(target_velocity_[i], -brake_cap, brake_cap);
      }
    }
  }

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const bool zero_target = std::abs(target_velocity_[i]) < velocity_zero_epsilon_;
    const bool reversing = (target_velocity_[i] * command_velocity_[i]) < 0.0;
    if (zero_target || reversing) {
      command_velocity_[i] = 0.0;
    }

    const double delta = clamp(
      target_velocity_[i] - command_velocity_[i],
      -velocity_slew_rate_,
      velocity_slew_rate_);
    command_velocity_[i] += delta;
    const double limited_velocity = command_velocity_[i];
    if (std::abs(command_velocity_[i]) < velocity_zero_epsilon_) {
      command_velocity_[i] = 0.0;
    }
    // Reject integration that pushes farther into velocity, brake or slew saturation.
    const double integral_delta = integral_velocity_[i] - previous_integral_velocity[i];
    if ((requested_velocity[i] - limited_velocity) * integral_delta > 1e-12) {
      integral_velocity_[i] = previous_integral_velocity[i];
    }
    command_interfaces_[i].set_value(command_velocity_[i]);
    last_error_[i] = error[i];
  }
  last_pd_time_ = time;

  return controller_interface::return_type::OK;
}

void Revo2PidController::on_target(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  JointArray parsed{};
  if (!parse_target(*msg, parsed)) {
    return;
  }

  std::lock_guard<std::mutex> lock(target_mutex_);
  target_position_ = parsed;
  target_time_ = get_node()->get_clock()->now();
  if (!target_ready_) {
    RCLCPP_INFO(get_node()->get_logger(), "Received first Revo2 target JointState.");
  }
  target_ready_ = true;
}

bool Revo2PidController::parse_target(
  const sensor_msgs::msg::JointState & msg,
  JointArray & target) const
{
  if (msg.name.empty()) {
    if (msg.position.size() < kJointCount) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Target JointState has no names and fewer than 6 positions.");
      return false;
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      target[i] = clamp_target(i, msg.position[i]);
    }
    return true;
  }

  std::unordered_map<std::string, std::size_t> name_to_index;
  for (std::size_t i = 0; i < msg.name.size(); ++i) {
    name_to_index[msg.name[i]] = i;
  }

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto full_it = name_to_index.find(joints_[i]);
    const auto short_it = name_to_index.find(short_joint_name(joints_[i]));
    const auto it = full_it != name_to_index.end() ? full_it : short_it;
    if (it == name_to_index.end() || it->second >= msg.position.size()) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Target JointState missing joint '%s'.",
        joints_[i].c_str());
      return false;
    }
    target[i] = clamp_target(i, msg.position[it->second]);
  }

  return true;
}

void Revo2PidController::zero_commands()
{
  integral_velocity_.fill(0.0);
  command_velocity_.fill(0.0);
  target_velocity_.fill(0.0);
  for (auto & interface : command_interfaces_) {
    interface.set_value(0.0);
  }
}

void Revo2PidController::reset_runtime_state()
{
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_position_.fill(0.0);
    target_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    target_ready_ = false;
  }
  filtered_target_.fill(0.0);
  last_error_.fill(0.0);
  filtered_derivative_.fill(0.0);
  filtered_target_velocity_.fill(0.0);
  integral_velocity_.fill(0.0);
  target_velocity_.fill(0.0);
  command_velocity_.fill(0.0);
  filter_initialized_ = false;
  last_pd_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_warn_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void Revo2PidController::warn_throttled(
  const rclcpp::Time & time,
  const std::string & message)
{
  if (last_warn_time_.nanoseconds() == 0 || (time - last_warn_time_).seconds() > 1.0) {
    last_warn_time_ = time;
    RCLCPP_WARN(get_node()->get_logger(), "%s", message.c_str());
  }
}

double Revo2PidController::deadband_for_joint(std::size_t index) const
{
  if (index < 2) {
    return thumb_velocity_deadband_;
  }
  return index == 4 ? ring_velocity_deadband_ : velocity_deadband_;
}

double Revo2PidController::clamp_target(std::size_t index, double value) const
{
  if (!std::isfinite(value)) {
    return joint_lower_limits_[index];
  }
  return clamp(value, joint_lower_limits_[index], joint_upper_limits_[index]);
}

double Revo2PidController::clamp(double value, double low, double high)
{
  if (low > high) {
    return value;
  }
  return std::min(std::max(value, low), high);
}

}  // namespace revo2_driver

PLUGINLIB_EXPORT_CLASS(
  revo2_driver::Revo2PidController,
  controller_interface::ControllerInterface)
