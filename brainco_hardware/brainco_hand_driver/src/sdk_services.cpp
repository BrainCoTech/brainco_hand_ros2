// Copyright (c) 2025 BrainCo
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "brainco_hand_driver/brainco_hand_hardware.hpp"

#include <exception>
#include <functional>

#include "stark-sdk.h"

namespace revo2_driver
{

BraincoHandHardware::~BraincoHandHardware()
{
  stop_sdk_services();
  const std::lock_guard<std::mutex> lock(io_mutex_);
  close_connection();
}

auto BraincoHandHardware::on_error(const rclcpp_lifecycle::State & previous_state)
-> hardware_interface::CallbackReturn
{
  return on_cleanup(previous_state);
}

void BraincoHandHardware::start_sdk_services()
{
  // Inherit the controller manager's namespace (__ns) while keeping a unique
  // node name even if a global __node remapping is present.
  const auto name = "revo2_sdk_" + std::to_string(config_.transport.slave_id);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + name});
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  sdk_service_node_ = std::make_shared<rclcpp::Node>(name, options);
  enable_touch_service_ = sdk_service_node_->create_service<srv::TouchSensorCommand>(
    "touch/enable",
    [this](
      const std::shared_ptr<srv::TouchSensorCommand::Request> request,
      std::shared_ptr<srv::TouchSensorCommand::Response> response)
    {
      handle_touch_command(false, request, response);
    });
  calibrate_touch_service_ = sdk_service_node_->create_service<srv::TouchSensorCommand>(
    "touch/calibrate",
    [this](
      const std::shared_ptr<srv::TouchSensorCommand::Request> request,
      std::shared_ptr<srv::TouchSensorCommand::Response> response)
    {
      handle_touch_command(true, request, response);
    });
  sdk_service_executor_ =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  sdk_service_executor_->add_node(sdk_service_node_);
  const auto executor = sdk_service_executor_;
  sdk_services_running_ = true;
  sdk_service_thread_ = std::thread(
    [this, executor]() {
      try {
        while (sdk_services_running_ && rclcpp::ok(
          sdk_service_node_->get_node_options().context()))
        {
          executor->spin_once(std::chrono::milliseconds(50));
        }
      } catch (const std::exception & exception) {
        BRAINCO_HAND_LOG_ERROR("SDK service executor stopped: %s", exception.what());
      }
    });
  BRAINCO_HAND_LOG_INFO(
    "SDK touch services: %s, %s (manual calls only); SDK file logging=%s",
    enable_touch_service_->get_service_name(), calibrate_touch_service_->get_service_name(),
    config_.transport.sdk_file_logging ? "true" : "false");
}

void BraincoHandHardware::stop_sdk_services()
{
  // Never call while holding io_mutex_: an in-flight callback may need it.
  sdk_services_running_ = false;
  if (sdk_service_executor_) {
    sdk_service_executor_->cancel();
  }
  if (sdk_service_thread_.joinable()) {
    sdk_service_thread_.join();
  }
  enable_touch_service_.reset();
  calibrate_touch_service_.reset();
  sdk_service_executor_.reset();
  sdk_service_node_.reset();
}

void BraincoHandHardware::handle_touch_command(
  bool calibrate,
  const std::shared_ptr<srv::TouchSensorCommand::Request> & request,
  const std::shared_ptr<srv::TouchSensorCommand::Response> & response)
{
  response->success = false;
  response->sdk_called = false;
  response->sdk_status = -1;
  const auto mask = request->sensor_mask;
  if (mask > 0x1F || (calibrate && mask == 0)) {
    response->message = calibrate ? "sensor_mask must be 1..31 for calibration" :
      "sensor_mask must be 0..31 for enable";
    return;
  }

  const std::lock_guard<std::mutex> lock(io_mutex_);
  if (!is_active_ || !api_.is_open()) {
    response->message = "Hardware must be active and connected";
    return;
  }
  // These services are verified for the same Revo2 capacitive-touch API used
  // by this driver's five-finger feedback. Other touch types need separate APIs.
  if (!connected_hardware_type_ ||
    *connected_hardware_type_ != STARK_HARDWARE_TYPE_REVO2_TOUCH)
  {
    response->message = "Device is not a verified Revo2 capacitive-touch hand";
    return;
  }

  try {
    response->sdk_called = true;
    response->sdk_status = calibrate ?
      api_.calibrate_touch_sensors(config_.transport.slave_id, mask) :
      api_.enable_touch_sensors(config_.transport.slave_id, mask);
    response->success = response->sdk_status == 0;
    response->message = response->success ?
      "Transport operation succeeded; sensor readiness, calibration completion and accuracy are not confirmed"
      :
      "SDK transport operation failed; inspect SDK logs and device connection";
    BRAINCO_HAND_LOG_INFO(
      "Touch %s mask=%u SDK status=%d", calibrate ? "calibrate" : "enable",
      static_cast<unsigned>(mask), response->sdk_status);
  } catch (const std::exception & exception) {
    response->message = std::string("SDK operation raised an exception: ") + exception.what();
  }
}

}  // namespace revo2_driver
