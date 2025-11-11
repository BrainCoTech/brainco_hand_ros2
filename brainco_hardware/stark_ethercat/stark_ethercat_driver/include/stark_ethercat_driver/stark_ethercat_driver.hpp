// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#ifndef ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
#define ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "stark_ethercat_driver/visibility_control.h"
#include "stark_ethercat_interface/ec_master.hpp"
#include "stark_ethercat_interface/ec_slave.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace stark_ethercat_driver
{

class EthercatDriver : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EthercatDriver)

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<std::unordered_map<std::string, std::string>> getEcModuleParam(
    std::string & urdf, std::string component_name, std::string component_type);

  std::vector<std::shared_ptr<stark_ethercat_interface::EcSlave>> ec_modules_;
  std::vector<std::unordered_map<std::string, std::string>> ec_module_parameters_;

  std::vector<std::vector<double>> hw_joint_commands_;
  std::vector<std::vector<double>> hw_sensor_commands_;
  std::vector<std::vector<double>> hw_gpio_commands_;
  std::vector<std::vector<double>> hw_joint_states_;
  std::vector<std::vector<double>> hw_sensor_states_;
  std::vector<std::vector<double>> hw_gpio_states_;

  // Aggregated joint buffers to support single EtherCAT module managing multiple joints
  std::vector<double> agg_joint_commands_;
  std::vector<double> agg_joint_states_;
  // Offsets into aggregated buffers for each joint
  std::vector<size_t> joint_command_offsets_;
  std::vector<size_t> joint_state_offsets_;

  struct JointInterfaceIndexMap
  {
    int position_command_index_in_hw;
    int position_state_index_in_hw;
    int velocity_state_index_in_hw;
    int effort_state_index_in_hw;
  };
  std::vector<JointInterfaceIndexMap> joint_interface_index_map_;

  pluginlib::ClassLoader<stark_ethercat_interface::EcSlave> ec_loader_{
    "stark_ethercat_interface", "stark_ethercat_interface::EcSlave"};

  int control_frequency_;
  stark_ethercat_interface::EcMaster master_;
  std::mutex ec_mutex_;
  bool activated_;

  // runtime health monitor
  std::thread health_thread_;
  std::atomic<bool> monitor_running_{false};
  std::atomic<bool> deactivate_pending_{false};
};
}  // namespace stark_ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
