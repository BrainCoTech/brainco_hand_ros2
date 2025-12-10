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

#include "stark_ethercat_driver/stark_ethercat_driver.hpp"

#include <tinyxml2.h>
#include <regex>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace stark_ethercat_driver
{
CallbackReturn EthercatDriver::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "build time: %s - %s", __DATE__, __TIME__);

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  hw_joint_states_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++)
  {
    hw_joint_states_[j].resize(
      info_.joints[j].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_states_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++)
  {
    hw_sensor_states_[s].resize(
      info_.sensors[s].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_states_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++)
  {
    hw_gpio_states_[g].resize(
      info_.gpios[g].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  }
  hw_joint_commands_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++)
  {
    hw_joint_commands_[j].resize(
      info_.joints[j].command_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_commands_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++)
  {
    hw_sensor_commands_[s].resize(
      info_.sensors[s].command_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_commands_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++)
  {
    hw_gpio_commands_[g].resize(
      info_.gpios[g].command_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  }

  // Build aggregated joint interfaces: flatten all joint commands and states
  size_t total_joint_command_size = 0;
  size_t total_joint_state_size = 0;
  joint_command_offsets_.assign(info_.joints.size(), 0);
  joint_state_offsets_.assign(info_.joints.size(), 0);
  for (uint j = 0; j < info_.joints.size(); j++)
  {
    joint_command_offsets_[j] = total_joint_command_size;
    joint_state_offsets_[j] = total_joint_state_size;
    total_joint_command_size += info_.joints[j].command_interfaces.size();
    total_joint_state_size += info_.joints[j].state_interfaces.size();
  }
  agg_joint_commands_.assign(total_joint_command_size, std::numeric_limits<double>::quiet_NaN());
  agg_joint_states_.assign(total_joint_state_size, std::numeric_limits<double>::quiet_NaN());

  for (uint j = 0; j < info_.joints.size(); j++)
  {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "joints");
    // check all joints for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.joints[j].name, "joint");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());

    for (auto i = 0ul; i < module_params.size(); i++)
    {
      for (auto k = 0ul; k < info_.joints[j].state_interfaces.size(); k++)
      {
        module_params[i]["state_interface/" + info_.joints[j].state_interfaces[k].name] =
          std::to_string(k);
      }
      for (auto k = 0ul; k < info_.joints[j].command_interfaces.size(); k++)
      {
        module_params[i]["command_interface/" + info_.joints[j].command_interfaces[k].name] =
          std::to_string(k);
      }
      try
      {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(module_params[i], &hw_joint_states_[j], &hw_joint_commands_[j]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"), "Setup of Joint module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        ec_modules_.push_back(module);
      }
      catch (pluginlib::PluginlibException & ex)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.joints[j].name.c_str(), ex.what());
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Got %li modules", ec_modules_.size());
  }
  for (uint g = 0; g < info_.gpios.size(); g++)
  {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "gpios");
    // check all gpios for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.gpios[g].name, "gpio");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++)
    {
      for (auto k = 0ul; k < info_.gpios[g].state_interfaces.size(); k++)
      {
        module_params[i]["state_interface/" + info_.gpios[g].state_interfaces[k].name] =
          std::to_string(k);
      }
      for (auto k = 0ul; k < info_.gpios[g].command_interfaces.size(); k++)
      {
        module_params[i]["command_interface/" + info_.gpios[g].command_interfaces[k].name] =
          std::to_string(k);
      }
      try
      {
        // Ensure only a single module instance is created for the whole device
        if (!ec_modules_.empty())
        {
          RCLCPP_INFO(
            rclcpp::get_logger("EthercatDriver"),
            "Skipping additional EC module for gpio '%s' as one is already created.",
            info_.gpios[g].name.c_str());
          continue;
        }

        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        // Bind the module to aggregated joint buffers so it can access all joints at once
        if (!module->setupSlave(module_params[i], &agg_joint_states_, &agg_joint_commands_))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"), "Setup of aggregated GPIO module %li FAILED.",
            i + 1);
          return CallbackReturn::ERROR;
        }
        ec_modules_.push_back(module);
      }
      catch (pluginlib::PluginlibException & ex)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.gpios[g].name.c_str(), ex.what());
      }
    }
  }
  for (uint s = 0; s < info_.sensors.size(); s++)
  {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "sensors");
    // check all sensors for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.sensors[s].name, "sensor");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++)
    {
      for (auto k = 0ul; k < info_.sensors[s].state_interfaces.size(); k++)
      {
        module_params[i]["state_interface/" + info_.sensors[s].state_interfaces[k].name] =
          std::to_string(k);
      }
      for (auto k = 0ul; k < info_.sensors[s].command_interfaces.size(); k++)
      {
        module_params[i]["command_interface/" + info_.sensors[s].command_interfaces[k].name] =
          std::to_string(k);
      }
      try
      {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(module_params[i], &hw_sensor_states_[s], &hw_sensor_commands_[s]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"), "Setup of Sensor module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        ec_modules_.push_back(module);
      }
      catch (pluginlib::PluginlibException & ex)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.sensors[s].name.c_str(), ex.what());
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Got %li modules", ec_modules_.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EthercatDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // export joint state interface
  for (uint j = 0; j < info_.joints.size(); j++)
  {
    for (uint i = 0; i < info_.joints[j].state_interfaces.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[j].name, info_.joints[j].state_interfaces[i].name, &hw_joint_states_[j][i]));
    }
  }
  // export sensor state interface
  for (uint s = 0; s < info_.sensors.size(); s++)
  {
    for (uint i = 0; i < info_.sensors[s].state_interfaces.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[s].name, info_.sensors[s].state_interfaces[i].name,
        &hw_sensor_states_[s][i]));
    }
  }
  // export gpio state interface
  for (uint g = 0; g < info_.gpios.size(); g++)
  {
    for (uint i = 0; i < info_.gpios[g].state_interfaces.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[g].name, info_.gpios[g].state_interfaces[i].name, &hw_gpio_states_[g][i]));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EthercatDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // export joint command interface
  std::vector<double> test;
  for (uint j = 0; j < info_.joints.size(); j++)
  {
    for (uint i = 0; i < info_.joints[j].command_interfaces.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[j].name, info_.joints[j].command_interfaces[i].name,
        &hw_joint_commands_[j][i]));
    }
  }
  // export sensor command interface
  for (uint s = 0; s < info_.sensors.size(); s++)
  {
    for (uint i = 0; i < info_.sensors[s].command_interfaces.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.sensors[s].name, info_.sensors[s].command_interfaces[i].name,
        &hw_sensor_commands_[s][i]));
    }
  }
  // export gpio command interface
  for (uint g = 0; g < info_.gpios.size(); g++)
  {
    for (uint i = 0; i < info_.gpios[g].command_interfaces.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[g].name, info_.gpios[g].command_interfaces[i].name, &hw_gpio_commands_[g][i]));
    }
  }
  return command_interfaces;
}

CallbackReturn EthercatDriver::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  if (activated_)
  {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Double on_activate()");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Starting ...please wait...");
  if (info_.hardware_parameters.find("control_frequency") == info_.hardware_parameters.end())
  {
    control_frequency_ = 100;
  }
  else
  {
    control_frequency_ = std::stod(info_.hardware_parameters["control_frequency"]);
  }

  if (control_frequency_ < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Invalid control frequency!");
    return CallbackReturn::ERROR;
  }

  // start EC and wait until state operative

  master_.setCtrlFrequency(control_frequency_);

  for (auto i = 0ul; i < ec_modules_.size(); i++)
  {
    master_.addSlave(
      std::stod(ec_module_parameters_[i]["alias"]), std::stod(ec_module_parameters_[i]["position"]),
      ec_modules_[i].get());
  }

  // configure SDO
  for (auto i = 0ul; i < ec_modules_.size(); i++)
  {
    for (auto & sdo : ec_modules_[i]->sdo_config)
    {
      uint32_t abort_code;
      int ret =
        master_.configSlaveSdo(std::stod(ec_module_parameters_[i]["position"]), sdo, &abort_code);
      RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"),
        "configSlaveSdo index: %04X, sub_index: %02X, data: %d, ret: %d", sdo.index, sdo.sub_index,
        sdo.data, ret);
      if (ret)
      {
        RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"),
          "Failed to download config SDO for module at position %s with Error: %d",
          ec_module_parameters_[i]["position"].c_str(), abort_code);
      }
    }
  }

  if (!master_.activate())
  {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "Activate EcMaster failed");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Activated EcMaster!");

  // start after one second
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  bool running = true;
  const double startup_timeout_sec =
    (info_.hardware_parameters.count("startup_timeout_sec")
       ? std::stod(info_.hardware_parameters.at("startup_"
                                                "timeout_sec"))
       : 5.0);
  auto start_tp = std::chrono::steady_clock::now();
  while (running)
  {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    // update EtherCAT bus

    master_.update();
    // RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "updated!");

    // check if operational
    bool isAllInit = true;
    for (auto & module : ec_modules_)
    {
      isAllInit = isAllInit && module->initialized();
    }
    if (isAllInit)
    {
      running = false;
    }
    // XXX 超时保护：防止无从站或未达 OP 时卡住
    auto now_tp = std::chrono::steady_clock::now();
    double elapsed =
      std::chrono::duration_cast<std::chrono::duration<double>>(now_tp - start_tp).count();
    if (elapsed > startup_timeout_sec)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("EthercatDriver"), "Startup timeout %.2fs. Slaves not OP.",
        startup_timeout_sec);
      return CallbackReturn::ERROR;
    }
    // calculate next shot. carry over nanoseconds into microseconds.
    t.tv_nsec += master_.getInterval();
    while (t.tv_nsec >= 1000000000)
    {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "System Successfully started!");

  activated_ = true;

  // 初始化命令=当前状态，以避免首次 write 发送未定义/零指令引发“抖动”
  // 进行一次同步读以填充聚合状态缓冲
  master_.readData();
  for (uint j = 0; j < info_.joints.size(); j++)
  {
    size_t state_base = joint_state_offsets_[j];
    size_t cmd_base = joint_command_offsets_[j];
    for (size_t i = 0; i < info_.joints[j].state_interfaces.size(); i++)
    {
      if (state_base + i < agg_joint_states_.size())
      {
        hw_joint_states_[j][i] = agg_joint_states_[state_base + i];
      }
    }
    // 将每个关节的命令初始化为其当前状态（同接口序）
    for (size_t i = 0; i < info_.joints[j].command_interfaces.size(); i++)
    {
      double init_cmd = std::numeric_limits<double>::quiet_NaN();
      if (i < info_.joints[j].state_interfaces.size())
      {
        init_cmd = hw_joint_states_[j][i];
      }
      hw_joint_commands_[j][i] = init_cmd;
      if (cmd_base + i < agg_joint_commands_.size())
      {
        // 同步聚合命令缓冲，保证插件读取到的初值与状态一致
        agg_joint_commands_[cmd_base + i] = init_cmd;
      }
    }
  }

  // 启动后台健康监测线程：检测链路和从站在线/OP状态，异常则主动降级释放
  monitor_running_ = true;
  health_thread_ = std::thread(
    [this]()
    {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      while (monitor_running_)
      {
        {
          const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
          if (lock.owns_lock() && activated_)
          {
            bool unhealthy =
              (!master_.isLinkUp() || master_.slavesResponding() == 0 ||
               !master_.anySlaveOperational());
            if (unhealthy)
            {
              RCLCPP_ERROR_THROTTLE(
                rclcpp::get_logger("EthercatDriver"), steady_clock, 2000,
                "EtherCAT offline/not OP. Deactivating master.");
              deactivate_pending_ = true;
            }
          }
        }
        if (deactivate_pending_)
        {
          const std::lock_guard<std::mutex> lock(ec_mutex_);
          master_.deactivate();
          activated_ = false;
          deactivate_pending_ = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    });

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;
  monitor_running_ = false;
  if (health_thread_.joinable())
  {
    health_thread_.join();
  }
  master_.deactivate();

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Stopping ...please wait...");

  // stop EC and disconnect
  master_.stop();

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatDriver::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (lock.owns_lock() && activated_)
  {
    // 运行时链路/从站健壮性检测
    if (!master_.isLinkUp() || master_.slavesResponding() == 0 || !master_.anySlaveOperational())
    {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("EthercatDriver"), steady_clock, 2000,
        "EtherCAT offline or not operational (link=%d, slaves=%u, anyOP=%d).", master_.isLinkUp(),
        master_.slavesResponding(), master_.anySlaveOperational());
      deactivate_pending_ = true;
      return hardware_interface::return_type::ERROR;
    }
    master_.readData();
    // Copy aggregated states into per-joint state buffers for ros2_control exposure
    for (uint j = 0; j < info_.joints.size(); j++)
    {
      size_t state_base = joint_state_offsets_[j];
      for (size_t i = 0; i < info_.joints[j].state_interfaces.size(); i++)
      {
        if (state_base + i < agg_joint_states_.size())
        {
          hw_joint_states_[j][i] = agg_joint_states_[state_base + i];
        }
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EthercatDriver::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (lock.owns_lock() && activated_)
  {
    // 运行时链路/从站健壮性检测
    if (!master_.isLinkUp() || master_.slavesResponding() == 0 || !master_.anySlaveOperational())
    {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("EthercatDriver"), steady_clock, 2000,
        "EtherCAT offline or not operational (link=%d, slaves=%u, anyOP=%d).", master_.isLinkUp(),
        master_.slavesResponding(), master_.anySlaveOperational());
      deactivate_pending_ = true;
      return hardware_interface::return_type::ERROR;
    }
    // 将每关节命令拷贝到聚合命令，若命令是 NaN/非有限，则回退为对应状态，避免首次写入产生跳变
    for (uint j = 0; j < info_.joints.size(); j++)
    {
      size_t cmd_base = joint_command_offsets_[j];
      size_t state_base = joint_state_offsets_[j];
      for (size_t i = 0; i < info_.joints[j].command_interfaces.size(); i++)
      {
        if (cmd_base + i < agg_joint_commands_.size())
        {
          double cmd_val = hw_joint_commands_[j][i];
          if (!std::isfinite(cmd_val))
          {
            // 优先使用同序的状态接口作为回退
            if (state_base + i < agg_joint_states_.size())
            {
              cmd_val = agg_joint_states_[state_base + i];
            }
          }
          agg_joint_commands_[cmd_base + i] = cmd_val;
        }
      }
    }
    master_.writeData();
  }
  return hardware_interface::return_type::OK;
}

std::vector<std::unordered_map<std::string, std::string>> EthercatDriver::getEcModuleParam(
  std::string & urdf, std::string component_name, std::string component_type)
{
  // Check if everything OK with URDF string
  if (urdf.empty())
  {
    throw std::runtime_error("empty URDF passed to robot");
  }
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error())
  {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }
  if (doc.Error())
  {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }

  tinyxml2::XMLElement * robot_it = doc.RootElement();
  if (std::string("robot").compare(robot_it->Name()))
  {
    throw std::runtime_error("the robot tag is not root element in URDF");
  }

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement("ros2_control");
  if (!ros2_control_it)
  {
    throw std::runtime_error("no ros2_control tag");
  }

  std::vector<std::unordered_map<std::string, std::string>> module_params;
  std::unordered_map<std::string, std::string> module_param;

  while (ros2_control_it)
  {
    const auto * ros2_control_child_it = ros2_control_it->FirstChildElement(component_type.c_str());
    while (ros2_control_child_it)
    {
      if (!component_name.compare(ros2_control_child_it->Attribute("name")))
      {
        const auto * ec_module_it = ros2_control_child_it->FirstChildElement("ec_module");
        while (ec_module_it)
        {
          module_param.clear();
          module_param["name"] = ec_module_it->Attribute("name");
          const auto * plugin_it = ec_module_it->FirstChildElement("plugin");
          if (NULL != plugin_it)
          {
            module_param["plugin"] = plugin_it->GetText();
          }
          const auto * param_it = ec_module_it->FirstChildElement("param");
          while (param_it)
          {
            module_param[param_it->Attribute("name")] = param_it->GetText();
            param_it = param_it->NextSiblingElement("param");
          }
          module_params.push_back(module_param);
          ec_module_it = ec_module_it->NextSiblingElement("ec_module");
        }
      }
      ros2_control_child_it = ros2_control_child_it->NextSiblingElement(component_type.c_str());
    }
    ros2_control_it = ros2_control_it->NextSiblingElement("ros2_control");
  }

  return module_params;
}

}  // namespace stark_ethercat_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(stark_ethercat_driver::EthercatDriver, hardware_interface::SystemInterface)
