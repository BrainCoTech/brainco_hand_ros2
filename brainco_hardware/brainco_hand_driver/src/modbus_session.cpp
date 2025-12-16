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

#include "brainco_hand_driver/modbus_session.hpp"

#include "brainco_hand_driver/sdk_helpers.hpp"

namespace brainco_hand_driver
{

ModbusSession::ModbusSession(BraincoHandApi::DriverConfig & config)
: SessionBase(config), handle_(nullptr, &modbus_close)
{
}

bool ModbusSession::open()
{
  close();

  BraincoHandApi::ConnectionInfo connection{};
  connection.slave_id = config_.slave_id;

  if (config_.modbus.auto_detect)
  {
    const char * hint =
      config_.modbus.auto_detect_port.empty() ? nullptr : config_.modbus.auto_detect_port.c_str();
    DeviceConfigPtr detected{::auto_detect_modbus_revo2(hint, config_.modbus.auto_detect_quick)};
    if (!detected)
    {
      return false;
    }

    connection.port = detected->port_name ? detected->port_name : std::string{};
    connection.baudrate = detected->baudrate;
    connection.slave_id = detected->slave_id;
    config_.slave_id = detected->slave_id;
  }
  else
  {
    connection.port = config_.modbus.port;
    connection.baudrate = config_.modbus.baudrate;
  }

  DeviceHandler * raw = ::modbus_open(connection.port.c_str(), connection.baudrate);
  if (!raw)
  {
    return false;
  }

  handle_.reset(raw);
  set_handler(raw);
  resolved_connection_ = connection;
  return true;
}

void ModbusSession::close()
{
  if (handle_)
  {
    handle_.reset();
  }
  clear_handler();
  resolved_connection_.reset();
}

bool ModbusSession::is_open() const { return static_cast<bool>(handle_); }

std::optional<BraincoHandApi::ConnectionInfo> ModbusSession::connection_info() const
{
  return resolved_connection_;
}

}  // namespace brainco_hand_driver

