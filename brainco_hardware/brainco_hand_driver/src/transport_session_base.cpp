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

#include "brainco_hand_driver/transport_session_base.hpp"

#include <memory>
#include <optional>

#include "brainco_hand_driver/logger_macros.hpp"
#include "brainco_hand_driver/sdk_helpers.hpp"

namespace brainco_hand_driver
{

SessionBase::SessionBase(BraincoHandApi::DriverConfig & config) : config_(config) {}

bool SessionBase::fetch_device_info(uint8_t slave_id, BraincoHandApi::DeviceInfoData & info) const
{
  if (!handler_)
  {
    BRAINCO_HAND_LOG_ERROR("Handler is not set");
    return false;
  }
  BRAINCO_HAND_LOG_INFO("Fetch device info for slave %u", slave_id);

  DeviceInfoPtr device_info{::stark_get_device_info(handler_, slave_id)};
  if (!device_info)
  {
    return false;
  }

  info.sku_type = static_cast<uint8_t>(device_info->sku_type);
  info.hardware_type = static_cast<uint8_t>(device_info->hardware_type);
  info.serial_number = device_info->serial_number ? device_info->serial_number : std::string{};
  info.firmware_version =
    device_info->firmware_version ? device_info->firmware_version : std::string{};
  return true;
}

bool SessionBase::ensure_physical_unit_mode(uint8_t slave_id)
{
  if (!handler_)
  {
    return false;
  }

  FingerUnitMode current_mode = ::stark_get_finger_unit_mode(handler_, slave_id);
  if (current_mode != FINGER_UNIT_MODE_PHYSICAL)
  {
    ::stark_set_finger_unit_mode(handler_, slave_id, FINGER_UNIT_MODE_PHYSICAL);
    current_mode = ::stark_get_finger_unit_mode(handler_, slave_id);
    return current_mode == FINGER_UNIT_MODE_PHYSICAL;
  }

  return true;
}

std::optional<BraincoHandApi::MotorStatus> SessionBase::get_motor_status(uint8_t slave_id) const
{
  if (!handler_)
  {
    return std::nullopt;
  }

  MotorStatusPtr raw_status{::stark_get_motor_status(handler_, slave_id)};
  if (!raw_status)
  {
    return std::nullopt;
  }

  BraincoHandApi::MotorStatus status{};
  for (std::size_t index = 0; index < BraincoHandApi::kFingerCount; ++index)
  {
    status.positions[index] = raw_status->positions[index];
    status.speeds[index] = raw_status->speeds[index];
    status.currents[index] = raw_status->currents[index];
    status.states[index] = raw_status->states[index];
  }

  return status;
}

bool SessionBase::set_finger_positions_and_durations(
  uint8_t slave_id, const uint16_t * positions, const uint16_t * durations, std::size_t count)
{
  if (!handler_ || !positions || !durations || count == 0)
  {
    return false;
  }

  ::stark_set_finger_positions_and_durations(handler_, slave_id, positions, durations, count);
  return true;
}

void SessionBase::set_handler(DeviceHandler * handler) { handler_ = handler; }

void SessionBase::clear_handler() { handler_ = nullptr; }

DeviceHandler * SessionBase::handler() const { return handler_; }

}  // namespace brainco_hand_driver

