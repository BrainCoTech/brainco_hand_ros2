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

#pragma once

#include "brainco_hand_driver/brainco_hand_api.hpp"

// Forward declarations from Stark SDK
struct DeviceHandler;

namespace brainco_hand_driver
{

// Base class for transport session implementations
class SessionBase : public BraincoHandApi::TransportSession
{
public:
  explicit SessionBase(BraincoHandApi::DriverConfig & config);

  bool fetch_device_info(uint8_t slave_id, BraincoHandApi::DeviceInfoData & info) const override;
  bool ensure_physical_unit_mode(uint8_t slave_id) override;
  std::optional<BraincoHandApi::MotorStatus> get_motor_status(uint8_t slave_id) const override;
  bool set_finger_positions_and_durations(
    uint8_t slave_id, const uint16_t * positions, const uint16_t * durations,
    std::size_t count) override;

protected:
  void set_handler(DeviceHandler * handler);
  void clear_handler();
  [[nodiscard]] DeviceHandler * handler() const;

  BraincoHandApi::DriverConfig & config_;

private:
  DeviceHandler * handler_{nullptr};
};

}  // namespace brainco_hand_driver

