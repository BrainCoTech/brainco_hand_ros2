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

#include "revo2_driver/brainco_hand_api.hpp"

// Forward declarations from Stark SDK
struct DeviceHandler;

namespace revo2_driver
{

// Base class for transport session implementations
class SessionBase : public BraincoHandApi::TransportSession
{
public:
  explicit SessionBase(BraincoHandApi::DriverConfig & config);

  bool fetch_device_info(uint8_t slave_id, BraincoHandApi::DeviceInfoData & info) const override;
  bool ensure_finger_unit_mode(
    uint8_t slave_id, FingerUnitModeSetting mode) override;
  std::optional<BraincoHandApi::MotorStatus> get_motor_status(uint8_t slave_id) const override;
  std::optional<BraincoHandApi::TouchStatus> get_touch_status(uint8_t slave_id) const override;
  int32_t enable_touch_sensors(uint8_t slave_id, uint8_t sensor_mask) override;
  int32_t calibrate_touch_sensors(uint8_t slave_id, uint8_t sensor_mask) override;
  std::optional<BraincoHandApi::FingerPositionRange> get_finger_position_range(
    uint8_t slave_id) const override;
  bool set_finger_positions_and_durations(
    uint8_t slave_id, const uint16_t * positions, const uint16_t * durations,
    std::size_t count) override;

  bool set_finger_positions_and_velocities(
    uint8_t slave_id, const uint16_t * positions, const uint16_t * velocities,
    std::size_t count) override;

  bool set_finger_speeds(uint8_t slave_id, const int16_t * speeds, 
    std::size_t count) override;

  bool set_finger_currents(uint8_t slave_id, const int16_t * currents, 
    std::size_t count) override;

  bool set_finger_pwms(uint8_t slave_id, const int16_t * pwms, 
    std::size_t count) override;

  bool set_thumb_aux_lock_current(uint8_t slave_id, uint16_t current_ma) override;
  bool set_thumb_aux_max_current(uint8_t slave_id, uint16_t current_ma) override;
  bool set_thumb_aux_protected_current(uint8_t slave_id, uint16_t current_ma) override;
  bool set_thumb_aux_max_speed(uint8_t slave_id, uint16_t speed_deg_s) override;
  
protected:
  void set_handler(DeviceHandler * handler);
  void clear_handler();
  [[nodiscard]] DeviceHandler * handler() const;

  BraincoHandApi::DriverConfig & config_;

private:
  DeviceHandler * handler_{nullptr};
};

}  // namespace revo2_driver
