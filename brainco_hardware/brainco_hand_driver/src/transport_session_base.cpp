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

#include "revo2_driver/transport_session_base.hpp"

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "revo2_driver/logger_macros.hpp"
#include "revo2_driver/sdk_helpers.hpp"

namespace revo2_driver
{

namespace
{
constexpr int kFingerUnitModeConfirmAttempts{8};
constexpr std::chrono::milliseconds kFingerUnitModeConfirmDelay{50};

auto copy_sdk_string(const char * value) -> std::string
{
  return value ? std::string{value} : std::string{};
}

auto finger_unit_mode_to_string(FingerUnitMode mode) -> const char *
{
  switch (mode)
  {
    case FINGER_UNIT_MODE_NORMALIZED:
      return "normalized";
    case FINGER_UNIT_MODE_PHYSICAL:
      return "physical";
  }
  return "unknown";
}
}  // namespace

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

  // SDK v2.0.9 is installed with its matching header; use its public ABI directly.
  info.sku_type = static_cast<uint8_t>(device_info->sku_type);
  info.hardware_type = static_cast<uint8_t>(device_info->hardware_type);
  info.serial_number = copy_sdk_string(device_info->serial_number);
  info.firmware_version = copy_sdk_string(device_info->firmware_version);
  return true;
}

bool SessionBase::ensure_finger_unit_mode(uint8_t slave_id, FingerUnitModeSetting mode)
{
  if (!handler_)
  {
    BRAINCO_HAND_LOG_WARN("Cannot ensure finger unit mode: handler is not set");
    return false;
  }

  if (mode == FingerUnitModeSetting::kKeepCurrent)
  {
    return true;
  }

  const FingerUnitMode target_mode =
    mode == FingerUnitModeSetting::kPhysical ? FINGER_UNIT_MODE_PHYSICAL
                                             : FINGER_UNIT_MODE_NORMALIZED;
  FingerUnitMode current_mode = ::stark_get_finger_unit_mode(handler_, slave_id);

  BRAINCO_HAND_LOG_INFO(
    "Finger unit mode before=%s target=%s slave_id=%u",
    finger_unit_mode_to_string(current_mode), finger_unit_mode_to_string(target_mode),
    static_cast<unsigned>(slave_id));

  // Always SET. GET can report physical on a hand whose SDK position APIs
  // still use unified 0-1000; skipping SET then writing physical-scale 57.3
  // sends ~80 into a 0-1000 range (~8° twitch).
  ::stark_set_finger_unit_mode(handler_, slave_id, target_mode);
  for (int attempt = 1; attempt <= kFingerUnitModeConfirmAttempts; ++attempt)
  {
    std::this_thread::sleep_for(kFingerUnitModeConfirmDelay);
    current_mode = ::stark_get_finger_unit_mode(handler_, slave_id);
    BRAINCO_HAND_LOG_INFO(
      "Finger unit mode confirm attempt %d/%d: current=%s target=%s slave_id=%u", attempt,
      kFingerUnitModeConfirmAttempts, finger_unit_mode_to_string(current_mode),
      finger_unit_mode_to_string(target_mode), static_cast<unsigned>(slave_id));
    if (current_mode == target_mode)
    {
      return true;
    }
  }

  BRAINCO_HAND_LOG_WARN(
    "Finger unit mode confirmation failed after %d attempts: current=%s target=%s slave_id=%u",
    kFingerUnitModeConfirmAttempts, finger_unit_mode_to_string(current_mode),
    finger_unit_mode_to_string(target_mode), static_cast<unsigned>(slave_id));
  return false;
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

std::optional<BraincoHandApi::FingerPositionRange> SessionBase::get_finger_position_range(
  uint8_t slave_id) const
{
  if (!handler_)
  {
    return std::nullopt;
  }

  // SDK finger ids are 1-based [Thumb..Pinky] and align with our 0-based
  // [Thumb, ThumbAux, Index, Middle, Ring, Pinky] once shifted by +1.
  static constexpr std::array<StarkFingerId, BraincoHandApi::kFingerCount> kFingerIds{
    STARK_FINGER_ID_THUMB, STARK_FINGER_ID_THUMB_AUX, STARK_FINGER_ID_INDEX,
    STARK_FINGER_ID_MIDDLE, STARK_FINGER_ID_RING, STARK_FINGER_ID_PINKY};

  BraincoHandApi::FingerPositionRange range{};
  for (std::size_t index = 0; index < BraincoHandApi::kFingerCount; ++index)
  {
    range.min_deg[index] =
      static_cast<double>(::stark_get_finger_min_position(handler_, slave_id, kFingerIds[index]));
    range.max_deg[index] =
      static_cast<double>(::stark_get_finger_max_position(handler_, slave_id, kFingerIds[index]));
  }

  return range;
}

std::optional<BraincoHandApi::TouchStatus> SessionBase::get_touch_status(uint8_t slave_id) const
{
  if (!handler_)
  {
    return std::nullopt;
  }

  TouchFingerDataPtr raw_status{::stark_get_touch_status(handler_, slave_id)};
  if (!raw_status)
  {
    return std::nullopt;
  }

  BraincoHandApi::TouchStatus status{};
  for (std::size_t index = 0; index < BraincoHandApi::kTouchFingerCount; ++index)
  {
    const auto & raw_item = raw_status->items[index];
    auto & dst_item = status.items[index];
    dst_item.tactile_normal_force = raw_item.normal_force1;
    dst_item.tactile_tangential_force = raw_item.tangential_force1;
    dst_item.tactile_tangential_direction = raw_item.tangential_direction1;
    dst_item.tactile_self_proximity = raw_item.self_proximity1;
    dst_item.tactile_status = raw_item.status;
  }

  return status;
}

int32_t SessionBase::enable_touch_sensors(uint8_t slave_id, uint8_t sensor_mask)
{
  if (!handler_ || sensor_mask > 0x1F)
  {
    return -1;
  }
  return ::stark_enable_touch_sensor_with_status(handler_, slave_id, sensor_mask);
}

int32_t SessionBase::calibrate_touch_sensors(uint8_t slave_id, uint8_t sensor_mask)
{
  if (!handler_ || sensor_mask == 0 || sensor_mask > 0x1F)
  {
    return -1;
  }
  return ::stark_calibrate_touch_sensor_with_status(handler_, slave_id, sensor_mask);
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

bool SessionBase::set_finger_positions_and_velocities(
  uint8_t slave_id, const uint16_t * positions, const uint16_t * velocities, std::size_t count)
{
  if (!handler_ || !positions || !velocities || count == 0)
  {
    return false;
  }

  ::stark_set_finger_positions_and_speeds(handler_, slave_id, positions, velocities, count);
  return true;
}

bool SessionBase::set_finger_speeds(
  uint8_t slave_id, const int16_t * speeds, std::size_t count)
{
  if (!handler_ || !speeds || count == 0)
  {
    return false;
  }

  constexpr std::size_t kKnownFingerCount = BraincoHandApi::kFingerCount;
  const auto finger_count = count < kKnownFingerCount ? count : kKnownFingerCount;
  ::stark_set_finger_speeds(handler_, slave_id, speeds, finger_count);
  return true;
}

bool SessionBase::set_finger_pwms(
  uint8_t slave_id, const int16_t * pwms, std::size_t count)
{
  if (!handler_ || !pwms || count == 0)
  {
    return false;
  }

  ::stark_set_finger_pwms(handler_, slave_id, pwms, count);
  return true;
}

bool SessionBase::set_finger_currents(
  uint8_t slave_id, const int16_t * currents, std::size_t count)
{
  if (!handler_ || !currents || count == 0)
  {
    return false;
  }

  ::stark_set_finger_currents(handler_, slave_id, currents, count);
  return true;
}

bool SessionBase::set_thumb_aux_lock_current(uint8_t slave_id, uint16_t current_ma)
{
  if (!handler_)
  {
    return false;
  }

  ::stark_set_thumb_aux_lock_current(handler_, slave_id, current_ma);
  return true;
}

bool SessionBase::set_thumb_aux_max_current(uint8_t slave_id, uint16_t current_ma)
{
  if (!handler_)
  {
    return false;
  }

  ::stark_set_finger_max_current(
    handler_, slave_id, STARK_FINGER_ID_THUMB_AUX, current_ma);
  return true;
}

bool SessionBase::set_thumb_aux_protected_current(uint8_t slave_id, uint16_t current_ma)
{
  if (!handler_)
  {
    return false;
  }

  ::stark_set_finger_protected_current(
    handler_, slave_id, STARK_FINGER_ID_THUMB_AUX, current_ma);
  return true;
}

bool SessionBase::set_thumb_aux_max_speed(uint8_t slave_id, uint16_t speed_deg_s)
{
  if (!handler_)
  {
    return false;
  }

  ::stark_set_finger_max_speed(handler_, slave_id, STARK_FINGER_ID_THUMB_AUX, speed_deg_s);
  return true;
}

void SessionBase::set_handler(DeviceHandler * handler) { handler_ = handler; }

void SessionBase::clear_handler() { handler_ = nullptr; }

DeviceHandler * SessionBase::handler() const { return handler_; }

}  // namespace revo2_driver
