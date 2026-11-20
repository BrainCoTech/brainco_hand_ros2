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

#include "revo2_driver/brainco_hand_api.hpp"

#include <memory>

#include "revo2_driver/modbus_session.hpp"
#include "revo2_driver/sdk_helpers.hpp"
#include "stark-sdk.h"

#if ENABLE_CANFD
#include "revo2_driver/canfd_session.hpp"
#endif

namespace revo2_driver
{

auto brainco_log_level_to_string(BraincoLogLevel level) -> std::string
{
  switch (level)
  {
    case BraincoLogLevel::kError:
      return "error";
    case BraincoLogLevel::kWarn:
      return "warn";
    case BraincoLogLevel::kInfo:
      return "info";
    case BraincoLogLevel::kDebug:
      return "debug";
    case BraincoLogLevel::kTrace:
      return "trace";
  }
  return "info";
}

// Implementation details for BraincoHandApi
struct BraincoHandApi::Impl
{
  DriverConfig config{};
  std::unique_ptr<TransportSession> session;

  void rebuild_session()
  {
    switch (config.protocol)
    {
      case Protocol::kModbus:
        session = std::make_unique<ModbusSession>(config);
        break;
#if ENABLE_CANFD
      case Protocol::kCanfd:
        session = std::make_unique<CanfdSession>(config);
        break;
#endif
      default:
        throw std::runtime_error("Unsupported protocol");
    }
  }
};

// Delay SDK logging until the real configuration is available. Some SDK builds
// initialize their process-wide logger only once.
BraincoHandApi::BraincoHandApi() : impl_(std::make_unique<Impl>()) {}

BraincoHandApi::BraincoHandApi(const DriverConfig & config) : impl_(std::make_unique<Impl>())
{
  configure(config);
}

BraincoHandApi::~BraincoHandApi() = default;

auto BraincoHandApi::configure(const DriverConfig & config) -> void
{
  if (!impl_)
  {
    impl_ = std::make_unique<Impl>();
  }

  close();
  impl_->config = config;

  ::init_logging_with_options(to_sdk_log_level(config.log_level), config.sdk_file_logging);
  impl_->rebuild_session();
}

auto BraincoHandApi::open() -> bool
{
  if (!impl_)
  {
    return false;
  }
  if (!impl_->session)
  {
    configure(impl_->config);
  }
  return impl_->session->open();
}

auto BraincoHandApi::close() -> void
{
  if (impl_ && impl_->session)
  {
    impl_->session->close();
  }
}

auto BraincoHandApi::is_open() const -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->is_open();
}

auto BraincoHandApi::fetch_device_info(uint8_t slave_id, DeviceInfoData & info) const -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->fetch_device_info(slave_id, info);
}

auto BraincoHandApi::ensure_finger_unit_mode(
  uint8_t slave_id, FingerUnitModeSetting mode) -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->ensure_finger_unit_mode(slave_id, mode);
}

auto BraincoHandApi::get_motor_status(uint8_t slave_id) const -> std::optional<MotorStatus>
{
  if (!impl_ || !impl_->session)
  {
    return std::nullopt;
  }
  return impl_->session->get_motor_status(slave_id);
}

auto BraincoHandApi::get_touch_status(uint8_t slave_id) const -> std::optional<TouchStatus>
{
  if (!impl_ || !impl_->session)
  {
    return std::nullopt;
  }
  return impl_->session->get_touch_status(slave_id);
}

auto BraincoHandApi::get_finger_position_range(uint8_t slave_id) const
  -> std::optional<FingerPositionRange>
{
  if (!impl_ || !impl_->session)
  {
    return std::nullopt;
  }
  return impl_->session->get_finger_position_range(slave_id);
}

auto BraincoHandApi::enable_touch_sensors(uint8_t slave_id, uint8_t sensor_mask) -> int32_t
{
  if (!impl_ || !impl_->session)
  {
    return -1;
  }
  return impl_->session->enable_touch_sensors(slave_id, sensor_mask);
}

auto BraincoHandApi::calibrate_touch_sensors(uint8_t slave_id, uint8_t sensor_mask) -> int32_t
{
  if (!impl_ || !impl_->session)
  {
    return -1;
  }
  return impl_->session->calibrate_touch_sensors(slave_id, sensor_mask);
}

auto BraincoHandApi::set_finger_positions_and_durations(
  uint8_t slave_id, const uint16_t * positions, const uint16_t * durations, std::size_t count)
  -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->set_finger_positions_and_durations(slave_id, positions, durations, count);
}

auto BraincoHandApi::set_finger_positions_and_velocities(
  uint8_t slave_id, const uint16_t * positions, const uint16_t * velocities, std::size_t count)
  -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->set_finger_positions_and_velocities(slave_id, positions, velocities, count);
}

auto BraincoHandApi::set_finger_speeds(
  uint8_t slave_id, const int16_t * speeds, std::size_t count) -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->set_finger_speeds(slave_id, speeds, count);
}

auto BraincoHandApi::set_finger_currents(
  uint8_t slave_id, const int16_t * currents, std::size_t count) -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->set_finger_currents(slave_id, currents, count);
}

auto BraincoHandApi::set_finger_pwms(
  uint8_t slave_id, const int16_t * pwms, std::size_t count) -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->set_finger_pwms(slave_id, pwms, count);
}

auto BraincoHandApi::set_thumb_aux_lock_current(uint8_t slave_id, uint16_t current_ma) -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->set_thumb_aux_lock_current(slave_id, current_ma);
}

auto BraincoHandApi::set_thumb_aux_max_current(uint8_t slave_id, uint16_t current_ma) -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->set_thumb_aux_max_current(slave_id, current_ma);
}

auto BraincoHandApi::set_thumb_aux_protected_current(uint8_t slave_id, uint16_t current_ma) -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->set_thumb_aux_protected_current(slave_id, current_ma);
}

auto BraincoHandApi::set_thumb_aux_max_speed(uint8_t slave_id, uint16_t speed_deg_s) -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->set_thumb_aux_max_speed(slave_id, speed_deg_s);
}

auto BraincoHandApi::resolved_connection() const -> std::optional<ConnectionInfo>
{
  if (!impl_ || !impl_->session)
  {
    return std::nullopt;
  }
  return impl_->session->connection_info();
}

}  // namespace revo2_driver
