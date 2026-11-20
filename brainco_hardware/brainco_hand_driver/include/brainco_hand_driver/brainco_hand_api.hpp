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

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

// Forward declarations from Stark SDK to decouple headers.
struct CDeviceConfig;
struct CDeviceInfo;
struct CMotorStatusData;
struct CTouchFingerData;
struct DeviceHandler;

namespace revo2_driver
{

enum class BraincoLogLevel : uint8_t
{
  kError = 0,
  kWarn = 1,
  kInfo = 2,
  kDebug = 3,
  kTrace = 4,
};

auto brainco_log_level_to_string(BraincoLogLevel level) -> std::string;

enum class Protocol : uint8_t
{
  kModbus = 0,
  kCanfd = 1,
};

enum class FingerUnitModeSetting : uint8_t
{
  kKeepCurrent = 0,
  kNormalized = 1,
  kPhysical = 2,
};

class BraincoHandApi
{
public:
  static constexpr std::size_t kFingerCount{6};
  static constexpr std::size_t kTouchFingerCount{5};

  struct ConnectionInfo
  {
    std::string port;
    uint32_t baudrate{0};
    uint8_t slave_id{0};
  };

  struct DeviceInfoData
  {
    uint8_t sku_type{0};
    uint8_t hardware_type{0};
    std::string serial_number;
    std::string firmware_version;
  };

  struct MotorStatus
  {
    std::array<uint16_t, kFingerCount> positions{};
    std::array<int16_t, kFingerCount> speeds{};
    std::array<int16_t, kFingerCount> currents{};
    std::array<uint8_t, kFingerCount> states{};
  };

  // Per-finger physical position limits read from the device (degrees).
  // Order matches MotorStatus: [Thumb, ThumbAux, Index, Middle, Ring, Pinky].
  // Firmware maps the unified 0~1000 SDK range onto 0..max_deg for each finger;
  // min_deg is the soft floor. Used to drive fingers in physical units while the
  // transport keeps sending the SDK-mandated 0~1000 values.
  struct FingerPositionRange
  {
    std::array<double, kFingerCount> min_deg{};
    std::array<double, kFingerCount> max_deg{};
  };

  struct TouchStatusItem
  {
    uint16_t tactile_normal_force{0};
    uint16_t tactile_tangential_force{0};
    uint16_t tactile_tangential_direction{0};
    uint32_t tactile_self_proximity{0};
    uint16_t tactile_status{0};
  };

  struct TouchStatus
  {
    std::array<TouchStatusItem, kTouchFingerCount> items{};
  };

  struct ModbusConfig
  {
    std::string port{"/dev/ttyUSB0"};
    uint32_t baudrate{460800};
    bool auto_detect{false};
    bool auto_detect_quick{true};
    std::string auto_detect_port;
  };

  struct CanfdBitTiming
  {
    uint8_t sjw{1};
    uint16_t brp{4};
    uint8_t tseg1{7};
    uint8_t tseg2{2};
    uint8_t smp{0};
  };

  struct CanfdConfig
  {
    uint32_t device_type{33};
    uint32_t card_index{0};
    uint32_t channel_index{0};
    uint32_t clock_hz{60000000};
    CanfdBitTiming arbitration{};
    CanfdBitTiming data{1, 0, 7, 2, 0};
    uint32_t rx_wait_time{100};
    uint32_t rx_buffer_size{1000};
    uint8_t master_id{1};
  };

  struct DriverConfig
  {
    Protocol protocol{Protocol::kModbus};
    uint8_t slave_id{126};
    BraincoLogLevel log_level{BraincoLogLevel::kInfo};
    bool sdk_file_logging{false};
    bool ensure_physical_mode{false};
    FingerUnitModeSetting finger_unit_mode{FingerUnitModeSetting::kNormalized};
    ModbusConfig modbus{};
    CanfdConfig canfd{};
  };

  class TransportSession
  {
  public:
    virtual ~TransportSession() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    [[nodiscard]] virtual bool is_open() const = 0;
    [[nodiscard]] virtual std::optional<ConnectionInfo> connection_info() const = 0;
    virtual bool fetch_device_info(uint8_t slave_id, DeviceInfoData & info) const = 0;
    virtual bool ensure_finger_unit_mode(uint8_t slave_id, FingerUnitModeSetting mode) = 0;
    [[nodiscard]] virtual std::optional<MotorStatus> get_motor_status(uint8_t slave_id) const = 0;
    [[nodiscard]] virtual std::optional<TouchStatus> get_touch_status(uint8_t slave_id) const = 0;
    // SDK status: 0 = transport success, -1 = invalid input/handle or operation failure.
    virtual int32_t enable_touch_sensors(uint8_t slave_id, uint8_t sensor_mask) = 0;
    virtual int32_t calibrate_touch_sensors(uint8_t slave_id, uint8_t sensor_mask) = 0;
    [[nodiscard]] virtual std::optional<FingerPositionRange> get_finger_position_range(
      uint8_t slave_id) const = 0;
    virtual bool set_finger_positions_and_durations(
      uint8_t slave_id, const uint16_t * positions, const uint16_t * durations,
      std::size_t count) = 0;
    virtual bool set_finger_positions_and_velocities(
      uint8_t slave_id, const uint16_t * positions, const uint16_t * velocities,
      std::size_t count) = 0;
    virtual bool set_finger_speeds(
      uint8_t slave_id, const int16_t * speeds, std::size_t count) = 0;
    virtual bool set_finger_currents(
      uint8_t slave_id, const int16_t * currents, std::size_t count) = 0;
    virtual bool set_finger_pwms(
      uint8_t slave_id, const int16_t * pwms, std::size_t count) = 0;
    virtual bool set_thumb_aux_lock_current(uint8_t slave_id, uint16_t current_ma) = 0;
    virtual bool set_thumb_aux_max_current(uint8_t slave_id, uint16_t current_ma) = 0;
    virtual bool set_thumb_aux_protected_current(uint8_t slave_id, uint16_t current_ma) = 0;
    virtual bool set_thumb_aux_max_speed(uint8_t slave_id, uint16_t speed_deg_s) = 0;
  };

  BraincoHandApi();
  explicit BraincoHandApi(const DriverConfig & config);
  ~BraincoHandApi();

  BraincoHandApi(const BraincoHandApi &) = delete;
  BraincoHandApi & operator=(const BraincoHandApi &) = delete;
  BraincoHandApi(BraincoHandApi &&) noexcept = default;
  BraincoHandApi & operator=(BraincoHandApi &&) noexcept = default;

  auto configure(const DriverConfig & config) -> void;

  auto open() -> bool;
  auto close() -> void;
  [[nodiscard]] auto is_open() const -> bool;

  auto fetch_device_info(uint8_t slave_id, DeviceInfoData & info) const -> bool;

  auto ensure_finger_unit_mode(uint8_t slave_id, FingerUnitModeSetting mode) -> bool;

  auto get_motor_status(uint8_t slave_id) const -> std::optional<MotorStatus>;

  auto get_touch_status(uint8_t slave_id) const -> std::optional<TouchStatus>;

  auto enable_touch_sensors(uint8_t slave_id, uint8_t sensor_mask) -> int32_t;
  auto calibrate_touch_sensors(uint8_t slave_id, uint8_t sensor_mask) -> int32_t;

  // Read per-finger physical position limits (degrees) from the device.
  // Returns nullopt when the transport is closed.
  auto get_finger_position_range(uint8_t slave_id) const -> std::optional<FingerPositionRange>;

  auto set_finger_positions_and_durations(
    uint8_t slave_id, const uint16_t * positions, const uint16_t * durations, std::size_t count)
    -> bool;

  auto set_finger_positions_and_velocities(
    uint8_t slave_id, const uint16_t * positions, const uint16_t * velocities, std::size_t count)
    -> bool;

  auto set_finger_speeds(
    uint8_t slave_id, const int16_t * speeds, std::size_t count) 
    -> bool;

  auto set_finger_currents(
    uint8_t slave_id, const int16_t * currents, std::size_t count) 
    -> bool;

  auto set_finger_pwms(
    uint8_t slave_id, const int16_t * pwms, std::size_t count) 
    -> bool;

  auto set_thumb_aux_lock_current(uint8_t slave_id, uint16_t current_ma) -> bool;
  auto set_thumb_aux_max_current(uint8_t slave_id, uint16_t current_ma) -> bool;
  auto set_thumb_aux_protected_current(uint8_t slave_id, uint16_t current_ma) -> bool;
  auto set_thumb_aux_max_speed(uint8_t slave_id, uint16_t speed_deg_s) -> bool;

  [[nodiscard]] auto resolved_connection() const -> std::optional<ConnectionInfo>;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace revo2_driver
