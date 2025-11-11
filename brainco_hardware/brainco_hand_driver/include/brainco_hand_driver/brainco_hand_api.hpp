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
struct DeviceConfig;
struct DeviceHandler;
struct DeviceInfo;
struct MotorStatusData;

namespace brainco_hand_driver
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

class BraincoHandApi
{
public:
  static constexpr std::size_t kFingerCount{6};

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
    bool ensure_physical_mode{true};
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
    virtual bool ensure_physical_unit_mode(uint8_t slave_id) = 0;
    [[nodiscard]] virtual std::optional<MotorStatus> get_motor_status(uint8_t slave_id) const = 0;
    virtual bool set_finger_positions_and_durations(
      uint8_t slave_id, const uint16_t * positions, const uint16_t * durations,
      std::size_t count) = 0;
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

  auto ensure_physical_unit_mode(uint8_t slave_id) -> bool;

  auto get_motor_status(uint8_t slave_id) const -> std::optional<MotorStatus>;

  auto set_finger_positions_and_durations(
    uint8_t slave_id, const uint16_t * positions, const uint16_t * durations, std::size_t count)
    -> bool;

  [[nodiscard]] auto resolved_connection() const -> std::optional<ConnectionInfo>;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace brainco_hand_driver
