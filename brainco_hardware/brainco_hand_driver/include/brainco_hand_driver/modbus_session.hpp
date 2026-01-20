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

#include "brainco_hand_driver/transport_session_base.hpp"
#include "stark-sdk.h"

#include <memory>
#include <optional>

namespace brainco_hand_driver
{

// Modbus transport session implementation
class ModbusSession final : public SessionBase
{
public:
  explicit ModbusSession(BraincoHandApi::DriverConfig & config);

  bool open() override;
  void close() override;
  [[nodiscard]] bool is_open() const override;
  [[nodiscard]] std::optional<BraincoHandApi::ConnectionInfo> connection_info() const override;

private:
  using ModbusHandlePtr = std::unique_ptr<DeviceHandler, decltype(&modbus_close)>;

  ModbusHandlePtr handle_;
  std::optional<BraincoHandApi::ConnectionInfo> resolved_connection_{};
};

}  // namespace brainco_hand_driver

