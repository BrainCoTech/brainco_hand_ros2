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
#include "zlgcan/zcan.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace brainco_hand_driver
{

// CAN FD transport session implementation
class CanfdSession final : public SessionBase
{
public:
  explicit CanfdSession(BraincoHandApi::DriverConfig & config);

  bool open() override;
  void close() override;
  [[nodiscard]] bool is_open() const override;
  [[nodiscard]] std::optional<BraincoHandApi::ConnectionInfo> connection_info() const override;

private:
  using CanfdHandlePtr = std::unique_ptr<DeviceHandler, decltype(&free_device_handler)>;

  static int32_t tx_callback(
    uint8_t slave_id, uint32_t can_id, const uint8_t * data, uintptr_t data_len);
  static int32_t rx_callback(
    uint8_t slave_id, uint32_t * can_id_out, uint8_t * data_out, uintptr_t * data_len_out);
  static int32_t noop_tx_callback(uint8_t, uint32_t, const uint8_t *, uintptr_t);
  static int32_t noop_rx_callback(uint8_t, uint32_t *, uint8_t *, uintptr_t *);

  int32_t handle_tx(uint8_t slave_id, uint32_t can_id, const uint8_t * data, uintptr_t data_len);
  int32_t handle_rx(
    uint8_t slave_id, uint32_t * can_id_out, uint8_t * data_out, uintptr_t * data_len_out);

  CanfdHandlePtr handle_;
  bool device_started_{false};
  bool callbacks_registered_{false};
  std::vector<ZCAN_FD_MSG> rx_buffer_{};
  std::mutex instance_mutex_{};

  static std::atomic<CanfdSession *> active_instance_;
};

}  // namespace brainco_hand_driver
