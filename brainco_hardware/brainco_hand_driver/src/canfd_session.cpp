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

#include "brainco_hand_driver/canfd_session.hpp"

#include <algorithm>
#include <cstdint>

#include "brainco_hand_driver/logger_macros.hpp"

namespace brainco_hand_driver
{

std::atomic<CanfdSession *> CanfdSession::active_instance_{nullptr};

CanfdSession::CanfdSession(BraincoHandApi::DriverConfig & config)
: SessionBase(config), handle_(nullptr, &free_device_handler)
{
}

bool CanfdSession::open()
{
  close();

  auto * current = active_instance_.load(std::memory_order_acquire);
  if (current && current != this)
  {
    BRAINCO_HAND_LOG_ERROR("CANFD session already active");
    return false;
  }

  const auto & canfd = config_.canfd;
  if (!VCI_OpenDevice(canfd.device_type, canfd.card_index, canfd.channel_index))
  {
    BRAINCO_HAND_LOG_ERROR(
      "VCI_OpenDevice failed (type=%u card=%u channel=%u)", canfd.device_type, canfd.card_index,
      canfd.channel_index);
    return false;
  }

  ZCAN_INIT init{};
  init.mode = 0;
  init.clk = canfd.clock_hz;
  init.aset.sjw = canfd.arbitration.sjw;
  init.aset.brp = canfd.arbitration.brp;
  init.aset.tseg1 = canfd.arbitration.tseg1;
  init.aset.tseg2 = canfd.arbitration.tseg2;
  init.aset.smp = canfd.arbitration.smp;
  init.dset.sjw = canfd.data.sjw;
  init.dset.brp = canfd.data.brp;
  init.dset.tseg1 = canfd.data.tseg1;
  init.dset.tseg2 = canfd.data.tseg2;
  init.dset.smp = canfd.data.smp;

  if (!VCI_InitCAN(canfd.device_type, canfd.card_index, canfd.channel_index, &init))
  {
    BRAINCO_HAND_LOG_ERROR("VCI_InitCAN failed");
    VCI_CloseDevice(canfd.device_type, canfd.card_index);
    return false;
  }

  if (!VCI_StartCAN(canfd.device_type, canfd.card_index, canfd.channel_index))
  {
    BRAINCO_HAND_LOG_ERROR("VCI_StartCAN failed");
    VCI_ResetCAN(canfd.device_type, canfd.card_index, canfd.channel_index);
    VCI_CloseDevice(canfd.device_type, canfd.card_index);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    device_started_ = true;
    const auto buffer_size = std::max<uint32_t>(canfd.rx_buffer_size, 1U);
    rx_buffer_.resize(buffer_size);
  }

  active_instance_.store(this, std::memory_order_release);
  set_can_tx_callback(&CanfdSession::tx_callback);
  set_can_rx_callback(&CanfdSession::rx_callback);
  callbacks_registered_ = true;

  DeviceHandler * raw = ::canfd_init(canfd.master_id);
  if (!raw)
  {
    BRAINCO_HAND_LOG_ERROR("canfd_init failed");
    close();
    return false;
  }

  handle_.reset(raw);
  set_handler(raw);
  return true;
}

void CanfdSession::close()
{
  CanfdSession * expected = this;
  const bool cleared =
    active_instance_.compare_exchange_strong(expected, nullptr, std::memory_order_acq_rel);
  if (cleared)
  {
    set_can_tx_callback(&CanfdSession::noop_tx_callback);
    set_can_rx_callback(&CanfdSession::noop_rx_callback);
  }
  callbacks_registered_ = false;

  if (handle_)
  {
    handle_.reset();
  }
  clear_handler();

  std::lock_guard<std::mutex> lock(instance_mutex_);
  if (device_started_)
  {
    const auto & canfd = config_.canfd;
    VCI_ResetCAN(canfd.device_type, canfd.card_index, canfd.channel_index);
    VCI_CloseDevice(canfd.device_type, canfd.card_index);
    device_started_ = false;
  }
  rx_buffer_.clear();
}

bool CanfdSession::is_open() const { return device_started_ && static_cast<bool>(handle_); }

std::optional<BraincoHandApi::ConnectionInfo> CanfdSession::connection_info() const
{
  return std::nullopt;
}

int32_t CanfdSession::tx_callback(
  uint8_t slave_id, uint32_t can_id, const uint8_t * data, uintptr_t data_len)
{
  auto * session = active_instance_.load(std::memory_order_acquire);
  if (!session)
  {
    return -1;
  }
  return session->handle_tx(slave_id, can_id, data, data_len);
}

int32_t CanfdSession::rx_callback(
  uint8_t slave_id, uint32_t * can_id_out, uint8_t * data_out, uintptr_t * data_len_out)
{
  auto * session = active_instance_.load(std::memory_order_acquire);
  if (!session)
  {
    return -1;
  }
  return session->handle_rx(slave_id, can_id_out, data_out, data_len_out);
}

int32_t CanfdSession::noop_tx_callback(uint8_t, uint32_t, const uint8_t *, uintptr_t) { return -1; }

int32_t CanfdSession::noop_rx_callback(uint8_t, uint32_t *, uint8_t *, uintptr_t *) { return -1; }

int32_t CanfdSession::handle_tx(
  uint8_t slave_id, uint32_t can_id, const uint8_t * data, uintptr_t data_len)
{
  (void)slave_id;
  std::lock_guard<std::mutex> lock(instance_mutex_);
  if (!device_started_)
  {
    return -1;
  }

  ZCAN_FD_MSG message{};
  message.hdr.inf.txm = 0;
  message.hdr.inf.fmt = 1;
  message.hdr.inf.sdf = 0;
  message.hdr.inf.sef = 1;
  message.hdr.inf.brs = 1;
  message.hdr.id = can_id;
  message.hdr.chn = static_cast<uint8_t>(config_.canfd.channel_index);
  const auto length =
    static_cast<uint8_t>(std::min<uintptr_t>(data_len, static_cast<uintptr_t>(64)));
  message.hdr.len = length;
  for (uint8_t index = 0; index < length; ++index)
  {
    message.dat[index] = data[index];
  }

  const auto result = VCI_TransmitFD(
    config_.canfd.device_type, config_.canfd.card_index, config_.canfd.channel_index, &message, 1);

  return result == 1 ? 0 : -1;
}

int32_t CanfdSession::handle_rx(
  uint8_t slave_id, uint32_t * can_id_out, uint8_t * data_out, uintptr_t * data_len_out)
{
  (void)slave_id;
  if (!can_id_out || !data_out || !data_len_out)
  {
    return -1;
  }

  std::lock_guard<std::mutex> lock(instance_mutex_);
  if (!device_started_ || rx_buffer_.empty())
  {
    return -1;
  }

  const auto received = VCI_ReceiveFD(
    config_.canfd.device_type, config_.canfd.card_index, config_.canfd.channel_index,
    rx_buffer_.data(), static_cast<uint32_t>(rx_buffer_.size()), config_.canfd.rx_wait_time);

  if (received < 1)
  {
    return -1;
  }

  const auto & frame = rx_buffer_.front();
  const auto payload_len = static_cast<uint8_t>(
    std::min<uintptr_t>(static_cast<uintptr_t>(frame.hdr.len), static_cast<uintptr_t>(64)));
  *can_id_out = frame.hdr.id;
  *data_len_out = payload_len;
  for (uint8_t index = 0; index < payload_len; ++index)
  {
    data_out[index] = frame.dat[index];
  }
  return 0;
}

}  // namespace brainco_hand_driver
