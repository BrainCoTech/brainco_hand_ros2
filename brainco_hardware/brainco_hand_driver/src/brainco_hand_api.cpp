#include "brainco_hand_driver/brainco_hand_api.hpp"

#include <algorithm>
#include <atomic>
#include <mutex>
#include <utility>
#include <vector>

#include "brainco_hand_driver/logger_macros.hpp"
#include "stark-sdk.h"
#include "zlgcan/zcan.h"

namespace brainco_hand_driver
{

namespace
{

struct DeviceConfigDeleter
{
  void operator()(DeviceConfig * config) const
  {
    if (config)
    {
      ::free_device_config(config);
    }
  }
};

struct DeviceInfoDeleter
{
  void operator()(DeviceInfo * info) const
  {
    if (info)
    {
      ::free_device_info(info);
    }
  }
};

struct MotorStatusDeleter
{
  void operator()(MotorStatusData * data) const
  {
    if (data)
    {
      ::free_motor_status_data(data);
    }
  }
};

using DeviceConfigPtr = std::unique_ptr<DeviceConfig, DeviceConfigDeleter>;
using DeviceInfoPtr = std::unique_ptr<DeviceInfo, DeviceInfoDeleter>;
using MotorStatusPtr = std::unique_ptr<MotorStatusData, MotorStatusDeleter>;

auto to_sdk_log_level(BraincoLogLevel level) -> LogLevel
{
  switch (level)
  {
    case BraincoLogLevel::kError:
      return LOG_LEVEL_ERROR;
    case BraincoLogLevel::kWarn:
      return LOG_LEVEL_WARN;
    case BraincoLogLevel::kInfo:
      return LOG_LEVEL_INFO;
    case BraincoLogLevel::kDebug:
      return LOG_LEVEL_DEBUG;
    case BraincoLogLevel::kTrace:
      return LOG_LEVEL_TRACE;
  }
  return LOG_LEVEL_INFO;
}

}  // namespace

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

class SessionBase : public BraincoHandApi::TransportSession
{
public:
  explicit SessionBase(BraincoHandApi::DriverConfig & config)
  : config_(config)
  {
  }

  bool fetch_device_info(uint8_t slave_id, BraincoHandApi::DeviceInfoData & info) const override
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
    info.firmware_version = device_info->firmware_version ? device_info->firmware_version : std::string{};
    return true;
  }

  bool ensure_physical_unit_mode(uint8_t slave_id) override
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

  std::optional<BraincoHandApi::MotorStatus> get_motor_status(uint8_t slave_id) const override
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

  bool set_finger_positions_and_durations(
    uint8_t slave_id,
    const uint16_t * positions,
    const uint16_t * durations,
    std::size_t count) override
  {
    if (!handler_ || !positions || !durations || count == 0)
    {
      return false;
    }

    ::stark_set_finger_positions_and_durations(
      handler_, slave_id, positions, durations, count);
    return true;
  }

protected:
  void set_handler(DeviceHandler * handler)
  {
    handler_ = handler;
  }

  void clear_handler()
  {
    handler_ = nullptr;
  }

  [[nodiscard]] DeviceHandler * handler() const
  {
    return handler_;
  }

  BraincoHandApi::DriverConfig & config_;

private:
  DeviceHandler * handler_{nullptr};
};

class ModbusSession final : public SessionBase
{
public:
  explicit ModbusSession(BraincoHandApi::DriverConfig & config)
  : SessionBase(config), handle_(nullptr, &::modbus_close)
  {
  }

  bool open() override
  {
    close();

    BraincoHandApi::ConnectionInfo connection{};
    connection.slave_id = config_.slave_id;

    if (config_.modbus.auto_detect)
    {
      const char * hint = config_.modbus.auto_detect_port.empty() ? nullptr : config_.modbus.auto_detect_port.c_str();
      DeviceConfigPtr detected{::auto_detect_modbus_revo2(hint, config_.modbus.auto_detect_quick)};
      if (!detected)
      {
        return false;
      }

      connection.port = detected->port_name ? detected->port_name : std::string{};
      connection.baudrate = detected->baudrate;
      connection.slave_id = detected->slave_id;
      config_.slave_id = detected->slave_id;
    }
    else
    {
      connection.port = config_.modbus.port;
      connection.baudrate = config_.modbus.baudrate;
    }

    DeviceHandler * raw = ::modbus_open(connection.port.c_str(), connection.baudrate);
    if (!raw)
    {
      return false;
    }

    handle_.reset(raw);
    set_handler(raw);
    resolved_connection_ = connection;
    return true;
  }

  void close() override
  {
    if (handle_)
    {
      handle_.reset();
    }
    clear_handler();
    resolved_connection_.reset();
  }

  [[nodiscard]] bool is_open() const override
  {
    return static_cast<bool>(handle_);
  }

  [[nodiscard]] std::optional<BraincoHandApi::ConnectionInfo> connection_info() const override
  {
    return resolved_connection_;
  }

private:
  using ModbusHandlePtr = std::unique_ptr<DeviceHandler, decltype(&::modbus_close)>;

  ModbusHandlePtr handle_;
  std::optional<BraincoHandApi::ConnectionInfo> resolved_connection_{};
};

class CanfdSession final : public SessionBase
{
public:
  explicit CanfdSession(BraincoHandApi::DriverConfig & config)
  : SessionBase(config), handle_(nullptr, &::free_device_handler)
  {
  }

  bool open() override
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
      BRAINCO_HAND_LOG_ERROR("VCI_OpenDevice failed (type=%u card=%u channel=%u)",
        canfd.device_type, canfd.card_index, canfd.channel_index);
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

  void close() override
  {
    CanfdSession * expected = this;
    const bool cleared = active_instance_.compare_exchange_strong(
      expected, nullptr, std::memory_order_acq_rel);
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

  [[nodiscard]] bool is_open() const override
  {
    return device_started_ && static_cast<bool>(handle_);
  }

  [[nodiscard]] std::optional<BraincoHandApi::ConnectionInfo> connection_info() const override
  {
    return std::nullopt;
  }

private:
  using CanfdHandlePtr = std::unique_ptr<DeviceHandler, decltype(&::free_device_handler)>;

  static int32_t tx_callback(uint8_t slave_id, uint32_t can_id, const uint8_t * data, uintptr_t data_len)
  {
    auto * session = active_instance_.load(std::memory_order_acquire);
    if (!session)
    {
      return -1;
    }
    return session->handle_tx(slave_id, can_id, data, data_len);
  }

  static int32_t rx_callback(uint8_t slave_id, uint32_t * can_id_out, uint8_t * data_out, uintptr_t * data_len_out)
  {
    auto * session = active_instance_.load(std::memory_order_acquire);
    if (!session)
    {
      return -1;
    }
    return session->handle_rx(slave_id, can_id_out, data_out, data_len_out);
  }

  static int32_t noop_tx_callback(uint8_t, uint32_t, const uint8_t *, uintptr_t)
  {
    return -1;
  }

  static int32_t noop_rx_callback(uint8_t, uint32_t *, uint8_t *, uintptr_t *)
  {
    return -1;
  }

  int32_t handle_tx(uint8_t slave_id, uint32_t can_id, const uint8_t * data, uintptr_t data_len)
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
    const auto length = static_cast<uint8_t>(std::min<uintptr_t>(data_len, static_cast<uintptr_t>(64)));
    message.hdr.len = length;
    for (uint8_t index = 0; index < length; ++index)
    {
      message.dat[index] = data[index];
    }

    const auto result = VCI_TransmitFD(
      config_.canfd.device_type,
      config_.canfd.card_index,
      config_.canfd.channel_index,
      &message,
      1);

    return result == 1 ? 0 : -1;
  }

  int32_t handle_rx(uint8_t slave_id, uint32_t * can_id_out, uint8_t * data_out, uintptr_t * data_len_out)
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
      config_.canfd.device_type,
      config_.canfd.card_index,
      config_.canfd.channel_index,
      rx_buffer_.data(),
      static_cast<uint32_t>(rx_buffer_.size()),
      config_.canfd.rx_wait_time);

    if (received < 1)
    {
      return -1;
    }

    const auto & frame = rx_buffer_.front();
    const auto payload_len = static_cast<uint8_t>(std::min<uintptr_t>(
      static_cast<uintptr_t>(frame.hdr.len), static_cast<uintptr_t>(64)));
    *can_id_out = frame.hdr.id;
    *data_len_out = payload_len;
    for (uint8_t index = 0; index < payload_len; ++index)
    {
      data_out[index] = frame.dat[index];
    }
    return 0;
  }

  CanfdHandlePtr handle_;
  bool device_started_{false};
  bool callbacks_registered_{false};
  std::vector<ZCAN_FD_MSG> rx_buffer_{};
  std::mutex instance_mutex_{};

  static std::atomic<CanfdSession *> active_instance_;
};

std::atomic<CanfdSession *> CanfdSession::active_instance_{nullptr};

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
      case Protocol::kCanfd:
        session = std::make_unique<CanfdSession>(config);
        break;
    }
  }
};

BraincoHandApi::BraincoHandApi()
: impl_(std::make_unique<Impl>())
{
  configure(DriverConfig{});
}

BraincoHandApi::BraincoHandApi(const DriverConfig & config)
: impl_(std::make_unique<Impl>())
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

  const auto protocol_type = (config.protocol == Protocol::kCanfd) ?
    STARK_PROTOCOL_TYPE_CAN_FD : STARK_PROTOCOL_TYPE_MODBUS;

  ::init_cfg(protocol_type, to_sdk_log_level(config.log_level));
  impl_->rebuild_session();
}

auto BraincoHandApi::open() -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
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

auto BraincoHandApi::ensure_physical_unit_mode(uint8_t slave_id) -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->ensure_physical_unit_mode(slave_id);
}

auto BraincoHandApi::get_motor_status(uint8_t slave_id) const -> std::optional<MotorStatus>
{
  if (!impl_ || !impl_->session)
  {
    return std::nullopt;
  }
  return impl_->session->get_motor_status(slave_id);
}

auto BraincoHandApi::set_finger_positions_and_durations(
  uint8_t slave_id,
  const uint16_t * positions,
  const uint16_t * durations,
  std::size_t count) -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->set_finger_positions_and_durations(slave_id, positions, durations, count);
}

auto BraincoHandApi::resolved_connection() const -> std::optional<ConnectionInfo>
{
  if (!impl_ || !impl_->session)
  {
    return std::nullopt;
  }
  return impl_->session->connection_info();
}

}  // namespace brainco_hand_driver