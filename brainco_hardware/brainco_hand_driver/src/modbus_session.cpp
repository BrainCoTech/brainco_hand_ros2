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

#include "brainco_hand_driver/modbus_session.hpp"

#include "brainco_hand_driver/logger_macros.hpp"
#include "brainco_hand_driver/sdk_helpers.hpp"

#include <filesystem>
#include <fstream>
#include <vector>
#include <string>

namespace brainco_hand_driver
{

ModbusSession::ModbusSession(BraincoHandApi::DriverConfig & config)
: SessionBase(config), handle_(nullptr, &modbus_close)
{
}

namespace
{
// Helper function to scan for available serial ports
std::vector<std::string> scan_serial_ports(const std::string & hint = "")
{
  std::vector<std::string> ports;
  
  // Common serial port patterns
  std::vector<std::string> patterns;
  if (!hint.empty())
  {
    patterns.push_back(hint);
  }
  else
  {
    patterns = {"/dev/ttyUSB", "/dev/ttyACM", "/dev/ttyS"};
  }
  
  for (const auto & pattern : patterns)
  {
    // Try ports from 0 to 15
    for (int i = 0; i < 16; ++i)
    {
      std::string port_path = pattern + std::to_string(i);
      if (std::filesystem::exists(port_path))
      {
        ports.push_back(port_path);
      }
    }
  }
  
  return ports;
}
}  // namespace

bool ModbusSession::open()
{
  close();

  BraincoHandApi::ConnectionInfo connection{};
  connection.slave_id = config_.slave_id;

  if (config_.modbus.auto_detect)
  {
    // Save the requested slave_id before detection
    const uint8_t requested_slave_id = config_.slave_id;

    BRAINCO_HAND_LOG_INFO(
      "Starting auto-detection for Modbus Revo2 device (requested slave_id: %u, quick: %s)",
      requested_slave_id, config_.modbus.auto_detect_quick ? "true" : "false");

    const char * hint =
      config_.modbus.auto_detect_port.empty() ? nullptr : config_.modbus.auto_detect_port.c_str();
    if (hint)
    {
      BRAINCO_HAND_LOG_INFO("Auto-detect port hint: %s", hint);
    }

    // Scan all available serial ports
    std::vector<std::string> available_ports = scan_serial_ports(hint ? std::string(hint) : "");
    
    if (available_ports.empty())
    {
      BRAINCO_HAND_LOG_WARN("No serial ports found. Trying SDK auto-detection...");
    }
    else
    {
      BRAINCO_HAND_LOG_INFO("Found %zu serial port(s), scanning for Revo2 devices...", available_ports.size());
    }

    DeviceConfigPtr detected{nullptr};
    std::string used_port;
    std::vector<std::pair<std::string, DeviceConfigPtr>> matching_devices;  // Devices matching requested slave_id
    std::vector<std::pair<std::string, DeviceConfigPtr>> other_devices;      // Other devices

    // First, try SDK's auto-detection (it's faster and more reliable)
    DeviceConfigPtr sdk_detected{::auto_detect_modbus_revo2(hint, config_.modbus.auto_detect_quick)};
    if (sdk_detected)
    {
      std::string detected_port = sdk_detected->port_name ? sdk_detected->port_name : std::string{};
      uint8_t detected_slave_id = sdk_detected->slave_id;
      
      // Try to open the detected port
      DeviceHandler * test_handle = ::modbus_open(detected_port.c_str(), sdk_detected->baudrate);
      if (test_handle)
      {
        // Successfully opened, check if slave_id matches
        ::modbus_close(test_handle);
        
        // Create a new DeviceConfigPtr by re-detecting on the specific port to ensure we have the correct config
        DeviceConfigPtr port_config{::auto_detect_modbus_revo2(detected_port.c_str(), config_.modbus.auto_detect_quick)};
        if (port_config)
        {
          if (requested_slave_id == 0 || detected_slave_id == requested_slave_id)
          {
            matching_devices.push_back({detected_port, std::move(port_config)});
          }
          else
          {
            other_devices.push_back({detected_port, std::move(port_config)});
          }
          BRAINCO_HAND_LOG_INFO("SDK auto-detection found device on %s (slave_id: %u)", detected_port.c_str(), detected_slave_id);
        }
      }
      else
      {
        // Port is in use, continue scanning other ports
        BRAINCO_HAND_LOG_WARN(
          "SDK detected device on %s but port is in use. Scanning other ports...",
          detected_port.c_str());
      }
    }

    // Scan all available ports manually to find all devices
    for (const auto & port : available_ports)
    {
      // Skip if this is the port SDK already detected (and it's in use)
      if (sdk_detected && sdk_detected->port_name && 
          std::string(sdk_detected->port_name) == port)
      {
        continue;
      }

      BRAINCO_HAND_LOG_INFO("Scanning port: %s", port.c_str());
      
      // Try to detect device on this specific port
      DeviceConfigPtr port_detected{::auto_detect_modbus_revo2(port.c_str(), config_.modbus.auto_detect_quick)};
      if (port_detected && port_detected->port_name && 
          std::string(port_detected->port_name) == port)
      {
        uint8_t detected_slave_id = port_detected->slave_id;
        
        // Try to open the port to verify it's available
        DeviceHandler * test_handle = ::modbus_open(port.c_str(), port_detected->baudrate);
        if (test_handle)
        {
          // Successfully opened, categorize by slave_id
          ::modbus_close(test_handle);
          
          if (requested_slave_id == 0 || detected_slave_id == requested_slave_id)
          {
            matching_devices.push_back({port, std::move(port_detected)});
            BRAINCO_HAND_LOG_INFO("Found matching device on port: %s (slave_id: %u)", port.c_str(), detected_slave_id);
          }
          else
          {
            other_devices.push_back({port, std::move(port_detected)});
            BRAINCO_HAND_LOG_INFO("Found device on port: %s (slave_id: %u, not matching requested %u)", 
                                  port.c_str(), detected_slave_id, requested_slave_id);
          }
        }
        else
        {
          BRAINCO_HAND_LOG_INFO("Port %s is in use, skipping...", port.c_str());
        }
      }
    }

    // Prioritize devices matching the requested slave_id
    if (!matching_devices.empty())
    {
      // Use the first matching device
      detected = std::move(matching_devices[0].second);
      used_port = matching_devices[0].first;
      BRAINCO_HAND_LOG_INFO("Selected device with matching slave_id from port: %s", used_port.c_str());
    }
    else if (!other_devices.empty() && requested_slave_id != 0)
    {
      // If no matching device found but other devices exist, warn and use the first one
      BRAINCO_HAND_LOG_WARN(
        "No device found with requested slave_id (%u). Found %zu device(s) with different slave_id(s). "
        "This may indicate:"
        "\n  1. The device with slave_id %u is not connected"
        "\n  2. Wrong hand is connected (left/right)"
        "\n  3. Device slave_id has been changed",
        requested_slave_id, other_devices.size(), requested_slave_id);
      
      // List all found devices
      for (const auto & dev : other_devices)
      {
        if (dev.second)
        {
          BRAINCO_HAND_LOG_INFO("  - Port: %s, slave_id: %u", dev.first.c_str(), dev.second->slave_id);
        }
      }
      
      BRAINCO_HAND_LOG_WARN("Using first available device, but slave_id may not match your configuration.");
      detected = std::move(other_devices[0].second);
      used_port = other_devices[0].first;
    }
    else if (!other_devices.empty())
    {
      // No specific slave_id requested, use first available device
      detected = std::move(other_devices[0].second);
      used_port = other_devices[0].first;
      BRAINCO_HAND_LOG_INFO("No specific slave_id requested, using first available device from port: %s", used_port.c_str());
    }

    if (!detected)
    {
      std::string error_msg = "Auto-detection failed: No available Revo2 device found.";
      if (requested_slave_id != 0)
      {
        error_msg += " No device found with slave_id " + std::to_string(requested_slave_id) + ".";
      }
      error_msg += " Please check:"
        "\n  1. Device is connected and powered on"
        "\n  2. USB cable is properly connected"
        "\n  3. Device permissions (try: sudo chmod 666 /dev/ttyUSB*)"
        "\n  4. For dual-hand setup, ensure both devices are connected and have different slave_id"
        "\n  5. Some ports may be in use by other hardware instances";
      if (requested_slave_id != 0)
      {
        error_msg += "\n  6. Verify that the device with slave_id " + std::to_string(requested_slave_id) + " is connected";
      }
      BRAINCO_HAND_LOG_ERROR("%s", error_msg.c_str());
      return false;
    }

    connection.port = detected->port_name ? detected->port_name : std::string{};
    connection.baudrate = detected->baudrate;
    connection.slave_id = detected->slave_id;
    config_.slave_id = detected->slave_id;

    BRAINCO_HAND_LOG_INFO(
      "Auto-detection successful: port=%s baudrate=%u detected_slave_id=%u", connection.port.c_str(),
      connection.baudrate, connection.slave_id);

    // For dual-hand scenarios: verify if detected slave_id matches the requested one
    if (requested_slave_id != 0 && requested_slave_id != connection.slave_id)
    {
      BRAINCO_HAND_LOG_WARN(
        "Detected device slave_id (%u) does not match requested slave_id (%u). "
        "This may indicate:"
        "\n  1. The device connected is not the expected hand (left/right)"
        "\n  2. Device slave_id has been changed"
        "\n  Using detected slave_id: %u",
        connection.slave_id, requested_slave_id, connection.slave_id);
    }
    else if (requested_slave_id != 0 && requested_slave_id == connection.slave_id)
    {
      BRAINCO_HAND_LOG_INFO(
        "Detected device slave_id (%u) matches requested slave_id - correct hand detected!",
        connection.slave_id);
    }
  }
  else
  {
    connection.port = config_.modbus.port;
    connection.baudrate = config_.modbus.baudrate;

    // Check if port exists before attempting to open
    if (!std::filesystem::exists(connection.port))
    {
      BRAINCO_HAND_LOG_ERROR(
        "Port %s does not exist. Please check:"
        "\n  1. Device is connected"
        "\n  2. Port path is correct"
        "\n  3. Consider enabling auto_detect: true",
        connection.port.c_str());
      return false;
    }

    BRAINCO_HAND_LOG_INFO(
      "Using manual configuration: port=%s baudrate=%u slave_id=%u", connection.port.c_str(),
      connection.baudrate, connection.slave_id);
  }

  // Open the connection
  DeviceHandler * raw = ::modbus_open(connection.port.c_str(), connection.baudrate);
  if (!raw)
  {
    BRAINCO_HAND_LOG_ERROR(
      "Failed to open Modbus connection on %s (baudrate: %u). Possible causes:"
      "\n  1. Device is not responding"
      "\n  2. Incorrect baudrate"
      "\n  3. Device is already in use by another process"
      "\n  4. Permission denied (try: sudo chmod 666 %s)",
      connection.port.c_str(), connection.baudrate, connection.port.c_str());
    return false;
  }

  handle_.reset(raw);
  set_handler(raw);
  resolved_connection_ = connection;
  BRAINCO_HAND_LOG_INFO(
    "Modbus connection established: port=%s baudrate=%u slave_id=%u", connection.port.c_str(),
    connection.baudrate, connection.slave_id);
  return true;
}

void ModbusSession::close()
{
  if (handle_)
  {
    handle_.reset();
  }
  clear_handler();
  resolved_connection_.reset();
}

bool ModbusSession::is_open() const { return static_cast<bool>(handle_); }

std::optional<BraincoHandApi::ConnectionInfo> ModbusSession::connection_info() const
{
  return resolved_connection_;
}

}  // namespace brainco_hand_driver

