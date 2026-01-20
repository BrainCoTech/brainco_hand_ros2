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

#include "brainco_hand_driver/brainco_hand_api.hpp"
#include "stark-sdk.h"

// Forward declarations from Stark SDK
struct DeviceConfig;
struct DeviceHandler;
struct DeviceInfo;
struct MotorStatusData;

namespace brainco_hand_driver
{

// Custom deleters for SDK resource management
struct DeviceConfigDeleter
{
  void operator()(DeviceConfig * config) const;
};

struct DeviceInfoDeleter
{
  void operator()(DeviceInfo * info) const;
};

struct MotorStatusDeleter
{
  void operator()(MotorStatusData * data) const;
};

// Smart pointer aliases for SDK resources
using DeviceConfigPtr = std::unique_ptr<DeviceConfig, DeviceConfigDeleter>;
using DeviceInfoPtr = std::unique_ptr<DeviceInfo, DeviceInfoDeleter>;
using MotorStatusPtr = std::unique_ptr<MotorStatusData, MotorStatusDeleter>;

// Convert BraincoLogLevel to SDK LogLevel
auto to_sdk_log_level(BraincoLogLevel level) -> LogLevel;

}  // namespace brainco_hand_driver
