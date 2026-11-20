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

#include "revo2_driver/brainco_hand_api.hpp"
#include "stark-sdk.h"

namespace revo2_driver
{

// Custom deleters for SDK resource management
struct DeviceConfigDeleter
{
  void operator()(CDeviceConfig * config) const;
};

struct DeviceInfoDeleter
{
  void operator()(CDeviceInfo * info) const;
};

struct MotorStatusDeleter
{
  void operator()(CMotorStatusData * data) const;
};

struct TouchFingerDataDeleter
{
  void operator()(CTouchFingerData * data) const;
};

// Smart pointer aliases for SDK resources
using DeviceConfigPtr = std::unique_ptr<CDeviceConfig, DeviceConfigDeleter>;
using DeviceInfoPtr = std::unique_ptr<CDeviceInfo, DeviceInfoDeleter>;
using MotorStatusPtr = std::unique_ptr<CMotorStatusData, MotorStatusDeleter>;
using TouchFingerDataPtr = std::unique_ptr<CTouchFingerData, TouchFingerDataDeleter>;

// Convert BraincoLogLevel to SDK LogLevel
auto to_sdk_log_level(BraincoLogLevel level) -> LogLevel;

}  // namespace revo2_driver
