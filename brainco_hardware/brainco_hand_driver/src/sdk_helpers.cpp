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

#include "brainco_hand_driver/sdk_helpers.hpp"

#include "stark-sdk.h"

namespace brainco_hand_driver
{

void DeviceConfigDeleter::operator()(DeviceConfig * config) const
{
  if (config)
  {
    ::free_device_config(config);
  }
}

void DeviceInfoDeleter::operator()(DeviceInfo * info) const
{
  if (info)
  {
    ::free_device_info(info);
  }
}

void MotorStatusDeleter::operator()(MotorStatusData * data) const
{
  if (data)
  {
    ::free_motor_status_data(data);
  }
}

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

}  // namespace brainco_hand_driver

