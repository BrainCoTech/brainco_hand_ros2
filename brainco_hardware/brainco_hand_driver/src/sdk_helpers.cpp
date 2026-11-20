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

#include "revo2_driver/sdk_helpers.hpp"

#include "stark-sdk.h"

namespace revo2_driver
{

void DeviceConfigDeleter::operator()(CDeviceConfig * config) const
{
  if (config)
  {
    ::free_device_config(config);
  }
}

void DeviceInfoDeleter::operator()(CDeviceInfo * info) const
{
  if (info)
  {
    ::free_device_info(info);
  }
}

void MotorStatusDeleter::operator()(CMotorStatusData * data) const
{
  if (data)
  {
    ::free_motor_status_data(data);
  }
}

void TouchFingerDataDeleter::operator()(CTouchFingerData * data) const
{
  if (data)
  {
    ::free_touch_finger_data(data);
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

}  // namespace revo2_driver
