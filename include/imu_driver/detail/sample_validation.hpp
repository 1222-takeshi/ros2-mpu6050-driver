// Copyright 2026 Takeshi Miura
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

#ifndef IMU_DRIVER__DETAIL__SAMPLE_VALIDATION_HPP_
#define IMU_DRIVER__DETAIL__SAMPLE_VALIDATION_HPP_

#include <array>
#include <cmath>

namespace imu_driver
{
namespace detail
{
inline bool isImuSampleValid(
  const std::array<double, 3> & accel, const std::array<double, 3> & gyro,
  double max_accel, double max_gyro)
{
  for (const auto value : accel) {
    if (!std::isfinite(value) || std::fabs(value) > max_accel) {
      return false;
    }
  }
  for (const auto value : gyro) {
    if (!std::isfinite(value) || std::fabs(value) > max_gyro) {
      return false;
    }
  }
  return true;
}
}  // namespace detail
}  // namespace imu_driver

#endif  // IMU_DRIVER__DETAIL__SAMPLE_VALIDATION_HPP_
