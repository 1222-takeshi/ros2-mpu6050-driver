// Copyright 2021 Takeshi Miura
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

#ifndef IMU_DRIVER__MPU6050_DRIVER_HPP_
#define IMU_DRIVER__MPU6050_DRIVER_HPP_

#include "imu_driver/i2c_interface.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

class Mpu6050Driver : public rclcpp::Node
{
public:
  /// Constructor.
  /// \param i2c  I2C backend to use. Pass nullptr to use the production wiringPi implementation.
  Mpu6050Driver(
    const std::string & node_name,
    const rclcpp::NodeOptions & options,
    II2CInterface * i2c = nullptr);

private:
  void initializeI2C();
  void onTimer();
  bool updateCurrentGyroData();
  bool updateCurrentAccelData();
  void calcRollPitch();
  void imuDataPublish();
  void checkHardwareStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void checkDataStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
  bool isLatestSampleValid() const;
  bool readReg8Checked(int fd, unsigned int reg, int * value);
  bool read2data(int fd, unsigned int reg, float * value);
  std::array<double, 3> parseBiasParameter(
    const std::string & parameter_name,
    const std::vector<double> & values) const;
  std::array<double, 9> parseCovarianceParameter(
    const std::string & parameter_name,
    const std::vector<double> & values) const;

  diagnostic_updater::Updater diagnostic_updater_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr roll_pitch_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<double, 3> gyro_{};
  std::array<double, 3> accel_{};
  std::array<double, 3> angular_velocity_bias_{};
  std::array<double, 3> linear_acceleration_bias_{};
  std::array<double, 9> angular_velocity_covariance_{};
  std::array<double, 9> linear_acceleration_covariance_{};
  rclcpp::Time last_sample_time_;
  double publish_rate_hz_ = 0.0;
  double expected_sample_interval_s_ = 0.0;
  double sample_interval_tolerance_s_ = 0.0;
  double latest_sample_interval_s_ = 0.0;
  double latest_sample_interval_error_s_ = 0.0;
  double max_sample_interval_error_s_ = 0.0;
  std::size_t sample_count_ = 0;
  bool latest_sample_read_ok_ = true;
  bool latest_sample_valid_ = false;
  bool sample_interval_available_ = false;

  std::unique_ptr<II2CInterface> owned_i2c_;  ///< Owns the instance when created internally.
  II2CInterface * i2c_;                        ///< Non-owning pointer used for all I2C calls.
  int fd_ = -1;                                ///< I2C file descriptor; -1 means not initialized.
};

#endif  // IMU_DRIVER__MPU6050_DRIVER_HPP_
