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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

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
  void updateCurrentGyroData();
  void updateCurrentAccelData();
  void calcRollPitch();
  void imuDataPublish();
  float get2data(int fd, unsigned int reg);

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr roll_pitch_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<float> gyro_;
  std::vector<float> accel_;

  std::unique_ptr<II2CInterface> owned_i2c_;  ///< Owns the instance when created internally.
  II2CInterface * i2c_;                        ///< Non-owning pointer used for all I2C calls.
  int fd_ = -1;                                ///< I2C file descriptor; -1 means not initialized.
};

#endif  // IMU_DRIVER__MPU6050_DRIVER_HPP_
