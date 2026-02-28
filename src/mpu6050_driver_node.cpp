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

#include "imu_driver/mpu6050_driver.hpp"
#include "imu_driver/wiringpi_i2c.hpp"

#include <rclcpp/timer.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#define ACCEL_X_OUT 0x3b
#define ACCEL_Y_OUT 0x3d
#define ACCEL_Z_OUT 0x3f
#define TEMP_OUT    0x41
#define GYRO_X_OUT  0x43
#define GYRO_Y_OUT  0x45
#define GYRO_Z_OUT  0x47

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define DEV_ADDR   0x68

Mpu6050Driver::Mpu6050Driver(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options,
  II2CInterface * i2c)
: rclcpp::Node(node_name, node_options)
{
  if (i2c == nullptr) {
    owned_i2c_ = std::make_unique<WiringPiI2C>();
    i2c_ = owned_i2c_.get();
  } else {
    i2c_ = i2c;
  }

  using std::chrono_literals::operator""ms;
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("output", rclcpp::QoS{10});
  auto on_timer_ = std::bind(&Mpu6050Driver::onTimer, this);
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_)>>(
    this->get_clock(), 100ms, std::move(on_timer_),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);

  initializeI2C();
}

void Mpu6050Driver::initializeI2C()
{
  fd_ = i2c_->setup(DEV_ADDR);
  if (fd_ == -1) {
    RCLCPP_ERROR(get_logger(), "I2C setup failed: no device found at address 0x%02X", DEV_ADDR);
    return;
  }
  // Wake the MPU6050 from sleep mode (SLEEP bit in PWR_MGMT_1 is set on power-on).
  i2c_->writeReg8(fd_, PWR_MGMT_1, 0x00);
}

void Mpu6050Driver::onTimer()
{
  if (fd_ == -1) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "I2C not initialized, skipping update");
    return;
  }
  updateCurrentGyroData();
  updateCurrentAccelData();
  calcRollPitch();
  imuDataPublish();
}

void Mpu6050Driver::updateCurrentGyroData()
{
  gyro_.push_back(get2data(fd_, GYRO_X_OUT) / 131.0f);
  gyro_.push_back(get2data(fd_, GYRO_Y_OUT) / 131.0f);
  gyro_.push_back(get2data(fd_, GYRO_Z_OUT) / 131.0f);
}

void Mpu6050Driver::updateCurrentAccelData()
{
  accel_.push_back(get2data(fd_, ACCEL_X_OUT) / 16384.0f);
  accel_.push_back(get2data(fd_, ACCEL_Y_OUT) / 16384.0f);
  accel_.push_back(get2data(fd_, ACCEL_Z_OUT) / 16384.0f);
}

float Mpu6050Driver::get2data(int fd, unsigned int reg)
{
  unsigned int h_value = static_cast<unsigned int>(i2c_->readReg8(fd, reg));
  unsigned int l_value = static_cast<unsigned int>(i2c_->readReg8(fd, reg + 1));
  float value = static_cast<float>((h_value << 8) + l_value);
  // Convert from unsigned 16-bit to signed using two's complement.
  // Values >= 32768 (0x8000) represent negative numbers.
  if (value >= 32768.0f) {
    return value - 65536.0f;  // 0x10000 = 65536
  }
  return value;
}

void Mpu6050Driver::imuDataPublish()
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = now();
  msg.header.frame_id = "imu";
  msg.angular_velocity.x = gyro_[0];
  msg.angular_velocity.y = gyro_[1];
  msg.angular_velocity.z = gyro_[2];
  msg.linear_acceleration.x = accel_[0];
  msg.linear_acceleration.y = accel_[1];
  msg.linear_acceleration.z = accel_[2];
  imu_pub_->publish(msg);
  gyro_.clear();
  accel_.clear();
}

void Mpu6050Driver::calcRollPitch()
{
  // TODO(user): roll/pitch are computed but not yet published.
  float roll = std::atan(accel_[1] / accel_[2]) * 57.324f;
  float pitch =
    std::atan(-accel_[0] / std::sqrt(accel_[1] * accel_[1] + accel_[2] * accel_[2])) * 57.324f;
  (void)roll;
  (void)pitch;
}
