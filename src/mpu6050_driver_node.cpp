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

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/timer.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#define ACCEL_X_OUT 0x3b
#define ACCEL_Y_OUT 0x3d
#define ACCEL_Z_OUT 0x3f
#define TEMP_OUT    0x41  // Built-in temperature sensor (for diagnostics: thermal monitoring)
#define GYRO_X_OUT  0x43
#define GYRO_Y_OUT  0x45
#define GYRO_Z_OUT  0x47

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C  // Axis standby flags: bits[5:3]=gyro XYZ, bits[2:0]=accel XYZ (for diagnostics)
#define DEV_ADDR   0x68

// MPU6050 sensitivity: ±250°/s range → 131 LSB/(°/s)
static constexpr float GYRO_SENSITIVITY_LSB = 131.0f;
// MPU6050 sensitivity: ±2g range → 16384 LSB/g
static constexpr float ACCEL_SENSITIVITY_LSB = 16384.0f;
// Radians to degrees conversion factor
static constexpr float RAD_TO_DEG = 180.0f / M_PI;
static constexpr double DEFAULT_PUBLISH_RATE_HZ = 100.0;
static constexpr int64_t MIN_TIMER_PERIOD_MS = 1;
static constexpr float TEMP_WARN_C = 70.0f;
static constexpr float TEMP_ERROR_C = 85.0f;
static constexpr float MAX_ACCEL_G = 2.5f;
static constexpr float MAX_GYRO_DPS = 260.0f;
static constexpr double STALE_SAMPLE_PERIOD_MULTIPLIER = 3.0;

Mpu6050Driver::Mpu6050Driver(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options,
  II2CInterface * i2c)
: rclcpp::Node(node_name, node_options),
  diagnostic_updater_(this)
{
  if (i2c == nullptr) {
    owned_i2c_ = std::make_unique<WiringPiI2C>();
    i2c_ = owned_i2c_.get();
  } else {
    i2c_ = i2c;
  }

  declare_parameter<double>("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ);
  double rate_hz = get_parameter("publish_rate_hz").as_double();
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
    RCLCPP_ERROR(
      get_logger(), "Invalid publish_rate_hz %.3f; falling back to %.1f Hz", rate_hz,
      DEFAULT_PUBLISH_RATE_HZ);
    rate_hz = DEFAULT_PUBLISH_RATE_HZ;
  }

  auto period_ms = static_cast<int64_t>(1000.0 / rate_hz);
  if (period_ms < MIN_TIMER_PERIOD_MS) {
    RCLCPP_WARN(
      get_logger(), "publish_rate_hz %.3f is above timer resolution; using 1 ms period", rate_hz);
    period_ms = MIN_TIMER_PERIOD_MS;
  }
  publish_rate_hz_ = rate_hz;

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("output", rclcpp::QoS{10});
  roll_pitch_pub_ =
    create_publisher<geometry_msgs::msg::Vector3Stamped>("roll_pitch", rclcpp::QoS{10});
  auto on_timer_ = std::bind(&Mpu6050Driver::onTimer, this);
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_)>>(
    this->get_clock(), std::chrono::milliseconds(period_ms), std::move(on_timer_),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);

  diagnostic_updater_.setHardwareID("MPU6050");
  diagnostic_updater_.add("Hardware Status", this, &Mpu6050Driver::checkHardwareStatus);
  diagnostic_updater_.add("Data Status", this, &Mpu6050Driver::checkDataStatus);

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
  gyro_ = {{
    get2data(fd_, GYRO_X_OUT) / GYRO_SENSITIVITY_LSB,
    get2data(fd_, GYRO_Y_OUT) / GYRO_SENSITIVITY_LSB,
    get2data(fd_, GYRO_Z_OUT) / GYRO_SENSITIVITY_LSB,
  }};
}

void Mpu6050Driver::updateCurrentAccelData()
{
  accel_ = {{
    get2data(fd_, ACCEL_X_OUT) / ACCEL_SENSITIVITY_LSB,
    get2data(fd_, ACCEL_Y_OUT) / ACCEL_SENSITIVITY_LSB,
    get2data(fd_, ACCEL_Z_OUT) / ACCEL_SENSITIVITY_LSB,
  }};
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
  last_sample_time_ = now();
  latest_sample_valid_ = isLatestSampleValid();
  ++sample_count_;

  msg.header.stamp = last_sample_time_;
  msg.header.frame_id = "imu";
  msg.angular_velocity.x = gyro_[0];
  msg.angular_velocity.y = gyro_[1];
  msg.angular_velocity.z = gyro_[2];
  msg.linear_acceleration.x = accel_[0];
  msg.linear_acceleration.y = accel_[1];
  msg.linear_acceleration.z = accel_[2];
  imu_pub_->publish(msg);
}

void Mpu6050Driver::calcRollPitch()
{
  const float roll = std::atan2(accel_[1], accel_[2]) * RAD_TO_DEG;
  const float pitch = std::atan2(-accel_[0], std::hypot(accel_[1], accel_[2])) * RAD_TO_DEG;

  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = "imu";
  msg.vector.x = roll;
  msg.vector.y = pitch;
  msg.vector.z = 0.0f;
  roll_pitch_pub_->publish(msg);
}

void Mpu6050Driver::checkHardwareStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  stat.add("I2C connection", fd_ == -1 ? "not initialized" : "ok");
  stat.add("Gyro sensitivity", "131 LSB/(deg/s) [+-250 deg/s]");
  stat.add("Accel sensitivity", "16384 LSB/g [+-2g]");

  if (fd_ == -1) {
    stat.summary(DiagnosticStatus::ERROR, "I2C not initialized");
    return;
  }

  const float temp_c = get2data(fd_, TEMP_OUT) / 340.0f + 36.53f;
  const int pwr_mgmt_2 = i2c_->readReg8(fd_, PWR_MGMT_2);
  const bool all_axes_active = (pwr_mgmt_2 & 0x3F) == 0x00;

  stat.add("Chip temperature C", temp_c);
  stat.add("PWR_MGMT_2", pwr_mgmt_2);
  stat.add("Axis power state", all_axes_active ? "all active" : "some axes in standby");

  if (temp_c > TEMP_ERROR_C) {
    stat.summary(DiagnosticStatus::ERROR, "Chip overheating");
  } else if (temp_c > TEMP_WARN_C || !all_axes_active) {
    stat.summary(DiagnosticStatus::WARN, "Sensor degraded");
  } else {
    stat.summary(DiagnosticStatus::OK, "Hardware OK");
  }
}

void Mpu6050Driver::checkDataStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  stat.add("Configured publish_rate_hz", publish_rate_hz_);
  stat.add("Published samples", sample_count_);

  if (fd_ == -1) {
    stat.summary(DiagnosticStatus::ERROR, "I2C not initialized");
    return;
  }
  if (sample_count_ == 0) {
    stat.summary(DiagnosticStatus::WARN, "No IMU samples published yet");
    return;
  }

  const double sample_age_s = (now() - last_sample_time_).seconds();
  const double expected_period_s = 1.0 / publish_rate_hz_;
  stat.add("Latest sample age sec", sample_age_s);
  stat.add("Latest sample valid", latest_sample_valid_);

  if (!latest_sample_valid_) {
    stat.summary(DiagnosticStatus::WARN, "Latest IMU sample outside expected range");
  } else if (sample_age_s > expected_period_s * STALE_SAMPLE_PERIOD_MULTIPLIER) {
    stat.summary(DiagnosticStatus::WARN, "No recent IMU sample");
  } else {
    stat.summary(DiagnosticStatus::OK, "Data OK");
  }
}

bool Mpu6050Driver::isLatestSampleValid() const
{
  for (const auto value : accel_) {
    if (!std::isfinite(value) || std::fabs(value) > MAX_ACCEL_G) {
      return false;
    }
  }
  for (const auto value : gyro_) {
    if (!std::isfinite(value) || std::fabs(value) > MAX_GYRO_DPS) {
      return false;
    }
  }
  return true;
}
