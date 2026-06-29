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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

static constexpr unsigned int ACCEL_X_OUT = 0x3b;
static constexpr unsigned int ACCEL_Y_OUT = 0x3d;
static constexpr unsigned int ACCEL_Z_OUT = 0x3f;
static constexpr unsigned int TEMP_OUT = 0x41;  // Built-in temperature sensor for diagnostics.
static constexpr unsigned int GYRO_X_OUT = 0x43;
static constexpr unsigned int GYRO_Y_OUT = 0x45;
static constexpr unsigned int GYRO_Z_OUT = 0x47;

static constexpr int PWR_MGMT_1 = 0x6B;
// Axis standby flags: bits[5:3]=gyro XYZ, bits[2:0]=accel XYZ.
static constexpr unsigned int PWR_MGMT_2 = 0x6C;
static constexpr int DEV_ADDR = 0x68;

// MPU6050 sensitivity: ±250°/s range → 131 LSB/(°/s)
static constexpr float GYRO_SENSITIVITY_LSB = 131.0f;
// MPU6050 sensitivity: ±2g range → 16384 LSB/g
static constexpr float ACCEL_SENSITIVITY_LSB = 16384.0f;
// Unit conversion factors
static constexpr float RAD_TO_DEG = 180.0f / M_PI;
static constexpr float DEG_TO_RAD = M_PI / 180.0f;
static constexpr float STANDARD_GRAVITY = 9.80665f;
static constexpr double DEFAULT_PUBLISH_RATE_HZ = 100.0;
static constexpr int64_t MIN_TIMER_PERIOD_MS = 1;
static constexpr float TEMP_WARN_C = 70.0f;
static constexpr float TEMP_ERROR_C = 85.0f;
static constexpr float MAX_ACCEL_MPS2 = 2.5f * STANDARD_GRAVITY;
static constexpr float MAX_GYRO_RADPS = 260.0f * DEG_TO_RAD;
static constexpr double STALE_SAMPLE_PERIOD_MULTIPLIER = 3.0;
static constexpr std::size_t COVARIANCE_SIZE = 9;

static std::vector<double> defaultCovarianceParameter()
{
  return std::vector<double>(COVARIANCE_SIZE, 0.0);
}

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
  declare_parameter<std::vector<double>>(
    "angular_velocity_covariance", defaultCovarianceParameter());
  declare_parameter<std::vector<double>>(
    "linear_acceleration_covariance", defaultCovarianceParameter());
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
  angular_velocity_covariance_ = getCovarianceParameter("angular_velocity_covariance");
  linear_acceleration_covariance_ = getCovarianceParameter("linear_acceleration_covariance");

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
  latest_sample_read_ok_ = updateCurrentGyroData() && updateCurrentAccelData();
  if (!latest_sample_read_ok_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "I2C register read failed, skipping update");
    return;
  }
  calcRollPitch();
  imuDataPublish();
}

bool Mpu6050Driver::updateCurrentGyroData()
{
  float gyro_x = 0.0f;
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;
  const bool read_ok =
    read2data(fd_, GYRO_X_OUT, &gyro_x) &&
    read2data(fd_, GYRO_Y_OUT, &gyro_y) &&
    read2data(fd_, GYRO_Z_OUT, &gyro_z);
  if (!read_ok) {
    return false;
  }
  gyro_ = {{
    gyro_x / GYRO_SENSITIVITY_LSB * DEG_TO_RAD,
    gyro_y / GYRO_SENSITIVITY_LSB * DEG_TO_RAD,
    gyro_z / GYRO_SENSITIVITY_LSB * DEG_TO_RAD,
  }};
  return true;
}

bool Mpu6050Driver::updateCurrentAccelData()
{
  float accel_x = 0.0f;
  float accel_y = 0.0f;
  float accel_z = 0.0f;
  const bool read_ok =
    read2data(fd_, ACCEL_X_OUT, &accel_x) &&
    read2data(fd_, ACCEL_Y_OUT, &accel_y) &&
    read2data(fd_, ACCEL_Z_OUT, &accel_z);
  if (!read_ok) {
    return false;
  }
  accel_ = {{
    accel_x / ACCEL_SENSITIVITY_LSB * STANDARD_GRAVITY,
    accel_y / ACCEL_SENSITIVITY_LSB * STANDARD_GRAVITY,
    accel_z / ACCEL_SENSITIVITY_LSB * STANDARD_GRAVITY,
  }};
  return true;
}

bool Mpu6050Driver::readReg8Checked(int fd, unsigned int reg, int * value)
{
  const int raw_value = i2c_->readReg8(fd, static_cast<int>(reg));
  if (raw_value < 0 || raw_value > 0xFF) {
    return false;
  }
  *value = raw_value;
  return true;
}

bool Mpu6050Driver::read2data(int fd, unsigned int reg, float * value)
{
  int h_value = 0;
  int l_value = 0;
  if (!readReg8Checked(fd, reg, &h_value) || !readReg8Checked(fd, reg + 1, &l_value)) {
    return false;
  }

  const int raw_value = (h_value << 8) + l_value;
  // Convert from unsigned 16-bit to signed using two's complement.
  // Values >= 32768 (0x8000) represent negative numbers.
  *value = static_cast<float>(raw_value >= 32768 ? raw_value - 65536 : raw_value);
  return true;
}

void Mpu6050Driver::imuDataPublish()
{
  sensor_msgs::msg::Imu msg;
  last_sample_time_ = now();
  latest_sample_valid_ = isLatestSampleValid();
  ++sample_count_;

  msg.header.stamp = last_sample_time_;
  msg.header.frame_id = "imu";
  msg.orientation_covariance[0] = -1.0;
  msg.angular_velocity.x = gyro_[0];
  msg.angular_velocity.y = gyro_[1];
  msg.angular_velocity.z = gyro_[2];
  msg.angular_velocity_covariance = angular_velocity_covariance_;
  msg.linear_acceleration.x = accel_[0];
  msg.linear_acceleration.y = accel_[1];
  msg.linear_acceleration.z = accel_[2];
  msg.linear_acceleration_covariance = linear_acceleration_covariance_;
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
  stat.add("IMU output angular velocity unit", "rad/s");
  stat.add("IMU output linear acceleration unit", "m/s^2");

  if (fd_ == -1) {
    stat.summary(DiagnosticStatus::ERROR, "I2C not initialized");
    return;
  }

  float temp_raw = 0.0f;
  int pwr_mgmt_2 = 0;
  if (!read2data(fd_, TEMP_OUT, &temp_raw) || !readReg8Checked(fd_, PWR_MGMT_2, &pwr_mgmt_2)) {
    stat.add("I2C register reads", "failed");
    stat.summary(DiagnosticStatus::ERROR, "I2C read failed");
    return;
  }

  const float temp_c = temp_raw / 340.0f + 36.53f;
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
  stat.add("Latest I2C read ok", latest_sample_read_ok_);
  if (!latest_sample_read_ok_) {
    stat.summary(DiagnosticStatus::ERROR, "I2C read failed");
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
    if (!std::isfinite(value) || std::fabs(value) > MAX_ACCEL_MPS2) {
      return false;
    }
  }
  for (const auto value : gyro_) {
    if (!std::isfinite(value) || std::fabs(value) > MAX_GYRO_RADPS) {
      return false;
    }
  }
  return true;
}

std::array<double, 9> Mpu6050Driver::getCovarianceParameter(const std::string & parameter_name)
{
  std::array<double, 9> covariance{};
  const auto values = get_parameter(parameter_name).as_double_array();
  if (values.size() != covariance.size()) {
    RCLCPP_ERROR(
      get_logger(), "Parameter '%s' must contain exactly %zu values; using all zeros",
      parameter_name.c_str(), covariance.size());
    return covariance;
  }
  std::copy(values.begin(), values.end(), covariance.begin());
  return covariance;
}
