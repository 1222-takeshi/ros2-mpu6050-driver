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
#include <rclcpp/timer.hpp>
#include <memory>
#include <string>
#include <cmath>
#include <utility>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#define ACCEL_X_OUT 0x3b
#define ACCEL_Y_OUT 0x3d
#define ACCEL_Z_OUT 0x3f
#define TEMP_OUT 0x41
#define GYRO_X_OUT 0x43 
#define GYRO_Y_OUT 0x45
#define GYRO_Z_OUT 0x47

#define PWR_MGMT_1 0x6B  //PWR_MGMT_1
#define PWR_MGMT_2 0x6c  //PWR_MGMT_2
#define DEV_ADDR 0x68    // I2C

Mpu6050Driver::Mpu6050Driver(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("output", rclcpp::QoS{10});
  auto on_timer_ = std::bind(&Mpu6050Driver::onTimer, this);
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_)>>(
    this->get_clock(), 100ms, std::move(on_timer_), this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void Mpu6050Driver::onTimer()
{
  updateCurrentGyroData();
  updateCurrentAccelData();
  calcRollPitch();

}

void Mpu6050Driver::updateCurrentGyroData()
{
  int fd = wiringPiI2CSetup(DEV_ADDR);
  if (fd == -1){
    printf("ERROR : No device!!");
  }else{
    // float gx,gy,gz,ax,ay,az;
    sensor_msgs::msg::Imu msg;

    gyro_.push_back(get2data(fd, GYRO_X_OUT)/131.0);
    gyro_.push_back(get2data(fd, GYRO_Y_OUT)/131.0);
    gyro_.push_back(get2data(fd, GYRO_Z_OUT)/131.0);
    
    msg.header.stamp = now();
    msg.header.frame_id = "imu";
    msg.angular_velocity.x = gyro_[0];
    msg.angular_velocity.y = gyro_[1];
    msg.angular_velocity.z = gyro_[2];
    // msg.linear_acceleration.x = ax;
    // msg.linear_acceleration.y = ay;
    // msg.linear_acceleration.z = az;
    imu_pub_->publish(msg);
  }
}

void Mpu6050Driver::updateCurrentAccelData()
{
  int fd = wiringPiI2CSetup(DEV_ADDR);
  if (fd == -1){
    printf("ERROR : No device!!");
  }else{
    // float gx,gy,gz,ax,ay,az;
    sensor_msgs::msg::Imu msg;

    accel_.push_back(get2data(fd, ACCEL_X_OUT)/16384.0);
    accel_.push_back(get2data(fd, ACCEL_Y_OUT)/16384.0);
    accel_.push_back(get2data(fd, ACCEL_Z_OUT)/16384.0);
    
    msg.header.stamp = now();
    msg.header.frame_id = "imu";
    // msg.angular_velocity.x = gx;
    // msg.angular_velocity.y = gy;
    // msg.angular_velocity.z = gz;
    msg.linear_acceleration.x = accel_[0];
    msg.linear_acceleration.y = accel_[1];
    msg.linear_acceleration.z = accel_[2];
    imu_pub_->publish(msg);
  }
}
float Mpu6050Driver::get2data(int fd, unsigned int reg){
  unsigned int h_value = wiringPiI2CReadReg8(fd, reg);
  unsigned int l_value = wiringPiI2CReadReg8(fd, reg+1);
  float value = (h_value << 8) + l_value;
  if(value>=32768) return value-65534;  //32768=0x8000, 65534 = 0xFFFF 
  else return value;
}
