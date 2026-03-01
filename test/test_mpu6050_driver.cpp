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

#include <gtest/gtest.h>

#include "imu_driver/i2c_interface.hpp"
#include "imu_driver/mpu6050_driver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <map>
#include <thread>

// ---------------------------------------------------------------------------
// Mock I2C implementation for testing without real hardware
// ---------------------------------------------------------------------------

class MockI2C : public II2CInterface
{
public:
  int setup_return_value = 5;      ///< Simulated valid file descriptor
  std::map<int, int> reg_values;   ///< Values returned by readReg8
  std::map<int, int> written_regs; ///< Records of writeReg8 calls

  int setup(int /*dev_addr*/) override { return setup_return_value; }

  int readReg8(int /*fd*/, int reg) override
  {
    auto it = reg_values.find(reg);
    return (it != reg_values.end()) ? it->second : 0;
  }

  int writeReg8(int /*fd*/, int reg, int data) override
  {
    written_regs[reg] = data;
    return 0;
  }
};

// ---------------------------------------------------------------------------
// Helper: spin the node until a message arrives or timeout elapses.
// The driver timer fires every 100 ms; we wait up to 1 s.
// ---------------------------------------------------------------------------

static sensor_msgs::msg::Imu::SharedPtr spinAndCapture(
  std::shared_ptr<Mpu6050Driver> node,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
{
  sensor_msgs::msg::Imu::SharedPtr received;
  auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "output", rclcpp::QoS{10},
    [&received](sensor_msgs::msg::Imu::SharedPtr msg) {
      received = msg;
    });

  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!received && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return received;
}

static geometry_msgs::msg::Vector3Stamped::SharedPtr spinAndCaptureRollPitch(
  std::shared_ptr<Mpu6050Driver> node,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
{
  geometry_msgs::msg::Vector3Stamped::SharedPtr received;
  auto sub = node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "roll_pitch", rclcpp::QoS{10},
    [&received](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
      received = msg;
    });

  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!received && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return received;
}

// ---------------------------------------------------------------------------
// Test suite
// rclcpp::init / shutdown are handled once in main() below.
// ---------------------------------------------------------------------------

/// Returns a unique node name derived from the current test case name.
static std::string testNodeName()
{
  auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
  return std::string("test_") + info->test_suite_name() + "_" + info->name();
}

TEST(Mpu6050DriverTest, WakesDeviceFromSleepOnInit)
{
  // PWR_MGMT_1 (0x6B) must be written with 0x00 to wake MPU6050 from sleep.
  MockI2C mock;
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  ASSERT_EQ(mock.written_regs.count(0x6B), 1u)
    << "PWR_MGMT_1 (0x6B) must be written during initialization";
  EXPECT_EQ(mock.written_regs.at(0x6B), 0)
    << "PWR_MGMT_1 must be cleared to 0 to exit sleep mode";
}

TEST(Mpu6050DriverTest, HandlesI2CSetupFailureGracefully)
{
  // When the I2C device is not found, the node must not crash.
  MockI2C mock;
  mock.setup_return_value = -1;
  rclcpp::NodeOptions opts;

  ASSERT_NO_THROW({
    auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);
    // Timer fires with fd==-1; must return early without UB.
    rclcpp::spin_some(node);
  });
}

TEST(Mpu6050DriverTest, DoesNotWritePwrMgmtWhenSetupFails)
{
  MockI2C mock;
  mock.setup_return_value = -1;
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  EXPECT_EQ(mock.written_regs.count(0x6B), 0u)
    << "PWR_MGMT_1 must not be written when I2C setup failed";
}

TEST(Mpu6050DriverTest, PositiveAccelValueConvertedCorrectly)
{
  // Raw = 0x1000 = 4096 => accel_x = 4096 / 16384.0 = 0.25 g
  MockI2C mock;
  mock.reg_values[0x3b] = 0x10;  // ACCEL_X high byte
  mock.reg_values[0x3c] = 0x00;  // ACCEL_X low byte
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  auto msg = spinAndCapture(node);
  ASSERT_NE(msg, nullptr) << "Expected an IMU message to be published";
  EXPECT_FLOAT_EQ(msg->linear_acceleration.x, 0.25f);
}

TEST(Mpu6050DriverTest, NegativeAccelValueConvertedCorrectly)
{
  // Raw = 0x8000 = 32768 => two's complement: 32768 - 65536 = -32768
  // => accel_x = -32768 / 16384.0 = -2.0 g
  // Regression: old code used value-65534, yielding -1.9999... instead of -2.0
  MockI2C mock;
  mock.reg_values[0x3b] = 0x80;  // ACCEL_X high byte
  mock.reg_values[0x3c] = 0x00;  // ACCEL_X low byte
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  auto msg = spinAndCapture(node);
  ASSERT_NE(msg, nullptr);
  EXPECT_FLOAT_EQ(msg->linear_acceleration.x, -2.0f);
}

TEST(Mpu6050DriverTest, MaxNegativeAccelValue)
{
  // Raw = 0xFFFF = 65535 => two's complement: -1 => -1 / 16384.0
  MockI2C mock;
  mock.reg_values[0x3b] = 0xFF;
  mock.reg_values[0x3c] = 0xFF;
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  auto msg = spinAndCapture(node);
  ASSERT_NE(msg, nullptr);
  EXPECT_NEAR(msg->linear_acceleration.x, -1.0f / 16384.0f, 1e-6f);
}

TEST(Mpu6050DriverTest, MaxPositiveAccelValue)
{
  // Raw = 0x7FFF = 32767 => 32767 / 16384.0
  MockI2C mock;
  mock.reg_values[0x3b] = 0x7F;
  mock.reg_values[0x3c] = 0xFF;
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  auto msg = spinAndCapture(node);
  ASSERT_NE(msg, nullptr);
  EXPECT_NEAR(msg->linear_acceleration.x, 32767.0f / 16384.0f, 1e-4f);
}

TEST(Mpu6050DriverTest, GyroScalingApplied)
{
  // Raw = 0x0083 = 131 => angular_velocity.x = 131 / 131.0 = 1.0
  MockI2C mock;
  mock.reg_values[0x43] = 0x00;
  mock.reg_values[0x44] = 0x83;
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  auto msg = spinAndCapture(node);
  ASSERT_NE(msg, nullptr);
  EXPECT_NEAR(msg->angular_velocity.x, 1.0f, 1e-4f);
}

TEST(Mpu6050DriverTest, NegativeGyroScalingApplied)
{
  // Raw = 0xFF7D = 65405 => two's complement: 65405 - 65536 = -131
  // => angular_velocity.x = -131 / 131.0 = -1.0
  MockI2C mock;
  mock.reg_values[0x43] = 0xFF;
  mock.reg_values[0x44] = 0x7D;
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  auto msg = spinAndCapture(node);
  ASSERT_NE(msg, nullptr);
  EXPECT_NEAR(msg->angular_velocity.x, -1.0f, 1e-4f);
}

TEST(Mpu6050DriverTest, ImuMessageHasCorrectFrameId)
{
  MockI2C mock;
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  auto msg = spinAndCapture(node);
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->header.frame_id, "imu");
}

TEST(Mpu6050DriverTest, ImuMessageTimestampIsSet)
{
  MockI2C mock;
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  auto msg = spinAndCapture(node);
  ASSERT_NE(msg, nullptr);
  const auto & stamp = msg->header.stamp;
  EXPECT_TRUE(stamp.sec != 0 || stamp.nanosec != 0)
    << "Message timestamp must be filled in";
}

TEST(Mpu6050DriverTest, AllAxesPublishedCorrectly)
{
  // Set distinct values on all 6 axes to verify they are mapped correctly.
  MockI2C mock;
  // Accel X: raw = 0x0200 = 512  => 512  / 16384.0 = 0.03125
  mock.reg_values[0x3b] = 0x02; mock.reg_values[0x3c] = 0x00;
  // Accel Y: raw = 0x0400 = 1024 => 1024 / 16384.0 = 0.0625
  mock.reg_values[0x3d] = 0x04; mock.reg_values[0x3e] = 0x00;
  // Accel Z: raw = 0x0800 = 2048 => 2048 / 16384.0 = 0.125
  mock.reg_values[0x3f] = 0x08; mock.reg_values[0x40] = 0x00;
  // Gyro X: raw = 0x0083 = 131   => 131  / 131.0   = 1.0
  mock.reg_values[0x43] = 0x00; mock.reg_values[0x44] = 0x83;
  // Gyro Y: raw = 0x0106 = 262   => 262  / 131.0   = 2.0
  mock.reg_values[0x45] = 0x01; mock.reg_values[0x46] = 0x06;
  // Gyro Z: raw = 0x0189 = 393   => 393  / 131.0   = 3.0
  mock.reg_values[0x47] = 0x01; mock.reg_values[0x48] = 0x89;

  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  auto msg = spinAndCapture(node);
  ASSERT_NE(msg, nullptr);
  EXPECT_NEAR(msg->linear_acceleration.x, 0.03125f, 1e-4f);
  EXPECT_NEAR(msg->linear_acceleration.y, 0.0625f,  1e-4f);
  EXPECT_NEAR(msg->linear_acceleration.z, 0.125f,   1e-4f);
  EXPECT_NEAR(msg->angular_velocity.x,    1.0f,     1e-4f);
  EXPECT_NEAR(msg->angular_velocity.y,    2.0f,     1e-4f);
  EXPECT_NEAR(msg->angular_velocity.z,    3.0f,     1e-4f);
}

TEST(Mpu6050DriverTest, RollPitchZeroWhenFlat)
{
  // Flat orientation: accel_x = 0, accel_y = 0, accel_z = 1g (raw 0x4000 = 16384).
  // Expected: roll = atan(0/1) * 57.324 = 0, pitch = atan(0/1) * 57.324 = 0.
  MockI2C mock;
  mock.reg_values[0x3f] = 0x40;  // ACCEL_Z high byte => 16384 / 16384 = 1.0 g
  mock.reg_values[0x40] = 0x00;  // ACCEL_Z low byte
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  auto msg = spinAndCaptureRollPitch(node);
  ASSERT_NE(msg, nullptr) << "Expected a roll_pitch message to be published";
  EXPECT_NEAR(msg->vector.x, 0.0f, 0.01f) << "Roll must be 0 when flat";
  EXPECT_NEAR(msg->vector.y, 0.0f, 0.01f) << "Pitch must be 0 when flat";
  EXPECT_EQ(msg->header.frame_id, "imu");
}

TEST(Mpu6050DriverTest, RollFortyFiveDegrees)
{
  // accel_x = 0, accel_y = 1g, accel_z = 1g => roll = atan(1) * 57.324 ≈ 45.024°.
  // raw 1g = 16384 = 0x4000, high byte = 0x40, low byte = 0x00.
  MockI2C mock;
  mock.reg_values[0x3d] = 0x40;  // ACCEL_Y high byte
  mock.reg_values[0x3e] = 0x00;  // ACCEL_Y low byte
  mock.reg_values[0x3f] = 0x40;  // ACCEL_Z high byte
  mock.reg_values[0x40] = 0x00;  // ACCEL_Z low byte
  rclcpp::NodeOptions opts;
  auto node = std::make_shared<Mpu6050Driver>(testNodeName(), opts, &mock);

  const float expected_roll = std::atan(1.0f) * 57.324f;
  auto msg = spinAndCaptureRollPitch(node);
  ASSERT_NE(msg, nullptr);
  EXPECT_NEAR(msg->vector.x, expected_roll, 0.01f) << "Roll must be ~45° when accel_y == accel_z";
  EXPECT_NEAR(msg->vector.y, 0.0f, 0.01f) << "Pitch must be 0 when accel_x == 0";
}

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
