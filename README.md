# ros2_mpu6050_driver

ROS2 driver for the MPU6050 IMU sensor (accelerometer + gyroscope), designed for use with a Raspberry Pi over I2C.

## Tested Environment

| Component | Version |
|-----------|---------|
| Raspberry Pi | 3B+, 4 (build only) |
| Ubuntu | 20.04, 22.04 (build only) |
| ROS2 | Foxy, Humble (build only) |

## Prerequisites

### WiringPi

Install WiringPi for I2C communication on Raspberry Pi:

```sh
# For Raspberry Pi OS / Ubuntu on RPi
sudo apt-get install wiringpi

# If the above doesn't work (newer Pi models), build from source:
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
./build
```

Verify the installation:

```sh
gpio -v
```

### Enable I2C

Enable I2C on Raspberry Pi:

```sh
sudo raspi-config
# Navigate to: Interface Options > I2C > Enable
```

Verify the MPU6050 is detected (default address: `0x68`):

```sh
i2cdetect -y 1
```

## Installation

```sh
# Create a ROS2 workspace (skip if you already have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/1222-takeshi/ros2-mpu6050-driver.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Usage

```sh
# Grant I2C permissions (run once per boot, or configure udev rules)
sudo chmod 666 /dev/i2c-1

# Launch the driver
ros2 launch imu_driver mpu6050_driver.launch.xml
```

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `output` | `sensor_msgs/Imu` | Raw IMU data (angular velocity + linear acceleration) |

### Sensor Configuration

The driver uses the following MPU6050 default settings:

| Parameter | Value |
|-----------|-------|
| Gyroscope range | ±250°/s |
| Accelerometer range | ±2g |
| Output rate | 10 Hz |
| I2C address | 0x68 |

## References

- [MPU6050 ROS2 driver reference](https://shizenkarasuzon.hatenablog.com/entry/2019/03/06/163906)
