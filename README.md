# ros2_mpu6050_driver

ROS2 driver for the MPU6050 IMU sensor (accelerometer + gyroscope), designed for use with a Raspberry Pi over I2C.

## Tested Environment

| Component | Version |
|-----------|---------|
| Raspberry Pi | 3B+, 4 |
| Ubuntu | 22.04 |
| ROS2 | Humble |

## Prerequisites

### WiringPi

On Ubuntu 22.04, WiringPi is not available via apt and must be built from source:

```sh
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
./build
```

Verify the installation:

```sh
gpio -v
```

### Enable I2C

On Ubuntu 22.04, enable I2C by editing the boot configuration directly (`raspi-config` is not available):

```sh
sudo nano /boot/firmware/config.txt
```

Add the following line:

```
dtparam=i2c_arm=on
```

Then reboot:

```sh
sudo reboot
```

Install i2c-tools and add your user to the `i2c` group (avoids `sudo` for every command):

```sh
sudo apt install i2c-tools
sudo usermod -aG i2c $USER
# Log out and back in for the group change to take effect
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
ros2 launch imu_driver mpu6050_driver.launch.xml
```

The publish rate is controlled by the `publish_rate_hz` parameter. The default is `100.0` Hz.

Example override:

```sh
ros2 launch imu_driver mpu6050_driver.launch.xml publish_rate_hz:=200.0
```

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `output` | `sensor_msgs/Imu` | Raw IMU data (angular velocity + linear acceleration) |
| `roll_pitch` | `geometry_msgs/Vector3Stamped` | Roll and pitch angles in degrees (`x=roll`, `y=pitch`, `z=0`) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Driver health status for ROS diagnostics tools |

### Diagnostics

The driver publishes two diagnostic statuses:

| Status | OK | WARN | ERROR |
|--------|----|------|-------|
| `Hardware Status` | I2C is initialized, temperature is normal, and all axes are active | Chip temperature is above 70°C or one or more axes are in standby | I2C is not initialized, an I2C register read fails, or chip temperature is above 85°C |
| `Data Status` | IMU samples are recent and within the expected range | No sample has been published yet, the latest sample is stale, or the latest sample is outside the expected range | I2C is not initialized or the latest sample read failed |

You can inspect the diagnostics with:

```sh
ros2 topic echo /diagnostics
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_rate_hz` | double | `100.0` | IMU publish rate in Hz |

### Sensor Configuration

The driver uses the following MPU6050 default settings:

| Parameter | Value |
|-----------|-------|
| Gyroscope range | ±250°/s |
| Accelerometer range | ±2g |
| Output rate | 100 Hz by default (`publish_rate_hz`) |
| I2C address | 0x68 |

## References

- [MPU6050 ROS2 driver reference](https://shizenkarasuzon.hatenablog.com/entry/2019/03/06/163906)
