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
| `output` | `sensor_msgs/Imu` | IMU data using ROS SI units (`rad/s` angular velocity, `m/s^2` linear acceleration) |
| `roll_pitch` | `geometry_msgs/Vector3Stamped` | Roll and pitch angles in degrees (`x=roll`, `y=pitch`, `z=0`) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Driver health status for ROS diagnostics tools |

### IMU Units

The `output` topic follows the standard `sensor_msgs/Imu` unit conventions:

| Field | Unit |
|-------|------|
| `angular_velocity` | `rad/s` |
| `angular_velocity_covariance` | `(rad/s)^2` |
| `linear_acceleration` | `m/s^2` |
| `linear_acceleration_covariance` | `(m/s^2)^2` |

Internally, the MPU6050 is configured for ±250 deg/s gyro range and ±2g accelerometer range. The driver converts those raw sensor units before publishing `sensor_msgs/Imu`.

If you used an earlier version of this driver that published deg/s and g on `output`, remove downstream unit adapters or disable their `gyro_in_degrees` / `accel_in_g` style conversions to avoid double conversion.

The driver does not estimate orientation on the `output` topic, so `orientation_covariance[0]` is always set to `-1.0`.

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
| `angular_velocity_bias` | double[3] | `[0.0, 0.0, 0.0]` | Static gyro bias to subtract from `angular_velocity` in `rad/s` |
| `linear_acceleration_bias` | double[3] | `[0.0, 0.0, 0.0]` | Static accelerometer bias to subtract from `linear_acceleration` in `m/s^2` |
| `angular_velocity_covariance` | double[9] | `[0.0, ...]` | Row-major covariance for `angular_velocity`; all-zero means unknown |
| `linear_acceleration_covariance` | double[9] | `[0.0, ...]` | Row-major covariance for `linear_acceleration`; all-zero means unknown |

Bias offsets are applied after raw sensor scaling and SI unit conversion:

```text
published_value = scaled_sensor_value - configured_bias
```

The corrected acceleration sample is also used by the `roll_pitch` topic.

For an initial stationary bias estimate:

1. Place the IMU in its normal mounted orientation and keep it still.
2. Record a short sample window from the `output` topic after startup.
3. Use the mean angular velocity on each axis as `angular_velocity_bias`.
4. For linear acceleration, subtract the expected gravity vector for the mounted orientation from the measured mean, then use the residual as `linear_acceleration_bias`.
5. Keep all values in ROS SI units (`rad/s` and `m/s^2`).

This is a static correction only. Re-estimate the offsets when the sensor, mounting, or operating environment changes.

Covariance parameters map directly to the 3x3 row-major arrays in `sensor_msgs/Imu`.
For a diagonal-only configuration, keep off-diagonal terms at `0.0`:

```sh
ros2 launch imu_driver mpu6050_driver.launch.xml \
  angular_velocity_covariance:="[0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]" \
  linear_acceleration_covariance:="[0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]"
```

For EKF or filter integration, replace these examples with values derived from measurement,
datasheet assumptions, or a calibration procedure. The all-zero defaults preserve ROS's
unknown-covariance semantics and are not tuned estimator values.

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
