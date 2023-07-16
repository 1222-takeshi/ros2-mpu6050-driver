# ros2_mpu6050_driver
This repository assumes that you are using the MPU6050 with a Raspberry Pi.

# Tested environment
I have tested it in the following environment.
- Raspberry Pi
  - 3B+
  - 4 (only build)
- ubuntu
  - 20.04
  - 22.04 (only build)
- ROS2
  - foxy
  - humble (only build)

# requirements
- WiringPI

# Usage
``` sh
sudo chmod 777 /dev/ttyAMA0 # Give permissions to imu, select imu's device.
ros2 launch imu_driver mpu6050_driver.launch.xml 
```

# References
I referred to the following.  
https://shizenkarasuzon.hatenablog.com/entry/2019/03/06/163906
