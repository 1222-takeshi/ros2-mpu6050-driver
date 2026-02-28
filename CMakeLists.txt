cmake_minimum_required(VERSION 3.5)
project(imu_driver)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Check for wiringPi (only available on Raspberry Pi hardware)
find_library(WIRINGPI_LIB wiringPi)

# Shared library containing driver logic (no main entry point)
ament_auto_add_library(mpu6050_driver_lib SHARED
  src/mpu6050_driver_node.cpp
)
if(WIRINGPI_LIB)
  target_compile_definitions(mpu6050_driver_lib PRIVATE HAVE_WIRINGPI)
  target_link_libraries(mpu6050_driver_lib ${WIRINGPI_LIB})
else()
  message(WARNING "wiringPi not found -- hardware I2C will be unavailable at runtime")
endif()

# Executable that links the driver library
ament_auto_add_executable(imu_driver
  src/mpu6050_driver_main.cpp
)
target_link_libraries(imu_driver mpu6050_driver_lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mpu6050_driver
    test/test_mpu6050_driver.cpp
  )
  target_include_directories(test_mpu6050_driver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_mpu6050_driver mpu6050_driver_lib)
  ament_target_dependencies(test_mpu6050_driver rclcpp sensor_msgs)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
