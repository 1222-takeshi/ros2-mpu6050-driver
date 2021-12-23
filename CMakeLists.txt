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

ament_auto_add_executable(imu_driver
  src/mpu6050_driver_node.cpp
  src/mpu6050_driver_main.cpp
)
ament_target_dependencies(imu_driver)
ament_auto_add_library(mpu6050_driver_node SHARED
  src/mpu6050_driver_node.cpp
  src/mpu6050_driver_main.cpp
  include/imu_driver/mpu6050_driver.hpp
)
target_link_libraries(imu_driver
  wiringPi
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
