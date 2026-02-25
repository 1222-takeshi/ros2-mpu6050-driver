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

#ifndef IMU_DRIVER__WIRINGPI_I2C_HPP_
#define IMU_DRIVER__WIRINGPI_I2C_HPP_

#include "imu_driver/i2c_interface.hpp"

#ifdef HAVE_WIRINGPI
#include <wiringPiI2C.h>

/// Production implementation of II2CInterface using the wiringPi library.
class WiringPiI2C : public II2CInterface
{
public:
  int setup(int dev_addr) override
  {
    return wiringPiI2CSetup(dev_addr);
  }

  int readReg8(int fd, int reg) override
  {
    return wiringPiI2CReadReg8(fd, reg);
  }

  int writeReg8(int fd, int reg, int data) override
  {
    return wiringPiI2CWriteReg8(fd, reg, data);
  }
};

#else

#include <cstdio>

/// Stub used when wiringPi is not available (e.g. CI environment without hardware).
class WiringPiI2C : public II2CInterface
{
public:
  int setup(int /*dev_addr*/) override
  {
    fprintf(stderr, "[WiringPiI2C] wiringPi not available on this platform\n");
    return -1;
  }

  int readReg8(int /*fd*/, int /*reg*/) override { return 0; }
  int writeReg8(int /*fd*/, int /*reg*/, int /*data*/) override { return -1; }
};

#endif  // HAVE_WIRINGPI

#endif  // IMU_DRIVER__WIRINGPI_I2C_HPP_
