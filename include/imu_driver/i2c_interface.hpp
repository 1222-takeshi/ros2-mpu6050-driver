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

#ifndef IMU_DRIVER__I2C_INTERFACE_HPP_
#define IMU_DRIVER__I2C_INTERFACE_HPP_

/// Abstract interface for I2C operations.
/// Allows dependency injection for testing without real hardware.
class II2CInterface
{
public:
  virtual ~II2CInterface() = default;

  /// Open I2C device at the given address. Returns file descriptor or -1 on failure.
  virtual int setup(int dev_addr) = 0;

  /// Read a single byte from the specified register. Returns the byte value.
  virtual int readReg8(int fd, int reg) = 0;

  /// Write a single byte to the specified register. Returns 0 on success, -1 on failure.
  virtual int writeReg8(int fd, int reg, int data) = 0;
};

#endif  // IMU_DRIVER__I2C_INTERFACE_HPP_
