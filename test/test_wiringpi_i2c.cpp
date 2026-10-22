// Copyright 2026 Takeshi Miura
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

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>

#include "gtest/gtest.h"
#include "imu_driver/wiringpi_i2c.hpp"

extern "C" {
int wiringPiI2CSetup(int) {return -1;}
int wiringPiI2CReadReg8(int, int) {return -1;}
int wiringPiI2CWriteReg8(int, int, int) {return -1;}
}

TEST(WiringPiI2CTest, ClosesRealFileDescriptorWithoutAnExtraWiringPiApi)
{
  const int fd = ::open("/dev/null", O_RDONLY);
  ASSERT_GE(fd, 0);
  WiringPiI2C backend;
  backend.close(fd);
  errno = 0;
  EXPECT_EQ(::fcntl(fd, F_GETFD), -1);
  EXPECT_EQ(errno, EBADF);
}
