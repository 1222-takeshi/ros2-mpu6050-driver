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

#ifndef WIRINGPI__WIRINGPII2C_H_
#define WIRINGPI__WIRINGPII2C_H_

// Only the public wiringPi I2C functions used by this backend are mocked.
// A wiringPiI2CClose function is deliberately absent: it is not a public API.
extern "C" {
int wiringPiI2CSetup(int dev_addr);
int wiringPiI2CReadReg8(int fd, int reg);
int wiringPiI2CWriteReg8(int fd, int reg, int data);
}

#endif  // WIRINGPI__WIRINGPII2C_H_
