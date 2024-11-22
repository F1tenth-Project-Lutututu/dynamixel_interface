// Copyright 2024 Maciej Krupka
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DYNAMIXEL_INTERFACE__DYNAMIXEL_INTERFACE_HPP_
#define DYNAMIXEL_INTERFACE__DYNAMIXEL_INTERFACE_HPP_

#include <cstdint>

#include "dynamixel_interface/visibility_control.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"


namespace dynamixel_interface
{

class DYNAMIXEL_INTERFACE_PUBLIC DynamixelInterface
{
public:
  DynamixelInterface();
  int64_t foo(int64_t bar) const;
};

}  // namespace dynamixel_interface

#endif  // DYNAMIXEL_INTERFACE__DYNAMIXEL_INTERFACE_HPP_
