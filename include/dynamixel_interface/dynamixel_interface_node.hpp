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

#ifndef DYNAMIXEL_INTERFACE__DYNAMIXEL_INTERFACE_NODE_HPP_
#define DYNAMIXEL_INTERFACE__DYNAMIXEL_INTERFACE_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "dynamixel_interface/dynamixel_interface.hpp"
#include <control_interfaces/msg/control.hpp>
#include <control_interfaces/msg/servo_state.hpp>
#include <bit>
namespace dynamixel_interface
{
using DynamixelInterfacePtr = std::unique_ptr<dynamixel_interface::DynamixelInterface>;
using Control = control_interfaces::msg::Control;
using ServoState = control_interfaces::msg::ServoState;
class DYNAMIXEL_INTERFACE_PUBLIC DynamixelInterfaceNode : public rclcpp::Node
{
public:
  explicit DynamixelInterfaceNode(const rclcpp::NodeOptions & options);

private:
  DynamixelInterfacePtr dynamixel_interface_{nullptr};
  int64_t param_name_{123};
  // std::unique_ptr<dynamixel::PortHandler> portHandler_;
  dynamixel::PortHandler * portHandler_;
  dynamixel::PacketHandler * packetHandler_;
  dynamixel::GroupBulkRead * groupBulkRead_;
  // std::unique_ptr<dynamixel::PacketHandler> packetHandler_;
  rclcpp::Subscription<Control>::SharedPtr sub_control_;
  rclcpp::Publisher<ServoState>::SharedPtr pub_state_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_servo_ok_;
  rclcpp::TimerBase::SharedPtr timer_;
  void getTelemetry();
  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
  rcl_interfaces::msg::SetParametersResult onParameterChanged(
    const std::vector<rclcpp::Parameter> & parameters);
};
}  // namespace dynamixel_interface

#endif  // DYNAMIXEL_INTERFACE__DYNAMIXEL_INTERFACE_NODE_HPP_
