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

#include "dynamixel_interface/dynamixel_interface_node.hpp"

#include <bitset>
constexpr int POSITION_MAX = 1420;
constexpr int POSITION_MIN = 843;
constexpr float STEERING_OFFSET = 0.5f;


constexpr float PROTOCOL_VERSION = 2.0;
constexpr int BAUDRATE = 4000000;
constexpr int ID = 24;
constexpr uint16_t ADDR_OPERATING_MODE = 11;
constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
constexpr uint16_t ADDR_GOAL_POSITION = 116;
constexpr uint16_t ADDR_PRESENT_POSITION = 132;
constexpr uint16_t MIN_POSITION_LIMIT = 48;
constexpr uint16_t MAX_POSITION_LIMIT = 52;
constexpr uint16_t POSITION_P_GAIN = 84;
constexpr uint16_t POSITION_I_GAIN = 82;
constexpr uint16_t POSITION_D_GAIN = 80;
constexpr uint16_t FEEDFORWARD_1ST_GAIN = 88;
constexpr uint16_t FEEDFORWARD_2ND_GAIN = 90;
constexpr uint16_t GOAL_POSITION = 116;
constexpr uint16_t REALTIME_TICK = 120;
constexpr uint16_t PRESENT_LOAD = 126;
constexpr uint16_t PRESENT_VELOCITY = 128;
constexpr uint16_t PRESENT_POSITION = 132;
constexpr uint16_t VELOCITY_TRAJECTORY = 136;
constexpr uint16_t POSITION_TRAJECTORY = 140;
constexpr uint16_t GROUP_LEN = 18;

constexpr uint8_t LEN_PRESENT_POSITION = 4;
constexpr uint8_t LEN_PRESENT_VELOCITY = 4;
constexpr uint8_t LEN_PRESENT_LOAD = 2;

inline constexpr uint32_t mapToRange(float value)
{
  return static_cast<uint32_t>((value + STEERING_OFFSET) * (POSITION_MAX - POSITION_MIN) +
         POSITION_MIN);
}

inline constexpr float inverseMapToRange(uint32_t value)
{
  return (static_cast<float>(value) - POSITION_MIN) / (POSITION_MAX - POSITION_MIN) -
         STEERING_OFFSET;
}
inline constexpr uint16_t extractLower16Bits(const uint32_t & value)
{
  // Apply the mask 0xFFFF to get the last 16 bits (lower 16 bits).
  return static_cast<uint16_t>(value & 0xFFFF);
}
inline constexpr int fromTwosComplement16bit(const uint16_t & value)
{
  // Check the sign bit (the most significant bit, 16th bit)
  if (value & 0x8000) {
    // If the sign bit is set, it's a negative value.
    // Convert from two's complement to signed integer.
    return static_cast<int>(static_cast<int16_t>(value));
  } else {
    // If the sign bit is not set, it's a non-negative value.
    // The value is the same for signed and unsigned representation.
    return static_cast<int>(value);
  }
}
inline constexpr int fromTwosComplement(const uint32_t & value)
{
  // Check the sign bit (the most significant bit, 32nd bit)
  if (value & 0x80000000) {
    // If the sign bit is set, it's a negative value.
    // Convert from two's complement to signed integer.
    return static_cast<int>(static_cast<int32_t>(value));
  } else {
    // If the sign bit is not set, it's a non-negative value.
    // The value is the same for signed and unsigned representation.
    return static_cast<int>(value);
  }
}
inline constexpr float calculateVelocity(uint32_t value)
{
  constexpr float RPM2RADPERSEC = 0.104719755f;
  constexpr float rpm = 0.229f;
  return static_cast<float>(fromTwosComplement(value)) * RPM2RADPERSEC * rpm;
}
inline constexpr float calculateTorque(uint32_t value)
{
  constexpr float valueToProcent = 0.1f;
  return static_cast<float>(fromTwosComplement16bit(extractLower16Bits(value))) * valueToProcent;
}


inline uint32_t make_word(uint16_t a, uint16_t b)
{
  return static_cast<uint32_t>(a) | (static_cast<uint32_t>(b) << 16);
}

namespace dynamixel_interface
{

DynamixelInterfaceNode::DynamixelInterfaceNode(const rclcpp::NodeOptions & options)
:  Node("dynamixel_interface", options)
{
  dynamixel_interface_ = std::make_unique<dynamixel_interface::DynamixelInterface>();
  uint8_t dxl_error = 0;
  auto serial_port = this->declare_parameter("serial_port", "/dev/ttyUSB0");
  auto kp = this->declare_parameter("kp", 2000);
  auto kd = this->declare_parameter("kd", 500);
  auto ki = this->declare_parameter("ki", 1000);
  // portHandler_ = std::make_unique<dynamixel::PortHandler>(serial_port_);
  // packetHandler_ = std::make_unique<dynamixel::PacketHandler>(PROTOCOL_VERSION);
  portHandler_ = dynamixel::PortHandler::getPortHandler(serial_port.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  groupBulkRead_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);
  if (!groupBulkRead_->addParam(ID, PRESENT_LOAD, 18)) {
    RCLCPP_ERROR(get_logger(), "Failed to add parameters to groupBulkRead.");
  }
  auto dxl_comm_result = portHandler_->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(get_logger(), "Failed to open the port!");
    return;
  }
  dxl_comm_result = portHandler_->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(get_logger(), "Failed to set the baudrate!");
    return;
  }
  dxl_comm_result = packetHandler_->write1ByteTxRx(
    portHandler_,
    ID,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to set Position Control Mode.");
  }
  dxl_comm_result = packetHandler_->write2ByteTxRx(
    portHandler_,
    ID,
    POSITION_P_GAIN,
    kp,
    &dxl_error
  );
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to set kp.");
  }
  dxl_comm_result = packetHandler_->write2ByteTxRx(
    portHandler_,
    ID,
    POSITION_D_GAIN,
    kd,
    &dxl_error
  );
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to set kd.");
  }
  dxl_comm_result = packetHandler_->write2ByteTxRx(
    portHandler_,
    ID,
    POSITION_I_GAIN,
    ki,
    &dxl_error
  );
  dxl_comm_result = packetHandler_->write1ByteTxRx(
    portHandler_,
    ID,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to enable torque.");
  }
  sub_control_ = this->create_subscription<Control>(
    "/commands/ctrl",
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    [this](const Control::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;
      uint32_t goal_position = mapToRange(msg->steering_angle);

      if (auto dxl_comm_result = packetHandler_->write4ByteTxRx(
        portHandler_,
        ID,
        ADDR_GOAL_POSITION,
        goal_position,
        &dxl_error
      ); dxl_comm_result != COMM_SUCCESS) [[unlikely]] {
        RCLCPP_INFO(get_logger(), "%s", packetHandler_->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) [[unlikely]] {
        RCLCPP_INFO(get_logger(), "%s", packetHandler_->getRxPacketError(dxl_error));
      }
    });

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / 300.0),
    std::bind(&DynamixelInterfaceNode::getTelemetry, this));
  pub_state_ = this->create_publisher<ServoState>("pub_position", rclcpp::SensorDataQoS());

  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DynamixelInterfaceNode::onParameterChanged, this, std::placeholders::_1));

}

void DynamixelInterfaceNode::getTelemetry()
{
  uint32_t present_position = 0;
  uint32_t present_load = 0;
  uint32_t present_velocity = 0;
  uint32_t present_position_trajectory = 0;
  uint32_t present_velocity_trajectory = 0;
  auto dxl_comm_bulk_read = groupBulkRead_->txRxPacket();
  if (dxl_comm_bulk_read != COMM_SUCCESS) [[unlikely]] {
    RCLCPP_ERROR(get_logger(), "Failed to get telemetry.");
    return;
  }

  // Torque (Load)
  if (auto load = groupBulkRead_->getData(ID, PRESENT_LOAD, LEN_PRESENT_LOAD);
    groupBulkRead_->isAvailable(ID, PRESENT_LOAD, LEN_PRESENT_LOAD))
  {
    present_load = load;
  } else [[unlikely]] {
    RCLCPP_ERROR(get_logger(), "Failed to retrieve torque");
  }

  // Velocity
  if (auto velocity = groupBulkRead_->getData(ID, PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
    groupBulkRead_->isAvailable(ID, PRESENT_VELOCITY, LEN_PRESENT_VELOCITY))
  {
    present_velocity = velocity;
  } else [[unlikely]] {
    RCLCPP_ERROR(get_logger(), "Failed to retrieve velocity");
  }

  // Position
  if (auto position = groupBulkRead_->getData(ID, PRESENT_POSITION, LEN_PRESENT_POSITION);
    groupBulkRead_->isAvailable(ID, PRESENT_POSITION, LEN_PRESENT_POSITION))
  {
    present_position = position;
  } else [[unlikely]] {
    RCLCPP_ERROR(get_logger(), "Failed to retrieve position");
  }

  // Velocity Trajectory
  if (auto velocity_trajectory =
    groupBulkRead_->getData(ID, VELOCITY_TRAJECTORY, LEN_PRESENT_VELOCITY);
    groupBulkRead_->isAvailable(ID, VELOCITY_TRAJECTORY, LEN_PRESENT_VELOCITY))
  {
    present_velocity_trajectory = velocity_trajectory;
  } else [[unlikely]] {
    RCLCPP_ERROR(get_logger(), "Failed to retrieve velocity trajectory");
  }

  // Position Trajectory
  if (auto position_trajectory =
    groupBulkRead_->getData(ID, POSITION_TRAJECTORY, LEN_PRESENT_POSITION);
    groupBulkRead_->isAvailable(ID, POSITION_TRAJECTORY, LEN_PRESENT_POSITION))
  {
    present_position_trajectory = position_trajectory;
  } else [[unlikely]] {
    RCLCPP_ERROR(get_logger(), "Failed to retrieve position trajectory");
  }

  auto msg = ServoState();
  msg.header.stamp = this->now();
  msg.torque = calculateTorque(present_load);
  msg.position = inverseMapToRange(present_position);
  msg.velocity = calculateVelocity(present_velocity);
  msg.velocity_setpoint = calculateVelocity(present_velocity_trajectory);
  msg.position_setpoint = inverseMapToRange(present_position_trajectory);

  pub_state_->publish(msg);
}

rcl_interfaces::msg::SetParametersResult DynamixelInterfaceNode::onParameterChanged(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  try {
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "kp") {
        RCLCPP_INFO(get_logger(), "kp is updated");
        auto kp = parameter.as_int();
        if (kp > 0 && kp < 16383) {
          uint8_t dxl_error = 0;

          auto dxl_comm_result = packetHandler_->write2ByteTxRx(
            portHandler_,
            ID,
            POSITION_P_GAIN,
            kp,
            &dxl_error
          );
          result.successful = !dxl_comm_result;
          result.reason = "kp is updated";
        }
      } else if (parameter.get_name() == "kd") {
        RCLCPP_INFO(get_logger(), "kd is updated");
        auto kd = parameter.as_int();
        if (kd > 0 && kd < 16383) {
          uint8_t dxl_error = 0;
          auto dxl_comm_result = packetHandler_->write2ByteTxRx(
            portHandler_,
            ID,
            POSITION_D_GAIN,
            kd,
            &dxl_error
          );
          // not dxl_comm_result
          result.successful = !dxl_comm_result;
          result.reason = "kd is updated";
        }
      } else if (parameter.get_name() == "ki") {
        RCLCPP_INFO(get_logger(), "ki is updated");
        auto ki = parameter.as_int();
        if (ki > 0 && ki < 16383) {
          uint8_t dxl_error = 0;
          auto dxl_comm_result = packetHandler_->write2ByteTxRx(
            portHandler_,
            ID,
            POSITION_I_GAIN,
            ki,
            &dxl_error
          );
          result.successful = !dxl_comm_result;
          result.reason = "ki is updated";
        }
      }
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }
  return result;
}
}  // namespace dynamixel_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(dynamixel_interface::DynamixelInterfaceNode)
