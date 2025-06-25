#include "ur_pick_and_place/gripper_controller_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace ur_pick_and_place
{

GripperControllerNode::GripperControllerNode(const rclcpp::NodeOptions & options)
: Node("gripper_controller_node", options)
{
  // Gripper 제어 서비스 설정
  gripper_service_ = this->create_service<ur_pick_and_place::srv::GripperControl>(
    "/gripper/control",
    std::bind(&GripperControllerNode::gripperServiceCallback, this, 
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Gripper Controller Node initialized");
}

void GripperControllerNode::gripperServiceCallback(
  const std::shared_ptr<ur_pick_and_place::srv::GripperControl::Request> request,
  std::shared_ptr<ur_pick_and_place::srv::GripperControl::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received gripper command: %s", request->command.c_str());
  
  bool success = executeGripperCommand(request->command);
  
  response->success = success;
  if (success) {
    response->message = "Gripper command executed successfully: " + request->command;
  } else {
    response->message = "Failed to execute gripper command: " + request->command;
  }
}

bool GripperControllerNode::executeGripperCommand(const std::string & command)
{
  if (command == "open") {
    RCLCPP_INFO(this->get_logger(), "Opening gripper");
    // TODO: 실제 gripper 제어 로직 구현
    // 현재는 placeholder로 1초 대기
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
  } else if (command == "close") {
    RCLCPP_INFO(this->get_logger(), "Closing gripper");
    // TODO: 실제 gripper 제어 로직 구현
    // 현재는 placeholder로 1초 대기
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid gripper command: %s", command.c_str());
    return false;
  }
}

}  // namespace ur_pick_and_place 