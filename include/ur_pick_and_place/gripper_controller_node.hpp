#ifndef UR_PICK_AND_PLACE_GRIPPER_CONTROLLER_NODE_HPP_
#define UR_PICK_AND_PLACE_GRIPPER_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ur_pick_and_place/srv/gripper_control.hpp>
#include <memory>

namespace ur_pick_and_place
{

class GripperControllerNode : public rclcpp::Node
{
public:
  explicit GripperControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void gripperServiceCallback(
    const std::shared_ptr<ur_pick_and_place::srv::GripperControl::Request> request,
    std::shared_ptr<ur_pick_and_place::srv::GripperControl::Response> response);
    
  bool executeGripperCommand(const std::string & command);

  rclcpp::Service<ur_pick_and_place::srv::GripperControl>::SharedPtr gripper_service_;
};

}  // namespace ur_pick_and_place

#endif  // UR_PICK_AND_PLACE_GRIPPER_CONTROLLER_NODE_HPP_ 