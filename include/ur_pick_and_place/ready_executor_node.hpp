#ifndef UR_PICK_AND_PLACE_READY_EXECUTOR_NODE_HPP_
#define UR_PICK_AND_PLACE_READY_EXECUTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <memory>
#include <vector>

namespace ur_pick_and_place
{

class ReadyExecutorNode : public rclcpp::Node
{
public:
  explicit ReadyExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service_;

  std::vector<double> ready_joint_positions_;

  void setupMoveGroup();
  void setupPlanningScene();
  void declareAndLoadParameters();

  bool moveToReady();

  void onReadyService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

}  // namespace ur_pick_and_place

#endif  // UR_PICK_AND_PLACE_READY_EXECUTOR_NODE_HPP_ 