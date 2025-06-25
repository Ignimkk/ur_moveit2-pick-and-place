#ifndef UR_PICK_AND_PLACE_PICK_EXECUTOR_NODE_HPP_
#define UR_PICK_AND_PLACE_PICK_EXECUTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ur_pick_and_place/action/pick.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ur_pick_and_place/srv/gripper_control.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>

namespace ur_pick_and_place
{

class PickExecutorNode : public rclcpp::Node
{
public:
  using PickAction = ur_pick_and_place::action::Pick;
  using GoalHandlePick = rclcpp_action::ServerGoalHandle<PickAction>;

  explicit PickExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<PickAction>::SharedPtr action_server_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Client<ur_pick_and_place::srv::GripperControl>::SharedPtr gripper_client_;
  
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void executePickDirectly(const geometry_msgs::msg::Pose & target_pose);
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickAction::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandlePick> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandlePick> goal_handle);
  void executePick(const std::shared_ptr<GoalHandlePick> goal_handle);
  
  bool moveToHome();
  bool moveToPickPosition(const geometry_msgs::msg::Pose & target_pose);
  bool descendToTarget(const geometry_msgs::msg::Pose & target_pose);
  bool ascendFromTarget(const geometry_msgs::msg::Pose & target_pose);
  
  void setupPlanningScene();
  void setupMoveGroup();
};

}  // namespace ur_pick_and_place

#endif  // UR_PICK_AND_PLACE_PICK_EXECUTOR_NODE_HPP_ 