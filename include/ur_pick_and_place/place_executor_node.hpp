#ifndef UR_PICK_AND_PLACE_PLACE_EXECUTOR_NODE_HPP_
#define UR_PICK_AND_PLACE_PLACE_EXECUTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ur_pick_and_place/action/place.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ur_pick_and_place/srv/gripper_control.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <atomic>

namespace ur_pick_and_place
{

class PlaceExecutorNode : public rclcpp::Node
{
public:
  using PlaceAction = ur_pick_and_place::action::Place;
  using GoalHandlePlace = rclcpp_action::ServerGoalHandle<PlaceAction>;

  explicit PlaceExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Place 단계 정의
  enum class PlaceStep {
    IDLE,
    MOVING_TO_PLACE_POSITION,
    APPROACHING,
    DROPPING,
    RETREATING,
    COMPLETED
  };

  rclcpp_action::Server<PlaceAction>::SharedPtr action_server_;
  rclcpp::Client<ur_pick_and_place::srv::GripperControl>::SharedPtr gripper_client_;
  
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  // Pause/Resume 관련
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  std::atomic<bool> is_paused_{false};
  std::atomic<bool> is_resuming_{false};
  PlaceStep paused_step_{PlaceStep::IDLE};
  geometry_msgs::msg::Pose paused_target_;
  std::mutex pause_mutex_;
  
  // Action 관련
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlaceAction::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandlePlace> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandlePlace> goal_handle);
  void executePlace(const std::shared_ptr<GoalHandlePlace> goal_handle);
  
  // Place 동작 단계별 함수
  bool moveToPlacePosition(const geometry_msgs::msg::Pose & target_pose);
  bool approachPlacePosition(const geometry_msgs::msg::Pose & target_pose);
  bool dropObject();
  bool retreatFromPlacePosition(const geometry_msgs::msg::Pose & target_pose);
  
  // Pause/Resume 관련 함수
  void cmdCallback(const std_msgs::msg::String::SharedPtr msg);
  bool executeTrajectoryWithPause(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    PlaceStep current_step);
  bool executePlanWithPause(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    PlaceStep current_step);
  void checkPauseAndWait();
  
  // Setup 함수
  void setupPlanningScene();
  void setupMoveGroup();
};

}  // namespace ur_pick_and_place

#endif  // UR_PICK_AND_PLACE_PLACE_EXECUTOR_NODE_HPP_ 