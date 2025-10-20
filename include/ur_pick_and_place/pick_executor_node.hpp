#ifndef UR_PICK_AND_PLACE_PICK_EXECUTOR_NODE_HPP_
#define UR_PICK_AND_PLACE_PICK_EXECUTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ur_pick_and_place/action/pick.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ur_pick_and_place/srv/gripper_control.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <atomic>

namespace ur_pick_and_place
{

class PickExecutorNode : public rclcpp::Node
{
public:
  using PickAction = ur_pick_and_place::action::Pick;
  using GoalHandlePick = rclcpp_action::ServerGoalHandle<PickAction>;

  explicit PickExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Pick 단계 정의
  enum class PickStep {
    IDLE,
    MOVING_TO_PICK_POSITION,
    OPENING_GRIPPER,
    DESCENDING,
    CLOSING_GRIPPER,
    ASCENDING,
    COMPLETED
  };

  rclcpp_action::Server<PickAction>::SharedPtr action_server_;
  rclcpp::Client<ur_pick_and_place::srv::GripperControl>::SharedPtr gripper_client_;
  
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  // Pause/Resume 관련
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
  std::atomic<bool> is_paused_{false};
  std::atomic<bool> is_resuming_{false};
  PickStep paused_step_{PickStep::IDLE};
  geometry_msgs::msg::Pose paused_target_;
  std::mutex pause_mutex_;
  
  // Action 관련
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickAction::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandlePick> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandlePick> goal_handle);
  void executePick(const std::shared_ptr<GoalHandlePick> goal_handle);
  
  // Pick 동작 단계별 함수
  bool moveToPickPosition(const geometry_msgs::msg::Pose & target_pose);
  bool descendToTarget(const geometry_msgs::msg::Pose & target_pose);
  bool ascendFromTarget(const geometry_msgs::msg::Pose & target_pose);
  
  // Pause/Resume 관련 함수
  void pauseCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void resumeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool executeTrajectoryWithPause(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    PickStep current_step);
  bool executePlanWithPause(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    PickStep current_step);
  void checkPauseAndWait();
  
  // Setup 함수
  void setupPlanningScene();
  void setupMoveGroup();
};

}  // namespace ur_pick_and_place

#endif  // UR_PICK_AND_PLACE_PICK_EXECUTOR_NODE_HPP_ 