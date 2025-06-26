#include "ur_pick_and_place/pick_executor_node.hpp"
#include <thread>

#define PI 3.141592

namespace ur_pick_and_place
{

PickExecutorNode::PickExecutorNode(const rclcpp::NodeOptions & options)
: Node("pick_executor_node", options)
{
  // MoveIt 설정
  setupMoveGroup();
  setupPlanningScene();
  
  // 액션 서버 설정
  this->action_server_ = rclcpp_action::create_server<PickAction>(
    this,
    "pick_action",
    std::bind(&PickExecutorNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PickExecutorNode::handleCancel, this, std::placeholders::_1),
    std::bind(&PickExecutorNode::handleAccepted, this, std::placeholders::_1));
    
  // 목표 수신 구독자 (직접 토픽 처리 비활성화 - manager를 통해서만 실행)
  // goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
  //   "/internal/pick_goal", 10,
  //   std::bind(&PickExecutorNode::goalCallback, this, std::placeholders::_1));
    
  // Gripper 클라이언트
  gripper_client_ = this->create_client<ur_pick_and_place::srv::GripperControl>("/gripper/control");

  RCLCPP_INFO(this->get_logger(), "Pick Executor Node initialized");
}

void PickExecutorNode::setupMoveGroup()
{
  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  
  move_group_arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    std::shared_ptr<rclcpp::Node>(this), PLANNING_GROUP_ARM);
    
  planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
  
  // 플래너 설정 (더 유연하게)
  move_group_arm_->setPlannerId("RRTConnect"); // RRTConnect가 더 안정적
  move_group_arm_->setPlanningTime(20.0); // Planning 시간 증가
  move_group_arm_->setNumPlanningAttempts(5); // 재시도 횟수 증가
  move_group_arm_->setGoalTolerance(0.01); // 목표 허용 오차 증가
  
  RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_arm_->getPlanningFrame().c_str());
  RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_arm_->getEndEffectorLink().c_str());
}

void PickExecutorNode::setupPlanningScene()
{
  // 바닥 평면 추가
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_arm_->getPlanningFrame();
  collision_object.id = "ground";

  shape_msgs::msg::Plane plane;
  plane.coef = {0, 0, 1, 0}; // z = 0 평면

  geometry_msgs::msg::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.position.z = -0.01;

  collision_object.planes.push_back(plane);
  collision_object.plane_poses.push_back(ground_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface_->addCollisionObjects(collision_objects);
}

rclcpp_action::GoalResponse PickExecutorNode::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PickAction::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received pick goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PickExecutorNode::handleCancel(
  const std::shared_ptr<GoalHandlePick> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel pick goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PickExecutorNode::handleAccepted(const std::shared_ptr<GoalHandlePick> goal_handle)
{
  std::thread{std::bind(&PickExecutorNode::executePick, this, goal_handle)}.detach();
}

void PickExecutorNode::executePick(const std::shared_ptr<GoalHandlePick> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing pick action");
  
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PickAction::Feedback>();
  auto result = std::make_shared<PickAction::Result>();
  
  try {
    // Pick 진행 전 Home 위치로 이동
    feedback->current_step = "Moving to home position before pick";
    feedback->completion_percentage = 10.0;
    goal_handle->publish_feedback(feedback);
    
    if (!moveToHome()) {
      result->success = false;
      result->message = "Failed to move to home position before pick";
      goal_handle->abort(result);
      return;
    }
    
    // 목표지점 + 0.08m 높이로 이동
    feedback->current_step = "Moving to pick position (0.08m above target)";
    feedback->completion_percentage = 30.0;
    goal_handle->publish_feedback(feedback);
    
    if (!moveToPickPosition(goal->target_pose)) {
      result->success = false;
      result->message = "Failed to move to pick position";
      goal_handle->abort(result);
      return;
    }
    
    // 그리퍼 열기 (추후 구현)
    feedback->current_step = "Opening gripper (placeholder)";
    feedback->completion_percentage = 40.0;
    goal_handle->publish_feedback(feedback);
    // TODO: 그리퍼 열기 구현
    
    // 0.08m 하강
    feedback->current_step = "Descending 0.08m to target position";
    feedback->completion_percentage = 60.0;
    goal_handle->publish_feedback(feedback);
    
    if (!descendToTarget(goal->target_pose)) {
      result->success = false;
      result->message = "Failed to descend to target position";
      goal_handle->abort(result);
      return;
    }
    
    // 그리퍼 닫기 (추후 구현)
    feedback->current_step = "Closing gripper (placeholder)";
    feedback->completion_percentage = 80.0;
    goal_handle->publish_feedback(feedback);
    // TODO: 그리퍼 닫기 구현
    
    // 0.08m 상승
    feedback->current_step = "Ascending 0.08m from target position";
    feedback->completion_percentage = 90.0;
    goal_handle->publish_feedback(feedback);
    
    if (!ascendFromTarget(goal->target_pose)) {
      result->success = false;
      result->message = "Failed to ascend from target position";
      goal_handle->abort(result);
      return;
    }
    
    // 완료
    feedback->current_step = "Pick completed";
    feedback->completion_percentage = 100.0;
    goal_handle->publish_feedback(feedback);
    
    result->success = true;
    result->message = "Pick action completed successfully";
    goal_handle->succeed(result);
    
    RCLCPP_INFO(this->get_logger(), "Pick action completed successfully");
    
  } catch (const std::exception & e) {
    result->success = false;
    result->message = std::string("Exception during pick execution: ") + e.what();
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Exception during pick execution: %s", e.what());
  }
}

bool PickExecutorNode::moveToHome()
{
  RCLCPP_INFO(this->get_logger(), "Going Home");
  
  move_group_arm_->setStartStateToCurrentState();
  
  std::vector<double> joint_group_positions(6);
  joint_group_positions[0] = 0.00;   // Shoulder Pan
  joint_group_positions[1] = -PI/2;  // Shoulder Lift
  joint_group_positions[2] = 0.00;   // Elbow
  joint_group_positions[3] = -PI/2;  // Wrist 1
  joint_group_positions[4] = 0.00;   // Wrist 2
  joint_group_positions[5] = 0.00;   // Wrist 3

  move_group_arm_->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    RCLCPP_INFO(this->get_logger(), "Home position planning successful");
    move_group_arm_->execute(my_plan);
    rclcpp::sleep_for(std::chrono::seconds(2));
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Home position planning failed!");
    return false;
  }
}

bool PickExecutorNode::moveToPickPosition(const geometry_msgs::msg::Pose & target_pose)
{
  RCLCPP_INFO(this->get_logger(), "Moving to pick position (0.08m above target)");
  
  move_group_arm_->setStartStateToCurrentState();
  
  tf2::Quaternion orientation;
  orientation.setRPY(0, -PI, 0);
  geometry_msgs::msg::Quaternion ros_orientation = tf2::toMsg(orientation);

  geometry_msgs::msg::Pose pick_pose = target_pose;
  pick_pose.orientation = ros_orientation;
  pick_pose.position.z += 0.08; // 목표지점 + 0.08m 높이

  move_group_arm_->setPoseTarget(pick_pose);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    move_group_arm_->execute(my_plan);
    rclcpp::sleep_for(std::chrono::seconds(2));
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Pick position planning failed!");
    return false;
  }
}

bool PickExecutorNode::descendToTarget(const geometry_msgs::msg::Pose & target_pose)
{
  RCLCPP_INFO(this->get_logger(), "Approach to object!");
  
  tf2::Quaternion orientation;
  orientation.setRPY(0, -PI, 0);
  geometry_msgs::msg::Quaternion ros_orientation = tf2::toMsg(orientation);
  
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  geometry_msgs::msg::Pose approach_pose = target_pose;
  approach_pose.orientation = ros_orientation;
  
  // 첫 번째 waypoint: target_pose + 0.04m (중간 지점)
  approach_pose.position.z += 0.04;
  approach_waypoints.push_back(approach_pose);
  
  // 두 번째 waypoint: target_pose (최종 목표)
  approach_pose.position.z -= 0.04;
  approach_waypoints.push_back(approach_pose);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm_->computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  if (fraction > 0.98) {
    RCLCPP_INFO(this->get_logger(), "Approach Cartesian path planning successful (%.2f%%)", fraction * 100);
    move_group_arm_->execute(trajectory_approach);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Approach Cartesian path planning failed! (%.2f%%)", fraction * 100);
    
    // Fallback: 직접 목표 위치로 이동
    geometry_msgs::msg::Pose final_target = target_pose;
    final_target.orientation = ros_orientation;
    
    move_group_arm_->setStartStateToCurrentState();
    move_group_arm_->setPoseTarget(final_target);
    
    moveit::planning_interface::MoveGroupInterface::Plan descend_plan;
    bool success = (move_group_arm_->plan(descend_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Descend pose target planning successful");
      move_group_arm_->execute(descend_plan);
      rclcpp::sleep_for(std::chrono::seconds(1));
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Both Cartesian and pose target descend planning failed!");
      return false;
    }
  }
}

bool PickExecutorNode::ascendFromTarget(const geometry_msgs::msg::Pose & target_pose)
{
  RCLCPP_INFO(this->get_logger(), "Retreat from object!");
  
  tf2::Quaternion orientation;
  orientation.setRPY(0, -PI, 0);
  geometry_msgs::msg::Quaternion ros_orientation = tf2::toMsg(orientation);
  
  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  geometry_msgs::msg::Pose retreat_pose = target_pose;
  retreat_pose.orientation = ros_orientation;
  
  // 첫 번째 waypoint: target_pose + 0.04m
  retreat_pose.position.z += 0.04;
  retreat_waypoints.push_back(retreat_pose);
  
  // 두 번째 waypoint: target_pose + 0.08m (최종 retreat 위치)
  retreat_pose.position.z += 0.04;
  retreat_waypoints.push_back(retreat_pose);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm_->computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  if (fraction > 0.98) {
    RCLCPP_INFO(this->get_logger(), "Retreat Cartesian path planning successful (%.2f%%)", fraction * 100);
    move_group_arm_->execute(trajectory_retreat);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Retreat Cartesian path planning failed! (%.2f%%)", fraction * 100);
    
    // Fallback: 목표 위치 + 0.08m로 이동
    geometry_msgs::msg::Pose final_ascend = target_pose;
    final_ascend.orientation = ros_orientation;
    final_ascend.position.z += 0.08;
    
    move_group_arm_->setStartStateToCurrentState();
    move_group_arm_->setPoseTarget(final_ascend);
    
    moveit::planning_interface::MoveGroupInterface::Plan ascend_plan;
    bool success = (move_group_arm_->plan(ascend_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Ascend pose target planning successful");
      move_group_arm_->execute(ascend_plan);
      rclcpp::sleep_for(std::chrono::seconds(1));
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Both Cartesian and pose target ascend planning failed!");
      return false;
    }
  }
}

}  // namespace ur_pick_and_place

// main 함수 추가
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_pick_and_place::PickExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 