#include "ur_pick_and_place/place_executor_node.hpp"
#include <thread>

#define PI 3.141592

namespace ur_pick_and_place
{

PlaceExecutorNode::PlaceExecutorNode(const rclcpp::NodeOptions & options)
: Node("place_executor_node", options)
{
  // MoveIt 설정
  setupMoveGroup();
  setupPlanningScene();
  
  // 액션 서버 설정
  this->action_server_ = rclcpp_action::create_server<PlaceAction>(
    this,
    "place_action",
    std::bind(&PlaceExecutorNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PlaceExecutorNode::handleCancel, this, std::placeholders::_1),
    std::bind(&PlaceExecutorNode::handleAccepted, this, std::placeholders::_1));
    
  // 목표 수신 구독자
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/internal/place_goal", 10,
    std::bind(&PlaceExecutorNode::goalCallback, this, std::placeholders::_1));
    
  // Gripper 클라이언트
  gripper_client_ = this->create_client<ur_pick_and_place::srv::GripperControl>("/gripper/control");

  RCLCPP_INFO(this->get_logger(), "Place Executor Node initialized");
}

void PlaceExecutorNode::setupMoveGroup()
{
  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  
  move_group_arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    std::shared_ptr<rclcpp::Node>(this), PLANNING_GROUP_ARM);
    
  planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
  
  // 플래너 설정
  move_group_arm_->setPlannerId("RRTConnect");
  move_group_arm_->setPlanningTime(15.0);
  
  RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_arm_->getPlanningFrame().c_str());
  RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_arm_->getEndEffectorLink().c_str());
}

void PlaceExecutorNode::setupPlanningScene()
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

void PlaceExecutorNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received place goal via topic");
  
  // 토픽으로 받은 목표를 직접 실행 (액션 서버를 거치지 않음)
  executePlaceDirectly(msg->pose);
}

void PlaceExecutorNode::executePlaceDirectly(const geometry_msgs::msg::Pose & target_pose)
{
  RCLCPP_INFO(this->get_logger(), "Executing place action directly");
  
  try {
    // Place 위치로 이동
    RCLCPP_INFO(this->get_logger(), "Moving to place position");
    if (!moveToPlacePosition(target_pose)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to place position");
      return;
    }
    
    // Approach
    RCLCPP_INFO(this->get_logger(), "Approaching place position");
    if (!approachPlacePosition(target_pose)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to approach place position");
      return;
    }
    
    // Drop
    RCLCPP_INFO(this->get_logger(), "Dropping object");
    if (!dropObject()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to drop object");
      return;
    }
    
    // Retreat
    RCLCPP_INFO(this->get_logger(), "Retreating from place position");
    if (!retreatFromPlacePosition(target_pose)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to retreat from place position");
      return;
    }
    
    // Home으로 이동
    RCLCPP_INFO(this->get_logger(), "Moving to home position");
    if (!moveToHome()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to home position");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Place action completed successfully");
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during place execution: %s", e.what());
  }
}

rclcpp_action::GoalResponse PlaceExecutorNode::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PlaceAction::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received place goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlaceExecutorNode::handleCancel(
  const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel place goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlaceExecutorNode::handleAccepted(const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  std::thread{std::bind(&PlaceExecutorNode::executePlace, this, goal_handle)}.detach();
}

void PlaceExecutorNode::executePlace(const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing place action");
  
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PlaceAction::Feedback>();
  auto result = std::make_shared<PlaceAction::Result>();
  
  try {
    // Place 위치로 이동
    feedback->current_step = "Moving to place position";
    feedback->completion_percentage = 20.0;
    goal_handle->publish_feedback(feedback);
    
    if (!moveToPlacePosition(goal->target_pose)) {
      result->success = false;
      result->message = "Failed to move to place position";
      goal_handle->abort(result);
      return;
    }
    
    // Approach
    feedback->current_step = "Approaching place position";
    feedback->completion_percentage = 50.0;
    goal_handle->publish_feedback(feedback);
    
    if (!approachPlacePosition(goal->target_pose)) {
      result->success = false;
      result->message = "Failed to approach place position";
      goal_handle->abort(result);
      return;
    }
    
    // Drop
    feedback->current_step = "Dropping object";
    feedback->completion_percentage = 70.0;
    goal_handle->publish_feedback(feedback);
    
    if (!dropObject()) {
      result->success = false;
      result->message = "Failed to drop object";
      goal_handle->abort(result);
      return;
    }
    
    // Retreat
    feedback->current_step = "Retreating from place position";
    feedback->completion_percentage = 90.0;
    goal_handle->publish_feedback(feedback);
    
    if (!retreatFromPlacePosition(goal->target_pose)) {
      result->success = false;
      result->message = "Failed to retreat from place position";
      goal_handle->abort(result);
      return;
    }
    
    // Home으로 이동
    feedback->current_step = "Moving to home position";
    feedback->completion_percentage = 100.0;
    goal_handle->publish_feedback(feedback);
    
    if (!moveToHome()) {
      result->success = false;
      result->message = "Failed to move to home position";
      goal_handle->abort(result);
      return;
    }
    
    result->success = true;
    result->message = "Place action completed successfully";
    goal_handle->succeed(result);
    
    RCLCPP_INFO(this->get_logger(), "Place action completed successfully");
    
  } catch (const std::exception & e) {
    result->success = false;
    result->message = std::string("Exception during place execution: ") + e.what();
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Exception during place execution: %s", e.what());
  }
}

bool PlaceExecutorNode::moveToPlacePosition(const geometry_msgs::msg::Pose & target_pose)
{
  RCLCPP_INFO(this->get_logger(), "Moving to place position");
  
  move_group_arm_->setStartStateToCurrentState();
  
  tf2::Quaternion orientation;
  orientation.setRPY(0, -PI, 0);
  geometry_msgs::msg::Quaternion ros_orientation = tf2::toMsg(orientation);

  geometry_msgs::msg::Pose place_pose = target_pose;
  place_pose.orientation = ros_orientation;
  place_pose.position.z += 0.08; // Place 높이

  move_group_arm_->setPoseTarget(place_pose);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    move_group_arm_->execute(my_plan);
    rclcpp::sleep_for(std::chrono::seconds(3));
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Place position planning failed!");
    return false;
  }
}

bool PlaceExecutorNode::approachPlacePosition(const geometry_msgs::msg::Pose & target_pose)
{
  RCLCPP_INFO(this->get_logger(), "Approaching place position");
  
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  geometry_msgs::msg::Pose approach_pose = target_pose;
  
  approach_pose.position.z -= 0.04;
  approach_waypoints.push_back(approach_pose);
  
  approach_pose.position.z -= 0.04;
  approach_waypoints.push_back(approach_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm_->computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory);

  if (fraction > 0.98) {
    move_group_arm_->execute(trajectory);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Place approach Cartesian path planning failed! (%.2f%%)", fraction * 100);
    return false;
  }
}

bool PlaceExecutorNode::dropObject()
{
  RCLCPP_INFO(this->get_logger(), "Dropping object");
  
  // Gripper 열기
  auto request = std::make_shared<ur_pick_and_place::srv::GripperControl::Request>();
  request->command = "open";
  
  auto future = gripper_client_->async_send_request(request);
  
  if (future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
    RCLCPP_ERROR(this->get_logger(), "Gripper service call timed out");
    return false;
  }
  
  auto response = future.get();
  return response->success;
}

bool PlaceExecutorNode::retreatFromPlacePosition(const geometry_msgs::msg::Pose & target_pose)
{
  RCLCPP_INFO(this->get_logger(), "Retreating from place position");
  
  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  geometry_msgs::msg::Pose retreat_pose = target_pose;
  
  retreat_pose.position.z += 0.04;
  retreat_waypoints.push_back(retreat_pose);
  
  retreat_pose.position.z += 0.04;
  retreat_waypoints.push_back(retreat_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm_->computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory);

  if (fraction > 0.98) {
    move_group_arm_->execute(trajectory);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Place retreat Cartesian path planning failed! (%.2f%%)", fraction * 100);
    return false;
  }
}

bool PlaceExecutorNode::moveToHome()
{
  RCLCPP_INFO(this->get_logger(), "Moving to home position");
  
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
    move_group_arm_->execute(my_plan);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Home position planning failed!");
    return false;
  }
}

}  // namespace ur_pick_and_place 