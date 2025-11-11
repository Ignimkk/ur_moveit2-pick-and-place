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
    
  // Gripper 클라이언트
  gripper_client_ = this->create_client<ur_pick_and_place::srv::GripperControl>("/gripper/control");
  
  // Pause/Resume 서비스 설정
  pause_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/pause",
    std::bind(&PlaceExecutorNode::pauseCallback, this, std::placeholders::_1, std::placeholders::_2));
  resume_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/resume",
    std::bind(&PlaceExecutorNode::resumeCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Place Executor Node initialized with pause/resume support");
}

void PlaceExecutorNode::setupMoveGroup()
{
  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  
  move_group_arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    std::shared_ptr<rclcpp::Node>(this), PLANNING_GROUP_ARM);
    
  planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
  
  // 플래너 설정
  move_group_arm_->setPlannerId("RRTConnect");
  move_group_arm_->setPlanningTime(15.0);  // 충분한 시간을 주어 더 나은 경로 탐색
  move_group_arm_->setNumPlanningAttempts(3);  // 여러 경로 중 최선 선택
  move_group_arm_->setGoalTolerance(0.01);
  
  // 최소 움직임을 위한 최적화 설정
  move_group_arm_->setMaxVelocityScalingFactor(0.2);  // 느린 속도로 부드러운 움직임
  move_group_arm_->setMaxAccelerationScalingFactor(0.2);  // 느린 가속으로 부드러운 움직임
  
  // Goal tolerance를 세밀하게 설정
  move_group_arm_->setGoalPositionTolerance(0.005);  // 5mm
  move_group_arm_->setGoalOrientationTolerance(0.01);  // ~0.57도
  
  // Planning pipeline 파라미터 설정 (최소 움직임 강조)
  // 이 설정들은 trajectory를 단순화하고 불필요한 움직임 제거
  move_group_arm_->allowReplanning(true);  // 더 나은 경로를 찾으면 재계획
  move_group_arm_->allowLooking(true);  // 환경 정보 활용
  
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
    // Resume 체크: 이전에 pause되었다면 중단된 단계부터 재개
    PlaceStep start_step = PlaceStep::MOVING_TO_PLACE_POSITION;
    geometry_msgs::msg::Pose target_pose = goal->target_pose;
    
    if (is_resuming_) {
      RCLCPP_INFO(this->get_logger(), "Resuming place action from paused step");
      start_step = paused_step_;
      target_pose = paused_target_;
      is_resuming_ = false;
    }
    
    // 각 단계 실행
    if (start_step <= PlaceStep::MOVING_TO_PLACE_POSITION) {
      // Place 위치로 이동
      feedback->current_step = "Moving to place position";
      feedback->completion_percentage = 20.0;
      goal_handle->publish_feedback(feedback);
      
      if (!moveToPlacePosition(target_pose)) {
        if (is_paused_) {
          RCLCPP_INFO(this->get_logger(), "Place action paused at MOVING_TO_PLACE_POSITION");
          return;
        }
        result->success = false;
        result->message = "Failed to move to place position";
        goal_handle->abort(result);
        return;
      }
      checkPauseAndWait();
    }
    
    if (start_step <= PlaceStep::APPROACHING) {
      // Approach
      feedback->current_step = "Approaching place position";
      feedback->completion_percentage = 50.0;
      goal_handle->publish_feedback(feedback);
      
      if (!approachPlacePosition(target_pose)) {
        if (is_paused_) {
          RCLCPP_INFO(this->get_logger(), "Place action paused at APPROACHING");
          return;
        }
        result->success = false;
        result->message = "Failed to approach place position";
        goal_handle->abort(result);
        return;
      }
      checkPauseAndWait();
    }
    
    if (start_step <= PlaceStep::DROPPING) {
      // Drop
      feedback->current_step = "Dropping object";
      feedback->completion_percentage = 70.0;
      goal_handle->publish_feedback(feedback);
      
      if (!dropObject()) {
        if (is_paused_) {
          RCLCPP_INFO(this->get_logger(), "Place action paused at DROPPING");
          return;
        }
        result->success = false;
        result->message = "Failed to drop object";
        goal_handle->abort(result);
        return;
      }
      checkPauseAndWait();
    }
    
    if (start_step <= PlaceStep::RETREATING) {
      // Retreat
      feedback->current_step = "Retreating from place position";
      feedback->completion_percentage = 90.0;
      goal_handle->publish_feedback(feedback);
      
      if (!retreatFromPlacePosition(target_pose)) {
        if (is_paused_) {
          RCLCPP_INFO(this->get_logger(), "Place action paused at RETREATING");
          return;
        }
        result->success = false;
        result->message = "Failed to retreat from place position";
        goal_handle->abort(result);
        return;
      }
      checkPauseAndWait();
    }
    
    // 완료 시 상태 리셋
    paused_step_ = PlaceStep::IDLE;
    is_resuming_ = false;
    is_paused_ = false;
    
    result->success = true;
    result->message = "Place action completed successfully";
    goal_handle->succeed(result);
    
    RCLCPP_INFO(this->get_logger(), "Place action completed successfully");
    
  } catch (const std::exception & e) {
    // 실패 시에도 상태 리셋
    paused_step_ = PlaceStep::IDLE;
    is_resuming_ = false;
    is_paused_ = false;
    
    result->success = false;
    result->message = std::string("Exception during place execution: ") + e.what();
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Exception during place execution: %s", e.what());
  }
}

bool PlaceExecutorNode::moveToPlacePosition(const geometry_msgs::msg::Pose & target_pose)
{
  RCLCPP_INFO(this->get_logger(), "Moving to place position (Carry phase)");
  
  // Pause/Resume 후 재시도를 위한 loop
  while (true) {
    // 현재 end effector 위치 가져오기
    move_group_arm_->setStartStateToCurrentState();
    geometry_msgs::msg::PoseStamped current_pose = move_group_arm_->getCurrentPose();
    
    RCLCPP_INFO(this->get_logger(), "Current position: x=%.3f, y=%.3f, z=%.3f", 
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    
    // 토픽으로 받은 target_pose를 place 위치로 사용
    tf2::Quaternion orientation;
    orientation.setRPY(0, -PI, 0);
    geometry_msgs::msg::Quaternion ros_orientation = tf2::toMsg(orientation);
    
    geometry_msgs::msg::Pose carry_pose = target_pose;
    carry_pose.orientation = ros_orientation;
    carry_pose.position.z += 0.08;  // Place 높이 (0.08m 위)
    
    RCLCPP_INFO(this->get_logger(), "Target place position: x=%.3f, y=%.3f, z=%.3f", 
                carry_pose.position.x, carry_pose.position.y, carry_pose.position.z);
    
    // Cartesian path로 이동 시도
    std::vector<geometry_msgs::msg::Pose> carry_waypoints;
    carry_waypoints.push_back(carry_pose);
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    
    double fraction = move_group_arm_->computeCartesianPath(
        carry_waypoints, eef_step, trajectory);
    
    if (fraction > 0.95) {  // Place는 조금 더 관대한 임계값 사용
      RCLCPP_INFO(this->get_logger(), "Carry Cartesian path planning successful (%.2f%%)", fraction * 100);
      paused_target_ = target_pose;
      if (!executeTrajectoryWithPause(trajectory, PlaceStep::MOVING_TO_PLACE_POSITION)) {
        // Pause/Resume - 재시도
        RCLCPP_INFO(this->get_logger(), "Paused and resumed, replanning from current position");
        continue;
      }
      rclcpp::sleep_for(std::chrono::seconds(3));
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Carry Cartesian path planning failed (%.2f%%), trying pose target", fraction * 100);
      
      // Fallback: pose target 사용
      move_group_arm_->setStartStateToCurrentState();
      move_group_arm_->setPoseTarget(carry_pose);
      
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (success) {
        RCLCPP_INFO(this->get_logger(), "Carry pose target planning successful");
        paused_target_ = target_pose;
        if (!executePlanWithPause(my_plan, PlaceStep::MOVING_TO_PLACE_POSITION)) {
          // Pause/Resume - 재시도
          RCLCPP_INFO(this->get_logger(), "Paused and resumed, replanning from current position");
          continue;
        }
        rclcpp::sleep_for(std::chrono::seconds(3));
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Both Cartesian and pose target carry planning failed!");
        return false;
      }
    }
  } // end while
}

bool PlaceExecutorNode::approachPlacePosition(const geometry_msgs::msg::Pose & target_pose)
{
  RCLCPP_INFO(this->get_logger(), "Approaching place position");
  
  // Pause/Resume 후 재시도를 위한 loop
  while (true) {
    // Orientation 설정 (moveToPlacePosition과 동일)
    tf2::Quaternion orientation;
    orientation.setRPY(0, -PI, 0);
    geometry_msgs::msg::Quaternion ros_orientation = tf2::toMsg(orientation);
    
    move_group_arm_->setStartStateToCurrentState();
    
    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    
    // 첫 번째 waypoint: target_pose + 0.04m (중간 높이)
    geometry_msgs::msg::Pose approach_pose1 = target_pose;
    approach_pose1.orientation = ros_orientation;
    approach_pose1.position.z += 0.04;
    approach_waypoints.push_back(approach_pose1);
    
    // 두 번째 waypoint: target_pose (최종 place 높이)
    geometry_msgs::msg::Pose approach_pose2 = target_pose;
    approach_pose2.orientation = ros_orientation;
    approach_waypoints.push_back(approach_pose2);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;

    double fraction = move_group_arm_->computeCartesianPath(
        approach_waypoints, eef_step, trajectory);

    if (fraction > 0.95) {
      RCLCPP_INFO(this->get_logger(), "Approach Cartesian path planning successful (%.2f%%)", fraction * 100);
      paused_target_ = target_pose;
      if (!executeTrajectoryWithPause(trajectory, PlaceStep::APPROACHING)) {
        // Pause/Resume - 재시도
        RCLCPP_INFO(this->get_logger(), "Paused and resumed, replanning from current position");
        continue;
      }
      rclcpp::sleep_for(std::chrono::seconds(1));
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Approach Cartesian path planning failed (%.2f%%), trying pose target", fraction * 100);
      
      // Fallback: 직접 최종 위치로 이동
      geometry_msgs::msg::Pose final_approach = target_pose;
      final_approach.orientation = ros_orientation;
      
      move_group_arm_->setStartStateToCurrentState();
      move_group_arm_->setPoseTarget(final_approach);
      
      moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
      bool success = (move_group_arm_->plan(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (success) {
        RCLCPP_INFO(this->get_logger(), "Approach pose target planning successful");
        paused_target_ = target_pose;
        if (!executePlanWithPause(approach_plan, PlaceStep::APPROACHING)) {
          // Pause/Resume - 재시도
          RCLCPP_INFO(this->get_logger(), "Paused and resumed, replanning from current position");
          continue;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Both Cartesian and pose target approach planning failed!");
        return false;
      }
    }
  } // end while
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
  
  // Pause/Resume 후 재시도를 위한 loop
  while (true) {
    // Orientation 설정 (일관성 유지)
    tf2::Quaternion orientation;
    orientation.setRPY(0, -PI, 0);
    geometry_msgs::msg::Quaternion ros_orientation = tf2::toMsg(orientation);
    
    move_group_arm_->setStartStateToCurrentState();
    
    std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
    
    // 첫 번째 waypoint: target_pose + 0.04m
    geometry_msgs::msg::Pose retreat_pose1 = target_pose;
    retreat_pose1.orientation = ros_orientation;
    retreat_pose1.position.z += 0.04;
    retreat_waypoints.push_back(retreat_pose1);
    
    // 두 번째 waypoint: target_pose + 0.08m (carry 높이로 복귀)
    geometry_msgs::msg::Pose retreat_pose2 = target_pose;
    retreat_pose2.orientation = ros_orientation;
    retreat_pose2.position.z += 0.08;
    retreat_waypoints.push_back(retreat_pose2);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;

    double fraction = move_group_arm_->computeCartesianPath(
        retreat_waypoints, eef_step, trajectory);

    if (fraction > 0.95) {
      RCLCPP_INFO(this->get_logger(), "Retreat Cartesian path planning successful (%.2f%%)", fraction * 100);
      paused_target_ = target_pose;
      if (!executeTrajectoryWithPause(trajectory, PlaceStep::RETREATING)) {
        // Pause/Resume - 재시도
        RCLCPP_INFO(this->get_logger(), "Paused and resumed, replanning from current position");
        continue;
      }
      rclcpp::sleep_for(std::chrono::seconds(1));
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Retreat Cartesian path planning failed (%.2f%%), trying pose target", fraction * 100);
      
      // Fallback: 최종 retreat 위치로 직접 이동
      geometry_msgs::msg::Pose final_retreat = target_pose;
      final_retreat.orientation = ros_orientation;
      final_retreat.position.z += 0.08;
      
      move_group_arm_->setStartStateToCurrentState();
      move_group_arm_->setPoseTarget(final_retreat);
      
      moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
      bool success = (move_group_arm_->plan(retreat_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (success) {
        RCLCPP_INFO(this->get_logger(), "Retreat pose target planning successful");
        paused_target_ = target_pose;
        if (!executePlanWithPause(retreat_plan, PlaceStep::RETREATING)) {
          // Pause/Resume - 재시도
          RCLCPP_INFO(this->get_logger(), "Paused and resumed, replanning from current position");
          continue;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Both Cartesian and pose target retreat planning failed!");
        return false;
      }
    }
  } // end while
}

// Pause/Resume 콜백 함수들
void PlaceExecutorNode::pauseCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  std::lock_guard<std::mutex> lock(pause_mutex_);
  
  if (is_paused_) {
    response->success = false;
    response->message = "Already paused";
    RCLCPP_WARN(this->get_logger(), "Pause requested but already paused");
    return;
  }
  
  is_paused_ = true;
  response->success = true;
  response->message = "Place action paused";
  RCLCPP_INFO(this->get_logger(), "Place action paused");
}

void PlaceExecutorNode::resumeCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  std::lock_guard<std::mutex> lock(pause_mutex_);
  
  if (!is_paused_) {
    response->success = false;
    response->message = "Not paused";
    RCLCPP_WARN(this->get_logger(), "Resume requested but not paused");
    return;
  }
  
  is_paused_ = false;
  is_resuming_ = true;
  response->success = true;
  response->message = "Place action resumed";
  RCLCPP_INFO(this->get_logger(), "Place action resumed");
}

void PlaceExecutorNode::checkPauseAndWait()
{
  while (is_paused_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                         "Place action paused, waiting for resume...");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

bool PlaceExecutorNode::executePlanWithPause(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan,
  PlaceStep current_step)
{
  // asyncExecute를 사용하여 non-blocking으로 실행
  auto execute_future = std::async(std::launch::async, [this, &plan]() {
    return move_group_arm_->execute(plan);
  });
  
  // 100ms마다 pause 상태 체크
  while (execute_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    if (is_paused_) {
      RCLCPP_WARN(this->get_logger(), "Pause detected during trajectory execution");
      
      // 즉시 정지 시도 - stop()만 호출
      move_group_arm_->stop();
      
      // 현재 단계 저장
      paused_step_ = current_step;
      
      RCLCPP_INFO(this->get_logger(), "Trajectory stopped, waiting for resume");
      
      // stop()으로 인해 execute가 실패하므로, future 결과를 받아서 버림
      execute_future.wait();
      
      // Pause 상태에서 대기
      checkPauseAndWait();
      
      RCLCPP_INFO(this->get_logger(), "Resumed, will replan from current position");
      
      // Resume되면 false 반환하여 재계획 요청
      return false;
    }
  }
  
  // 실행 완료 확인
  auto result = execute_future.get();
  return (result == moveit::core::MoveItErrorCode::SUCCESS);
}

bool PlaceExecutorNode::executeTrajectoryWithPause(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  PlaceStep current_step)
{
  // asyncExecute를 사용하여 non-blocking으로 실행
  auto execute_future = std::async(std::launch::async, [this, &trajectory]() {
    return move_group_arm_->execute(trajectory);
  });
  
  // 100ms마다 pause 상태 체크
  while (execute_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    if (is_paused_) {
      RCLCPP_WARN(this->get_logger(), "Pause detected during trajectory execution");
      
      // 즉시 정지 시도 - stop()만 호출
      move_group_arm_->stop();
      
      // 현재 단계 저장
      paused_step_ = current_step;
      
      RCLCPP_INFO(this->get_logger(), "Trajectory stopped, waiting for resume");
      
      // stop()으로 인해 execute가 실패하므로, future 결과를 받아서 버림
      execute_future.wait();
      
      // Pause 상태에서 대기
      checkPauseAndWait();
      
      RCLCPP_INFO(this->get_logger(), "Resumed, will replan from current position");
      
      // Resume되면 false 반환하여 재계획 요청
      return false;
    }
  }
  
  // 실행 완료 확인
  auto result = execute_future.get();
  return (result == moveit::core::MoveItErrorCode::SUCCESS);
}

}  // namespace ur_pick_and_place

// main 함수 추가  
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_pick_and_place::PlaceExecutorNode>();
  
  // MultiThreadedExecutor 사용 - pause/resume service callback이 동시에 처리되어야 함
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
} 