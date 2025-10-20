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
  
  // Pause/Resume 서비스 설정
  pause_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/pause",
    std::bind(&PickExecutorNode::pauseCallback, this, std::placeholders::_1, std::placeholders::_2));
  resume_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/resume",
    std::bind(&PickExecutorNode::resumeCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Pick Executor Node initialized with pause/resume support");
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
    // Resume 체크: 이전에 pause되었다면 중단된 단계부터 재개
    PickStep start_step = PickStep::MOVING_TO_PICK_POSITION;
    geometry_msgs::msg::Pose target_pose = goal->target_pose;
    
    if (is_resuming_) {
      RCLCPP_INFO(this->get_logger(), "Resuming pick action from paused step");
      start_step = paused_step_;
      target_pose = paused_target_;
      is_resuming_ = false;
    }
    
    // 각 단계 실행
    if (start_step <= PickStep::MOVING_TO_PICK_POSITION) {
      // 목표지점 + 0.08m 높이로 이동 (준비자세는 manager가 별도 처리)
      feedback->current_step = "Moving to pick position (0.08m above target)";
      feedback->completion_percentage = 30.0;
      goal_handle->publish_feedback(feedback);
      
      if (!moveToPickPosition(target_pose)) {
        if (is_paused_) {
          RCLCPP_INFO(this->get_logger(), "Pick action paused at MOVING_TO_PICK_POSITION");
          return;  // Pause 상태에서 종료 (action은 abort하지 않음)
        }
        result->success = false;
        result->message = "Failed to move to pick position";
        goal_handle->abort(result);
        return;
      }
      checkPauseAndWait();  // 단계 사이 pause 체크
    }
    
    if (start_step <= PickStep::OPENING_GRIPPER) {
      // 그리퍼 열기 (추후 구현)
      feedback->current_step = "Opening gripper (placeholder)";
      feedback->completion_percentage = 40.0;
      goal_handle->publish_feedback(feedback);
      // TODO: 그리퍼 열기 구현
      checkPauseAndWait();
    }
    
    if (start_step <= PickStep::DESCENDING) {
      // 0.08m 하강
      feedback->current_step = "Descending 0.08m to target position";
      feedback->completion_percentage = 60.0;
      goal_handle->publish_feedback(feedback);
      
      if (!descendToTarget(target_pose)) {
        if (is_paused_) {
          RCLCPP_INFO(this->get_logger(), "Pick action paused at DESCENDING");
          return;
        }
        result->success = false;
        result->message = "Failed to descend to target position";
        goal_handle->abort(result);
        return;
      }
      checkPauseAndWait();
    }
    
    if (start_step <= PickStep::CLOSING_GRIPPER) {
      // 그리퍼 닫기 (추후 구현)
      feedback->current_step = "Closing gripper (placeholder)";
      feedback->completion_percentage = 80.0;
      goal_handle->publish_feedback(feedback);
      // TODO: 그리퍼 닫기 구현
      checkPauseAndWait();
    }
    
    if (start_step <= PickStep::ASCENDING) {
      // 0.08m 상승
      feedback->current_step = "Ascending 0.08m from target position";
      feedback->completion_percentage = 90.0;
      goal_handle->publish_feedback(feedback);
      
      if (!ascendFromTarget(target_pose)) {
        if (is_paused_) {
          RCLCPP_INFO(this->get_logger(), "Pick action paused at ASCENDING");
          return;
        }
        result->success = false;
        result->message = "Failed to ascend from target position";
        goal_handle->abort(result);
        return;
      }
      checkPauseAndWait();
    }
    
    // 완료
    feedback->current_step = "Pick completed";
    feedback->completion_percentage = 100.0;
    goal_handle->publish_feedback(feedback);
    
    // 완료 시 상태 리셋
    paused_step_ = PickStep::IDLE;
    is_resuming_ = false;
    is_paused_ = false;
    
    result->success = true;
    result->message = "Pick action completed successfully";
    goal_handle->succeed(result);
    
    RCLCPP_INFO(this->get_logger(), "Pick action completed successfully");
    
  } catch (const std::exception & e) {
    // 실패 시에도 상태 리셋
    paused_step_ = PickStep::IDLE;
    is_resuming_ = false;
    is_paused_ = false;
    
    result->success = false;
    result->message = std::string("Exception during pick execution: ") + e.what();
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Exception during pick execution: %s", e.what());
  }
}

bool PickExecutorNode::moveToPickPosition(const geometry_msgs::msg::Pose & target_pose)
{
  RCLCPP_INFO(this->get_logger(), "Moving to pick position (0.08m above target)");
  
  // Pause/Resume 후 재시도를 위한 loop
  while (true) {
    // Ready pose 이동 완료 후 현재 상태를 확실히 가져오기
    // 1. 잠시 대기하여 시스템 안정화
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    
    // 2. 현재 상태를 명시적으로 업데이트
    move_group_arm_->setStartStateToCurrentState();
    
    // 3. 현재 joint 상태 로깅 (디버깅용)
    auto current_joints = move_group_arm_->getCurrentJointValues();
    RCLCPP_INFO(this->get_logger(), "Current joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                current_joints[0], current_joints[1], current_joints[2], 
                current_joints[3], current_joints[4], current_joints[5]);
    
    tf2::Quaternion orientation;
    orientation.setRPY(0, -PI, 0);
    geometry_msgs::msg::Quaternion ros_orientation = tf2::toMsg(orientation);

    geometry_msgs::msg::Pose pick_pose = target_pose;
    pick_pose.orientation = ros_orientation;
    pick_pose.position.z += 0.08; // 목표지점 + 0.08m 높이

    // Planning 설정을 더 유연하게 설정
    move_group_arm_->setPlanningTime(30.0);  // Planning 시간 증가
    move_group_arm_->setGoalTolerance(0.02); // 목표 허용 오차 증가
    move_group_arm_->setNumPlanningAttempts(10); // 재시도 횟수 증가
    
    move_group_arm_->setPoseTarget(pick_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Pick position planning successful");
      // Pause 지원하는 실행으로 변경
      paused_target_ = target_pose;  // Resume을 위해 목표 저장
      if (!executePlanWithPause(my_plan, PickStep::MOVING_TO_PICK_POSITION)) {
        // Pause되었다가 resume됨 - 재계획하여 다시 시도
        RCLCPP_INFO(this->get_logger(), "Paused and resumed, replanning from current position");
        continue;  // 처음부터 다시 (현재 위치에서 재계획)
      }
      rclcpp::sleep_for(std::chrono::seconds(2));
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Initial pick position planning failed, trying with different planner");
      
      // Fallback 1: 다른 플래너 시도
      move_group_arm_->setPlannerId("RRTstar");
      move_group_arm_->setPlanningTime(40.0);
      success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (success) {
        RCLCPP_INFO(this->get_logger(), "Pick position planning successful with RRTstar");
        paused_target_ = target_pose;
        if (!executePlanWithPause(my_plan, PickStep::MOVING_TO_PICK_POSITION)) {
          // Pause/Resume - 재시도
          move_group_arm_->setPlannerId("RRTConnect");
          continue;
        }
        rclcpp::sleep_for(std::chrono::seconds(2));
        // 원래 플래너로 복원
        move_group_arm_->setPlannerId("RRTConnect");
        return true;
      }
      
      // Fallback 2: 더 높은 위치에서 시도 (0.12m 높이)
      RCLCPP_WARN(this->get_logger(), "Trying higher pick position (0.12m above target)");
      pick_pose.position.z = target_pose.position.z + 0.12;
      move_group_arm_->setPoseTarget(pick_pose);
      move_group_arm_->setPlannerId("RRTConnect");
      
      success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (success) {
        RCLCPP_INFO(this->get_logger(), "Pick position planning successful at higher position");
        paused_target_ = target_pose;
        if (!executePlanWithPause(my_plan, PickStep::MOVING_TO_PICK_POSITION)) {
          // Pause/Resume - 재시도
          continue;
        }
        rclcpp::sleep_for(std::chrono::seconds(2));
        return true;
      }
      
      RCLCPP_ERROR(this->get_logger(), "All pick position planning attempts failed!");
      return false;
    }
  } // end while
}

bool PickExecutorNode::descendToTarget(const geometry_msgs::msg::Pose & target_pose)
{
  RCLCPP_INFO(this->get_logger(), "Approach to object!");
  
  // Pause/Resume 후 재시도 loop
  while (true) {
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
      paused_target_ = target_pose;
      if (!executeTrajectoryWithPause(trajectory_approach, PickStep::DESCENDING)) {
        continue;  // Pause/Resume - 재시도
      }
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
        paused_target_ = target_pose;
        if (!executePlanWithPause(descend_plan, PickStep::DESCENDING)) {
          continue;  // Pause/Resume - 재시도
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Both Cartesian and pose target descend planning failed!");
        return false;
      }
    }
  } // end while
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
    paused_target_ = target_pose;
    if (!executeTrajectoryWithPause(trajectory_retreat, PickStep::ASCENDING)) {
      return false;
    }
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
      paused_target_ = target_pose;
      if (!executePlanWithPause(ascend_plan, PickStep::ASCENDING)) {
        return false;
      }
      rclcpp::sleep_for(std::chrono::seconds(1));
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Both Cartesian and pose target ascend planning failed!");
      return false;
    }
  }
}

// Pause/Resume 콜백 함수들
void PickExecutorNode::pauseCallback(
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
  response->message = "Pick action paused";
  RCLCPP_INFO(this->get_logger(), "Pick action paused");
}

void PickExecutorNode::resumeCallback(
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
  response->message = "Pick action resumed";
  RCLCPP_INFO(this->get_logger(), "Pick action resumed");
}

void PickExecutorNode::checkPauseAndWait()
{
  while (is_paused_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                         "Pick action paused, waiting for resume...");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

bool PickExecutorNode::executePlanWithPause(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan,
  PickStep current_step)
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
      // (에러 로그를 방지하기 위해 명시적으로 완료 대기)
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

bool PickExecutorNode::executeTrajectoryWithPause(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  PickStep current_step)
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
  auto node = std::make_shared<ur_pick_and_place::PickExecutorNode>();
  
  // MultiThreadedExecutor 사용 - pause/resume service callback이 동시에 처리되어야 함
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
} 