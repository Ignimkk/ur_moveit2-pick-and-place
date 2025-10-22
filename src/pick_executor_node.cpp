#include "ur_pick_and_place/pick_executor_node.hpp"
#include <thread>

#define PI 3.141592

namespace ur_pick_and_place
{

PickExecutorNode::PickExecutorNode(const rclcpp::NodeOptions & options)
: Node("pick_executor_node", options)
{
  // 파라미터 선언 및 가져오기
  this->declare_parameter<bool>("use_cartesian_path", true);
  use_cartesian_path_ = this->get_parameter("use_cartesian_path").as_bool();
  
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

  // Parameter callback 등록 (런타임에 파라미터 변경 가능)
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PickExecutorNode::parametersCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "Pick Executor Node initialized with pause/resume support");
  RCLCPP_INFO(this->get_logger(), "Planning strategy: %s", 
              use_cartesian_path_ ? "Cartesian Path (with RRT fallback)" : "RRT only");
}

void PickExecutorNode::setupMoveGroup()
{
  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  
  move_group_arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    std::shared_ptr<rclcpp::Node>(this), PLANNING_GROUP_ARM);
    
  planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
  
  // Trajectory 실행 설정 (Cartesian Path와 Standard Planning 모두에 영향)
  move_group_arm_->setMaxVelocityScalingFactor(0.2);      // 느린 속도로 부드러운 움직임
  move_group_arm_->setMaxAccelerationScalingFactor(0.2);  // 느린 가속으로 부드러운 움직임
  
  // Fallback 플래너 설정 (Cartesian Path 실패 시만 사용)
  move_group_arm_->setPlannerId("RRTConnect");            // 기본 플래너
  move_group_arm_->setPlanningTime(15.0);                 // Planning 시간
  move_group_arm_->setNumPlanningAttempts(3);             // 재시도 횟수
  move_group_arm_->setGoalPositionTolerance(0.005);       // 위치 허용오차 5mm
  move_group_arm_->setGoalOrientationTolerance(0.01);     // 방향 허용오차 ~0.57도
  
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
      // 픽킹 위치로 바로 이동 (준비자세는 manager가 별도 처리)
      feedback->current_step = "Moving to pick position";
      feedback->completion_percentage = 40.0;
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
      feedback->completion_percentage = 60.0;
      goal_handle->publish_feedback(feedback);
      // TODO: 그리퍼 열기 구현
      checkPauseAndWait();
    }
    
    if (start_step <= PickStep::CLOSING_GRIPPER) {
      // 그리퍼 닫기 (잡기)
      feedback->current_step = "Closing gripper (placeholder)";
      feedback->completion_percentage = 80.0;
      goal_handle->publish_feedback(feedback);
      // TODO: 그리퍼 닫기 구현
      checkPauseAndWait();
    }
    
    if (start_step <= PickStep::ASCENDING) {
      // 상승 (0.08m)
      feedback->current_step = "Ascending from pick position";
      feedback->completion_percentage = 90.0;
      goal_handle->publish_feedback(feedback);
      
      if (!ascendFromTarget(target_pose)) {
        if (is_paused_) {
          RCLCPP_INFO(this->get_logger(), "Pick action paused at ASCENDING");
          return;
        }
        result->success = false;
        result->message = "Failed to ascend from pick position";
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
  RCLCPP_INFO(this->get_logger(), "Moving to pick position");
  
  // Pause/Resume 후 재시도를 위한 loop
  while (true) {
    // Ready pose 이동 완료 후 현재 상태를 확실히 가져오기
    // 1. 잠시 대기하여 시스템 안정화
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    
    // 2. 현재 상태를 명시적으로 업데이트
    move_group_arm_->setStartStateToCurrentState();
    
    // 3. 현재 위치 확인
    geometry_msgs::msg::PoseStamped current_pose = move_group_arm_->getCurrentPose();
    auto current_joints = move_group_arm_->getCurrentJointValues();
    RCLCPP_INFO(this->get_logger(), "Current joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                current_joints[0], current_joints[1], current_joints[2], 
                current_joints[3], current_joints[4], current_joints[5]);
    
    tf2::Quaternion orientation;
    orientation.setRPY(0, -PI, 0);
    geometry_msgs::msg::Quaternion ros_orientation = tf2::toMsg(orientation);

    geometry_msgs::msg::Pose pick_pose = target_pose;
    pick_pose.orientation = ros_orientation;
    
    // **방법 1: Cartesian Path (파라미터로 활성화/비활성화 가능)**
    if (use_cartesian_path_) {
      RCLCPP_INFO(this->get_logger(), "Trying Cartesian path for minimum end-effector travel distance");
      
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(pick_pose);
      
      moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
      const double eef_step = 0.01;  // 1cm 단위로 보간
      const double jump_threshold = 0.0;  // Jump 허용 안함 (직선 경로 강제)
      
      double fraction = move_group_arm_->computeCartesianPath(
          waypoints, eef_step, jump_threshold, cartesian_trajectory);
      
      // Cartesian path가 95% 이상 성공하면 사용 (거의 직선 경로)
      if (fraction > 0.95) {
        // Cartesian 경로 길이 계산
        double cart_distance = 0.0;
        for (size_t i = 1; i < cartesian_trajectory.joint_trajectory.points.size(); ++i) {
          // 간단한 거리 추정 (joint space distance)
          double segment_dist = 0.0;
          for (size_t j = 0; j < 6; ++j) {
            double diff = cartesian_trajectory.joint_trajectory.points[i].positions[j] - 
                         cartesian_trajectory.joint_trajectory.points[i-1].positions[j];
            segment_dist += diff * diff;
          }
          cart_distance += std::sqrt(segment_dist);
        }
        
        RCLCPP_INFO(this->get_logger(), 
                    "Cartesian path planning successful (%.2f%%, waypoints: %zu, distance: %.3f)", 
                    fraction * 100, cartesian_trajectory.joint_trajectory.points.size(), cart_distance);
        
        paused_target_ = target_pose;
        if (!executeTrajectoryWithPause(cartesian_trajectory, PickStep::MOVING_TO_PICK_POSITION)) {
          RCLCPP_INFO(this->get_logger(), "Paused and resumed, replanning from current position");
          continue;
        }
        rclcpp::sleep_for(std::chrono::seconds(2));
        return true;
      }
      
      RCLCPP_WARN(this->get_logger(), 
                  "Cartesian path incomplete (%.2f%%), falling back to standard planning", fraction * 100);
    } else {
      RCLCPP_INFO(this->get_logger(), "Cartesian path disabled, using standard planning (RRT)");
    }

    // **방법 2: Standard Planning (RRT - setupMoveGroup에서 설정한 기본 플래너 사용)**
    // setupMoveGroup()에서 이미 RRTConnect, planning time, goal tolerance 등 설정됨
    move_group_arm_->setPoseTarget(pick_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      auto trajectory = my_plan.trajectory_;
      size_t num_points = trajectory.joint_trajectory.points.size();
      RCLCPP_INFO(this->get_logger(), 
                  "Standard planning successful (waypoints: %zu)", num_points);
      
      paused_target_ = target_pose;
      if (!executePlanWithPause(my_plan, PickStep::MOVING_TO_PICK_POSITION)) {
        RCLCPP_INFO(this->get_logger(), "Paused and resumed, replanning from current position");
        continue;
      }
      rclcpp::sleep_for(std::chrono::seconds(2));
      return true;
    }
    
    RCLCPP_ERROR(this->get_logger(), "All pick position planning attempts failed!");
    return false;
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

rcl_interfaces::msg::SetParametersResult PickExecutorNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto & param : parameters) {
    if (param.get_name() == "use_cartesian_path") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        bool new_value = param.as_bool();
        if (use_cartesian_path_ != new_value) {
          use_cartesian_path_ = new_value;
          RCLCPP_INFO(this->get_logger(), 
                      "Planning strategy changed to: %s", 
                      use_cartesian_path_ ? "Cartesian Path (with RRT fallback)" : "RRT only");
          result.reason = "Planning strategy updated";
        }
      } else {
        result.successful = false;
        result.reason = "use_cartesian_path must be a boolean";
      }
    }
  }
  
  return result;
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