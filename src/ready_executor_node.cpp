#include "ur_pick_and_place/ready_executor_node.hpp"

#define PI 3.141592

namespace ur_pick_and_place
{

ReadyExecutorNode::ReadyExecutorNode(const rclcpp::NodeOptions & options)
: Node("ready_executor_node", options)
{
  setupMoveGroup();
  setupPlanningScene();
  declareAndLoadParameters();

  ready_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/ready/move",
    std::bind(&ReadyExecutorNode::onReadyService, this, std::placeholders::_1, std::placeholders::_2));
  
  // Pause/Resume용 별도 callback group 생성 (ready service와 동시 실행 가능하도록)
  pause_resume_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
  // Pause/Resume 서비스 설정 - 별도 callback group 사용
  pause_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/pause",
    std::bind(&ReadyExecutorNode::pauseCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    pause_resume_callback_group_);
  resume_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/resume",
    std::bind(&ReadyExecutorNode::resumeCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    pause_resume_callback_group_);

  RCLCPP_INFO(this->get_logger(), "Ready Executor Node initialized with pause/resume support");
}

void ReadyExecutorNode::setupMoveGroup()
{
  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

  move_group_arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    std::shared_ptr<rclcpp::Node>(this), PLANNING_GROUP_ARM);

  planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

  move_group_arm_->setPlannerId("RRTConnect");
  move_group_arm_->setPlanningTime(20.0);
  move_group_arm_->setNumPlanningAttempts(5);
  move_group_arm_->setGoalTolerance(0.01);
}

void ReadyExecutorNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_arm_->getPlanningFrame();
  collision_object.id = "ground";

  shape_msgs::msg::Plane plane;
  plane.coef = {0, 0, 1, 0};

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

void ReadyExecutorNode::declareAndLoadParameters()
{
  // 기본값: 살짝 펼친 준비자세 (예시)
  std::vector<double> default_ready = {0.0, -PI/2, PI/2, -PI/2, -PI/2, 0.0};
  this->declare_parameter<std::vector<double>>("ready_joint_positions", default_ready);
  this->get_parameter("ready_joint_positions", ready_joint_positions_);

  if (ready_joint_positions_.size() != 6) {
    RCLCPP_WARN(this->get_logger(), "ready_joint_positions has size %zu, expected 6. Falling back to defaults.", ready_joint_positions_.size());
    ready_joint_positions_ = default_ready;
  }
}

bool ReadyExecutorNode::moveToReady()
{
  RCLCPP_INFO(this->get_logger(), "Moving to ready pose");

  // Pause/Resume 후 재시도를 위한 loop
  while (true) {
    move_group_arm_->setStartStateToCurrentState();
    move_group_arm_->setJointValueTarget(ready_joint_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      // Pause 지원 실행
      if (!executePlanWithPause(plan)) {
        // Pause되었다가 resume됨 - 재계획하여 다시 시도
        RCLCPP_INFO(this->get_logger(), "Paused and resumed, replanning from current position");
        continue;  // 처음부터 다시 (현재 위치에서 재계획)
      }
      rclcpp::sleep_for(std::chrono::seconds(1));
      
      // 완료 시 상태 리셋
      is_paused_ = false;
      
      return true;
    }
    RCLCPP_ERROR(this->get_logger(), "Ready pose planning failed");
    return false;
  } // end while
}

void ReadyExecutorNode::onReadyService(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Ready service called, starting move to ready pose");
  // MultiThreadedExecutor를 사용하므로 pause/resume service는 다른 스레드에서 처리 가능
  bool ok = moveToReady();
  response->success = ok;
  response->message = ok ? "Moved to ready pose" : "Failed to move to ready pose";
  RCLCPP_INFO(this->get_logger(), "Ready service completed: %s", ok ? "success" : "failed");
}

// Pause/Resume 콜백 함수들
void ReadyExecutorNode::pauseCallback(
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
  response->message = "Ready motion paused";
  RCLCPP_INFO(this->get_logger(), "Ready motion paused");
}

void ReadyExecutorNode::resumeCallback(
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
  response->success = true;
  response->message = "Ready motion resumed";
  RCLCPP_INFO(this->get_logger(), "Ready motion resumed");
}

void ReadyExecutorNode::checkPauseAndWait()
{
  while (is_paused_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                         "Ready motion paused, waiting for resume...");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

bool ReadyExecutorNode::executePlanWithPause(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan)
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
      
      RCLCPP_INFO(this->get_logger(), "Ready trajectory stopped, waiting for resume");
      
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

} // namespace ur_pick_and_place

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_pick_and_place::ReadyExecutorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
} 