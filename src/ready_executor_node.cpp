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

  RCLCPP_INFO(this->get_logger(), "Ready Executor Node initialized (service: /ready/move)");
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

  move_group_arm_->setStartStateToCurrentState();
  move_group_arm_->setJointValueTarget(ready_joint_positions_);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    move_group_arm_->execute(plan);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
  }
  RCLCPP_ERROR(this->get_logger(), "Ready pose planning failed");
  return false;
}

void ReadyExecutorNode::onReadyService(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  bool ok = moveToReady();
  response->success = ok;
  response->message = ok ? "Moved to ready pose" : "Failed to move to ready pose";
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