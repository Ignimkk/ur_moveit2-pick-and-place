#include "ur_pick_and_place/goal_receiver_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ur_pick_and_place
{

GoalReceiverNode::GoalReceiverNode(const rclcpp::NodeOptions & options)
: Node("goal_receiver_node", options)
{
  // 구독자 설정
  pick_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/pick_goal", 10, 
    std::bind(&GoalReceiverNode::pickGoalCallback, this, std::placeholders::_1));
    
  place_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/place_goal", 10, 
    std::bind(&GoalReceiverNode::placeGoalCallback, this, std::placeholders::_1));
    
  trigger_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/pick_place_trigger", 10, 
    std::bind(&GoalReceiverNode::triggerCallback, this, std::placeholders::_1));

  // Place trigger 구독자 제거 - manager가 시퀀스 관리
  // place_trigger_sub_ = this->create_subscription<std_msgs::msg::String>(
  //   "/internal/place_trigger", 10,
  //   std::bind(&GoalReceiverNode::placeTriggerCallback, this, std::placeholders::_1));

  // 발행자 설정
  pick_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/internal/pick_goal", 10);
  place_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/internal/place_goal", 10);

  RCLCPP_INFO(this->get_logger(), "Goal Receiver Node initialized");
}

void GoalReceiverNode::pickGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_pick_goal_ = msg;
  RCLCPP_INFO(this->get_logger(), "Received pick goal: x=%.3f, y=%.3f, z=%.3f", 
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void GoalReceiverNode::placeGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_place_goal_ = msg;
  RCLCPP_INFO(this->get_logger(), "Received place goal: x=%.3f, y=%.3f, z=%.3f", 
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void GoalReceiverNode::triggerCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received trigger: %s", msg->data.c_str());
  
  if (msg->data == "start_pick_place") {
    if (!last_pick_goal_ || !last_place_goal_) {
      RCLCPP_ERROR(this->get_logger(), "Missing pick or place goals for sequence");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Starting pick and place sequence");
    
    // Pick과 place goal 모두 발행하여 manager가 시퀀스 관리하도록 함
    pick_goal_pub_->publish(*last_pick_goal_);
    place_goal_pub_->publish(*last_place_goal_);
    RCLCPP_INFO(this->get_logger(), "Sent both pick and place goals to manager");
    
  } else if (msg->data == "start_pick") {
    if (!last_pick_goal_) {
      RCLCPP_ERROR(this->get_logger(), "Missing pick goal");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Forwarding pick goal to manager");
    pick_goal_pub_->publish(*last_pick_goal_);
    RCLCPP_INFO(this->get_logger(), "Sent pick goal to internal topic");
    
  } else if (msg->data == "start_place") {
    if (!last_place_goal_) {
      RCLCPP_ERROR(this->get_logger(), "Missing place goal");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Forwarding place goal to manager");
    place_goal_pub_->publish(*last_place_goal_);
    RCLCPP_INFO(this->get_logger(), "Sent place goal to internal topic");
    
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid trigger command: %s", msg->data.c_str());
  }
}

}  // namespace ur_pick_and_place

// main 함수 추가
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_pick_and_place::GoalReceiverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 