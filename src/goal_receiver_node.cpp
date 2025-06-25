#include "ur_pick_and_place/goal_receiver_node.hpp"

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
  if (msg->data == "start_pick_place") {
    if (!last_pick_goal_ || !last_place_goal_) {
      RCLCPP_ERROR(this->get_logger(), "Missing pick or place goals for sequence");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Triggering pick and place sequence");
    
    // Pick 목표만 먼저 발행 (Place는 pick 완료 후 manager에서 처리)
    pick_goal_pub_->publish(*last_pick_goal_);
    RCLCPP_INFO(this->get_logger(), "Sent pick goal to internal topic");
    
  } else if (msg->data == "start_pick") {
    if (!last_pick_goal_) {
      RCLCPP_ERROR(this->get_logger(), "Missing pick goal");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Triggering pick only");
    pick_goal_pub_->publish(*last_pick_goal_);
    RCLCPP_INFO(this->get_logger(), "Sent pick goal to internal topic");
    
  } else if (msg->data == "start_place") {
    if (!last_place_goal_) {
      RCLCPP_ERROR(this->get_logger(), "Missing place goal");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Triggering place only");
    place_goal_pub_->publish(*last_place_goal_);
    RCLCPP_INFO(this->get_logger(), "Sent place goal to internal topic");
    
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid trigger command: %s", msg->data.c_str());
  }
}

}  // namespace ur_pick_and_place 