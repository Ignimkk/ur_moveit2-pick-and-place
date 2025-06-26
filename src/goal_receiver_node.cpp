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

  // 발행자 설정
  pick_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/internal/pick_goal", 10);
  place_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/internal/place_goal", 10);

  RCLCPP_INFO(this->get_logger(), "Goal Receiver Node initialized (Auto-execution mode)");
}

void GoalReceiverNode::pickGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_pick_goal_ = msg;
  RCLCPP_INFO(this->get_logger(), "Received pick goal: x=%.3f, y=%.3f, z=%.3f", 
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
              
  // Pick goal을 받으면 즉시 manager에게 전달
  pick_goal_pub_->publish(*last_pick_goal_);
  RCLCPP_INFO(this->get_logger(), "Forwarded pick goal to manager");
}

void GoalReceiverNode::placeGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_place_goal_ = msg;
  RCLCPP_INFO(this->get_logger(), "Received place goal: x=%.3f, y=%.3f, z=%.3f", 
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
              
  // Place goal을 받으면 즉시 manager에게 전달  
  place_goal_pub_->publish(*last_place_goal_);
  RCLCPP_INFO(this->get_logger(), "Forwarded place goal to manager");
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