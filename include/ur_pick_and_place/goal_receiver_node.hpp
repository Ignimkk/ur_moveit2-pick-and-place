#ifndef UR_PICK_AND_PLACE_GOAL_RECEIVER_NODE_HPP_
#define UR_PICK_AND_PLACE_GOAL_RECEIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

namespace ur_pick_and_place
{

class GoalReceiverNode : public rclcpp::Node
{
public:
  explicit GoalReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pickGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void placeGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void triggerCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pick_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr place_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_sub_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pick_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr place_goal_pub_;
  
  geometry_msgs::msg::PoseStamped::SharedPtr last_pick_goal_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_place_goal_;
};

}  // namespace ur_pick_and_place

#endif  // UR_PICK_AND_PLACE_GOAL_RECEIVER_NODE_HPP_ 