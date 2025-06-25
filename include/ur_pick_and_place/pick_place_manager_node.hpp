#ifndef UR_PICK_AND_PLACE_PICK_PLACE_MANAGER_NODE_HPP_
#define UR_PICK_AND_PLACE_PICK_PLACE_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ur_pick_and_place/action/pick.hpp>
#include <ur_pick_and_place/action/place.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

namespace ur_pick_and_place
{

class PickPlaceManagerNode : public rclcpp::Node
{
public:
  using PickAction = ur_pick_and_place::action::Pick;
  using PlaceAction = ur_pick_and_place::action::Place;
  using GoalHandlePick = rclcpp_action::ClientGoalHandle<PickAction>;
  using GoalHandlePlace = rclcpp_action::ClientGoalHandle<PlaceAction>;

  explicit PickPlaceManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Client<PickAction>::SharedPtr pick_action_client_;
  rclcpp_action::Client<PlaceAction>::SharedPtr place_action_client_;
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pick_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr place_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_sub_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  
  geometry_msgs::msg::PoseStamped::SharedPtr current_pick_goal_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_place_goal_;
  
  void pickGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void placeGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void triggerCallback(const std_msgs::msg::String::SharedPtr msg);
  
  void sendPickGoal();
  void sendPlaceGoal();
  void executePickAndPlaceSequence();
  
  void pickGoalResponseCallback(const GoalHandlePick::SharedPtr & goal_handle);
  void pickFeedbackCallback(
    GoalHandlePick::SharedPtr,
    const std::shared_ptr<const PickAction::Feedback> feedback);
  void pickResultCallback(const GoalHandlePick::WrappedResult & result);
  
  void placeGoalResponseCallback(const GoalHandlePlace::SharedPtr & goal_handle);
  void placeFeedbackCallback(
    GoalHandlePlace::SharedPtr,
    const std::shared_ptr<const PlaceAction::Feedback> feedback);
  void placeResultCallback(const GoalHandlePlace::WrappedResult & result);
  
  void publishStatus(const std::string & status);
};

}  // namespace ur_pick_and_place

#endif  // UR_PICK_AND_PLACE_PICK_PLACE_MANAGER_NODE_HPP_ 