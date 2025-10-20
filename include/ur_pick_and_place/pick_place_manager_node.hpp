#ifndef UR_PICK_AND_PLACE_PICK_PLACE_MANAGER_NODE_HPP_
#define UR_PICK_AND_PLACE_PICK_PLACE_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ur_pick_and_place/action/pick.hpp>
#include <ur_pick_and_place/action/place.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <std_srvs/srv/trigger.hpp>

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
  // 현재 실행 상태 추적
  enum class ExecutionState {
    IDLE,
    EXECUTING_READY,
    EXECUTING_PICK,
    EXECUTING_PLACE
  };

  rclcpp_action::Client<PickAction>::SharedPtr pick_action_client_;
  rclcpp_action::Client<PlaceAction>::SharedPtr place_action_client_;
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pick_goal_sub_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  
  geometry_msgs::msg::PoseStamped::SharedPtr current_pick_goal_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_place_goal_;  // 하드코딩된 위치 저장

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ready_client_;
  
  // Pause/Resume 통합 제어
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pick_pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pick_resume_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr place_pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr place_resume_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ready_pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ready_resume_client_;
  ExecutionState current_state_{ExecutionState::IDLE};
  
  void initializeHardcodedPlaceGoal();
  void pickGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  // Pause/Resume 콜백
  void pauseCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void resumeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  void executePickAndPlaceSequence();
  void sendPickGoal();
  void sendPlaceGoal();
  
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
  bool callReady();
  void callReadyAsync(std::function<void(bool)> on_done);
};

}  // namespace ur_pick_and_place

#endif  // UR_PICK_AND_PLACE_PICK_PLACE_MANAGER_NODE_HPP_ 