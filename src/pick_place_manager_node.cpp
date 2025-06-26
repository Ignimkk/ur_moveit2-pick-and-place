#include "ur_pick_and_place/pick_place_manager_node.hpp"
#include <functional>
#include <memory>
#include <thread>

namespace ur_pick_and_place
{

PickPlaceManagerNode::PickPlaceManagerNode(const rclcpp::NodeOptions & options)
: Node("pick_place_manager_node", options)
{
  // 액션 클라이언트 설정
  pick_action_client_ = rclcpp_action::create_client<PickAction>(
    this, "pick_action");
  place_action_client_ = rclcpp_action::create_client<PlaceAction>(
    this, "place_action");
    
  // 구독자 설정 (internal goal만 처리)
  pick_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/internal/pick_goal", 10,
    std::bind(&PickPlaceManagerNode::pickGoalCallback, this, std::placeholders::_1));
    
  place_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/internal/place_goal", 10,
    std::bind(&PickPlaceManagerNode::placeGoalCallback, this, std::placeholders::_1));
    
  // Trigger 구독자 제거 - goal receiver가 이미 적절한 순서로 goal 발행
  // trigger_sub_ = this->create_subscription<std_msgs::msg::String>(
  //   "/pick_place_trigger", 10,
  //   std::bind(&PickPlaceManagerNode::triggerCallback, this, std::placeholders::_1));
    
  // 발행자 설정
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/pick_place_status", 10);

  RCLCPP_INFO(this->get_logger(), "Pick Place Manager Node initialized");
}

void PickPlaceManagerNode::pickGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pick_goal_ = msg;
  RCLCPP_INFO(this->get_logger(), "Manager received pick goal");
  
  // Pick과 place는 반드시 함께 실행되어야 함
  if (current_place_goal_) {
    RCLCPP_INFO(this->get_logger(), "Both pick and place goals available, starting pick and place sequence");
    executePickAndPlaceSequence();
  } else {
    RCLCPP_INFO(this->get_logger(), "Pick goal stored, waiting for place goal (pick-only execution not allowed)");
  }
}

void PickPlaceManagerNode::placeGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_place_goal_ = msg;
  RCLCPP_INFO(this->get_logger(), "Manager received place goal");
  
  // Pick과 place는 반드시 함께 실행되어야 함  
  if (current_pick_goal_) {
    RCLCPP_INFO(this->get_logger(), "Both pick and place goals available, starting pick and place sequence");
    executePickAndPlaceSequence();
  } else {
    RCLCPP_INFO(this->get_logger(), "Place goal stored, waiting for pick goal (place-only execution not allowed)");
  }
}

void PickPlaceManagerNode::executePickAndPlaceSequence()
{
  RCLCPP_INFO(this->get_logger(), "Starting pick and place sequence");
  publishStatus("starting pick and place sequence");
  
  // Pick부터 시작 (pick executor가 자체적으로 home 이동 후 pick 실행)
  // Place는 pick 완료 후 자동 실행
  sendPickGoal();
}

void PickPlaceManagerNode::sendPickGoal()
{
  if (!current_pick_goal_) {
    RCLCPP_ERROR(this->get_logger(), "No pick goal available");
    publishStatus("error: no pick goal");
    return;
  }
  
  if (!pick_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Pick action server not available");
    publishStatus("error: pick action server not available");
    return;
  }
  
  auto goal_msg = PickAction::Goal();
  goal_msg.target_pose = current_pick_goal_->pose;
  
  RCLCPP_INFO(this->get_logger(), "Sending pick goal");
  publishStatus("sending pick goal");
  
  auto send_goal_options = rclcpp_action::Client<PickAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&PickPlaceManagerNode::pickGoalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&PickPlaceManagerNode::pickFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&PickPlaceManagerNode::pickResultCallback, this, std::placeholders::_1);
    
  pick_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void PickPlaceManagerNode::sendPlaceGoal()
{
  if (!current_place_goal_) {
    RCLCPP_ERROR(this->get_logger(), "No place goal available");
    publishStatus("error: no place goal");
    return;
  }
  
  if (!place_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Place action server not available");
    publishStatus("error: place action server not available");
    return;
  }
  
  auto goal_msg = PlaceAction::Goal();
  goal_msg.target_pose = current_place_goal_->pose;
  
  RCLCPP_INFO(this->get_logger(), "Sending place goal");
  publishStatus("sending place goal");
  
  auto send_goal_options = rclcpp_action::Client<PlaceAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&PickPlaceManagerNode::placeGoalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&PickPlaceManagerNode::placeFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&PickPlaceManagerNode::placeResultCallback, this, std::placeholders::_1);
    
  place_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void PickPlaceManagerNode::pickGoalResponseCallback(const GoalHandlePick::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Pick goal was rejected by server");
    publishStatus("error: pick goal rejected");
  } else {
    RCLCPP_INFO(this->get_logger(), "Pick goal accepted by server");
    publishStatus("pick goal accepted");
  }
}

void PickPlaceManagerNode::pickFeedbackCallback(
  GoalHandlePick::SharedPtr,
  const std::shared_ptr<const PickAction::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Pick feedback: %s (%.1f%%)", 
              feedback->current_step.c_str(), feedback->completion_percentage);
  publishStatus("pick: " + feedback->current_step);
}

void PickPlaceManagerNode::pickResultCallback(const GoalHandlePick::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Pick action succeeded: %s", result.result->message.c_str());
      publishStatus("pick completed successfully");
      
      // Pick과 place는 반드시 함께 실행되어야 함
      if (current_place_goal_) {
        RCLCPP_INFO(this->get_logger(), "Pick completed, starting place action");
        publishStatus("pick completed, starting place action");
        
        // 잠시 대기 후 place 실행
        std::thread([this]() {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          sendPlaceGoal();
        }).detach();
      } else {
        // 이 상황은 발생하면 안 됨 (시스템 설계상 에러)
        RCLCPP_ERROR(this->get_logger(), "Critical error: Pick completed but no place goal available");
        publishStatus("error: incomplete sequence - missing place goal");
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Pick action aborted: %s", result.result->message.c_str());
      publishStatus("error: pick aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Pick action canceled");
      publishStatus("error: pick canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Pick action unknown result code");
      publishStatus("error: pick unknown result");
      break;
  }
}

void PickPlaceManagerNode::placeGoalResponseCallback(const GoalHandlePlace::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Place goal was rejected by server");
    publishStatus("error: place goal rejected");
  } else {
    RCLCPP_INFO(this->get_logger(), "Place goal accepted by server");
    publishStatus("place goal accepted");
  }
}

void PickPlaceManagerNode::placeFeedbackCallback(
  GoalHandlePlace::SharedPtr,
  const std::shared_ptr<const PlaceAction::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Place feedback: %s (%.1f%%)", 
              feedback->current_step.c_str(), feedback->completion_percentage);
  publishStatus("place: " + feedback->current_step);
}

void PickPlaceManagerNode::placeResultCallback(const GoalHandlePlace::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Place action succeeded: %s", result.result->message.c_str());
      publishStatus("pick and place sequence completed successfully");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Place action aborted: %s", result.result->message.c_str());
      publishStatus("error: place aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Place action canceled");
      publishStatus("error: place canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Place action unknown result code");
      publishStatus("error: place unknown result");
      break;
  }
}

void PickPlaceManagerNode::publishStatus(const std::string & status)
{
  auto status_msg = std_msgs::msg::String();
  status_msg.data = status;
  status_pub_->publish(status_msg);
}

}  // namespace ur_pick_and_place

// main 함수 추가
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_pick_and_place::PickPlaceManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 