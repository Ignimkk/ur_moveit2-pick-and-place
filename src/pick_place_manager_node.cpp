#include "ur_pick_and_place/pick_place_manager_node.hpp"
#include <functional>
#include <memory>
#include <thread>
#include <algorithm>
#include <string>

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
    
  // 구독자 설정 - Pick goal만 수신
  pick_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/internal/pick_goal", 10,
    std::bind(&PickPlaceManagerNode::pickGoalCallback, this, std::placeholders::_1));
    
  // 발행자 설정
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/pick_place_status", 10);

  // Ready 서비스 클라이언트
  ready_client_ = this->create_client<std_srvs::srv::Trigger>("/ready/move");
  
  // Pause/Resume 토픽 기반 제어
  cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/pick_place/cmd", 10,
    std::bind(&PickPlaceManagerNode::cmdCallback, this, std::placeholders::_1));
  
  pick_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/pick_executor_node/cmd", 10);
  place_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/place_executor_node/cmd", 10);
  ready_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/ready_executor_node/cmd", 10);

  // 하드코딩된 place 위치 초기화
  initializeHardcodedPlaceGoal();

  RCLCPP_INFO(this->get_logger(), "Pick Place Manager Node initialized with topic-based pause/resume control");
}

void PickPlaceManagerNode::initializeHardcodedPlaceGoal()
{
  // 하드코딩된 place 위치 설정
  current_place_goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  current_place_goal_->header.frame_id = "base_link";
  current_place_goal_->pose.position.x = -0.340;
  current_place_goal_->pose.position.y = 0.310;
  current_place_goal_->pose.position.z = 0.264;
  current_place_goal_->pose.orientation.w = 1.0;
  current_place_goal_->pose.orientation.x = 0.0;
  current_place_goal_->pose.orientation.y = 0.0;
  current_place_goal_->pose.orientation.z = 0.0;
  
  RCLCPP_INFO(this->get_logger(), "Hardcoded place position set: x=%.3f, y=%.3f, z=%.3f",
              current_place_goal_->pose.position.x,
              current_place_goal_->pose.position.y,
              current_place_goal_->pose.position.z);
}

void PickPlaceManagerNode::pickGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pick_goal_ = msg;
  RCLCPP_INFO(this->get_logger(), "Manager received pick goal");
  
  // Pick goal을 받으면 바로 시퀀스 시작 (place는 이미 하드코딩됨)
  RCLCPP_INFO(this->get_logger(), "Pick goal received, starting pick and place sequence with hardcoded place position");
  executePickAndPlaceSequence();
}

void PickPlaceManagerNode::executePickAndPlaceSequence()
{
  RCLCPP_INFO(this->get_logger(), "Starting pick and place sequence");
  publishStatus("starting pick and place sequence");
  
  // 준비자세 이동
  publishStatus("moving to ready pose");
  current_state_ = ExecutionState::EXECUTING_READY;
  
  callReadyAsync([this](bool ok){
    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to ready pose before pick");
      publishStatus("error: failed to move to ready before pick");
      current_state_ = ExecutionState::IDLE;
      return;
    }
    
    // Ready pose 완료 후 안정화를 위한 추가 대기
    RCLCPP_INFO(this->get_logger(), "Ready pose completed, waiting for stabilization");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    
    // Pick 시작
    sendPickGoal();
  });
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
  
  // 상태 업데이트
  current_state_ = ExecutionState::EXECUTING_PICK;
  
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
  
  // 상태 업데이트
  current_state_ = ExecutionState::EXECUTING_PLACE;
  
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
      
      // Pick 완료 후 place 실행
      if (current_place_goal_) {
        RCLCPP_INFO(this->get_logger(), "Pick completed, starting place action");
        publishStatus("pick completed, starting place action");
        
        std::thread([this]() {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          sendPlaceGoal();
        }).detach();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Critical error: Pick completed but no place goal available");
        publishStatus("error: incomplete sequence - missing place goal");
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Pick action aborted: %s", result.result->message.c_str());
      publishStatus("error: pick aborted");
      current_state_ = ExecutionState::IDLE;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Pick action canceled");
      publishStatus("error: pick canceled");
      current_state_ = ExecutionState::IDLE;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Pick action unknown result code");
      publishStatus("error: pick unknown result");
      current_state_ = ExecutionState::IDLE;
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

      // 완료 후 준비자세 복귀 (비동기, 실패해도 플로우 유지)
      publishStatus("returning to ready pose");
      current_state_ = ExecutionState::EXECUTING_READY;
      
      callReadyAsync([this](bool ok){
        if (!ok) {
          RCLCPP_ERROR(this->get_logger(), "Failed to move to ready pose after place");
          publishStatus("error: failed to move to ready after place");
        }
        // Ready 완료 후 IDLE로
        current_state_ = ExecutionState::IDLE;
      });
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Place action aborted: %s", result.result->message.c_str());
      publishStatus("error: place aborted");
      current_state_ = ExecutionState::IDLE;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Place action canceled");
      publishStatus("error: place canceled");
      current_state_ = ExecutionState::IDLE;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Place action unknown result code");
      publishStatus("error: place unknown result");
      current_state_ = ExecutionState::IDLE;
      break;
  }
}

void PickPlaceManagerNode::publishStatus(const std::string & status)
{
  auto status_msg = std_msgs::msg::String();
  status_msg.data = status;
  status_pub_->publish(status_msg);
}

bool PickPlaceManagerNode::callReady()
{
  if (!ready_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Ready service not available");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = ready_client_->async_send_request(request);

  // MultiThreadedExecutor를 사용하므로, std::future 대기만으로도 응답 처리가 가능함
  auto status = future.wait_for(std::chrono::seconds(20));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Ready service call timed out");
    return false;
  }

  auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(this->get_logger(), "Ready service returned failure: %s", response->message.c_str());
  }
  return response->success;
}

void PickPlaceManagerNode::callReadyAsync(std::function<void(bool)> on_done)
{
  if (!ready_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Ready service not available");
    on_done(false);
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  ready_client_->async_send_request(request,
    [this, on_done](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
      try {
        auto response = future.get();
        if (!response->success) {
          RCLCPP_ERROR(this->get_logger(), "Ready service returned failure: %s", response->message.c_str());
        }
        on_done(response->success);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while calling ready service: %s", e.what());
        on_done(false);
      }
    });
}

// 토픽 기반 Pause/Resume 콜백
void PickPlaceManagerNode::cmdCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string cmd = msg->data;
  // 소문자로 정규화
  std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
  
  RCLCPP_INFO(this->get_logger(), "Received command: '%s', current state: %d", 
              cmd.c_str(), static_cast<int>(current_state_));
  
  if (cmd != "pause" && cmd != "resume") {
    RCLCPP_WARN(this->get_logger(), "Unknown command: '%s'. Expected 'pause' or 'resume'", msg->data.c_str());
    return;
  }
  
  // 현재 상태에 따라 적절한 executor에 명령 전달
  switch (current_state_) {
    case ExecutionState::EXECUTING_PICK:
      {
        auto cmd_msg = std::make_shared<std_msgs::msg::String>();
        cmd_msg->data = cmd;
        pick_cmd_pub_->publish(*cmd_msg);
        RCLCPP_INFO(this->get_logger(), "Sent '%s' command to pick executor", cmd.c_str());
      }
      break;
      
    case ExecutionState::EXECUTING_PLACE:
      {
        auto cmd_msg = std::make_shared<std_msgs::msg::String>();
        cmd_msg->data = cmd;
        place_cmd_pub_->publish(*cmd_msg);
        RCLCPP_INFO(this->get_logger(), "Sent '%s' command to place executor", cmd.c_str());
      }
      break;
      
    case ExecutionState::EXECUTING_READY:
      {
        auto cmd_msg = std::make_shared<std_msgs::msg::String>();
        cmd_msg->data = cmd;
        ready_cmd_pub_->publish(*cmd_msg);
        RCLCPP_INFO(this->get_logger(), "Sent '%s' command to ready executor", cmd.c_str());
      }
      break;
      
    case ExecutionState::IDLE:
      RCLCPP_WARN(this->get_logger(), "Command '%s' received but no action is currently executing", cmd.c_str());
      break;
  }
}

}  // namespace ur_pick_and_place

// main 함수 추가
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_pick_and_place::PickPlaceManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
} 