#include "ur_pick_and_place/place_executor_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_pick_and_place::PlaceExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 