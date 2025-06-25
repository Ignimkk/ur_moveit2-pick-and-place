#include "ur_pick_and_place/pick_place_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_pick_and_place::PickPlaceManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 