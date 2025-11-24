cmake_minimum_required(VERSION 3.10)
project(ur_pick_and_place)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(action_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Generate action and service interfaces first
set(action_files
  action/PickAndPlace.action
  action/Pick.action
  action/Place.action
)

set(service_files
  srv/GripperControl.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${service_files}
  DEPENDENCIES
    action_msgs
    geometry_msgs
    std_msgs
)

# Create executables
add_executable(ur_pick_and_place_moveit src/ur_pick_and_place_moveit.cpp)

# Dependencies for ur_pick_and_place_moveit
ament_target_dependencies(ur_pick_and_place_moveit
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  Eigen3
)

# Link action interfaces to the executable that uses them
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(ur_pick_and_place_moveit "${cpp_typesupport_target}")

# Create individual executables for modular nodes
add_executable(goal_receiver_node 
  src/goal_receiver_node.cpp)

add_executable(gripper_controller_node 
  src/gripper_controller_node.cpp)

add_executable(pick_executor_node 
  src/pick_executor_node.cpp)

add_executable(place_executor_node 
  src/place_executor_node.cpp)

add_executable(pick_place_manager_node 
  src/pick_place_manager_node.cpp)

add_executable(ready_executor_node
  src/ready_executor_node.cpp)

# Dependencies for modular node executables
ament_target_dependencies(goal_receiver_node
  rclcpp
  rclcpp_action
  geometry_msgs
)

ament_target_dependencies(gripper_controller_node
  rclcpp
  rclcpp_action
  std_msgs
)

ament_target_dependencies(pick_executor_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  Eigen3
  std_msgs
)

ament_target_dependencies(place_executor_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  Eigen3
  std_msgs
)

ament_target_dependencies(pick_place_manager_node
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  std_srvs
)

ament_target_dependencies(ready_executor_node
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  Eigen3
  std_srvs
  std_msgs
)

# Set include directories for modular node executables
target_include_directories(goal_receiver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(gripper_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(pick_executor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(place_executor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(pick_place_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ready_executor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link interfaces to modular node executables
target_link_libraries(goal_receiver_node "${cpp_typesupport_target}")
target_link_libraries(gripper_controller_node "${cpp_typesupport_target}")
target_link_libraries(pick_executor_node "${cpp_typesupport_target}")
target_link_libraries(place_executor_node "${cpp_typesupport_target}")
target_link_libraries(pick_place_manager_node "${cpp_typesupport_target}")

# Install targets
install(TARGETS 
  ur_pick_and_place_moveit
  goal_receiver_node
  gripper_controller_node
  pick_executor_node
  place_executor_node
  pick_place_manager_node
  ready_executor_node
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install action and service files
install(DIRECTORY
  action
  srv
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/test_modular_system.py
  scripts/real_robot_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
