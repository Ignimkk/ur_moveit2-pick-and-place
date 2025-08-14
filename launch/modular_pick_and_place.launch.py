#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get package directory
    pkg_share = FindPackageShare('ur_pick_and_place')
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Goal Receiver Node
    goal_receiver_node = Node(
        package='ur_pick_and_place',
        executable='goal_receiver_node',
        name='goal_receiver_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Gripper Controller Node
    gripper_controller_node = Node(
        package='ur_pick_and_place',
        executable='gripper_controller_node',
        name='gripper_controller_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Ready Executor Node
    ready_executor_node = Node(
        package='ur_pick_and_place',
        executable='ready_executor_node',
        name='ready_executor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Pick Executor Node
    pick_executor_node = Node(
        package='ur_pick_and_place',
        executable='pick_executor_node',
        name='pick_executor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Place Executor Node
    place_executor_node = Node(
        package='ur_pick_and_place',
        executable='place_executor_node',
        name='place_executor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Pick Place Manager Node
    pick_place_manager_node = Node(
        package='ur_pick_and_place',
        executable='pick_place_manager_node',
        name='pick_place_manager_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        goal_receiver_node,
        gripper_controller_node,
        ready_executor_node,
        pick_executor_node,
        place_executor_node,
        pick_place_manager_node,
    ]) 