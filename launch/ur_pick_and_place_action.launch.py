#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Action server node
    action_server_node = Node(
        package='ur_pick_and_place',
        executable='ur_pick_and_place_action_server',
        name='pick_and_place_action_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Action client node (delayed start)
    action_client_node = Node(
        package='ur_pick_and_place',
        executable='ur_pick_and_place_action_client',
        name='pick_and_place_action_client',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Delay client start by 5 seconds to ensure server is ready
    delayed_client = TimerAction(
        period=5.0,
        actions=[action_client_node]
    )

    return LaunchDescription([
        declare_use_sim_time,
        action_server_node,
        delayed_client
    ]) 