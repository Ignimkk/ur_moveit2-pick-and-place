#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5e',
        description='Type/series of used UR robot'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # Launch configurations
    ur_type = LaunchConfiguration('ur_type')
    launch_rviz = LaunchConfiguration('launch_rviz')
    
    # Table URDF
    table_urdf_path = PathJoinSubstitution([
        FindPackageShare('ur_pick_and_place'), 'urdf', 'work_table.urdf.xacro'
    ])
    table_description = Command(['xacro ', table_urdf_path])
    
    # 1. UR Control with Fake Hardware (원래 ur_description 사용)
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur_robot_driver'), 
            '/launch/ur_control.launch.py'
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': 'xxx.yyy.zzz.www',  # Fake IP (not used)
            'use_fake_hardware': 'true',
            'fake_sensor_commands': 'true',
            'launch_rviz': 'false',
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'activate_joint_controller': 'true',
        }.items()
    )
    
    # 2. MoveIt Configuration (원래 ur_description 사용)
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur_moveit_config'), 
            '/launch/ur_moveit.launch.py'
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'use_sim_time': 'false',
            'launch_rviz': 'false',
            'launch_servo': 'false',  # servo 비활성화 (planning group name 불일치 문제 회피)
        }.items()
    )

    
    # 4. RViz 직접 실행 (MoveIt 설정 사용)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ur_moveit_config'),
        'rviz',
        'view_robot.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(launch_rviz)
    )
    
    # 5. Pick and Place Application Nodes
    # Goal Receiver Node
    goal_receiver_node = Node(
        package='ur_pick_and_place',
        executable='goal_receiver_node',
        name='goal_receiver_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Gripper Controller Node
    gripper_controller_node = Node(
        package='ur_pick_and_place',
        executable='gripper_controller_node',
        name='gripper_controller_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Ready Executor Node
    ready_executor_node = Node(
        package='ur_pick_and_place',
        executable='ready_executor_node',
        name='ready_executor_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Pick Executor Node
    pick_executor_node = Node(
        package='ur_pick_and_place',
        executable='pick_executor_node',
        name='pick_executor_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Place Executor Node
    place_executor_node = Node(
        package='ur_pick_and_place',
        executable='place_executor_node',
        name='place_executor_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Pick Place Manager Node
    pick_place_manager_node = Node(
        package='ur_pick_and_place',
        executable='pick_place_manager_node',
        name='pick_place_manager_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        # Launch arguments
        ur_type_arg,
        launch_rviz_arg,
        
        # Robot control and planning
        ur_control_launch,
        ur_moveit_launch,
        
        
        # Visualization
        rviz_node,
        
        # Pick and place application
        goal_receiver_node,
        gripper_controller_node,
        ready_executor_node,
        pick_executor_node,
        place_executor_node,
        pick_place_manager_node,
    ])

