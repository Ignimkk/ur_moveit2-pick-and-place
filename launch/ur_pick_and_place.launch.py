from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pick_place_node = Node(
        package='ur_pick_and_place',
        executable='ur_pick_and_place_moveit',
        name='ur_pick_and_place_moveit',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    return LaunchDescription([
        pick_place_node
    ]) 