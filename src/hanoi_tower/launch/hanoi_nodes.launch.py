import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hanoi_pkg = get_package_share_directory('hanoi_tower')
    config_file = os.path.join(hanoi_pkg, 'config', 'hanoi_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    vision_node = Node(
        package='hanoi_tower',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
    )

    solver_node = Node(
        package='hanoi_tower',
        executable='hanoi_solver',
        name='hanoi_solver',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
    )
    motion_executor_node = Node(
        package='hanoi_tower',
        executable='motion_executor',
        name='motion_executor',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        vision_node,
        solver_node,
        motion_executor_node,
    ])