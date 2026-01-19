import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    hanoi_pkg = get_package_share_directory('hanoi_tower')
    models_path = os.path.join(hanoi_pkg, 'models')
    config_file = os.path.join(hanoi_pkg, 'config', 'hanoi_params.yaml')
    
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    
    gazebo_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    linkattacher_lib_path = os.path.expanduser('~/ros2_ws/install/ros2_linkattacher/lib')
    
    xarm_moveit_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hanoi_pkg, 'launch', 'xarm_hanoi_init.launch.py')
        ),
        launch_arguments={
            'dof': '6',
            'robot_type': 'xarm',
            'add_gripper': 'true',
            'add_vacuum_gripper': 'false',
            'no_gui_ctrl': 'false',
            'show_rviz': 'true',
            'rviz_config_file': os.path.join(hanoi_pkg, 'rviz', 'hanoi.rviz'),
            'attach_to': 'world',
            'attach_xyz': '"-0.2 -0.5 1.021"',
            'attach_rpy': '"0 0 1.571"',
        }.items(),
    )
    
    xarm_planner = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('xarm_planner'),
                        'launch',
                        '_robot_planner.launch.py'
                    ])
                ),
                launch_arguments={
                    'dof': '6',
                    'robot_type': 'xarm',
                    'add_gripper': 'true',
                }.items(),
            )
        ]
    )
    
    bin_y = -0.65
    
    spawn_bin_a = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', 'bin_A',
                     '-file', os.path.join(models_path, 'bin', 'model.sdf'),
                     '-x', '0.05', '-y', str(bin_y), '-z', '1.015'],
                output='screen'
            )
        ]
    )
    
    spawn_bin_b = TimerAction(
        period=10.5,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', 'bin_B',
                     '-file', os.path.join(models_path, 'bin', 'model.sdf'),
                     '-x', '0.20', '-y', str(bin_y), '-z', '1.015'],
                output='screen'
            )
        ]
    )
    
    spawn_bin_c = TimerAction(
        period=11.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', 'bin_C',
                     '-file', os.path.join(models_path, 'bin', 'model.sdf'),
                     '-x', '0.35', '-y', str(bin_y), '-z', '1.015'],
                output='screen'
            )
        ]
    )
    
    cube_x = 0.05
    
    spawn_cube_large = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', 'cube_large',
                     '-file', os.path.join(models_path, 'cube_large', 'model.sdf'),
                     '-x', str(cube_x), '-y', str(bin_y), '-z', '1.059'],


                output='screen'
            )
        ]
    )
    
    spawn_cube_medium = TimerAction(
        period=22.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', 'cube_medium',
                     '-file', os.path.join(models_path, 'cube_medium', 'model.sdf'),
                     '-x', str(cube_x), '-y', str(bin_y), '-z', '1.118'],


                output='screen'
            )
        ]
    )
    
    spawn_cube_small = TimerAction(
        period=32.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', 'cube_small',
                     '-file', os.path.join(models_path, 'cube_small', 'model.sdf'),
                     '-x', str(cube_x), '-y', str(bin_y), '-z', '1.167'],


                output='screen'
            )
        ]
    )
    
    spawn_camera = TimerAction(
        period=11.5,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', 'external_camera',
                     '-file', os.path.join(models_path, 'camera', 'model.sdf'),
                     '-x', '0.20', '-y', '-1.15', '-z', '1.015',
                     '-Y', '-1.5708'],
                output='screen'
            )
        ]
    )

    vision_node = TimerAction(
        period=42.0,
        actions=[
            Node(
                package='hanoi_tower',
                executable='vision_node',
                name='vision_node',
                parameters=[config_file, {'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    solver_node = TimerAction(
        period=43.0,
        actions=[
            Node(
                package='hanoi_tower',
                executable='hanoi_solver',
                name='hanoi_solver',
                parameters=[config_file, {'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    motion_executor_node = TimerAction(
        period=43.0,
        actions=[
            Node(
                package='hanoi_tower',
                executable='motion_executor',
                name='motion_executor',
                parameters=[config_file, {'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    camera_urdf_file = os.path.join(hanoi_pkg, 'urdf', 'camera.urdf')
    with open(camera_urdf_file, 'r') as f:
        camera_desc = f.read()

    camera_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_world_tf_publisher',
        arguments=['--x', '0.20', '--y', '-1.15', '--z', '1.015', '--roll', '0', '--pitch', '0', '--yaw', '-1.5708', '--frame-id', 'world', '--child-frame-id', 'camera_base_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    camera_base_to_body_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_to_body_tf_publisher',
        arguments=['--x', '-0.16', '--y', '0', '--z', '0.23', '--roll', '0', '--pitch', '0.45', '--yaw', '3.14159', '--frame-id', 'camera_base_link', '--child-frame-id', 'camera_body_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    camera_body_to_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_body_to_optical_tf_publisher',
        arguments=['--x', '0.0125', '--y', '0', '--z', '0', '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708', '--frame-id', 'camera_body_link', '--child-frame-id', 'camera_color_optical_frame'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', f'{gazebo_model_path}:{models_path}'),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', f'{gazebo_plugin_path}:{linkattacher_lib_path}'),
        xarm_moveit_gazebo,
        xarm_planner,    
        spawn_bin_a,
        spawn_bin_b,
        spawn_bin_c,
        spawn_cube_large,
        spawn_cube_medium,
        spawn_cube_small,
        spawn_camera,
        camera_world_tf,
        camera_base_to_body_tf,
        camera_body_to_optical_tf,
        vision_node,
        solver_node,
        motion_executor_node,
    ])
