from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('person_follow_bringup')) / 'config' / 'following.yaml')
    distance = ParameterValue(LaunchConfiguration('desired_distance'), value_type=float)
    optical = LaunchConfiguration('camera_optical_frame')
    shared = {'use_sim_time': False, 'desired_distance': distance,
              'base_frame': 'follow_preview_base', 'fixed_frame': 'follow_preview_odom'}
    return LaunchDescription([
        DeclareLaunchArgument('desired_distance', default_value='1.5'),
        DeclareLaunchArgument('camera_root_frame', default_value='camera_link'),
        DeclareLaunchArgument('camera_optical_frame', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('camera_x', default_value='0.0'),
        DeclareLaunchArgument('camera_height', default_value='1.0'),
        DeclareLaunchArgument('log_path', default_value=''),
        Node(package='person_follow_sim', executable='vision_preview_fixture',
             name='vision_preview_fixture', output='screen', parameters=[{
                 'use_sim_time': False, 'camera_root_frame': LaunchConfiguration('camera_root_frame'),
                 'camera_x': ParameterValue(LaunchConfiguration('camera_x'), value_type=float),
                 'camera_height': ParameterValue(LaunchConfiguration('camera_height'), value_type=float),
                 'log_path': LaunchConfiguration('log_path')}]),
        Node(package='opennav_following', executable='opennav_following',
             name='following_server', output='screen', parameters=[config, shared,
                 {'odom_topic': '/follow_preview/odom'}],
             remappings=[('cmd_vel', '/follow/cmd_vel_raw')]),
        Node(package='person_follow_control', executable='follow_supervisor',
             name='follow_supervisor', output='screen', parameters=[config, shared,
                 {'camera_frame': optical}],
             remappings=[('/odom', '/follow_preview/odom'), ('/cmd_vel', '/follow_preview/cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_following', output='screen', parameters=[{
                 'use_sim_time': False, 'autostart': True, 'bond_timeout': 4.0,
                 'node_names': ['following_server']}]),
    ])
