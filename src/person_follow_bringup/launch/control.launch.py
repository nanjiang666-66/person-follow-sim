from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('person_follow_bringup')) / 'config' / 'following.yaml')
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    distance = ParameterValue(LaunchConfiguration('desired_distance'), value_type=float)
    shared = {'use_sim_time': clock, 'desired_distance': distance}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('desired_distance', default_value='1.5'),
        Node(package='opennav_following', executable='opennav_following',
             name='following_server', output='screen', parameters=[config, shared],
             remappings=[('cmd_vel', '/follow/cmd_vel_raw')]),
        Node(package='person_follow_control', executable='follow_supervisor',
             name='follow_supervisor', output='screen', parameters=[config, shared]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_following', output='screen', parameters=[
                 {'use_sim_time': clock, 'autostart': True, 'bond_timeout': 4.0,
                  'node_names': ['following_server']}]),
    ])
