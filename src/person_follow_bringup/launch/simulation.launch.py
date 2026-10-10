from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = Path(get_package_share_directory('person_follow_description'))
    bringup = Path(get_package_share_directory('person_follow_bringup'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    robot = ParameterValue(Command(['xacro ', str(description / 'urdf' / 'follow_bot.urdf.xacro')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='demo'),
        DeclareLaunchArgument('desired_distance', default_value='1.5'),
        DeclareLaunchArgument('log_path', default_value=''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo / 'launch' / 'gazebo.launch.py')),
                                 launch_arguments={'world': str(description / 'worlds' / 'empty_follow.world'),
                                                   'gui': LaunchConfiguration('gui'), 'verbose': 'false',
                                                   'params_file': str(bringup / 'config' / 'gazebo.yaml')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot, 'use_sim_time': True}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'follow_bot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.02']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup / 'launch' / 'control.launch.py')),
                                 launch_arguments={'use_sim_time': 'true',
                                                   'desired_distance': LaunchConfiguration('desired_distance')}.items()),
        Node(package='person_follow_sim', executable='target_simulator', name='simulated_person', output='screen',
             parameters=[str(bringup / 'config' / 'following.yaml'),
                         {'use_sim_time': True, 'scenario': LaunchConfiguration('scenario')}]),
        Node(package='person_follow_sim', executable='follow_monitor', output='screen',
             parameters=[{'use_sim_time': True, 'log_path': LaunchConfiguration('log_path')}]),
        Node(package='rviz2', executable='rviz2', output='screen', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(description / 'rviz' / 'follow.rviz')], parameters=[{'use_sim_time': True}]),
    ])
