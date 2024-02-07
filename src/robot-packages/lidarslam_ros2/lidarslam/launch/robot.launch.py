from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcl_filter',
            executable='filter_node',
            name='filter_node'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='ps4_controller',
            executable='ps4_controller_node',
            name='ps4_controller_node'
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('octomap_server'),
                    'launch',
                    'octomap_mapping.launch.xml'
                ])
            ])
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('velodyne'),
                    'launch',
                    'velodyne-all-nodes-VLP16-launch.py'
                ])
            ])
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lidarslam'),
                    'launch',
                    'lidarslam.launch.py'
                ])
            ])
        )
    ])
