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
            package='hololens_downsampler',
            executable='holo_downsampler_node',
            name='holo_downsampler_node'
        ),
        Node(
            package='pcl_downsampler',
            executable='downsampler_node',
            name='downsampler_node'
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
                    FindPackageShare('ros_tcp_endpoint'),
                    'launch',
                    'endpoint.py'
                ])
            ])
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('octomap_merger'),
                    'launch',
                    'merger.launch.py'
                ])
            ])
        )
    ])
