import os
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():
    config=PathJoinSubstitution([
                FindPackageShare('octomap_merger'),
                'config',
                'merger.yaml'
                ])
    # config = os.path.join(
    #   get_package_share_directory('octomap_merger'),
    #   'config',
    #   'merger.yaml'
    #   )
    return LaunchDescription([
        Node(
            package='octomap_merger',
            executable='merger_node',
            name='merger_node',
            parameters=[config]
        )
    ])

