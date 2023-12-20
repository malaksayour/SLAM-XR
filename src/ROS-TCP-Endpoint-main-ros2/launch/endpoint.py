from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ros_ip_arg = DeclareLaunchArgument(
        'ROS_IP', default_value="192.168.1.110"
    )
    ros_port_arg = DeclareLaunchArgument(
        'ROS_TCP_PORT', default_value="10000"
    )
            
    return LaunchDescription(
        [
            ros_ip_arg,
            ros_port_arg,
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                emulate_tty=True,
                parameters=[{'ROS_IP':LaunchConfiguration('ROS_IP')}
                            , {"ROS_TCP_PORT": LaunchConfiguration('ROS_TCP_PORT')}],
            )
        ]
    )
