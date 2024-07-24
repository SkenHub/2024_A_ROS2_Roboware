from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller',
            executable='web_socket_node',
            name='web_socket_node',
            output='screen'
        ),
        Node(
            package='robot_controller',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
        Node(
            package='robot_controller',
            executable='cmd_vel_to_serial_node',
            name='cmd_vel_to_serial_node',
            output='screen'
        ),
        Node(
            package='robot_controller',
            executable='serial_to_position_node',
            name='serial_to_position_node',
            output='screen'
        ),
        Node(
            package='robot_controller',
            executable='realsense_position_node',
            name='realsense_position_node',
            output='screen'
        ),
    ])
