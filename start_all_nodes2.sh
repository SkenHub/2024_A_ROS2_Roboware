#!/bin/bash

# Set up the ROS 2 environment
#colcon build
source /opt/ros/humble/setup.bash
source ~/2024_A_ROS2_Roboware/install/setup.bash

# Start web_socket_node in a new tab
gnome-terminal --tab --title="web_socket_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/2024_A_ROS2_Roboware/install/setup.bash; ros2 run robot_controller web_socket_node; exec bash"

# Start controller_node in a new tab
gnome-terminal --tab --title="controller_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/2024_A_ROS2_Roboware/install/setup.bash; ros2 run robot_controller controller_node; exec bash"

# Start cmd_vel_to_serial_node in a new tab
gnome-terminal --tab --title="serial_send_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/2024_A_ROS2_Roboware/install/setup.bash; ros2 run robot_controller serial_send_node; exec bash"

# Start serial_to_position_node in a new tab
gnome-terminal --tab --title="serial_read_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/2024_A_ROS2_Roboware/install/setup.bash; ros2 run robot_controller serial_read_node; exec bash"

# Start realsense_position_node in a new tab
gnome-terminal --tab --title="position_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/2024_A_ROS2_Roboware/install/setup.bash; ros2 run robot_controller position_node; exec bash"

# Start realsense_position_node in a new tab
gnome-terminal --tab --title="robot_position_to_rviz_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/2024_A_ROS2_Roboware/install/setup.bash; ros2 run robot_controller robot_position_to_rviz_node; exec bash"
