#!/bin/bash

# Set up the ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_robocon_ws/2024_A_ROS2_Roboware/install/setup.bash

# Start web_socket_node in a new tab
gnome-terminal --tab --title="web_socket_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/ros2_robocon_ws/2024_A_ROS2_Roboware/install/setup.bash; ros2 run robot_controller web_socket_node; exec bash"

# Start controller_node in a new tab
gnome-terminal --tab --title="controller_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/ros2_robocon_ws/2024_A_ROS2_Roboware/install/setup.bash; ros2 run robot_controller controller_node; exec bash"

# Start serial_read_node in a new tab
gnome-terminal --tab --title="serial_read_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/ros2_robocon_ws/2024_A_ROS2_Roboware/install/setup.bash; ros2 run robot_controller serial_read_node; exec bash"

# Start serial_send_node in a new tab
gnome-terminal --tab --title="serial_send_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/ros2_robocon_ws/2024_A_ROS2_Roboware/install/setup.bash; ros2 run robot_controller serial_send_node; exec bash"

# Start position_node in a new tab
gnome-terminal --tab --title="position_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/ros2_robocon_ws/2024_A_ROS2_Roboware/install/setup.bash; ros2 run robot_controller position_node; exec bash"
