ROS 2 Safe Teleoperation (Assignment 2)

Author: (Amri Abdelouafi)
Student ID: (7708121)
ROS 2 Distro: Jazzy
Date: (01-02-26)

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

1. Project Overview

This project implements a Safe Teleoperation System for a mobile robot in simulation using Gazebo.

The user sends raw motion commands via /cmd_vel_raw. A dedicated safety node processes LaserScan data from /scan, evaluates potential collision risks, and determines what command is safe to forward to the robot via /cmd_vel.

Objectives
	•	Enable manual robot control.
	•	Prevent collisions:
	•	If an obstacle is detected within a safety threshold, forward motion is blocked.
	•	Rotational motion remains allowed to help the robot escape from obstacles.

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

2. System Components

2.1 Packages

assignment2_rt
Custom interfaces (messages and services):
	•	msg/ObstacleInfo.msg
	•	srv/SetThreshold.srv
	•	srv/GetAvgVel.srv

safe_teleop
Core control and safety nodes:
	•	teleop_node
Publishes user commands to /cmd_vel_raw.
	•	safety_node
	•	Subscribes to /scan and /cmd_vel_raw.
	•	Applies safety logic.
	•	Publishes safe commands to /cmd_vel.
	•	Publishes obstacle information to /obstacle_info.

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

3. Topics

3.1 Published Topics
	•	/cmd_vel_raw — geometry_msgs/Twist
Raw user velocity commands (published by teleop_node).
	•	/cmd_vel — geometry_msgs/Twist
Final safe velocity command (published by safety_node).
	•	/obstacle_info — assignment2_rt/msg/ObstacleInfo
Summary of the closest detected obstacle:
	•	distance (meters)
	•	direction (front / left / right)
	•	threshold (meters)

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

3.2 Subscribed Topics
	•	/scan — sensor_msgs/LaserScan
Laser data from simulation.
	•	/cmd_vel_raw — geometry_msgs/Twist
Raw velocity commands from user input.

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

4. Services
	•	/set_threshold — assignment2_rt/srv/SetThreshold
Dynamically changes the safety distance during runtime.
	•	/get_last_velocity — assignment2_rt/srv/GetAvgVel
Returns the average of the last five received raw velocity commands.

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

5. Prerequisites
	•	ROS 2 Jazzy
	•	Gazebo simulator
	•	ROS–Gazebo bridge (provided in the course simulation environment)

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

6. Installation and Build

6.1 Create a Workspace (if not already created)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

6.2 Clone the Project
Option 1:git clone https://github.com/wafiadrar20-design/Amri-ouafi
Option 2:git clone https://github.com/CarmineD8/bme_gazebo_sensors.git

6.3 Build the Workspace
cd ~/ros2_ws
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --merge-install
source install/setup.bash

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

7. Running the System

Typically, three terminals are required.

Terminal 1 – Launch Simulation and Robot

Example using bme_gazebo_sensors:
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch bme_gazebo_sensors spawn_robot.launch.py

Terminal 2 – Run Safety Node
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run safe_teleop safety_node

Terminal 3 – Run Teleoperation Node (Keyboard Control)

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run safe_teleop teleop_node

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

8. Keyboard Controls (teleop_node)

Focus on the terminal running teleop_node, then use:
	•	w → Move forward
	•	s → Move backward
	•	a → Turn left
	•	d → Turn right
	•	space → Stop
	•	q → Quit

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

9. Changing the Safety Threshold

Example: Set safety distance to 0.8 meters
source ~/ros2_ws/install/setup.bash
ros2 service call /set_threshold assignment2_rt/srv/SetThreshold "{threshold: 0.8}"
