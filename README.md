# ENPM808X turtlebot_walker
## Overview
The turtlebot_walker project creates a ROS node that works in conjunction with the turtlebot_gazebo turtlebot_world.launch file to operate a simulated Turtlebot in the Gazebo simulation environment.  The Turtlebot behavior is intended to somewhat approximate the behavior of a Roomba robot: "... move forward until it reaches an obstacle (but not colliding), then rotate in place until the way ahead is clear, then move forward again and repeat.

## Prerequisites
This ROS package has been implemented and tested on ROS Indigo Igloo only.  It was developed on an Ubuntu 14.04 system with the Turtlebot simulation stack installed using the command "$ sudo apt-get install ros-indigo-turtlebot-gazebo ros-indigo-turtlebot-apps ros-indigo-turtlebot-rviz-launchers".  The code for the node is written in C++ and was originally compiled using the Ubuntu 14.04 gcc compiler.

## Technical Notes
The sensor information used by the turtlebot_walker_node is read from the "/scan" topic, and consists of sensor_msgs/LaserScan messages published using the simulated Turtlebot depth camera device published by the /laserscan_nodelet_manager.  Although the sensor subscription code is generalizable and will ultimately be made non-device specific, it is at this time specific to data produced by a simulated asus_xtion_pro.  This device's output, when processed into a simulated LaserScan, produces range information in an approximately 60 degree arc centered on the forward direction of travel for the Turtlebot (positive x-axis of the base_link frame) with 640 individual range readings from each scan.

The turtlebot_walker_node sends velocity commands to the Turtlebot by publishing on the topic /cmd_vel_mux/input/teleop in the form of geometry_msgs/Twist messages (only the linear x velocity and angular z velocity are used, commanding the forward/backward movement of the Turtlebot and its rotation about the vertical z-axis, respectively).

## Importing this package into your ROS catkin workspace
To import the project into your catkin workspace, clone or download it into the src subdirectory of your catkin workspace directory. Once this package is part of your catkin workspace, it will build along with any other packages you have in that workspace using the "catkin_make" command executed at the top-level directory of your catkin workspace.

Be sure to run "catkin_make" in your workspace before trying to run the turtlebot_walker, as the package only includes the source code.

## Running the turtlebot_walker_node
The package includes a launch file (turtlebot_walker.launch) that starts the gazebo environment with a simulated Turtlebot and a world with some obstacles by including the "turtlebot_gazebo turtlebot_world.launch" file.

Prior to trying to start the turtlebot_walker, be sure to have "source"ed the setup.bash file for your catkin workspace into which you placed the turtlebot_walker package.  Then, use this command to start everything up:

    - roslaunch turtlebot_walker turtlebot_walker.launch
    
The Gazebo simulator will start up along with the ros master and support nodes, and the simulated Turtlebot will begin its "walker" behavior in the turtlebot world.

## Data collection and playback
The launch file includes the ability to record all topics published during its operation (except for the voluminous /camera/* topics, which are excluded).  If a recording is made, the bag file will be stored in the directory ~/.ros, unless ROS_HOME is defined, in which case it wil be stored there.  The bag filename is prefixed with "turtlebot_walker" and includes the date and time of the recording.  To record data, use a modified version of the launch command:

    - roslaunch turtlebot_walker turtlebot_walker.launch record:=true
    
Use Ctl-C in the terminal window running roslaunch to terminate the run and data collection.  To inspect the resulting bag file, use the command:

    - rosbag info <bagfile>
    
To playback the contents of the bag file, use the command:

    - rosbag play <bagfile>
    
Be sure to have a ros master running prior to playing back the bag file.  Do not have the turtlebot_gazebo world running during playback, as the topics collected by the bag file will duplicate topics being published by the simulation.