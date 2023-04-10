===========================================================================================================
Part1
===========================================================================================================
proj3p2_part1_Neha_Aman.py script in folder Part1 generates the path from given start node to the given goal node using A* Search algorithm.

﻿It has following dependencies:
1. numpy
2. collections module
3. math
4. pygame
5. time

Steps to run ﻿proj3p2_part1_Neha_Aman.py:
1. Run the script. It will prompt the user to enter the clearance value.The obstacles in the map will be created based on this value.
2. The script will then ask the user to enter start and goal node coordinates and start orientation.It will also ask to enter 2 values of RPM.Those will be used for action set.
3. The start and goal nodes are w.r.t the gazebo map origin frame and they are in centimeters.
4. The code will search for the optimal path and display the explored and optimal paths in a map.

Sample Input:
Clearance=5
start_x=0
start_y=0
start_theta=0
goal_x=500
goal_y=0
RPM1=3
RPM2=6

It will also generate a text file containing x and y coordinates of waypoints for Gazebo visualization.
===========================================================================================================
Part 2
===========================================================================================================
Place the package in catkin_ws which has turtlebot3 packages.

It has a publisher test.py in scripts folder.
Publisher test.py has following dependencies:
rospy
rospkg
time
math

It also has a launch file planning.launch in launch folder.
Command to run the simulation after building the package:
roslaunch project3 planning.launch 

This launch file will read the text file "Waypoints.txt" in the script folder and move the turtlebot in map.world as per those points.

Simulation is run on following parameters(in meters):
start_x=0, start_y=0, start_theta=0
goal_x=5, goal_y=0
Clearance = 0.010
RPM1=5
RPM2=10


===========================================================================================================

Team Members:
1. Name: Neha Nitin Madhekar
   DirectoryId: nehanm97
   UID: 119374436
   
1. Name: Aman Sharma
   DirectoryId: ashrm007
   UID: 119085431

# Github link of the code repository:
# https://github.com/NehaMadhekar09/ENPM661_Project3


# Google drive link of the video:
2D Visualization(Pygame):
https://drive.google.com/file/d/1HCkgCW1e1vBR555Tr-T0mXHKQPQuy8oF/view?usp=sharing

3D Visualization(Gazebo):
https://drive.google.com/file/d/1o6zxUk3u5m6G61C_menk4qYYDSDdRSJ9/view?usp=sharing

