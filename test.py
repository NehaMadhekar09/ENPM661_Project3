#! /usr/bin/env python3
import rospy
import rospkg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from collections import deque
import time
import math
rospack = rospkg.RosPack()
folder_path = rospack.get_path('project3')
file_path = folder_path+"/scripts/Waypoints.txt"

roll = pitch = yaw = 0.0

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)


def ComputeDistance(node_xy,goal_node_xytheta):
    return np.sqrt((goal_node_xytheta[0]-node_xy[0])*(goal_node_xytheta[0]-node_xy[0])+(goal_node_xytheta[1]-node_xy[1])*(goal_node_xytheta[1]-node_xy[1]))

def go_to_point(curr_x,curr_y,target_x,target_y):

    rospy.init_node('test')
    total_distance=ComputeDistance((curr_x,curr_y),(target_x,target_y))
    if(target_x-curr_x==0):
        if target_y-curr_y < 0:
            target_angle=-1.57
        else:
            target_angle=1.57
    else:
        target_angle=math.atan2((target_y-curr_y),(target_x-curr_x))
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
    
    vel_msg = Twist()

    while abs(yaw-target_angle) > 0.05:
        # print(abs(yaw-target_angle))
        if(target_angle-yaw < 0):
            omega=-0.1
        else:
            omega=0.1
        vel_msg.angular.z=omega
        pub.publish(vel_msg)
    vel_msg.angular.z = 0.0
    pub.publish(vel_msg)

    start_time = time.time()
    distance=0
    while distance < total_distance:
        vel=0.15
        vel_msg.linear.x=vel
        pub.publish(vel_msg)
        curr_time = time.time()
        distance=vel*(curr_time-start_time)
    vel_msg.linear.x = 0.0
    pub.publish(vel_msg)
import os
import time
def main():
    time.sleep(5.0)
    print("Path", os.getcwd())
    # file_path = os.getcwd() + "/src/project3/scripts/Waypoints10.txt"
    points=[]
    # # Reading the file and storing the trajectory in a list
    with open(file_path, "r") as f:
        for line in f:
            print(line)
            left_vel, right_vel = line.strip().split(" ")
            left_vel = float(left_vel)
            right_vel = float(right_vel)
            points.append([left_vel, right_vel])

# # passing the velocity for left and right to cal the robots velocity
    # points=[(0,0),(1.25,-0.7),(1.5,-0.5),(1.5,0.7),(5,0.7),(5,0)]
    i=1
    while not rospy.is_shutdown():
        if i==len(points):
            exit()
        go_to_point(points[i-1][0],points[i-1][1],points[i][0],points[i][1])
        i=i+1
    

if __name__ == '__main__':
    main()

