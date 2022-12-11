#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import threading
from geometry_msgs.msg import PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf


def goal_pose(point):
    goal_pose=MoveBaseGoal()
    goal_pose.target_pose.header.frame_id="map"
    goal_pose.target_pose.pose.position.x=point[0]
    goal_pose.target_pose.pose.position.y=point[1]
    goal_pose.target_pose.pose.position.z=0
 
    # r, p, y  欧拉角转四元数
    x,y,z,w=tf.transformations.quaternion_from_euler(0,0,0)
 
    goal_pose.target_pose.pose.orientation.x=x
    goal_pose.target_pose.pose.orientation.y=y
    goal_pose.target_pose.pose.orientation.z=z
    goal_pose.target_pose.pose.orientation.w=w
    return goal_pose


if __name__ == '__main__':
    rospy.init_node('image_process', anonymous=False)
    client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    while not rospy.is_shutdown():
        goal=goal_pose([5, 5])
        client.send_goal(goal)
        client.wait_for_result()
