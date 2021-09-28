#!/usr/bin/env python3
# coding: utf-8

# **Script name** : Local Planner
# 
# **Description** : Multi-agent payload transport global planner based on APF.
# 
# **Author**      : Swapnil Kalhapure
# 
# **Email**       : kalhapure.swapnil@gmail.com

# ## Imports

import rospy
import cv2
import time
import rospkg
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist,PoseStamped,TransformStamped
from scipy.ndimage.morphology import distance_transform_edt as bwdist
from scipy.ndimage.morphology import grey_dilation
from scipy.spatial import distance
from geometry_msgs.msg import PoseStamped
from tracking_pid.msg import FollowPathActionResult
from tracking_pid.msg import traj_point

current_odom = None
trajectory_pub = None
odom_pub = None
offset = 0.0

def odom_callback(data):
    global current_odom
    global offset
    global odom_pub
    global trajectory_pub
    current_odom = data
    t_a_b = TransformStamped()
    t_a_b.header.stamp = rospy.Time.now()
    t_a_b.transform.translation.x = current_odom.pose.pose.position.x
    t_a_b.transform.translation.y = current_odom.pose.pose.position.y
    t_a_b.transform.translation.z = current_odom.pose.pose.position.z
    t_a_b.transform.rotation.x = current_odom.pose.pose.orientation.x
    t_a_b.transform.rotation.y = current_odom.pose.pose.orientation.y
    t_a_b.transform.rotation.z = current_odom.pose.pose.orientation.z
    t_a_b.transform.rotation.w = current_odom.pose.pose.orientation.w
    ttr = PoseStamped()
    ttr.header.stamp=rospy.Time.now()
    ttr.pose.position.y= offset
    ttr.pose.orientation.w = 1.0
    temp = tf2_geometry_msgs.do_transform_pose(ttr, t_a_b)
    agent_current_odom = Odometry()
    agent_current_odom.header.frame_id = "map"
    agent_current_odom.header.stamp = rospy.Time.now()
    agent_current_odom.pose.pose.position = temp.pose.position
    agent_current_odom.pose.pose.orientation = temp.pose.orientation
    odom_pub.publish(agent_current_odom)
    cmd = traj_point()
    cmd.pose.header.frame_id = "map"
    cmd.pose.pose.position = agent_current_odom.pose.pose.position
    cmd.pose.pose.orientation = agent_current_odom.pose.pose.orientation
    trajectory_pub.publish(cmd)


rospy.init_node('LocaltrackerPlanner')
rospy.Subscriber("/jackal_gazebo_odom", Odometry, odom_callback)
odom_pub = rospy.Publisher("/odom_test", Odometry, queue_size=1)
trajectory_pub = rospy.Publisher("/trajectory", traj_point, queue_size=1)
offset = rospy.get_param("~offset")
rospy.spin()