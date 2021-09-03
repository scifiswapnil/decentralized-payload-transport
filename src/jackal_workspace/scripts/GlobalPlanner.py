#!/usr/bin/env python3
# coding: utf-8

# **Script name** : Global Planner
# 
# **Description** : Multi-agent payload transport global planner based on RRT.
# 
# **Author**      : Swapnil Kalhapure
# 
# **Email**       : kalhapure.swapnil@gmail.com

# ## Imports

import cv2 
import math
import time
import rospy
import tf2_ros
import numpy as np
from PIL import Image
import tf2_geometry_msgs  
import random
from matplotlib import path
from nav_msgs.msg import Path
from scipy import interpolate
from numpy.linalg import norm
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from scipy.spatial import ConvexHull
import matplotlib.patches as patches
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from matplotlib.patches import Polygon
from geometry_msgs.msg import PoseStamped
from scipy.ndimage.morphology import distance_transform_edt as bwdist
from scipy.ndimage.morphology import grey_dilation


# ## Global variables

res_remap = 0.04

map_x = 0.0
map_y = 0.0
map_org_x = 0.0
map_org_y = 0.0
map_res = 0.0

mapdata = None
orgmapdata = None

current_xy = None
path_pub = None


# ## Helper Functions

def meters2grid(pose_m):
    pose_on_grid = np.array((np.array(pose_m) - [map_org_x, map_org_y])/ map_res)
    pose_on_grid[1] = map_y - pose_on_grid[1] 
    return pose_on_grid


def grid2meters(pose_grid):
    x = pose_grid[0] * (map_res) + map_org_x #+ map_res /2
    y = (map_y - pose_grid[1]) * (map_res) + map_org_y #+ map_res /2
    a = []
    a.append(float(x))
    a.append(float(y))
    return a


def get_line(x1, y1, x2, y2):
    x1 = int(x1)
    y1 = int(y1)
    x2 = int(x2)
    y2 = int(y2)
    points = []
    issteep = abs(y2-y1) > abs(x2-x1)
    if issteep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    rev = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        rev = True
    deltax = x2 - x1
    deltay = abs(y2-y1)
    error = int(deltax / 2)
    y = y1
    ystep = None
    if y1 < y2:
        ystep = 1
    else:
        ystep = -1
    for x in range(x1, x2 + 1):
        if issteep:
            points.append((y, x))
        else:
            points.append((x, y))
        error -= deltay
        if error < 0:
            y += ystep
            error += deltax
    # Reverse the list if the coordinates were reversed
    if rev:
        points.reverse()
    return points


def path_smoothing(path):
    current_point = 0 
    mapd = mapdata
    smoothedpath = []
    smoothedpath.append([path[current_point][0],path[current_point][1]])
    while (current_point <= len(path)):
        org_current_point = current_point
        for j in range(current_point,len(path)):
            point_lists = get_line(path[current_point][0],path[current_point][1],path[j][0],path[j][1])
            a = 0
            for i in range(len(point_lists)):
                if (mapd[point_lists[i][1],point_lists[i][0]] <= 0.0):
                    a = 1
                    current_point = j 
                    smoothedpath.append([path[j][0],path[j][1]])
                    break
            if (a == 1):
                break
        if (org_current_point == current_point):
            break
    smoothedpath.append([path[-1][0],path[-1][1]])
    return smoothedpath


### RRT path planner

class RRT:
    """
    Class for RRT planning
    """
    class Node:
        """
        RRT Node
        """
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
    def __init__(self,
                 start,
                 goal,
                 map_grid,
                 rand_area,
                 expand_dis=100,
                 path_resolution=10,
                 goal_sample_rate=5,
                 max_iter=5900):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate#
        self.grid_map = map_grid
        self.max_iter = max_iter
        self.node_list = []

    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node,self.grid_map):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(final_node,self.grid_map):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 10) > self.goal_sample_rate:
            rnd = self.Node(
                int(random.uniform(self.min_rand, self.max_rand)),
                int(random.uniform(self.min_rand, self.max_rand)))
        else: 
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, drawthis=None, rnd=None):
        plt.figure(figsize=(10,10))
        plt.clf()
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        plt.imshow(self.grid_map)
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.grid(True)
        plt.pause(0.0001)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_collision(node,grid_map):
        if node is None:
            return False
        if (grid_map[int(node.y),int(node.x)] != 0.0):
            return True
        else:
            return False

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


### ROS Code
def map_callback(data):
    global mapdata
    global orgmapdata
    global map_x
    global map_y
    global map_org_x
    global map_org_y
    global map_res
    global res_remap
    
    map_x = data.info.width
    map_y = data.info.height
    map_org_x = data.info.origin.position.x
    map_org_y = data.info.origin.position.y
    map_res = data.info.resolution
    orgmapdata = np.asarray(data.data).reshape(data.info.height,data.info.width).astype(np.uint8)
    orgmapdata = np.flip(orgmapdata,0)
    mapdata = orgmapdata
    mapdata = np.where(mapdata<254,mapdata,0)
    mapdata = grey_dilation(mapdata,size=(int(res_remap*map_y),int(res_remap*map_x)))
    mapdata = np.invert(mapdata)
    mapdata = cv2.threshold(mapdata, 200, 255, cv2.THRESH_BINARY)[1]

#     plt.figure(figsize=(10,10))
#     plt.grid()
#     plt.imshow(mapdata)
#     plt.xlabel('X, m')
#     plt.ylabel('Y, m')
#     plt.show() 

def callbackodom(data):
    global current_xy
    current_xy = data
    
def callbackgoal(data):
    global xy_start
    global path_pub
    global orgpath
    
    tfbuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfbuffer)
    
    t_a_b = tfbuffer.lookup_transform("map",current_xy.header.frame_id, rospy.Time(0), rospy.Duration(10.0))
    start1 = tf2_geometry_msgs.do_transform_pose(current_xy.pose, t_a_b)
    
    t_a_c = tfbuffer.lookup_transform("map",data.header.frame_id, rospy.Time(0), rospy.Duration(10.0))
    goal1 = tf2_geometry_msgs.do_transform_pose(data, t_a_c)
    
    start = meters2grid((start1.pose.position.x, start1.pose.position.y))
    goal = meters2grid((goal1.pose.position.x, goal1.pose.position.y))
    
    rrt = RRT(
        start=[start[0],start[1]],
        goal=[goal[0],goal[1]],
        rand_area=[50, 1200],
        map_grid=mapdata,
        expand_dis=20,
        path_resolution=5,
        goal_sample_rate=1,
        max_iter=15000)
    path = rrt.planning(animation=False)
#     rrt.draw_graph(orgmapdata+mapdata)
    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        orgpath = path
        decomp_X = []
        decomp_Y = []
        Sdecomp_X = []
        Sdecomp_Y = []
        smoothpath = path_smoothing(orgpath)
        for i in range(len(orgpath)):
            decomp_X.append(orgpath[i][0])
            decomp_Y.append(orgpath[i][1])
        for j in range(len(smoothpath)):
            Sdecomp_X.append(smoothpath[j][0])
            Sdecomp_Y.append(smoothpath[j][1])
        if (len(Sdecomp_X) <= 2):
            f, u = interpolate.splprep([Sdecomp_X,Sdecomp_Y],s=0.05,k=1)
        else :
            f, u = interpolate.splprep([Sdecomp_X,Sdecomp_Y],s=0.05,k=2)
        xsmooth, ysmooth = interpolate.splev(np.linspace(0, 1.0, len(Sdecomp_X)*5), f)
        ros_path = np.vstack((xsmooth,ysmooth))
        
        tt=[]
        for i in range(len(ros_path[0])):
            if (mapdata[int(ros_path[1][i]),int(ros_path[0][i])] > 0):
                tt.append([ros_path[0][i],ros_path[1][i]])
        ttt = np.array(tt) 
    
#         plt.figure(figsize=(10,10))
#         plt.imshow(mapdata+orgmapdata)
#         plt.plot(ttt[:,0],ttt[:,1],'bo--',linewidth=1, markersize=5)
#         plt.plot(decomp_X,decomp_Y,'go--',linewidth=1, markersize=5)
#         plt.plot(Sdecomp_X,Sdecomp_Y,'ro--',linewidth=1, markersize=5)
#         plt.show()
        
        rrtsmoothpath = Path()
        rrtsmoothpath.header.frame_id = "map"
        rrtsmoothpath.header.stamp = rospy.Time.now()
        for i in range(1,len(ttt[:,0])+1):
            pose = PoseStamped()
            ans = grid2meters([ttt[-i][0],ttt[-i][1]])
            pose.header = rrtsmoothpath.header
            pose.pose.position.x = ans[0] 
            pose.pose.position.y = ans[1]
            pose.pose.orientation.w = 1
            rrtsmoothpath.poses.append(pose)            
        path_pub.publish(rrtsmoothpath)


rospy.init_node('GlobalPlanner', anonymous=True)
path_pub = rospy.Publisher('global_path', Path, queue_size=10)
rospy.Subscriber("/map", OccupancyGrid, map_callback)
rospy.Subscriber("/odometry", Odometry, callbackodom)
rospy.Subscriber("/move_base_simple/goal", PoseStamped, callbackgoal)
rospy.spin()