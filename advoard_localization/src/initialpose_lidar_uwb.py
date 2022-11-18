#!/usr/bin/env python3
'''
__author__ = "Bekir Bostanci"
__license__ = "BSD"
__version__ = "0.0.1"
__maintainer__ = "Bekir Bostanci"
__email__ = "bekirbostanci@gmail.com"
'''

import sys
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from pozyx_simulation.msg import  uwb_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid

from threading import Timer
import time
import math
import numpy as np

import map_matcher


global counter_point 
global occupancy_map 
global robot_realtime_pose 
global lidar_ranges 

occupancy_map = None
robot_realtime_pose = None
lidar_ranges = None

global sub_map
global sub_scan
global sub_local

def setInitialPosition(pose_ini_x,pose_ini_y,theta):
    #publish initialpose node 
    publisher_initial_pose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
    
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.seq = 1
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"
    
    initial_pose.pose.pose.position.x = pose_ini_x
    initial_pose.pose.pose.position.y = pose_ini_y
    initial_pose.pose.pose.position.z = 0.0

    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = math.sin(theta/2)
    initial_pose.pose.pose.orientation.w = math.cos(theta/2)
    #standart covariance 
    initial_pose.pose.covariance = [0.0041843402126411355, 4.2477366476223466e-05, 0.0, 0.0, 0.0, 0.0, 4.2477366476223466e-05, 0.0005426193303021687,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0003465551204496198]
    #before the one message publishing we should sleep 1 second
    rospy.sleep(1)
    for i in range(10):
        time.sleep(0.1)
        publisher_initial_pose.publish(initial_pose)

def start_initializing():
    global occupancy_map
    global counter_point 
    global robot_realtime_pose
    global lidar_ranges

    global sub_map
    global sub_scan
    global sub_local
    
    map_width = occupancy_map.info.width
    map_resolution = occupancy_map.info.resolution     #map resolution convert meter to milli meter 
    map_origin = [occupancy_map.info.origin.position.x   ,occupancy_map.info.origin.position.y ]
    
    map_max_radius = 1400       #map max radius millimeter 
    
    sub_map.unregister()        #stop listen map data
    sub_scan.unregister()       #stop listen lidar data
    sub_local.unregister()      #stop listen robot position

    processed_lidar = []
    processed_map =[]
        
    counter_map = -1 
    for i in occupancy_map.data:
        counter_map = counter_map +1 
        if i>99 : 
            x = ((counter_map % map_width) * map_resolution) + map_origin[0]
            y = ((counter_map / map_width) * map_resolution) + map_origin[1]
            if abs(x -(robot_realtime_pose.point.x)) <  map_max_radius  and abs(y - (robot_realtime_pose.point.y)) < map_max_radius:
                processed_map.append([x,y])
    
    for i in range(0,360):
        if  lidar_ranges.ranges[i] <= 2:    #max 2 meter radius 
            degree_lidar_c = math.cos(np.deg2rad(i))
            degree_lidar_s = math.sin(np.deg2rad(i))
            #lidar data convert cartesian coordinate 
            processed_lidar.append([lidar_ranges.ranges[i] * degree_lidar_c  , lidar_ranges.ranges[i] * degree_lidar_s ]) 
    
    #find lidar and map data coverage ratio of maximum degree
    final_degree = map_matcher.main(processed_lidar,processed_map,robot_realtime_pose.point.x,robot_realtime_pose.point.y)
    #set the initial pose and direction 
    setInitialPosition(robot_realtime_pose.point.x,robot_realtime_pose.point.y,math.radians(final_degree))
    
     
def subscribe_lidar(LaserScan):
    global lidar_ranges
    lidar_ranges= LaserScan

def subscribe_uwb(Point):
    global robot_realtime_pose
    robot_realtime_pose  = Point

def subscribe_map(OccupancyGrid):
    global occupancy_map
    occupancy_map= OccupancyGrid

if __name__ == "__main__":
    
    
    global sub_map
    global sub_scan
    global sub_local
    
    rospy.init_node('initialpose_lidar_uwb', anonymous=True)
    sub_map = rospy.Subscriber("map",OccupancyGrid, subscribe_map)                  #get map data
    sub_scan = rospy.Subscriber("scan", LaserScan, subscribe_lidar)                 #get lidar data
    sub_local = rospy.Subscriber("localization_data_topic", PointStamped, subscribe_uwb)    #get robot position

    time.sleep(1)
    control_ready = 0
    while control_ready < 20 :
        control_ready = control_ready + 1 
        time.sleep(0.1)
        if occupancy_map != None and lidar_ranges != None and robot_realtime_pose != None : 
            rospy.loginfo("Initial pose estimation is working.")
            start_initializing()
            break 


    if occupancy_map == None :
        rospy.logwarn("Initial pose estimation is not working.\nPlease control map server.")
    elif lidar_ranges == None : 
        rospy.logwarn("Initial pose estimation is not working.\nPlease control scan topic")
    elif robot_realtime_pose == None :
        rospy.logwarn("Initial pose estimation is not working.\nPlease control localization topic ")

    rospy.loginfo("Initial pose estimation is closing.")
    rospy.signal_shutdown('initialpose_lidar_uwb')
    rospy.spin()

