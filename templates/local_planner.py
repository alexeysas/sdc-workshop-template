#!/usr/bin/env python

import os
import csv
import rospy
import numpy as np

# TODO: Import reqired messages

# We will use KDTree to find  
from scipy.spatial import KDTree

# define how many futher waypoints will be used for local path
MAX_WAYPOINTS = 10

# define vehicle speed we want to drive with
PLANNED_SPEED = 8

class LocalPlanner(object):

    def __init__(self):
        # 
        self.waypoints_xy = None
        
        # store last received vehilce position
        self.position = None

        # indicates either ROSA master is initialized
        self.start_time = None
        
        # store last received vehilce position
        # TODO: Init local planner node 
        
        # wait ROS master initialization
        self.wait_master_initialziation()

        # TODO: subscribe to mission_planner to get waypoints, please use init_mission callback
        
        # Subscribe to  vehicle Odometry
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.process_position)
               
        # Prepare publisher to publish Local Path 
        self.local_publisher = rospy.Publisher('/planner/local_waypoints', LocalPath, queue_size=1)

        # wait till we get planneed path and vehicle Odometry
        self.wait_initialziation()

        self.loop()  
    
    def wait_master_initialziation(self):
        # wait till ROS master node is initalzied 
        while not self.start_time and not rospy.is_shutdown():
            self.start_time = rospy.Time.now().to_nsec()

        if not not rospy.is_shutdown():
            rospy.loginfo('Local Planner: Ros master initialized.')


    def wait_initialziation(self):
        rate = rospy.Rate(10)

        # wait till we get position initalized 
        while not rospy.is_shutdown() and not self.position:
            rate.sleep()

        if not not rospy.is_shutdown():
            rospy.loginfo('Local Planner: Connected to vechicle - got vehicle position.')

    def is_mission_initialized(self):
        return True if self.waypoints_xy else False

    def init_mission(self, path):
        # get waypoints from  message
        self.mission_waypoints = path.waypoints

        # We need to find waypoint from the path which is closest to the current car position
        # We can use simple search in the list but it have O(n) complexity which is generaly slow
        # KD tree allows to search in Log(n) time https://en.wikipedia.org/wiki/K-d_tree
        # https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.spatial.KDTree.html
        if not self.waypoints_xy:
            self.waypoints_xy = [[waypoint.x , waypoint.y] for waypoint in self.mission_waypoints]

            self.waypoint_tree = KDTree(self.waypoints_xy)
            rospy.loginfo('Local Planner: Received waypoints')

    def process_position(self, position):
        # TODO: Store postion received from car odometry
        self.position = None
        pass


    def loop(self):
        # Plan futher path each 1/10 sec
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_mission_initialized() and self.position:
                self.plan_path()
     
            rate.sleep()


    def plan_path(self):
        # determine closest waypoint in the path going forward
        closest_waypoint_idx = self.get_car_closest_waypoint_idx()
        
        # prepare  planned path forward
        local_path = self.prepare_local_path(closest_waypoint_idx)

        # TODO: publish local path to the /planner/local_waypoints topic
       

  

    # find trajecttry start waypoint starting from closest waypoint to the car
    def get_car_closest_waypoint_idx(self):
        # Remember structure of the Odometry message to correctly get x and y coordinates
        # http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
        x = self.position.pose.position.x
        y = self.position.pose.position.y
    
        # TODO: query KDTree to get clsoest waypoint        
       
        # TODO: find closest waypoint which is just before the car    
     
        return closest_idx

    def prepare_local_path(self, closest_waypoint_idx):
        
        # find trajectory last index
        farthest_idx = closest_waypoint_idx + MAX_WAYPOINTS
        last_index = len(self.waypoints_xy) - 1

        base_waypoints = self.mission_waypoints[closest_waypoint_idx:farthest_idx]

        # TODO: Create new LocalPath object with selected waypoints, 
        # add desired speed to each endpoint   
    
        return path
     

if __name__ == '__main__':
    try:
        LocalPlanner()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start local planner node.')
        pass