#!/usr/bin/env python

import os
import csv
import rospy

# TODO Import waypoint messages

class MissionPlanner(object):

    def __init__(self):
        self.start_time = None

        # TODO: Init mission planner Node
        
        self.wait_master_initialziation()
        
        # TODO: Create publisher to publish  mission path 

        # TODO: Get waypoints file path from parameters
        
        # TODO: Load waypoints from fie

        # TODO: Publish waypoints
        
        # TODO: Run ROS loop without logic inside the loop

        pass

    # Wait ROS Master node is initialized 
    def wait_master_initialziation(self):
        while not self.start_time and not rospy.is_shutdown():
            self.start_time = rospy.Time.now().to_nsec()

        if not rospy.is_shutdown():
            rospy.loginfo('Mission Planner: Ros master initialized.')
            

    def load_waypoints(self, path):
        waypoints = []

        if os.path.isfile(path):
            waypointsFile = open(path, 'r')  
           
            with waypointsFile:  
                reader = csv.reader(waypointsFile)
                for row in reader:
                   # row[0] to access first element
                   # row[1] to access second element

                   #TODO:  create new BaseWaypoint and add to waypoints array

            rospy.loginfo('Waypoints Loaded: found %d waypoints', len(waypoints))
        else:
            rospy.logerr('%s is not a file', path)
        
        return waypoints

    def publish_waypoints(self, waypoints):
        # TODO: Crete new Path message and publish to the topic
        self.waypoints_publisher.publish(path)     


if __name__ == '__main__':
    try:
        MissionPlanner()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start mission planner node.')
        pass