#!/usr/bin/env python

import math
import rospy
import tf
from pid import PID
from stanley import Stanley

from nav_msgs.msg import Odometry

# TODO: Import required custom message types 

class SimpleController(object):

    def __init__(self):
        self.position = None
        self.x = None
        self.y = None
        self.v = None
        self.yaw = None
        self.start_time = None
        self.previous_time = None
        self.waypoints = None

        # TODO: Initialize ROS Node
      
        # wait ROS master is ready
        self.wait_master_initialziation()

        # TODO: create publisher to publish CarlaVehicleControl to /carla/ego_vehicle/vehicle_control_cmd 
        
        # TODO: subscribe to local planner node to get trajectory with self.process_waypoints callback

        # TODO: subscribe to Carla odometry to get Odometry with self.process_position callback

        # wait we start getting odometry and waypoints from planner
        self.wait_initialziation()

        # initialzie longitudial and lateral controllers
        self.init_controllers()

        # run node loop
        self.loop()

    def init_controllers(self)
        # PID Controller
        P = 0.8
        I = 0
        D = 0.3
        mn = 0.
        mx = 1.0
        
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        self.steering_controller = Stanley(1.22, 0.04)

    def process_position(self, position):
        # TODO: we need to get position from message: x, y, yaw angle
        self.position = position.pose
        
        # TODO: get x and y from Odometry message
        self.x = None
        self.y = None
       
        # Odometry message stored car heading using quaternions
        # https://www.youtube.com/watch?v=zjMuIxRvygQ
        # So we need to convert it to euler "yaw" angle to determine current heading
        self.yaw = None

        # TODO: we cam get x and y components of the velocity from the message
        # TODO: calculate full velocity hope it helps: https://www.mathplanet.com/education/pre-algebra/right-triangles-and-algebra/the-pythagorean-theorem :) 
        v_x = position.twist.twist.linear.x
        v_y = position.twist.twist.linear.y
        self.v = None

       
    def process_waypoints(self, path):     
        # TODO: Just store last endpoints in class field
        pass

    def wait_master_initialziation(self):
        rate = rospy.Rate(10)

        # wait till ROS master node is initalzied 
        while not self.start_time and not rospy.is_shutdown():
            self.start_time = rospy.Time.now().to_nsec()

        if not rospy.is_shutdown():
            rospy.loginfo('Controller: Ros Master initialized.')


    def wait_initialziation(self):
        rate = rospy.Rate(10)

        # wait till we get position initalized 
        while not rospy.is_shutdown() and not self.position:
            rate.sleep()

        if not rospy.is_shutdown():
            rospy.loginfo('Controller: Connected to vechicle - got vehicle position.')

        # wait till we get waypoints
        while not rospy.is_shutdown() and not self.waypoints:
            rate.sleep()
        
        if not rospy.is_shutdown():
            rospy.loginfo('Controller: Got waypoints')


    def loop(self):
        rate = rospy.Rate(10)
     
        while not rospy.is_shutdown():
            if not self.previous_time:
                # we need to track previous loop iteration time so we can calculate delta for PID controller
                self.previous_time = rospy.Time.now().to_nsec() / 1000.0
                throttle_output = 0
                steer_output    = 0
                brake_output = 0
            else:
                # get cuurent time
                now = rospy.Time.now().to_nsec() / 1000.0
                
                # get time passed from previous command
                dt = now - self.previous_time
                self.previous_time = now

                # main method to determine  contril commands
                throttle_output, steer_output, brake_output =  self.control(dt)

                # TODO: publish controll messages to the /carla/ego_vehicle/vehicle_control_cmd topic
            rate.sleep()

    def get_yaw_last_position(self):
        quaternion = (
                        self.position.pose.orientation.x,
                        self.position.pose.orientation.y,
                        self.position.pose.orientation.z,
                        self.position.pose.orientation.w)
            
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        return yaw

    def control(self, dt):
        
        #
        ## TODO Implement PID controller to control acceleration
        #

        # TODO: get desired speed form next waypoint
        waypoints  = self.waypoints
        v_desired   =  None
        
        # TODO: calculate velocity error
        vel_error = None

        # implement PID Controller 
        throttle = self.throttle_controller.step(vel_error, dt)

        if throttle > 0:
            throttle_output = throttle
            brake_output    = 0
        else:
            throttle_output =  0
            brake_output    = -throttle
            

        #
        ## TODO Implement Stanley controller to control steering angle
        #


        # TODO determine trajectory heading direction - you can use: math.atan2 to find angle 

    
        
        # TODO determine Cross track error e
        # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line see "Line defined by two points" section
        

        # TODO: termine angle between desired car heading  and trajectory heading        
        diff = theta_radians - self.yaw 

        if diff > math.pi:
            diff = diff - 2 * math.pi 

        if diff < -math.pi:
            diff = diff + 2 * math.pi 

        # TODO implement heading controler by providing it with current velocity, cross track error e and heading error 
        # Minus sign just to compensate the the way how Carla simulator with ROS bridge works 
        steer = -self.steering_controller.step(diff, self.v, e)

        rospy.loginfo('theta_radians: %s, yaw: %s, angle difference: %s, error: %s, speed %s, steer: %s', theta_radians, self.yaw, diff, e, self.v, steer)

        steer_output  = steer
      
        return throttle_output, steer_output, 0
 
if __name__ == '__main__':
    try:
        SimpleController()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start mission planner node.')
        pass