# TODO: Add indicator that node should be run by python

# line above indicates that python is responsible for running this node
import os
import csv
import rospy
import numpy as np
import pygame

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

# python class definition
class CameraTester(object):

    # python constuctor definition
    def __init__(self):

        # inialize node start time - will be populated when master node 
        self.start_time = None
        
        # Store images rece
        self.image  = None
        self.got_image = False
        self.init_pygame()
        self.bridge = CvBridge()

        # TODO: Init node

        # wait master node is initialized and record start time
        self.wait_master_initialziation()
        
        # TODO: Subscribe to the ROS Bridge Camera topic

        #  wait till we got first image
        self.wait_initialziation()

        # run node infinite loop      
        self.loop()  
    

    # TODO: Write callback method for the subscriber

    # Initialize pygame to display camera images
    def init_pygame(self):
        pygame.init()
        pygame.display.set_caption("Camera images")
        self.screen = pygame.display.set_mode([1280,720])

    # wait master node is initialized and record start time
    def wait_master_initialziation(self):
        while not self.start_time and not rospy.is_shutdown():
            self.start_time = rospy.Time.now().to_nsec()

        if not rospy.is_shutdown():
            rospy.loginfo('CameraTester: Ros master initialized.')


    def wait_initialziation(self):
        # define sleep rate foe the loop
        rate = rospy.Rate(10)

        # wait till we get position initalized 
        while not rospy.is_shutdown() and not self.got_image:
            rate.sleep()

        if not rospy.is_shutdown():
            rospy.loginfo('CameraTester: Connected to vechicle - got camera images')
   

    # main node loop 
    def loop(self):
        
        # define loop rate in Hz
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            
            # process stored image and display it in pygame window
            self.process_frame()

            # update pygame window
            pygame.display.flip()

            # wait 1/20 sec
            rate.sleep()

    # convert open cv image to pygame image and display
    def process_frame(self):
        frame = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)
        frame = np.flip(frame, 0)
        frame = pygame.surfarray.make_surface(frame)
        self.screen.blit(frame,(0,0))
        return frame

# python way to indicate what to do if this file is run as executable rather then imported as library
if __name__ == '__main__':
    try:
        # create CameraTester instance and initiate loop sequence
        CameraTester()
    except rospy.ROSInterruptException:
        # catch and log ROS errors
        rospy.logerr('Could not start camera tester node.')
        pass
    finally:
        pygame.quit()