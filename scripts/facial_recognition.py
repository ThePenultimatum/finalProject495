#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
print ('OpenCV version: ', cv2.__version__)

# Path planning
from edgesToPath import *


class FacialRecognition(object):

    def __init__(self):
        rospy.init_node("facial_recognition", anonymous=True)
        # Transfer image between ROS and Opencv
        self.bridge = CvBridge()
        self.state = "n"
        rospy.Subscriber('/camera_driver/image_raw', Image, self.callback)

        self.image_process()

        # self.state = raw_input("Take a photo? [y/n]")


    def callback(self, image):
        # Take a photo when type "y" in terminal
        if self.state == "y":
            self.state = "n"

            # Read in the image through CVBridge
            try:
                self.image_in = self.bridge.imgmsg_to_cv2(image, "bgr8")
                print(type(self.image_in))
            except CvBridgeError as e:
                print(e)

            # self.photo_retake()
            self.image_process()



    def photo_retake(self):
        plt.imshow(cv2.cvtColor(self.image_in, cv2.COLOR_BGR2RGB))
        plt.show()

        self.state = raw_input("Wanna retake a photo? [y/n]")


    def image_process(self):
        # Apply Canny Edge Detector
        self.image_in = cv2.imread('/home/ethan/sawyer_ws/src/me495_vision/scripts/lenna.png')
        plt.imshow(cv2.cvtColor(self.image_in, cv2.COLOR_BGR2RGB))
        plt.show()
        edges = resize_edge(self.image_in)
        plt.imshow(edges)
        plt.show()
        pathlist = getPointsFromEdges(edges)
        print(pathlist)



if __name__ == "__main__":
    fr = FacialRecognition()
    while not rospy.is_shutdown():
        rospy.spin()
