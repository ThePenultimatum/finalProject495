#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int8MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Path planning
from edgesToPath import *


class FacialRecognition(object):

    def __init__(self):
        rospy.init_node("facial_recognition", anonymous=True)
        # Transfer image between ROS and Opencv
        self.bridge = CvBridge()

        # Wait for taking and processing a photo
        rospy.Subscriber("/me495/command", String, self.callback)

        # Publish the drawing trajectory
        self.trajectory_pub = rospy.Publisher("/me495/trajectory", Int8MultiArray, queue_size = 1)


    def callback(self, string):
        # Take a photo when
        if string.data == "Take Photo":
            print("Taking Photo")

            # Read in the image and transfer through CVBridge
            # image = rospy.wait_for_message('/camera_driver/image_raw', Image)
            # try:
            #     self.image_in = self.bridge.imgmsg_to_cv2(image, "bgr8")
            #     print(type(self.image_in))
            # except CvBridgeError as e:
            #     print(e)

            # self.photo_retake()
            self.image_process()


    def image_process(self):
        # Apply Canny Edge Detector
        print("Processing Image")
        self.image_in = cv2.imread("/home/ethan/sawyer_ws/src/me495_vision/scripts/lenna.png")
        edges = resize_edge(self.image_in)
        pathlist = getPointsFromEdges(edges)
        pathlist.append((-1,-1))
        print("Path length: ", len(pathlist))
        # plt.imshow(edges)
        # plt.show()

        # Publish to drawing node
        path = Int8MultiArray()
        pathlist = np.array(pathlist).reshape(2*len(pathlist))
        path.data = pathlist
        self.trajectory_pub.publish(path)
        print("Publishing Trajectory")



if __name__ == "__main__":
    fr = FacialRecognition()
    while not rospy.is_shutdown():
        rospy.spin()
