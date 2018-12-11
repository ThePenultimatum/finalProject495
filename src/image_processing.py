#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def main():
    rospy.init_node("image_processer")
    s = rospy.Service('image_capture', rospy_tutorials.srv.AddTwoInts, get_new_image)
    msg = rospy.wait_for_message('/camera_driver/image_raw', JointTrajectoryControllerState)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")
