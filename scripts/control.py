#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ProcessControl(object):

    def __init__(self):

        rospy.init_node("process_control", anonymous=True)

        # Topics to conmmand and hear back from the other nodes
        self.command_pub = rospy.Publisher("/me495/command", String, queue_size = 10)
        rospy.Subscriber('/me495/result', String, self.callback)

        #-----------------------------------------------------------------------
        # The main pipeline
        rospy.sleep(5)

        self.drawing = False

        self.command_pub.publish("Take Photo")
        self.command_pub.publish("Localize Board")

        while not rospy.is_shutdown():
            # Start drawing
            if self.drawing == True:
                print("Command to start drawing")
                self.command_pub.publish("Start Drawing")
                self.drawing = False


    def callback(self, string):

        if string.data == "All Set":
            self.drawing = True



if __name__ == "__main__":
    processcontrol = ProcessControl()
    while not rospy.is_shutdown():
        rospy.spin()
