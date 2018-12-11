#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8

class Control(object):
    def __init__(self):
        rospy.init_node('controller')
        self.cmd_pub = rospy.Publisher('cmd', Int8, queue_size = 1)
        self.res_sub = rospy.Subscriber('result', Int8, self.res_callback)
        self.cmd_pub.publish(0) # Start
    
    def res_callback(self, msg)


def main():
    controller = Control()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
