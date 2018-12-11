#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class Control(object):
    def __init__(self):
        rospy.init_node('controller')
        self.cmd_pub = rospy.Publisher('cmd', String, queue_size = 1)
        self.res_sub = rospy.Subscriber('result', String, self.res_callback)
        rospy.sleep(5)
        self.cmd_pub.publish('S') # Start
    
    def res_callback(self, msg):
        pass

def main():
    controller = Control()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
