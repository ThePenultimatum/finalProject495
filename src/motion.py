#!/usr/bin/env python

import rospy

class Motion(object):
    def __init__(self):
        rospy.init_node('motion')
        self.cmd_sub = rospy.Subscriber('cmd', String, self.cmd_callback)
        self.res_pub = rospy.Publisher('result', String, queue_size = 1)
        # A service here
        self.plan_pub = rospy.Publisher('plane', Point, queue_size = 1)
    
    def cmd_callback(self, msg)
        pass
        

def main():
    arm_motion = Motion()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
