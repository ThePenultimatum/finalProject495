#!/usr/bin/env python
import rospy
import modern_robotics as mr
import numpy as np
import intera_interface
from geometry_msgs.msg import Point


class PositionControl(object):

    def __init__(self):
        # Initialize ROS node
        rospy.init_node("position_control", anonymous=True)
        self.point_pub = rospy.Publisher("/me495/current-position", Point, queue_size = 10)

        # Communicate with perception node
        rospy.Subscriber('/me495/target_position', Point, self.callback_point)

        # Enable the Sawyer
        rs = intera_interface.RobotEnable()
        rs.enable()
        # Set the right arm and velocity ratio
        self.mylimb = intera_interface.Limb("right")
        self.mylimb.set_joint_position_speed(0.2)
        # Dictionary for store target position in joint space
        self.waypoints = self.mylimb.joint_angles()

        # Command sent to perception node
        self.command = Point()
        self.command.x = 0
        self.command.y = 0
        self.command.z = 0

        # IK parameter
        self.Slist = np.array([[0, 0,  1,      0,     0,       0],
                               [0, 1,  0, -0.317,     0,   0.081],
                               [1, 0,  0,      0, 0.317, -0.1925],
                               [0, 1,  0, -0.317,     0,   0.481],
                               [1, 0,  0,      0, 0.317,  -0.024],
                               [0, 1,  0, -0.317,     0,   0.881]]).T
        self.M = np.array([[ 0,  0, 1, 1.01475],
                           [-1,  0, 0,  0.1603],
                           [ 0, -1, 0,   0.317],
                           [ 0,  0, 0,       1]])
        self.eomg = 0.01
        self.ev = 0.001

        #-----------------------------------------------------------------------
        # First observe all the AR_tags
        # Target point
        self.point = Point()
        self.point.x = 0.675
        self.point.y = 0.1603
        self.point.z = 0.650
        # This is the initial configuration of right_hand_camera in base frame
        # The desired end-effector configuration
        self.T = np.array([[ 0,  0,  1, self.point.x],
		                   [-1,  0,  0, self.point.y],
		                   [ 0, -1,  0, self.point.z],
		                   [ 0,  0,  0,            1]])
        # thetalist0; current joint angles
        self.thetalist = [0.00722578, -1.13799861, -0.01079448, 1.7510739, 0.00573321, 0.95772104]
        # Move to initial position
        self.move_to_high(self.thetalist)


    def move_to_high(self, thetalist0):
        # Solve IK
        self.thetalist, success = mr.IKinSpace(self.Slist, self.M, self.T, thetalist0, self.eomg, self.ev)

        # Nove to the initial position
        self.waypoints['right_j0'] = self.thetalist[0]
        self.waypoints['right_j1'] = self.thetalist[1]
        self.waypoints['right_j2'] = self.thetalist[2]
        self.waypoints['right_j3'] = self.thetalist[3]
        self.waypoints['right_j4'] = self.thetalist[4]
        self.waypoints['right_j5'] = self.thetalist[5]
        self.waypoints['right_j6'] = 0.0
        self.mylimb.move_to_joint_positions(self.waypoints, timeout = 20.0, threshold = 0.05)

        # Publish the command to perception node
        rospy.sleep(1)
        # (-1, 0, 0) for perceiving all four tags
        self.command.x = -1
        self.point_pub.publish(self.command)


    def move_to_each(self, thetalist0):
        # Solve IK
        self.thetalist, success = mr.IKinSpace(self.Slist, self.M, self.T, thetalist0, self.eomg, self.ev)

        # Nove to the initial position
        self.waypoints['right_j0'] = self.thetalist[0]
        self.waypoints['right_j1'] = self.thetalist[1]
        self.waypoints['right_j2'] = self.thetalist[2]
        self.waypoints['right_j3'] = self.thetalist[3]
        self.waypoints['right_j4'] = self.thetalist[4]
        self.waypoints['right_j5'] = self.thetalist[5]
        self.waypoints['right_j6'] = 0.0
        self.mylimb.move_to_joint_positions(self.waypoints, timeout = 20.0, threshold = 0.05)

        # Publish the command to perception node
        # (0, 0, 0) for perceiving each tags
        rospy.sleep(0.5)
        self.command.x = 0
        self.point_pub.publish(self.command)


    def callback_point(self, point):
        self.point = point
        rospy.sleep(0.1)
        # print("recieving", self.point)

        if not (point.x == 0 and point.y == 0 and point.z == 0):
            self.T[0][3] = point.x
            self.T[1][3] = point.y
            self.T[2][3] = point.z

            # self.thetalist = [self.mylimb.joint_angle(joint) for joint in self.mylimb.joint_names()]
            # self.thetalist = self.thetalist[0:6]
            self.thetalist = [0.0, -0.55, 0.0, 1.89, 0.0, -1.37]
            self.move_to_each(self.thetalist)
            rospy.sleep(0.5)



if __name__ == "__main__":
    pc = PositionControl()
    while not rospy.is_shutdown():
        rospy.spin()
