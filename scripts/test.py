#!/usr/bin/env python

import rospy
import modern_robotics as mr
import numpy as np
import intera_interface

def main():
    # I don't care too much about the variables. Some may be redundant.
    rospy.init_node("my_first_node")
    mylimb = intera_interface.Limb("right")
    rs = intera_interface.RobotEnable()
    rs.enable()
    waypoints = mylimb.joint_angles()
    mylimb.set_joint_position_speed(1)
    # We are about to draw a circle. center: x=0.575, r=0.225
    # Here is the joint angles at its center, which can be the first initial guess.
    thetalist0 = [0.00722578, -1.13799861, -0.01079448, 1.7510739, 0.00573321, 0.95772104, -0.00563395]
    waypoints['right_j0'] = 0.00722578
    waypoints['right_j1'] = -1.13799861
    waypoints['right_j2'] = -0.01079448
    waypoints['right_j3'] = 1.7510739
    waypoints['right_j4'] = 0.00573321
    waypoints['right_j5'] = 0.95772104
    waypoints['right_j6'] = -0.00563395
    #TODO: We need to find a way to convert list to values in dictionary.
    # To do the opposite direction: dict.values() (Check the order afterwards).
    mylimb.move_to_joint_positions(waypoints, timeout = 20.0, threshold = 0.05)
    Slist = np.array([[0, 0,  1,      0,     0,       0],
                      [0, 1,  0, -0.317,     0,   0.081],
                      [1, 0,  0,      0, 0.317, -0.1925],
                      [0, 1,  0, -0.317,     0,   0.481],
                      [1, 0,  0,      0, 0.317,  -0.024],
                      [0, 1,  0, -0.317,     0,   0.881],
                      [1, 0,  0,      0, 0.317, -0.1603]]).T
    M = np.array([[0, 0, 1, 1.01475], [-1, 0, 0, 0.1603], [0, -1, 0, 0.317], [0, 0, 0, 1]])
    # This is the configuration of the center point.
    T = np.array([[ 0, -1,  0,  0.575],
		          [-1,  0,  0, 0.1603],
		          [ 0,  0, -1,  0.317],
		          [ 0,  0,  0,      1]])
    x = 0.575
    y = 0.1603
    eomg = 0.01
    ev = 0.001
    i = 0
    N = 100
    dt = 2 * np.pi / N
    r = 0.225
    mylimb.set_joint_position_speed(0.5)
    while i < 2 * N:
        T[0][3] = x + r * np.cos(i * dt)
        T[1][3] = y + r * np.sin(i * dt)
        thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
        waypoints['right_j0'] = thetalist0[0]
        waypoints['right_j1'] = thetalist0[1]
        waypoints['right_j2'] = thetalist0[2]
        waypoints['right_j3'] = thetalist0[3]
        waypoints['right_j4'] = thetalist0[4]
        waypoints['right_j5'] = thetalist0[5]
        waypoints['right_j6'] = thetalist0[6]
        mylimb.move_to_joint_positions(waypoints, timeout = 20.0, threshold = 0.05)
        i = i + 1

if __name__ == '__main__':
    main()
