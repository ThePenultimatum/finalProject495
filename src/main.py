#!/usr/bin/env python
import argparse
import sys

from copy import copy

import rospy

import actionlib

import modern_robotics as mr
import numpy as np

from geometry_msgs.msg import Point
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)


import intera_interface

from intera_interface import CHECK_VERSION

from Trajectory import Trajectory

class drawing_control(object):
    def __init__(self):
        self.plot_points=[(4, 60), (4, 60), [5, 61], [5, 62], [5, 63], [6, 64], [7, 65], [8, 66], [9, 67], [10, 68], [11, 69], [12, 70], [13, 71], [14, 71], [15, 72], [16, 73], [17, 73], [18, 73], [19, 74], [20, 74], [21, 74], [22, 74], [23, 74], [24, 74], [25, 73], [26, 73], [27, 73], (-1, -1), (5, 31), (5, 31), [6, 30], [7, 29], [8, 28], [9, 27], [10, 26], [11, 25], [12, 24], [13, 24], [14, 24], [15, 23], [16, 23], [17, 23], [18, 22], [19, 22], [20, 22], [21, 22], [22, 22], [23, 22], [24, 22], [25, 22], [26, 22], [27, 22], [28, 22], [29, 22], [30, 23], [31, 23], [32, 24], [33, 25], [34, 25], [35, 26], [34, 27], [33, 27], [32, 27], [31, 27], [30, 27], [29, 27], [28, 27], [27, 28], [28, 29], [28, 30], [28, 31], [28, 32], [28, 33], [28, 34], [28, 35], [28, 36], [28, 37], [28, 38], [29, 39], [29, 40], [29, 41], (-1, -1), [26, 28], [25, 28], [24, 28], [23, 28], [22, 28], [21, 29], [20, 30], [19, 31], [18, 32], [17, 32], [16, 33], [15, 34], [14, 35], [13, 36], [13, 37], [13, 38], [12, 39], [11, 40], [11, 41], [11, 42], [11, 43], [12, 44], [12, 45], [12, 46], [12, 47], [12, 48], [12, 49], [11, 50], [12, 51], [12, 52], [12, 53], [13, 54], [13, 55], [13, 56], [14, 57], [15, 58], [16, 59], [17, 60], [18, 61], [19, 62], [20, 62], [21, 62], [22, 63], [23, 63], [24, 63], [25, 64], [26, 64], [27, 64], [28, 64], [29, 65], [30, 65], [31, 66], [32, 66], [33, 67], [34, 67], [35, 67], [36, 67], [37, 66], [38, 66], [39, 66], [40, 66], [41, 66], [42, 66], [43, 67], [44, 67], [45, 67], [46, 67], [47, 67], [48, 67], [49, 66], [50, 66], [51, 66], [52, 66], (-1, -1), [42, 68], [43, 69], [43, 70], [43, 71], [42, 72], [41, 73], [40, 74], [39, 74], [38, 74], [37, 75], [36, 75], [35, 75], [34, 75], [33, 75], [32, 75], [31, 75], [30, 74], [29, 73], [29, 72], [30, 71], [31, 71], [32, 72], [33, 73], (-1, -1), [32, 70], [33, 70], [34, 71], [35, 71], [36, 70], [37, 69], [38, 68], [39, 68], [40, 68], [41, 68], (-1, -1), [31, 69], (-1, -1), [10, 50], [9, 50], [8, 50], [7, 49], [8, 48], [9, 48], (-1, -1), [32, 22], [33, 22], [34, 21], [35, 21], [36, 22], [37, 22], [38, 22], [39, 23], [40, 23], [41, 24], [42, 25], [43, 26], [44, 27], [45, 28], [46, 28], [47, 28], [48, 28], [49, 29], [50, 29], [51, 30], [52, 30], [53, 31], [54, 31], [55, 32], [56, 33], [57, 33], [58, 34], [59, 35], [60, 36], [61, 36], [62, 37], [63, 38], [64, 39], [65, 40], [66, 41], [66, 42], [67, 43], [67, 44], [66, 45], [67, 46], [67, 47], [67, 48], [67, 49], [67, 50], [66, 51], [66, 52], [65, 53], [65, 54], [64, 55], [63, 56], [62, 57], [61, 58], [60, 59], [59, 60], [58, 61], [57, 62], [57, 63], [56, 64], [55, 65], [54, 66], [55, 67], [56, 68], [57, 69], [58, 70], [59, 71], [60, 72], [60, 73], [60, 74], [61, 75], [61, 76], [61, 77], [62, 78], [62, 79], [62, 80], [63, 81], [63, 82], [63, 83], [64, 84], [64, 85], [64, 86], [65, 87], [65, 88], [65, 89], [65, 90], [66, 91], [66, 92], [66, 93], [67, 94], [67, 95], [67, 96], [68, 97], [68, 98], [68, 99], (-1, -1), [63, 36], [64, 36], [65, 36], [66, 37], [67, 37], [68, 38], [69, 38], [70, 37], [71, 38], [72, 39], [73, 40], [73, 41], [73, 42], [74, 43], [74, 44], [74, 45], [74, 46], [74, 47], [74, 48], [74, 49], [75, 50], [75, 51], [75, 52], [75, 53], [74, 54], [74, 55], [74, 56], [73, 57], [72, 58], [72, 59], [71, 60], [70, 61], [70, 62], [69, 63], [68, 64], [67, 65], [66, 66], [65, 67], [64, 67], [63, 67], [62, 67], [61, 67], [60, 67], [59, 67], [58, 67], (-1, -1), [69, 36], [68, 35], [68, 34], [67, 33], [66, 33], [65, 34], [64, 34], [63, 34], [62, 34], [61, 34], (-1, -1), [58, 32], [59, 32], [60, 31], [61, 30], [62, 29], [62, 28], [63, 27], [63, 26], [64, 25], [64, 24], [64, 23], [65, 22], [65, 21], [66, 20], [66, 19], [67, 18], [67, 17], [67, 16], [68, 15], [68, 14], [69, 13], [69, 12], [70, 11], [70, 10], [71, 9], [71, 8], [72, 7], [72, 6], [73, 5], [73, 4], [74, 3], [75, 2], [76, 1], [77, 0], (-1, -1), (9, 39), (9, 39), [10, 38], [10, 37], [11, 36], [11, 35], [12, 34], (-1, -1), [9, 40], (-1, -1), (25, 68), (25, 68), [26, 68], [27, 67], [28, 67], (-1, -1), [25, 69], [26, 70], [27, 71], (-1, -1), (26, 52), (26, 52), [27, 51], [27, 50], [28, 49], [28, 48], [29, 47], (-1, -1), [26, 53], [26, 54], [26, 55], [26, 56], [26, 57], [26, 58], [26, 59], [27, 60], [27, 61], [28, 62], (-1, -1), (30, 44), (30, 44), [31, 45], [32, 46], [33, 47], [34, 48], [34, 49], [34, 50], [35, 51], [35, 52], [35, 53], [35, 54], [36, 55], [36, 56], [36, 57], [35, 58], [35, 59], [34, 60], (-1, -1), [31, 43], [31, 42], [32, 41], [33, 41], [34, 40], [34, 39], [35, 38], [36, 37], [36, 36], [37, 35], [37, 34], [37, 33], [37, 32], (-1, -1), (30, 54), (30, 54), [30, 55], [30, 56], [31, 57], [32, 58], [32, 59], [32, 60], [32, 61], [31, 62], [30, 63], (-1, -1), [32, 56], [32, 55], [32, 54], (-1, -1), (31, 32), (31, 32), [31, 33], [31, 34], [31, 35], [32, 36], [33, 36], [34, 35], [33, 34], [33, 33], [33, 32], [34, 31], [34, 30], (-1, -1), (38, 50), (38, 50), [38, 51], [39, 52], [39, 53], [40, 54], [40, 55], [41, 56], [42, 57], [43, 58], [44, 59], [45, 60], [46, 61], [47, 61], [48, 60], [49, 59], (-1, -1), [47, 59], [46, 59], (-1, -1), (39, 37), (39, 37), [40, 38], [41, 38], [42, 39], [43, 40], [44, 41], [44, 42], [45, 43], (-1, -1), [44, 39], [45, 38], (-1, -1), [40, 39], (-1, -1), [40, 36], [41, 35], [42, 34], [43, 33], [44, 32], [45, 33], (-1, -1), (43, 44), (43, 44), [43, 45], [44, 46], [45, 46], [44, 47], [43, 48], [43, 49], [44, 50], [44, 51], (-1, -1), (46, 49), (46, 49), [47, 48], [47, 47], [47, 46], [47, 45], [47, 44], [47, 43], [47, 42], [47, 41], [47, 40], [47, 39], [48, 38], [48, 37], [47, 36], [48, 35], [48, 34], [49, 33], [50, 34], [51, 34], [50, 35], [51, 36], [52, 36], (-1, -1), [50, 37], [50, 38], (-1, -1), [47, 33], (-1, -1), [46, 50], [46, 51], (-1, -1), (47, 55), (47, 55), [47, 56], [47, 57], (-1, -1), (49, 55), (49, 55), [50, 55], [51, 54], [52, 53], [52, 52], [52, 51], (-1, -1), [49, 56], [49, 57], (-1, -1), (50, 62), (50, 62), [51, 61], [52, 60], [52, 59], [53, 58], [54, 57], [55, 56], [56, 56], (-1, -1), (53, 34), (53, 34), [54, 35], [55, 36], [56, 37], [57, 37], [58, 38], [59, 38], [60, 39], [61, 40], [62, 40], [63, 41], [63, 42], [64, 43], [64, 44], [64, 45], [64, 46], (-1, -1), (58, 56), (58, 56), [59, 55], [60, 54], [60, 53], [61, 52], [62, 51], [62, 50], (-1, -1), [58, 57], (-1, -1), (75, 60), (75, 60), [76, 59], [76, 58], [77, 57], [78, 56], [78, 55], [78, 54], [79, 53], [80, 52], [80, 51], [81, 50], [82, 51], [83, 50], [82, 49], [82, 48], [83, 47], [84, 46], [84, 45], [84, 44], [84, 43], [84, 42], [85, 41], [85, 40], [86, 39], [86, 38], [86, 37], [86, 36], [86, 35], [85, 34], [86, 33], (-1, -1), [84, 35], [83, 36], [82, 37], [81, 38], [81, 39], [80, 40], [79, 41], [78, 42], [77, 43], [78, 44], [79, 45], [78, 46], [78, 47], [79, 48], [79, 49], [79, 50], (-1, -1), [84, 33], (-1, -1), [78, 52], [77, 51], (-1, -1), [75, 61], [76, 62], [77, 63], [77, 64], [78, 65], [79, 66], [79, 67], [80, 68], [80, 69], [81, 70], [82, 71], [83, 72], [84, 72], [85, 71], [85, 70], [85, 69], [85, 68], [85, 67], [84, 66], [84, 65], [84, 64], [83, 63], [83, 62], [83, 61], [82, 60], [83, 59], [84, 58], [84, 57], [84, 56], [84, 55], [84, 54], [83, 53], [82, 53], (-1, -1)]

    def enable_robot(self):
        rp = intera_interface.RobotParams()
        valid_limbs = rp.get_limb_names()
        if not valid_limbs:
            rp.log_message(("Cannot detect any limb parameters on this robot. "
              "Exiting."), "ERROR")
            return

        arg_fmt = argparse.RawDescriptionHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                         description=main.__doc__)
        parser.add_argument(
            '-l', '--limb', choices=valid_limbs, default=valid_limbs[0],
            help='send joint trajectory to which limb'
        )

        args = parser.parse_args(rospy.myargv()[1:])
        limb = args.limb

        print("Initializing node... ")
        rospy.init_node("motion_planner".format(limb))
        #print("Getting robot state... ")
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        #print("Enabling robot... ")
        rs.enable()
        print("Running. Ctrl-c to quit")   
        limb_interface = intera_interface.Limb(limb)

        return limb, limb_interface

    def get_parameters_for_IK(self):
        Slist = np.array([[0, 0,  1,      0,     0,       0],
                          [0, 1,  0, -0.317,     0,   0.081],
                          [1, 0,  0,      0, 0.317, -0.1925],
                          [0, 1,  0, -0.317,     0,   0.481],
                          [1, 0,  0,      0, 0.317,  -0.024],
                          [0, 1,  0, -0.317,     0,   0.881],
                          [1, 0,  0,      0, 0.317, -0.1603]]).T
        M = np.array([[0, 0, 1, 1.01475], [-1, 0, 0, 0.1603], [0, -1, 0, 0.317], [0, 0, 0, 1]])
        # This is the configuration of the center point.

        eomg = 0.01
        ev = 0.001
        return Slist,M,eomg,ev
        
    def view_trajectory(self,limb,limb_interface):
        view_traj = Trajectory(limb, limb_interface.joint_names())
        rospy.on_shutdown(view_traj.stop)
        # get Current Joint Positions first
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        view_traj.add_point(current_angles, 0.0)   
        #joint-angles when sawyer at view point
        thetalist0 = [0, -np.pi / 2.0, 0, np.pi / 2, 0, np.pi / 2, 0]
        n_sec = 10.0
        view_traj.add_point(thetalist0, n_sec)
        view_traj.start()
        view_traj.wait(view_traj.duration)
        #now we have moved to the view point
        #publish a message that robot have arrived at view point
        overview_point = Point()
        overview_point.z = 1.0
        point_pub = rospy.Publisher('current_position', Point, queue_size = 1)
        point_pub.publish(overview_point)



    def view_target_trajectory(self,limb,limb_interface):
        view_target_traj=Trajectory(limb, limb_interface.joint_names())
        rospy.on_shutdown(view_target_traj.stop)
        Slist, M, eomg, ev = self.get_parameters_for_IK()
        T = np.array([[ 0, -1,  0,  0.575],
                      [-1,  0,  0, 0.1603],
                      [ 0,  0, -1,  0.317],
                      [ 0,  0,  0,      1]])

        # Wait for the target_position   
        #target = rospy.wait_for_message('/me495/location', Point)
        target = Point()
        target.x = 0.575
        target.y = 0.1603
        target.z = 0.0
        #default step: add current__angles at beginning of every trajectory
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        view_target_traj.add_point(current_angles, 0.0)
        #set the target point
        T[0][3] = target.x
        T[1][3] = target.y
        T[2][3] = target.z + 0.1
        #solve the IK
        thetalist0, success = mr.IKinSpace(Slist, M, T, current_angles, eomg, ev)
        n_sec = 10.0
        view_target_traj.add_point(thetalist0, n_sec)
        view_target_traj.start()
        view_target_traj.wait(view_target_traj.duration)



    def plot_circle_trajectory(self,limb,limb_interface):
        plot_traj=Trajectory(limb, limb_interface.joint_names())
        rospy.on_shutdown(plot_traj.stop)
        print("here we are circle_plot_trajectory")
        Slist,M,eomg,ev = self.get_parameters_for_IK()
        N = 10
        dt = 2 * np.pi / N
        x = 0.575
        y = 0.1603
        r = 0.1
        n_sec = 5.0
        T = np.array([[ 0, -1,  0,  0.575],
                      [-1,  0,  0, 0.1603],
                      [ 0,  0, -1,  0.1],
                      [ 0,  0,  0,      1]])

        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        thetalist0=current_angles
        plot_traj.add_point(current_angles, 0.0)
        # Suppose we have a non-continuous trajectory as a list
        plot_points = []
        i = 0   
        while i < 2 * N:
            plot_points.append([x + r * np.cos(i * dt), y + r * np.sin(i * dt)])
            i+=1
       
        print("1")  
        i=0  

        while i < len(plot_points):
            #print(thetalist0)
            T[0][3] = plot_points[i][0]
            T[1][3] = plot_points[i][1]
            thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
            plot_traj.add_point(thetalist0, n_sec)
            n_sec = 0.5        
            i += 1
            print("adding traj point")

        plot_traj.start()
        print("have start the traj")
        plot_traj.wait(plot_traj.duration)


    def plot_face_trajectory(self,limb, limb_interface):
        Slist, M, eomg, ev = self.get_parameters_for_IK()
        N = 10
        dt = 2 * np.pi / N
        x = 0.575
        y = 0.1603
        r = 0.1
        n_sec = 10.0
        z_touch=0.117
        z_hang=0.317
        T = np.array([[0, -1, 0, 0.575],
                      [-1, 0, 0, 0.1603],
                      [0, 0, -1, 0.317],
                      [0, 0, 0, 1]])

        #Wait for the trajectory list
        #plot_points=rospy.wait_for_message('/me495/trajectory',unit8MultiArray)
        #plot_points = [(0.575,0.1603),(0.575,0.2603),(-1,-1),(0.575,0.1603)]
        #Once all set, ask controller whether to start
        # start_pub = rospy.Publisher('/me495/result', String, queue_size = 1)
        # start_pub.publish("All Set")
        #Receive the start flag
        # start_flag=rospy.wait_for_message('/me495/command/',String)
        plot_points=self.plot_points

        i=0
        marker_hanup=True
        line_traj = Trajectory(limb, limb_interface.joint_names())
        rospy.on_shutdown(line_traj.stop)

        #default step, add current_angles as initial thetalist0
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        thetalist0=current_angles
        line_traj.add_point(current_angles, 0.0)
        #start drawing position
        thetalist0 = [0, -np.pi / 4.0, 0, np.pi / 2, 0, np.pi / 4, 0]
        line_traj.add_point(current_angles, n_sec)

        while i < len(plot_points):
            if marker_hanup==True:
                #move right above the start point
                T[0][3] = plot_points[i][0]*0.005 + 0.575 -0.25
                T[1][3] = plot_points[i][1]*0.005 +0.1603 -0.25
                thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
                line_traj.add_point(thetalist0, n_sec)
                #drop down marker
                T[2][3] = z_touch
                thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
                line_traj.add_point(thetalist0, n_sec)

                marker_hanup=False
                i+=1
            else:
                if plot_points[i][0]==-1 and plot_points[i][1]==-1:
                    # lift up the marker
                    T[2][3] = z_hang
                    thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
                    line_traj.add_point(thetalist0, n_sec)
                    marker_hanup=True
                else:
                    T[0][3] = plot_points[i][0]*0.005 +0.575 -0.25
                    T[1][3] = plot_points[i][1]*0.005 +0.1603 -0.25
                    thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
                    line_traj.add_point(thetalist0, n_sec)
                i+=1

        #start to draw
        line_traj.start()
        line_traj.wait(line_traj.duration)



def main():
    _drawing_control=drawing_control()
    #first enable the robot
    limb, limb_interface=_drawing_control.enable_robot()
    #define the view_traj which move robot from current joint_positions to view position
    _drawing_control.view_trajectory(limb,limb_interface)
    #define the traj from view point to target point
    _drawing_control.view_target_trajectory(limb,limb_interface)
    #define the test circle trajectory
    #plot_circle_trajectory(limb,limb_interface)
    #define the face trajectory
    _drawing_control.plot_face_trajectory(limb,limb_interface)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")