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

def enable_robot():
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

def get_parameters_for_IK():
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
    
def view_trajectory(limb,limb_interface):
    view_traj = Trajectory(limb, limb_interface.joint_names())
    rospy.on_shutdown(view_traj.stop)
    # get Current Joint Positions first
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    view_traj.add_point(current_angles, 0.0)   
    #joint-angles when sawyer at view point
    thetalist0 = [0, -np.pi / 2.0, 0, np.pi / 4, 0, np.pi / 4, 0]
    n_sec = 5.0
    view_traj.add_point(thetalist0, n_sec)
    view_traj.start()
    view_traj.wait(view_traj.duration)
    #now we have moved to the view point
    #publish a message that robot have arrived at view point
    overview_point = Point()
    overview_point.z = 1.0
    point_pub = rospy.Publisher('current_position', Point, queue_size = 1)
    point_pub.publish(overview_point)



def view_target_trajectory(limb,limb_interface):
    view_target_traj=Trajectory(limb, limb_interface.joint_names())
    rospy.on_shutdown(view_target_traj.stop)
    Slist, M, eomg, ev = get_parameters_for_IK()
    T = np.array([[ 0, -1,  0,  0.575],
                  [-1,  0,  0, 0.1603],
                  [ 0,  0, -1,  0.317],
                  [ 0,  0,  0,      1]])
    rospy.Subscriber('target_position', Point, view_target_traj.target_callback) #16
    # Wait for the target_position
    # These are for test
    view_target_traj.if_target = True
    view_target_traj.target.x = 0.575
    view_target_traj.target.y = 0.1603
    while view_target_traj.if_target == False:
        pass
    #default step: add current__angles at beginning of every trajectory
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    view_target_traj.add_point(current_angles, 0.0)
    #set the target point
    T[0][3] = view_target_traj.target.x
    T[1][3] = view_target_traj.target.y
    T[2][3] = view_target_traj.target.z + 0.1
    #solve the IK
    thetalist0, success = mr.IKinSpace(Slist, M, T, current_angles, eomg, ev)
    n_sec = 10.0
    view_target_traj.add_point(thetalist0, n_sec)
    view_target_traj.start()
    view_target_traj.wait(view_target_traj.duration)



def plot_circle_trajectory(limb,limb_interface):
    plot_traj=Trajectory(limb, limb_interface.joint_names())
    rospy.on_shutdown(plot_traj.stop)

    Slist,M,eomg,ev = get_parameters_for_IK()
    N = 10
    dt = 2 * np.pi / N
    x = 0.575
    y = 0.1603
    r = 0.1
    n_sec = 5.0
    T = np.array([[ 0, -1,  0,  0.575],
                  [-1,  0,  0, 0.1603],
                  [ 0,  0, -1,  0.317],
                  [ 0,  0,  0,      1]])


    # Suppose we have a non-continuous trajectory as a list
    plot_points = []
    i = 0   
    while i < 2 * N:
        plot_points.append([x + r * np.cos(i * dt), y + r * np.sin(i * dt)])
    i = 0
    while i < 2 * N:
        plot_points.append([x - r * np.cos(i * dt), y + r * np.sin(i * dt)])    
    
    n_sec = 5.0
    while i < len(plot_points):
        #print(thetalist0)
        T[0][3] = plot_points[i][0]
        T[1][3] = plot_points[i][1]
        thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
        plot_traj.add_point(thetalist0, n_sec)
        n_sec = 0.5        
        i += 1
    print('1')
    plot_traj.start()
    print('2')
    plot_traj.wait(plot_traj.duration)
    print('3')  
    print('4')


def plot_face_trajectory(limb, limb_interface):
    Slist, M, eomg, ev = get_parameters_for_IK()
    N = 10
    dt = 2 * np.pi / N
    x = 0.575
    y = 0.1603
    r = 0.1
    n_sec = 5.0
    z_touch=-0.1
    z_hang=0.1
    T = np.array([[0, -1, 0, 0.575],
                  [-1, 0, 0, 0.1603],
                  [0, 0, -1, 0.317],
                  [0, 0, 0, 1]])

    # Suppose we have a non-continuous trajectory as a list
    #plot_points = [[(0,0),(0,1),(1,1)],[(2,1),(3,1)]]
    plot_points = [(0,0),(0,1)(-1,-1),(1,1)]
    i=0
    marker_hanup=True
    line_traj = Trajectory(limb, limb_interface.joint_names())
    rospy.on_shutdown(line_traj.stop)
    while i < len(plot_points):
        if marker_hanup==True:
            #move right above the start point
            T[0][3] = plot_points[i][0]
            T[1][3] = plot_points[i][1]
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
                T[0][3] = plot_points[i][0]
                T[1][3] = plot_points[i][1]
                thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
                line_traj.add_point(thetalist0, n_sec)
            i+=1

    #start to draw
    line_traj.start()
    line_traj.wait(line_traj.duration)



def main():
    #first enable the robot
    limb, limb_interface=enable_robot()
    #define the view_traj which move robot from current joint_positions to view position
    view_trajectory(limb,limb_interface)
    #define the traj from view point to target point
    view_target_trajectory(limb,limb_interface)
    #define the test circle trajectory
    plot_circle_trajectory(limb,limb_interface)
    #define the face trajectory
    plot_face_trajectory(limb,limb_interface)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")