#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Intera SDK Joint Trajectory Action Client Example
"""
import argparse
import sys

from copy import copy

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import intera_interface

from intera_interface import CHECK_VERSION

import modern_robotics as mr
import numpy as np

class Trajectory(object):
    def __init__(self, limb, joint_names):
        self._joint_names = joint_names
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = self._joint_names


def main():
    """SDK Joint Trajectory Example: Simple Action Client

    Creates a client of the Joint Trajectory Action Server
    to send commands of standard action type,
    control_msgs/FollowJointTrajectoryAction.

    Make sure to start the joint_trajectory_action_server.py
    first. Then run this example on a specified limb to
    command a short series of trajectory points for the arm
    to follow.
    """
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
    rospy.init_node("sdk_joint_trajectory_client_{0}".format(limb))
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    limb_interface = intera_interface.Limb(limb)
    traj = Trajectory(limb, limb_interface.joint_names())
    rospy.on_shutdown(traj.stop)
    # Command Current Joint Positions first
    limb_interface.move_to_neutral()
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    #traj.add_point(current_angles, 0.0)

    p1 = current_angles
    n_sec = 1.0

    mylimb = intera_interface.Limb("right")
    waypoints = mylimb.joint_angles()
    thetalist0 = [0.00722578, -1.13799861, -0.01079448, 1.7510739, 0.00573321, 0.95772104, -0.00563395]
    thetalist_x= [thetalist0[i] for i in range(7)]
    print(thetalist_x)
    # waypoints['right_j0'] = 0.00722578
    # waypoints['right_j1'] = -1.13799861
    # waypoints['right_j2'] = -0.01079448
    # waypoints['right_j3'] = 1.7510739
    # waypoints['right_j4'] = 0.00573321
    # waypoints['right_j5'] = 0.95772104
    # waypoints['right_j6'] = -0.00563395

    traj.add_point(thetalist0, n_sec)
    #neutral_pose=[0.0, -1.18, 0.0, 2.18, 0.0, 0.57, 3.3161]
    #traj.add_point(neutral_pose, n_sec)

    #set all arguments that solving ik need
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
    N = 15
    dt = 2 * np.pi / N
    r = 0.225
    n_sec += 3.0
    while i < N:
        T[0][3] = x + r * np.cos(i * dt)
        T[1][3] = y + r * np.sin(i * dt)
        thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
        #waypoints=[thetalist0[j] for j in range(7)]
        # waypoints['right_j0'] = thetalist0[0]
        # waypoints['right_j1'] = thetalist0[1]
        # waypoints['right_j2'] = thetalist0[2]
        # waypoints['right_j3'] = thetalist0[3]
        # waypoints['right_j4'] = thetalist0[4]
        # waypoints['right_j5'] = thetalist0[5]
        # waypoints['right_j6'] = thetalist0[6] 
        thetalist_x[6] =   thetalist0[6]    
        print(thetalist_x)
        traj.add_point(thetalist_x, n_sec)
        n_sec += 3.0
        i = i + 1


    traj.start()
    traj.wait(n_sec)
    print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    main()
