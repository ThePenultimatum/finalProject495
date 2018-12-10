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

#TODO: Know more about torso.

'''
This is the main limb motion node for the whole project
'''
import intera_interface
# This line may be not necessary (2)
import argparse
import rospy
import actionlib
import sys
from copy import copy
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import JointTrajectoryPoint

class Trajectory(object):
    # Here I do not know which of these are really useful (3)
    def __init__(self, limb, joint_names):
        self._joint_names = joint_names
        ns = 'robot/limb/' + limb + '/'
        # TODO: These 5 lines seem important
        self._client = actionlib.SimpleActionClient(ns + "follow_joint_trajectory", FollowJointTrajectoryAction,)
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout = rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)
        # Here I add a duration variable
        self.duration = 0.0

    # This is necessary
    def stop(self):
        self._client.cancel_goal()
    
    # TODO: Long trajectory should go through post-process before running
    def add_point(self, positions, dt):
        self.duration += dt
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(self.duration)        
        # Here we add velocities and accelerations 
        # v
        if len(self._goal.trajectory.points) > 1:
            point.velocities = 1.0 * (positions - self._goal.trajectory.points[-1].positions) / dt
        else:
            point.velocities = [0, 0, 0, 0, 0, 0, 0]
        # a
        if len(self._goal.trajectory.points) > 2:
            point.accelerations = 1.0 * (point.velocities - self._goal.trajectory.points[-1].velocities) / dt
        else:
            point.accelerations = [0, 0, 0, 0, 0, 0, 0]
        # Code ends       
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)  

    def stop(self):
        self._client.cancel_goal()  

    def wait(self, timeout = 15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    # TODO: May be very useful
    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = self._joint_names   

def main():
    rp = intera_interface.RobotParams()
    # These lines may be not necessary (1)
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
          "Exiting."), "ERROR")
        return   

    # This lines may be replaced by rospy.get_param('') (2)
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class = arg_fmt, description=main.__doc__)
    parser.add_argument('-l', '--limb', choices = valid_limbs, default = valid_limbs[0], help = 'send joint trajectory to which limb')
    args = parser.parse_args(rospy.myargv()[1:])
    # TODO: Replace the variable anywhere by its value if possible
    limb = args.limb
    
    # Create the node, format may be not necessary
    rospy.init_node("motion_planner".format(limb))     
    # Enable the robot
    rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    rs.enable()

    # This is necessary
    limb_interface = intera_interface.Limb(limb)
    image_traj = Trajectory(limb, limb_interface.joint_names())
    rospy.on_shutdown(image_traj.stop)

    # Here we need to move the head to face to people, showing the image in the display and wait for user capturing
    # Here we wait for a topic   
    # TODO: Move the head, write a subscriber and its callback
    
    # Now suppose we have trajectory now as a list of x, y lists


    # Parameters for IK solver
    # This is for the end-effector
    # TODO: x, y, z of the last link may be changed
    Slist = np.array([[0, 0,  1,      0,     0,       0],
                      [0, 1,  0, -0.317,     0,   0.081],
                      [1, 0,  0,      0, 0.317, -0.1925],
                      [0, 1,  0, -0.317,     0,   0.481],
                      [1, 0,  0,      0, 0.317,  -0.024],
                      [0, 1,  0, -0.317,     0,   0.881],
                      [1, 0,  0,      0, 0.317, -0.1603]]).T
    M = np.array([[0, 0, 1, 1.01475], [-1, 0, 0, 0.1603], [0, -1, 0, 0.317], [0, 0, 0, 1]])
    eomg = 0.01
    ev = 0.001    
    
    # Go to the overview spot
    move_to_traj = Trajectory(limb, limb_interface.joint_names())
    rospy.on_shutdown(move_to_traj.stop)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    move_to_traj.add_point(current_angles, 0.0)
    thetalist0 = [0, -np.pi / 2.0, 0, np.pi / 4, 0, np.pi / 4, 0]
    move_to_traj.add_point(thetalist0, 10.0)
    move_to_traj.start()
    move_to_traj.wait(move_to_traj.duration)
    
    # Go through above every tag
    # The first one
    # TODO: This can be a function actually
    move_to_traj = Trajectory(limb, limb_interface.joint_names())
    rospy.on_shutdown(move_to_traj.stop)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    move_to_traj.add_point(current_angles, 0.0)
    T = np.array([[1, 0, 0, x], 
		          [0, 1, 0, y], 
		          [0, 0, 1, z], 
		          [0, 0, 0, 1]])    
    # Not sure if IK works 
    thetalist0, success = mr.IKinSpace(Slist, M, T, current_angles, eomg, ev)
    move_to_traj.add_point(thetalist0, 5.0)
    move_to_traj.start()
    move_to_traj.wait(move_to_traj.duration)
    # Three more tag motions should be added

    # Go to above the center
    # Same as above

    # Plan the motion
    plot_traj = Trajectory(limb, limb_interface.joint_names())
    rospy.on_shutdown(plot_traj.stop)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    plot_traj.add_point(current_angles, 0.0)
    # TODO: The code below is not correct!!
    while i < 2 * N:
        # Set the time step based on the linear distance
        n_sec += 0.5 
        T[0][3] = x + r * np.cos(i * dt)
        T[1][3] = y + r * np.sin(i * dt)
        thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
        plot_traj.add_point(thetalist0, n_sec)              
        i += 1    
 
    # Start drawing
    plot_traj.start()
    plot_traj.wait(plot_traj.duration)    

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")
