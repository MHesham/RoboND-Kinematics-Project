#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from kr210_solver import *
from time import time

# Use KR210 solver factory method to instantiate a single IK solver to use
# throught the IK_server service lifetime.
solver = Kr210SolverFactory.create("Kr210Solver")

# If set to True, the output of the IK step will be used to compute FK and the
# the error will be computed and logged to console, otherwise set to False.
debug_IK = False

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Initialize service response
        joint_trajectory_list = []
        ee_offset_list = []
        solve_times_list = []
        start_time = time()
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            solve_start_time = time()

            [theta1, theta2, theta3, theta4, theta5, theta6], WC_0 = solver.solve_IK(
                px, py, pz, roll, pitch, yaw)

            if debug_IK == True:
                # Calcualte EE position offset error from FK by comparing against
                # the requested position
                fk_px, fk_py, fk_pz = solver.solve_FK(
                    theta1, theta2, theta3, theta4, theta5, theta6)

            # Record start and end times of doing IK followed by FK to measure
            # performance
            solve_times_list.append(time() - solve_start_time)

            if debug_IK == True:            
                # Compute error between FK and IK and append to list
                ee_x_e = abs(fk_px - px)
                ee_y_e = abs(fk_py - py)
                ee_z_e = abs(fk_pz - pz)
                ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
                ee_offset_list.append(ee_offset)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [
                theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s, total time: %04.4f",
                      len(joint_trajectory_list), time() - start_time)
        rospy.loginfo("Solve time avg %04.8fs min %04.8fs max %04.8fs", min(
            solve_times_list), max(solve_times_list), sum(solve_times_list) / len(solve_times_list))

        if debug_IK == True:
            rospy.loginfo("EE offset: avg %04.8f min %04.8f max %04.8f units",
                        min(ee_offset_list), max(ee_offset_list), sum(ee_offset_list) / len(ee_offset_list))

        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
