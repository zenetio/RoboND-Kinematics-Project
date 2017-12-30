#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
import pickle as pickle
import numpy as np
from My_TMatrix import My_TMatrix


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Define symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        r, p, y = symbols('r p y ')

        # Define a dictionary
        s = {alpha0: 0, a0: 0, d1: 0.75,
             alpha1: -np.pi / 2., a1: 0.35, d2: 0, q2: q2 - np.pi / 2.,
             alpha2: 0, a2: 1.25, d3: 0,
             alpha3: -np.pi / 2., a3: -.054, d4: 1.50,
             alpha4: np.pi / 2., a4: 0, d5: 0,
             alpha5: -np.pi / 2., a5: 0, d6: 0,
             alpha6: 0, a6: 0, d7: 0.303, q7: 0}

        oClass = None  # variable to store the serialized transformation matrix

        # Load serialized transformation matrix or initialize if it
        # doesn't exist in disk
        try:
            with open('ik.pkl', 'rb') as f:
                oClass = pickle.load(f)
        except OSError as err:
            print("Ops! {0}".format(err))
        except EOFError as err:
            print("Ops! {0}".format(err))
        except:
            print("Unexpected error:", sys.exc_info()[0])

        if oClass is None:
            try:
                oClass = My_TMatrix(s)
            except:
                print("Unexpected error creating object:", sys.exc_info()[0])
            # save to disk
            try:
                with open('ik.pkl', 'wb') as f:
                    pickle.dump(oClass, f)
            except ValueError:
                print("Could not convert")
            except:
                print("Unexpected error:", sys.exc_info()[0])

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            # roll, pitch, yaw = end-effector orientation
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            # define the joint angles
            theta1 = 0
            theta2 = 0
            theta3 = 0
            theta4 = 0
            theta5 = 0
            theta6 = 0

            # define and set end-effector variable
            EE = oClass.EE.evalf(subs={'Px': px, 'Py': py, 'Pz': pz})
            # calculate the rotational matrix
            Rot_G = oClass.R_G.evalf(subs={r: roll, p: pitch, y: yaw})
            # define and set the wrist variable
            Wc = EE - (0.303 * Rot_G[:, 2])

            # Calculate joint angles using geometric IK method
            # calculate theta1
            theta1 = atan2(Wc[1], Wc[0])

            # SSS triangle for theta2 and theta3
            # using geometry to calculate the triangle sides
            side_a = 1.501
            side_b = sqrt(pow((sqrt(Wc[0] * Wc[0] + Wc[1] * Wc[1]) - 0.35), 2) + pow((Wc[2] - 0.75), 2))
            side_c = 1.25

            # using geometry to calculate the triangle angles
            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

            # calculate theta2
            theta2 = np.pi / 2. - angle_a - atan2(Wc[2] - 0.75, sqrt(Wc[0] * Wc[0] + Wc[1] * Wc[1]) - 0.35)
            # calculate theta3
            theta3 = np.pi / 2. - (angle_b + 0.036)

            # Using rotational matrix to calculate remaining joint angles
            # from transformation matrix we can extract the rotational matrix
            R0_3 = oClass.T0_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.T * Rot_G

            # Now get Euler angles from rotation matrix
            # calculate theta4
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            # calculate theta5
            theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
            # calculate theta6
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():

    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.loginfo("Ready to receive an IK request")
    rospy.spin()


if __name__ == "__main__":
    IK_server()
