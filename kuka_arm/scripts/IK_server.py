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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        # link offsets
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')

        # link lengths
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

        # twist angles
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    
        # joint angles
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

    
        # Create Modified DH parameters
        # substitution table
        DH_Parameters = { alpha0:     0, a0:      0, d1:  0.75, q1:          q1,
                          alpha1: -pi/2, a1:   0.35, d2:     0, q2: -pi/2. + q2, 
                          alpha2:     0, a2:   1.25, d3:     0, q3:          q3,
                          alpha3: -pi/2, a3: -0.054, d4:   1.5, q4:          q4,
                          alpha4:  pi/2, a4:      0, d5:     0, q5:          q5,
                          alpha5: -pi/2, a5:      0, d6:     0, q6:          q6,
                          alpha6:     0, a6:      0, d7: 0.303, q7:           0}
        
        # Define Modified DH Transformation matrix
        def TF_compute(alpha, a, d, q):
            TF = Matrix([[cos(q),           -sin(q),             0,           a],
                    	 [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    	 [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
                    	 [0, 				 0, 				 0, 		  1]])
            return TF

        # Apply transformations
        T01 = TF_compute(alpha0, a0, d1, q1).subs(DH_Parameters)
        T12 = TF_compute(alpha1, a1, d2, q2).subs(DH_Parameters)
        T23 = TF_compute(alpha2, a2, d3, q3).subs(DH_Parameters)
        T34 = TF_compute(alpha3, a3, d4, q4).subs(DH_Parameters)
        T45 = TF_compute(alpha4, a4, d5, q5).subs(DH_Parameters)
        T56 = TF_compute(alpha5, a5, d6, q6).subs(DH_Parameters)
        T67 = TF_compute(alpha6, a6, d7, q7).subs(DH_Parameters)
    
        T_overall = T01*T12*T23*T34*T45*T56*T67;
    
        ###

        # Initialize service response
        joint_trajectory_list = []
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

            ### Your IK code here
            # Wrist center calculation
            # Rotation matrix from end effector to base
            r = symbols('r') 
            p = symbols('p')
            y = symbols('y')

            Rr = Matrix([[1, 0, 0],
                         [0, cos(r), -sin(r)],
                         [0, sin(r), cos(r)]])
            Rp = Matrix([[cos(p), 0, sin(p)],
                         [0, 1, 0],
                         [-sin(p), 0, cos(p)]])
            Ry = Matrix([[cos(y), -sin(y), 0],
                         [sin(y), cos(y), 0],
                         [0, 0, 1]])

            # End effector transformation from DH convention to URDF convention
            RCorr = Ry.subs(y, radians(180))*Rp.subs(p, radians(-90))

            R_EE = Ry*Rp*Rr*RCorr;

            R_EE_num = R_EE.subs({'r': roll, 'p':pitch, 'y': yaw});

            # From urdf
            LinkLength = 0.303
            
            EE = Matrix([[px],
                         [py],
                         [pz]])

            WC = EE - LinkLength*R_EE_num[:,2]

            # From a top down view of the robot
            theta1 = atan2(WC[1],WC[0]);

            # Using law of cosines
            r = sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35
            sa = 1.501;
            sb = sqrt(r*r + (WC[2]-0.75)*(WC[2]-0.75))
            sc = 1.25;

            aa = acos((sb*sb + sc*sc - sa*sa)/(2*sb*sc))
            theta2 = pi/2 - aa - atan2(WC[2]-0.75, r)
    
            ab = acos((sa*sa + sc*sc - sb*sb)/(2*sa*sc))
            theta3 = pi/2 - (ab + 0.036);
    
            R03 = T01[0:3,0:3]*T12[0:3,0:3]*T23[0:3,0:3];
            R03_num = R03.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            R36 = R03_num.T*R_EE_num

            theta4 = atan2(R36[2,2], -R36[0,2])
            theta5 = atan2(sqrt(R36[0,2]*R36[0,2] + R36[2,2]*R36[2,2]),R36[1,2])
            theta6 = atan2(-R36[1,1],R36[1,0])
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
    rospy.spin()

if __name__ == "__main__":
    IK_server()
