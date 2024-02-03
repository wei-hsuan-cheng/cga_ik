#!/usr/bin/env python3

from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotTrajectory import *
import numpy as np
from math import pi

mm2m = 10 ** (-3) # [mm] to [m]

def main():
    np.set_printoptions(precision=3, suppress=True)

    # # Define Craig version of D-H parameters
    # # Reference robot: TM5-900, unit: [mm], , see details of the D-H parameters at "../TM5-900_DH_para.jpeg"
    # # alpha{i-1}, a_{i-1}, d{i}, theta_offset{i}
    # alpha0 =       0; a0 =            0; d1 =  165.2 * mm2m; theta_offset1 = 0;
    # alpha1 = -pi / 2; a1 =            0; d2 =             0; theta_offset2 = 0;
    # alpha2 =       0; a2 = 536.1 * mm2m; d3 =             0; theta_offset3 = 0;
    # alpha3 =       0; a3 = 457.9 * mm2m; d4 = -156.3 * mm2m; theta_offset4 = 0;
    # alpha4 = -pi / 2; a4 =            0; d5 =    106 * mm2m; theta_offset5 = 0;
    # alpha5 = -pi / 2; a5 =            0; d6 = 113.15 * mm2m; theta_offset6 = 0;
    # alpha6 =       0; a6 =            0;

    # # IK starting pose 
    # theta_offset1 = 0; theta_offset2 = -pi / 2; theta_offset3 = pi / 2; theta_offset4 = -pi / 2; theta_offset5 = pi / 2; theta_offset6 = 0;

    # # Adapt into the D-H parameters in this package
    # dh_params = np.array([[d1, a1, alpha1, theta_offset1],  
    #                       [d2, a2, alpha2, theta_offset2],  
    #                       [d3, a3, alpha3, theta_offset3],  
    #                       [d4, a4, alpha4, theta_offset4],  
    #                       [d5, a5, alpha5, theta_offset5],  
    #                       [d6, a6, alpha6, theta_offset6]]) 
                          
    # Reference robot: AUBO-i10 serial robot, see details of the D-H parameters at "../AUBO-i10_DH_para.jpeg"
    dh_params = np.array([[0.163 , 0.    , 0.5 * pi , 0.       ],  # d1, a1, alpha1, theta1
                          [0.    , 0.632 , pi       , 0.5 * pi ],  # d2, a2, alpha2, theta2
                          [0.    , 0.6005, pi       , 0.       ],  # d3, a3, alpha3, theta3
                          [0.2013, 0.    , -0.5 * pi, -0.5 * pi],  # d4, a4, alpha4, theta4
                          [0.1025, 0.    , 0.5 * pi , 0.       ],  # d5, a5, alpha5, theta5
                          [0.094 , 0.    , 0.       , 0.       ]]) # d6, a6, alpha6, theta6

    robot = RobotSerial(dh_params)

    # =====================================
    # trajectory
    # =====================================

    # frames = [Frame.from_euler_3(np.array([0.5 * pi, 0., pi]), np.array([[0.28127], [0.], [1.13182]])), # orientation, position
    #           Frame.from_euler_3(np.array([0.25 * pi, 0., 0.75 * pi]), np.array([[0.48127], [0.], [1.13182]])),
    #           Frame.from_euler_3(np.array([0.5 * pi, 0., pi]), np.array([[0.48127], [0.], [0.63182]]))]
    
    frames = [Frame.from_euler_3(np.array([0 * pi, 0., 0 * pi]), np.array([[0.48127], [0.], [0.63182]])), # orientation, position
              Frame.from_euler_3(np.array([0 * pi, 0., 0 * pi]), np.array([[0.48127], [0.1], [0.63182 + .2]])),
              Frame.from_euler_3(np.array([0 * pi, 0., 0 * pi]), np.array([[0.48127], [0.2], [0.63182 + .2]]))]
    
    time_points = np.array([0., 6., 10.])

    trajectory = RobotTrajectory(robot, frames, time_points)

    trajectory.show(motion="lin")
    # trajectory.show(motion = "p2p")


if __name__ == "__main__":
    main()
