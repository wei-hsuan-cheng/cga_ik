#!/usr/bin/env python3

from visual_kinematics.RobotSerial import *
import numpy as np
from math import pi

mm2m = 10 ** (-3) # [mm] to [m]

def main():
    np.set_printoptions(precision=3, suppress=True)

    # Define Craig version of D-H parameters
    
    # Reference robot: TM5-700, unit: [mm], see details of the D-H parameters at "../TM5-700_TM5-900_DH_para.pdf"
    # alpha{i-1}, a_{i-1}, d{i}, theta_offset{i}
    alpha0 =       0; a0 =            0; d1 =  145.2 * mm2m; theta_offset1 = 0;
    alpha1 = -pi / 2; a1 =            0; d2 =             0; theta_offset2 = 0;
    alpha2 =       0; a2 = 329.0 * mm2m; d3 =             0; theta_offset3 = 0;
    alpha3 =       0; a3 = 311.5 * mm2m; d4 = -122.3 * mm2m; theta_offset4 = 0;
    alpha4 = -pi / 2; a4 =            0; d5 =    106 * mm2m; theta_offset5 = 0;
    alpha5 = -pi / 2; a5 =            0; d6 = 113.15 * mm2m; theta_offset6 = 0;
    alpha6 =       0; a6 =            0;
    
    # # Reference robot: TM5-900, unit: [mm], see details of the D-H parameters at "../TM5-700_TM5-900_DH_para.jpeg"
    # # alpha{i-1}, a_{i-1}, d{i}, theta_offset{i}
    # alpha0 =       0; a0 =            0; d1 =  165.2 * mm2m; theta_offset1 = 0;
    # alpha1 = -pi / 2; a1 =            0; d2 =             0; theta_offset2 = 0;
    # alpha2 =       0; a2 = 536.1 * mm2m; d3 =             0; theta_offset3 = 0;
    # alpha3 =       0; a3 = 457.9 * mm2m; d4 = -156.3 * mm2m; theta_offset4 = 0;
    # alpha4 = -pi / 2; a4 =            0; d5 =    106 * mm2m; theta_offset5 = 0;
    # alpha5 = -pi / 2; a5 =            0; d6 = 113.15 * mm2m; theta_offset6 = 0;
    # alpha6 =       0; a6 =            0;

    # Straight pose
    theta_offset1 =       0 
    theta_offset2 = -pi / 2
    theta_offset3 =       0 
    theta_offset4 = -pi / 2
    theta_offset5 =       0 
    theta_offset6 =       0

    # # IK starting pose 
    # theta_offset1 = 0 
    # theta_offset2 = -pi / 2 
    # theta_offset3 = pi / 2 
    # theta_offset4 = -pi / 2 
    # theta_offset5 = pi / 2 
    # theta_offset6 = 0

    # Adapt into the D-H parameters in this package
    dh_params = np.array([[d1, a1, alpha1, theta_offset1],  
                          [d2, a2, alpha2, theta_offset2],  
                          [d3, a3, alpha3, theta_offset3],  
                          [d4, a4, alpha4, theta_offset4],  
                          [d5, a5, alpha5, theta_offset5],  
                          [d6, a6, alpha6, theta_offset6]]) 
    
    
    # # Reference robot: AUBO-i10 serial robot, see details of the D-H parameters at "../AUBO-i10_DH_para.jpeg"
    # dh_params = np.array([[0.163 , 0.    , 0.5 * pi , 0.       ],  # d1, a1, alpha1, theta1
    #                       [0.    , 0.632 , pi       , 0.5 * pi ],  # d2, a2, alpha2, theta2
    #                       [0.    , 0.6005, pi       , 0.       ],  # d3, a3, alpha3, theta3
    #                       [0.2013, 0.    , -0.5 * pi, -0.5 * pi],  # d4, a4, alpha4, theta4
    #                       [0.1025, 0.    , 0.5 * pi , 0.       ],  # d5, a5, alpha5, theta5
    #                       [0.094 , 0.    , 0.       , 0.       ]]) # d6, a6, alpha6, theta6
    
    robot = RobotSerial(dh_params, dh_type = "normal") # "normal" or "modified" D-H parameters
    # TBD: idk why when i try to run TM5-700 and TM5-900, the modified and normal D-H parameters seems to be messed up.

    # =====================================
    # forward
    # =====================================

    theta = np.array([0, 0, 0, 0, 0, 0])
    f = robot.forward(theta)

    print("-------forward-------")
    print("end frame t_4_4:")
    print(f.t_4_4)
    print("end frame xyz:")
    print(f.t_3_1.reshape([3, ]))
    print("end frame abc:")
    print(f.euler_3)
    print("end frame rotational matrix:")
    print(f.r_3_3)
    print("end frame quaternion:")
    print(f.q_4)
    print("end frame angle-axis:")
    print(f.r_3)

    robot.show()


if __name__ == "__main__":
    main()
