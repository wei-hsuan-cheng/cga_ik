from TM_kinematics.RobotSerial import *
import numpy as np
from math import pi

## Unit translform
r2d = 180 / pi
d2r = 1 / r2d
mm2m = 10 ** (-3) # [mm] to [m]
m2mm = 1 / mm2m

def main():
    np.set_printoptions(precision=3, suppress=True)
    # Define Craig version of D-H parameters
    # Reference robot: TM5-700, unit: [mm]
    # alpha{i-1}, a_{i-1}, d{i}, theta_offset{i}
    alpha0 =       0; a0 =            0; d1 =  145.2 * mm2m; theta_offset1 = 0
    alpha1 = -pi / 2; a1 =            0; d2 =             0; theta_offset2 = 0 - pi / 2 
    alpha2 =       0; a2 = 329.0 * mm2m; d3 =             0; theta_offset3 = 0
    alpha3 =       0; a3 = 311.5 * mm2m; d4 = -122.3 * mm2m; theta_offset4 = 0 - pi / 2 
    alpha4 = -pi / 2; a4 =            0; d5 =    106 * mm2m; theta_offset5 = 0
    alpha5 = -pi / 2; a5 =            0; d6 = 113.15 * mm2m; theta_offset6 = 0 + pi
    alpha6 =       0; a6 =            0

    # Adapt into the D-H parameters in this package
    dh_params = np.array([[d1, a1, alpha1, theta_offset1],  
                          [d2, a2, alpha2, theta_offset2],  
                          [d3, a3, alpha3, theta_offset3],  
                          [d4, a4, alpha4, theta_offset4],  
                          [d5, a5, alpha5, theta_offset5],  
                          [d6, a6, alpha6, theta_offset6]]) 
    
    robot = RobotSerial(dh_params, dh_type = "normal") # "normal" or "modified" D-H parameters
    # TBD: idk why when i try to run TM5-700 and TM5-900, the modified and normal D-H parameters seems to be messed up.


    # # Reference robot: AUBO-i10 serial robot, see details of the D-H parameters at "../AUBO-i10_DH_para.jpeg"
    # dh_params = np.array([[0.163 , 0.    , 0.5 * pi , 0.       ],  # d1, a1, alpha1, theta1
    #                       [0.    , 0.632 , pi       , 0.5 * pi ],  # d2, a2, alpha2, theta2
    #                       [0.    , 0.6005, pi       , 0.       ],  # d3, a3, alpha3, theta3
    #                       [0.2013, 0.    , -0.5 * pi, -0.5 * pi],  # d4, a4, alpha4, theta4
    #                       [0.1025, 0.    , 0.5 * pi , 0.       ],  # d5, a5, alpha5, theta5
    #                       [0.094 , 0.    , 0.       , 0.       ]]) # d6, a6, alpha6, theta6

    # =====================================
    # inverse
    # =====================================

    xyz = np.array([[0.4175], [-0.1223], [0.3611]])
    abc = np.array([pi, pi / 4, pi])
    end = Frame.from_euler_3(abc, xyz)
    robot.inverse(end)

    print("inverse is successful: {0}".format(robot.is_reachable_inverse))
    print("axis values: \n{0}".format(robot.axis_values))
    robot.show()

    # # example of unsuccessful inverse kinematics
    # xyz = np.array([[2.2], [0.], [1.9]])
    # end = Frame.from_euler_3(abc, xyz)
    # robot.inverse(end)

    # print("inverse is successful: {0}".format(robot.is_reachable_inverse))


if __name__ == "__main__":
    main()
