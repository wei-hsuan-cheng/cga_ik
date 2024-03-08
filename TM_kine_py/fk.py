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

    # =====================================
    # forward
    # =====================================
    theta_ik_start = np.array([0, 0, pi / 2, 0, pi / 2, -pi / 2])
    theta_zero = np.array([0, 0, 0, 0, 0, 0])

    theta1 = 140.161
    theta2 = -78.589
    theta3 = 60.045 
    theta4 = 146.063
    theta5 = 63.064 
    theta6 = 59.461 
    theta = np.array([theta1, theta2, theta3, theta4, theta5, theta6]) * d2r
    

    # Test for cga_ik
    # Testing pose
    theta = np.array([-2.0997135429518626e-13, -5.251630935607801, 115.2856690398151, -65.03403810420734, 89.99999999999986, -90.00000000000016]) * d2r # (1, 1, 1)
    # theta = np.array([-2.0997135429518626e-13, 105.09540027690205, -115.28566903981513, 55.19026876291304, 89.99999999999986, -90.00000000000016]) * d2r # (-1, 1, 1)
    # theta = np.array([140.16097522540764, 7.5620148600304224, -115.28702022150955, 55.24376655881911, -63.06394405225434, -120.5387162995598]) * d2r # (1, -1, 1)
    # theta = np.array([140.16097522540764, -102.78623888439832, 115.28702022150958, -64.98202013977128, -63.06394405225434, -120.5387162995598]) * d2r # (-1, -1, 1)
    # theta = np.array([-2.0997135429518626e-13, 18.92377851068305, 60.046685477792266, 146.02953601152464, -89.99999999999986, 89.99999999999984]) * d2r # (1, 1, -1)
    # theta = np.array([-2.0997135429518626e-13, 77.16127745687446, -60.04668547779227, -152.11459197908223, -89.99999999999986, 89.99999999999984]) * d2r # (-1, 1, -1)
    # theta = np.array([140.16097522540764, -20.353282143641284, -60.04527541294856, -152.0826812460702, 63.06394405225434, 59.4612837004402]) * d2r # (1, -1, -1) 
    # theta = np.array([140.16097522540764, -78.58942240447962, 60.04527541294855, 146.06290818887106, 63.06394405225434, 59.4612837004402]) * d2r # (-1, -1, -1) 

    # Close to straight pose
    # theta = np.array([-2.5109386197960175e-10, -2.9316599799091945, 6.049611049642179, -3.124365156630687, 2.5109386040623304e-10, -0.0064140868977084365]) * d2r # (1, 1, 1)
    # theta = np.array([-2.5109386197960175e-10, 2.9525075350928605, -6.049611049642177, 3.090689427651614, 2.5109386040623304e-10, -0.0064140868977084365]) * d2r # (-1, 1, 1)
    # theta = np.array([0.09931890634444754, 2.9325921409176634, -6.04961888092655, 3.1170267400251026, -0.09931890634444754, 1.6208106656738083e-11]) * d2r # (1, -1, 1)
    # theta = np.array([0.09931890634444754, -2.9515829908022253, 6.049618880926549, -3.098035890108106, -0.09931890634444754, 1.6208106656738083e-11]) * d2r # (-1, -1, 1)
    # Impossible for wrist flipped configuration

    # theta = np.array([-1.4323717960069242e-13, -5.251630935607782, 115.28566903981513, -65.03403810420738, 90.0000000000001, -90.0000000000001]) * d2r # (1, 1, 1)

    f = robot.forward(theta)

    print("-------forward-------")
    print("end frame t_4_4:")
    print(f.t_4_4)
    print("end frame xyz:")
    print(f.t_3_1.reshape([3, ]) * m2mm)
    print("end frame abc:")
    print(f.euler_3 * r2d)
    print("end frame rotational matrix:")
    print(f.r_3_3)
    print("end frame quaternion:")
    print(f.q_4)
    print("end frame angle-axis:")
    print(f.r_3)

    robot.show(True,False)
    # robot.show(True,True)


if __name__ == "__main__":
    main()
