from timeit import timeit
from math import pi, sqrt, atan2
import numpy as np
# from scipy.spatial.transform import Rotation 
from cga_ik import CGAIK_CONFIG, CGAIK_ALL, CGAIK_BEST, down, DHParamsandCGAOffsets
from cga_fk import CGAFK

## Unit translform
r2d = 180 / pi
d2r = 1 / r2d
mm2m = 10 ** (-3) # [mm] to [m]
m2mm = 1 / mm2m

def main():
    alpha0, a0, d1, \
    alpha1, a1, d2, \
    alpha2, a2, d3, \
    alpha3, a3, d4, \
    alpha4, a4, d5, \
    alpha5, a5, d6, \
    cga_offset1, cga_offset2, cga_offset3, cga_offset4, cga_offset5, cga_offset6 = DHParamsandCGAOffsets()

    # R6vec_ik = [a3 + d5, d4, d1 + a2 - d6, pi, pi / 4, pi] # [x, y, z, Rx, Ry, Rz] [m] [rad]
    # degrees = False
    R6vec_ik = [a3 + d5, d4, d1 + a2 - d6, 180, 45, 180] # [x, y, z, Rx, Ry, Rz] [m] [deg]
    degrees = True

    
    config_params = [1, 1, -1]
    kud, klr, kfn, PPcd, PP4d, PP3d, PP2d, reachable, X0, X1, X2, X3, X4, X5, X6, joints_config = CGAIK_CONFIG(R6vec_ik, degrees, config_params)
    # print(f"\nRobot configuration = [{kud}, {klr}, {kfn}]")
    # print(f"\nPoint pair distances = [{PPcd}, {PP4d}, {PP3d}, {PP2d}]")
    # print(f"\nReachable or not: {reachable}")
    # print(f"\nFrame origins [m] (IK):\nX0 = {down(X0)},\nX1 = {down(X1)},\nX2 = {down(X2)},\nX3 = {down(X3)},\nX4 = {down(X4)},\nX5 = {down(X5)},\nX6 = {down(X6)},")
    print(f"\njoints_config [deg] = {joints_config}")
    
    
    
    # joints_all, sol_num = CGAIK_ALL(R6vec_ik, degrees)
    # for i in range(len(joints_all)):
    #     print(f"\njoints[{i}] [deg] = {joints_all[i]}")
    # print(f"\nnumber of solutions = {sol_num}")


    # # joints_previous = np.array([0, 0, 0, 0, 0, 0]) # zero-configuration
    # joints_previous = np.array([0, 0, 90, 0, 90, -90]) # IK starting pose
    # joints_best, sol_num = CGAIK_BEST(R6vec_ik, joints_previous, degrees)
    # print(f"\njoints_best [deg] = {joints_best}")
    # print(f"\nnumber of solutions = {sol_num}")
    
    '''
    Verify IK by FK
    '''
    orgi, R6vec_fk = CGAFK(joints_config, degrees)
    # print(f"\nFrame origins [m] (FK):\norg0 = {down(orgi[0])},\norg1 = {down(orgi[1])},\norg2 = {down(orgi[2])},\norg3 = {down(orgi[3])},\norg4 = {down(orgi[4])},\norg5 = {down(orgi[5])},\norg6 = {down(orgi[6])},")
    print(f"\nR6vec_fk [m][deg] = {R6vec_fk}")
    print(f"\nR6vec_IK [m][deg] = {R6vec_ik}")
    
if __name__ == "__main__":
    # main()
    loop_num = 100
    avg_time = timeit(main, number = loop_num) / loop_num  # Run main() various times
    print(f"\nAverage computing time = {avg_time * 1000} [ms]")
