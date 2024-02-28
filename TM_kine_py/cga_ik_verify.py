# from time import time
from timeit import timeit
from math import pi, sqrt, atan2
# import numpy as np
# from scipy.spatial.transform import Rotation 
from cga_ik import CGAIK, down, DHParamsandCGAOffsets

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

    R6vec = [a3 + d5, d4, d1 + a2 - d6, 180, 45, 180] # [x, y, z, Rx, Ry, Rz]
    degrees = True
    config_params = [1, -1, -1]

    kud, klr, kfn, PPcd, PP4d, PP3d, PP2d, reachable, X0, X1, X2, X3, X4, X5, X6, joints = CGAIK(R6vec, degrees, config_params)

    # print(f"\nRobot configuration = [{kud}, {klr}, {kfn}]")
    # print(f"\nPoint pair distances = [{PPcd}, {PP4d}, {PP3d}, {PP2d}]")
    # print(f"\nReachable or not: {reachable}")
    # print(f"\nFrame origins:\nX0 = {down(X0)},\nX1 = {down(X1)},\nX2 = {down(X2)},\nX3 = {down(X3)},\nX4 = {down(X4)},\nX5 = {down(X5)},\nX6 = {down(X6)},")
    # print(f"\njoints = {joints}")

if __name__ == "__main__":
    # main()
    # Measure the average time taken by main() function using timeit
    avg_time = timeit(main, number = 1000) / 1000  # Run main() 1000 times
    print(f"\nAverage computing time = {avg_time * 1000} [ms]")
