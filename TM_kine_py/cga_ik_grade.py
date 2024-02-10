'''
cga_ik, an inverse kinematics (IK) solver based on conformal geometric algebra (CGA)
Written by wei-hsuan-cheng

References for this code:
https://slides.com/hugohadfield/game2020
https://www.mic-journal.no/ABS/MIC-2016-1-6.asp/
https://www.sciencedirect.com/science/article/pii/S0094114X22001045
'''

'''
Potential issues:
rooms for optimization (maybe no need for Grade() everytime)
'''
from time import time
import numpy as np
from math import pi, sqrt, atan2
from cga import CGA

start_time = time() # [s]
# Define bases
e1 = CGA(1.0, 1); e2 = CGA(1.0, 2); e3 = CGA(1.0, 3); e4 = CGA(1.0, 4); e5 = CGA(1.0, 5)
ni = e5 + e4; no = (e5 - e4) * 0.5
# Define other multi-vector bases
# bivectors
e12 = CGA(1.0, 6); e13 = CGA(1.0, 7); e14 = CGA(1.0, 8); e15 = CGA(1.0, 9)
e23 = CGA(1.0, 10); e24 = CGA(1.0, 11); e25 = CGA(1.0, 12)
e34 = CGA(1.0, 13); e35 = CGA(1.0, 14)
e45 = CGA(1.0, 15)
# trivectors
e123 = CGA(1.0, 16); e124 = CGA(1.0, 17); e125 = CGA(1.0, 18)
e134 = CGA(1.0, 19); e135 = CGA(1.0, 20); e145 = CGA(1.0, 21)
e234 = CGA(1.0, 22); e235 = CGA(1.0, 23); e245 = CGA(1.0, 24)
e345 = CGA(1.0, 25)
# 4-vectors
e1234 = CGA(1.0, 26)
e1235 = CGA(1.0, 27)
e1245 = CGA(1.0, 28)
e1345 = CGA(1.0, 29)
e2345 = CGA(1.0, 30)
# 5-vector
e12345 = CGA(1.0, 31)

## Unit translform
r2d = 180 / pi
d2r = 1 / r2d
mm2m = 10 ** (-3) # [mm] to [m]
m2mm = 1 / mm2m

'''
Self-defined functions
'''
def constrained_angle(angle):
    if angle >= pi and angle < 2 * pi:
        angle -= 2 * pi
    elif angle >= -2 * pi and angle < -pi:
        angle += 2 * pi
    else:
        angle = angle
    return angle

def R2G(array): # R3 array to G3 vec
    if len(array) == 3:
        return array[0] * e1 + array[1] * e2 + array[2] * e3
    else:
        print(f"\nDimension error (not a R3 element)")

def Grade(vec_G41, k): # grade projection (for those component not of grade-k, set as zero)
    if len(vec_G41) == 32:
        if k == 0:
            res = vec_G41[0] * 1
        elif k == 1:
            res = vec_G41[1] * e1 + vec_G41[2] * e2 + vec_G41[3] * e3 + vec_G41[4] * e4 + vec_G41[5] * e5
        elif k == 2:
            res = vec_G41[6] * e12 + vec_G41[7] * e13 + vec_G41[8] * e14 + vec_G41[9] * e15 + vec_G41[10] * e23 + vec_G41[11] * e24 + vec_G41[12] * e25 + vec_G41[13] * e34 + vec_G41[14] * e35 + vec_G41[15] * e45
        elif k == 3:
            res = vec_G41[16] * e123 + vec_G41[17] * e124 + vec_G41[18] * e125 + vec_G41[19] * e134 + vec_G41[10] * e135 + vec_G41[21] * e145 + vec_G41[22] * e234 + vec_G41[23] * e235 + vec_G41[24] * e245 + vec_G41[25] * e345 
        elif k == 4:
            res = vec_G41[26] * e1234 + vec_G41[27] * e1235 + vec_G41[28] * e1245 + vec_G41[29] * e1345 + vec_G41[30] * e2345
        elif k == 5:
            res = vec_G41[31] * e12345
        else:
            print(f"\nGrade error (can only be an integer from 0 to 5)")
        return res
    else:
        print(f"\nDimension error (not a G41 element)")

def rot(bivector, angle):
    if len(bivector) == 32 and (x == 0 for x in bivector[0:6]) and  (x == 0 for x in bivector[16:]):
        if type(angle) in (int, float, np.float64):
            return float(np.cos(angle / 2)) - float(np.sin(angle / 2)) * bivector.Normalized()
        else:
            print(f"\nDimension error (angle has to be a scalar)")
    else:
        print(f"\nDimension error (not a bivector)")

def trans(trans_vec):
    if len(trans_vec) == 32 and trans_vec[0] == 0 and  (x == 0 for x in trans_vec[4:]):
        return 1 - 0.5 * trans_vec * ni
    else:
        print(f"\nDimension error (translation vector should be a G3 element)")

def up(vec_G3): # up-projection (G3 vec to G41 vec)
    if len(vec_G3) == 32 and vec_G3[0] == 0 and  (x == 0 for x in vec_G3[4:]):
        return vec_G3 + 0.5 * vec_G3 * vec_G3 * ni + no
    else:
        print(f"\nDimension error (not a G3 element)")

def down(vec_G41): # down-projection (G41 vec to G3 vec)
    if len(vec_G41) == 32 and vec_G41[0] == 0 and  (x == 0 for x in vec_G41[6:]):
        return (vec_G41 ^ (no ^ ni)) * (no ^ ni) * ((-1) * vec_G41 | ni).Inverse()
    else:
        print(f"\nDimension error (not a G41 element or not able to down-project to G3)")


'''
D-H params, base frame, target pose, and the configuration params
'''
# Set up robot D-H parameters (angle in [rad], length in [m]). The modified D-H params are used here, while the angle offsets are different from the convection of D-H params. See detail in later
# TM5M-700
alpha0 =       0; a0 =            0; d1 =  145.2 * mm2m; # frame {0} -> {1}
alpha1 = -pi / 2; a1 =            0; d2 =             0; # frame {1} -> {2}
alpha2 =       0; a2 = 329.0 * mm2m; d3 =             0; # frame {2} -> {3}
alpha3 =       0; a3 = 311.5 * mm2m; d4 = -122.3 * mm2m; # frame {3} -> {4}
alpha4 = -pi / 2; a4 =            0; d5 =    106 * mm2m; # frame {4} -> {5}
alpha5 = -pi / 2; a5 =            0; d6 = 113.15 * mm2m; # frame {5} -> {6}

# X0 or {0}-org
X0 = no
org0 = X0
# {0}-axes
x0 = e1
y0 = e2
z0 = e3

# Target pose 

# # X6 or {6}-org (IK starting pose)
# X6 = up((a3 + d5) * e1 + (d4) * e2 + (d1 + a2 - d6) * e3)
# org6 = X6
# # {6}-axes (IK starting pose)
# x6 = (-1) * e1
# y6 =  1   * e2
# z6 = (-1) * e3

# X6 or {6}-org (testing pose)
X6 = up((a3 + d5) * e1 + (d4) * e2 + (d1 + a2 - d6) * e3)
org6 = X6
# {6}-axes (testing pose)
Rt1 = rot(e1 * e3, pi / 4)
Rt2 = rot(e2 * e3, 0 * pi / 4)
Rt = Rt2 * Rt1
x6 = Rt * (-1) * e1 * ~Rt
y6 = Rt *  1   * e2 * ~Rt
z6 = Rt * (-1) * e3 * ~Rt

# Robot configuration (# 8 configurations, closed-form solution)
kud = 1; # elbow up: 1, elbow down: -1
klr = 1; # should right: 1, should left: -1 (shouler here is connected to the big arm)
kfn = 1; # wrist not flipped: 1, wrist flipped: -1


# # Later
# CGA motor to 4x4 to RzRyRx
#  Rotation6 = [[x6 | e1, y6 | e1, z6 | e1], 
#                     [x6 | e2, y6 | e2, z6 | e2], 
#                     [x6 | e3, y6 | e3, z6 | e3]];
#  Ry_6 = Math.atan2(-Rotation6[2][0], Math.sqrt(Rotation6[0][0]**2 + Rotation6[1][0]**2));
#  Rz_6 = Math.atan2(Rotation6[1][0] / Math.cos(Ry_6), Rotation6[0][0] / Math.cos(Ry_6));
#  Rx_6 = Math.atan2(Rotation6[2][1] / Math.cos(Ry_6), Rotation6[2][2] / Math.cos(Ry_6));
    
#  Translation6 = [[down(X6) | e1], 
#                     [down(X6) | e2], 
#                     [down(X6) | e3]];
                    

'''
Inverse kinematics of the robot
Solving the null points (frame origines)
'''
## X5 and X1 (trivial)
X5 = up(down(X6) - d6 * z6)
org5 = X5
X1 = up(down(X0) + d1 * e3)
org1 = X1


## X3 and X4
# Intersect 2 spheres gives 1 circle
Sc = (X5 - 0.5 * (d4 ** 2) * ni).Dual() # grade-4 sphere
K0 = (no - (Sc.Dual() | no) * ni).Dual(); K0 = Grade(K0, 4).Normalized() # grade-4 sphere
C5k = (Sc & K0).Normalized() # grade-3 circle
# Intersect the circle and a plane gives a point pair
PPc = (X5 ^ e1 ^ e2 ^ ni) & C5k; PPc = (-1) * Grade(PPc, 2).Normalized() # grade-2 point pair
PPcd = (PPc | PPc) * ((PPc ^ ni) | (PPc ^ ni)).Inverse(); PPcd = PPcd[0] # point pair distance

# The square of the point pair describes if the spheres intersect
if (PPc * PPc)[0] > 0:
    # If the spheres intersect then we can just choose one solution. Extract each point in the point pair by method of projectors
    Xc = (1 + klr * PPc * ( 1 / sqrt((PPc * PPc)[0]) )) * (PPc | ni); Xc = up(down(Xc))
    endpoint5 = X5
else:
    # If the sphere at origin vanishes (radius = 0), the point pair becomes a single point
     endpoint5 = X5
     Xc = no

     
# Vertical plane passing through Xc
Pc = no ^ e3 ^ Xc ^ ni
# Pc shift to pass through X5
Pc_ver = ( Pc.Dual() + (X5 | Pc.Dual()) * ni ).Dual()
# Plane perp to Pc_ver and pass through X5
P56 = (X5 ^ X6).Dual() ^ ni
n56 = (-1) * ((P56 | no) | ni).Normalized()
Pc_hor = X5 ^ n56 ^ ni
# Line passing through X4 and X5
L54 = Pc_ver & Pc_hor
# Sphere centred at X5 w/ radius d5
S5 = (X5 - 0.5 * (d5 ** 2) * ni).Dual(); S5 = Grade(S5, 4).Normalized();
# Intersect the sphere and a line gives a point pair
PP4 = L54.Dual() | S5
PP4d = (PP4 | PP4) * ((PP4 ^ ni) | (PP4 ^ ni)).Inverse(); PP4d = PP4d[0] # point pair distance

if (PP4 * PP4)[0] > 0:
    # If the spheres intersect then we can just choose one solution. Extract each point in the point pair by method of projectors
    X4 = (1 + kfn * PP4 * (1 / sqrt((PP4 * PP4)[0]))) * (PP4 | ni); X4 = up(down(X4))
    endpoint4 = X4

# Sphere centred at X4 w/ radius d4
S4 = (X4 - 0.5 * (d4 ** 2) * ni).Dual(); S4 = Grade(S4, 4).Normalized()
L34 = X4 ^ Pc.Dual() ^ ni
# Intersect the sphere and a line gives a point pair
PP3 = L34.Dual() | S4
PP3d = (PP3 | PP3) * ((PP3 ^ ni) | (PP3 ^ ni)).Inverse(); PP3d = PP3d[0] # point pair distance

if (PP3 * PP3)[0] > 0:
    # If the spheres intersect then we can just choose one solution. Extract each point in the point pair by method of projectors
    X3 = (1 + klr * PP3 * (1 / sqrt((PP3 * PP3)[0]))) * (PP3 | ni); X3 = up(down(X3)) 
    endpoint3 = X3

## X2
# Spheres centred at X1 w/ radius a2 and centred at X3 w/ radius a3
S3 = (X3 - 0.5 * (a3 ** 2) * ni).Dual(); S3 = Grade(S3, 4).Normalized() # grade-4 plane
S1 = (X1 - 0.5 * (a2 ** 2) * ni).Dual(); S1 = Grade(S1, 4).Normalized() # grade-4 plane
C2 = (S1 & S3).Normalized() # grade-3 circle
# Intersect the circle and a plane gives a point pair
PP2 = (-1) * Pc & C2; PP2 = Grade(PP2, 2).Normalized() # grade-2 point pair
PP2d = (PP2 | PP2) * ((PP2 ^ ni) | (PP2 ^ ni)).Inverse(); PP2d = PP2d[0] # point pair distance

if (PP2 * PP2)[0] > 0:
    # If the spheres intersect then we can just choose one solution. Extract each point in the point pair by method of projectors
    X2 = (1 + kud * PP2 * (1 / sqrt((PP2 * PP2)[0]))) * (PP2 | ni); X2 = up(down(X2))
    endpoint2 = X2
    unreachable = []
else:
    # If the spheres do not intersect then we will just reach for the object.
    endpoint2 = up(down(X1) + (a2 + a3) * (down(X3)- down(X1)).Normalized())
    X2 = up(down(X1) + a2 * (down(X3)- down(X1)).Normalized())
    unreachable = "unreachable!"


'''
Forming geometric relations among the null points
'''
# Form lines
L01 = (no ^ e3 ^ ni).Normalized() # grade-3 line
L12 = (X1 ^ X2 ^ ni).Normalized() # grade-3 line
L23 = (X2 ^ X3 ^ ni).Normalized() # grade-3 line

# Solving the joint variables by defining the angle offsets for zero configuration, the vectors, and the bivectors
# The cga_offset in cga_ik may be different from the cga_offset in D-H params since we calculate the angle between LINKS (check the link relation of TM zero config)
cga_offset1 = 0
cga_offset2 = 0 
cga_offset3 = 0 
cga_offset4 = 0 
cga_offset5 = 0
cga_offset6 = pi

# Vectors and bivectors
ath1 =                    e2; ath1 = Grade(ath1, 1).Normalized()
bth1 = klr * (1) * Pc.Dual(); bth1 = Grade(bth1, 1).Normalized()
Nth1 =               e1 ^ e2; Nth1 = Grade(Nth1, 2).Normalized()

ath2 =      L01 | (ni ^ no); ath2 = Grade(ath2, 1).Normalized()
bth2 =      L12 | (ni ^ no); bth2 = Grade(bth2, 1).Normalized()
Nth2 = klr * (Pc | no) | ni; Nth2 = Grade(Nth2, 2).Normalized()

ath3 =      L12 | (ni ^ no); ath3 = Grade(ath3, 1).Normalized()
bth3 =      L23 | (ni ^ no); bth3 = Grade(bth3, 1).Normalized()
Nth3 = klr * (Pc | no) | ni; Nth3 = Grade(Nth3, 2).Normalized()

ath4 =                    L23 | (ni ^ no); ath4 = Grade(ath4, 1).Normalized()
bth4 = kfn * ( (-1) * (L54 | (ni ^ no)) ); bth4 = Grade(bth4, 1).Normalized()
Nth4 =               klr * (Pc | no) | ni; Nth4 = Grade(Nth4, 2).Normalized()

ath5 =                klr * (1) * Pc.Dual(); ath5 = Grade(ath5, 1).Normalized()
bth5 =                             (-1) * z6; bth5 = Grade(bth5, 1).Normalized()
Nth5 = kfn * ( (1) * L54.Dual() ^ no ) | ni; Nth5 = Grade(Nth5, 2).Normalized()

# Still have rooms to be improved for calculating theta6
ath6 = kfn * ( (-1) * (L54 | (ni ^ no)) ); ath6 = Grade(ath6, 1).Normalized() # y6 lies in vec_54 (L54 unit direction vector) as theta6 = 0
bth6 =                          (-1) * y6; bth6 = Grade(bth6, 1).Normalized()
Nth6 =                          (1) * n56; Nth6 = Grade(Nth6, 2).Normalized()

'''
The sign of the configuration params may vary, depending on different definition of the DUAL operator (different sign). Other kind of definition may lead to:
X3 = (1 - klr * PP3 * (1 / sqrt((PP3 * PP3)[0]))) * (PP3 | ni); X3 = up(down(X3))bth1 = klr * (-1) * Pc.Dual(); bth1 = Grade(bth1, 1).Normalized()
ath5 =                klr * (-1) * Pc.Dual(); ath5 = Grade(ath5, 1).Normalized()
Nth5 = kfn * ( (-1) * L54.Dual() ^ no ) | ni; Nth5 = Grade(Nth5, 2).Normalized()
Nth6 =                         (-1) * n56; Nth6 = Grade(Nth6, 2).Normalized()
'''

# Solving joint variables [deg]
theta1 = atan2( ( (ath1 ^ bth1) * Nth1.Inverse() )[0], (ath1 | bth1)[0] ) + cga_offset1; theta1 = constrained_angle(theta1) * r2d
theta2 = atan2( ( (ath2 ^ bth2) * Nth2.Inverse() )[0], (ath2 | bth2)[0] ) + cga_offset2; theta2 = constrained_angle(theta2) * r2d
theta3 = atan2( ( (ath3 ^ bth3) * Nth3.Inverse() )[0], (ath3 | bth3)[0] ) + cga_offset3; theta3 = constrained_angle(theta3) * r2d
theta4 = atan2( ( (ath4 ^ bth4) * Nth4.Inverse() )[0], (ath4 | bth4)[0] ) + cga_offset4; theta4 = constrained_angle(theta4) * r2d
theta5 = atan2( ( (ath5 ^ bth5) * Nth5.Inverse() )[0], (ath5 | bth5)[0] ) + cga_offset5; theta5 = constrained_angle(theta5) * r2d
theta6 = atan2( ( (ath6 ^ bth6) * Nth6.Inverse() )[0], (ath6 | bth6)[0] ) + cga_offset6; theta6 = constrained_angle(theta6) * r2d

joints = [theta1, theta2, theta3, theta4, theta5, theta6]
print(f"\njoints = {joints}")
print(f"\nframes:\nX0 = {down(X0)},\nX1 = {down(X1)},\nX2 = {down(X2)},\nX3 = {down(X3)},\nX4 = {down(X4)},\nX5 = {down(X5)},\nX6 = {down(X6)},")
end_time = time() # [s]
print(f"\ncomputing time = {(end_time - start_time) * 1000} [ms]")
# testing pose
# Correct: 
# Wrong: 
