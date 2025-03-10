'''
cga_fk, an forward kinematics (FK) solver based on conformal geometric algebra (CGA)
Written by wei-hsuan-cheng

G41 structure: e1 * e1 = e2 * e2 = e3 * e3 = e4 * e4 = +1, e5 * e5 = -1
Definition of DUAL here: M.Dual() = e12345 * M = M * e12345
Null bases: ni = e5 + e4, no = 0.5 * (e5 - e4)


References for this code:
https://slides.com/hugohadfield/game2020
https://www.sciencedirect.com/science/article/pii/S0094114X22001045
'''

'''
Potential issues:
1) Plot in matplotlib
'''

from math import pi, sqrt, atan2
import numpy as np
from scipy.spatial.transform import Rotation 
from cga import CGA

# Define bases
e1 = CGA(1.0, 1); e2 = CGA(1.0, 2); e3 = CGA(1.0, 3); e4 = CGA(1.0, 4); e5 = CGA(1.0, 5)
ni = e5 + e4; no = 0.5 * (e5 - e4)
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
            return float(np.cos(angle / 2)) - float(np.sin(angle / 2)) * bivector.Normalized() # [rad]
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

def  DHParamsandDHOffsets():
    # Set up robot D-H parameters (angle in [rad], length in [m]). The modified D-H params are used here, while the angle offsets are different from the convection of D-H params. See detail in later
    # TM5M-700
    alpha0 =       0; a0 =            0; d1 =  145.2 * mm2m; # frame {0} -> {1}
    alpha1 = -pi / 2; a1 =            0; d2 =             0; # frame {1} -> {2}
    alpha2 =       0; a2 = 329.0 * mm2m; d3 =             0; # frame {2} -> {3}
    alpha3 =       0; a3 = 311.5 * mm2m; d4 = -122.3 * mm2m; # frame {3} -> {4}
    alpha4 = -pi / 2; a4 =            0; d5 =    106 * mm2m; # frame {4} -> {5}
    alpha5 = -pi / 2; a5 =            0; d6 = 113.15 * mm2m; # frame {5} -> {6}
    # The cga_offset in cga_ik may be different from the dh_offset in D-H params since we calculate the angle between LINKS (check the link relation of TM zero config)
    cga_offset1 = 0
    cga_offset2 = 0 
    cga_offset3 = 0 
    cga_offset4 = 0 
    cga_offset5 = 0
    cga_offset6 = pi
    # Angle offsets in D-H params, dh_offsets [deg]
    dh_offset1 = 0 
    dh_offset2 = -pi / 2
    dh_offset3 = 0 
    dh_offset4 = -pi / 2
    dh_offset5 = 0 
    dh_offset6 = pi
    return alpha0, a0, d1, \
           alpha1, a1, d2, \
           alpha2, a2, d3, \
           alpha3, a3, d4, \
           alpha4, a4, d5, \
           alpha5, a5, d6, \
           dh_offset1, dh_offset2, dh_offset3, dh_offset4, dh_offset5, dh_offset6

def MotorToSix(X, x, y, z):
    Rotation = [[(x | e1)[0], (y | e1)[0], (z | e1)[0]],
                [(x | e2)[0], (y | e2)[0], (z | e2)[0]],
                [(x | e3)[0], (y | e3)[0], (z | e3)[0]]]
    Ry = atan2(-Rotation[2][0], sqrt(Rotation[0][0] ** 2 + Rotation[1][0] ** 2)) * r2d
    Rz = atan2( Rotation[1][0] / np.cos(Ry),  Rotation[0][0] / np.cos(Ry)) * r2d
    Rx = atan2( Rotation[2][1] / np.cos(Ry),  Rotation[2][2] / np.cos(Ry)) * r2d
    R6vec = [down(X)[1], down(X)[2], down(X)[3], Rx, Ry, Rz] # [mm] [deg]
    return R6vec


'''
Defining transformations for FK
'''
def CGAFK(joints, degrees):
    if len(joints) != 6:
        return "Dimension error (not 6D joint positions)"
    
    '''
    D-H params, base frame, target pose, and the configuration params
    '''
    alpha0, a0, d1, \
    alpha1, a1, d2, \
    alpha2, a2, d3, \
    alpha3, a3, d4, \
    alpha4, a4, d5, \
    alpha5, a5, d6, \
    dh_offset1, dh_offset2, dh_offset3, dh_offset4, dh_offset5, dh_offset6 =  DHParamsandDHOffsets()
    
    
    
    if degrees == True:
        dh_params = [[alpha0, a0, d1, joints[0] * d2r + dh_offset1],  
                     [alpha1, a1, d2, joints[1] * d2r + dh_offset2],  
                     [alpha2, a2, d3, joints[2] * d2r + dh_offset3],  
                     [alpha3, a3, d4, joints[3] * d2r + dh_offset4],  
                     [alpha4, a4, d5, joints[4] * d2r + dh_offset5],  
                     [alpha5, a5, d6, joints[5] * d2r + dh_offset6]] 
    elif degrees == False:
        dh_params = [[alpha0, a0, d1, joints[0] + dh_offset1],  
                     [alpha1, a1, d2, joints[1] + dh_offset2],  
                     [alpha2, a2, d3, joints[2] + dh_offset3],  
                     [alpha3, a3, d4, joints[3] + dh_offset4],  
                     [alpha4, a4, d5, joints[4] + dh_offset5],  
                     [alpha5, a5, d6, joints[5] + dh_offset6]] 
    else:
        return "Type error (degrees can only be True or False)"
        

    # Frame transformation described by CGA rotors
    # Ralpha_{i-1}, Ta_{i-1}, Td_{i}, Rtheta_{i}
    Ralpha_iminus1 = []
    Ta_iminus1 = []
    Td_i = []
    Rtheta_i = []
    M_iminus1_i = []
    M_0_i = [1]
    orgi = [no]
    R_iminus1_i = []
    R_0_i = [1]
    xi = [e1]
    yi = [e2]
    zi = [e3]
    
    for i in range(len(joints)):
        # Rotors and Translators
        Ralpha_iminus1.append( rot(e23, dh_params[i][0]) )
        Ta_iminus1.append( trans(dh_params[i][1] * e1) )
        Td_i.append( trans(dh_params[i][2] * e3) )
        Rtheta_i.append( rot(e12, dh_params[i][3]) )
        
        # Motors for frame origins, and rotors for frame orientations
        M_iminus1_i.append( Ralpha_iminus1[i] * Ta_iminus1[i] * Td_i[i] * Rtheta_i[i] )
        M_0_i.append( M_0_i[i] * M_iminus1_i[i] )
        orgi.append( M_0_i[i + 1] * no * ~M_0_i[i + 1] )
        
        R_iminus1_i.append( Ralpha_iminus1[i] * Rtheta_i[i] )
        R_0_i.append( R_0_i[i] * R_iminus1_i[i] )
        xi.append( R_0_i[i + 1] * e1 * ~R_0_i[i + 1] )
        yi.append( R_0_i[i + 1] * e2 * ~R_0_i[i + 1] )
        zi.append( R_0_i[i + 1] * e3 * ~R_0_i[i + 1] )
             
    R6vec = MotorToSix(orgi[6], xi[6], yi[6], zi[6]) # x, y, z, Rx, Ry, Rz [m] [deg]      
        
    return orgi, R6vec