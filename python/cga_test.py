'''
This is a testing file for the library cga.py
'''

import numpy as np
from cga import CGA

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
# pseudo scalar
I5 = e12345

G41_bases = 1 + e1 + e2 + e3 + e4 + e5 + e12 + e13 + e14 + e15 + e23 + e24 + e25 + e34 + e35 + e45 + e123 + e124 + e125 + e134 + e135 + e145 + e234 + e235 + e245 + e345 + e1234 + e1235 + e1245 + e1345 + e2345 + e12345
print(f"\nG41 bases = {G41_bases}")
print(f"\nG41 dimension = {len(G41_bases)}")


# Self-defined functions
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

####################################################################################################
# Extract each element of a G41 vec
a = 2 * e1 + 1 * e2
print(f"\na[1] = {a[1]}")

# array to G41 n-vec
b = np.ones(32).tolist()
b = CGA.fromarray(b)
print(f"\nb = {b}")

# Reverse
b_reverse = ~b
print(f"\nreverse of b = {b_reverse}")

# Dual
dual_b = b.Dual()
print(f"\ndual of b = {dual_b}")

# Inner product "|"
inner_e1e1 = e1 | e1
inner_e1e2 = e1 | e2
inner_nino = ni | no
print(f"\ninner product of e1 and e1 = {inner_e1e1}")
print(f"\ninner product of e1 and e2 = {inner_e1e2}")
print(f"\ninner product of e_inf and e_o = {inner_nino}")

# Outer product "^"
outer_e1e1 = e1 ^ e1
outer_e1e2 = e1 ^ e2
outer_nino = ni ^ no
print(f"\nouter product of e1 and e1 = {outer_e1e1}")
print(f"\nouter product of e1 and e2 = {outer_e1e2}")
print(f"\nouter product of e_inf and e_o = {outer_nino}")

# Geometric product "*"
geo_e1e1 = e1 * e1
geo_e1e2 = e1 * e2
geo_nino = ni * no
print(f"\ngeometric product of e1 and e1 = {geo_e1e1}")
print(f"\ngeometric product of e1 and e2 = {geo_e1e2}")
print(f"\ngeometric product of e_inf and e_o = {geo_nino}")

# Regressive product "&"
S1 = (no - ni).Dual()
S2 = (up(e1) - ni).Dual()
regressive_S1S2 = S1 & S2
print(f"\nregressive product to S1 and S2 = {regressive_S1S2}")

# Norm
norm_a = a.Norm() # or "CGA.Norm(a)
print(f"\nnorm of a = {norm_a}")

# Normalized
normalized_a = a.Normalized() # or "normalized_a = CGA.Normalized(a)"
print(f"\nnormalized a = {normalized_a}")

# Inverse
inverse_e12 = e12.Inverse()
print(f"\ninverse of e12 = {inverse_e12}")
print(f"\n e12 * inverse_e12 = {e1 * e2 * inverse_e12}")

# Grade projection
d = 0 * 1 + e1 + 2 * e2 + 3 * e3 + 4 * e4 + 5 * e5 + 12 * e12 + 1235 * e1235 + 12345 * e12345
print(f"\nd = {d}")
grade0_d = Grade(d, 0)
print(f"\ngrade-0 component of d = {grade0_d}")
grade1_d = Grade(d, 1)
print(f"\ngrade-1 component of d = {grade1_d}")
grade2_d = Grade(d, 2)
print(f"\ngrade-2 component of d = {grade2_d}")
grade3_d = Grade(d, 3)
print(f"\ngrade-3 component of d = {grade3_d}")
grade4_d = Grade(d, 4)
print(f"\ngrade-4 component of d = {grade4_d}")
grade5_d = Grade(d, 5)
print(f"\ngrade-5 component of d = {grade5_d}")

# CGA rotor and translator
rotor_x_45 = rot(e23, np.pi / 4)
print(f"\nrotor(x, 45 [deg]) = {rotor_x_45}")
trans_x_3 = trans(3 * e1)
print(f"\ntranslator(x, 3 [m]) = {trans_x_3}")


# R3 array
PX_R3 = [1, 0, 0]
# R3 array to G3 vec
PX_G3 = R2G(PX_R3)
# G3 vec to G41 vec
PX = up(PX_G3)
# G41 vec to G3 vec
PX_down = down(PX)

# Basics (point, line, and sphere)
LINE = PX ^ no ^ ni
SPHERE = (no - ni).Dual()
print(f"\nGrade-1 point: {PX}")
print(f"\nG3 point by down-projection: {PX_down}")
print(f"\nGrade-3 line: {LINE}")
print(f"\nGrade-4 sphere: {SPHERE}")


