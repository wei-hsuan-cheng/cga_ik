import numpy as np

d2r = np.pi / 180

# Units: [m, deg]
r_c = 0.145
gamma = 54.735610
r_b = r_c * np.tan(gamma * d2r)
r_s_l = r_c / np.cos(gamma * d2r)

d = 0.05939697
r_e = 0.05939697
r_s_u = np.sqrt(d**2 + r_e**2)

# Print all parameters
print(f"r_c [m] = {r_c}")
print(f"gamma [deg] = {gamma}")
print(f"r_b [m] = {r_b}")
print(f"r_s_l [m] = {r_s_l}")

print(f"d [m] = {d}")
print(f"r_e [m] = {r_e}")
print(f"r_s_u [m] = {r_s_u}")
