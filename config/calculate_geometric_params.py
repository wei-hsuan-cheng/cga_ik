import numpy as np

d2r = np.pi / 180

# Units: [m, deg]
r_c = 0.145
ang_b_m = 54.735610
r_b = r_c * np.tan(ang_b_m * d2r)
d = 0.035
r_e = 0.15272197 / 2

r_s_epl = np.sqrt(d**2 + r_e**2)

# Print all parameters
print(f"r_c [m] = {r_c}")
print(f"ang_b_m [deg] = {ang_b_m}")
print(f"r_b [m] = {r_b}")
print(f"d [m] = {d}")
print(f"r_e [m] = {r_e}")

print(f"r_s_epl [m] = {r_s_epl}")