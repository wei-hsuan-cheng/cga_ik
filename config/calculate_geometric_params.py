import numpy as np

d2r = np.pi / 180

# Units: [m, deg]
# Lower parts
r_c = 0.145
ang_b_m = 54.735610
r_b = r_c * np.tan(ang_b_m * d2r)

# Upper parts
d = 0.04824849
r_s_epl = 0.084
r_e = np.sqrt(r_s_epl**2 - d**2)


# Print all parameters
print(f"\n[Lower part params]")
print(f"r_c [m] = {r_c}")
print(f"ang_b_m [deg] = {ang_b_m}")
print(f"r_b [m] = {r_b}")

print(f"\n[Upper part params]")
print(f"d [m] = {d}")
print(f"r_s_epl [m] = {r_s_epl}")
print(f"r_e [m] = {r_e}")

