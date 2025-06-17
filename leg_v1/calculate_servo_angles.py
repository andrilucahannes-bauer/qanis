import numpy as np

def calc_angle_to_dxl(angle_rad):
    max_rad = np.deg2rad(300)
    return round(angle_rad / max_rad * 1023)