import numpy as np

def _ik(x, y, upper, lower):
    d_squared = x ** 2 + y ** 2
    
    cos_theta2 = (d_squared - upper ** 2 - lower ** 2) / (2 * upper * lower)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = np.arccos(cos_theta2)

    k1 = upper + lower * cos_theta2
    k2 = lower * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return theta1, theta2

def transform_position_to_angles(foot_position, up, low):
    x,y,z = foot_position
    # r = np.hypot(x, y)
    # does this work if y is not 0?
    #r = r if x >= 0 else -r

    # theta0 = np.arctan2(y, x)

    theta0 = np.arctan2(y,z)
    r_yz = np.sqrt(y**2 + z**2)

    theta1, theta2 = _ik(x, r_yz, up, low)

    return theta0, theta1, theta2

pos = (4,-1,10)

up = low = 10

res = transform_position_to_angles(pos, up, low)

print(np.rad2deg(res))
