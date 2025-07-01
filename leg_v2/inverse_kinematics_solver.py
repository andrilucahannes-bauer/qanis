import numpy as np

def _ik(x, z, upper, lower):
    d_squared = x ** 2 + z ** 2
    
    cos_theta2 = (d_squared - upper ** 2 - lower ** 2) / (2 * upper * lower)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = np.arccos(cos_theta2)

    k1 = upper + lower * cos_theta2
    k2 = lower * np.sin(theta2)
    theta1 = np.arctan2(z, x) - np.arctan2(k2, k1)

    return theta1, theta2

def transform_position_to_angles(foot_position, leg):
    x,y,z = foot_position
    r = np.hypot(x, y)

    # does this work if y is not 0?
    r = r if x >= 0 else -r

    theta0 = np.arctan2(y,z)
    
    r_yz = np.sqrt(y**2 + z**2)
    
    theta1, theta2 = _ik(x, r_yz, leg.upper_length, leg.lower_length)

    return theta0, theta1, theta2
