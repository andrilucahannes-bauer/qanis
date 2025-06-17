import numpy as np

def ik(x, y, upper, lower):
    d_squared = x ** 2 + y ** 2
    
    cos_theta2 = (d_squared - upper ** 2 - lower ** 2) / (2 * upper * lower)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = np.arccos(cos_theta2)

    k1 = upper + lower * cos_theta2
    k2 = lower * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return theta1, theta2

def ik_old(x, y, upper, lower):
    d_squared = x ** 2 + y ** 2
    cos_theta2 = (d_squared - upper ** 2 - lower ** 2) / -(2 * upper * lower)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    cos_theta1 = (upper ** 2 + d_squared - lower ** 2) / (2 * upper * np.sqrt(d_squared))
    cos_theta1 = np.clip(cos_theta1, -1.0, 1.0)

    theta2 = np.arccos(cos_theta2)
    theta1 = np.arctan2(y, x) + np.arccos(cos_theta1)

    #print(f'cos_theta2: {cos_theta2}, cos_theta1: {cos_theta1}')
    return theta1, theta2