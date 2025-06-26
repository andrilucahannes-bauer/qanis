import numpy as np

# alles in rad übergeben
def _calculate_4_bar(th2, a, b, c, d):
    # position of B
    x_b = a * np.cos(th2)
    y_b = a * np.sin(th2)

    # define diagnonal f zwischen b und d
    f = np.sqrt((d - x_b) ** 2 + y_b ** 2)

    # Kosinussatz BDC
    beta = np.arccos(np.clip((f ** 2 + c ** 2 - b ** 2) / (2 * f * c), -1.0, 1.0))

    # winkel zwischen f und d (BDA)
    gamma = np.arctan2(y_b, d - x_b)

    # winkel zwischen c und d (CDA)
    th4 = np.pi - gamma - beta

    # position of C
    x_c = c * np.cos(th4) + d
    y_c = c * np.sin(th4)

    # winkel zwischen b und x-achse (also steigung von B nach C)
    th3 = np.arctan2((y_c - y_b), (x_c - x_b))

    ## Calculate remaining internal angles of linkage
    ABC = np.pi - th2 + th3
    BCD = th4 - th3
    CDA = np.pi * 2 - th2 - ABC - BCD

    return ABC, BCD, CDA


def lower_leg_angle_to_servo_angle(th1, th2, leg): #c_e_offset, motor_angle, a, b, c, d, e, f, g, h):

    # calculate first 4-bar linkage
    ABC, BCD, CDA = _calculate_4_bar(th2, leg.a, leg.b, leg.c, leg.upper_length)

    # calculate angle between upper leg and motor axis (triangle ADG 180° - (upper_leg to horizontal + motor_angle to horizontal))
    upper_leg_to_motor_angle = np.pi - (th1 + leg.motor_angle)

    # calculate angle GDE for second 4-bar linkage, full circle - all other angles
    th2_new = 2*np.pi -  (CDA + leg.c_e_offset + upper_leg_to_motor_angle)

    # calculate second 4-bar linkage
    DEF, EFG, FGD = _calculate_4_bar(th2_new, leg.e, leg.f, leg.g, leg.h)

    # check if angles are within bounds
    if ABC > leg.ABC_max or ABC < leg.ABC_min:
        raise ValueError(f"ABC angle {ABC} out of bounds: [{leg.ABC_min}, {leg.ABC_max}]")
    if EFG > leg.EFG_max or EFG < leg.EFG_min:
        raise ValueError(f"EFG angle {EFG} out of bounds: [{leg.EFG_min}, {leg.EFG_max}]")
    
    return FGD


# debugging purposes
def servo_angle_to_lower_leg_angle(th1, th2, leg):

    DEF, EFG, FGD = _calculate_4_bar(th2, leg.e, leg.f, leg.g, leg.h)

    upper_leg_to_motor_angle = np.pi - (th1 + leg.motor_angle)
    th2_new = 2*np.pi -  (FGD + leg.c_e_offset + upper_leg_to_motor_angle)

    ABC, BCD, CDA = _calculate_4_bar(th2_new, leg.a, leg.b, leg.c, leg.upper_length)

    return CDA