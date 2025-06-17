import numpy as np

# alles ind rad übergeben
def calculate_4_bar(th2, a, b, c, d):
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


def lower_leg_angle_to_servo_angle(th2, c_e_offset, a, b, c, d, e, f, g, h):

    ABC, BCD, CDA = calculate_4_bar(th2, a, b, c, d)
    DEF, EFG, FGD = calculate_4_bar(CDA + c_e_offset, e, f, g, h)

    print(f'ABC: {ABC}, BCD: {BCD}, CDA: {CDA}')
    print(f'DEF: {DEF}, EFG: {EFG}, FGD: {FGD}')

    # test for max angle
    return FGD