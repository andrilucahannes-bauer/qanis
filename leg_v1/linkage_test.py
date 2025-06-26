from calculate_linkage_angles import lower_leg_angle_to_servo_angle
from calculate_servo_angles import calc_left_upper_angle_to_dxl, calc_left_lower_angle_to_dxl
from config import Config
from inverse_kinematics import _ik

def main():
    config = Config()
    leg = config.leg_lf
    p = (5, 16)
    point_to_left_motor_angle(leg, p[0], p[1])

def point_to_left_motor_angle(leg, x, z):
    upper_servo_angle, lower_leg_angle = _ik(x, z, leg.upper_length, leg.lower_length)
    lower_servo_angle = lower_leg_angle_to_servo_angle(
            upper_servo_angle,
            lower_leg_angle,
            leg
        )
    
    upper_dxl_angle = calc_left_upper_angle_to_dxl(leg, upper_servo_angle)
    lower_dxl_angle = calc_left_lower_angle_to_dxl(leg, lower_servo_angle)

    # print(f'upper leg angle: {np.rad2deg(upper_servo_angle)}°')
    # print(f'lower leg angle: {np.rad2deg(lower_leg_angle)}°')
    # print(f'lower servo horn angle: {np.rad2deg(lower_servo_angle)}°')

    # print(f'upper dxl angle: {upper_dxl_angle}')
    # print(f'lower dxl angle: {lower_dxl_angle}')

    return upper_dxl_angle, lower_dxl_angle


if __name__ == "__main__":
    main()