import numpy as np
import time
from inverse_kinematics import ik
from handle_motor_movement import MotorHandler
from trajectory import linear_motion, gait_trajectory
from config import Config
from calculate_linkage_angles import lower_leg_angle_to_servo_angle
from calculate_servo_angles import calc_angle_to_dxl

def walk(x_off, z_off, x_fore, x_hind, z_top, z_btm, circles, ticks, config, motor_handler):
    phi = np.linspace(2 * np.pi, 0, ticks)
    positions = [gait_trajectory(x_off, z_off, x_fore, x_hind, z_top, z_btm, phi_i) for phi_i in phi]

    traj = np.concatenate([
        np.linspace(0, 5, ticks // 4),
        np.linspace(5, 0, ticks // 4)[1:]
    ])


    #positions = linear_motion(traj, z_off)
    pos_multiple_circles = positions * circles

    pos_test = [(0, 18)]
    # change to calculated rotation --> use IMU
    # rotated_positions = [config.rotation_matrix @ np.array([x, z]) for (x, z) in pos_multiple_circles]

    #motor_handler.move_to_default_position(config.leg_lf)
    #motor_handler.move_to_default_position(config.leg_rf)
    #motor_handler.move_to_default_position(config.leg_lh)
    #motor_handler.move_to_default_position(config.leg_rh)
    #time.sleep(3)

    all_dxl_angles = {"left_front": [], "right_front": [], "left_hind": [], "right_hind": []}

    try:
        for p in pos_test:
            for leg in config.legs:
                upper_servo_angle, lower_leg_angle = ik(p[0], p[1], leg.upper_length, leg.lower_length)
                lower_servo_angle = lower_leg_angle_to_servo_angle(
                    lower_leg_angle,
                    leg.c_e_offset,
                    leg.a,
                    leg.b,
                    leg.c,
                    leg.upper_length,
                    leg.e,
                    leg.f,
                    leg.g,
                    leg.h
                )
                #print(f'upper_servo_angle: {upper_servo_angle}, lower_servo_angle: {lower_servo_angle}')
                upper_dxl_angle = calc_angle_to_dxl(upper_servo_angle) + leg.upper_dxl_offset
                lower_dxl_angle = calc_angle_to_dxl(lower_servo_angle) + leg.lower_dxl_offset
                print(f'{leg.name} upper angle: {upper_dxl_angle}, lower angle: {lower_dxl_angle}')
                if leg.name == "left_front":
                    motor_handler.move_motor(leg.upper_id, upper_dxl_angle)
                    motor_handler.move_motor(leg.lower_id, lower_dxl_angle)
                #motor_handler.read_current_load(leg.upper_id)
                #motor_handler.read_current_load(leg.lower_id)
                #motor_handler.read_current_load(leg.inner_id)

                all_dxl_angles[leg.name].append((upper_dxl_angle, lower_dxl_angle))
                # if self.check_valid_goal_positions(upper_dxl_angle, lower_dxl_angle):
                #motor_handler.move_motor(leg.upper_id, upper_dxl_angle)
                #motor_handler.move_motor(leg.lower_id, lower_dxl_angle)
                time.sleep(1)
        print(f'all_dxl_angles: {all_dxl_angles}')
    except KeyboardInterrupt:
        print("❌ Interrupted by user")
        motor_handler.clean_up([id for leg in config.legs for id in [leg.upper_id, leg.lower_id, leg.inner_id]])
        print("✅ Clean up successful")
    finally:
        motor_handler.clean_up([id for leg in config.legs for id in [leg.upper_id, leg.lower_id, leg.inner_id]])


if __name__ == '__main__':
    conf = Config()
    mh = MotorHandler('COM6', 1000000, 1.0)
    for leg in conf.legs:
        mh.set_speed(leg.upper_id, 100)
        mh.set_speed(leg.lower_id, 100)
        mh.set_speed(leg.inner_id, 100)
        mh.set_torque(leg.upper_id, 1)
        mh.set_torque(leg.lower_id, 1)
        mh.set_torque(leg.inner_id, 1)
        mh.read_max_torque(leg.upper_id)
        mh.read_max_torque(leg.lower_id)
        mh.read_max_torque(leg.inner_id)

    walk(
        x_off=-5.0,
        z_off=15.0,
        x_fore=5,
        x_hind=5,
        z_top=5,
        z_btm=1,
        circles=5,
        ticks=36,
        config=conf,
        motor_handler=mh
    )
