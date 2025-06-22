import numpy as np
import time
from inverse_kinematics import ik
from handle_motor_movement import MotorHandler
from trajectory import linear_motion, gait_trajectory, gait_trajectory_with_offset
from config import Config
from calculate_linkage_angles import lower_leg_angle_to_servo_angle
from calculate_servo_angles import (
    calc_left_upper_angle_to_dxl,
    calc_right_upper_angle_to_dxl,
    calc_left_lower_angle_to_dxl,
    calc_right_lower_angle_to_dxl,
)


def walk(
    x_off, z_off, x_fore, x_hind, z_top, z_btm, circles, ticks, config, motor_handler
):
    phi = np.linspace(0, 2 * np.pi, ticks)
    positions_0 = [gait_trajectory(0, 17, 2, 2, 1, 3, phi_i) for phi_i in phi]
    positions_1 = [
        gait_trajectory_with_offset(0, 17, 2, 2, 1, 3, phi_i) for phi_i in phi
    ]
    traj = np.concatenate(
        [np.linspace(0, 5, ticks // 4), np.linspace(5, 0, ticks // 4)[1:]]
    )
    print(positions_0)
    # positions = linear_motion(traj, z_off)
    pos_multiple_circles_0 = positions_0 * circles
    pos_multiple_circles_1 = positions_1 * circles
    pos_test = [(0, 15), (0, 20), (0, 10)]
    # change to calculated rotation --> use IMU
    # rotated_positions = [config.rotation_matrix @ np.array([x, z]) for (x, z) in pos_multiple_circles]

    try:
        for (x_0, z_0), (x_1, z_1) in zip(
            pos_multiple_circles_0, pos_multiple_circles_1
        ):
            for leg in config.legs:
                if leg.leg_index == 1 or leg.leg_index == 4:
                    upper_servo_angle, lower_leg_angle = ik(
                        x_0, z_0, leg.upper_length, leg.lower_length
                    )
                    lower_servo_angle = lower_leg_angle_to_servo_angle(
                        upper_servo_angle, lower_leg_angle, leg
                    )
                else:
                    upper_servo_angle, lower_leg_angle = ik(
                        x_1, z_1, leg.upper_length, leg.lower_length
                    )
                    lower_servo_angle = lower_leg_angle_to_servo_angle(
                        upper_servo_angle, lower_leg_angle, leg
                    )
                if leg.leg_index % 2 == 0:
                    upper_dxl_angle = calc_right_upper_angle_to_dxl(
                        leg, upper_servo_angle
                    )
                    lower_dxl_angle = calc_right_lower_angle_to_dxl(
                        leg, lower_servo_angle
                    )
                else:
                    upper_dxl_angle = calc_left_upper_angle_to_dxl(
                        leg, upper_servo_angle
                    )
                    lower_dxl_angle = calc_left_lower_angle_to_dxl(
                        leg, lower_servo_angle
                    )
                motor_handler.move_motor(leg.upper_id, upper_dxl_angle)
                motor_handler.move_motor(leg.lower_id, lower_dxl_angle)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("❌ Interrupted by user")
        motor_handler.clean_up(
            [
                id
                for leg in config.legs
                for id in [leg.upper_id, leg.lower_id, leg.inner_id]
            ]
        )
        print("✅ Clean up successful")
    except ValueError as e:
        print(f"❌ angle out of bounds error: {e}")
    finally:
        motor_handler.clean_up(
            [
                id
                for leg in config.legs
                for id in [leg.upper_id, leg.lower_id, leg.inner_id]
            ]
        )


if __name__ == "__main__":
    conf = Config()
    mh = MotorHandler("COM7", 1000000, 1.0)
    for leg in conf.legs:
        mh.set_speed(leg.upper_id, 20)
        mh.set_speed(leg.lower_id, 20)
        mh.set_speed(leg.inner_id, 20)
        mh.set_torque(leg.upper_id, 1)
        mh.set_torque(leg.lower_id, 1)
        mh.set_torque(leg.inner_id, 1)
        mh.read_max_torque(leg.upper_id)
        mh.read_max_torque(leg.lower_id)
        mh.read_max_torque(leg.inner_id)

    walk(
        x_off=0,
        z_off=0,
        x_fore=0,
        x_hind=0,
        z_top=0,
        z_btm=0,
        circles=5,
        ticks=10,
        config=conf,
        motor_handler=mh,
    )
