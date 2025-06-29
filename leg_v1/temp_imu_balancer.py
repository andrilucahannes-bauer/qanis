from handle_motor_movement import MotorHandler
from imu import IMU
from config import Config
from inverse_kinematics import _ik
from calculate_linkage_angles import lower_leg_angle_to_servo_angle
from calculate_servo_angles import (
    calc_left_lower_angle_to_dxl,
    calc_left_upper_angle_to_dxl,
    calc_right_lower_angle_to_dxl,
    calc_right_upper_angle_to_dxl,
)
from transforms3d.euler import euler2mat
import numpy as np
import time

def main():
    try:
        mh = MotorHandler("/dev/ttyUSB0", 1000000, 1.0)
        imu = IMU(sample_freq=100.0, beta=0.1)
        config = Config()
        for leg in config.legs:
            base_pos = (0, 15)
            upper_servo_angle, lower_leg_angle = _ik(
                base_pos[0], base_pos[1], leg.upper_length, leg.lower_length
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

            mh.move_motor(leg.upper_id, upper_dxl_angle)
            mh.move_motor(leg.lower_id, lower_dxl_angle)
            mh.move_motor(leg.inner_id, leg.horizontal_inner)
            time.sleep(3)
        
        # foot positions
        rotated_foot_locations = np.array([
            [0, 0, 0, 0],     # x
            [0, 0, 0, 0],     # y
            [15, 15, 15, 15]  # z
        ])  # Shape: (3, 4)


        balance_offset_matrix = np.array([
            [9.5,   9.5,    -12.3,  -12.3],     # x offsets
            [12.2,  -12.2,  12.2,   -12.2],     # y offsets
            [0,     0,      0,      0]          # z offsets
        ])  # Shape: (3, 4)

        foot_locations = rotated_foot_locations + balance_offset_matrix

        print(f"Foot Locations:\n{foot_locations}")

        while True:
            roll, pitch, yaw = imu.get_euler()
            correction_factor = 0.8
            max_tilt = 0.4

            roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
            pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)

            rmat = euler2mat(roll_compensation, pitch_compensation, 0, 'sxyz')
            compensated = rmat.T @ foot_locations

            # change logic to handle a matrix
            # move motors

            offset_reversed = compensated - balance_offset_matrix

            print(f"Roll: {roll:+6.2f}, Pitch: {pitch:+6.2f}, Yaw: {yaw:+6.2f}")
            print(f"actual foot position:\n{offset_reversed}")

            time.sleep(1 / 100.0)

    except KeyboardInterrupt:
        print("❌ Interrupted by user")
    except ValueError as e:
        print(f"❌ angle out of bounds error: {e}")
    finally:
        pass
        # mh.clean_up(
        #     [
        #         id
        #         for leg in config.legs
        #         for id in [leg.upper_id, leg.lower_id, leg.inner_id]
        #     ]
        # )

if __name__ == "__main__":
    main()
