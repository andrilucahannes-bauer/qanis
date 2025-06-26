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
from transform3d.euler import euler2mat
import numpy as np
import time

def main():
    try:
        mh = MotorHandler("COM7", 1000000, 1.0)
        imu = IMU()
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

            #mh.move_motor(leg.upper_id, upper_dxl_angle)
            #mh.move_motor(leg.lower_id, lower_dxl_angle)
        
        # foot positions
        rotated_foot_locations = np.array([
            [0, 0, 0, 0],     # x
            [0, 0, 0, 0],     # y
            [15, 15, 15, 15]  # z
        ])  # Shape: (3, 4)

        while True:
            yaw,pitch,roll = imu.get_gyro_data()
            correction_factor = 0.8
            max_tilt = 0.4

            roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
            pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)

            rmat = euler2mat(roll_compensation, pitch_compensation, 0)
            compensated = rmat.T @ rotated_foot_locations

            # change logic to handle a matrix
            # move motors

            time.sleep(1)
    except KeyboardInterrupt:
        print("❌ Interrupted by user")
        mh.clean_up(
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
        mh.clean_up(
            [
                id
                for leg in config.legs
                for id in [leg.upper_id, leg.lower_id, leg.inner_id]
            ]
        )
if __name__ == "__main__":
    main()
