from abc import ABC, abstractmethod
from inverse_kinematics_solver import transform_position_to_angles, _ik
from linkage_solver import lower_leg_angle_to_servo_angle
from config import LegSide, LegOrientation, TrajectoryType
from dxl_translation import (
    calc_left_lower_angle_to_dxl,
    calc_left_upper_angle_to_dxl,
    calc_right_lower_angle_to_dxl,
    calc_right_upper_angle_to_dxl,
    calc_lf_rh_inner_angle_to_dxl,
    calc_rf_lh_inner_angle_to_dxl,
)
from transforms3d.euler import euler2mat
import numpy as np
import time


class Controller(ABC):
    @abstractmethod
    def tick(self):
        pass

    @abstractmethod
    def initial_setup(self):
        pass


class GaitController(Controller):
    def __init__(self, trajectory_controller, motor_handler, imu_controller, config):
        self.trajectory_controller = trajectory_controller
        self.motor_handler = motor_handler
        self.imu_controller = imu_controller
        self.config = config

    def tick(self):
        if (
            self.config.gait_config.current_tick
            < self.config.gait_config.total_ticks[
                self.trajectory_controller.trajectory_type
            ]
        ):
            current_tick = self.config.gait_config.current_tick
        else:
            current_tick = self.config.gait_config.current_tick = 0

        kwargs = self.config.gait_config.get_default_gait_params()
        if self.trajectory_controller.trajectory_type == TrajectoryType.WALK:    
            kwargs["gait_phase_offset"] = self.config.gait_config.phase_offset
            kwargs["leg_movement_order"] = self.config.gait_config.leg_movement_order

        leg_positions = self.trajectory_controller.get_next_position(
            current_tick, **kwargs
        )

        # NOTE not used yet
        self.config.gait_config.leg_positions = leg_positions

        for pos, leg in zip(leg_positions.T, self.config.legs):
            upper_dxl_angle, lower_dxl_angle, inner_dxl_angle = _calculate_movement(
                pos, leg
            )
            self._execute_movement(
                upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg
            )

        self.config.gait_config.current_tick += 1

    def _execute_movement(self, upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg):
        self.motor_handler.move_motor(leg.upper_id, upper_dxl_angle)
        self.motor_handler.move_motor(leg.lower_id, lower_dxl_angle)
        # TODO
        self.motor_handler.move_motor(leg.inner_id, leg.horizontal_inner)

    def initial_setup(self):
        for leg in self.config.legs:
            for motor_id in [leg.upper_id, leg.lower_id, leg.inner_id]:
                self.motor_handler.set_speed(motor_id, self.config.default_motor_speed)
                self.motor_handler.set_torque(motor_id, 1)

        positions = self.config.gait_config.default_position
        for pos, leg in zip(positions.T, self.config.legs):
            upper_dxl_angle, lower_dxl_angle, inner_dxl_angle = _calculate_movement(
                pos, leg
            )
            self._execute_movement(
                upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg
            )
            time.sleep(1)
        time.sleep(1)


class BalanceController(Controller):
    def __init__(self, imu_controller, motor_handler, config):
        self.imu_controller = imu_controller
        self.motor_handler = motor_handler
        self.config = config

    def tick(self):
        base_foot_locations = self.config.balance_config.default_position
        offset_matrix = self.config.balance_config.offset_matrix
        global_foot_locations = base_foot_locations + offset_matrix

        roll, pitch, yaw = self.imu_controller.get_euler()
        correction_factor = self.config.balance_config.correction_factor
        max_tilt = self.config.balance_config.max_tilt

        roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
        pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)

        rmat = euler2mat(roll_compensation, pitch_compensation, 0, "sxyz")
        compensated = rmat.T @ global_foot_locations

        reverted_foot_locations = compensated - offset_matrix

        for pos, leg in zip(reverted_foot_locations.T, self.config.legs):
            print(f"pos: {pos}, leg: {leg.name}")
            upper_dxl_angle, lower_dxl_angle, inner_dxl_angle = _calculate_movement(
                pos, leg
            )
            print(
                f"upper={upper_dxl_angle}, lower={lower_dxl_angle}, inner={inner_dxl_angle}"
            )
            print("-" * 20)
            # self._execute_movement(upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg)

    def _execute_movement(self, upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg):
        self.motor_handler.move_motor(leg.upper_id, upper_dxl_angle)
        self.motor_handler.move_motor(leg.lower_id, lower_dxl_angle)
        # TODO
        self.motor_handler.move_motor(leg.inner_id, leg.horizontal_inner)

    def initial_setup(self):
        for leg in self.config.legs:
            for motor_id in [leg.upper_id, leg.lower_id, leg.inner_id]:
                self.motor_handler.set_speed(motor_id, self.config.default_motor_speed)
                self.motor_handler.set_torque(motor_id, 1)

        positions = self.config.balance_config.default_position
        for pos, leg in zip(positions.T, self.config.legs):
            upper_dxl_angle, lower_dxl_angle, inner_dxl_angle = _calculate_movement(
                pos, leg
            )
            print(f"pos: {pos}, leg: {leg.name}")
            print(
                f"upper={upper_dxl_angle}, lower={lower_dxl_angle}, inner={inner_dxl_angle}"
            )
            self._execute_movement(
                upper_dxl_angle, lower_dxl_angle, inner_dxl_angle, leg
            )
            time.sleep(1)
        time.sleep(1)


def _calculate_movement(pos, leg):
    inner_servo_angle, upper_servo_angle, lower_leg_angle = (
        transform_position_to_angles(pos, leg)
    )

    lower_servo_angle = lower_leg_angle_to_servo_angle(
        upper_servo_angle, lower_leg_angle, leg
    )

    print(
        f"upper_servo_angle: {np.degrees(upper_servo_angle)}, lower_servo_angle: {np.degrees(lower_servo_angle)}, inner_servo_angle: {np.degrees(inner_servo_angle)}"
    )

    if leg.side == LegSide.LEFT:
        upper_dxl_angle = calc_left_upper_angle_to_dxl(leg, upper_servo_angle)
        lower_dxl_angle = calc_left_lower_angle_to_dxl(leg, lower_servo_angle)

        if leg.orientation == LegOrientation.FRONT:
            inner_dxl_angle = calc_lf_rh_inner_angle_to_dxl(leg, inner_servo_angle)
        else:
            inner_dxl_angle = calc_rf_lh_inner_angle_to_dxl(leg, inner_servo_angle)
    else:
        upper_dxl_angle = calc_right_upper_angle_to_dxl(leg, upper_servo_angle)
        lower_dxl_angle = calc_right_lower_angle_to_dxl(leg, lower_servo_angle)

        if leg.orientation == LegOrientation.FRONT:
            inner_dxl_angle = calc_rf_lh_inner_angle_to_dxl(leg, inner_servo_angle)
        else:
            inner_dxl_angle = calc_lf_rh_inner_angle_to_dxl(leg, inner_servo_angle)

    return upper_dxl_angle, lower_dxl_angle, inner_dxl_angle
