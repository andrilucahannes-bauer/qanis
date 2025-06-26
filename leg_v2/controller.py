from abc import ABC, abstractmethod
from inverse_kinematics_solver import transform_position_to_angles, _ik
from linkage_solver import lower_leg_angle_to_servo_angle
from config import LegSide
from dxl_translation import (
    calc_left_lower_angle_to_dxl,
    calc_left_upper_angle_to_dxl,
    calc_right_lower_angle_to_dxl,
    calc_right_upper_angle_to_dxl,
)
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


        
        # NOTE currently all legs do the same, change later
        leg_positions = self.trajectory_controller.get_next_position(current_tick)

        # NOTE not used yet
        self.config.gait_config.leg_positions = leg_positions

        for pos, leg in zip(leg_positions.T, self.config.legs):
            self._execute_movement(pos, leg)

        self.config.gait_config.current_tick += 1

    def _execute_movement(self, pos, leg):
        inner_servo_angle, upper_servo_angle, lower_leg_angle = (
            transform_position_to_angles(pos, leg)
        )

        lower_servo_angle = lower_leg_angle_to_servo_angle(
            upper_servo_angle, lower_leg_angle, leg
        )

        if leg.side == LegSide.LEFT:
            upper_dxl_angle = calc_left_upper_angle_to_dxl(leg, upper_servo_angle)
            lower_dxl_angle = calc_left_lower_angle_to_dxl(leg, lower_servo_angle)
        else:
            upper_dxl_angle = calc_right_upper_angle_to_dxl(leg, upper_servo_angle)
            lower_dxl_angle = calc_right_lower_angle_to_dxl(leg, lower_servo_angle)
        
        self.motor_handler.move_motor(leg.upper_id, upper_dxl_angle)
        self.motor_handler.move_motor(leg.lower_id, lower_dxl_angle)
        # TODO change inner motor to be variable
        self.motor_handler.move_motor(leg.inner_id, leg.horizontal_inner)

    
    def initial_setup(self):
        for leg in self.config.legs:
            for motor_id in [leg.upper_id, leg.lower_id, leg.inner_id]:
                self.motor_handler.set_speed(motor_id, self.config.default_motor_speed)
                self.motor_handler.set_torque(motor_id, 1)

        positions = self.config.gait_config.default_position
        for pos, leg in zip(positions.T, self.config.legs):
            self._execute_movement(pos, leg)
            time.sleep(1)
        time.sleep(3)


class BalanceController(Controller):
    def __init__(self, imu_controller, motor_handler, config):
        self.imu_controller = imu_controller
        self.motor_handler = motor_handler
        self.config = config

    def tick(self):
        pass

    def initial_setup(self):
        pass