import numpy as np
from enum import Enum


class MovementType(Enum):
    GAIT = ("gait",)
    BALANCE = ("balance",)


class TrajectoryType(Enum):
    LINEAR = ("linear",)
    TROT = ("trot",)

class LegSide(Enum):
    LEFT = "left"
    RIGHT = "right"

class LegOrientation(Enum):
    FRONT = "front"
    HIND = "hind"


class Config:
    def __init__(self):

        self.leg_lf = LegDimensions(
            leg_index=1,
            name="left_front",
            side=LegSide.LEFT,
            orientation=LegOrientation.FRONT,
            upper_id=1,
            lower_id=2,
            inner_id=3,
            horizontal_inner=203,
        )
        self.leg_rf = LegDimensions(
            leg_index=2,
            name="right_front",
            side=LegSide.RIGHT,
            orientation=LegOrientation.FRONT,
            upper_id=4,
            lower_id=5,
            inner_id=6,
            horizontal_inner=193,
        )
        self.leg_lh = LegDimensions(
            leg_index=3,
            name="left_hind",
            side=LegSide.LEFT,
            orientation=LegOrientation.HIND,
            upper_id=7,
            lower_id=8,
            inner_id=9,
            horizontal_inner=408,
        )
        self.leg_rh = LegDimensions(
            leg_index=4,
            name="right_hind",
            side=LegSide.RIGHT,
            orientation=LegOrientation.HIND,
            upper_id=10,
            lower_id=11,
            inner_id=12,
            horizontal_inner=311,
        )

        self.legs = [self.leg_lf, self.leg_rf, self.leg_lh, self.leg_rh]

        self.default_motor_speed = 20

        self.gait_config = GaitConfig()
        self.sleep_time = {MovementType.GAIT: 0.02, MovementType.BALANCE: 0.1}


class LegDimensions:
    def __init__(
        self,
        leg_index,
        name,
        side,
        orientation,
        upper_id,
        lower_id,
        inner_id,
        horizontal_inner,
    ):
        self.leg_index = leg_index
        self.name = name
        self.side = side
        self.orientation = orientation
        self.upper_id = upper_id
        self.lower_id = lower_id
        self.inner_id = inner_id

        # temp
        self.horizontal_inner = horizontal_inner  # for now till inner servo is used

        self.horizontal_position_servo = (
            555  # horizontal position of servo horn in dxl steps
        )
        self.horizontal_position_upper_left = (
            538  # horizontal position of upper leg in dxl steps
        )
        self.horizontal_position_upper_right = (
            484  # horizontal position of upper leg in dxl steps
        )
        self.dxl_starting_position_offset = (
            71  # 20.7° (50.7° motor angle - 30° dxl starting angle)
        )
        self.lower_leg_angle_offset = 43  # 555 (our horizontal) - 512 (dxl horizontal)
        self.ABC_max = 2.13  # 122°
        self.ABC_min = 0.77  # 44°
        self.EFG_max = 2.77  # 159° changed from 2.77
        self.EFG_min = 0.52  # 30°

        self.motor_angle = 0.885  # 50.7°, angle between horizontal and the axis on which the servos are aligned
        self.c_e_offset = 1.745  # 100°, bell crank angle

        self.upper_length = 13
        self.lower_length = 13
        self.a = 3.5
        self.b = 14.0
        self.c = 4.5
        self.e = 4.5
        self.f = 4.5
        self.g = 3.5
        self.h = 4.264


class GaitConfig:
    def __init__(self):
        self.x_off = 0.0
        self.z_off = 17.0
        self.x_fore = 2.0
        self.x_hind = 2.0
        self.z_top = 1
        self.z_btm = 3

        # distance for linear motion
        self.distance = 5.0

        # phase offset for trot gait
        self.phase_offset = np.pi / 2

        # Number of ticks for trajectory generation. NOTE LINEAR must be even
        self.total_ticks = {TrajectoryType.TROT: 20, TrajectoryType.LINEAR: 10}

        self.current_tick = 0

        self.leg_positions = np.zeros((3, 4))

        self.default_position = np.array(
            [
                [0, 0, 0, 0],                                       # x
                [0, 0, 0, 0],                                       # y
                [self.z_off, self.z_off, self.z_off, self.z_off],   # z
            ]
        )

    def get_default_gait_params(self):
        return {
            "x_off": self.x_off,
            "z_off": self.z_off,
            "x_fore": self.x_fore,
            "x_hind": self.x_hind,
            "z_top": self.z_top,
            "z_btm": self.z_btm,
            "distance": self.distance,
        }
