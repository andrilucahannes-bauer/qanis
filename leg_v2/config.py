import numpy as np
from enum import Enum


class MovementType(Enum):
    GAIT = ("gait",)
    BALANCE = ("balance",)


class TrajectoryType(Enum):
    NO_TRAJECTORY = ("no_trajectory",)
    LINEAR = ("linear",)
    WALK = ("walk",)
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
            leg_index=0,
            name="left_front",
            side=LegSide.LEFT,
            orientation=LegOrientation.FRONT,
            upper_id=1,
            lower_id=2,
            inner_id=3,
            horizontal_inner=203,
        )
        self.leg_rf = LegDimensions(
            leg_index=1,
            name="right_front",
            side=LegSide.RIGHT,
            orientation=LegOrientation.FRONT,
            upper_id=4,
            lower_id=5,
            inner_id=6,
            horizontal_inner=612,
        )
        self.leg_lh = LegDimensions(
            leg_index=2,
            name="left_hind",
            side=LegSide.LEFT,
            orientation=LegOrientation.HIND,
            upper_id=7,
            lower_id=8,
            inner_id=9,
            horizontal_inner=408,
        )
        self.leg_rh = LegDimensions(
            leg_index=3,
            name="right_hind",
            side=LegSide.RIGHT,
            orientation=LegOrientation.HIND,
            upper_id=10,
            lower_id=11,
            inner_id=12,
            horizontal_inner=311,
        )

        self.legs = [self.leg_lf, self.leg_rf, self.leg_lh, self.leg_rh]

        self.default_motor_speed = 200

        self.gait_config = GaitConfig()
        self.sleep_time = {MovementType.GAIT: 0.02, MovementType.BALANCE: 0.02}
        self.balance_config = BalanceConfig()
        self.twerk_config = TwerkConfig()


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

        self.horizontal_position_right_servo = (
            555  # horizontal position of servo horn in dxl steps
        )
        self.horizontal_position_left_servo = (
            662  # horizontal position of servo horn in dxl steps
        )
        self.horizontal_position_upper_left = (
            558  # horizontal position of upper leg in dxl steps
        )
        self.horizontal_position_upper_right = (
            463  # horizontal position of upper leg in dxl steps
        )
        self.dxl_starting_position_offset = (
            71  # 20.7° (50.7° motor angle - 30° dxl starting angle)
        )
        self.left_lower_leg_angle_offset = (
            150  # 662 (our horizontal) - 512 (dxl horizontal)
        )
        self.right_lower_leg_angle_offset = (
            43  # 555 (our horizontal) - 512 (dxl horizontal)
        )
        self.th2_max = 2.38  # 136°
        self.th2_min = 0.75  # 55°
        self.EFG_max = 2.95  # 169° - bad torque conversion
        self.EFG_min = 0.70  # 40°

        self.motor_angle = 0.885  # 50.7°, angle between horizontal and the axis on which the servos are aligned
        self.c_e_offset = 1.745  # 100°, bell crank angle

        self.upper_length = 10
        self.lower_length = 10
        self.a = 4.0
        self.b = 10.0
        self.c = 4.5
        self.e = 4.5
        self.f = 4
        self.g = 3.5
        self.h = 4.264

class TwerkConfig:
    def __init__(self):
        self.motor_speed = 60

class GaitConfig:
    def __init__(self):
        self.x_off = 0.0
        self.z_off = 12.0
        self.x_off_front = 0.0
        self.x_off_hind = 3
        self.x_fore = 2.0
        self.x_hind = 2.0
        self.z_top = 4.0
        self.z_btm = 0.0

        # distance for linear motion
        self.distance = 4.0

        # Number of ticks for trajectory generation. NOTE LINEAR must be even, WALK must be divisible by 4
        self.total_ticks = {TrajectoryType.WALK: 12, TrajectoryType.LINEAR: 2}

        self.ticks_stance = int(self.total_ticks[TrajectoryType.WALK] * (4 / 8))
        self.ticks_swing = int(self.total_ticks[TrajectoryType.WALK] * (4 / 8))
        self.ticks_linear = int(self.total_ticks[TrajectoryType.LINEAR])

        self.stance_phase_distance = 1.0

        # phase offset for walk gait
        self.phase_offset = int(self.total_ticks[TrajectoryType.WALK] * (4 / 8))
        # self.leg_movement_order = [1,3,0,2]  # walk
        self.leg_movement_order = [0, 1, 3, 2]

        self.ticks_per_phase = 4
        self.phase_distance = -2
        self.stance_height_down = 0
        self.stance_height_up = 4
        self.swing_height_up = 3
        self.legs_phase_order = [
            [2, 1, 0, 2],
            [0, 2, 2, 1],
            [1, 2, 2, 0],
            [2, 0, 1, 2],
        ]

        self.current_tick = 0

        self.leg_positions = np.zeros((3, 4))

        # leg order = config.legs
        self.default_position = np.array(
            [
                [-2, -2, 2, 2],  # x
                [0, 0, 0, 0],  # y
                [17.0, 17.0, 17.0, 17.0],  # z
            ]
        )

    def get_default_gait_params(self):
        return {
            "x_off": self.x_off,
            "z_off": self.z_off,
            "x_off_front": self.x_off_front,
            "x_off_hind": self.x_off_hind,
            "x_fore": self.x_fore,
            "x_hind": self.x_hind,
            "z_top": self.z_top,
            "z_btm": self.z_btm,
            "distance": self.distance,
            "linear_ticks": self.ticks_linear,
            "ticks_stance": self.ticks_stance,
            "ticks_swing": self.ticks_swing,
            "ticks_per_phase": self.ticks_per_phase,
            "phase_distance": self.phase_distance,
            "stance_height_down": self.stance_height_down,
            "stance_height_up": self.stance_height_up,
            "swing_height_up": self.swing_height_up,
            "all_legs_phase_order": self.legs_phase_order,
        }


class BalanceConfig:
    def __init__(self):
        self.x_off = 0.0
        self.z_off = 15.0
        self.x_off_front = 0.0
        self.x_off_hind = 0.0

        self.correction_factor = 0.8
        self.max_tilt = 0.4
        self.offset_matrix = np.array(
            [
                [9.5, 9.5, -12.3, -12.3],  # x offsets
                [12.2, -12.2, 12.2, -12.2],  # y offsets
                [0, 0, 0, 0],  # z offsets
            ]
        )

        self.default_position = np.array(
            [
                [0, 0, 0, 0],  # x
                [0, 0, 0, 0],  # y
                [self.z_off, self.z_off, self.z_off, self.z_off],  # z
            ]
        )
