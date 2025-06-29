import numpy as np


class Config:
    def __init__(self):

        self.leg_lf = LegDimensions(leg_index=0, side="left", orientation="front", name="left_front", upper_id=1, lower_id=2, inner_id=3, horizontal_inner=203)
        self.leg_rf = LegDimensions(leg_index=1, side="right", orientation="front", name="right_front", upper_id=4, lower_id=5, inner_id=6, horizontal_inner=193)
        self.leg_lh = LegDimensions(leg_index=2, side="left", orientation="hind", name="left_hind", upper_id=7, lower_id=8, inner_id=9, horizontal_inner=408)
        self.leg_rh = LegDimensions(leg_index=3, side="right", orientation="hind", name="right_hind", upper_id=10, lower_id=11, inner_id=12, horizontal_inner=311)

        self.legs = [self.leg_lf, self.leg_rf, self.leg_lh, self.leg_rh]

        # not is use yet, maybe delete
        self.leg_rotation_offset = np.deg2rad(45)
        self.rotation_matrix = np.array([
            [np.cos(self.leg_rotation_offset), -np.sin(self.leg_rotation_offset)],
            [np.sin(self.leg_rotation_offset),  np.cos(self.leg_rotation_offset)]
        ])


class LegDimensions:
    def __init__(self, leg_index, name, side, orientation, upper_id, lower_id, inner_id, horizontal_inner):
        self.leg_index = leg_index
        self.name = name
        self.side = side
        self.orientation = orientation
        self.upper_id = upper_id
        self.lower_id = lower_id
        self.inner_id = inner_id

        #temp
        self.horizontal_inner = horizontal_inner # for now till inner servo is used

        self.horizontal_position_servo = 555 # horizontal position of servo horn in dxl steps
        self.horizontal_position_upper_left = 538 # horizontal position of upper leg in dxl steps
        self.horizontal_position_upper_right = 484 # horizontal position of upper leg in dxl steps
        self.dxl_starting_position_offset = 71 # 20.7° (50.7° motor angle - 30° dxl starting angle)
        self.lower_leg_angle_offset = 43 # 555 (our horizontal) - 512 (dxl horizontal)
        self.ABC_max = 2.2 # 122°
        self.ABC_min = 0.77 # 44°
        self.EFG_max = 2.77 # 159° changed from 2.77
        self.EFG_min = 0.52 # 30°

        self.motor_angle = 0.885 # 50.7°, angle between horizontal and the axis on which the servos are aligned
        self.c_e_offset = 1.745 # 59°, bell crank angle

        self.upper_length = 13
        self.lower_length = 13
        self.a = 3.5
        self.b = 14.0
        self.c = 4.5
        self.e = 4.5
        self.f = 4.5
        self.g = 3.5
        self.h = 4.264