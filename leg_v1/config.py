import numpy as np


class Config:
    def __init__(self):

        self.leg_lf = LegDimensions(name="left_front", upper_id=1, lower_id=2, inner_id=3, upper_dxl_offset=291, lower_dxl_offset=395, inner_offset=203)
        self.leg_rf = LegDimensions(name="right_front", upper_id=4, lower_id=5, inner_id=6, upper_dxl_offset=698, lower_dxl_offset=744, inner_offset=193)
        self.leg_lh = LegDimensions(name="left_hind", upper_id=7, lower_id=8, inner_id=9, upper_dxl_offset=327, lower_dxl_offset=372, inner_offset=408)
        self.leg_rh = LegDimensions(name="right_hind", upper_id=10, lower_id=11, inner_id=12, upper_dxl_offset=699, lower_dxl_offset=35, inner_offset=311)

        self.legs = [self.leg_lf, self.leg_rf, self.leg_lh, self.leg_rh]

        self.leg_rotation_offset = np.deg2rad(45)
        self.rotation_matrix = np.array([
            [np.cos(self.leg_rotation_offset), -np.sin(self.leg_rotation_offset)],
            [np.sin(self.leg_rotation_offset),  np.cos(self.leg_rotation_offset)]
        ])


class LegDimensions:
    def __init__(self, name, upper_id, lower_id, inner_id, upper_dxl_offset, lower_dxl_offset, inner_offset):
        self.name = name
        self.upper_id = upper_id
        self.lower_id = lower_id
        self.inner_id = inner_id
        self.upper_dxl_offset = upper_dxl_offset
        self.lower_dxl_offset = lower_dxl_offset
        self.inner_offset = inner_offset

        self.upper_length = 13
        self.lower_length = 14.5
        self.c_e_offset = 1.03 # 59° in rad
        self.a = 3.5
        self.b = 14.0
        self.c = 4.5
        self.e = 4.5
        self.f = 4.5
        self.g = 3.5
        self.h = 4.264