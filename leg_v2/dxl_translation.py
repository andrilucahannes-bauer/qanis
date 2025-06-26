import numpy as np

_STEP_RAD = 0.00507 # 0.29° in rad (1 step)

def _calc_angle_to_dxl(angle_rad):
    return round(angle_rad / _STEP_RAD)

def calc_left_upper_angle_to_dxl(leg, angle_rad):
    dxl_angle = _calc_angle_to_dxl(angle_rad)
    return leg.horizontal_position_upper_left - dxl_angle

def calc_right_upper_angle_to_dxl(leg, angle_rad):
    dxl_angle = _calc_angle_to_dxl(angle_rad)
    return leg.horizontal_position_upper_right + dxl_angle

def calc_left_lower_angle_to_dxl(leg, angle_rad):
    dxl_angle = _calc_angle_to_dxl(angle_rad)
    return leg.dxl_starting_position_offset + leg.lower_leg_angle_offset + dxl_angle

def calc_right_lower_angle_to_dxl(leg, angle_rad):
    dxl_angle = _calc_angle_to_dxl(angle_rad)
    return 1023 - (leg.dxl_starting_position_offset + dxl_angle) + leg.lower_leg_angle_offset


# dxl to angle: (left working, right not tested yet)
def _calc_dxl_to_angle(dxl_steps: int) -> float:
    return dxl_steps * _STEP_RAD

def calc_left_upper_dxl_to_angle(leg, dxl_val: int) -> float:
    dxl_steps = leg.horizontal_position_upper_left - dxl_val
    return _calc_dxl_to_angle(dxl_steps)

def calc_right_upper_dxl_to_angle(leg, dxl_val: int) -> float:
    dxl_steps = dxl_val - leg.horizontal_position_upper_right
    return _calc_dxl_to_angle(dxl_steps)

def calc_left_lower_dxl_to_angle(leg, dxl_val: int) -> float:
    base = leg.dxl_starting_position_offset + leg.lower_leg_angle_offset
    dxl_steps = dxl_val - base
    return _calc_dxl_to_angle(dxl_steps)

def calc_right_lower_dxl_to_angle(leg, dxl_val: int) -> float:
    # from: dxl_val = 1023 - (offset + dxl_steps) + lower_offset
    # rearranged: offset + dxl_steps = 1023 + lower_offset - dxl_val
    dxl_steps = (1023 + leg.lower_leg_angle_offset - dxl_val) - leg.dxl_starting_position_offset
    return _calc_dxl_to_angle(dxl_steps)
