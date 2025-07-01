import numpy as np
from config import TrajectoryType
from abc import ABC, abstractmethod
import time


class TrajectoryController(ABC):
    @abstractmethod
    def get_next_position(self):
        pass

    @abstractmethod
    def generate_trajectory(self):
        pass


class WalkTrajectoryController:
    def __init__(self, trajectory_type, params):
        self.trajectory_type = trajectory_type
        self.generate_trajectory(**params)

    def get_next_position(self, index, **kwargs):
        if index < len(self.trajectory):
            position_matrix = np.zeros((3, 4))
            leg_movement_order = kwargs["leg_movement_order"]
            phase_offset = kwargs["gait_phase_offset"]

            for i, leg_index in enumerate(leg_movement_order):
                x, y, z = self.trajectory[
                    (index + (i * phase_offset)) % len(self.trajectory)
                ]
                # TODO change to individual leg
                leg_orientation_x_offset = (
                    kwargs.get("x_off_front", 0)
                    if leg_index < 2
                    else kwargs.get("x_off_hind", 0)
                )
                position_matrix[0, leg_index] = x + leg_orientation_x_offset
                position_matrix[1, leg_index] = y  # optional add offset
                position_matrix[2, leg_index] = z
            return position_matrix
        else:
            raise IndexError("Index out of bounds for trajectory")

    def generate_trajectory(self, **kwargs):

        try:
            ticks_stance = kwargs["ticks_stance"]
            ticks_swing = kwargs["ticks_swing"]
            phi_stance = np.linspace(0, np.pi, ticks_stance)
            phi_swing = np.linspace(np.pi, 2 * np.pi, ticks_swing)
            x_stance = np.linspace(0, kwargs["stance_phase_distance"], ticks_stance)

            stance_trajectory = [
                self._stance_trajectory(
                    kwargs["x_off"],
                    kwargs["z_off"],
                    kwargs["x_fore"],
                    kwargs["x_hind"],
                    kwargs["z_btm"],
                    phi_i,
                )
                for phi_i in phi_stance
            ]
            swing_trajectory = [
                self._swing_trajectory(
                    kwargs["x_off"],
                    kwargs["z_off"],
                    kwargs["x_fore"],
                    kwargs["x_hind"],
                    kwargs["z_top"],
                    phi_i,
                )
                for phi_i in phi_swing
            ]

            self.trajectory = stance_trajectory + swing_trajectory

        except KeyError as e:
            raise KeyError(f"Missing required parameter: {e}")
        except Exception as e:
            raise RuntimeError(f"Error generating trajectory: {e}")

    def _stance_trajectory(self, x_off, z_off, x_fore, x_hind, z_btm, phi):
        x = np.where(
            (phi > np.pi / 2),
            x_off - x_fore * np.cos(phi),
            x_off - x_hind * np.cos(phi),
        )
        y = 0  # TODO
        z = z_off + z_btm * np.sin(phi)
        return float(x), float(y), float(z)

    def _stance_up_trajectory(self, phi):
        x = phi / np.pi
        y = 0
        z = np.sin(phi)
        return float(x), float(y), float(z)

    def _swing_trajectory(self, x_off, z_off, x_fore, x_hind, z_top, phi):
        x = np.where(
            (phi > 3 * np.pi / 2),
            x_off - x_fore * np.cos(phi),
            x_off - x_hind * np.cos(phi),
        )
        y = 0  # TODO
        z = z_off + z_top * np.sin(phi)
        return float(x), float(y), float(z)


class LinearTrajectoryController(TrajectoryController):
    def __init__(self, trajectory_type, params):
        self.trajectory_type = trajectory_type
        self.generate_trajectory(**params)

    def get_next_position(self, tick_index, **kwargs):
        if tick_index < len(self.trajectory):
            position_matrix = np.zeros((3, 4))
            x, y, z = self.trajectory[tick_index]
            # change logic to walk trajectory if different positions for legs are needed
            for leg_index in range(4):
                leg_orientation_x_offset = (
                    kwargs.get("x_off_front", 0)
                    if leg_index < 2
                    else kwargs.get("x_off_hind", 0)
                )
                position_matrix[0, leg_index] = x + leg_orientation_x_offset
                position_matrix[1, leg_index] = y  # optional add offset
                position_matrix[2, leg_index] = z
            return position_matrix
        else:
            raise IndexError("Index out of bounds for trajectory")

    def generate_trajectory(self, **kwargs):
        distance = kwargs["distance"]
        linear_ticks = kwargs["linear_ticks"]
        z_traj = np.concatenate(
            [
                np.linspace(0, distance, linear_ticks),
                np.linspace(distance, 0, linear_ticks),
            ]
        )
        z_off = kwargs["z_off"]

        self.trajectory = self._linear_trajectory(z_traj, z_off)

    def _linear_trajectory(self, z_traj, z_off):
        return [(0, 0, float(z_off + z)) for z in z_traj]

def _stance_up_trajectory(x_phi, z_phi, x_scale=1.0, z_scale=1.0):
    x = x_scale * (x_phi / np.pi)
    y = 0
    z = z_scale * np.sin(z_phi)
    return float(x), float(y), float(z)

phi = np.linspace(0, np.pi, 4)
phi_2 = np.linspace(0, np.pi, 4)

x_scale = 3
z_scale = 1

b = []
for phi_i, phi_i_2 in zip(phi, phi_2):
    b.append(_stance_up_trajectory(phi_i, -phi_i_2, x_scale, z_scale))
print(b)