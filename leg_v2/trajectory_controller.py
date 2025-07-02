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
        self.trajectory = self.generate_trajectory(**params)

    def get_next_position(self, index):
        if index < len(self.trajectory):
            return self.trajectory[index]

        else:
            raise IndexError("Index out of bounds for trajectory")

    def generate_trajectory(self, **kwargs):
        try:
            ticks_per_phase = kwargs["ticks_per_phase"]
            phase_distance = kwargs["phase_distance"]
            stance_height_down = kwargs["stance_height_down"]
            stance_height_up = kwargs["stance_height_up"]
            swing_height_up = kwargs["swing_height_up"]
            all_legs_phase_order = kwargs["all_legs_phase_order"]

            phi = np.linspace(0, np.pi, ticks_per_phase)

            stance_down = self._stance_trajectory(
                -phi, phi, -phase_distance, stance_height_down
            )
            stance_up = self._stance_trajectory(
                phi, -phi, phase_distance, stance_height_up
            )
            swing = self._stance_trajectory(
                -phi, -phi, 3 * phase_distance, swing_height_up
            )

            # Build trajectory for each leg
            leg_trajectories = []
            for leg_order in all_legs_phase_order:
                leg_trajectory = []
                cumulative_x = 0
                
                for phase in leg_order:
                    if phase == 0:  # swing
                        # Create copy and apply offset
                        swing_segment = np.array(swing[:-1])
                        swing_segment[:, 0] += cumulative_x  # Add to x-coordinate
                        leg_trajectory.extend(swing_segment.tolist())
                        cumulative_x -= 3 * phase_distance
                        
                    elif phase == 1:  # stance_up
                        stance_up_segment = np.array(stance_up[:-1])
                        stance_up_segment[:, 0] += cumulative_x
                        leg_trajectory.extend(stance_up_segment.tolist())
                        cumulative_x += phase_distance
                        
                    else:  # stance_down
                        stance_down_segment = np.array(stance_down[:-1])
                        stance_down_segment[:, 0] += cumulative_x
                        leg_trajectory.extend(stance_down_segment.tolist())
                        cumulative_x += phase_distance
                
                leg_trajectories.append(leg_trajectory)

            # Convert to desired output format (N, 3, 4)
            N = len(leg_trajectories[0])  # Should be ticks_per_phase * 4 - 3
            ges_positions = np.zeros((N, 3, 4))

            for i in range(N):
                for leg_idx in range(4):  # 4 legs: lf, rf, lh, rh
                    ges_positions[i, :, leg_idx] = leg_trajectories[leg_idx][i]
            ges_positions[:, 2, :] += 16
            ges_positions[:, 1, :] -= 1
            #ges_positions[:,0, [0,1]] += 2
            #ges_positions[:,0, [2,3]] -= 2
            return ges_positions[::-1]

        except KeyError as e:
            raise KeyError(f"Missing required parameter: {e}")
        except Exception as e:
            raise RuntimeError(f"Error generating trajectory: {e}")

    def _stance_trajectory(self, phi_x, phi_z, x_scale=1.0, z_scale=1.0):
        traj = []
        for phi_x_i, phi_z_i in zip(phi_x, phi_z):
            x = x_scale * (phi_x_i / np.pi)
            y = 0
            z = z_scale * np.sin(phi_z_i)
            traj.append([float(x), float(y), float(z)])
        return traj


class TwerkTrajectoryController(TrajectoryController):
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
