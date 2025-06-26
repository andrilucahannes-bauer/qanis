import numpy as np
from config import TrajectoryType

class TrajectoryController:
    def __init__(self, trajectory_type, ticks, params):
        self.trajectory_type = trajectory_type
        self.generate_trajectory(trajectory_type, ticks, **params)

    def get_next_position(self, index):
        if index < len(self.trajectory):
            x, z = self.trajectory[index]
            position_matrix = np.zeros((3, 4))
            position_matrix[0, :] = x
            position_matrix[1, :] = 0
            position_matrix[2, :] = z
            return position_matrix
        else:
            raise IndexError("Index out of bounds for trajectory")

    def generate_trajectory(self, trajectory_type, ticks, **kwargs):

        try:
            if trajectory_type == TrajectoryType.LINEAR:
                distance = kwargs['distance']
                z_traj = np.concatenate([np.linspace(0, distance, ticks), np.linspace(distance, 0, ticks)])
                z_off = kwargs["z_off"]
                
                self.trajectory = self._linear_motion(z_traj, z_off)

            elif trajectory_type == TrajectoryType.TROT:
                phi = np.linspace(0, 2 * np.pi, ticks)
                phase_offset = kwargs.get("phase_offset")
                
                if phase_offset is not None and False:
                    phi = (phi + phase_offset) % (2 * np.pi)
                self.trajectory = [
                    self._gait_trajectory(
                        kwargs["x_off"],
                        kwargs["z_off"],
                        kwargs["x_fore"],
                        kwargs["x_hind"],
                        kwargs["z_top"],
                        kwargs["z_btm"],
                        phi_i,
                    )
                    for phi_i in phi
                ]

            else:
                raise ValueError("Unsupported trajectory type")
        except KeyError as e:
            raise KeyError(f"Missing required parameter: {e}")
        except Exception as e:
            raise RuntimeError(f"Error generating trajectory: {e}")

    def _linear_motion(z_traj, z_off):
        return [(0, float(z_off + z)) for z in z_traj]

    def _gait_trajectory(self, x_off, z_off, x_fore, x_hind, z_top, z_btm, phi):
        x = np.where(
            (phi > np.pi / 2) & (phi <= 3 * np.pi / 2),
            x_off - x_fore * np.cos(phi),
            x_off - x_hind * np.cos(phi),
        )

        z = np.where(
            (phi > 0) & (phi <= np.pi),
            z_off + z_top * np.sin(phi),
            z_off + z_btm * np.sin(phi),
        )
        return float(x), float(z)
