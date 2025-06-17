import numpy as np

def linear_motion(z_traj, z_off):
    return [(0, float(z_off + z)) for z in z_traj]

def gait_trajectory(x_off, z_off, x_fore, x_hind, z_top, z_btm, phi):
    x = np.where(
        (phi > np.pi / 2) & (phi <= 3 * np.pi / 2),
        x_off - x_fore * np.cos(phi),
        x_off - x_hind * np.cos(phi)
    )

    z = np.where(
        (phi > 0) & (phi <= np.pi),
        z_off + z_top * np.sin(phi),
        z_off + z_btm * np.sin(phi)
    )
    return float(x), float(z)