import numpy as np
import matplotlib.pyplot as plt
from trajectory import gait_trajectory
from linkage_test import point_to_left_motor_angle
from calculate_linkage_angles import lower_leg_angle_to_servo_angle
from calculate_servo_angles import calc_left_upper_dxl_to_angle, calc_left_lower_dxl_to_angle
from config import Config


upper_length = 13.0
lower_length = 13.0

config = Config()
leg = config.leg_lf
ticks = 72

phi = np.linspace(0, 2 * np.pi, ticks)
positions = [gait_trajectory(5, 15, 2, 2, 3, 1, phi_i) for phi_i in phi]

angles = []

for p in positions:
    angles.append(point_to_left_motor_angle(leg, p[0], p[1]))


foot_positions = []


for upper, lower in angles:
    theta1 = calc_left_upper_dxl_to_angle(leg, upper)

    theta2 = calc_left_lower_dxl_to_angle(leg, lower) # here servo because reverse

    lower_leg_angle = lower_leg_angle_to_servo_angle(theta1, theta2, leg)

    x = upper_length * np.cos(theta1) + lower_length * np.cos(
        theta1 + lower_leg_angle
    )
    z = upper_length * np.sin(theta1) + lower_length * np.sin(
        theta1 + lower_leg_angle
    )
    foot_positions.append((x, z))

# Plot trajectory
x_vals, z_vals = zip(*foot_positions)
plt.plot(x_vals, z_vals)
plt.xlabel("X (forward)")
plt.ylabel("Z (up)")
plt.title("Foot trajectory")
plt.axis("equal")
plt.grid()
plt.show()