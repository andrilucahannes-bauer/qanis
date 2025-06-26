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
positions = [(-1.2246467991473532e-16, 17.0), (0.6493989384093669, 16.89163448340127), (1.2284254253793356, 16.578281018792786), (1.674332956525057, 16.093896316244855), (1.938800531878661, 15.490970974281598), (1.9931689860133397, 14.917420654527668), (1.831546653310115, 14.598304575347031), (1.4714478213462636, 14.322718428374259), (0.9518947860741467, 14.12052624879351), (0.32918918056146906, 14.013638696597278), (-0.32918918056146834, 14.013638696597278), (-0.951894786074146, 14.12052624879351), (-1.4714478213462625, 14.322718428374259), (-1.8315466533101146, 14.59830457534703), (-1.9931689860133397, 14.917420654527668), (-1.938800531878661, 15.490970974281598), (-1.6743329565250573, 16.09389631624485), (-1.2284254253793359, 16.578281018792786), (-0.6493989384093671, 16.89163448340127), (-1.2246467991473532e-16, 17.0)]

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