import matplotlib.pyplot as plt
from dynamixel_sdk import *
import numpy as np
import time

DEVICE_NAME = 'COM6'
BAUD_RATE = 1000000
PROTOCOL_VERSION = 1.0

DXL_ID_UPPER = 3
DXL_ID_LOWER = 2

MAX_POS = 900
MIN_POS = 200

DEFAULT_POS_UPPER = 800
DEFAULT_POS_LOWER = 500

UPPER_LENGTH = 11.8
LOWER_LENGTH = 8.6

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_CURRENT_POSITION = 36

def ik(x, y, upper, lower):
    d_squared = x ** 2 + y ** 2
    cos_theta2 = (d_squared - upper ** 2 - lower ** 2) / -(2 * upper * lower)
    cos_theta1 = (upper ** 2 + d_squared - lower ** 2) / (2 * upper * np.sqrt(d_squared))
    cos_theta1 = np.clip(cos_theta1, -1.0, 1.0)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = np.arccos(cos_theta2)
    theta1 = np.arctan2(y, x) + np.arccos(cos_theta1)
    return np.degrees(theta1), np.degrees(theta2)

def circular_trajectory(cx, cy, radius, angular_speed, t):
    theta = angular_speed * t
    x = cx + radius * np.cos(theta)
    y = cy + radius * np.sin(theta)
    return x, y

def gait_trajectory(x_fore, x_hind, z_top, z_btm, phi):
    x_off = 0.0
    z_off = 14.0

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
    return x, z

def plot_arm_position(x, y, theta1, theta2):
    theta1_rad = np.radians(theta1)
    theta2_rad = np.radians(theta2)

    x1 = UPPER_LENGTH * np.cos(theta1_rad)
    y1 = UPPER_LENGTH * np.sin(theta1_rad)
    x2 = x1 - LOWER_LENGTH * np.cos(theta1_rad + theta2_rad)
    y2 = y1 - LOWER_LENGTH * np.sin(theta1_rad + theta2_rad)

    fig, ax = plt.subplots(figsize=(8, 8))

    ax.plot([0, x1], [0, y1], 'b-', lw=2, label="Link 1")
    ax.plot([x1, x2], [y1, y2], 'g-', lw=2, label="Link 2")

    ax.plot(x, y, 'ro', label="Target (x, y)")
    ax.plot(x2, y2, 'go', label="End Effector")

    ax.set_xlim(-16, 16)
    ax.set_ylim(0, 24)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    ax.set_title("2-Link Arm with Angle Arcs")

    plt.show()

class Orchestrator:
    def __init__(self, device_name, baud_rate, protocol_version):
        self.protocol_version = protocol_version
        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(protocol_version)

        if not self.port_handler.openPort():
            print("❌ Port konnte nicht geöffnet werden.")
            exit()
        print("✅ Port geöffnet")

        if not self.port_handler.setBaudRate(baud_rate):
            print("❌ Baudrate konnte nicht gesetzt werden.")
            exit()
        print("✅ Baudrate gesetzt")

    def print_result(self, dxl_comm_result, dxl_error, msg):
        if dxl_comm_result != COMM_SUCCESS:
            print(f"❌ result != success: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            exit()
        elif dxl_error != 0:
            print(f"❌ error: {self.packet_handler.getRxPacketError(dxl_error)}")
            exit()
        else:
            print(f"✅ success: {msg}")

    def set_torque(self, dxl_id, torque_state):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, torque_state)
        self.print_result(dxl_comm_result, dxl_error, f"Torque to {torque_state}")

    def set_speed(self, dxl_id, goal_speed):
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_MOVING_SPEED, goal_speed)
        self.print_result(dxl_comm_result, dxl_error, f"speed to {goal_speed}")

    def get_position(self, dxl_id):
        pos, read_result, read_error = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, ADDR_CURRENT_POSITION)
        self.print_result(read_result, read_error, f"Position {dxl_id}: {pos}")

    def move_motor(self, dxl_id, goal_position):
        #print(f'id: {dxl_id}, goal position: {goal_position}')
        write_result, write_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, ADDR_GOAL_POSITION, goal_position)

    def clean_up(self, dxl_list):
        for dxl in dxl_list:
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl, ADDR_TORQUE_ENABLE, 0)
        self.port_handler.closePort()
        print("✅ clean up successful")

    def calc_angle_to_dxl(self, angle):
        return round(angle / 300 * 1023)

    def check_valid_goal_positions(self, goal_position_upper, goal_position_lower):
        print(f'goal_position upper: {goal_position_upper}, goal_position lower: {goal_position_lower}')
        valid_positions =  (MAX_POS >= goal_position_upper >= MIN_POS and
                            MAX_POS >= goal_position_lower >= MIN_POS)
        return valid_positions

    def move_to_pos(self, x, y):
        upper_angle, lower_angle = ik(x, y, UPPER_LENGTH, LOWER_LENGTH)
        print(f'Upper angle: {upper_angle}, Lower angle: {lower_angle}')
        upper_dxl_angle = 818 - self.calc_angle_to_dxl(upper_angle)
        lower_dxl_angle = 1123 - self.calc_angle_to_dxl(lower_angle)
        print(f'Upper dxl angle: {upper_dxl_angle}, Lower dxl angle: {lower_dxl_angle}')
        # plot_arm_position(x, y, upper_angle, lower_angle)

        if self.check_valid_goal_positions(upper_dxl_angle, lower_dxl_angle):
            self.move_motor(DXL_ID_UPPER, upper_dxl_angle)
            self.move_motor(DXL_ID_LOWER, lower_dxl_angle)
        else:
            print("Error moving, probably out of range")

    def move_in_circle(self, x, y, radius, angular_speed, duration, _ticks):
        t_values = np.linspace(0,duration, _ticks)
        positions = [circular_trajectory(x, y, radius, angular_speed, t) for t in t_values]

        for p in positions:
            upper_angle, lower_angle = ik(p[0], p[1], UPPER_LENGTH, LOWER_LENGTH)
            upper_dxl_angle = 818 - self.calc_angle_to_dxl(upper_angle)
            lower_dxl_angle = 1123 - self.calc_angle_to_dxl(lower_angle)
            #if self.check_valid_goal_positions(upper_dxl_angle, lower_dxl_angle):
            self.move_motor(DXL_ID_UPPER, upper_dxl_angle)
            self.move_motor(DXL_ID_LOWER, lower_dxl_angle)
            time.sleep(0.01)
            #else:
             #   print("Out of range")

    def move_gait(self, _ticks, x_fore, x_hind, z_top, z_btm):
        phi = np.linspace(0, 2 * np.pi, _ticks)
        positions = [gait_trajectory(x_fore, x_hind, z_top, z_btm, phi_i) for phi_i in phi]
        pos_5_rounds = positions * 5

        for p in pos_5_rounds:
            upper_angle, lower_angle = ik(p[0], p[1], UPPER_LENGTH, LOWER_LENGTH)
            upper_dxl_angle = 818 - self.calc_angle_to_dxl(upper_angle)
            lower_dxl_angle = 1123 - self.calc_angle_to_dxl(lower_angle)
            #if self.check_valid_goal_positions(upper_dxl_angle, lower_dxl_angle):
            self.move_motor(DXL_ID_UPPER, upper_dxl_angle)
            self.move_motor(DXL_ID_LOWER, lower_dxl_angle)
            time.sleep(0.02)


if __name__ == '__main__':
    orchestrator = Orchestrator(DEVICE_NAME, BAUD_RATE, PROTOCOL_VERSION)

    orchestrator.set_speed(DXL_ID_UPPER, 100)
    orchestrator.set_speed(DXL_ID_LOWER, 100)

    orchestrator.set_torque(DXL_ID_UPPER, 1)
    orchestrator.set_torque(DXL_ID_LOWER, 1)


    while True:

        try:
            user_input = input("Input: ")
            if user_input == '1':
                orchestrator.get_position(DXL_ID_UPPER)
                orchestrator.get_position(DXL_ID_LOWER)

            elif user_input == '2':
                orchestrator.move_to_pos(5,19)

            elif user_input == '3':
                orchestrator.move_to_pos(5,17)

            elif user_input == '4':
                orchestrator.move_to_pos(0,16)

            elif user_input == '5':
                orchestrator.move_to_pos(0, 18.4)

            elif user_input == '6':
                orchestrator.move_to_pos(0, 20.4)

            elif user_input == '0':
                orchestrator.move_to_pos(0, 16)
                time.sleep(2)
                r = 3
                v = 2 * np.pi / 0.5
                d = 3
                ticks = 500
                orchestrator.move_in_circle(0, 16, r, -v, d, ticks)

            elif user_input == '9':
                orchestrator.move_to_pos(0,14)
                time.sleep(2)
                # 36 good tick rate (10° per tick)
                ticks = 72
                orchestrator.move_gait(ticks, 5, 5, 3, 0.5)

            elif user_input == 'x':
                print("🚪 Beenden...")
                orchestrator.clean_up([DXL_ID_UPPER, DXL_ID_LOWER])
                break

            else:
                print("❗ Ungültige Eingabe")

        except KeyboardInterrupt:
            print("\n⛔ Abbruch durch Benutzer.")
            orchestrator.clean_up([DXL_ID_UPPER, DXL_ID_LOWER])


