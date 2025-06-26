from dynamixel_sdk import *

class MotorHandler:
    def __init__(self, device_name, baud_rate, protocol_version):
        self.protocol_version = protocol_version
        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(protocol_version)
        self.ADDR_GOAL_POSITION = 30
        self.ADDR_TORQUE_ENABLE = 24
        self.ADDR_MOVING_SPEED = 32
        self.ADDR_CURRENT_POSITION = 36
        self.ADDR_PRESENT_LOAD = 40

        if not self.port_handler.openPort():
            print("❌ Port konnte nicht geöffnet werden.")
            exit()
        print("✅ Port geöffnet")

        if not self.port_handler.setBaudRate(baud_rate):
            print("❌ Baudrate konnte nicht gesetzt werden.")
            exit()
        print("✅ Baudrate gesetzt")


    def move_motor(self, dxl_id, goal_position):
        write_result, write_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, self.ADDR_GOAL_POSITION, goal_position)


    def move_to_default_position(self, leg):
        pass


    def set_speed(self, dxl_id, goal_speed):
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, self.ADDR_MOVING_SPEED, goal_speed)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"❌ result != success: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            exit()
        elif dxl_error != 0:
            print(f"❌ error: {self.packet_handler.getRxPacketError(dxl_error)}")
            exit()
        else:
            print(f"✅ success: speed set to {goal_speed}")


    def set_torque(self, dxl_id, torque_state):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, torque_state)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"❌ result != success: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            exit()
        elif dxl_error != 0:
            print(f"❌ error: {self.packet_handler.getRxPacketError(dxl_error)}")
            exit()
        else:
            print(f"✅ success: Torque set to {torque_state}")


    def clean_up(self, dxl_list):
        for dxl in dxl_list:
            self.set_torque(dxl, 0)
        self.port_handler.closePort()
        print("✅ clean up successful")


    def read_max_torque(self, dxl_id):
        dxl_present_max_torque, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, 14)

        if dxl_comm_result != COMM_SUCCESS:
            print("Communication error:", self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("Dynamixel error:", self.packet_handler.getRxPacketError(dxl_error))
        else:
            print(f"Max Torque value: {dxl_present_max_torque} ({dxl_present_max_torque / 1023:.1%})")


    def read_current_load(self, dxl_id):
        dxl_present_load, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, self.ADDR_PRESENT_LOAD)

        if dxl_comm_result != COMM_SUCCESS:
            print("Communication error:", self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("Dynamixel error:", self.packet_handler.getRxPacketError(dxl_error))
        else:
            direction = "CW" if dxl_present_load > 1023 else "CCW"
            load_raw = dxl_present_load & 0x3FF
            load_percent = (load_raw / 1023) * 100
            print(f"Load: {load_percent:.1f}% in {direction} direction")
