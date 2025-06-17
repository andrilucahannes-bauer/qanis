from dynamixel_sdk import *

DEVICE_NAME = 'COM6'
BAUD_RATE = 1000000
PROTOCOL_VERSION = 1.0

DXL_ID = 9
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_POSITION = 36

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MIN_POSITION_VALUE = 0
DXL_MAX_POSITION_VALUE = 1023
DXL_MOVING_STATUS_THRESHOLD = 10

# port und packet öffnen
portHandler = PortHandler(DEVICE_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if portHandler.openPort():
    print("✅ Port geöffnet")
else:
    print("❌ Port konnte nicht geöffnet werden")
    quit()

if portHandler.setBaudRate(BAUD_RATE):
    print("✅ Baudrate gesetzt")
else:
    print("❌ Baudrate konnte nicht gesetzt werden")
    quit()


# torque aktivieren
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)

if dxl_comm_result != COMM_SUCCESS:
    print(f"Kommunikationsfehler: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Fehler: {packetHandler.getRxPacketError(dxl_error)}")
else:
    print("✅ Torque aktiviert")


# move to position
positions = [DXL_MIN_POSITION_VALUE, DXL_MAX_POSITION_VALUE]
for pos in positions:
    print(f"➡️ Bewege zu Position {pos}")
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, pos)
    if dxl_comm_result != COMM_SUCCESS:
        print(f'Error moving: {packetHandler.getTxRxResult(dxl_comm_result)}')


    while True:
        dxl_present_position, _, _ = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
        # print(f"Aktuelle Position: {dxl_present_position}")
        if abs(pos - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD:
            break

# cleanup
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
portHandler.closePort()
