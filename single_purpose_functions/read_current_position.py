from dynamixel_sdk import *

DXL_ID = 11
BAUDRATE = 1000000
DEVICENAME = 'COM7'

ADDR_PRESENT_POSITION = 36

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(1.0)

if not portHandler.openPort():
    print("❌ Port konnte nicht geöffnet werden.")
    exit()

if not portHandler.setBaudRate(BAUDRATE):
    print("❌ Baudrate konnte nicht gesetzt werden.")
    exit()

dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
    portHandler, DXL_ID, ADDR_PRESENT_POSITION)

if dxl_comm_result != COMM_SUCCESS:
    print(f"❌ Kommunikationsfehler: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"❗ Servo-Fehler: {packetHandler.getRxPacketError(dxl_error)}")
else:
    print(f"✅ Aktuelle Position: {dxl_present_position} (0–1023 entspricht 0–300 Grad)")

portHandler.closePort()
