from dynamixel_sdk import *

PORT_NAME = "COM6"
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0

CURRENT_ID = 12
NEW_ID = 5
ADDR_MX_ID = 3
ADDR_TORQUE_ENABLE = 24

portHandler = PortHandler(PORT_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# Disable torque before changing ID
packetHandler.write1ByteTxRx(portHandler, CURRENT_ID, ADDR_TORQUE_ENABLE, 0)

# Write new ID to address 3
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, CURRENT_ID, ADDR_MX_ID, NEW_ID)

if dxl_comm_result != COMM_SUCCESS:
    print(f"❌ Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"❌ Packet Error: {packetHandler.getRxPacketError(dxl_error)}")
else:
    print(f"✅ ID successfully changed to {NEW_ID}")

portHandler.closePort()
