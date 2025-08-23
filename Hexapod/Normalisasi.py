from dynamixel_sdk import *

DEVICENAME = 'COM21'   # ganti sesuai port, cek di device manager. Kalo di raspi pake ini "/dev/ttyUSB0"
BAUDRATE   = 1000000
PROTOCOL   = 1.0

# ID Kaki (searah jarum jam)
DXL_IDS = [
    (1, 2, 3),     # Kaki 1: Coxa, Femur, Tibia
    (4, 5, 6),     # Kaki 2
    (7, 8, 9),     # Kaki 3
    (10, 11, 12),  # Kaki 4
    (13, 14, 15),  # Kaki 5
    (16, 17, 18)   # Kaki 6
]

# Alamat control table
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30

TORQUE_ENABLE = 1

# Posisi Normalisasi
COXA_HOME  = 512       
FEMUR_HOME = 468  #77.18 derajat
TIBIA_HOME = 463  #75.66 derajat

# Init Port
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL)

if not portHandler.openPort():
    print("❌ Gagal membuka port")
    quit()

if not portHandler.setBaudRate(BAUDRATE):
    print("❌ Gagal set baudrate")
    quit()

# Aktifkan Torque
for leg in DXL_IDS:
    for servo_id in leg:
        packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# Set Normalisasi
for (coxa, femur, tibia) in DXL_IDS:
    packetHandler.write2ByteTxRx(portHandler, coxa,  ADDR_GOAL_POSITION, COXA_HOME)
    packetHandler.write2ByteTxRx(portHandler, femur, ADDR_GOAL_POSITION, FEMUR_HOME)
    packetHandler.write2ByteTxRx(portHandler, tibia, ADDR_GOAL_POSITION, TIBIA_HOME)

print("✅ Hexapod sudah di-normalisasi")

portHandler.closePort()
