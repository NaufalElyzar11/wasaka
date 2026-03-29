import math
import time
from dynamixel_sdk import *

# --- 1. PENGATURAN DYNAMIXEL ---
ADDR_AX_TORQUE_ENABLE    = 24
ADDR_AX_GOAL_POSITION    = 30
PROTOCOL_VERSION         = 1.0
BAUDRATE                 = 1000000 
DEVICENAME               = 'COM21'

# --- 2. DATA KAKI ROBOT ---
L1, L2, L3 = 3.4, 4.0, 5.45
CENTER_VAL = 512
DEG_TO_DYN = 1023 / 300.0

# Daftar ID Servo berdasarkan tabel Anda: [Coxa, Femur, Tibia]
LEGS = {
    "L1_DepanKiri":   [1, 2, 3],
    "L2_TengahKiri":  [4, 5, 6],
    "L3_BelakangKiri":[7, 8, 9],
    "L4_DepanKanan":  [10, 11, 12],
    "L5_TengahKanan": [13, 14, 15],
    "L6_BelakangKanan":[16, 17, 18]
}

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def calculate_ik(x, y, z):
    """Fungsi IK yang sudah terbukti berhasil di Kaki Kiri"""
    if x == 0 and y == 0: x = 0.001 
    gamma_rad = math.atan2(y, x)
    L_eff = math.sqrt(x**2 + y**2) - L1
    D = math.sqrt(L_eff**2 + z**2)
    
    if D > (L2 + L3): D = (L2 + L3) - 0.01

    alpha1 = math.atan2(z, L_eff)
    alpha2 = math.acos(max(-1.0, min(1.0, (L2**2 + D**2 - L3**2) / (2 * L2 * D))))
    theta_femur_rad = alpha1 - alpha2  # Spider stance
    
    beta = math.acos(max(-1.0, min(1.0, (L2**2 + L3**2 - D**2) / (2 * L2 * L3))))
    theta_tibia_rad = math.pi - beta 

    coxa_deg = math.degrees(gamma_rad)
    femur_deg = math.degrees(theta_femur_rad)
    tibia_deg = math.degrees(theta_tibia_rad)

    coxa_val = int(CENTER_VAL - (coxa_deg * DEG_TO_DYN))
    femur_val = int(CENTER_VAL + (femur_deg * DEG_TO_DYN))
    tibia_val = int(CENTER_VAL - (tibia_deg * DEG_TO_DYN))

    return coxa_val, femur_val, tibia_val

def move_servo(dxl_id, goal_position):
    packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_AX_GOAL_POSITION, goal_position)

# ================= MAIN PROGRAM =================
if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE): quit()

# Kumpulkan semua ID jadi satu list (1 sampai 18)
ALL_IDS = [id for leg in LEGS.values() for id in leg]

# Enable Torque untuk SEMUA servo
for servo_id in ALL_IDS:
    packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_AX_TORQUE_ENABLE, 1)
print("Torque ON untuk 18 Servo. Robot bersiap berdiri!")
time.sleep(1)

# Target Standby: X=7cm (Merentang keluar), Z=5cm (Tinggi bodi)
target_x, target_y, target_z = 7.0, 0.0, 5.0
v_coxa, v_femur, v_tibia = calculate_ik(target_x, target_y, target_z)

print(f"Mengirim nilai IK ke semua kaki: Coxa={v_coxa}, Femur={v_femur}, Tibia={v_tibia}")

# Eksekusi ke semua kaki
for leg_name, ids in LEGS.items():
    move_servo(ids[0], v_coxa)
    move_servo(ids[1], v_femur)
    move_servo(ids[2], v_tibia)

print("\nSilakan periksa fisik robotnya (masih digantung).")
print("Apakah kaki KANAN (Leg 4, 5, 6) posturnya sama persis dengan kaki KIRI?")
time.sleep(10)

# Disable Torque
for servo_id in ALL_IDS:
    packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_AX_TORQUE_ENABLE, 0)
portHandler.closePort()
print("Selesai.")