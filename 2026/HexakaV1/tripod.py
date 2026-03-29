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

# Daftar ID Servo: [Coxa, Femur, Tibia]
LEGS = {
    "L1_DepanKiri":   [1, 2, 3],
    "L2_TengahKiri":  [4, 5, 6],
    "L3_BelakangKiri":[7, 8, 9],
    "L4_DepanKanan":  [10, 11, 12],
    "L5_TengahKanan": [13, 14, 15],
    "L6_BelakangKanan":[16, 17, 18]
}

# Pembagian Grup Tripod
GROUP_A = [LEGS["L1_DepanKiri"], LEGS["L3_BelakangKiri"], LEGS["L5_TengahKanan"]]
GROUP_B = [LEGS["L2_TengahKiri"], LEGS["L4_DepanKanan"], LEGS["L6_BelakangKanan"]]
ALL_IDS = [id for leg in LEGS.values() for id in leg]

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def calculate_ik(x, y, z):
    if x == 0 and y == 0: x = 0.001 
    gamma_rad = math.atan2(y, x)
    L_eff = math.sqrt(x**2 + y**2) - L1
    D = math.sqrt(L_eff**2 + z**2)
    
    if D > (L2 + L3): D = (L2 + L3) - 0.01

    alpha1 = math.atan2(z, L_eff)
    alpha2 = math.acos(max(-1.0, min(1.0, (L2**2 + D**2 - L3**2) / (2 * L2 * D))))
    theta_femur_rad = alpha1 - alpha2  
    
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

def send_to_group(group, v_coxa, v_femur, v_tibia):
    for leg in group:
        move_servo(leg[0], v_coxa)
        move_servo(leg[1], v_femur)
        move_servo(leg[2], v_tibia)

# ================= MAIN PROGRAM =================
if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE): quit()

# 1. Nyalakan Torque dan Berdiri Standby
for servo_id in ALL_IDS:
    packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_AX_TORQUE_ENABLE, 1)

print("Berdiri Standby...")
standby_c, standby_f, standby_t = calculate_ik(7.0, 0.0, 5.0)
for leg in LEGS.values():
    move_servo(leg[0], standby_c)
    move_servo(leg[1], standby_f)
    move_servo(leg[2], standby_t)
time.sleep(2)

print("Mulai Berjalan! (Tekan Ctrl+C untuk berhenti)")

# --- PARAMETER LANGKAH (GAIT) YANG DIPERHALUS ---
X_REST = 7.0         
Z_GROUND = 5.0       
LIFT_HEIGHT = 2.5    
STRIDE_LENGTH = 6.0  

# KUNCI KEHALUSAN ADA DI SINI:
SPEED = 0.3         # Sebelumnya 0.3. Semakin kecil, semakin banyak titik (semakin halus).
DELAY = 0.01         # Sebelumnya 0.02. Dipercepat agar aliran data ke servo lebih rapat.

t = 0.0 # Waktu (Fase gelombang)

# Kunci nama kaki untuk memisahkan Grup
GROUP_A_KEYS = ["L1_DepanKiri", "L3_BelakangKiri", "L5_TengahKanan"]

try:
    while True:
        for leg_name, leg_ids in LEGS.items():
            
            if leg_name in GROUP_A_KEYS:
                phase = t                
            else:
                phase = t + math.pi      
                
            y_target = -math.cos(phase) * (STRIDE_LENGTH / 2.0)
            z_target = Z_GROUND - (max(0, math.sin(phase)) * LIFT_HEIGHT)
            
            if "Kanan" in leg_name:
                y_target = -y_target 
                
            v_c, v_f, v_t = calculate_ik(X_REST, y_target, z_target)
            
            move_servo(leg_ids[0], v_c)
            move_servo(leg_ids[1], v_f)
            move_servo(leg_ids[2], v_t)

        t += SPEED
        if t > 2 * math.pi: 
            t -= 2 * math.pi
            
        time.sleep(DELAY) # Jeda loop yang baru

except KeyboardInterrupt:
    print("\nBerhenti berjalan.")

# Kembali Standby
for leg in LEGS.values():
    move_servo(leg[0], standby_c)
    move_servo(leg[1], standby_f)
    move_servo(leg[2], standby_t)
time.sleep(1)

for servo_id in ALL_IDS:
    packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_AX_TORQUE_ENABLE, 0)
portHandler.closePort()
print("Selesai.")