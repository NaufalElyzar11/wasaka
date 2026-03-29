import math
import time
from dynamixel_sdk import *

# --- 1. PENGATURAN DYNAMIXEL AX-12A ---
ADDR_AX_TORQUE_ENABLE    = 24
ADDR_AX_GOAL_POSITION    = 30
PROTOCOL_VERSION         = 1.0
BAUDRATE                 = 1000000 
DEVICENAME               = 'COM21'   # GANTI INI!

TORQUE_ENABLE            = 1
TORQUE_DISABLE           = 0

# --- 2. SPESIFIKASI & ID KAKI 1 (Kiri Depan) ---
ID_COXA  = 1
ID_FEMUR = 2
ID_TIBIA = 3

L1 = 3.4   # cm
L2 = 4.0   # cm
L3 = 5.45  # cm
CENTER_VAL = 512
DEG_TO_DYN = 1023 / 300.0

# Inisialisasi SDK
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def calculate_ik(x, y, z):
    """Menghitung IK dengan Konfigurasi Spider/Hexapod (Lutut di Atas)"""
    if x == 0 and y == 0: x = 0.001 
    
    gamma_rad = math.atan2(y, x)
    L_xy = math.sqrt(x**2 + y**2)
    L_eff = L_xy - L1
    D = math.sqrt(L_eff**2 + z**2)
    
    if D > (L2 + L3):
        print(f"WARNING: Target (X:{x}, Z:{z}) terlalu jauh! Diset ke batas maksimal.")
        D = (L2 + L3) - 0.01

    alpha1 = math.atan2(z, L_eff)
    cos_alpha2 = max(-1.0, min(1.0, (L2**2 + D**2 - L3**2) / (2 * L2 * D)))
    alpha2 = math.acos(cos_alpha2)
    
    # --- PERBAIKAN KRUSIAL DI SINI ---
    # Mengubah ke (alpha1 - alpha2) agar lutut menekuk ke ATAS (Spider Stance)
    theta_femur_rad = alpha1 - alpha2
    
    cos_beta = max(-1.0, min(1.0, (L2**2 + L3**2 - D**2) / (2 * L2 * L3)))
    beta = math.acos(cos_beta)
    theta_tibia_rad = math.pi - beta 

    coxa_deg = math.degrees(gamma_rad)
    femur_deg = math.degrees(theta_femur_rad)
    tibia_deg = math.degrees(theta_tibia_rad)

    # --- PENERAPAN POLARITAS FINAL ---
    # Coxa: -X menjauhi bodi
    coxa_val = int(CENTER_VAL - (coxa_deg * DEG_TO_DYN))
    
    # Femur: +Turun, -Naik (Kini nilainya akan sering minus/di bawah 512 agar paha terangkat)
    femur_val = int(CENTER_VAL + (femur_deg * DEG_TO_DYN))
    
    # Tibia: Dikembalikan ke minus (-) agar ujung kaki menekuk ke BAWAH menyentuh tanah
    tibia_val = int(CENTER_VAL - (tibia_deg * DEG_TO_DYN))

    # Proteksi Batasan Fisik
    femur_val = max(191, min(854, femur_val))
    tibia_val = max(82, min(929, tibia_val))

    return coxa_val, femur_val, tibia_val

# ... [Biarkan kode PortHandler dan PacketHandler seperti sebelumnya] ...

def move_servo(dxl_id, goal_position):
    packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_AX_GOAL_POSITION, goal_position)

# ================= MAIN PROGRAM =================
if not portHandler.openPort(): quit()
if not portHandler.setBaudRate(BAUDRATE): quit()

# Enable Torque
for servo_id in [ID_COXA, ID_FEMUR, ID_TIBIA]:
    packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
print("Torque ON. Robot bersiap melangkah!")
time.sleep(1)

# --- LINTASAN LANGKAH (WAYPOINTS) ---
# Format: [X, Y, Z]
# Y kita buat 0.0 dulu agar kakinya melangkah lurus ke depan
# --- LINTASAN LANGKAH (WAYPOINTS) YANG DIPERBAIKI ---
# Format: [X, Y, Z]
step_sequence = [
    [5.0, 0.0, 5.0],  # 1. Posisi awal (belakang, menapak di Z=5)
    [5.0, 0.0, 2.0],  # 2. Angkat kaki (Z naik menjadi 2cm dari lantai)
    [9.0, 0.0, 2.0],  # 3. Ayun ke depan (X maju ke 9cm, kaki masih melayang)
    [9.0, 0.0, 5.0],  # 4. Injak tanah (Z turun kembali ke 5cm)
    [7.0, 0.0, 5.0],  # 5. Dorong ke belakang (X ditarik ke 7cm, tetap menapak)
]

print("\nMemulai simulasi melangkah (5 siklus)...")

try:
    for siklus in range(5): 
        print(f"--- Siklus Langkah {siklus + 1} ---")
        for i, waypoint in enumerate(step_sequence):
            target_x, target_y, target_z = waypoint
            
            val_coxa, val_femur, val_tibia = calculate_ik(target_x, target_y, target_z)
            
            move_servo(ID_COXA, val_coxa)
            move_servo(ID_FEMUR, val_femur)
            move_servo(ID_TIBIA, val_tibia)
            
            time.sleep(0.3) 
            
except KeyboardInterrupt:
    print("\nDihentikan oleh pengguna.")

# Kembalikan ke posisi standby yang aman
print("Kembali ke posisi standby...")
v_c, v_f, v_t = calculate_ik(7.0, 0.0, 5.0) 
move_servo(ID_COXA, v_c)
move_servo(ID_FEMUR, v_f)
move_servo(ID_TIBIA, v_t)
time.sleep(1)

# Disable Torque
for servo_id in [ID_COXA, ID_FEMUR, ID_TIBIA]:
    packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
portHandler.closePort()
print("Selesai.")