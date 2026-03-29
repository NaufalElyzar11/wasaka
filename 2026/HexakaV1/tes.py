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
print("Torque ON. Robot bergerak!")
time.sleep(1)

# --- UJI COBA POSISI BERDIRI STABIL ---
# Kita buat kaki merentang sedikit lebih jauh (8cm) ke luar agar stabil
TEST_X = 8.0 
TEST_Y = 0.0
TEST_Z = 7.0 

print(f"\nMenghitung IK untuk Posisi Berdiri -> X:{TEST_X}, Z:{TEST_Z}...")
val_coxa, val_femur, val_tibia = calculate_ik(TEST_X, TEST_Y, TEST_Z)

print(f"Mengirim nilai ke ID Kaki 1 -> C:{val_coxa}, F:{val_femur}, T:{val_tibia}")

# Eksekusi
move_servo(ID_COXA, val_coxa)
move_servo(ID_FEMUR, val_femur)
move_servo(ID_TIBIA, val_tibia)

print("Kaki seharusnya membentuk lutut yang menekuk ke bawah (seperti kucing berdiri).")
time.sleep(10) # Tahan agak lama untuk inspeksi fisik

# Disable Torque
for servo_id in [ID_COXA, ID_FEMUR, ID_TIBIA]:
    packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
portHandler.closePort()
print("Selesai.")