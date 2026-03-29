import math

# --- 1. SPESIFIKASI ROBOT ---
# Panjang Segmen Kaki (cm)
L1 = 3.4   # Coxa
L2 = 4.0   # Femur
L3 = 5.45  # Tibia

# Konstanta Dynamixel AX-12A
CENTER_VAL = 512
DEG_TO_DYN = 1023 / 300.0  # 1 derajat = ~3.41 unit dynamixel

def calculate_ik(x, y, z):
    """
    Menghitung Inverse Kinematics untuk 1 kaki.
    x: Jarak lurus ke depan dari engsel coxa (cm)
    y: Jarak menyamping dari engsel coxa (cm)
    z: Jarak ke bawah (tanah) dari engsel coxa (cm) - Nilai positif = ke bawah
    """
    # 1. Sudut Coxa (Gamma)
    gamma_rad = math.atan2(y, x)
    
    # 2. Jarak Efektif (L_eff) dan Diagonal (D)
    L_xy = math.sqrt(x**2 + y**2)
    L_eff = L_xy - L1
    D = math.sqrt(L_eff**2 + z**2)
    
    # Proteksi Matematis: Cegah target di luar jangkauan fisik
    if D > (L2 + L3):
        print(f"WARNING: Target (X:{x}, Y:{y}, Z:{z}) di luar jangkauan fisik!")
        D = (L2 + L3) - 0.01  # Batasi agar tidak error "math domain"

    # 3. Hitung Sudut Femur
    alpha1 = math.atan2(z, L_eff)
    alpha2 = math.acos((L2**2 + D**2 - L3**2) / (2 * L2 * D))
    theta_femur_rad = alpha1 + alpha2
    
    # 4. Hitung Sudut Tibia
    beta = math.acos((L2**2 + L3**2 - D**2) / (2 * L2 * L3))
    theta_tibia_rad = math.pi - beta # 180 derajat (pi) - sudut dalam

    # --- KONVERSI KE DYNAMIXEL ---
    # Convert Radian ke Derajat
    coxa_deg = math.degrees(gamma_rad)
    femur_deg = math.degrees(theta_femur_rad)
    tibia_deg = math.degrees(theta_tibia_rad)

    # Implementasi Polaritas Anda:
    # Coxa: Asumsi +value bergerak ke Y positif (menyesuaikan nanti)
    coxa_val = int(CENTER_VAL + (coxa_deg * DEG_TO_DYN))
    
    # Femur: Anda bilang +value = Turun. Karena kita menghitung sudut turun, maka ditambah (+)
    femur_val = int(CENTER_VAL + (femur_deg * DEG_TO_DYN))
    
    # Tibia: Anda bilang +value = Naik. Karena kita mau menekuk ke BAWAH (lutut), maka dikurangi (-)
    tibia_val = int(CENTER_VAL - (tibia_deg * DEG_TO_DYN))

    # Terapkan Batasan (Limits) Anda
    femur_val = max(191, min(854, femur_val))
    tibia_val = max(82, min(929, tibia_val))

    return coxa_val, femur_val, tibia_val

# --- UJI COBA (SIMULASI SCRIPT) ---
# Misalnya kita ingin kaki berdiri di posisi: X = 6 cm, Y = 0 cm, Z = 6 cm ke bawah
target_x = 6.0
target_y = 0.0
target_z = 6.0 

coxa, femur, tibia = calculate_ik(target_x, target_y, target_z)

print(f"Target Posisi Ujung Kaki -> X: {target_x}cm, Y: {target_y}cm, Z: {target_z}cm")
print(f"Nilai Dynamixel yang harus dikirim:")
print(f"- Coxa  (ID 1): {coxa}")
print(f"- Femur (ID 2): {femur}")
print(f"- Tibia (ID 3): {tibia}")