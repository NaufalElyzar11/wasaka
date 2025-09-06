# lib/hexapod_constant.py
import math

# ===== Dimensi link (meter) =====
coxa_length  = 0.031      # 31 mm
femur_length = 0.040      # 40 mm
tibia_length = 0.0545     # 54.5 mm

# ===== Tata letak bodi: segi enam sama sisi =====
# r = jarak pusat bodi ke poros coxa (pivot) tiap kaki
r = 0.105                   # 84 mm (dari gambarmu)

# Kaki diagonal berada ±60° dari sumbu +Y (depan)
leg_angle = math.radians(60.0)

# Proyeksi koordinat untuk kaki diagonal
# (sesuai catatanmu: x = r*sin60 ≈ 72.75 mm, y = r*cos60 = 42 mm)
x = r * math.sin(leg_angle)   # ≈ 0.07275 m
y = r * math.cos(leg_angle)   # = 0.04200 m

# Ketinggian pangkal kaki (coxa pivot) terhadap pusat bodi
# Negatif ke arah bawah (konvensi kode-mu)
z = -0.046                 # 46 mm (22+24 pada catatanmu)

# ===== Posisi pangkal (frame bodi) untuk 6 kaki =====
# Urutan & orientasi disesuaikan dengan kode asli:
# 0:[-x,+y], 1:[0,+r], 2:[+x,+y], 3:[+x,-y], 4:[0,-r], 5:[-x,-y]
leg_base_positions = [
    [-x, +y, z],   # Leg 1  (depan-kiri diagonal)
    [ 0.0, +r, z], # Leg 2  (depan-tengah)
    [ +x, +y, z],  # Leg 3  (depan-kanan diagonal)
    [ +x, -y, z],  # Leg 4  (belakang-kanan diagonal)
    [ 0.0, -r, z], # Leg 5  (belakang-tengah)
    [ -x, -y, z],  # Leg 6  (belakang-kiri diagonal)
]
