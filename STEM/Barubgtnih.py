#!/usr/bin/env python3
# hexapod_simulation.py
# Modifikasi dari hexapod_control.py untuk simulasi visual 2D dengan Tkinter.

import time, math, threading
import numpy as np
from threading import Lock
from tkinter import Tk, Scale, HORIZONTAL, Canvas, Frame, LEFT, RIGHT, BOTH, TOP, YES

# [SIMULASI] Menghapus import servo asli
# from lib.servo import DynamixelServo
# from lib.hexapod_constant import (
#     coxa_length, femur_length, tibia_length,
#     leg_base_positions, leg_angle
# )

# ================== [SIMULASI] KONSTANTA ==================
# Konstanta ini sebelumnya diimpor dari lib.hexapod_constant
# Nilai-nilai ini adalah tebakan umum untuk hexapod hobi (dalam meter)
coxa_length = 0.026
femur_length = 0.070
tibia_length = 0.100

# (x, y, z) posisi dasar kaki relatif terhadap pusat sasis
_BASE_X = 0.060
_BASE_Y_FRONT = 0.030
_BASE_Y_MID = 0.045
leg_base_positions = [
    (_BASE_X, _BASE_Y_FRONT, 0),   # 0: Kanan Depan
    (0, _BASE_Y_MID, 0),           # 1: Kanan Tengah
    (-_BASE_X, _BASE_Y_FRONT, 0),  # 2: Kanan Belakang
    (-_BASE_X, -_BASE_Y_FRONT, 0), # 3: Kiri Belakang
    (0, -_BASE_Y_MID, 0),           # 4: Kiri Tengah
    (_BASE_X, -_BASE_Y_FRONT, 0)    # 5: Kiri Depan
]
leg_angle = math.radians(60) # Sudut default untuk sendi coxa

# ================== [SIMULASI] MOCK SERVO ==================
# Kelas palsu untuk menggantikan DynamixelServo
class MockServo:
    def __init__(self, device_name, baudrate):
        print(f"[Simulasi] MockServo terhubung ke port {device_name} @ {baudrate} baud")

    def enable_torque(self, servo_ids):
        print(f"[Simulasi] Torque diaktifkan untuk ID: {servo_ids}")

    def disable_torque(self, servo_ids):
        print(f"[Simulasi] Torque dinonaktifkan untuk ID: {servo_ids}")

    def write(self, servo_ids, goal_positions):
        # Di dunia nyata, ini akan mengirim perintah. Di sini, kita melewatinya.
        pass

    def close(self):
        print("[Simulasi] Port ditutup.")

# ================== PORT / BUS ==================
PORT = "COM21" # Port palsu
BAUD = 1_000_000
# [SIMULASI] Menggunakan MockServo
servo = MockServo(device_name=PORT, baudrate=BAUD) 
servo_ids = list(range(1, 19))
goal_positions = [512] * 18
# [SIMULASI] Baris ini aman dipanggil karena kita menggunakan MockServo
servo.enable_torque(servo_ids) 

# ================== KONSTANTA (Asli) ==================
COXA_ZERO_DEG  = 240.0
COXA_TRIM_DEG  = 0.0
LIFT_SIGN      = -1.0
RADIAL_SWING_BOOST = 0.0
COXA_SWING_GAIN = [1.0, 1.2, 1.0, 1.2, 1.0, 1.2]
start_time = time.time()

# ================== UTILS ==================
def ease01(s):
    return 0.5 - 0.5*math.cos(math.pi*s)

def _clamp(x, lo, hi):
    return max(lo, min(hi, x))

def dxl_pos(radians):
    max_radians = math.radians(300.0)
    radians = _clamp(radians, 0.0, max_radians)
    return int(round((radians / max_radians) * 1023))

# ================== TRAJECTORY ==================
# (Tidak ada perubahan, fungsi ini sudah benar)
def trajectory(t, p, phase, step_height, step_duration, vx, vy, v_rot, leg_index):
    cycle_time = (t + phase) % step_duration
    progress   = cycle_time / step_duration

    angle = math.pi/2 + math.atan2(
        leg_base_positions[leg_index][0],
        leg_base_positions[leg_index][1]
    )
    offx, offy = v_rot * math.sin(angle), v_rot * math.cos(angle)

    if progress < p:
        s  = progress / p
        se = ease01(s)
        z  = LIFT_SIGN * step_height * math.sin(math.pi * s)
        x  = (vx + offx) * (se - 0.5)
        y  = (vy + offy) * (se - 0.5)

        if RADIAL_SWING_BOOST > 0.0:
            bell = s * (1.0 - s)
            x += RADIAL_SWING_BOOST * math.sin(angle) * bell
            y += RADIAL_SWING_BOOST * math.cos(angle) * bell
    else:
        s  = (progress - p) / (1.0 - p)
        se = ease01(s)
        z  = 0.0
        x  = (vx + offx) * (0.5 - se)
        y  = (vy + offy) * (0.5 - se)

    if vx == 0.0 and vy == 0.0 and v_rot == 0.0:
        z = 0.0

    x *= COXA_SWING_GAIN[leg_index]
    y *= COXA_SWING_GAIN[leg_index]

    return x, y, z

# ================== IK ==================
# (Tidak ada perubahan, fungsi ini sudah benar)
def inverse_kinematics(x, y, z):
    coxa_angle = math.pi/2 + math.atan2(x, y)
    r_xy = max(1e-6, math.hypot(x, y) - coxa_length)
    d = math.hypot(r_xy, z)
    d = _clamp(d, 1e-9, femur_length + tibia_length - 1e-9)

    a1 = math.atan2(z, r_xy)

    cosA = (d*d + femur_length*femur_length - tibia_length*tibia_length) / (2*d*femur_length)
    A = math.acos(_clamp(cosA, -1.0, 1.0))
    femur_angle = math.pi/2 - (A + a1)

    cosB = (femur_length*femur_length + tibia_length*tibia_length - d*d) / (2*femur_length*tibia_length)
    B = math.acos(_clamp(cosB, -1.0, 1.0))
    tibia_angle = math.pi - B

    return coxa_angle, femur_angle, tibia_angle

# ================== BODY KINEMATICS ==================
# (Tidak ada perubahan, fungsi ini sudah benar)
def body_kinematics(body_position, body_orientation):
    x_trans, y_trans, z_trans = body_position
    r, p, y = body_orientation
    roll, pitch, yaw = map(math.radians, (r, p, y))

    c_r, s_r = math.cos(roll), math.sin(roll)
    c_p, s_p = math.cos(pitch), math.sin(pitch)
    c_y, s_y = math.cos(yaw), math.sin(yaw)

    rot = np.array([
        [c_y*c_p, c_y*s_p*s_r - s_y*c_r, c_y*s_p*c_r + s_y*s_r],
        [s_y*c_p, s_y*s_p*s_r + c_y*c_r, s_y*s_p*c_r - c_y*s_r],
        [-s_p,    c_p*s_r,                 c_p*c_r]
    ])

    legs = []
    for base in leg_base_positions:
        rotated = np.dot(rot, np.array(base))
        legs.append((rotated[0] + x_trans,
                     rotated[1] + y_trans,
                     rotated[2] + z_trans))
    return legs


# ================== JOINT CONTROL ==================
# [SIMULASI] Menghapus print() agar tidak spam konsol
def joint_control(joint_index, pos):
    leg = (joint_index - 1) // 3
    x, y, z = pos
    if leg >= 3: # kaki belakang dibalik
        x, y = -x, -y

    coxa_angle, femur_angle, tibia_angle = inverse_kinematics(x, y, z)

    base   = math.radians(COXA_ZERO_DEG + COXA_TRIM_DEG)
    orient = [-leg_angle, 0.0, +leg_angle, -leg_angle, 0.0, +leg_angle][leg]

    coxa_servo  = base + orient - coxa_angle
    femur_servo = math.radians(180 + 35) - femur_angle
    tibia_servo = -math.radians(30) + tibia_angle

    # print(f"Leg {leg} | Robot angle: {math.degrees(coxa_angle):.1f}° "
    #       f"=> Servo {math.degrees(coxa_servo):.1f}°") # [SIMULASI] Dihapus

    i = leg * 3
    goal_positions[i+0] = dxl_pos(coxa_servo)
    goal_positions[i+1] = dxl_pos(femur_servo)
    goal_positions[i+2] = dxl_pos(tibia_servo)


# ================== UI (SLIDERS) ==================
root = Tk()
root.title("Hexapod Control & Simulation")

# [SIMULASI] Mengatur layout utama
main_frame = Frame(root)
main_frame.pack(fill=BOTH, expand=YES)

# [SIMULASI] Frame untuk kanvas
canvas_frame = Frame(main_frame, width=500, height=500)
canvas_frame.pack(side=LEFT, fill=BOTH, expand=YES, padx=10, pady=10)

# [SIMULASI] Frame untuk slider
slider_frame = Frame(main_frame, width=400)
slider_frame.pack(side=RIGHT, fill=BOTH, padx=10, pady=10)

slider_length = 380 # [SIMULASI] Diperkecil agar pas

def make_slider(label, frm, to, res, init):
    # [SIMULASI] Mengarahkan slider ke slider_frame
    s = Scale(slider_frame, from_=frm, to=to, resolution=res,
              orient=HORIZONTAL, label=label, length=slider_length)
    s.set(init); s.pack(pady=2); return s # [SIMULASI] Ditambahkan pady

vx_slider   = make_slider("vx (maju/mundur)", -0.05, 0.05, 0.01, 0.0)
vy_slider   = make_slider("vy (kanan/kiri)", -0.05, 0.05, 0.01, 0.0)
vrot_slider = make_slider("v_rot (putar)", -0.05, 0.05, 0.01, 0.0)
sh_slider   = make_slider("step height (tinggi angkat)", 0, 0.04, 0.01, 0.01)
sd_slider   = make_slider("step duration (durasi langkah)", 0.1, 10, 0.1, 1.0)
cpg_slider  = make_slider("cpg (koordinasi)", 0.01, 20, 0.1, 2.0)
x_slider    = make_slider("pos x", -0.03, 0.03, 0.001, 0.0)
y_slider    = make_slider("pos y", -0.03, 0.03, 0.001, 0.0)
z_slider    = make_slider("pos z", -0.03, 0.03, 0.001, 0.0)
r_slider    = make_slider("roll (guling)", -30, 30, 0.01, 0.0)
p_slider    = make_slider("pitch (angguk)", -30, 30, 0.01, 0.0)
yaw_slider  = make_slider("yaw (geleng)", -30, 30, 0.01, 0.0)

# ================== [SIMULASI] SETUP KANVAS ==================
CANVAS_WIDTH = 500
CANVAS_HEIGHT = 500
# Skala: 1 meter di dunia nyata = 1500 pixel di kanvas
SIM_SCALE = 1500 
CENTER_X = CANVAS_WIDTH / 2
CENTER_Y = CANVAS_HEIGHT / 2

kanvas = Canvas(canvas_frame, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg='white')
kanvas.pack(side=TOP)

# Variabel untuk menyimpan data gambar (posisi 3D)
sim_draw_points = {
    "bases": [(0,0,0)] * 6, # Posisi dasar 6 kaki
    "tips": [(0,0,0)] * 6   # Posisi ujung 6 kaki
}

def to_canvas_coords(x, y):
    """Konversi koordinat robot (meter) ke koordinat kanvas (pixel)"""
    # y dibalik karena (0,0) kanvas ada di kiri atas
    new_x = CENTER_X + (x * SIM_SCALE)
    new_y = CENTER_Y - (y * SIM_SCALE) 
    return new_x, new_y

def perbarui_kanvas():
    """Fungsi yang menggambar ulang simulasi di kanvas"""
    global sim_draw_points
    kanvas.delete("all") # Hapus gambar sebelumnya

    # Ambil data gambar terbaru (gunakan lock untuk keamanan thread)
    with params_lock:
        bases = list(sim_draw_points["bases"])
        tips = list(sim_draw_points["tips"])

    # Gambar badan robot (poligon menghubungkan dasar kaki)
    body_points = []
    for b in bases:
        body_points.extend(to_canvas_coords(b[0], b[1]))
    kanvas.create_polygon(body_points, fill='lightblue', outline='black', width=2)
    
    # Gambar setiap kaki
    for i in range(6):
        b = bases[i] # (x,y,z) dasar
        t = tips[i]  # (x,y,z) ujung

        # Konversi ke pixel
        bx, by = to_canvas_coords(b[0], b[1])
        tx, ty = to_canvas_coords(t[0], t[1])

        # Gambar garis kaki
        kanvas.create_line(bx, by, tx, ty, fill='gray', width=4)

        # Tentukan warna ujung kaki
        # Jika Z (ketinggian) ujung kaki > Z dasar kaki, berarti terangkat
        is_lifted = (t[2] - b[2]) * LIFT_SIGN > 0.005 # 5mm
        color = 'red' if is_lifted else 'black'
        
        # Gambar lingkaran di ujung kaki
        r = 6 if is_lifted else 4 # Buat lebih besar jika terangkat
        kanvas.create_oval(tx-r, ty-r, tx+r, ty+r, fill=color, outline='')

    # Jadwalkan penggambaran ulang
    if running:
        root.after(20, perbarui_kanvas) # Sekitar 50 FPS

# ================== LOOP PARAMETER ==================
running = True
params_lock = Lock()
params = dict(vx=0.0, vy=0.0, v_rot=0.0,
              step_height=0.05, step_duration=1.0, cpg=2.0,
              x=0.0, y=0.0, z=0.0,
              roll=0.0, pitch=0.0, yaw=0.0)

def refresh_params():
    with params_lock:
        params.update(vx=vx_slider.get(), vy=vy_slider.get(), v_rot=vrot_slider.get(),
                      step_height=sh_slider.get(), step_duration=sd_slider.get(), cpg=cpg_slider.get(),
                      x=x_slider.get(), y=y_slider.get(), z=z_slider.get(),
                      roll=r_slider.get(), pitch=p_slider.get(), yaw=yaw_slider.get())
    if running: root.after(20, refresh_params)

# ================== LOOP ROBOT ==================
def update_robot():
    while running:
        try:
            t = time.time() - start_time
            with params_lock:
                vx, vy, v_rot = params['vx'], params['vy'], params['v_rot']
                step_h, step_d, cpg = params['step_height'], params['step_duration'], params['cpg']
                body_pos = (params['x'], params['y'], params['z'])
                body_ori = (params['roll'], params['pitch'], params['yaw'])

            leg_pos_body = body_kinematics(body_pos, body_ori)
            
            # [SIMULASI] Kita butuh list untuk menyimpan 6 posisi ujung kaki
            all_leg_tips = []

            for leg_index in range(6):
                phase = (leg_index * step_d) / cpg
                # pos = (x,y,z) dari trajektori
                pos = trajectory(t, 1/cpg, phase, step_h, step_d, vx, vy, v_rot, leg_index)
                
                lb = leg_pos_body[leg_index]
                # leg_pos = (x,y,z) final dari ujung kaki
                leg_pos = (lb[0] + pos[0], lb[1] + pos[1], lb[2] + pos[2])
                
                # [SIMULASI] Simpan posisi ujung kaki
                all_leg_tips.append(leg_pos) 
                
                # Tetap panggil joint_control untuk menghitung goal_positions
                # (meskipun kita tidak mengirimnya ke servo asli)
                joint_control(joint_index=leg_index*3 + 1, pos=leg_pos)

            # [SIMULASI] Perbarui data gambar untuk thread kanvas
            with params_lock:
                sim_draw_points["bases"] = leg_pos_body
                sim_draw_points["tips"] = all_leg_tips

            # [SIMULASI] Panggilan ini sekarang hanya ke MockServo
            servo.write(servo_ids, goal_positions) 
            time.sleep(1/120) # Target ~120 Hz update
        except Exception as e:
            print("Error:", e)
            break

def on_close():
    global running
    running = False
    time.sleep(0.1) # Beri waktu thread untuk berhenti
    try:
        servo.disable_torque(servo_ids)
        servo.close()
    finally:
        root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# ================== START ==================
threading.Thread(target=update_robot, daemon=True).start()
refresh_params()
perbarui_kanvas() # [SIMULASI] Mulai loop penggambaran
root.mainloop()

# Pastikan semua berhenti saat ditutup
running = False
servo.disable_torque(servo_ids)
servo.close()
