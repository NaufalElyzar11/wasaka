#!/usr/bin/env python3
# hexapod_control.py
# Kontrol hexapod dengan Tkinter slider + inverse kinematics + gait halus.

import time, math, threading
import numpy as np
from threading import Lock
from tkinter import Tk, Scale, HORIZONTAL
from lib.servo import DynamixelServo
from lib.hexapod_constant import (
    coxa_length, femur_length, tibia_length,
    leg_base_positions, leg_angle
)

# ================== PORT / BUS ==================
PORT = "COM21"
BAUD = 1_000_000
servo = DynamixelServo(device_name=PORT, baudrate=BAUD)
servo_ids = list(range(1, 19))
goal_positions = [512] * 18
servo.enable_torque(servo_ids)

# ================== KONSTANTA ==================
# ===== di bagian KONSTANTA =====
# ================== KONSTANTA ==================
COXA_ZERO_DEG  = 240.0
COXA_TRIM_DEG  = 0.0
LIFT_SIGN      = -1.0
RADIAL_SWING_BOOST = 0.0

# Sementara samakan gain semua dulu (hindari saturasi saat debug)
COXA_SWING_GAIN = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

# TRIM PER-LEGI (urut leg 0..5) → geser pusat coxa supaya jauh dari endstop
COXA_TRIM_DEG_PER_LEG = [
    +8.0,  # leg 0 (kanan depan / front-right)
    0.0,   # leg 1 (kanan tengah)
    -8.0,  # leg 2 (kanan belakang)
    -8.0,  # leg 3 (kiri belakang)
    0.0,   # leg 4 (kiri tengah)
    +8.0   # leg 5 (kiri depan / front-left)
]


start_time = time.time()

# ================== UTILS ==================
def ease01(s):
    """Cosine easing in [0..1]."""
    return 0.5 - 0.5*math.cos(math.pi*s)

def _clamp(x, lo, hi):
    return max(lo, min(hi, x))

def dxl_pos(radians):
    max_radians = math.radians(300.0)
    radians = _clamp(radians, 0.0, max_radians)
    return int(round((radians / max_radians) * 1023))

# ================== TRAJECTORY ==================
def smoothstep(u):
    u = _clamp(u, 0.0, 1.0)
    return u*u*(3 - 2*u)

def trajectory(t, p, phase, step_height, step_duration, vx, vy, v_rot, leg_index):
    """
    p: duty ratio swing (0..1), saran: 0.5 (pakai slider cpg=2.0 seperti punyamu).
    Di dalam 1 siklus:
      - stance:  durasi = (1-p) * step_duration, z = 0
      - swing:   durasi = p * step_duration, z = bell (parabola)
    XY selalu menutup loop: ±0.5 * stride per siklus.
    """

    # --- waktu siklus & progress 0..1 ---
    cycle_time = (t + phase) % step_duration
    u = cycle_time / step_duration

    # --- orientasi radial karena yaw tubuh (v_rot) ---
    angle = math.pi/2 + math.atan2(leg_base_positions[leg_index][0],
                                   leg_base_positions[leg_index][1])
    offx, offy = v_rot * math.sin(angle), v_rot * math.cos(angle)

    # --- stride per siklus (world-frame target), konsisten untuk semua kaki ---
    stride_x = (vx + offx) * step_duration
    stride_y = (vy + offy) * step_duration

    # batas duty
    p = _clamp(p, 0.05, 0.95)       # jaga supaya ada stance & swing

    if u < (1.0 - p):
        # ================= STANCE (geser dulu) =================
        s = u / (1.0 - p)          # 0..1
        se = smoothstep(s)

        # dari +½ stride -> -½ stride (glide halus)
        x = stride_x * ( 0.5 - se)
        y = stride_y * ( 0.5 - se)
        z = 0.0                    # DIJAMIN rata
    else:
        # ================= SWING (angkat -> geser -> turun) =================
        s = (u - (1.0 - p)) / p    # 0..1
        se = smoothstep(s)

        # XY: -½ stride -> +½ stride (loop tertutup)
        x = stride_x * (se - 0.5)
        y = stride_y * (se - 0.5)

        # Z: parabola (lonceng sinus), puncak di s=0.5
        z = LIFT_SIGN * step_height * math.sin(math.pi * s)

    # Diam total? jangan angkat
    if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(v_rot) < 1e-6:
        z = 0.0

    # Matikan dulu gain per kaki (agar semua identik dan stabil)
    # x *= COXA_SWING_GAIN[leg_index]
    # y *= COXA_SWING_GAIN[leg_index]

    return x, y, z

# ================== IK ==================
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
        [-s_p,    c_p*s_r,               c_p*c_r]
    ])

    legs = []
    for base in leg_base_positions:
        rotated = np.dot(rot, np.array(base))
        legs.append((rotated[0] + x_trans,
                     rotated[1] + y_trans,
                     rotated[2] + z_trans))
    return legs


# ================== JOINT CONTROL ==================
def _rad2deg(a): 
    return math.degrees(a)

def joint_control(joint_index, pos):
    leg = (joint_index - 1) // 3
    x, y, z = pos

    # Kaki belakang dibalik koordinat XY (sama seperti kodenya sebelumnya)
    if leg >= 3:
        x, y = -x, -y

    # IK di ruang dunia
    coxa_angle, femur_angle, tibia_angle = inverse_kinematics(x, y, z)

    # ===== gunakan TRIM PER-LEGI di sini =====
    base   = math.radians(COXA_ZERO_DEG + COXA_TRIM_DEG_PER_LEG[leg])
    orient = [-leg_angle, 0.0, +leg_angle, -leg_angle, 0.0, +leg_angle][leg]

    # Mapping ke sudut servo
    coxa_servo  = base + orient - coxa_angle
    femur_servo = math.radians(180 + 35) - femur_angle
    tibia_servo = -math.radians(30) + tibia_angle

    # ---- SAFE MARGIN: jaga jauh dari clamp 0..300° (pakai 15..285°) ----
    LO = math.radians(15.0)
    HI = math.radians(285.0)

    # Jika ada yang mendekati batas, kecilkan stride XY utk kaki ini saja
    tries = 0
    while tries < 2 and (not (LO < coxa_servo < HI) or 
                         not (LO < femur_servo < HI) or 
                         not (LO < tibia_servo < HI)):
        tries += 1
        x *= 0.9; y *= 0.9   # kecilkan 10%
        coxa_angle, femur_angle, tibia_angle = inverse_kinematics(x, y, z)
        coxa_servo  = base + orient - coxa_angle
        femur_servo = math.radians(180 + 35) - femur_angle
        tibia_servo = -math.radians(30) + tibia_angle

    # Debug singkat (optional)
    # print(f"Leg {leg} | coxa={_rad2deg(coxa_servo):.1f} femur={_rad2deg(femur_servo):.1f} tibia={_rad2deg(tibia_servo):.1f}")

    i = leg * 3
    goal_positions[i+0] = dxl_pos(coxa_servo)
    goal_positions[i+1] = dxl_pos(femur_servo)
    goal_positions[i+2] = dxl_pos(tibia_servo)

# ================== UI (SLIDERS) ==================
root = Tk()
root.title("Hexapod Control")
slider_length = 400

def make_slider(label, frm, to, res, init):
    s = Scale(root, from_=frm, to=to, resolution=res,
              orient=HORIZONTAL, label=label, length=slider_length)
    s.set(init); s.pack(); return s

vx_slider   = make_slider("vx", -0.05, 0.05, 0.01, 0.0)
vy_slider   = make_slider("vy", -0.05, 0.05, 0.01, 0.0)
vrot_slider = make_slider("v_rot", -0.05, 0.05, 0.01, 0.0)
sh_slider   = make_slider("step height", 0, 0.04, 0.01, 0.01)
sd_slider   = make_slider("step duration", 0.1, 10, 0.1, 1.0)
cpg_slider  = make_slider("cpg", 0.01, 20, 0.1, 2.0)
x_slider    = make_slider("pos x", -0.03, 0.03, 0.001, 0.0)
y_slider    = make_slider("pos y", -0.03, 0.03, 0.001, 0.0)
z_slider    = make_slider("pos z", -0.03, 0.03, 0.001, 0.0)
r_slider    = make_slider("roll", -30, 30, 0.01, 0.0)
p_slider    = make_slider("pitch", -30, 30, 0.01, 0.0)
yaw_slider  = make_slider("yaw", -30, 30, 0.01, 0.0)

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

            for leg_index in range(6):
                phase = (leg_index * step_d) / cpg
                pos = trajectory(t, 1/cpg, phase, step_h, step_d, vx, vy, v_rot, leg_index)
                lb = leg_pos_body[leg_index]
                leg_pos = (lb[0] + pos[0], lb[1] + pos[1], lb[2] + pos[2])
                joint_control(joint_index=leg_index*3 + 1, pos=leg_pos)

            servo.write(servo_ids, goal_positions)
            time.sleep(1/120)
        except Exception as e:
            print("Error:", e)
            break

def on_close():
    global running
    running = False
    try:
        servo.disable_torque(servo_ids)
        servo.close()
    finally:
        root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# ================== START ==================
threading.Thread(target=update_robot, daemon=True).start()
refresh_params()
root.mainloop()

running = False
servo.disable_torque(servo_ids)
servo.close()
