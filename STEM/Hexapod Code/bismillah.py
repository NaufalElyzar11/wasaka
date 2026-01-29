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
COXA_ZERO_DEG  = 240.0     # pusat servo 150° + 90° = 240°
COXA_TRIM_DEG  = 0.0
LIFT_SIGN      = -1.0      # sumbu Z negatif = naik
RADIAL_SWING_BOOST = 0.0   # opsional dorong radial

# 1.0 = normal, >1.0 = ayunan lebih besar
COXA_SWING_GAIN = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # Sama untuk semua kaki dulu

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
def trajectory(t, p, phase, step_height, step_duration, vx, vy, v_rot, leg_index):
    cycle_time = (t + phase) % step_duration
    progress = cycle_time / step_duration

    angle = math.pi/2 + math.atan2(
        leg_base_positions[leg_index][0],
        leg_base_positions[leg_index][1]
    )
    offx, offy = v_rot * math.sin(angle), v_rot * math.cos(angle)

    # Pastikan p (swing phase ratio) tidak terlalu kecil
    p = max(0.3, min(0.7, p))  # Batasi antara 30%-70%

    if progress < p:
        # SWING PHASE (kaki diangkat)
        s = progress / p
        # Gunakan easing yang lebih smooth untuk swing
        se = math.sin(math.pi * s)  # Sinus untuk gerakan lebih natural
        z = LIFT_SIGN * step_height * se  # Tinggi maksimum di tengah swing
        
        # Gerakan horizontal selama swing - mulai dari belakang ke depan
        x = (vx + offx) * (s - 0.5)
        y = (vy + offy) * (s - 0.5)

        # Tambahkan sedikit dorongan radial untuk gerakan lebih natural
        if RADIAL_SWING_BOOST > 0.0:
            bell = math.sin(math.pi * s)  # Bell curve
            x += RADIAL_SWING_BOOST * math.sin(angle) * bell
            y += RADIAL_SWING_BOOST * math.cos(angle) * bell
    else:
        # STANCE PHASE (kaki menapak)
        s = (progress - p) / (1.0 - p)
        # Gunakan easing linear untuk stance agar gerakan konstan
        z = 0.0  # Kaki tetap di tanah
        
        # Gerakan horizontal selama stance - mendorong robot
        # Mulai dari depan ke belakang dengan kecepatan konstan
        x = (vx + offx) * (0.5 - s)
        y = (vy + offy) * (0.5 - s)

    # Jika benar-benar diam, pastikan kaki tidak terangkat
    if abs(vx) < 0.001 and abs(vy) < 0.001 and abs(v_rot) < 0.001:
        z = 0.0
        x, y = 0.0, 0.0  # Tetap di posisi netral

    # Terapkan gain khusus per kaki
    x *= COXA_SWING_GAIN[leg_index]
    y *= COXA_SWING_GAIN[leg_index]

    if leg_index == 0 and int(t * 10) % 50 == 0:  # Print setiap ~5 detik
        phase_type = "SWING" if progress < p else "STANCE"
        print(f"Leg {leg_index} | Progress: {progress:.2f} | Phase: {phase_type} | "
              f"Z: {z:.3f} | X: {x:.3f}, Y: {y:.3f}")
    
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
def joint_control(joint_index, pos):
    leg = (joint_index - 1) // 3
    x, y, z = pos
    if leg >= 3:  # kaki belakang dibalik
        x, y = -x, -y

    coxa_angle, femur_angle, tibia_angle = inverse_kinematics(x, y, z)

    base   = math.radians(COXA_ZERO_DEG + COXA_TRIM_DEG)
    orient = [-leg_angle, 0.0, +leg_angle, -leg_angle, 0.0, +leg_angle][leg]

    coxa_servo  = base + orient - coxa_angle
    femur_servo = math.radians(180 + 35) - femur_angle
    tibia_servo = -math.radians(30) + tibia_angle

    print(f"Leg {leg} | Robot angle: {math.degrees(coxa_angle):.1f}° "
        f"=> Servo {math.degrees(coxa_servo):.1f}°")


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

            # Pattern gait tripod - kaki 0, 2, 4 sefase, kaki 1, 3, 5 sefase
            tripod1_phase = 0.0  # Kaki 0, 2, 4
            tripod2_phase = step_d / 2  # Kaki 1, 3, 5 (180° out of phase)

            for leg_index in range(6):
                # Atur fase berdasarkan pola tripod
                if leg_index in [0, 2, 4]:  # Tripod 1
                    phase = tripod1_phase
                else:  # Tripod 2 - kaki 1, 3, 5
                    phase = tripod2_phase
                
                # Swing ratio - atur berapa lama kaki diangkat vs menapak
                swing_ratio = 0.4  # 40% swing, 60% stance (lebih natural)
                
                pos = trajectory(t, swing_ratio, phase, step_h, step_d, vx, vy, v_rot, leg_index)
                lb = leg_pos_body[leg_index]
                leg_pos = (lb[0] + pos[0], lb[1] + pos[1], lb[2] + pos[2])
                joint_control(joint_index=leg_index*3 + 1, pos=leg_pos)

            servo.write(servo_ids, goal_positions)
            time.sleep(1/120)  # ~120Hz
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
