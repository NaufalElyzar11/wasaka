#!/usr/bin/env python3
# Kontrol hexapod via stik PS4 (Bluetooth) + IK + gait halus
# Left stick: vx (strafe), vy (maju/mundur); Right stick X: v_rot
# L1/R1: step height; D-pad Up/Down: body z; X: panic stop; Options/Share: torque ON/OFF; PS: exit

import time, math, threading, os, sys
import numpy as np

# ==== GAMEPAD ====
import pygame

from threading import Lock
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

# ================== KONSTANTA ==================
# Kinematika & map servo
COXA_ZERO_DEG  = 240.0     # pusat servo 150° + 90° = 240°
COXA_TRIM_DEG  = 0.0
LIFT_SIGN      = -1.0      # sumbu Z negatif = naik
RADIAL_SWING_BOOST = 0.0

# Per-kaki gain coxa (genap diperbesar, sesuai keluhanmu)
COXA_SWING_GAIN = [1.0, 1.2, 1.0, 1.2, 1.0, 1.2]

# Batasan
STEP_H_MIN, STEP_H_MAX = 0.0, 0.04
BODY_Z_MIN, BODY_Z_MAX = -0.03, 0.03

# Kecepatan awal sengaja pelan
V_MAX    = 0.02    # m (arah x/y)
VROT_MAX = 0.02    # m ekivalen untuk pola rotasi (ikuti desain traj kamu)
DEADZONE = 0.15
SMOOTH_ALPHA = 0.20  # low-pass filter pada input

start_time = time.time()

# ================== UTILS ==================
def ease01(s):
    return 0.5 - 0.5*math.cos(math.pi*s)

def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def dxl_pos(radians):
    max_radians = math.radians(300.0)
    radians = _clamp(radians, 0.0, max_radians)
    return int(round((radians / max_radians) * 1023))

# ================== TRAJECTORY ==================
def trajectory(t, p, phase, step_height, step_duration, vx, vy, v_rot, leg_index):
    cycle_time = (t + phase) % step_duration
    progress   = cycle_time / step_duration

    angle = math.pi/2 + math.atan2(
        leg_base_positions[leg_index][0],
        leg_base_positions[leg_index][1]
    )
    offx, offy = v_rot * math.sin(angle), v_rot * math.cos(angle)

    if progress < p:
        # SWING
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
        # STANCE
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

    # Debug singkat
    # print(f"Leg {leg} | IK: {math.degrees(coxa_angle):.1f}° -> Servo: {math.degrees(coxa_servo):.1f}°")

    i = leg * 3
    goal_positions[i+0] = dxl_pos(coxa_servo)
    goal_positions[i+1] = dxl_pos(femur_servo)
    goal_positions[i+2] = dxl_pos(tibia_servo)

# ================== PARAMETER (SHARED) ==================
params_lock = Lock()
params = dict(
    vx=0.0, vy=0.0, v_rot=0.0,
    step_height=0.01, step_duration=1.0, cpg=2.0,
    x=0.0, y=0.0, z=0.0,
    roll=0.0, pitch=0.0, yaw=0.0
)

running = True

# ================== GAMEPAD THREAD ==================
def deadzone(v, dz):
    return 0.0 if abs(v) < dz else v

def init_gamepad():
    # cegah pygame minta window
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("Tidak ada gamepad terdeteksi. Pair dulu di Bluetooth.")
        return None
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"[Gamepad] Nama: {js.get_name()}")
    print(f"  Axes: {js.get_numaxes()}  Buttons: {js.get_numbuttons()}  Hats: {js.get_numhats()}")
    return js

def gamepad_loop():
    js = init_gamepad()
    if js is None:
        return

    # state untuk smoothing
    vx_f = vy_f = vrot_f = 0.0

    global running
    while running:
        # perlu pump agar nilai update
        pygame.event.pump()

        # Mapping umum DS4 di Windows (bisa beda di KW)
        ax0 = js.get_axis(0) if js.get_numaxes() > 0 else 0.0  # LS X
        ax1 = js.get_axis(1) if js.get_numaxes() > 1 else 0.0  # LS Y
        ax2 = js.get_axis(2) if js.get_numaxes() > 2 else 0.0  # RS X

        # Apply deadzone
        lx = deadzone(ax0, DEADZONE)
        ly = deadzone(ax1, DEADZONE)
        rx = deadzone(ax2, DEADZONE)

        # Konvensi: up pada stick biasanya -1, kita balik agar up=maju(+vy)
        vx_raw   = -ly * V_MAX   # up  = maju, down = mundur
        vy_raw   =  lx * V_MAX   # right = strafe kanan, left = strafe kiri
        vrot_raw = rx * VROT_MAX # rotasi tetap sama (RS X)

        # Low-pass
        vx_f   += SMOOTH_ALPHA * (vx_raw - vx_f)
        vy_f   += SMOOTH_ALPHA * (vy_raw - vy_f)
        vrot_f += SMOOTH_ALPHA * (vrot_raw - vrot_f)

        # Buttons
        btn = lambda i: js.get_button(i) if i < js.get_numbuttons() else 0

        panic = btn(1)  # X di DS4 (kadang 1)
        if panic:
            vx_f = vy_f = vrot_f = 0.0

        # L1/R1 step height
        if btn(4):  # L1
            with params_lock:
                params['step_height'] = _clamp(params['step_height'] - 0.005, STEP_H_MIN, STEP_H_MAX)
        if btn(5):  # R1
            with params_lock:
                params['step_height'] = _clamp(params['step_height'] + 0.005, STEP_H_MIN, STEP_H_MAX)

        # D-pad (hat)
        if js.get_numhats() > 0:
            hatx, haty = js.get_hat(0)
            if haty == 1:
                with params_lock:
                    params['z'] = _clamp(params['z'] + 0.002, BODY_Z_MIN, BODY_Z_MAX)
            elif haty == -1:
                with params_lock:
                    params['z'] = _clamp(params['z'] - 0.002, BODY_Z_MIN, BODY_Z_MAX)

        # Torque ON/OFF
        if btn(9):   # Options
            try:
                servo.enable_torque(servo_ids)
                print("[Torque] ON")
                time.sleep(0.2)
            except Exception as e:
                print("Torque ON error:", e)
        if btn(8):   # Share
            try:
                servo.disable_torque(servo_ids)
                print("[Torque] OFF")
                time.sleep(0.2)
            except Exception as e:
                print("Torque OFF error:", e)

        # PS button untuk keluar
        if btn(12):
            print("PS ditekan. Keluar...")
            break

        # Commit velocity ke params
        with params_lock:
            params['vx'] = vx_f
            params['vy'] = vy_f
            params['v_rot'] = vrot_f

        time.sleep(1/60.0)

# ================== ROBOT LOOP ==================
def update_robot():
    # Soft-start: torque ON dulu
    servo.enable_torque(servo_ids)

    global running
    while running:
        try:
            t = time.time() - start_time
            with params_lock:
                vx, vy, v_rot = params['vx'], params['vy'], params['v_rot']
                step_h, step_d, cpg = params['step_height'], params['step_duration'], params['cpg']
                body_pos = (params['x'], params['y'], params['z'])
                body_ori = (params['roll'], params['pitch'], params['yaw'])

            # Kinematika bodi
            leg_pos_body = body_kinematics(body_pos, body_ori)

            # Hitung tiap kaki
            for leg_index in range(6):
                phase = (leg_index * step_d) / cpg
                pos = trajectory(t, 1.0/cpg, phase, step_h, step_d, vx, vy, v_rot, leg_index)
                lb = leg_pos_body[leg_index]
                leg_pos = (lb[0] + pos[0], lb[1] + pos[1], lb[2] + pos[2])
                joint_control(joint_index=leg_index*3 + 1, pos=leg_pos)

            servo.write(servo_ids, goal_positions)
            time.sleep(1/120)
        except Exception as e:
            print("Error loop robot:", e)
            break

def cleanup():
    try:
        servo.disable_torque(servo_ids)
    except Exception:
        pass
    try:
        servo.close()
    except Exception:
        pass

def main():
    global running
    try:
        th_gp = threading.Thread(target=gamepad_loop, daemon=True)
        th_rb = threading.Thread(target=update_robot, daemon=True)
        th_gp.start()
        th_rb.start()
        while th_gp.is_alive() and th_rb.is_alive():
            time.sleep(0.2)
    finally:
        running = False
        cleanup()

if __name__ == "__main__":
    main()
