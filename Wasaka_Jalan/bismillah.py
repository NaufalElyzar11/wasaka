#!/usr/bin/env python3
import time, math, threading
import numpy as np
import msvcrt
from lib.servo import DynamixelServo
from lib.hexapod_constant import femur_length, tibia_length, leg_base_positions

# ================== KONFIG ==================
PORT = "COM21"
BAUD = 1_000_000
SERVO_IDS = list(range(1, 19))  # 1..18 urutan: per-kaki [coxa,femur,tibia]
HZ = 200.0

# Pose berdiri yang kamu minta (coxa=512, femur=469, tibia=172)
STAND_BASE_TICKS = (512, 469, 172)

# Gandakan untuk 6 kaki -> 18 servo
STAND_TICKS_18 = []
for _ in range(6):
    STAND_TICKS_18.extend(STAND_BASE_TICKS)

# Arah rotasi servo (+1 atau -1) per-servo (18 angka).
# Default +1 semua. Jika ada joint bergerak kebalik, cukup ubah jadi -1 pada indeks itu.
DIRS = [1]*18

# Body pose (bisa diubah dari keyboard)
body_x = 0.0
body_y = 0.0
body_z = 0.0
body_roll = 0.0
body_pitch = 0.0
body_yaw = 0.0

# Kecepatan perintah (WASD/QE mengubah ini)
vx = 0.0
vy = 0.0
vrot = 0.0

# Gait params
STEP_DURATION = 0.80
STEP_HEIGHT   = 0.020
DUTY          = 0.5
VX_MAX, VY_MAX, VROT_MAX = 0.045, 0.045, 0.05
DV, DROT, DZ, DHEIGHT, DDUR = 0.005, 0.005, 0.002, 0.005, 0.10

# ====== INTERNAL ======
TICKS_PER_RAD = 1023.0 / math.radians(300.0)
goal_positions = [512]*18
A_intercepts = [0.0]*18   # tick = A[i] + DIRS[i]*TICKS_PER_RAD * angle_rad

running = True
start_time = time.time()

GROUP_A = {0,2,4}
GROUP_B = {1,3,5}

# ========== UTIL ==========
def clamp(v, lo, hi): return max(lo, min(hi, v))

def body_kinematics(body_position, body_orientation_deg):
    x_trans, y_trans, z_trans = body_position
    r_deg, p_deg, y_deg = body_orientation_deg
    roll, pitch, yaw = map(math.radians, (r_deg, p_deg, y_deg))
    c_r, s_r = math.cos(roll), math.sin(roll)
    c_p, s_p = math.cos(pitch), math.sin(pitch)
    c_y, s_y = math.cos(yaw), math.sin(yaw)
    R = np.array([
        [c_y*c_p, c_y*s_p*s_r - s_y*c_r, c_y*s_p*c_r + s_y*s_r],
        [s_y*c_p, s_y*s_p*s_r + c_y*c_r, s_y*s_p*c_r - c_y*s_r],
        [-s_p,    c_p*s_r,               c_p*c_r              ]
    ])
    legs = []
    for base in leg_base_positions:
        rb = R @ np.array(base)
        legs.append((rb[0] + x_trans, rb[1] + y_trans, rb[2] + z_trans))
    return legs

def inverse_kinematics(x, y, z):
    coxa_angle = math.pi/2 + math.atan2(x, y)
    r = math.sqrt(x*x + y*y)
    d = math.sqrt(r*r + z*z)
    L1, L2 = femur_length, tibia_length
    d = min(d, L1 + L2)
    a1 = math.atan2(z, r)
    A = math.acos((d*d + L1*L1 - L2*L2) / (2.0*d*L1))
    femur_angle = math.pi/2 - (A + a1)
    B = math.acos((L1*L1 + L2*L2 - d*d) / (2.0*L1*L2))
    tibia_angle = math.pi - B
    return coxa_angle, femur_angle, tibia_angle

def foot_trajectory(t, leg_index, step_height, step_duration, vx, vy, v_rot):
    phase = 0.0 if leg_index in GROUP_A else 0.5
    progress = ((t / step_duration) + phase) % 1.0

    base = leg_base_positions[leg_index]
    angle = math.pi/2 + math.atan2(base[0], base[1])
    offx = v_rot * math.sin(angle)
    offy = v_rot * math.cos(angle)

    if progress < DUTY:
        u = progress / DUTY
        z = step_height * math.sin(math.pi * u)
        x = (vx + offx) * (u - 0.5)
        y = (vy + offy) * (u - 0.5)
    else:
        u = (progress - DUTY) / (1.0 - DUTY)
        z = 0.0
        x = (vx + offx) * (0.5 - u)
        y = (vy + offy) * (0.5 - u)

    if abs(vx)<1e-9 and abs(vy)<1e-9 and abs(v_rot)<1e-9:
        x = y = z = 0.0
    return x, y, z

def idx3(leg, j):  # j: 0=coxa,1=femur,2=tibia
    return leg*3 + j

# ====== KALIBRASI: anchor IK->tick ke pose berdiri ======
def compute_neutral_angles():
    """Sudut IK saat robot diam & body pose netral, urutan 18 (coxa,femur,tibia per-kaki)."""
    ang = [0.0]*18
    legs = body_kinematics((0,0,0), (0,0,0))
    for leg in range(6):
        x,y,z = legs[leg]
        a,b,c = inverse_kinematics(x,y,z)
        ang[idx3(leg,0)] = a
        ang[idx3(leg,1)] = b
        ang[idx3(leg,2)] = c
    return ang

def calibrate_linear_map():
    """Hitung A_intercepts supaya pada sudut netral, tick = STAND_TICKS_18 (coxa=512, dst)."""
    global A_intercepts
    neutral = compute_neutral_angles()
    A_intercepts = []
    for i in range(18):
        A = STAND_TICKS_18[i] - DIRS[i]*TICKS_PER_RAD*neutral[i]
        A_intercepts.append(A)

def angles_to_ticks(leg, coxa_ang, femur_ang, tibia_ang):
    i0 = idx3(leg,0); i1 = idx3(leg,1); i2 = idx3(leg,2)
    t0 = A_intercepts[i0] + DIRS[i0]*TICKS_PER_RAD*coxa_ang
    t1 = A_intercepts[i1] + DIRS[i1]*TICKS_PER_RAD*femur_ang
    t2 = A_intercepts[i2] + DIRS[i2]*TICKS_PER_RAD*tibia_ang
    return (
        int(clamp(t0, 0, 1023)),
        int(clamp(t1, 0, 1023)),
        int(clamp(t2, 0, 1023)),
    )

# ====== SERVO HELPERS ======
def go_home(servo):
    servo.write(SERVO_IDS, [512]*18)

def go_stand(servo):
    servo.write(SERVO_IDS, STAND_TICKS_18)

# ====== KEYBOARD ======
def print_controls():
    print("=== HEXAPOD CMD CONTROL ===")
    print("W/S: maju/mundur   A/D: strafe   Q/E: rotasi   Space: stop")
    print("Z/X: turun/naik badan   H/J: step height +/-   K/L: step dur +/-")
    print("R: HOME   T: STAND   ESC: keluar")
    print_status()

def print_status():
    print(f"vx={vx:+.3f} vy={vy:+.3f} vrot={vrot:+.3f}  z={body_z:+.3f} "
          f"H={STEP_HEIGHT:.3f}  T={STEP_DURATION:.2f}", end="\r")

def keyboard_loop(servo):
    global vx, vy, vrot, body_z, STEP_HEIGHT, STEP_DURATION, running
    print_controls()
    while running:
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if not ch: continue
            c = ch.decode('utf-8', errors='ignore').lower()

            if c == 'w': vx = clamp(vx + DV, -VX_MAX, VX_MAX)
            elif c == 's': vx = clamp(vx - DV, -VX_MAX, VX_MAX)
            elif c == 'a': vy = clamp(vy + DV, -VY_MAX, VY_MAX)
            elif c == 'd': vy = clamp(vy - DV, -VY_MAX, VY_MAX)
            elif c == 'q': vrot = clamp(vrot + DROT, -VROT_MAX, VROT_MAX)
            elif c == 'e': vrot = clamp(vrot - DROT, -VROT_MAX, VROT_MAX)
            elif c == ' ': vx = vy = vrot = 0.0
            elif c == 'z': body_z -= DZ
            elif c == 'x': body_z += DZ
            elif c == 'h': STEP_HEIGHT = max(0.0, STEP_HEIGHT + DHEIGHT)
            elif c == 'j': STEP_HEIGHT = max(0.0, STEP_HEIGHT - DHEIGHT)
            elif c == 'k': STEP_DURATION = max(0.20, STEP_DURATION + DDUR)
            elif c == 'l': STEP_DURATION = max(0.20, STEP_DURATION - DDUR)
            elif c == 'r': go_home(servo)
            elif c == 't': go_stand(servo)
            elif ch == b'\x1b':
                running = False
                break
            print_status()
        time.sleep(0.01)

# ====== MAIN CONTROL LOOP ======
def control_loop(servo):
    dt = 1.0 / HZ
    while running:
        t = time.time() - start_time
        leg_pos_body = body_kinematics((body_x, body_y, body_z), (body_roll, body_pitch, body_yaw))

        for leg in range(6):
            dx, dy, dz = foot_trajectory(t, leg, STEP_HEIGHT, STEP_DURATION, vx, vy, vrot)
            x, y, z = leg_pos_body[leg]
            coxa, femur, tibia = inverse_kinematics(x+dx, y+dy, z+dz)
            t0, t1, t2 = angles_to_ticks(leg, coxa, femur, tibia)
            goal_positions[idx3(leg,0)] = t0
            goal_positions[idx3(leg,1)] = t1
            goal_positions[idx3(leg,2)] = t2

        servo.write(SERVO_IDS, goal_positions)
        time.sleep(dt)

# ====== BOOT ======
servo = DynamixelServo(device_name=PORT, baudrate=BAUD)

if __name__ == "__main__":
    try:
        servo.enable_torque(SERVO_IDS)

        # 1) Kirim pose berdiri yang pasti (coxa=512, femur=469, tibia=172)
        go_stand(servo)
        time.sleep(0.5)

        # 2) Kalibrasi linear agar IK->tick “menyetir” tepat ke pose berdiri saat diam
        calibrate_linear_map()

        # 3) Mulai kontrol
        kb = threading.Thread(target=keyboard_loop, args=(servo,), daemon=True)
        kb.start()
        control_loop(servo)

    except Exception as e:
        print("\nError:", e)
    finally:
        running = False
        try:
            go_home(servo)
            time.sleep(0.3)
        except:
            pass
        servo.disable_torque(SERVO_IDS)
        servo.close()
        print("\nSelesai.")
