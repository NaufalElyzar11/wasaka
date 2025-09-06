import time
import math
import numpy as np
import threading
from threading import Lock
from tkinter import Tk, Scale, HORIZONTAL
from lib.servo import *
from lib.hexapod_constant import *

start_time = time.time()

# Initialize the servo
servo = DynamixelServo(device_name="COM21", baudrate=1000000)
servo_ids = list(range(1, 19))  # Servo IDs from 1 to 18
goal_positions = [512 for _ in range(18)]  # All servos to position 512
servo.enable_torque(servo_ids)

COXA_ZERO_DEG = 240.0      # 150° (tengah servo) + 90° (definisi sudut coxa)
COXA_TRIM_DEG = 0.0        # boleh kamu isi ± beberapa derajat jika perlu

# +1  jika z positif = naik
# -1  jika z negatif = naik (kasusmu sekarang)
LIFT_SIGN = -1.0

# Tambahkan konstanta ini di atas
LIFT_SIGN = -1.0  # sumbu Z kamu: negatif = naik
def ease01(s):     # cosine ease-in/out di [0..1]
    return 0.5 - 0.5*math.cos(math.pi*s)

RADIAL_SWING_BOOST = 0.0  # 0.0..0.010 (meter). Opsional: dorong kaki sedikit keluar saat swing


# Define trajectory function with vx and vy for wave gait
def trajectory(t, p, phase, step_height, step_duration, vx, vy, v_rot, leg_index):
    # Phase & progress
    cycle_time = (t + phase) % step_duration
    progress = cycle_time / step_duration

    # Komponen rotasi badan → offset XY
    angle = math.pi/2 + math.atan2(leg_base_positions[leg_index][0],
                                   leg_base_positions[leg_index][1])
    offx = v_rot * math.sin(angle)
    offy = v_rot * math.cos(angle)

    if progress < p:
        # ===== SWING (angkat & maju) =====
        s = progress / p                 # 0..1
        se = ease01(s)                   # halus 0..1

        # Z: setengah-sinus (naik lalu turun ke 0 di akhir swing)
        z = LIFT_SIGN * step_height * math.sin(math.pi * s)

        # XY: S-curve (tidak linier) → kurangi kick di liftoff/touchdown
        x = (vx + offx) * (se - 0.5)
        y = (vy + offy) * (se - 0.5)

        # Opsional: dorong sedikit radial keluar di pertengahan swing
        if RADIAL_SWING_BOOST > 0.0:
            bell = s * (1.0 - s)         # puncak di tengah swing
            x += RADIAL_SWING_BOOST * math.sin(angle) * bell
            y += RADIAL_SWING_BOOST * math.cos(angle) * bell
    else:
        # ===== STANCE (turun & mundur) =====
        s = (progress - p) / (1.0 - p)   # 0..1
        se = ease01(s)

        z = 0.0
        # XY: balik arah dengan S-curve juga (halus di transisi)
        x = (vx + offx) * (0.5 - se)
        y = (vy + offy) * (0.5 - se)

    # Jika benar2 diam, jangan angkat
    if vx == 0.0 and vy == 0.0 and v_rot == 0.0:
        z = 0.0

    return x, y, z


# helper
def _clamp(x, lo, hi):
    return max(lo, min(hi, x))

# --- REPLACE old inverse_kinematics with this ---
def inverse_kinematics(x, y, z):
    # sudut coxa sesuai konvensimu
    coxa_angle = math.pi / 2 + math.atan2(x, y)

    # jarak pada bidang XY dikurangi lengan coxa
    r_xy = math.hypot(x, y) - coxa_length
    if r_xy < 1e-6:
        r_xy = 1e-6

    d = math.hypot(r_xy, z)

    # batasi d agar segitiga valid dan hindari acos(>1) karena pembulatan
    d = _clamp(d, 1e-9, femur_length + tibia_length - 1e-9)

    a1 = math.atan2(z, r_xy)

    cosA = (d*d + femur_length*femur_length - tibia_length*tibia_length) / (2.0*d*femur_length)
    cosA = _clamp(cosA, -1.0, 1.0)
    A = math.acos(cosA)
    femur_angle = math.pi/2 - (A + a1)

    cosB = (femur_length*femur_length + tibia_length*tibia_length - d*d) / (2.0*femur_length*tibia_length)
    cosB = _clamp(cosB, -1.0, 1.0)
    B = math.acos(cosB)
    tibia_angle = math.pi - B

    return coxa_angle, femur_angle, tibia_angle


# Define body kinematics with rotation
def body_kinematics(body_position, body_orientation):
    x_trans, y_trans, z_trans = body_position
    r, p, y = body_orientation
    roll = math.radians(r)
    pitch = math.radians(p)
    yaw = math.radians(y)

    # Compute the rotation matrix for the given roll, pitch, and yaw
    c_r, s_r = math.cos(roll), math.sin(roll)
    c_p, s_p = math.cos(pitch), math.sin(pitch)
    c_y, s_y = math.cos(yaw), math.sin(yaw)

    # Rotation matrix combining roll, pitch, and yaw
    rotation_matrix = np.array([
        [c_y * c_p, c_y * s_p * s_r - s_y * c_r, c_y * s_p * c_r + s_y * s_r],
        [s_y * c_p, s_y * s_p * s_r + c_y * c_r, s_y * s_p * c_r - c_y * s_r],
        [-s_p, c_p * s_r, c_p * c_r]
    ])

    leg_positions = []

    for leg_base in leg_base_positions:
        # Apply rotation to the leg base position
        rotated_leg_base = np.dot(rotation_matrix, np.array(leg_base))

        # Apply translation to the rotated position
        leg_x = rotated_leg_base[0] + x_trans
        leg_y = rotated_leg_base[1] + y_trans
        leg_z = rotated_leg_base[2] + z_trans

        leg_positions.append((leg_x, leg_y, leg_z))

    return leg_positions

def dxl_pos(radians):
    max_radians = math.radians(300.0)
    radians = _clamp(radians, 0.0, max_radians)
    return int(round((radians / max_radians) * 1023))

# Control each leg joint
def joint_control(joint_index, pos):
    # leg id 0..5 dari joint_index (1,4,7,10,13,16)
    leg = (joint_index - 1) // 3
    x, y, z = pos

    # Kaki 3 terakhir (belakang) tetap dibalik XY seperti kode lamamu
    if leg >= 3:
        x, y = -x, -y

    # IK
    coxa_angle, femur_angle, tibia_angle = inverse_kinematics(x, y, z)

    # Basis coxa + offset orientasi tiap kaki
    base = math.radians(COXA_ZERO_DEG + COXA_TRIM_DEG)
    orient = [-leg_angle, 0.0, +leg_angle, -leg_angle, 0.0, +leg_angle][leg]

    coxa_servo  = base + orient - coxa_angle
    femur_servo = math.radians(180 + 30) - femur_angle
    tibia_servo = -math.radians(15) + tibia_angle

    i = leg * 3
    goal_positions[i + 0] = dxl_pos(coxa_servo)
    goal_positions[i + 1] = dxl_pos(femur_servo)
    goal_positions[i + 2] = dxl_pos(tibia_servo)

# Create a Tkinter window
root = Tk()
root.title("Hexapod Control")

# Create sliders
slider_length = 400

vx_slider = Scale(root, from_=-0.05, to=0.05, resolution=0.01, orient=HORIZONTAL, label="vx", length=slider_length)
vx_slider.set(0.0)
vx_slider.pack()

vy_slider = Scale(root, from_=-0.05, to=0.05, resolution=0.01, orient=HORIZONTAL, label="vy", length=slider_length)
vy_slider.set(0.0)
vy_slider.pack()

v_rot_slider = Scale(root, from_=-0.05, to=0.05, resolution=0.01, orient=HORIZONTAL, label="v_rot", length=slider_length)
v_rot_slider.set(0.0)
v_rot_slider.pack()

step_height_slider = Scale(root, from_=0, to=0.2, resolution=0.01, orient=HORIZONTAL, label="step height", length=slider_length)
step_height_slider.set(0.05)
step_height_slider.pack()

step_duration_slider = Scale(root, from_=0.1, to=10, resolution=0.1, orient=HORIZONTAL, label="step duration", length=slider_length)
step_duration_slider.set(1)
step_duration_slider.pack()

cpg_slider = Scale(root, from_=0.01, to=20, resolution=0.1, orient=HORIZONTAL, label="cpg", length=slider_length)
cpg_slider.set(2)
cpg_slider.pack()

x_slider = Scale(root, from_=-0.03, to=0.03, resolution=0.001, orient=HORIZONTAL, label="pos x", length=slider_length)
x_slider.set(0)
x_slider.pack()

y_slider = Scale(root, from_=-0.03, to=0.03, resolution=0.001, orient=HORIZONTAL, label="pos y", length=slider_length)
y_slider.set(0)
y_slider.pack()

z_slider = Scale(root, from_=-0.03, to=0.03, resolution=0.001, orient=HORIZONTAL, label="pos z", length=slider_length)
z_slider.set(0)
z_slider.pack()

r_slider = Scale(root, from_=-30, to=30, resolution=0.01, orient=HORIZONTAL, label="roll", length=slider_length)
r_slider.set(0)
r_slider.pack()

p_slider = Scale(root, from_=-30, to=30, resolution=0.01, orient=HORIZONTAL, label="pitch", length=slider_length)
p_slider.set(0)
p_slider.pack()

yaw_slider = Scale(root, from_=-30, to=30, resolution=0.01, orient=HORIZONTAL, label="yaw", length=slider_length)
yaw_slider.set(0)
yaw_slider.pack()

# Function to update the robot's movement
running = True
params_lock = Lock()
params = dict(vx=0.0, vy=0.0, v_rot=0.0,
              step_height=0.05, step_duration=1.0, cpg=2.0,
              x=0.0, y=0.0, z=0.0,
              roll=0.0, pitch=0.0, yaw=0.0)

def refresh_params():
    with params_lock:
        params['vx']   = vx_slider.get()
        params['vy']   = vy_slider.get()
        params['v_rot']= v_rot_slider.get()
        params['step_height']   = step_height_slider.get()
        params['step_duration'] = step_duration_slider.get()
        params['cpg']  = cpg_slider.get()
        params['x']    = x_slider.get()
        params['y']    = y_slider.get()
        params['z']    = z_slider.get()
        params['roll'] = r_slider.get()
        params['pitch']= p_slider.get()
        params['yaw']  = yaw_slider.get()
    if running:
        root.after(20, refresh_params)   # ~50 Hz refresh UI→worker


def update_robot():
    last_update_time = 0
    while running:
        try:
            t_start = time.time()
            t = t_start - start_time

            # Ambil snapshot parameter tanpa menyentuh Tk
            with params_lock:
                vx = params['vx']; vy = params['vy']; v_rot = params['v_rot']
                step_height = params['step_height']; step_duration = params['step_duration']; cpg = params['cpg']
                body_position   = (params['x'], params['y'], params['z'])
                body_orientation= (params['roll'], params['pitch'], params['yaw'])

            # ---- sisa kode-mu tetap ----
            leg_pos_body = body_kinematics(body_position, body_orientation)

            for leg_index in range(6):
                i = 1 / cpg
                phase_shifts = [0, step_duration/cpg, 2*step_duration/cpg, 3*step_duration/cpg, 4*step_duration/cpg, 5*step_duration/cpg]
                phase = phase_shifts[leg_index]
                pos = trajectory(t, i, phase, step_height, step_duration, vx, vy, v_rot, leg_index)
                lb = leg_pos_body[leg_index]
                leg_pos = (lb[0] + pos[0], lb[1] + pos[1], lb[2] + pos[2])
                joint_control(joint_index=leg_index * 3 + 1, pos=leg_pos)

            servo.write(servo_ids, goal_positions)
            time.sleep(1/120)   # 120 Hz cukup halus dan aman
        except Exception as e:
            print(e)
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


# Start the robot control thread
robot_thread = threading.Thread(target=update_robot, daemon=True)
robot_thread.start()

# Start the Tkinter main loop
refresh_params()  # mulai pembacaan slider di main thread
robot_thread = threading.Thread(target=update_robot, daemon=True)
robot_thread.start()
root.mainloop()
running = False


servo.disable_torque(servo_ids)
servo.close()