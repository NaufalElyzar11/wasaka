import time
import math
import numpy as np
import threading
from tkinter import Tk, Scale, HORIZONTAL
from lib.servo import *
from lib.hexapod_constant import *

start_time = time.time()

DXL_RANGE_DEG = 300.0
DXL_MAX_TICK  = 1023
CENTER_DEG    = 150.0
CENTER_TICK   = 512

COXA_SIGN  = [+1, +1, +1, +1, +1, +1]
FEMUR_SIGN = [+1, +1, +1, +1, +1, +1]
TIBIA_SIGN = [+1, +1, +1, +1, +1, +1]

# nol matematika per sendi (rad) sehingga -> 512 tick
# awalnya isi pakai perkiraan, nanti benarkan lewat kalibrasi (bagian E)
coxa_zero  = base_yaw[:]               # coxa keluar = 512
femur_zero = [math.radians(90)]*6      # tebakan awal
tibia_zero = [math.radians(0)]*6       # tibia lurus = 0 rad (sesuai IK ini)

# Initialize the servo
servo = DynamixelServo(device_name="COM21", baudrate=1000000)
servo_ids = list(range(1, 19))  # Servo IDs from 1 to 18
goal_positions = [512 for _ in range(18)]  # All servos to position 512
servo.enable_torque(servo_ids)

def deg_to_tick(deg: float) -> int:
    deg = max(0.0, min(DXL_RANGE_DEG, deg))
    return int(round(deg / DXL_RANGE_DEG * DXL_MAX_TICK))

def tick_centered(angle_rad: float, zero_rad: float, sign: int) -> int:
    """
    angle_rad: sudut sendi (hasil IK) pada definisi matematis kita.
    zero_rad : sudut sendi yang harus menghasilkan tick 512 pada servo itu.
    sign     : +1 jika sudut meningkat membuat tick bertambah, -1 jika kebalikannya.
    """
    # konversi ke derajat relatif terhadap nol matematika
    delta_deg = math.degrees(angle_rad - zero_rad)
    servo_deg = CENTER_DEG + sign * delta_deg
    return deg_to_tick(servo_deg)

# Define trajectory function with vx and vy for wave gait
def trajectory(t, p, phase, step_height, step_duration, vx, vy, v_rot, leg_index):
    p = max(1e-3, min(1-1e-3, p))
    cycle_time = (t + phase) % step_duration
    progress = cycle_time / step_duration

    # gunakan base_yaw untuk komponen rotasi; konsisten dengan hexagon
    angle = base_yaw[leg_index]
    offset_x = v_rot * (-math.sin(angle))
    offset_y = v_rot * ( math.cos(angle))

    if progress < p:
        z = step_height * math.sin(2 * math.pi * progress)
        x = (vx + offset_x) * (progress / p - 0.5)
        y = (vy + offset_y) * (progress / p - 0.5)
    else:
        z = 0.0
        x = (vx + offset_x) * (0.5 - (progress - p) / (1 - p))
        y = (vy + offset_y) * (0.5 - (progress - p) / (1 - p))

    if vx == 0 and vy == 0 and v_rot == 0:
        z = 0.0
    return x, y, z


# Define inverse kinematics function
def inverse_kinematics(x, y, z):
    # sudut coxa: arah dari pusat ke EE di bidang xy
    coxa_angle = math.atan2(y, x)

    r = math.hypot(x, y)
    d = math.hypot(r, z)

    # batas workspace secara aman
    max_d = femur_length + tibia_length - 1e-6
    min_d = abs(femur_length - tibia_length) + 1e-6 
    d = max(min(d, max_d), min_d)

    a1 = math.atan2(z, r)

    # clamp argumen acos supaya pasti di dalam [-1, 1]
    def clamp(x): 
        return max(-1.0, min(1.0, x))

    cosA = clamp((d**2 + femur_length**2 - tibia_length**2) / (2 * d * femur_length))
    A = math.acos(cosA)
    femur_angle = math.pi/2 - (A + a1)

    cosB = clamp((femur_length**2 + tibia_length**2 - d**2) / (2 * femur_length * tibia_length))
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
    """Map a value from 0 to 300 degrees (in radians) to 0-1023 for Dynamixel."""
    max_radians = math.radians(300)  # Convert 300 degrees to radians (~5.24)
    
    if radians < 0 or radians > max_radians:
        print(f"Radians must be between 0 and {max_radians:.2f} (300 degrees)")
        if radians < 0:
            radians = 0
        elif radians>300:
            radians = 300

    return int((radians / max_radians) * 1023)

# Control each leg joint
def joint_control(leg_index: int, pos):
    x, y, z = pos
    coxa_angle, femur_angle, tibia_angle = inverse_kinematics(x, y, z)

    # konversi ke tick
    i = leg_index
    goal_positions[3*i + 0] = tick_centered(coxa_angle,  coxa_zero[i],  COXA_SIGN[i])
    goal_positions[3*i + 1] = tick_centered(femur_angle, femur_zero[i], FEMUR_SIGN[i])
    goal_positions[3*i + 2] = tick_centered(tibia_angle, tibia_zero[i], TIBIA_SIGN[i])

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
def update_robot():
    last_update_time = 0
    while True:
        try:
            t_start = time.time()
            t = time.time() - start_time

            if t_start - last_update_time >= 0.01:
                last_update_time = t_start
                # Read slider values for vx and vy
                vx = vx_slider.get()
                vy = vy_slider.get()
                v_rot = v_rot_slider.get()
                step_height = step_height_slider.get()
                step_duration = step_duration_slider.get()
                cpg = cpg_slider.get()

                body_position = (
                    x_slider.get(),
                    y_slider.get(),
                    z_slider.get(),
                )

                body_orientation = (
                    r_slider.get(),
                    p_slider.get(),
                    yaw_slider.get(),
                )

            # Compute leg positions
            leg_pos_body = body_kinematics(body_position, body_orientation)

            for leg_index in range(6):
                duty = 1.0 / cpg             # sebelumnya variabel kamu 'i'
                phase_shifts = [k*step_duration/cpg for k in range(6)]
                phase = phase_shifts[leg_index]

                pos = trajectory(t, duty, phase, step_height, step_duration, vx, vy, v_rot, leg_index)
                leg_base = leg_pos_body[leg_index]
                leg_pos = (leg_base[0] + pos[0], leg_base[1] + pos[1], leg_base[2] + pos[2])

                joint_control(leg_index, leg_pos)   # BUKAN joint_control(i, ...)

            
            # Move servos
            # print(servo_ids)
            # print(goal_positions)
            servo.write(servo_ids, goal_positions)
            time.sleep(1 / 240)  # Adjust simulation speed if necessary

            t_end = time.time()
            t_total = t_end-t_start + 1/1e10
            # print(f"Robot: time: {t_total:.2f} seconds, fps: {1/t_total:.2f}")

        except Exception as e:
            print(e)
            servo.disable_torque(servo_ids)
            servo.close()
            break

# Start the robot control thread
robot_thread = threading.Thread(target=update_robot, daemon=True)
robot_thread.start()

# Start the Tkinter main loop
root.mainloop()

servo.disable_torque(servo_ids)
servo.close()