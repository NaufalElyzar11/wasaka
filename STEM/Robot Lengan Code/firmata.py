import cv2
import mediapipe as mp
import math
import time
from pyfirmata import Arduino, util

# -----------------------------
# Pengaturan Arduino
# -----------------------------

time.sleep(3)
try:
    board = Arduino('COM3')
    it = util.Iterator(board)
    it.start()
except Exception as e:
    print(f"Gagal terhubung ke Arduino di COM3. Error: {e}. Program berjalan dalam mode simulasi.")
    board = None

last_angles = [45, 138, 180, 20, 0] 


if board:
    print("Terhubung ke Arduino di port COM3")
    servo_pins = [2, 3, 4, 5, 6] 

    servos = [board.get_pin(f'd:{pin}:s') for pin in servo_pins]
    print("Servo diinisialisasi pada pin:", servo_pins)
    time.sleep(2)
    
    print("Mengatur servo ke posisi awal...")
    for i, servo in enumerate(servos):
        angle = max(0, min(180, last_angles[i]))
        last_angles[i] = angle  
        servo.write(angle)
        time.sleep(0.1) 
    
    print(f"Posisi awal diatur ke: {last_angles}")
    time.sleep(3) 
else:
    servos = None

# =================================================================
# Konfigurasi Kecepatan Servo
# =================================================================
SERVO_SPEED = 3  # makin kecil makin smooth
# =================================================================

# -----------------------------
# Konfigurasi Sudut Servo
# -----------------------------
servo_mins = [0, 40, 180, 0, 40]  
servo_maxs = [90, 140, 0, 90, 0]  

# -----------------------------
# Filter Smoothing
# -----------------------------
smoothed = list(last_angles)
def smooth_angle(i, new_value, alpha=0.2, threshold=2):
    """
    i = index servo
    new_value = sudut target
    alpha = konstanta low-pass filter
    threshold = batas perubahan minimal agar servo bergerak
    """
    global smoothed
    smoothed[i] = alpha * new_value + (1 - alpha) * smoothed[i]
    
    if abs(smoothed[i] - last_angles[i]) < threshold:
        return last_angles[i]
    return int(smoothed[i])

# -----------------------------
# Pengaturan Mediapipe
# -----------------------------
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

calibrated = False
min_dists = [None]*5
max_dists = [None]*5

# Rentang X pixel untuk servo 1
servo1_minX = 100
servo1_maxX = 540

def calc_distance(lm1, lm2, w, h):
    x1, y1 = int(lm1.x * w), int(lm1.y * h)
    x2, y2 = int(lm2.x * w), int(lm2.y * h)
    return math.hypot(x2 - x1, y2 - y1), (x1, y1), (x2, y2)

def map_to_servo(value, min_val, max_val, servo_min, servo_max):
    if max_val <= min_val:
        max_val = min_val + 1
    t = (value - min_val) / (max_val - min_val)
    t = max(0.0, min(1.0, t))
    if servo_min > servo_max:  
        return int(servo_min - t * (servo_min - servo_max))
    else:
        return int(servo_min + t * (servo_max - servo_min))

with mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5) as hands:

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            break

        h, w, _ = image.shape
        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)
        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        
        cv2.putText(image, f"Kecepatan: {SERVO_SPEED} (+/-)", 
                    (w - 250, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                lm = hand_landmarks.landmark
                wrist = lm[0]

                if calibrated:
                    target_angles = [0]*5
                    # Servo 1: posisi X tangan
                    x_pos = int(wrist.x * w)
                    target_angles[0] = map_to_servo(x_pos, servo1_minX, servo1_maxX, servo_mins[0], servo_maxs[0])
                    
                    # Servo 2–5: jarak jari ke pergelangan
                    pairs = [(8,0), (12,0), (16,0), (20,0)]
                    for i, (tip, wr) in enumerate(pairs, start=1):
                        dist, (x1,y1), (x2,y2) = calc_distance(lm[tip], lm[wr], w, h)
                        target_angles[i] = map_to_servo(dist, min_dists[i], max_dists[i], servo_mins[i], servo_maxs[i])
                        cv2.line(image, (x1,y1), (x2,y2), (255,0,0), 2)

                    # -----------------------------
                    # Gerakan Servo + Smoothing
                    # -----------------------------
                    for i in range(5):
                        target_angle = smooth_angle(i, target_angles[i])

                        current_angle = last_angles[i]
                        if current_angle < target_angle:
                            last_angles[i] = min(current_angle + SERVO_SPEED, target_angle)
                        elif current_angle > target_angle:
                            last_angles[i] = max(current_angle - SERVO_SPEED, target_angle)

                        if servos:
                            final_angle = max(0, min(180, last_angles[i]))
                            last_angles[i] = final_angle 
                            servos[i].write(final_angle)
                        
                        cv2.putText(image, f"S{i+1}: {last_angles[i]} deg", 
                                    (10, 40 + i*30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                else:
                    cv2.putText(image, "Kalibrasi: Tekan 'o' (Open), 'c' (Close), 's' (Save)", 
                                (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        cv2.imshow('Kontrol Servo Tangan', image)
        key = cv2.waitKey(5) & 0xFF

        if key == 27:  # ESC keluar
            break
        elif key in (ord('+'), ord('=')):
            SERVO_SPEED = min(SERVO_SPEED + 1, 20)
        elif key == ord('-'):
            SERVO_SPEED = max(SERVO_SPEED - 1, 1)
            
        # Kalibrasi
        elif key == ord('o') and results.multi_hand_landmarks:
            lm = results.multi_hand_landmarks[0].landmark
            pairs = [(8,0), (12,0), (16,0), (20,0)]
            for i, (tip, wr) in enumerate(pairs, start=1):
                max_dists[i], _, _ = calc_distance(lm[tip], lm[wr], w, h)
            print("MAX terkalibrasi:", max_dists)
        elif key == ord('c') and results.multi_hand_landmarks:
            lm = results.multi_hand_landmarks[0].landmark
            pairs = [(8,0), (12,0), (16,0), (20,0)]
            for i, (tip, wr) in enumerate(pairs, start=1):
                min_dists[i], _, _ = calc_distance(lm[tip], lm[wr], w, h)
            print("MIN terkalibrasi:", min_dists)
        elif key == ord('s'):
            if all(d is not None for d in min_dists[1:]) and all(d is not None for d in max_dists[1:]):
                calibrated = True
                print("Kalibrasi tersimpan! Kontrol servo aktif.")

cap.release()
cv2.destroyAllWindows()
if board:
    print("Mematikan koneksi Arduino...")
    for i, servo in enumerate(servos):
        servo.write(last_angles[i])
        time.sleep(0.1)
    board.exit()
    print("Koneksi ditutup.")
