import cv2
import mediapipe as mp
import math
import time

import inspect
if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec

from pyfirmata import Arduino, util

# -----------------------------
# Pengaturan Arduino
# -----------------------------
try:
    # Ganti 'COM12' dengan port Arduino Anda
    board = Arduino('COM13')
    it = util.Iterator(board) # Iterator diperlukan agar board tidak hang
    it.start()
    print("Terhubung ke Arduino di port COM11")
except Exception as e:
    print(f"Gagal terhubung ke Arduino. Error: {e}. Program berjalan dalam mode simulasi.")
    board = None

# Inisialisasi pin dan servo hanya jika board terhubung
if board:
    servo_pins = [2, 3, 4, 5, 6]  # Pin untuk Servo1–Servo5
    servos = [board.get_pin(f'd:{pin}:s') for pin in servo_pins]
    time.sleep(2)  # Tunggu board siap
else:
    servos = None

# =================================================================
# Konfigurasi Kecepatan Servo
# =================================================================
# Kecepatan: seberapa besar sudut berubah per frame (derajat).
# Nilai lebih kecil = gerakan lebih lambat & mulus.
# Coba di antara 1 sampai 10.
SERVO_SPEED = 3
# =================================================================

# -----------------------------
# Konfigurasi Sudut Servo
# -----------------------------
servo_mins = [0, 40, 180, 0, 0]    # Batas minimal masing-masing servo
servo_maxs = [90, 180, 80, 90, 30]  # Batas maksimal masing-masing servo

# 'last_angles' sekarang melacak posisi servo saat ini, bukan hanya target terakhir
last_angles = [45, 120, 130, 45, 0] # Mulai dari posisi tengah

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
servo1_maxX = 540 # Sesuaikan dengan lebar frame webcam Anda

def calc_distance(lm1, lm2, w, h):
    x1, y1 = int(lm1.x * w), int(lm1.y * h)
    x2, y2 = int(lm2.x * w), int(lm2.y * h)
    return math.hypot(x2 - x1, y2 - y1), (x1, y1), (x2, y2)

def map_to_servo(value, min_val, max_val, servo_min, servo_max):
    if max_val <= min_val:
        max_val = min_val + 1
    t = (value - min_val) / (max_val - min_val)
    t = max(0.0, min(1.0, t))
    # Balikkan rentang jika min lebih besar dari max
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
        image = cv2.flip(image, 1) # Flip agar gerakan intuitif
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)
        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        
        # Tampilkan info kecepatan
        cv2.putText(image, f"Kecepatan: {SERVO_SPEED} (+/-)", (w - 250, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                lm = hand_landmarks.landmark
                wrist = lm[0]

                if calibrated:
                    target_angles = [0]*5
                    # -----------------------------
                    # Hitung sudut TARGET untuk semua servo
                    # -----------------------------
                    # Servo 1: Posisi X
                    x_pos = int(wrist.x * w)
                    target_angles[0] = map_to_servo(x_pos, servo1_minX, servo1_maxX, servo_mins[0], servo_maxs[0])
                    
                    # Servo 2–5: Jarak jari ke pergelangan tangan
                    pairs = [(8,0), (12,0), (16,0), (20,0)] # telunjuk, tengah, manis, kelingking
                    for i, (tip, wr) in enumerate(pairs, start=1):
                        dist, (x1,y1), (x2,y2) = calc_distance(lm[tip], lm[wr], w, h)
                        target_angles[i] = map_to_servo(dist, min_dists[i], max_dists[i], servo_mins[i], servo_maxs[i])
                        cv2.line(image, (x1,y1), (x2,y2), (255,0,0), 2)

                    # -----------------------------
                    # Logika Gerakan Servo Bertahap
                    # -----------------------------
                    for i in range(5):
                        current_angle = last_angles[i]
                        target_angle = target_angles[i]

                        # Gerakkan sudut saat ini menuju target
                        if current_angle < target_angle:
                            last_angles[i] = min(current_angle + SERVO_SPEED, target_angle)
                        elif current_angle > target_angle:
                            last_angles[i] = max(current_angle - SERVO_SPEED, target_angle)
                        
                        # Kirim sudut yang sudah disesuaikan kecepatannya ke Arduino
                        if servos:
                            servos[i].write(last_angles[i])
                        
                        # Tampilkan info di layar (S1 untuk servo pertama, dst.)
                        cv2.putText(image, f"S{i+1}: {last_angles[i]} deg", (10, 40 + i*30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                else:
                    cv2.putText(image, "Kalibrasi: Tekan 'o' (Tangan Terbuka), 'c' (Terkepal), lalu 's' (Simpan)",
                                (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        cv2.imshow('Kontrol Servo Tangan', image)
        key = cv2.waitKey(5) & 0xFF

        if key == 27:  # ESC untuk keluar
            break
        # Tombol untuk mengatur kecepatan
        elif key == ord('+') or key == ord('='):
            SERVO_SPEED = min(SERVO_SPEED + 1, 20) # Batas atas kecepatan 20
        elif key == ord('-'):
            SERVO_SPEED = max(SERVO_SPEED - 1, 1) # Batas bawah kecepatan 1
            
        # Logika kalibrasi (tidak berubah)
        elif key == ord('o') and results.multi_hand_landmarks:
            lm = results.multi_hand_landmarks[0].landmark
            pairs = [(8,0), (12,0), (16,0), (20,0)]
            # i+1 agar sesuai dengan index min_dists/max_dists (1-4)
            for i, (tip, wr) in enumerate(pairs, start=1):
                max_dists[i], _, _ = calc_distance(lm[tip], lm[wr], w, h)
            print("Nilai MAX terkalibrasi:", max_dists)
        elif key == ord('c') and results.multi_hand_landmarks:
            lm = results.multi_hand_landmarks[0].landmark
            pairs = [(8,0), (12,0), (16,0), (20,0)]
            for i, (tip, wr) in enumerate(pairs, start=1):
                min_dists[i], _, _ = calc_distance(lm[tip], lm[wr], w, h)
            print("Nilai MIN terkalibrasi:", min_dists)
        elif key == ord('s'):
            if all(d is not None for d in min_dists[1:]) and all(d is not None for d in max_dists[1:]):
                calibrated = True
                print("Kalibrasi tersimpan! Kontrol servo aktif.")

cap.release()
cv2.destroyAllWindows()
if board:
    board.exit()