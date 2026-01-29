import cv2
from ultralytics import YOLO
import time
# --- FIX UNTUK PYTHON 3.11+ DAN PYFIRMATA ---
import inspect
if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec
# --------------------------------------------
import pyfirmata

# --- Konfirgurasi ---
# 1. Setup StandardFirmata pada Arduino
# 2. Buat environment conda baru dengan python versi 3.8 
# 3. Install opencv, pyfirmata dan ultralytics
# 4. Run this code


# -- PyFirmata & Pin --
PORT = "COM13" 
SERVO_PINS = {
    'pan': 2,  # Rotasi
    'b': 3,    # Link 1
    'c': 4,   # Link 2
    'd': 5,   # Link 3
}

# -- Derajat Servo --
INITIAL_ANGLES = {
    'pan': 90, 
    'b': 180, 
    'c': 180, 
    'd': 30
}

UP_ANGLES = {
    'b': 50, 
    'c': 80, 
    'd': 40
}

DOWN_ANGLES = {
    'b': 140, 
    'c': 180, 
    'd': 30
}

# Derajat maximum dari servo pan atau rotasi--
SERVO_PAN_MIN_ANGLE = 90
SERVO_PAN_MAX_ANGLE = 180

# -- Kontrol --
SMOOTHING_FACTOR = 0.1 # mengatur seberapa smooth gerakan servo
LOST_TARGET_THRESHOLD = 15 
lost_target_counter = 0

# -- YOLO & Keypoints --
LEFT_WRIST_INDEX = 9
RIGHT_WRIST_INDEX = 10
MIN_KEYPOINT_CONFIDENCE = 0.5 

# --- Inisialisasi ---
board = None
servos = {}
try:
    board = pyfirmata.Arduino(PORT)
    print(f"Successfully connected to Arduino on port {PORT}")
    it = pyfirmata.util.Iterator(board)
    it.start()
    
    for name, pin in SERVO_PINS.items():
        servos[name] = board.get_pin(f'd:{pin}:s')
        print(f"Servo '{name}' on pin {pin} initialized.")

    print("Waiting 2 seconds for board to settle...")
    time.sleep(2) 

    print("Moving all servos to initial positions...")
    for name, servo_obj in servos.items():
        angle = INITIAL_ANGLES[name]
        servo_obj.write(angle)
    print("All servos set to initial positions.")

except Exception as e:
    print(f"FATAL ERROR connecting to Arduino: {e}")
    print("Please check the port, ensure StandardFirmata is uploaded, and check servo power.")
    exit() # Exit jika tidak bisa tersambung pada Arduino

# Pakai model YOLO dan pakai kamera
model = YOLO('best.pt')
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Inisialisasi variabel derajat servo
current_angles = INITIAL_ANGLES.copy()
target_angles = INITIAL_ANGLES.copy()

print("Starting hand detection... Press 'q' to exit.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        vertical_midpoint = h // 2
        target_found_in_frame = False

        results = model.predict(frame, task='pose', verbose=False)

        if results and results[0].keypoints and results[0].keypoints.xy.shape[0] > 0:
            all_keypoints = results[0].keypoints.xy.cpu().numpy()
            confidences = results[0].keypoints.conf.cpu().numpy()
            best_confidence = -1
            best_keypoint_coords = None

            for person_keypoints, person_confidences in zip(all_keypoints, confidences):
                for wrist_index in [LEFT_WRIST_INDEX, RIGHT_WRIST_INDEX]:
                    if len(person_confidences) > wrist_index and person_confidences[wrist_index] > MIN_KEYPOINT_CONFIDENCE:
                        if person_confidences[wrist_index] > best_confidence:
                            best_confidence = person_confidences[wrist_index]
                            best_keypoint_coords = person_keypoints[wrist_index]

            if best_keypoint_coords is not None:
                target_found_in_frame = True
                lost_target_counter = 0 
                keypoint_x, keypoint_y = int(best_keypoint_coords[0]), int(best_keypoint_coords[1])

                # --- MEMETAKAN KEYPOINT POSITION KE DERAJAT SERVO ---
                
                # Pan Servo (Rotasi)
                # Memetakan posisi horizontal (keypoint_x) ke derajat servo
                pan_angle = SERVO_PAN_MAX_ANGLE - ((keypoint_x / w) * (SERVO_PAN_MAX_ANGLE - SERVO_PAN_MIN_ANGLE))
                target_angles['pan'] = pan_angle

                # Memetakan posisi vertikal ke derajat servo
                if keypoint_y < vertical_midpoint:
                    target_angles.update(UP_ANGLES)
                else:
                    target_angles.update(DOWN_ANGLES)

                # Visualisasi
                cv2.circle(frame, (keypoint_x, keypoint_y), 10, (0, 255, 255), -1)
                cv2.putText(frame, "Tracking Hand", (keypoint_x + 15, keypoint_y + 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        if not target_found_in_frame:
            lost_target_counter += 1
            if lost_target_counter > LOST_TARGET_THRESHOLD:
                # Jika target tidak ditemukan maka derajat servo akan kembali ke derajat inisial
                target_angles = INITIAL_ANGLES.copy()

        # --- KONTROL GERAKAN SERVO ---
        for name, servo_obj in servos.items():
            new_angle = current_angles[name] + (target_angles[name] - current_angles[name]) * SMOOTHING_FACTOR

            if name == 'pan':
                new_angle = max(SERVO_PAN_MIN_ANGLE, min(SERVO_PAN_MAX_ANGLE, new_angle))

            if abs(new_angle - current_angles[name]) >= 1:
                current_angles[name] = int(new_angle)
                servo_obj.write(current_angles[name])
        
        # Menampilkan Frame
        cv2.line(frame, (0, vertical_midpoint), (w, vertical_midpoint), (255, 255, 0), 1)
        cv2.putText(frame, "UP", (10, vertical_midpoint - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, "DOWN", (10, vertical_midpoint + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        cv2.imshow('YOLOv8 Multi-Servo Hand Tracking', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print("Cleaning up...")
    cap.release()
    cv2.destroyAllWindows()
    if board:
        print("Returning servos to initial positions...")
        for name, servo_obj in servos.items():
            servo_obj.write(INITIAL_ANGLES[name])
        time.sleep(1)
        board.exit()
    print("Done.")
