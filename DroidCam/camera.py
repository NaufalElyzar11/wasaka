import cv2

# Sumber video, 0 untuk kamera utama,  
cam = cv2.VideoCapture(1) # 1 .. n-1 kamera untuk kamera lainnya, disini droidcam berada di index 1

while True:
    ret, frame = cam.read()

    # Menampilkan frame yang diterima
    cv2.imshow('Camera', frame)

    # q untuk selesai
    if cv2.waitKey(1) == ord('q'):
        break

# Melepaskan semua objek yang digunakan
cam.release()
cv2.destroyAllWindows()