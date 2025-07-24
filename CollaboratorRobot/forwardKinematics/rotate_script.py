from pyfirmata import Arduino, util
import time

board = Arduino('COM10') # Replace with your COM port
it = util.Iterator(board)
it.start()

# Define the servo pin. 'd' for digital, '9' for pin 9, 's' for servo
# Use a PWM-capable pin on your Arduino for best results (e.g., 3, 5, 6, 9, 10, 11 on Uno)
servo = board.get_pin("d:6:s")

print("Controlling servo. Enter angle (0-180) or 'q' to quit.")

def swing(current, next):
    if current > next:
        for j in range(current, next, -1):
            servo.write(j)
    else:
        for j in range(current, next):
            servo.write(j)
    current = next

# currentAngle = [0, 0, 0]
# swing([0, 0, 0,], currentAngle)

servo.write(90)
current = 90
try:
    while True:
        angle1 = int(input("enter angle : "))
        try:
            
            if 90 <= angle1 <= 180:
                swing(current, angle1)
                print("current = ", current)
            else:
                print("Angle must be between 0 and 180.")
            
        except ValueError:
            print("Invalid input. Please enter a number or 'q'.")
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    board.exit()
    print("Serial connection closed.")