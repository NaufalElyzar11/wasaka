from pyfirmata import Arduino, util
import time

board = Arduino('COM10') # Replace with your COM port
it = util.Iterator(board)
it.start()

# Define the servo pin. 'd' for digital, '9' for pin 9, 's' for servo
# Use a PWM-capable pin on your Arduino for best results (e.g., 3, 5, 6, 9, 10, 11 on Uno)
servo = [board.get_pin('d:11:s'), board.get_pin('d:10:s'), board.get_pin('d:9:s')]

print("Controlling servo. Enter angle (0-180) or 'q' to quit.")

def swing(current, next):
    for i in range(3):
        for j in range(current[i], next[i]):
            servo[i].write(j)
        current[i] = next[i]

def swing(current, next, servo):
    if(current > next):
        for j in range(current, next, -1):
            servo.write(j)
    else:
        for j in range(current, next):
            servo.write(j)
    current = next

# currentAngle = [0, 0, 0]
# swing([0, 0, 0,], currentAngle)

current1 = 90
servo[0].write(current1)
current2 = 0
servo[1].write(current2)
current3 = 0
servo[2].write(current3)
try:
    while True:
        angle1 = int(input("enter angle 1 : "))
        angle2 = int(input("enter angle 2 : "))
        angle3 = int(input("enter angle 3 : "))
        try:
            
            if 0 <= angle1 <= 180:
                if current1 > angle1:
                    for j in range(current1, angle1, -1):
                        servo[0].write(j)
                else:
                    for j in range(current1, angle1):
                        servo[0].write(j)
                current1 = angle1
            if 0 <= angle2 <= 80:
                if current2 > angle2:
                    for j in range(current2, angle2, -1):
                        servo[1].write(j)
                else:
                    for j in range(current2, angle2):
                        servo[1].write(j)
                current2 = angle2
            if 0 <= angle3 <= 180:
                if current3 > angle3:
                    for j in range(current3, angle3, -1):
                        servo[2].write(j)
                else:
                    for j in range(current3, angle3):
                        servo[2].write(j)
                current3 = angle3
            else:
                print("Angle must be between 0 and 180.")
            print("current = ", current1, " ", current2, " ", current3)
        except ValueError:
            print("Invalid input. Please enter a number or 'q'.")
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    board.exit()
    print("Serial connection closed.")