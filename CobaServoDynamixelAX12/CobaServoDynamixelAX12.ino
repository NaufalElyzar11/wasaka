#include <AX12A.h>

#define DirectionPin  (10u)
#define BaudRate      (1000000ul)
#define ID            (1u)

// Initialize Dynamixel motor
AX12A ax12a;

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Initialize Dynamixel motor
//  ax12a.begin(1000000, 2); // Baud rate dan pin Control
  ax12a.begin(BaudRate, DirectionPin, &Serial);

  // Set torque on
//  ax12a.setTorqueStatus(1, 1);
}

void loop() {
  // Move servo to angle 90 degrees
  uint16_t goal_position = 512; // 90 degrees
  uint8_t moving_speed = 100; // 0-1023
  ax12a.moveSpeed(1, goal_position, moving_speed);

  // Wait for servo to reach goal position
  while (ax12a.readPosition(1) != goal_position) {
    delay(100);
  }

  // Print current position
  uint16_t current_position = ax12a.readPosition(1);
  Serial.print("Current position: ");
  Serial.println(current_position);

  // Wait for a moment
  delay(1000);
}
