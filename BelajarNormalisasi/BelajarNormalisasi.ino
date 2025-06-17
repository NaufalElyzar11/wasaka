#include <VarSpeedServo.h>

// Kaki 1
VarSpeedServo myservo1;
VarSpeedServo myservo2;
VarSpeedServo myservo3;

// Kaki 1 //
#define servoPin1 2 
#define servoPin2 3
#define servoPin3 4 

// Mendeklarasikan Panjang dari kaki
//=========Body VARIABLE
float cx = 40; //coxa a1
float fm = 61; //femur a2
float tb = 118;//tibia a3
float Zo = 61;//40


void setup() {
  Serial.begin(9600);
  myservo1.attach(servoPin1);  // joint 1  
  myservo2.attach(servoPin2);  // joint 2
  myservo3.attach(servoPin3);  // joint 3
 
}

void loop() {
//   kinematik(50, 20, 40);
//
//
//  delay(1000);

  myservo1.write(90);
  myservo2.write(0);
  myservo3.write(0); 
  delay(1000);

//  myservo1.write(105);
//  myservo2.write(10);
//  myservo3.write(0); 
//  delay(1000);
  
//  myservo1.write(135);
//  myservo2.write(30);
//  myservo3.write(0); 
//  delay(1000);
}

void kinematik(float x, float y, float zet) {
  float tetha1;
  float tetha2;
  float tetha3;
  
  float z = Zo + zet;
  float L1 = sqrt(sq(x) + sq(y));
  // === tetha1 === //
  if(x > 0 || y > 0) {
  tetha1 = atan(x / y) / PI * 180;}
  else {tetha1 = 0;}
//  float inverseSudutCoxa = 90 - tetha1;
  float L = sqrt(sq(L1 - cx) + sq(z));
  // === tetha3 === //
  tetha3 = acos( ( sq(tb) + sq(fm) - sq(L) ) / ( 2 * tb * fm ) ) / PI * 180;
  float a1 = acos( z / L ) / PI * 180;
  float a2 = acos( ( sq(fm) + sq(L) - sq(tb) ) / ( 2 * fm * L ) ) / PI * 180;
  // === tetha2 === //
  tetha2 = a1 + a2;

  Serial.print("Sudut1 : ");
  Serial.println(tetha1);
  Serial.print("Sudut2 : ");
  Serial.println(tetha2);
  Serial.print("Sudut3 : ");
  Serial.println(tetha3);

  myservo1.write(tetha1);
  myservo2.write(tetha2);
  myservo3.write(tetha3); 
}
