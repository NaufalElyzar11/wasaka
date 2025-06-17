#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

//Please see eManual Control Table section of your DYNAMIXEL.
//This example is written for DYNAMIXEL AX & MX series with Protocol 1.0.
//For MX 2.0 with Protocol 2.0, refer to write_x.ino example.
#define CW_ANGLE_LIMIT_ADDR         6
#define CCW_ANGLE_LIMIT_ADDR        8
#define ANGLE_LIMIT_ADDR_LEN        2
#define OPERATING_MODE_ADDR_LEN     2
#define TORQUE_ENABLE_ADDR          24
#define TORQUE_ENABLE_ADDR_LEN      1
#define LED_ADDR                    25
#define LED_ADDR_LEN                1
#define GOAL_POSITION_ADDR          30
#define GOAL_POSITION_ADDR_LEN      2
#define PRESENT_POSITION_ADDR       36
#define PRESENT_POSITION_ADDR_LEN   2
#define TIMEOUT 10    //default communication timeout 10ms
 

//  ===== Variable Servo ===== //

const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const uint8_t DXL_ID3 = 3;
const uint8_t DXL_ID4 = 4;
const uint8_t DXL_ID5 = 5;
const uint8_t DXL_ID6 = 6;
const uint8_t DXL_ID7 = 7;
const uint8_t DXL_ID8 = 8;
const uint8_t DXL_ID9 = 9;
const uint8_t DXL_ID10 = 10;
const uint8_t DXL_ID11 = 11;
const uint8_t DXL_ID12 = 12;
const uint8_t DXL_ID13 = 13;
const uint8_t DXL_ID14 = 14;
const uint8_t DXL_ID15 = 15;
const uint8_t DXL_ID16 = 16;
const uint8_t DXL_ID17 = 17;
const uint8_t DXL_ID18 = 18;

const float DXL_PROTOCOL_VERSION = 1.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

#include <Servo.h>
#include <EEPROMex.h>
#include "Arduino.h"
#include <math.h>
#include <avr/pgmspace.h>
#include <Adafruit_SSD1306.h>
//#include <NewTone.h>
Adafruit_SSD1306 Oled(128, 64, &Wire, 4);
//#include <TimerThree.h>
#include <TimerFreeTone.h>
#include <SimpleTimer.h>

#define ledOledPin A0
#define buzzerPin A1
//#define btnOledPin A2
#define bat2sSensPin A3
#define bat1sSensPin A4
#define bluetoothEnaPin A6
#define bluetoothStatusPin 11

//#define sApi1Pin A8
//#define sApi2Pin A7
//#define sApi3Pin A12
//#define sApi4Pin A5
//#define sApi5Pin A2
//
//#define kipasPin 28
//Depan
#define echo1Pin 23
#define trig1Pin 22

//Kanan Depan
#define trig2Pin 50
#define echo2Pin 51

//Kiri Depan
#define trig3Pin 24
#define echo3Pin 25

//Belakang
#define trig4Pin 28
#define echo4Pin 29

//Kanan Belakang
#define trig5Pin 40
#define echo5Pin 41

//Kiri Belakang
#define trig6Pin 34
#define echo6Pin 35

#define jumlahPing 4
#define jarakMak 250
#define MAX_DISTANCE 250

long durasi1, durasi2, durasi3, jarak1, jarak2, jarak3;     // membuat variabel durasi dan jarak

const int buttonPin = 47;

#include <NewPing.h>
NewPing pinG[jumlahPing] = {
//  NewPing(trig1Pin, echo1Pin, jarakMak),
//                            NewPing(trig2Pin, echo2Pin, jarakMak),
//                            NewPing(trig3Pin, echo3Pin, jarakMak),
//                            NewPing(trig4Pin, echo4Pin, jarakMak),
//                            NewPing(trig5Pin, echo5Pin, jarakMak),
//                            NewPing(trig6Pin, echo6Pin, jarakMak),
      NewPing(trig1Pin, echo1Pin, MAX_DISTANCE), //depan
      NewPing(trig2Pin, echo2Pin, MAX_DISTANCE), //kanan depan
      NewPing(trig3Pin, echo3Pin, MAX_DISTANCE), //kiri depan
      NewPing(trig4Pin, echo4Pin, MAX_DISTANCE), //belakang
//      NewPing(2, 1, MAX_DISTANCE), //SRF05
//      NewPing(26, 27, MAX_DISTANCE), //kiri belakang
//      NewPing(28, 29, MAX_DISTANCE), //belakang
//      NewPing(36, 37, MAX_DISTANCE)//capit
}; //(triger, echo, jarakMak);

//Sensor ultra sonic===============================
NewPing depan(trig1Pin, echo1Pin, 200);
NewPing kananDepan(trig2Pin, echo2Pin, 200);
NewPing kiriBelakang(trig5Pin, echo5Pin, 200);
NewPing kiriDepan(trig3Pin, echo3Pin, 200);
NewPing kananBelakang(trig6Pin, echo6Pin, 200);
NewPing belakang(trig4Pin, echo4Pin, 200);

unsigned int distanceDepan;
unsigned int distanceKananDepan;
unsigned int distanceKananBelakang;
unsigned int distanceKiriDepan;
unsigned int distanceKiriBelakang;
unsigned int distanceBelakang;

//=================================================

#define jumlahWarna 3

int warna[3];
int jarak[4];
int api[6];
byte jarakLilin = 0;
float voltageReading;
float voltageReading1;
float batVol, batVol1, batPct;

//Variabel Konversi
//    int goalPosition = 512;  // Nilai goal position yang ingin dikonversi
    int positionMin = 0;     // Nilai minimum posisi servo
    int positionMax = 1023;  // Nilai maksimum posisi servo
    int degreeMin = 0;       // Sudut minimum yang diinginkan dalam derajat
    int degreeMax = 300;     // Sudut maksimum yang diinginkan dalam derajat
    float positionRange = (float)(positionMax - positionMin);
    float degreeRange = (float)(degreeMax - degreeMin);

int degreeToGoalPosition(float degree, int positionMin, int positionMax, int degreeMin, int degreeMax) {
    float positionRange = (float)(positionMax - positionMin);
    float degreeRange = (float)(degreeMax - degreeMin);
    float goalPosition = ((float)(degree - degreeMin) / degreeRange) * positionRange + positionMin;
    return (int)goalPosition;
}

int US1Dekat = 13; //Depan
int US1Sedang = 18;
int US1Jauh = 20;

int US2Dekat = 14; //Serong Kanan
int US2Sedang = 20;
int US2Jauh = 30;

int US3Dekat = 14; //kiri
int US3Sedang = 16;
int US3Jauh = 25;

//int US4Dekat = 12; //Belakang
//int US4Sedang = 20;
//int US4Jauh = 35;

int US5Dekat = 13; //kanan
int US5Sedang = 16;
int US5Jauh = 25;

int US6Dekat = 14; //Serong Kiri
int US6Sedang = 16;
int US6Jauh = 25;

  int US1;
  int US3;
  int US5;
//Ultra


//Sensor Warna
//#define WS01 A9  VCC
//#define WS11 A11  GND
#define WS21 A4
#define WS31 28
#define WsensorOut 31
int hitam = 1;
int abu = 0;
int putih = 0;
int redColor = 0;
int greenColor = 0;
int blueColor = 0;

      
      //This namespace is required to use Control table item names
      using namespace ControlTableItem;
 //=====================================================================

 
Servo srv_1A; Servo srv_1B; Servo srv_1C;
Servo srv_2A; Servo srv_2B; Servo srv_2C;
Servo srv_3A; Servo srv_3B; Servo srv_3C;
Servo srv_4A; Servo srv_4B; Servo srv_4C;
Servo srv_5A; Servo srv_5B; Servo srv_5C;
Servo srv_6A; Servo srv_6B; Servo srv_6C;
Servo myservoA; Servo myservoB; Servo myservoC;
Servo srv_GR; Servo srv_UP; Servo srv_k;

Servo CapitAtas; Servo CapitTengah; Servo CapitBawah;

#define JSD 25//25//19
#define stp 47//47//36
//=========PID VARIABLE
float Kp = 5;
float Ki = 1;
float Kd = 0.1;
//=========PIT STOP VARIABLE
byte startPoint = 0;
byte stopPoint = 99;
byte wallDistance = 15;//10
byte frontDistance = 15;
int  fireLimit = 300;

//=========RUN VARIABLE
extern int JS;
int addStep = JS + JS - 2;
byte sp = 1;
float left = 1;
float right = 1;

int ulang = 0;

float leftCalibration = 1;
float rightCalibration = 1;

byte langkah = 0;
byte gaitsMode = 3; //GAITS 1. Metachronal, 2. Ripple, 3. Tripod
byte xspd = 28; //max 20
byte spd = 28 - xspd;//30
//byte spd = 5;


int dly  = 5;
float wide = 2; //max 25 //25=naik rintangan //10
float lift = 30; //max 25 //45=naik 

//int dly  = 5;
//float wide = 39; //max 25 //25=naik rintangan //10
//float lift = 25; //max 25 //45=naik rintangan

float yAxis = 20; //45  //betul = 30
float xAxis = 40;//Min 60 
float zAxis = -20;

//=========Body VARIABLE
float cx = 31; //coxa a1
float fm = 40; //femur a2
float tb = 80;//tibia a3
float Zo = 50;//40

//=======Kinematik===============================
int runAngle[] = {0, 0, 0, 0, 0, 0, 0};
byte cxAngle = 150;
byte fmAngle = 150;
byte tbAngle = 50;

byte cx0 = 0;
byte fm0 = 3;
byte tb0 = 20;

long numberStep = 0;
long jumlahLangkah = 0;

byte testSt = 0;
byte hitungJalan = 0;

//=========CALIBRATION VARIABLE
int calA[] = {0, 0, 0, 0, 0, 0, 0};
int calB[] = {0, 10, 10, 10, 10, 10, 10};
int calC[] = {0, 10, 10, 10, 10, 10, 10};

float Adl1; // untuk mencek Hasil perhitungan

byte batSt = 2;
byte readySt = 0;

byte direction = 1;
byte directionSt = 0;
long startStep = 0;
long runtStepNumber = 0;

int pos0 = 180;

//======== Trajectory variable
float U1[stp];
float W1[stp];
float U2[stp];
float W2[stp];

//======== Gaits variable
byte s0[95];
byte s1[stp];
byte s6[stp];
byte s2[stp];
byte s5[stp];
byte s4[stp];
byte s3[stp];

float pos_1A = cxAngle + calA[1], pos_1B = fmAngle + calB[1], pos_1C = tbAngle + calC[1];
float pos_2A = cxAngle + calA[2], pos_2B = fmAngle + calB[2], pos_2C = tbAngle + calC[2];
float pos_3A = cxAngle + calA[3], pos_3B = fmAngle + calB[3], pos_3C = tbAngle + calC[3];
float pos_4A = cxAngle + calA[4], pos_4B = fmAngle + calB[4], pos_4C = tbAngle + calC[4];
float pos_5A = cxAngle + calA[5], pos_5B = fmAngle + calB[5], pos_5C = tbAngle + calC[5];
float pos_6A = cxAngle + calA[6], pos_6B = fmAngle + calB[6], pos_6C = tbAngle + calC[6];

float AdL[JSD], BdL[JSD], CdL[JSD], AdR[JSD], BdR[JSD], CdR[JSD];
float AtL[JSD], BtL[JSD], CtL[JSD], AtR[JSD], BtR[JSD], CtR[JSD];
float AbL[JSD], BbL[JSD], CbL[JSD], AbR[JSD], BbR[JSD], CbR[JSD];

void beep1() {
   //tone(buzzerPin, 3136, 100);
   //tone(buzzerPin, 3729, 100);
//   NewTone(buzzerPin, 4186, 100);
//   delay(100);
//   noNewTone(buzzerPin);
//   delay(100);
}
void beep() {
   //tone(buzzerPin, 3136, 100);
   //tone(buzzerPin, 3729, 100);
   //tone(buzzerPin, 4186, 100);
//   tone(buzzerPin, 4435, 100);
  TimerFreeTone(buzzerPin, 4435, 100);
  delay(100);
//   noTone(buzzerPin);
TimerFreeTone(buzzerPin, 0,0,0);
   delay(100);
}

void beepOld() {
  //tone(buzzerPin, 5000);
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(ledOledPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
  //noTone(buzzerPin);
  digitalWrite(ledOledPin, LOW);
  delay(100);
}

float mapFloat(float x, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (x - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}


void SensorWarna(){
    // Setting the outputs
//  pinMode(WS01, OUTPUT);
//  pinMode(WS11, OUTPUT);
  pinMode(WS21, OUTPUT);
  pinMode(WS31, OUTPUT);
  // Setting the sensorOut as an input
  pinMode(WsensorOut, INPUT);
//  pinMode(A9, OUTPUT);
//  digitalWrite(A9, HIGH);
//  pinMode(A11, OUTPUT);
//  digitalWrite(A11, HIGH);
//  pinMode(A4, OUTPUT);
//  digitalWrite(A4, HIGH);

//  digitalWrite(WS01,HIGH);
//  digitalWrite(WS11 ,LOW);

  //Sensor Warna ke-2 =============================================================================
//  pinMode(A5, OUTPUT);
//  digitalWrite(A5, HIGH);
//  pinMode(A12, OUTPUT);
//  digitalWrite(A12, HIGH);
//  pinMode(A7, OUTPUT);
//  digitalWrite(A7, HIGH);
//  pinMode(A8, OUTPUT);
//  digitalWrite(A8, HIGH);
  
  // Setting frequency scaling to 20%
//  digitalWrite(S0,HIGH);
//  digitalWrite(S1,LOW);
}


void printSerial(String S1, String S2, String S3, float dA, float dB, float dC, int s) {
  Serial.print(S1 + "[" + String(s) + "] ");
  Serial.print(dA, 1); Serial.print(" \t");
  Serial.print(S2 + "[" + String(s) + "] ");
  Serial.print(dB, 1); Serial.print(" \t");
  Serial.print(S3 + "[" + String(s) + "] ");
  Serial.print(dC, 1);
  Serial.println();
  if (s == JS) Serial.println();
}

void printSerial_1() {
  Serial.print ("xAxis "); Serial.print(" \t");
  Serial.print (xAxis); Serial.print(" \t");
  Serial.print ("yAxis "); Serial.print(" \t");
  Serial.print (yAxis); Serial.print(" \t");
  Serial.print ("zAxis "); Serial.print(" \t");
  Serial.println (zAxis);
  Serial.print ("delay "); Serial.print(" \t");
  Serial.print (dly); Serial.print(" \t");
  Serial.print ("Speed "); Serial.print(" \t");
  Serial.println (spd);
}
