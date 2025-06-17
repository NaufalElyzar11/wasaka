/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

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

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID1);
  dxl.ping(DXL_ID2);
  dxl.ping(DXL_ID3);
  dxl.ping(DXL_ID4);
  dxl.ping(DXL_ID5);
  dxl.ping(DXL_ID6);
  dxl.ping(DXL_ID7);
  dxl.ping(DXL_ID8);
  dxl.ping(DXL_ID9);
  dxl.ping(DXL_ID10);
  dxl.ping(DXL_ID11);
  dxl.ping(DXL_ID12);
  dxl.ping(DXL_ID13);
  dxl.ping(DXL_ID14);
  dxl.ping(DXL_ID15);
  dxl.ping(DXL_ID16);
  dxl.ping(DXL_ID17);
  dxl.ping(DXL_ID18);

  // Turn off torque when configuring items in EEPROM area

  // ===== Coxa ===== //

  // Kanan
  dxl.torqueOff(DXL_ID3);
  dxl.setOperatingMode(DXL_ID3, OP_POSITION);
  dxl.torqueOn(DXL_ID3);

  dxl.torqueOff(DXL_ID9);
  dxl.setOperatingMode(DXL_ID9, OP_POSITION);
  dxl.torqueOn(DXL_ID9);

  dxl.torqueOff(DXL_ID15);
  dxl.setOperatingMode(DXL_ID15, OP_POSITION);
  dxl.torqueOn(DXL_ID15);


  // Kiri
  dxl.torqueOff(DXL_ID6);
  dxl.setOperatingMode(DXL_ID6, OP_POSITION);
  dxl.torqueOn(DXL_ID6);
  
  dxl.torqueOff(DXL_ID12);
  dxl.setOperatingMode(DXL_ID12, OP_POSITION);
  dxl.torqueOn(DXL_ID12);

  dxl.torqueOff(DXL_ID18);
  dxl.setOperatingMode(DXL_ID18, OP_POSITION);
  dxl.torqueOn(DXL_ID18);

  // ===== Femur ===== //

  // Kanan
  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_POSITION);
  dxl.torqueOn(DXL_ID2);

  dxl.torqueOff(DXL_ID8);
  dxl.setOperatingMode(DXL_ID8, OP_POSITION);
  dxl.torqueOn(DXL_ID8);

  dxl.torqueOff(DXL_ID14);
  dxl.setOperatingMode(DXL_ID14, OP_POSITION);
  dxl.torqueOn(DXL_ID14);

  // Kiri
  dxl.torqueOff(DXL_ID5);
  dxl.setOperatingMode(DXL_ID5, OP_POSITION);
  dxl.torqueOn(DXL_ID5);

  dxl.torqueOff(DXL_ID11);
  dxl.setOperatingMode(DXL_ID11, OP_POSITION);
  dxl.torqueOn(DXL_ID11);

  dxl.torqueOff(DXL_ID17);
  dxl.setOperatingMode(DXL_ID17, OP_POSITION);
  dxl.torqueOn(DXL_ID17);

  // ===== Tibia ===== //

  // Kanan
  dxl.torqueOff(DXL_ID1);
  dxl.setOperatingMode(DXL_ID1, OP_POSITION);
  dxl.torqueOn(DXL_ID1);

  dxl.torqueOff(DXL_ID7);
  dxl.setOperatingMode(DXL_ID7, OP_POSITION);
  dxl.torqueOn(DXL_ID7);

  dxl.torqueOff(DXL_ID13);
  dxl.setOperatingMode(DXL_ID13, OP_POSITION);
  dxl.torqueOn(DXL_ID13);

  // Kiri
  dxl.torqueOff(DXL_ID4);
  dxl.setOperatingMode(DXL_ID4, OP_POSITION);
  dxl.torqueOn(DXL_ID4);

  dxl.torqueOff(DXL_ID10);
  dxl.setOperatingMode(DXL_ID10, OP_POSITION);
  dxl.torqueOn(DXL_ID10);

  dxl.torqueOff(DXL_ID16);
  dxl.setOperatingMode(DXL_ID16, OP_POSITION);
  dxl.torqueOn(DXL_ID16);
  
  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID4, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID5, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID6, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID7, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID8, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID9, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID10, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID11, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID12, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID13, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID14, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID15, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID16, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID17, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID18, 30);

  // ===== Normalisasi ===== //
  dxl.setGoalVelocity(DXL_ID1, 200);
  dxl.setGoalPosition(DXL_ID1, 512);

  dxl.setGoalVelocity(DXL_ID2, 200);
  dxl.setGoalPosition(DXL_ID2, 512);

  dxl.setGoalVelocity(DXL_ID3, 200);
  dxl.setGoalPosition(DXL_ID3, 512);

  dxl.setGoalVelocity(DXL_ID4, 200);
  dxl.setGoalPosition(DXL_ID4, 512);

  dxl.setGoalVelocity(DXL_ID5, 200);
  dxl.setGoalPosition(DXL_ID5, 512);

//  dxl.setGoalVelocity(DXL_ID6, 200);
//  dxl.setGoalPosition(DXL_ID6, 512);

  dxl.setGoalVelocity(DXL_ID7, 200);
  dxl.setGoalPosition(DXL_ID7, 512);

  dxl.setGoalVelocity(DXL_ID8, 200);
  dxl.setGoalPosition(DXL_ID8, 512);

  dxl.setGoalVelocity(DXL_ID9, 200);
  dxl.setGoalPosition(DXL_ID9, 512);

  dxl.setGoalVelocity(DXL_ID10, 200);
  dxl.setGoalPosition(DXL_ID10, 512);
  
  dxl.setGoalVelocity(DXL_ID11, 200);
  dxl.setGoalPosition(DXL_ID11, 512);

  dxl.setGoalVelocity(DXL_ID12, 200);
  dxl.setGoalPosition(DXL_ID12, 512);

  dxl.setGoalVelocity(DXL_ID13, 200);
  dxl.setGoalPosition(DXL_ID13, 512);
  
  dxl.setGoalVelocity(DXL_ID14, 200);
  dxl.setGoalPosition(DXL_ID14, 512);

  dxl.setGoalVelocity(DXL_ID15, 200);
  dxl.setGoalPosition(DXL_ID15, 512);

  dxl.setGoalVelocity(DXL_ID16, 200);
  dxl.setGoalPosition(DXL_ID16, 512);

  dxl.setGoalVelocity(DXL_ID17, 200);
  dxl.setGoalPosition(DXL_ID17, 512);

  dxl.setGoalVelocity(DXL_ID18, 200);
  dxl.setGoalPosition(DXL_ID18, 512);

  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 

  // ===== Goals Servo 1 ===== //
  
  dxl.setGoalVelocity(DXL_ID1, 200);
  dxl.setGoalPosition(DXL_ID1, 205);

  dxl.setGoalVelocity(DXL_ID2, 200);
  dxl.setGoalPosition(DXL_ID2, 512);

  dxl.setGoalVelocity(DXL_ID3, 200);
  dxl.setGoalPosition(DXL_ID3, 512);

  dxl.setGoalVelocity(DXL_ID4, 200);
  dxl.setGoalPosition(DXL_ID4, 818);

  dxl.setGoalVelocity(DXL_ID5, 200);
  dxl.setGoalPosition(DXL_ID5, 512);

//  dxl.setGoalVelocity(DXL_ID6, 200);
//  dxl.setGoalPosition(DXL_ID6, 512);

  dxl.setGoalVelocity(DXL_ID7, 200);
  dxl.setGoalPosition(DXL_ID7, 205);

  dxl.setGoalVelocity(DXL_ID8, 200);
  dxl.setGoalPosition(DXL_ID8, 512);

  dxl.setGoalVelocity(DXL_ID9, 200);
  dxl.setGoalPosition(DXL_ID9, 512);

  dxl.setGoalVelocity(DXL_ID10, 200);
  dxl.setGoalPosition(DXL_ID10, 818);
  
  dxl.setGoalVelocity(DXL_ID11, 200);
  dxl.setGoalPosition(DXL_ID11, 512);

  dxl.setGoalVelocity(DXL_ID12, 200);
  dxl.setGoalPosition(DXL_ID12, 512);

  dxl.setGoalVelocity(DXL_ID13, 200);
  dxl.setGoalPosition(DXL_ID13, 205);
  
  dxl.setGoalVelocity(DXL_ID14, 200);
  dxl.setGoalPosition(DXL_ID14, 512);

  dxl.setGoalVelocity(DXL_ID15, 200);
  dxl.setGoalPosition(DXL_ID15, 512);

  dxl.setGoalVelocity(DXL_ID16, 200);
  dxl.setGoalPosition(DXL_ID16, 818);

  dxl.setGoalVelocity(DXL_ID17, 200);
  dxl.setGoalPosition(DXL_ID17, 512);

  dxl.setGoalVelocity(DXL_ID18, 200);
  dxl.setGoalPosition(DXL_ID18, 512);

  delay(2000);

  // ===== Goals Servo 2 ===== //
  
  dxl.setGoalVelocity(DXL_ID1, 200);
  dxl.setGoalPosition(DXL_ID1, 512);

  dxl.setGoalVelocity(DXL_ID2, 200);
  dxl.setGoalPosition(DXL_ID2, 818);

  dxl.setGoalVelocity(DXL_ID3, 200);
  dxl.setGoalPosition(DXL_ID3, 512);

  dxl.setGoalVelocity(DXL_ID4, 200);
  dxl.setGoalPosition(DXL_ID4, 512);

  dxl.setGoalVelocity(DXL_ID5, 200);
  dxl.setGoalPosition(DXL_ID5, 205);

//  dxl.setGoalVelocity(DXL_ID6, 200);
//  dxl.setGoalPosition(DXL_ID6, 512);

  dxl.setGoalVelocity(DXL_ID7, 200);
  dxl.setGoalPosition(DXL_ID7, 512);

  dxl.setGoalVelocity(DXL_ID8, 200);
  dxl.setGoalPosition(DXL_ID8, 818);

  dxl.setGoalVelocity(DXL_ID9, 200);
  dxl.setGoalPosition(DXL_ID9, 512);

  dxl.setGoalVelocity(DXL_ID10, 200);
  dxl.setGoalPosition(DXL_ID10, 512);
  
  dxl.setGoalVelocity(DXL_ID11, 200);
  dxl.setGoalPosition(DXL_ID11, 205);

  dxl.setGoalVelocity(DXL_ID12, 200);
  dxl.setGoalPosition(DXL_ID12, 512);

  dxl.setGoalVelocity(DXL_ID13, 200);
  dxl.setGoalPosition(DXL_ID13, 512);
  
  dxl.setGoalVelocity(DXL_ID14, 200);
  dxl.setGoalPosition(DXL_ID14, 818);

  dxl.setGoalVelocity(DXL_ID15, 200);
  dxl.setGoalPosition(DXL_ID15, 512);

  dxl.setGoalVelocity(DXL_ID16, 200);
  dxl.setGoalPosition(DXL_ID16, 512);

  dxl.setGoalVelocity(DXL_ID17, 200);
  dxl.setGoalPosition(DXL_ID17, 205);

  dxl.setGoalVelocity(DXL_ID18, 200);
  dxl.setGoalPosition(DXL_ID18, 512);

  delay(2000);

  // ===== Goals Servo 3 ===== //
  
  dxl.setGoalVelocity(DXL_ID1, 200);
  dxl.setGoalPosition(DXL_ID1, 65);

  dxl.setGoalVelocity(DXL_ID2, 200);
  dxl.setGoalPosition(DXL_ID2, 372);

  dxl.setGoalVelocity(DXL_ID3, 200);
  dxl.setGoalPosition(DXL_ID3, 512);

  dxl.setGoalVelocity(DXL_ID4, 200);
  dxl.setGoalPosition(DXL_ID4, 958);

  dxl.setGoalVelocity(DXL_ID5, 200);
  dxl.setGoalPosition(DXL_ID5, 651);

//  dxl.setGoalVelocity(DXL_ID6, 200);
//  dxl.setGoalPosition(DXL_ID6, 512);

  dxl.setGoalVelocity(DXL_ID7, 200);
  dxl.setGoalPosition(DXL_ID7, 65);

  dxl.setGoalVelocity(DXL_ID8, 200);
  dxl.setGoalPosition(DXL_ID8, 372);

  dxl.setGoalVelocity(DXL_ID9, 200);
  dxl.setGoalPosition(DXL_ID9, 512);

  dxl.setGoalVelocity(DXL_ID10, 200);
  dxl.setGoalPosition(DXL_ID10, 958);
  
  dxl.setGoalVelocity(DXL_ID11, 200);
  dxl.setGoalPosition(DXL_ID11, 651);

  dxl.setGoalVelocity(DXL_ID12, 200);
  dxl.setGoalPosition(DXL_ID12, 512);

  dxl.setGoalVelocity(DXL_ID13, 200);
  dxl.setGoalPosition(DXL_ID13, 65);
  
  dxl.setGoalVelocity(DXL_ID14, 200);
  dxl.setGoalPosition(DXL_ID14, 372);

  dxl.setGoalVelocity(DXL_ID15, 200);
  dxl.setGoalPosition(DXL_ID15, 512);

  dxl.setGoalVelocity(DXL_ID16, 200);
  dxl.setGoalPosition(DXL_ID16, 958);

  dxl.setGoalVelocity(DXL_ID17, 200);
  dxl.setGoalPosition(DXL_ID17, 651);

  dxl.setGoalVelocity(DXL_ID18, 200);
  dxl.setGoalPosition(DXL_ID18, 512);

  delay(2000);

  // Set Goal Position in DEGREE value
//  dxl.setGoalPosition(DXL_ID, 5.7, UNIT_DEGREE);
//  
//  while (abs(5.7 - f_present_position) > 2.0)
//  {
//    f_present_position = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
//    DEBUG_SERIAL.print("Present_Position(degree) : ");
//    DEBUG_SERIAL.println(f_present_position);
//  }
//  delay(1000);
}
