

//================================================================================================


byte a1, a2, a3, a4, a5, a6, b1, b2, b3, b4, b5, b6, c1, c2, c3, c4, c5, c6, g1, g2, g3;
int tunda = 2;

byte calibStA = 0;
byte calibStB = 0;
byte calibStC = 0;
//Dynaimxel===========================================
void activeServoDynamixel(byte x) {
  switch  (x) {
    case 1:
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
     
      break;
  }
  //saveCalibrationVariable();
  EEPROM.writeInt(349, 1);
  ukurJarak();
  openCalibrationVariable();
  activeGriper();
//  activeLeg1(); delay(tunda);// kaki 1
//  activeLeg2(); delay(tunda);// kaki 2
//  activeLeg3(); delay(tunda);// kaki 3
//  activeLeg4(); delay(tunda);// kaki 4
//  activeLeg5(); delay(tunda);// kaki 5
  activeLeg6(); delay(tunda);// kaki 6
}




//=======================================================
void activeServo(byte x) {
  switch  (x) {
    case 1:
      a1 = 38, b1 = 39, c1 = 40;
      a2 = 18, b2 = 19, c2 = 22;
      a3 = 35, b3 = 33, c3 = 37;
      a4 = 6,  b4 = 8,  c4 = 9;
      a5 = 32, b5 = 34, c5 = 36;
      a6 = 2,  b6 = 3,  c6 = 4;
      g1 = 20, g2 = 31, g3 = 30;
      break;
  }
  //saveCalibrationVariable();
  EEPROM.writeInt(349, 1);
//  ukurJarak();
  openCalibrationVariable();
  activeGriper();
  activeLeg1(); delay(tunda);// kaki 1
  activeLeg2(); delay(tunda);// kaki 2
  activeLeg3(); delay(tunda);// kaki 3
  activeLeg4(); delay(tunda);// kaki 4
  activeLeg5(); delay(tunda);// kaki 5
  activeLeg6(); delay(tunda);// kaki 6
}
byte posGp = 15;//100 buka - 30 tutup
byte posUd = 115  ;//100 naik - 30 turun
byte gripperSt = 1;
byte UDSt = 0;

//Dynamixel
void setupServoDynamixel(byte x) {
  //Serial.print ("Test Value AdL[1] ");
  //Serial.println (Adl1);
  if ((Adl1 > 0) and (Adl1 < 180)) {
    activeServoDynamixel(x);
    readySt = 1;
    testSt = 0;
    Serial.println ("READY TO RUN");
  } else {
    readySt = 0;
    testSt = 0;
    Serial.println ("NOT READY ");
    beep();
  }

  printOut = 0;
}






void setupServo(byte x) {
  //Serial.print ("Test Value AdL[1] ");
  //Serial.println (Adl1);
  if ((Adl1 > 0) and (Adl1 < 180)) {
    activeServo(x);
    readySt = 1;
    testSt = 0;
    Serial.println ("READY TO RUN");
  } else {
    readySt = 0;
    testSt = 0;
    Serial.println ("NOT READY ");
    beep();
  }

//  Serial.println();
  printOut = 0;
}

void activeLeg1() {
//  if (calibStA == 0)srv_1A.attach(a1, 600, 2400); srv_1A.write(pos_1A); delay(tunda);
//  if (calibStB == 0)srv_1B.attach(b1, 600, 2400); srv_1B.write(pos_1B); delay(tunda);
//  if (calibStC == 0)srv_1C.attach(c1, 600, 2400); srv_1C.write(pos_1C);//delay(tunda);

//  dxl.setGoalVelocity(DXL_ID6, 500);
//  dxl.setGoalPosition(DXL_ID6, pos_1A, UNIT_DEGREE); delay(tunda);
//  dxl.setGoalVelocity(DXL_ID5, 500);
//  dxl.setGoalPosition(DXL_ID5, pos_1B, UNIT_DEGREE); delay(tunda);
//  dxl.setGoalVelocity(DXL_ID4, 500);
//  dxl.setGoalPosition(DXL_ID4, pos_1C, UNIT_DEGREE); //delay(tunda);

  // Konversi 
  int pos_1AKonversi = degreeToGoalPosition(pos_1A, positionMin, positionMax, degreeMin, degreeMax);
  int pos_1BKonversi = degreeToGoalPosition(pos_1B, positionMin, positionMax, degreeMin, degreeMax);
  int pos_1CKonversi = degreeToGoalPosition(pos_1C, positionMin, positionMax, degreeMin, degreeMax);
  dxl.write(DXL_ID6, GOAL_POSITION_ADDR, (uint8_t*)&pos_1AKonversi, GOAL_POSITION_ADDR_LEN, TIMEOUT);
  dxl.write(DXL_ID5, GOAL_POSITION_ADDR, (uint8_t*)&pos_1BKonversi, GOAL_POSITION_ADDR_LEN, TIMEOUT);
  dxl.write(DXL_ID4, GOAL_POSITION_ADDR, (uint8_t*)&pos_1CKonversi, GOAL_POSITION_ADDR_LEN, TIMEOUT);
  
}

void activeLeg2() {
//  if (calibStA == 0)srv_2A.attach(a2, 600, 2400); srv_2A.write(pos_2A); delay(tunda);
//  if (calibStB == 0)srv_2B.attach(b2, 600, 2400); srv_2B.write(pos_2B); delay(tunda);
//  if (calibStC == 0)srv_2C.attach(c2, 600, 2400); srv_2C.write(pos_2C);//delay(tunda);

//  dxl.setGoalVelocity(DXL_ID3, 500);
  dxl.setGoalPosition(DXL_ID3, pos_2A, UNIT_DEGREE); // delay(tunda);
//  dxl.setGoalVelocity(DXL_ID2, 500);
  dxl.setGoalPosition(DXL_ID2, pos_2B, UNIT_DEGREE); // delay(tunda);
//  dxl.setGoalVelocity(DXL_ID1, 500);
  dxl.setGoalPosition(DXL_ID1, pos_2C, UNIT_DEGREE); //delay(tunda);
}

void activeLeg3() {
//  if (calibStA == 0)srv_3A.attach(a3, 600, 2400); srv_3A.write(pos_3A); delay(tunda);
//  if (calibStB == 0)srv_3B.attach(b3, 600, 2400); srv_3B.write(pos_3B); delay(tunda);
//  if (calibStC == 0)srv_3C.attach(c3, 600, 2400); srv_3C.write(pos_3C);//delay(tunda);

//  dxl.setGoalVelocity(DXL_ID12, 500);
  dxl.setGoalPosition(DXL_ID12, pos_3A, UNIT_DEGREE); // delay(tunda);
//  dxl.setGoalVelocity(DXL_ID11, 500);
  dxl.setGoalPosition(DXL_ID11, (pos_3B), UNIT_DEGREE); // delay(tunda);
//  dxl.setGoalVelocity(DXL_ID10, 500);
  dxl.setGoalPosition(DXL_ID10, pos_3C, UNIT_DEGREE); //delay(tunda);
  
}

void activeLeg4() {
//  if (calibStA == 0)srv_4A.attach(a4, 600, 2400); srv_4A.write(pos_4A); delay(tunda);
//  if (calibStB == 0)srv_4B.attach(b4, 600, 2400); srv_4B.write(pos_4B); delay(tunda);
//  if (calibStC == 0)srv_4C.attach(c4, 600, 2400); srv_4C.write(pos_4C);//delay(tunda);

//  dxl.setGoalVelocity(DXL_ID9, 500);
  dxl.setGoalPosition(DXL_ID9, pos_4A, UNIT_DEGREE); // delay(tunda);
//  dxl.setGoalVelocity(DXL_ID8, 500);
  dxl.setGoalPosition(DXL_ID8, pos_4B, UNIT_DEGREE); // delay(tunda);
//  dxl.setGoalVelocity(DXL_ID7, 500);
  dxl.setGoalPosition(DXL_ID7, pos_4C, UNIT_DEGREE); //delay(tunda);

}

void activeLeg5() {
//  if (calibStA == 0)srv_5A.attach(a5, 600, 2400); srv_5A.write(pos_5A); delay(tunda);
//  if (calibStB == 0)srv_5B.attach(b5, 600, 2400); srv_5B.write(pos_5B); delay(tunda);
//  if (calibStC == 0)srv_5C.attach(c5, 600, 2400); srv_5C.write(pos_5C);//delay(tunda);

//  dxl.setGoalVelocity(DXL_ID18, 500);
  dxl.setGoalPosition(DXL_ID18, pos_5A, UNIT_DEGREE); // delay(tunda);
//  dxl.setGoalVelocity(DXL_ID17, 500);
  dxl.setGoalPosition(DXL_ID17, pos_5B, UNIT_DEGREE); // delay(tunda);
//  dxl.setGoalVelocity(DXL_ID16, 500);
  dxl.setGoalPosition(DXL_ID16, pos_5C, UNIT_DEGREE); //delay(tunda);

}

void activeLeg6() {
//  if (calibStA == 0)srv_6A.attach(a6, 600, 2400); srv_6A.write(pos_6A); delay(tunda);
//  if (calibStB == 0)srv_6B.attach(b6, 600, 2400); srv_6B.write(pos_6B); delay(tunda);
//  if (calibStC == 0)srv_6C.attach(c6, 600, 2400); srv_6C.write(pos_6C);//delay(tunda);

//  dxl.setGoalVelocity(DXL_ID15, 500);
  dxl.setGoalPosition(DXL_ID15, pos_6A, UNIT_DEGREE); // delay(tunda);
//  dxl.setGoalVelocity(DXL_ID14, 500);
  dxl.setGoalPosition(DXL_ID14, pos_6B, UNIT_DEGREE); // delay(tunda);
//  dxl.setGoalVelocity(DXL_ID13, 500);
  dxl.setGoalPosition(DXL_ID13, pos_6C, UNIT_DEGREE); //delay(tunda);
}

void servoRun() {
//  dxl.setGoalVelocity(DXL_ID6, 500);
  dxl.setGoalPosition(DXL_ID6, pos_1A, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID5, 500);
  dxl.setGoalPosition(DXL_ID5, pos_1B, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID4, 500);
  dxl.setGoalPosition(DXL_ID4, pos_1C, UNIT_DEGREE); //delay(tunda);

//  dxl.setGoalVelocity(DXL_ID3, 500);
  dxl.setGoalPosition(DXL_ID3, pos_2A, UNIT_DEGREE); // delay(tunda);
//  dxl.setGoalVelocity(DXL_ID2, 500);
  dxl.setGoalPosition(DXL_ID2, pos_2B, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID1, 500);
  dxl.setGoalPosition(DXL_ID1, pos_2C, UNIT_DEGREE); //delay(tunda);

//  dxl.setGoalVelocity(DXL_ID12, 500);
  dxl.setGoalPosition(DXL_ID12, pos_3A, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID11, 500);
  dxl.setGoalPosition(DXL_ID11, pos_3B, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID10, 500);
  dxl.setGoalPosition(DXL_ID10, pos_3C, UNIT_DEGREE); //delay(tunda);

//  dxl.setGoalVelocity(DXL_ID9, 500);
  dxl.setGoalPosition(DXL_ID9, pos_4A, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID8, 500);
  dxl.setGoalPosition(DXL_ID8, pos_4B, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID7, 500);
  dxl.setGoalPosition(DXL_ID7, pos_4C, UNIT_DEGREE); //delay(tunda);

//  dxl.setGoalVelocity(DXL_ID18, 500);
  dxl.setGoalPosition(DXL_ID18, pos_5A, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID17, 500);
  dxl.setGoalPosition(DXL_ID17, pos_5B, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID16, 500);
  dxl.setGoalPosition(DXL_ID16, pos_5C, UNIT_DEGREE); //delay(tunda);

//  dxl.setGoalVelocity(DXL_ID15, 500);
  dxl.setGoalPosition(DXL_ID15, pos_6A, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID14, 500);
  dxl.setGoalPosition(DXL_ID14, pos_6B, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID13, 500);
  dxl.setGoalPosition(DXL_ID13, pos_6C, UNIT_DEGREE); //delay(tunda);
}

void step0() {
//  dxl.setGoalVelocity(DXL_ID6, 500);
  dxl.setGoalPosition(DXL_ID6, pos_1A, UNIT_DEGREE); // delay(tunda);
//  dxl.setGoalVelocity(DXL_ID5, 500);
  dxl.setGoalPosition(DXL_ID5, pos_1B, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID4, 500);
  dxl.setGoalPosition(DXL_ID4, pos_1C, UNIT_DEGREE); //delay(tunda);

//  dxl.setGoalVelocity(DXL_ID3, 500);
  dxl.setGoalPosition(DXL_ID3, pos_2A, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID2, 500);
  dxl.setGoalPosition(DXL_ID2, pos_2B, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID1, 500);
  dxl.setGoalPosition(DXL_ID1, pos_2C, UNIT_DEGREE);// delay(tunda);

//  dxl.setGoalVelocity(DXL_ID12, 500);
  dxl.setGoalPosition(DXL_ID12, pos_3A, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID11, 500);
  dxl.setGoalPosition(DXL_ID11, pos_3B, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID10, 500);
  dxl.setGoalPosition(DXL_ID10, pos_3C, UNIT_DEGREE); //delay(tunda);

//  dxl.setGoalVelocity(DXL_ID9, 500);
  dxl.setGoalPosition(DXL_ID9, pos_4A, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID8, 500);
  dxl.setGoalPosition(DXL_ID8, pos_4B, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID7, 500);
  dxl.setGoalPosition(DXL_ID7, pos_4C, UNIT_DEGREE); //delay(tunda);

//  dxl.setGoalVelocity(DXL_ID18, 500);
  dxl.setGoalPosition(DXL_ID18, pos_5A, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID17, 500);
  dxl.setGoalPosition(DXL_ID17, pos_5B, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID16, 500);
  dxl.setGoalPosition(DXL_ID16, pos_5C, UNIT_DEGREE); //delay(tunda);

//  dxl.setGoalVelocity(DXL_ID15, 500);
  dxl.setGoalPosition(DXL_ID15, pos_6A, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID14, 500);
  dxl.setGoalPosition(DXL_ID14, pos_6B, UNIT_DEGREE); //delay(tunda);
//  dxl.setGoalVelocity(DXL_ID13, 500);
  dxl.setGoalPosition(DXL_ID13, pos_6C, UNIT_DEGREE); //delay(tunda);
}

void setServoA(int x, int p) {
  myservoA.attach(x);
  myservoA.write(p);
}

void setServoB(int x, int p) {
  myservoB.attach(x);
  myservoB.write(p);
}

void setServoC(int x, int p) {
  myservoC.attach(x);
  myservoC.write(p);
}

int pos = 0;
void testServo(int x) {
  myservoC.attach(x);
  for (pos = 0; pos <= 170; pos += 1) {
    myservoC.write(pos);
    delay(25);
  }

  for (pos = 170; pos >= 0; pos -= 1) {
    myservoC.write(pos);
    delay(25);
  }
}
