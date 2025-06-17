void EEPROM_ARR(int d) {
  int saveDataSt = EEPROM.readInt(0);
  if (saveDataSt != d) {
    savePitStopVariable();
    saveRunningVariable();
    savePIDvariable();
    saveBodyVariable();
    saveDataSt = d;
    EEPROM.writeInt(0, saveDataSt);
    Serial.println("Saving Data");
  }
  openPitStopVariable();
  openRunningVariable();
  openPIDvariable();
  openBodyVariable();
}

void savePitStopVariable() {
  EEPROM.write(401, startPoint);
  EEPROM.write(402, stopPoint);
  EEPROM.write(403, wallDistance);
  EEPROM.write(404, frontDistance);
  EEPROM.writeInt(405, fireLimit);
}
void openPitStopVariable() {
  startPoint = EEPROM.read(401);
  stopPoint = EEPROM.read(402);
  wallDistance = EEPROM.read(403);
  frontDistance = EEPROM.read(404);
  fireLimit = EEPROM.readInt(405);
}
void saveRunningVariable() {
  EEPROM.write(410, JS);
  EEPROM.write(411, gaitsMode);
  EEPROM.write(412, xspd);
  EEPROM.write(413, dly);
  EEPROM.write(414, wide);
  EEPROM.write(415, lift);
  EEPROM.write(416, smothtly);

  EEPROM.writeFloat(451, xAxis);
  EEPROM.writeFloat(455, yAxis);
  EEPROM.writeFloat(459, zAxis);
  delay(50);
  openRunningVariable();
}

void openRunningVariable() {
  JS = EEPROM.read(410);
  gaitsMode = EEPROM.read(411);
  xspd = EEPROM.read(412);
  dly = EEPROM.read(413);
  wide = EEPROM.read(414);
  lift = EEPROM.read(415);
  smothtly = EEPROM.read(416);
  spd = 23 - xspd;
  xAxis = EEPROM.readFloat(451);
  yAxis = EEPROM.readFloat(455);
  zAxis = EEPROM.readFloat(459);
  //spd = 15 - xspd;
  //spdTemp = spd;
  //Serial.print("xspd ");
  //Serial.println(xspd);
  //Serial.print("Ambil spd ");
  //Serial.println(spd);
}

void savePIDvariable() {
  EEPROM.writeFloat(601, Kp);
  EEPROM.writeFloat(605, Ki);
  EEPROM.writeFloat(609, Kd);
  delay(50);
  openPIDvariable();
}
void saveBodyVariable() {
  EEPROM.writeFloat(621, cx);
  EEPROM.writeFloat(625, fm);
  EEPROM.writeFloat(629, tb);
  EEPROM.writeFloat(633, Zo);
  delay(50);
  openBodyVariable();
}

void openBodyVariable() {
  cx = EEPROM.readFloat(621);
  fm = EEPROM.readFloat(625);
  tb = EEPROM.readFloat(629);
  Zo = EEPROM.readFloat(633);
}

void openPIDvariable() {
  Kp = EEPROM.readFloat(601);
  Ki = EEPROM.readFloat(605);
  Kd = EEPROM.readFloat(609);
}

void saveCalibrationVariable() {
  EEPROM.writeInt(349, 1);
  
  EEPROM.writeInt(351, calA[1]);
  EEPROM.writeInt(353, calB[1]);
  EEPROM.writeInt(355, calC[1]);

  EEPROM.writeInt(357, calA[2]);
  EEPROM.writeInt(359, calB[2]);
  EEPROM.writeInt(361, calC[2]);

  EEPROM.writeInt(363, calA[3]);
  EEPROM.writeInt(365, calB[3]);
  EEPROM.writeInt(367, calC[3]);

  EEPROM.writeInt(369, calA[4]);
  EEPROM.writeInt(371, calB[4]);
  EEPROM.writeInt(373, calC[4]);

  EEPROM.writeInt(375, calA[5]);
  EEPROM.writeInt(377, calB[5]);
  EEPROM.writeInt(379, calC[5]);

  EEPROM.writeInt(381, calA[6]);
  EEPROM.writeInt(383, calB[6]);
  EEPROM.writeInt(385, calC[6]);
  delay(50);
  openCalibrationVariable();
}

void openCalibrationVariable() {
  if (EEPROM.readInt(349) != 1) {
    saveCalibrationVariable();
  }
  
  calA[1] = EEPROM.readInt(351);
  calB[1] = EEPROM.readInt(353);
  calC[1] = EEPROM.readInt(355);

  calA[2] = EEPROM.readInt(357);
  calB[2] = EEPROM.readInt(359);
  calC[2] = EEPROM.readInt(361);

  calA[3] = EEPROM.readInt(363);
  calB[3] = EEPROM.readInt(365);
  calC[3] = EEPROM.readInt(367);

  calA[4] = EEPROM.readInt(369);
  calB[4] = EEPROM.readInt(371);
  calC[4] = EEPROM.readInt(373);

  calA[5] = EEPROM.readInt(375);
  calB[5] = EEPROM.readInt(377);
  calC[5] = EEPROM.readInt(379);

  calA[6] = EEPROM.readInt(381);
  calB[6] = EEPROM.readInt(383);
  calC[6] = EEPROM.readInt(385);
  
  pos_1A = cxAngle + calA[1], pos_1B = fmAngle + calB[1], pos_1C = tbAngle + calC[1];
  pos_2A = cxAngle + calA[2], pos_2B = fmAngle + calB[2], pos_2C = tbAngle + calC[2];
  pos_3A = cxAngle + calA[3], pos_3B = fmAngle + calB[3], pos_3C = tbAngle + calC[3];
  pos_4A = cxAngle + calA[4], pos_4B = fmAngle + calB[4], pos_4C = tbAngle + calC[4];
  pos_5A = cxAngle + calA[5], pos_5B = fmAngle + calB[5], pos_5C = tbAngle + calC[5];
  pos_6A = cxAngle + calA[6], pos_6B = fmAngle + calB[6], pos_6C = tbAngle + calC[6];
}
