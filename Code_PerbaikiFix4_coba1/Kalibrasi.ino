void kalibrasi () {
  if (Serial.available()) {
    char temp = Serial.read();
    //=======================
    if (temp == 's') {
      printKalibrasi();
    }
    if (temp == 'a') {
      calA[1] --;
      srv_1A.write(cxAngle + calA[1]);
      Serial.print("calA[1]: ");
      Serial.println(calA[1]);
    }
    else if (temp == 'A') {
      calA[1] ++;
      srv_1A.write(cxAngle + calA[1]);
      Serial.print("calA[1]: ");
      Serial.println(calA[1]);
    }
    //=========
    if (temp == 'b') {
      calB[1] ++;
      srv_1B.write(fmAngle + calB[1]);
      Serial.print("calB[1]: ");
      Serial.println(calB[1]);
    }
    else if (temp == 'B') {
      calB[1] --;
      srv_1B.write(fmAngle + calB[1]);
      Serial.print("calB[1]: ");
      Serial.println(calB[1]);
    }
    //=========
    if (temp == 'c') {
      calC[1] ++;
      srv_1C.write(tbAngle + calC[1]);
      Serial.print("calC[1]: ");
      Serial.println(calC[1]);
    }
    else if (temp == 'C') {
      calC[1] --;
      srv_1C.write(tbAngle + calC[1]);
      Serial.print("calC[1]: ");
      Serial.println(calC[1]);
    }
    //=====================
    if (temp == 'd') {
      calA[2] ++;
      srv_2A.write(cxAngle + calA[2]);
      Serial.print("calA[2]: ");
      Serial.println(calA[2]);
    }
    else if (temp == 'D') {
      calA[2] --;
      srv_2A.write(cxAngle + calA[2]);
      Serial.print("calA[2]: ");
      Serial.println(calA[2]);
    }
    //=========
    if (temp == 'e') {
      calB[2] --;
      srv_2B.write(fmAngle + calB[2]);
      Serial.print("calB[2]: ");
      Serial.println(calB[2]);
    }
    else if (temp == 'E') {
      calB[2] ++;
      srv_2B.write(fmAngle + calB[2]);
      Serial.print("calB[2]: ");
      Serial.println(calB[2]);
    }
    //=========
    if (temp == 'f') {
      calC[2] --;
      srv_2C.write(tbAngle + calC[2]);
      Serial.print("calC[2]: ");
      Serial.println(calC[2]);
    }
    else if (temp == 'F') {
      calC[2] ++;
      srv_2C.write(tbAngle + calC[2]);
      Serial.print("calC[2]: ");
      Serial.println(calC[2]);
    }
    //====================
    //=======================
    if (temp == 'g') {
      calA[3] --;
      srv_3A.write(cxAngle + calA[3]);
      Serial.print("calA[3]: ");
      Serial.println(calA[3]);
    }
    else if (temp == 'G') {
      calA[3] ++;
      srv_3A.write(cxAngle + calA[3]);
      Serial.print("calA[3]: ");
      Serial.println(calA[3]);
    }
    //=========
    if (temp == 'h') {
      calB[3] ++;
      srv_3B.write(fmAngle + calB[3]);
      Serial.print("calB[3]: ");
      Serial.println(calB[3]);
    }
    else if (temp == 'H') {
      calB[3] --;
      srv_3B.write(fmAngle + calB[3]);
      Serial.print("calB[3]: ");
      Serial.println(calB[3]);
    }
    //=========
    if (temp == 'i') {
      calC[3] ++;
      srv_3C.write(tbAngle + calC[3]);
      Serial.print("calC[3]: ");
      Serial.println(calC[3]);
    }
    else if (temp == 'I') {
      calC[3] --;
      srv_3C.write(tbAngle + calC[3]);
      Serial.print("calC[3]: ");
      Serial.println(calC[3]);
    }
    //=====================
    if (temp == 'j') {
      calA[4] ++;
      srv_4A.write(cxAngle + calA[4]);
      Serial.print("calA[4]: ");
      Serial.println(calA[4]);
    }
    else if (temp == 'J') {
      calA[4] --;
      srv_4A.write(cxAngle + calA[4]);
      Serial.print("calA[4]: ");
      Serial.println(calA[4]);
    }
    //=========
    if (temp == 'k') {
      calB[4] --;
      srv_4B.write(fmAngle + calB[4]);
      Serial.print("calB[4]: ");
      Serial.println(calB[4]);
    }
    else if (temp == 'K') {
      calB[4] ++;
      srv_4B.write(fmAngle + calB[4]);
      Serial.print("calB[4]: ");
      Serial.println(calB[4]);
    }
    //=========
    if (temp == 'l') {
      calC[4] --;
      srv_4C.write(tbAngle + calC[4]);
      Serial.print("calC[4]: ");
      Serial.println(calC[4]);
    }
    else if (temp == 'L') {
      calC[4] ++;
      srv_4C.write(tbAngle + calC[4]);
      Serial.print("calC[4]: ");
      Serial.println(calC[4]);
    }
    //=======================
    if (temp == 'm') {
      calA[5] --;
      srv_5A.write(cxAngle + calA[5]);
      Serial.print("calA[5]: ");
      Serial.println(calA[5]);
    }
    else if (temp == 'M') {
      calA[5] ++;
      srv_5A.write(cxAngle + calA[5]);
      Serial.print("calA[5]: ");
      Serial.println(calA[5]);
    }
    //=========
    if (temp == 'n') {
      calB[5] ++;
      srv_5B.write(fmAngle + calB[5]);
      Serial.print("calB[5]: ");
      Serial.println(calB[5]);
    }
    else if (temp == 'N') {
      calB[5] --;
      srv_5B.write(fmAngle + calB[5]);
      Serial.print("calB[5]: ");
      Serial.println(calB[5]);
    }
    //=========
    if (temp == 'o') {
      calC[5] ++;
      srv_5C.write(tbAngle + calC[5]);
      Serial.print("calC[5]: ");
      Serial.println(calC[5]);
    }
    else if (temp == 'O') {
      calC[5] --;
      srv_5C.write(tbAngle + calC[5]);
      Serial.print("calC[5]: ");
      Serial.println(calC[5]);
    }
    //=====================
    if (temp == 'p') {
      calA[6] ++;
      srv_6A.write(cxAngle + calA[6]);
      Serial.print("calA[6]: ");
      Serial.println(calA[6]);
    }
    else if (temp == 'P') {
      calA[6] --;
      srv_6A.write(cxAngle + calA[6]);
      Serial.print("calA[6]: ");
      Serial.println(calA[6]);
    }
    //=========
    if (temp == 'q') {
      calB[6] --;
      srv_6B.write(fmAngle + calB[6]);
      Serial.print("calB[6]: ");
      Serial.println(calB[6]);
    }
    else if (temp == 'Q') {
      calB[6] ++;
      srv_6B.write(fmAngle + calB[6]);
      Serial.print("calB[6]: ");
      Serial.println(calB[6]);
    }
    //=========
    if (temp == 'r') {
      calC[6] --;
      srv_6C.write(tbAngle + calC[6]);
      Serial.print("calC[6]: ");
      Serial.println(calC[6]);
    }
    else if (temp == 'R') {
      calC[6] ++;
      srv_6C.write(tbAngle + calC[6]);
      Serial.print("calC[6]: ");
      Serial.println(calC[6]);
    }
    if (testSt != 1) {
      saveCalibrationVariable();
    }
  }
}

void printKalibrasi () {
  Serial.print("calA[1]: "); Serial.print("\t"); Serial.print(calA[1]); Serial.print("\t"); Serial.print("calB[1]: "); Serial.print(calB[1]); Serial.print("\t"); Serial.print("calC[1]: "); Serial.println(calC[1]); //Serial.print("\t");
  Serial.print("calA[2]: "); Serial.print("\t"); Serial.print(calA[2]); Serial.print("\t"); Serial.print("calB[2]: "); Serial.print(calB[2]); Serial.print("\t"); Serial.print("calC[2]: "); Serial.println(calC[2]); //Serial.print("\t");
  Serial.print("calA[3]: "); Serial.print("\t"); Serial.print(calA[3]); Serial.print("\t"); Serial.print("calB[3]: "); Serial.print(calB[3]); Serial.print("\t"); Serial.print("calC[3]: "); Serial.println(calC[3]); //Serial.print("\t");
  Serial.print("calA[4]: "); Serial.print("\t"); Serial.print(calA[4]); Serial.print("\t"); Serial.print("calB[4]: "); Serial.print(calB[4]); Serial.print("\t"); Serial.print("calC[4]: "); Serial.println(calC[4]); //Serial.print("\t");
  Serial.print("calA[5]: "); Serial.print("\t"); Serial.print(calA[5]); Serial.print("\t"); Serial.print("calB[5]: "); Serial.print(calB[5]); Serial.print("\t"); Serial.print("calC[5]: "); Serial.println(calC[5]); //Serial.print("\t");
  Serial.print("calA[6]: "); Serial.print("\t"); Serial.print(calA[6]); Serial.print("\t"); Serial.print("calB[6]: "); Serial.print(calB[6]); Serial.print("\t"); Serial.print("calC[6]: "); Serial.println(calC[6]); //Serial.print("\t");
}
