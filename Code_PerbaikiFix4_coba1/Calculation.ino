float x0[7], y0[7], z0[7];
float x1[7], y1[7], z1[7];

float zPs[JSD];
float xPs[JSD];
float yPs[JSD];

void moveToCalculation(byte d) {
  long y = millis();
  if ((printOut == 5) or (printOut == 1)) {
    printSerial_1();
  }
  float yAdjdb;
  float yAdjt;
  direction = d;

  if (d == 1) { // maju
    yAdjdb = 30;
    yAdjt = 0;
    runAngle[1] = -15;
    runAngle[2] = 15;
    runAngle[3] = 0;
    runAngle[4] = 0;
    runAngle[5] = 15;
    runAngle[6] = -15;

    y0[1] = (yAdjt + yAxis) / 2;          y1[1] = (yAdjt + yAxis) / 2 * -1;
    y0[2] = (yAdjt + yAxis) / 2 * -1;     y1[2] = (yAdjt + yAxis) / 2;
    y0[3] = (yAdjt + yAxis  ) / 2;          y1[3] = (yAdjt + yAxis) / 2 * -1;

    y0[4] = (yAdjt + yAxis) / 2;          y1[4] = (yAdjt + yAxis) / 2 * -1;
    y0[5] = (yAdjt + yAxis) / 2 * -1;     y1[5] = (yAdjt + yAxis) / 2;
    y0[6] = (yAdjt + yAxis) / 2;          y1[6] = (yAdjt + yAxis) / 2 * -1;

    


//    y0[1] = yAdjdb * -1/2;                y1[1] = (yAxis + yAdjdb) * -1 /2;
//    y0[2] = (yAxis + yAdjt) / 2 * -1;     y1[2] = (yAxis + yAdjt) / 2;
//    y0[3] = yAxis + yAdjdb /2;               y1[3] = yAdjdb /2;
//
//    y0[4] = yAxis + yAdjdb /2;               y1[4] = yAdjdb /2; //DR
//    y0[5] = (yAxis + yAdjt) / 2 * -1;     y1[5] = (yAxis + yAdjt) / 2;
//    y0[6] = yAdjdb * -1 /2;                  y1[6] = (yAxis + yAdjdb) * -1 /2; ////BR

    for (int a = 1; a <= 6; a++) {
      x0[a] = xAxis;
      x1[a] = xAxis;
      z0[a] = zAxis;
      z1[a] = zAxis;
    }
  }

  if (d == 2) {// muter
    yAdjdb = 0;
    yAdjt = 0;//yAdjdb * 1;
    for (int a = 1; a <= 6; a++) {
      runAngle[a] = 0;
    }

    y0[1] = (yAdjt + yAxis) / 2;          y1[1] = (yAdjt + yAxis) / 2 * -1;
    y0[2] = (yAdjt + yAxis) / 2 * -1;     y1[2] = (yAdjt + yAxis) / 2;
    y0[3] = (yAdjt + yAxis) / 2;          y1[3] = (yAdjt + yAxis) / 2 * -1;

    y0[4] = (yAdjt + yAxis) / 2 * -1;     y1[4] = (yAdjt + yAxis) / 2;
    y0[5] = (yAdjt + yAxis) / 2;          y1[5] = (yAdjt + yAxis) / 2 * -1;
    y0[6] = (yAdjt + yAxis) / 2 * -1;     y1[6] = (yAdjt + yAxis) / 2;

    for (int a = 1; a <= 6; a++) {
      x0[a] = xAxis;
      x1[a] = xAxis;
      z0[a] = zAxis;
      z1[a] = zAxis;
    }
  }

  if (d == 3) { //geser kiri
//    yAdjdb = 60;
//    yAdjt = 5;
//    runAngle[1] = -43; //45
//    runAngle[2] = 30;
//    runAngle[3] = 0;
//    runAngle[4] = 0;
//    runAngle[5] = 30;
//    runAngle[6] = -55;
    yAdjdb = 50;
    yAdjt = 0;
    runAngle[1] = -15;
    runAngle[2] = 15;
    runAngle[3] = 0;
    runAngle[4] = 0;
    runAngle[5] = 15;
    runAngle[6] = -15;

    for (int a = 1; a <= 6; a++) {
      //x0[a] = xAxis;
      //x1[a] = xAxis;
      y0[a] = 0;//yAxis;
      y1[a] = 0;//yAxis;
      z0[a] = zAxis;
      z1[a] = zAxis;
    }

    x0[1] = xAxis;          x1[1] = xAxis + 15;
    x0[2] = xAxis + 15;     x1[2] = xAxis;
    x0[3] = xAxis;          x1[3] = xAxis + 15;

    x0[4] = xAxis;          x1[4] = xAxis + 15; //DR
    x0[5] = xAxis + 15;     x1[5] = xAxis;
    x0[6] = xAxis;          x1[6] = xAxis + 15; ////BR

    //y0[1] = yAdjdb * -1;                  y1[1] = (yAxis + yAdjdb) * -1;
    //y0[2] = (yAxis + yAdjt) / 2 * -1;     y1[2] = (yAxis + yAdjt) / 2;
    //y0[3] = yAxis + yAdjdb;               y1[3] = yAdjdb + 0;

    //y0[4] = yAxis + yAdjdb;               y1[4] = yAdjdb + 0; //DR
    //y0[5] = (yAxis + yAdjt) / 2 * -1;     y1[5] = (yAxis + yAdjt) / 2;
    //y0[6] = yAdjdb * -1;                  y1[6] = (yAxis + yAdjdb) * -1; ////BR


    Serial.println("Geser <<");
  }

  if (d == 4) { //geser kanan
    yAdjdb = 50;
    yAdjt = 0;
    runAngle[1] = -10;
    runAngle[2] = 10;
    runAngle[3] = 0;
    runAngle[4] = 0;
    runAngle[5] = 10;
    runAngle[6] = -10;

//    yAdjdb = 60;
//    yAdjt = 5;
//    runAngle[1] = -43; //45
//    runAngle[2] = 30;
//    runAngle[3] = 0;
//    runAngle[4] = 0;
//    runAngle[5] = 30;
//    runAngle[6] = -55;

    for (int a = 1; a <= 6; a++) {
      //x0[a] = xAxis;
      //x1[a] = xAxis;
      y0[a] = 0;//yAxis;
      y1[a] = 0;//yAxis;
      z0[a] = zAxis;
      z1[a] = zAxis;
    }
    //default = 10
    x0[1] = xAxis + 15;          x1[1] = xAxis;
    x0[2] = xAxis;               x1[2] = xAxis + 15;
    x0[3] = xAxis + 15;          x1[3] = xAxis;

    x0[4] = xAxis + 15;          x1[4] = xAxis; //DR
    x0[5] = xAxis;               x1[5] = xAxis + 15;
    x0[6] = xAxis + 15;          x1[6] = xAxis; ////BR

    //y0[1] = yAdjdb * -1;                  y1[1] = (yAxis + yAdjdb) * -1;
    //y0[2] = (yAxis + yAdjt) / 2 * -1;     y1[2] = (yAxis + yAdjt) / 2;
    //y0[3] = yAxis + yAdjdb;               y1[3] = yAdjdb + 0;

    //y0[4] = yAxis + yAdjdb;               y1[4] = yAdjdb + 0; //DR
    //y0[5] = (yAxis + yAdjt) / 2 * -1;     y1[5] = (yAxis + yAdjt) / 2;
    //y0[6] = yAdjdb * -1;                  y1[6] = (yAxis + yAdjdb) * -1; ////BR


    Serial.println("Geser >>");
  }
  
  if (d == 5) {
    yAdjdb = 30;//30
    yAdjt = 0;//yAdjdb * 1;
    runAngle[1] = -60;//-60;
    runAngle[2] = 60;//60;
    runAngle[3] = 0;
    runAngle[4] = 0;
    runAngle[5] = 60;//60;
    runAngle[6] = -60;//-60;

    y0[1] = yAdjdb * -1;                  y1[1] = (yAxis + yAdjdb) * -1;
    y0[2] = (yAxis + yAdjt) / 2 * -1;     y1[2] = (yAxis + yAdjt) / 2;
    y0[3] = yAxis + yAdjdb;               y1[3] = yAdjdb + 0;

    y0[4] = yAxis + yAdjdb;               y1[4] = yAdjdb + 0; //DR
    y0[5] = (yAxis + yAdjt) / 2 * -1;     y1[5] = (yAxis + yAdjt) / 2;
    y0[6] = yAdjdb * -1;                  y1[6] = (yAxis + yAdjdb) * -1; ////BR

    z0[1] = 10;     z1[1] = 0;
    z0[2] = 10;     z1[2] = 0;
    z0[3] = 10;     z1[3] = 0;

    z0[4] = 10;     z1[4] = 0; //DR
    z0[5] = 10;     z1[5] = 0;
    z0[6] = 10;     z1[6] = 0; //BR

    for (int a = 1; a <= 6; a++) {
      x0[a] = xAxis;
      x1[a] = xAxis;
      //y0[a] = yAxis;
      //y1[a] = yAxis;
      //z0[a] = zAxis;
      //z1[a] = zAxis;
    } 
  }

    if (d == 6) { //geser kiri
//    yAdjdb = 60;
//    yAdjt = 5;
//    runAngle[1] = -43; //45
//    runAngle[2] = 30;
//    runAngle[3] = 0;
//    runAngle[4] = 0;
//    runAngle[5] = 30;
//    runAngle[6] = -55;
    yAdjdb = 60;
    yAdjt = 0;
    runAngle[1] = -30;
    runAngle[2] = 30;
    runAngle[3] = 0;
    runAngle[4] = 0;
    runAngle[5] = 30;
    runAngle[6] = -30;

    for (int a = 1; a <= 6; a++) {
      //x0[a] = xAxis;
      //x1[a] = xAxis;
      y0[a] = 0;//yAxis;
      y1[a] = 0;//yAxis;
      z0[a] = zAxis;
      z1[a] = zAxis;
    }

    x0[1] = xAxis;          x1[1] = xAxis + 15;
    x0[2] = xAxis + 15;     x1[2] = xAxis;
    x0[3] = xAxis;          x1[3] = xAxis + 15;

    x0[4] = xAxis;          x1[4] = xAxis + 15; //DR
    x0[5] = xAxis + 15;     x1[5] = xAxis;
    x0[6] = xAxis;          x1[6] = xAxis + 15; ////BR

    //y0[1] = yAdjdb * -1;                  y1[1] = (yAxis + yAdjdb) * -1;
    //y0[2] = (yAxis + yAdjt) / 2 * -1;     y1[2] = (yAxis + yAdjt) / 2;
    //y0[3] = yAxis + yAdjdb;               y1[3] = yAdjdb + 0;

    //y0[4] = yAxis + yAdjdb;               y1[4] = yAdjdb + 0; //DR
    //y0[5] = (yAxis + yAdjt) / 2 * -1;     y1[5] = (yAxis + yAdjt) / 2;
    //y0[6] = yAdjdb * -1;                  y1[6] = (yAxis + yAdjdb) * -1; ////BR


    Serial.println("Geser <<");
  }

  if (d == 7) { //geser kanan
    yAdjdb = 60;
    yAdjt = 0;
    runAngle[1] = -30;
    runAngle[2] = 30;
    runAngle[3] = 0;
    runAngle[4] = 0;
    runAngle[5] = 30;
    runAngle[6] = -30;

//    yAdjdb = 60;
//    yAdjt = 5;
//    runAngle[1] = -43; //45
//    runAngle[2] = 30;
//    runAngle[3] = 0;
//    runAngle[4] = 0;
//    runAngle[5] = 30;
//    runAngle[6] = -55;

    for (int a = 1; a <= 6; a++) {
      //x0[a] = xAxis;
      //x1[a] = xAxis;
      y0[a] = 0;//yAxis;
      y1[a] = 0;//yAxis;
      z0[a] = zAxis;
      z1[a] = zAxis;
    }
    //default = 10
    x0[1] = xAxis + 15;          x1[1] = xAxis;
    x0[2] = xAxis;               x1[2] = xAxis + 15;
    x0[3] = xAxis + 15;          x1[3] = xAxis;

    x0[4] = xAxis + 15;          x1[4] = xAxis; //DR
    x0[5] = xAxis;               x1[5] = xAxis + 15;
    x0[6] = xAxis + 15;          x1[6] = xAxis; ////BR

    //y0[1] = yAdjdb * -1;                  y1[1] = (yAxis + yAdjdb) * -1;
    //y0[2] = (yAxis + yAdjt) / 2 * -1;     y1[2] = (yAxis + yAdjt) / 2;
    //y0[3] = yAxis + yAdjdb;               y1[3] = yAdjdb + 0;

    //y0[4] = yAxis + yAdjdb;               y1[4] = yAdjdb + 0; //DR
    //y0[5] = (yAxis + yAdjt) / 2 * -1;     y1[5] = (yAxis + yAdjt) / 2;
    //y0[6] = yAdjdb * -1;                  y1[6] = (yAxis + yAdjdb) * -1; ////BR


    Serial.println("Geser >>");
  }

  if (d == 10) { // majunaik
    yAdjdb = 45;
    yAdjt = 10;
    runAngle[1] = -45;
    runAngle[2] = 45;
    runAngle[3] = 0;
    runAngle[4] = 0;
    runAngle[5] = 45;
    runAngle[6] = -45;

    y0[1] = yAdjdb * -1;                  y1[1] = (yAxis + yAdjdb) * -1;
    y0[2] = (yAxis + yAdjt) / 2 * -1;     y1[2] = (yAxis + yAdjt) / 2;
    y0[3] = yAxis + yAdjdb;               y1[3] = yAdjdb + 0;

    y0[4] = yAxis + yAdjdb;               y1[4] = yAdjdb + 0; //DR
    y0[5] = (yAxis + yAdjt) / 2 * -1;     y1[5] = (yAxis + yAdjt) / 2;
    y0[6] = yAdjdb * -1;                  y1[6] = (yAxis + yAdjdb) * -1; ////BR

    for (int a = 1; a <= 6; a++) {
      x0[a] = xAxis;
      x1[a] = xAxis;
      z0[a] = zAxis;
      z1[a] = zAxis;
    }
  }

  if (d == 11) { // majunaik // 
    yAdjdb = 45;
    yAdjt = 10;
    runAngle[1] = -30;
    runAngle[2] = 30;
    runAngle[3] = 0;
    runAngle[4] = 0;
    runAngle[5] = 45;
    runAngle[6] = -45;

    y0[1] = yAdjdb * -1;                  y1[1] = (yAxis + yAdjdb) * -1;
    y0[2] = (yAxis + yAdjt) / 2 * -1;     y1[2] = (yAxis + yAdjt) / 2;
    y0[3] = yAxis + yAdjdb;               y1[3] = yAdjdb + 0;

    y0[4] = yAxis + yAdjdb;               y1[4] = yAdjdb + 0; //DR
    y0[5] = (yAxis + yAdjt) / 2 * -1;     y1[5] = (yAxis + yAdjt) / 2;
    y0[6] = yAdjdb * -1;                  y1[6] = (yAxis + yAdjdb) * -1; ////BR

    for (int a = 1; a <= 6; a++) {
      x0[a] = xAxis;
      x1[a] = xAxis;
      z0[a] = zAxis;
      z1[a] = zAxis;
    }
  }


    if (d == 12) { // majunaik
    yAdjdb = 30;
    yAdjt = 0;
    runAngle[1] = -45;
    runAngle[2] = 45;
    runAngle[3] = 0;
    runAngle[4] = 0;
    runAngle[5] = 60;
    runAngle[6] = -60;

    y0[1] = yAdjdb * -1;                  y1[1] = (yAxis + yAdjdb) * -1;
    y0[2] = (yAxis + yAdjt) / 2 * -1;     y1[2] = (yAxis + yAdjt) / 2;
    y0[3] = yAxis + yAdjdb;               y1[3] = yAdjdb + 0;

    y0[4] = yAxis + yAdjdb;               y1[4] = yAdjdb + 0; //DR
    y0[5] = (yAxis + yAdjt) / 2 * -1;     y1[5] = (yAxis + yAdjt) / 2;
    y0[6] = yAdjdb * -1;                  y1[6] = (yAxis + yAdjdb) * -1; ////BR

    for (int a = 1; a <= 6; a++) {
      x0[a] = xAxis;
      x1[a] = xAxis;
      z0[a] = zAxis;
      z1[a] = zAxis;
    }

    x0[1] = xAxis;                        x1[1] = 50;
    x0[6] = xAxis;                        x1[6] = 50;
    x0[2] = xAxis;                        x1[2] = 50;
    x0[5] = xAxis;                        x1[5] = 50;

    z0[2] = 0;                        z1[2] = 50;
    z1[5] = 0;                        z1[5] = 50;
  }
  
  if (d == 15) { // maju naik rintangan 2
    yAdjdb = 30;
    yAdjt = 0;
    runAngle[1] = -45;
    runAngle[2] = 45;
    runAngle[3] = 0;
    runAngle[4] = 0;
    runAngle[5] = 60;
    runAngle[6] = -60;

    y0[1] = yAdjdb * -1;                  y1[1] = (yAxis + yAdjdb) * -1;
    y0[2] = (yAxis + yAdjt) / 2 * -1;     y1[2] = (yAxis + yAdjt) / 2;
    y0[3] = yAxis + yAdjdb;               y1[3] = yAdjdb + 0;

    y0[4] = yAxis + yAdjdb;               y1[4] = yAdjdb + 0; //DR
    y0[5] = (yAxis + yAdjt) / 2 * -1;     y1[5] = (yAxis + yAdjt) / 2;
    y0[6] = yAdjdb * -1;                  y1[6] = (yAxis + yAdjdb) * -1; ////BR
    
    for (int a = 1; a <= 6; a++) {
      x0[a] = xAxis + 15;
      x1[a] = xAxis + 15;
      z0[a] = zAxis;
      z1[a] = zAxis;
    }
  }

  

  for (int a = 1; a <= JS; a++) {
    xPs[a] = (x0[a] - x1[a]) / (JS - 1);
    yPs[a] = (y0[a] - y1[a]) / (JS - 1);
    zPs[a] = (z0[a] - z1[a]) / (JS - 1);
  }

  Serial.println("GO_FR");
  Serial.println();
  int invers = 1;
  for (int p = 1; p <= P36; p++) {
    for (int s = 1; s <= JS; s++) {
      kinematics(x0[p] - xPs[p] * (s - 1), y0[p] - yPs[p] * (s - 1), zAxis, s, p);
    }
  }
  Serial.print("Calculation Time : ");
  Serial.print(millis() - y);
  Serial.println(" ms");
}

int alphaAdj = 0;//10;
void kinematics(float x, float y, float zet, int s, int pos) {
  float  z = Zo + zet;
  if (P36 == 6) {
    if (pos <= 3) y = y * right * rightCalibration; else y = y * left * leftCalibration;
  }
  //if (pos % 2 == 0) y = y * right; else y = y * left;
  float L1 = sqrt(sq(x) + sq(y));
  float GAMA = (atan(x / y) / PI * 180);
  if (GAMA > 0) GAMA = GAMA - 90;
  else GAMA = GAMA + 90;//Penyesuaian sudut awal
  float L2 = sqrt(sq(L1 - cx) + sq(z));
  float BETA = acos((sq(tb) + sq(fm) - sq(L2)) / (2 * tb * fm)) / PI * 180;
  float alpha1 = acos(z / L2) / PI * 180;
  float alpha2 = acos((sq(fm) + sq(L2) - sq(tb)) / (2 * fm * L2)) / PI * 180;
  float ALPHA = alpha1 + alpha2;

  if (pos == 1) {
    AdL[s] = cxAngle + runAngle[1] + GAMA + calA[1];
    BdL[s] = fmAngle - fm0 + 180 + alphaAdj - ALPHA + calB[1];
    CdL[s] = tbAngle - tb0 + BETA + calC[1];

    if (P36 == 3) {
      AbR[s] = cxAngle + runAngle[6] + GAMA + calA[6];
      BbR[s] = fmAngle - fm0 + 180 + alphaAdj -  ALPHA + calB[6];
      CbR[s] = tbAngle - tb0 + BETA + calC[6];
    }
    if ((printOut == 1) and (pos == 1)) {
      printSerial("AdL", "BdL", "CdL", AdL[s], BdL[s], CdL[s], s);
      if (P36 == 3) {
        printSerial("AbR", "BbR", "CbR", AbR[s], BbR[s], CbR[s], s);
      }
    }
  }

  if (pos == 2) {
    AtL[s] = cxAngle + runAngle[3] + GAMA + calA[3];
    BtL[s] = fmAngle - fm0 + 180 + alphaAdj - ALPHA + calB[3];
    CtL[s] = tbAngle - tb0 + BETA + calC[3];

    if (P36 == 3) {
      AtR[s] = cxAngle + runAngle[4] + GAMA + calA[4];
      BtR[s] = fmAngle - fm0 + 180 + alphaAdj - ALPHA + calB[4];
      CtR[s] = tbAngle - tb0 + BETA + calC[4];
    }

    if ((printOut == 1) and (pos == 2)) {
      printSerial("AtL", "BtL", "CtL", AtL[s], BtL[s], CtL[s], s);
      if (P36 == 3) {
        printSerial("AtR", "BtR", "CtR", AtR[s], BtR[s], CtR[s], s);
      }
    }
  }

  if (pos == 3) {
    AbL[s] = cxAngle + runAngle[5] + GAMA + calA[5];
    BbL[s] = fmAngle - fm0 + 180 + alphaAdj - ALPHA + calB[5];
    CbL[s] = tbAngle - tb0 + BETA + calC[5];

    if (P36 == 3) {
      AdR[s] = cxAngle + runAngle[2] + GAMA + calA[2];
      BdR[s] = fmAngle - fm0 + 180 + alphaAdj - ALPHA + calB[2];
      CdR[s] = tbAngle - tb0 + BETA + calC[2];
    }

    if ((printOut == 1) and (pos == 3)) {
      printSerial("AbL", "BbL", "CbL", AbL[s], BbL[s], CbL[s], s);
      if (P36 == 3) {
        printSerial("AdR", "BdR", "CdR", AdR[s], BdR[s], CdR[s], s);
      }
    }
  }

  if (pos == 4) {
    AdR[s] = cxAngle + runAngle[2] + GAMA + calA[2];
    BdR[s] = fmAngle - fm0 + 180 + alphaAdj - ALPHA + calB[2];
    CdR[s] = tbAngle - tb0 + BETA + calC[2];
    if ((printOut == 1) and (pos == 4)) {
      printSerial("AdR", "BdR", "CdR", AdR[s], BdR[s], CdR[s], s);
    }
  }

  if (pos == 5) {
    AtR[s] = cxAngle + runAngle[4] + GAMA + calA[4];
    BtR[s] = fmAngle - fm0 + 180 + alphaAdj - ALPHA + calB[4];
    CtR[s] = tbAngle - tb0 + BETA + calC[4];
    if ((printOut == 1) and (pos == 5)) {
      printSerial("AtR", "BtR", "CtR", AtR[s], BtR[s], CtR[s], s);
    }
  }

  if (pos == 6) {
    AbR[s] = cxAngle + runAngle[6] + GAMA + calA[6];
    BbR[s] = fmAngle - fm0 + 180 + alphaAdj -  ALPHA + calB[6];
    CbR[s] = tbAngle - tb0 + BETA + calC[6];
    if ((printOut == 1) and (pos == 6)) {
      printSerial("AbR", "BbR", "CbR", AbR[s], BbR[s], CbR[s], s);
    }
  }

  Adl1 = AdL[1] - calA[1];
  if (hitungJalan == 1) {
    moveTo(s);
  }
}

void gaitsTrajectoryCalculation(byte sm) {
  if (sm <= 1) JS = 6;
  if (sm == 2) JS = 12;
  if (sm == 3) JS = 18;
  if (sm >= 4) JS = 24;

  Serial.print("Gait Calculation "); Serial.println("JS " + String(JS));
  int a;
  byte ax = JS + JS - 1;
  byte ax0 = 4 * JS - 1;
  float jsx = JS;
  byte ax2, ax21;// = ax - a;

  for (a = 0; a < ax0; a++) {//s1
    if (a <= JS) s0[a] = a;
    else if (a < 2 * JS ) s0[a] = 2 * JS - a;
    else s0[a] = a - 2 * JS + 2;
    if (a > 3 * JS - 2) s0[a] = s0[a] = (JS - 2) - (a - 3 * JS)  ;
    //Serial.print("S0 " + String(a) + " : ");   Serial.print("\t");  Serial.print(s0[a]); Serial.println("\t");//Serial.print(" \t");
  }
  for (a = 0; a < ax; a++) {
    //=========Gait = 3
    s1[a] = s0[a]; s2[a] = s0[a]; s3[a] = s0[a]; s4[a] = s0[a]; s5[a] = s0[a]; s6[a] = s0[a];

    //=========Trajectory
    ax2 = ax - a - 1;
    ax21 = a + 2;
    U1[a] = ((a - 1) - (1 / (jsx - 1) * (a - 1) * (a - 1))) * (1 / (0.2513 * (jsx - 1) - 0.0288)) * lift;
    W1[a] = ((a - 1) - (1 / (jsx - 1) * (a - 1) * (a - 1))) * (1 / (0.2513 * (jsx - 1) - 0.0288)) * wide;
    U2[ax2] = ((ax21 - 1) - (1 / (jsx - 1) * (ax21 - 1) * (ax21 - 1))) * (1 / (0.2513 * (jsx - 1) - 0.0288)) * lift;
    W2[ax2] = ((ax21 - 1) - (1 / (jsx - 1) * (ax21 - 1) * (ax21 - 1))) * (1 / (0.2513 * (jsx - 1) - 0.0288)) * wide;;
    if (U1[a] <= 0)U1[a] = 0; if (U2[ax2] <= 0)U2[ax2] = 0; if (W1[a] <= 0)W1[a] = 0; if (W2[ax2] <= 0)W2[ax2] = 0;

    if (printOut == 2) {
      Serial.print("S1 " + String(a) + ":");   Serial.print("\t");  Serial.print(s1[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("S2 " + String(a) + ":");   Serial.print("\t");  Serial.print(s2[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("S3 " + String(a) + ":");   Serial.print("\t");  Serial.print(s3[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("S4 " + String(a) + ":");   Serial.print("\t");  Serial.print(s4[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("S5 " + String(a) + ":");   Serial.print("\t");  Serial.print(s5[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("S6 " + String(a) + ":");   Serial.print("\t");  Serial.println(s6[a]); //Serial.println("\t");//Serial.print(" \t");
    }
    if (printOut == 3) {
      Serial.print("U1 " + String(a) + ":");   Serial.print("\t");  Serial.print(U1[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("U2 " + String(ax2) + ":");   Serial.print("\t");  Serial.print(U2[ax2]); Serial.print(" \t");
      Serial.print("W1 " + String(a) + ":");   Serial.print("\t");  Serial.print(W1[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("W2 " + String(ax2) + ":");   Serial.print("\t");  Serial.println(W2[ax2]); //Serial.print(" \t");
    }
  }
}

void runStep( int  a1, int  a2, int a3, int a4, int a5, int a6, int  b1, int  b2, int b3, int b4, int b5, int b6,
              int  c1, int  c2, int c3, int c4, int c5, int c6, int sp1) {

  while ((pos_1A != a1) || (pos_2A != a2) || (pos_3A != a3) || (pos_4A != a4) || (pos_5A != a5) || (pos_6A != a6) ||
         (pos_1B != b1) || (pos_2B != b2) || (pos_3B != b3) || (pos_4B != b4) || (pos_5B != b5) || (pos_6B != b6) ||
         (pos_1C != c1) || (pos_2C != c2) || (pos_3C != c3) || (pos_4C != c4) || (pos_5C != c5) || (pos_6C != c6) ) {

    //=================1A
    if (pos_1A < a1) {
      if ((pos_1A + sp1) <= a1) pos_1A = pos_1A + sp1;
      else pos_1A = a1;
    }
    if (pos_1A > a1) {
      if ((pos_1A - sp1) >= a1) pos_1A = pos_1A - sp1;
      else pos_1A = a1;
    }
    //======================1B
    if (pos_1B < b1) {
      if ((pos_1B + sp1) <= b1) pos_1B = pos_1B + sp1;
      else pos_1B = b1;
    }
    if (pos_1B > b1) {
      if ((pos_1B - sp1) >= b1) pos_1B = pos_1B - sp1;
      else pos_1B = b1;
    }
    //======================1C
    if (pos_1C < c1) {
      if ((pos_1C + sp1) <= c1) pos_1C = pos_1C + sp1;
      else pos_1C = c1;
    }
    if (pos_1C > c1) {
      if ((pos_1C - sp1) >= c1) pos_1C = pos_1C - sp1;
      else pos_1C = c1;
    }
    //=================2A
    if (pos_2A < a2) {
      if ((pos_2A + sp1) <= a2) pos_2A = pos_2A + sp1;
      else pos_2A = a2;
    }
    if (pos_2A > a2) {
      if ((pos_2A - sp1) >= a2) pos_2A = pos_2A - sp1;
      else pos_2A = a2;
    }
    //======================2B
    if (pos_2B < b2) {
      if ((pos_2B + sp1) <= b2) pos_2B = pos_2B + sp1;
      else pos_2B = b2;
    }
    if (pos_2B > b2) {
      if ((pos_2B - sp1) >= b2) pos_2B = pos_2B - sp1;
      else pos_2B = b2;
    }
    //servoR.writeMicroseconds(srv_2B, 600 + 10 * pos_2B);

    //======================2C
    if (pos_2C < c2) {
      if ((pos_2C + sp1) <= c2) pos_2C = pos_2C + sp1;
      else pos_2C = c2;
    }
    if (pos_2C > c2) {
      if ((pos_2C - sp1) >= c2) pos_2C = pos_2C - sp1;
      else pos_2C = c2;
    }
    //servoR.writeMicroseconds(srv_2C, 600 + 10 * pos_2C);
    //=================3A
    if (pos_3A < a3) {
      if ((pos_3A + sp1) <= a3) pos_3A = pos_3A + sp1;
      else pos_3A = a3;
    }
    if (pos_3A > a3) {
      if ((pos_3A - sp1) >= a3) pos_3A = pos_3A - sp1;
      else pos_3A = a3;
    }
    //======================3B
    if (pos_3B < b3) {
      if ((pos_3B + sp1) <= b3) pos_3B = pos_3B + sp1;
      else pos_3B = b3;
    }
    if (pos_3B > b3) {
      if ((pos_3B - sp1) >= b3) pos_3B = pos_3B - sp1;
      else pos_3B = b3;
    }
    //======================3C
    if (pos_3C < c3) {
      if ((pos_3C + sp1) <= c3) pos_3C = pos_3C + sp1;
      else pos_3C = c3;
    }
    if (pos_3C > c3) {
      if ((pos_3C - sp1) >= c3) pos_3C = pos_3C - sp1;
      else pos_3C = c3;
    }
    //=================4A
    if (pos_4A < a4) {
      if ((pos_4A + sp1) <= a4) pos_4A = pos_4A + sp1;
      else pos_4A = a4;
    }
    if (pos_4A > a4) {
      if ((pos_4A - sp1) >= a4) pos_4A = pos_4A - sp1;
      else pos_4A = a4;
    }
    //======================4B
    if (pos_4B < b4) {
      if ((pos_4B + sp1) <= b4) pos_4B = pos_4B + sp1;
      else pos_4B = b4;
    }
    if (pos_4B > b4) {
      if ((pos_4B - sp1) >= b4) pos_4B = pos_4B - sp1;
      else pos_4B = b4;
    }
    //======================4C
    if (pos_4C < c4) {
      if ((pos_4C + sp1) <= c4) pos_4C = pos_4C + sp1;
      else pos_4C = c4;
    }
    if (pos_4C > c4) {
      if ((pos_4C - sp1) >= c4) pos_4C = pos_4C - sp1;
      else pos_4C = c4;
    }
    //=================5A
    if (pos_5A < a5) {
      if ((pos_5A + sp1) <= a5) pos_5A = pos_5A + sp1;
      else pos_5A = a5;
    }
    if (pos_5A > a5) {
      if ((pos_5A - sp1) >= a5) pos_5A = pos_5A - sp1;
      else pos_5A = a5;
    }
    //======================5B
    if (pos_5B < b5) {
      if ((pos_5B + sp1) <= b5) pos_5B = pos_5B + sp1;
      else pos_5B = b5;
    }
    if (pos_5B > b5) {
      if ((pos_5B - sp1) >= b5) pos_5B = pos_5B - sp1;
      else pos_5B = b5;
    }
    //======================5C
    if (pos_5C < c5) {
      if ((pos_5C + sp1) <= c5) pos_5C = pos_5C + sp1;
      else pos_5C = c5;
    }
    if (pos_5C > c5) {
      if ((pos_5C - sp1) >= c5) pos_5C = pos_5C - sp1;
      else pos_5C = c5;
    }
    //=================6A
    if (pos_6A < a6) {
      if ((pos_6A + sp1) <= a6) pos_6A = pos_6A + sp1;
      else pos_6A = a6;
    }
    if (pos_6A > a6) {
      if ((pos_6A - sp1) >= a6) pos_6A = pos_6A - sp1;
      else pos_6A = a6;
    }
    //======================6B
    if (pos_6B < b6) {
      if ((pos_6B + sp1) <= b6) pos_6B = pos_6B + sp1;
      else pos_6B = b6;
    }
    if (pos_6B > b6) {
      if ((pos_6B - sp1) >= b6) pos_6B = pos_6B - sp1;
      else pos_6B = b6;
    }
    //======================6C
    if (pos_6C < c6) {
      if ((pos_6C + sp1) <= c6) pos_6C = pos_6C + sp1;
      else pos_6C = c6;
    }
    if (pos_6C > c6) {
      if ((pos_6C - sp1) >= c6) pos_6C = pos_6C - sp1;
      else pos_6C = c6;
    }
    servoRun();
    delay(2);
  }
}

void gaitsAngkat(byte sm, float lebar, float angkat) {
  wide = lebar;
  lift = angkat;
  if (sm <= 1) JS = 6;
  if (sm == 2) JS = 12;
  if (sm == 3) JS = 18;
  if (sm >= 4) JS = 24;

  Serial.print("Gait Calculation "); Serial.println("JS " + String(JS));
  int a;
  byte ax = JS + JS - 1;
  byte ax0 = 4 * JS - 1;
  float jsx = JS;
  byte ax2, ax21;// = ax - a;

  for (a = 0; a < ax0; a++) {//s1
    if (a <= JS) s0[a] = a;
    else if (a < 2 * JS ) s0[a] = 2 * JS - a;
    else s0[a] = a - 2 * JS + 2;
    if (a > 3 * JS - 2) s0[a] = s0[a] = (JS - 2) - (a - 3 * JS)  ;
    //Serial.print("S0 " + String(a) + " : ");   Serial.print("\t");  Serial.print(s0[a]); Serial.println("\t");//Serial.print(" \t");
  }
  for (a = 0; a < ax; a++) {
    //=========Gait = 3
    s1[a] = s0[a]; s2[a] = s0[a]; s3[a] = s0[a]; s4[a] = s0[a]; s5[a] = s0[a]; s6[a] = s0[a];

    //=========Trajectory
    ax2 = ax - a - 1;
    ax21 = a + 2;
    U1[a] = ((a - 1) - (1 / (jsx - 1) * (a - 1) * (a - 1))) * (1 / (0.2513 * (jsx - 1) - 0.0288)) * lift;
    W1[a] = ((a - 1) - (1 / (jsx - 1) * (a - 1) * (a - 1))) * (1 / (0.2513 * (jsx - 1) - 0.0288)) * wide;
    U2[ax2] = ((ax21 - 1) - (1 / (jsx - 1) * (ax21 - 1) * (ax21 - 1))) * (1 / (0.2513 * (jsx - 1) - 0.0288)) * lift;
    W2[ax2] = ((ax21 - 1) - (1 / (jsx - 1) * (ax21 - 1) * (ax21 - 1))) * (1 / (0.2513 * (jsx - 1) - 0.0288)) * wide;;
    if (U1[a] <= 0)U1[a] = 0; if (U2[ax2] <= 0)U2[ax2] = 0; if (W1[a] <= 0)W1[a] = 0; if (W2[ax2] <= 0)W2[ax2] = 0;

    if (printOut == 2) {
      Serial.print("S1 " + String(a) + ":");   Serial.print("\t");  Serial.print(s1[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("S2 " + String(a) + ":");   Serial.print("\t");  Serial.print(s2[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("S3 " + String(a) + ":");   Serial.print("\t");  Serial.print(s3[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("S4 " + String(a) + ":");   Serial.print("\t");  Serial.print(s4[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("S5 " + String(a) + ":");   Serial.print("\t");  Serial.print(s5[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("S6 " + String(a) + ":");   Serial.print("\t");  Serial.println(s6[a]); //Serial.println("\t");//Serial.print(" \t");
    }
    if (printOut == 3) {
      Serial.print("U1 " + String(a) + ":");   Serial.print("\t");  Serial.print(U1[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("U2 " + String(ax2) + ":");   Serial.print("\t");  Serial.print(U2[ax2]); Serial.print(" \t");
      Serial.print("W1 " + String(a) + ":");   Serial.print("\t");  Serial.print(W1[a]); Serial.print("\t");//Serial.print(" \t");
      Serial.print("W2 " + String(ax2) + ":");   Serial.print("\t");  Serial.println(W2[ax2]); //Serial.print(" \t");
    }
  }
}
