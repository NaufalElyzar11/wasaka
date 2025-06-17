
int jangka_waktu1 = 10; // time interval in ms for updating panel indicators
unsigned long waktu_akhir1 = 0; // time of last update
int jangka_waktu2 = 1500; // time interval in ms for updating panel indicators
unsigned long waktu_akhir2 = 0; // time of last update
void moveTo(int a) {
  long y1 = millis();
  long y2 = millis();
  runStep
  (AdL[s1[a]], AdR[s2[a]], AtL[s3[a]], AtR[s4[a]], AbL[s5[a]], AbR[s6[a]],

   BdL[s1[a]] - U1[a], BdR[s2[a]] - U2[a],
   BtL[s3[a]] - U2[a], BtR[s4[a]] - U1[a],
   BbL[s5[a]] - U1[a], BbR[s6[a]] - U2[a],

   CdL[s1[a]] + W1[a], CdR[s2[a]] + W2[a],
   CtL[s3[a]] + W2[a], CtR[s4[a]] + W1[a],
   CbL[s5[a]] + W1[a], CbR[s6[a]] + W2[a], sp);
  delay(1);
  if (runSt == 1) numberStep += 1;
  if (runSt == 2) numberStep -= 1;
  runtStepNumber++;
  if (a == addStep) {//addStep
    jumlahLangkah++;
//      ukurJarak();
wasakaUltraSonic();

    if ((y2 - waktu_akhir2) > jangka_waktu2) {
      waktu_akhir2 = y2;
//      ukurWarna();
    }
    
//    Serial.print("jumlahLangkah, ");
//    Serial.println(jumlahLangkah);
  }
}

void action_1(int a) {
  //spd = 12;
  directionSt = 9;
  runStep
  (110, 70,     130,    45,     AbL[1], AbR[1],
   15, 15,      45,     45, BbL[1], BbR[1],
   180,  180,   45,     45, CbL[1], CbR[1], sp);
  runStep
  (70, 110,     130,    45,     AbL[1], AbR[1],
   15, 15,      45,     45, BbL[1], BbR[1],
   180, 180,    45,     45, CbL[1], CbR[1], sp);
}

void action_2(int a) {
  //spd = 12;
  directionSt = 9;
  runStep
  (AbL[1], AbR[1],     130,    45,     110, 30,
   BbL[1], BbR[1],      45,     45,    15,      15,
   CbL[1], CbR[1],    45,     45, 180,      180,  sp);
  runStep
  (AbL[1], AbR[1],     130,    45,     70, 110, 
    BbL[1], BbR[1],    45,     45,     15, 15,  
     CbL[1], CbR[1],  45,     45,      180, 180, sp);
}

void stby0(int a) {
  //spd = 12;
  runStep
  (AdL[1], AdR[1], AtL[1], AtR[1], AbL[1], AbR[1],
   BdL[1], BdR[1], BtL[1], BtR[1], BbL[1], BbR[1],
   CdL[1], CdR[1], CtL[1], CtR[1], CbL[1], CbR[1], sp);
  runStep
  (70, 110, 90, AtR[1], AbL[1], AbR[1],
   15, 15, BtL[1], BtR[1], BbL[1], BbR[1],
   180, 180, CtL[1], CtR[1], CbL[1], CbR[1], sp);
}

void geserKanan(byte x) {
  left = 1;
  right = 1;
  if (directionSt != 11) {
    moveToCalculation(3);
    directionSt = 11;
  }
  if (langkah < addStep) {
    langkah++;
    numberStep += 2;
  } else {
    langkah = 1;
  }
  if (x != 0) {
    if (jumlahLangkah < x) moveTo(langkah);
    else {
      //jumlahLangkah = 0;
    }
  } else moveTo(langkah);
}

void geserKananSpd(byte x, int sp) {
  left = 1;
  right = 1;
  spd = sp;
  if (directionSt != 11) {
    moveToCalculation(3);
    directionSt = 11;
  }
  if (langkah < addStep) {
    langkah++;
    numberStep += 2;
  } else {
    langkah = 1;
  }
  if (x != 0) {
    if (jumlahLangkah < x) moveTo(langkah);
    else {
      //jumlahLangkah = 0;
    }
  } else moveTo(langkah);
}

void geserKiri(byte x) {
  left = 1;
  right = 1;
  if (directionSt != 12) {
    moveToCalculation(4);
    directionSt = 12;
  }
  if (langkah < addStep) {
    langkah++;
    numberStep += 2;
  } else {
    langkah = 1;
  }
  if (x != 0) {
    if (jumlahLangkah < x) moveTo(langkah);
    else {
      //jumlahLangkah = 0;
    }
  } else moveTo(langkah);
}

void geserKiriSpd(byte x, int sp) {
  left = 1;
  right = 1;
  spd = sp;
  if (directionSt != 12) {
    moveToCalculation(4);
    directionSt = 12;
  }
  if (langkah < addStep) {
    langkah++;
    numberStep += 2;
  } else {
    langkah = 1;
  }
  if (x != 0) {
    if (jumlahLangkah < x) moveTo(langkah);
    else {
      //jumlahLangkah = 0;
    }
  } else moveTo(langkah);
}

void mundur(long x) {
  left = 1;
  right = 1;
  if (directionSt != 1) {
    moveToCalculation(1);
    directionSt = 1;
  }
  if (langkah < addStep) {
    langkah++;
    numberStep += 1;
  } else {
    langkah = 1;
  }

  if (x != 0) {
    if (jumlahLangkah < x) moveTo(langkah);
    else {
      jumlahLangkah = 0;
    }
  } else moveTo(langkah);

}

void mundurSpd(long x, int sp) {
  left = 1;
  right = 1;
  spd = sp;
  if (directionSt != 1) {
    moveToCalculation(1);
    directionSt = 1;
  }
  if (langkah < addStep) {
    langkah++;
    numberStep += 1;
  } else {
    langkah = 1;
  }

  if (x != 0) {
    if (jumlahLangkah < x) moveTo(langkah);
    else {
      jumlahLangkah = 0;
    }
  } else moveTo(langkah);

}

void mundurAction(long s, float x, float y, float z, int spid, int n) {
  left = 1; right = 1; yAxis = y; xAxis = x; zAxis = z;
  spd = spid;
  if (directionSt != 11) {
    moveToCalculation(1);
    directionSt = 11;
  }
  if (langkah < addStep) {
    langkah++;
    numberStep += 1;
  } else {
    langkah = 1;
  }

  if (s != 0) {
    if (jumlahLangkah < s) moveTo(langkah);
    else {
      jumlahLangkah = 0;
      runSt = n;
    }
  } else moveTo(langkah);

}

void majuAction(long s, float x, float y, float z, int spid, byte nextSt) {
  left = 1; right = 1; yAxis = y; xAxis = x; zAxis = z;
  spd = spid;
  if (directionSt != 22) {
    moveToCalculation(1);
    directionSt = 22;
  }
  if (langkah > 1) {
    numberStep -= 1;
    langkah--;
  } else {
    langkah = addStep;
  }
  if (s != 0) {
    if (jumlahLangkah < s) moveTo(langkah);
    else {
      jumlahLangkah = 0;
      runSt = nextSt;
    }
  }  else moveTo(langkah);
}

void mundur1(long s) {//mundur setelah menarok korban
  if (directionSt != 2) {
    moveToCalculation(1);
    directionSt = 2;
  }
  if (langkah > 1) {
    numberStep -= 1;
    langkah--;
  } else {
    langkah = addStep;
  }
  if (s != 0) {
    if (jumlahLangkah < s) moveTo(langkah);
    else {
      jumlahLangkah = 0;
//      gripperClose();
//      angkat();
//      runSt = 61;
    }
  }  else moveTo(langkah);
}

void maju(long s, int ke) {
  mtc = ke;
  if (directionSt != 2) {
    moveToCalculation(1);
    directionSt = 2;
  }
  if (langkah > 1) {
    numberStep -= 1;
    langkah--;
  } else {
    langkah = addStep;
  }
  if (s != 0) {
    if (jumlahLangkah < s) moveTo(langkah);
    else {
      jumlahLangkah = 0;
      runSt = 99;
    }
  }  else moveTo(langkah);
}

void majuSpd(long s, int sp) {
  spd = sp;
  if (directionSt != 2) {
    moveToCalculation(1);
    directionSt = 2;
  }
  if (langkah > 1) {
    numberStep -= 1;
    langkah--;
  } else {
    langkah = addStep;
  }
  if (s != 0) {
    if (jumlahLangkah < s) moveTo(langkah);
    else {
      jumlahLangkah = 0;
      runSt = 99;
    }
  }  else moveTo(langkah);
}

void majuJalan(long s, int sp) {
  spd = sp;
  if (directionSt != 2) {
    moveToCalculation(10);
    directionSt = 2;
  }
  if (langkah > 1) {
    numberStep -= 1;
    langkah--;
  } else {
    langkah = addStep;
  }
  if (s != 0) {
    if (jumlahLangkah < s) moveTo(langkah);
    else {
      jumlahLangkah = 0;
      runSt = 99;
    }
  }  else moveTo(langkah);
}

void majuAngkat(long s, float lebar, float angkat, int cepat, int ke){
  gaitsAngkat(2,lebar,angkat);
  mtc = ke;
  spd = cepat;
  if (directionSt != 2) {
    moveToCalculation(mtc);
    directionSt = 2;
  }
  if (langkah > 1) {
    numberStep -= 1;
    langkah--;
  } else {
    langkah = addStep;
  }
  if (s != 0) {
    if (jumlahLangkah < s) moveTo(langkah);
    else {
      jumlahLangkah = 0;
      runSt = 99;
    }
  }  else moveTo(langkah);
}


void putarKiriAction(long s, float x, float y, float z, float n, int next) {
  left = 1; right = 1; yAxis = y; xAxis = x; zAxis = z;
  spd = n;
  if (directionSt != 33) {
    moveToCalculation(2);
    directionSt = 33;
  }
  if (langkah > 1) {
    numberStep -= 1;
    langkah--;
  } else {
    langkah = addStep;
  }
  
  if (s != 0) {
    if (jumlahLangkah < s) {
      moveTo(langkah);
    } else {
      jumlahLangkah = 0;
      runSt = next;
      // frameRunning();
    }
  } else moveTo(langkah);
}

void putarKananAction(long s, float x, float y, float z, float n, int next) {
  left = 1; right = 1; yAxis = y; xAxis = x; zAxis = z;
  spd = n;
  if (directionSt != 44) {
    moveToCalculation(2);
    directionSt = 44;
  }
  if (langkah < addStep) {
    langkah++;
    numberStep += 1;
  } else {
    langkah = 1;
  }
  if (s != 0) {
    if (jumlahLangkah < s) {
      moveTo(langkah);
    } else {
      jumlahLangkah = 0;
      runSt = next;
//      // frameRunning();
    }
  } else moveTo(langkah);
}

void putarKanan(long s) {
  if (directionSt != 3) {
    moveToCalculation(2);
    directionSt = 3;
  }
  if (langkah < addStep) {
    langkah++;
    numberStep += 1;
  } else {
    langkah = 1;
  }
  if (s != 0) {
    if (jumlahLangkah < s) {
      moveTo(langkah);
    } else {

    }
  } else moveTo(langkah);
}

void putarKananSpd(long s, int sp) {
  spd = sp;
  if (directionSt != 3) {
    moveToCalculation(2);
    directionSt = 3;
  }
  if (langkah < addStep) {
    langkah++;
    numberStep += 1;
  } else {
    langkah = 1;
  }
  if (s != 0) {
    if (jumlahLangkah < s) {
      moveTo(langkah);
    } else {

    }
  } else moveTo(langkah);
}

void putarKiri(long s) {
  if (directionSt != 4) {
    moveToCalculation(2);
    directionSt = 4;
  }
  if (langkah > 1) {
    numberStep -= 1;
    langkah--;
  } else {
    langkah = addStep;
  }
  if (s != 0) {
    if (jumlahLangkah < s) moveTo(langkah);
    else {

    }
  } else moveTo(langkah);
}

void putarKiriSpd(long s, int sp) {
  spd = sp;
  if (directionSt != 4) {
    moveToCalculation(2);
    directionSt = 4;
  }
  if (langkah > 1) {
    numberStep -= 1;
    langkah--;
  } else {
    langkah = addStep;
  }
  if (s != 0) {
    if (jumlahLangkah < s) moveTo(langkah);
    else {

    }
  } else moveTo(langkah);
}

void belokKanan(float d) {
  left = d;
  right = 1;
  if (directionSt != 5) {
    moveToCalculation(1);
    directionSt = 5;
  }
  if (langkah < addStep) {
    langkah++;
    numberStep += 1;
  } else {
    langkah = 1;
  }
  moveTo(langkah);
}

void belokKiri(float d) {
  left = 1;
  right = d;
  if (directionSt != 6) {
    moveToCalculation(1);
    directionSt = 6;
  }
  if (langkah < addStep) {
    langkah++;
    numberStep += 1;
    moveTo(langkah);
  } else {
    langkah = 1;
  }
  moveTo(langkah);
}

void activeGriper() {
//  pinMode(kipasPin, OUTPUT);
//  pinMode(pompaPin, OUTPUT);
//  digitalWrite(pompaPin, HIGH);
//  pinMode(sApi1Pin, INPUT);
//  pinMode(sApi2Pin, INPUT);
//  pinMode(sApi3Pin, INPUT);
//  pinMode(sApi4Pin, INPUT);
//  pinMode(sApi5Pin, INPUT);
//  digitalWrite(kipasPin, HIGH);
  srv_GR.attach(g1, 600, 2400); srv_GR.write(posGp); delay(tunda);// Gripper 100 tutup, 30 buka
  srv_UP.attach(g2, 600, 2400); srv_UP.write(posUd); delay(tunda);// 0 UP  180 Down
}

// gripper 95 tutup, 30 buka

void gripperClose() {
  if (gripperSt == 0) {
    for (posGp = 100; posGp >= 25; posGp -= 1) {
      srv_GR.write(posGp);
      delay(5);
    }
    gripperSt = 1;
  }
}

void gripperClose1() {
  if (gripperSt == 0) {
    for (posGp = 40; posGp <= 70; posGp += 1) {
      srv_GR.write(posGp);
      delay(15);
    }
    gripperSt = 1;
  }
}

void gripperOpen() {
  if (gripperSt == 1) {
    for (posGp = 25; posGp <= 100; posGp += 1) {
      srv_GR.write(posGp);
      delay(5);
    }
    gripperSt = 0;
  }
}

void tarok() {
  if (UDSt == 0) {
    for (int i = 100; i >= 0; i -= 1) {
      posUd=0;
      srv_UP.write(i);
      delay(5);
    }
    UDSt = 1;
    gripperOpen();
//    delay(1000);
    done=true;
  }
}

void angkat() {
  gripperClose();
//  delay(1000);
  if (UDSt == 1) {
    for (int j=0; j <= 105; j += 1) {
      srv_UP.write(j);
      delay(5);
    }
    UDSt = 0;
//    jalan = 0;
  }
}

//void normalisasiKapit() {
//  srv_Capit.write(95);
//  srv_UP.write(90);
//  srv_GR.write(65);
//}

void pompaOn() {
//  digitalWrite(pompaPin, LOW);
}
void pompaOff() {
//  digitalWrite(pompaPin, HIGH);
}

void kipasOn() {
//  digitalWrite(kipasPin, LOW);
}
void kipasOff() {
//  digitalWrite(kipasPin, HIGH);
}

bool bacaButtonje() {
  buttonState = digitalRead(buttonPin);  
//  Serial.println(buttonState);
  if (buttonState == 0) {  
    // turn LED on:
    while(buttonState==0){ buttonState = digitalRead(buttonPin);
//    ukurJarak();
    Serial.println(buttonState);
    }
    statRobot++;
  }
  if(statRobot==1){
      Kondisi = true;
      statRobot=0;
      tombol = false;
      startRobot = true;
  }
//  if(jarak[2]>jarak[4]){
//      runSt = 2;
//  }
//  runSt = 20;
  return Kondisi;
}

bool bacaButton() {
  buttonState = digitalRead(buttonPin);  
//  Serial.println(buttonState);
  if (buttonState == 0) {  
    // turn LED on:
    while(buttonState==0){ buttonState = digitalRead(buttonPin);
//    ukurJarak();
//    Serial.println(buttonState);
    }
    statRobot++;
  }
  if(statRobot==1){
      Kondisi = true;
      statRobot=0;
      tombol = false;
      startRobot = true;
  }
  if(jarak[2]>jarak[4]){
      runSt = 2;
  }
//  runSt = 20;
  return Kondisi;
}

void bacaWarna() {
//  if(redColor > greenColor && redColor > blueColor){
//      Serial.println(" - RED detected!");
//  }
//  if(greenColor > redColor && greenColor > blueColor){
//    Serial.println(" - GREEN detected!");
//  }
  /*if(blueColor > redColor && blueColor > greenColor){
    Serial.println(" - BLUE detected!");
    //R= (-1450) - (-1950) G= (-2050) - (-2550) B= (-650) - (-1150) its black
    
    //R= (-100) - (-1000) /<(-1000) ?(-1400)  G= (-200 - (-2000) /< (-2000) ?(-2200)  B= (-30) - (800)
     
  }*/
  if((warna[0] < -1100 && warna[0] > -1950) && (warna[1] < -1700 && warna[1] > -2550) && (warna[2] < -500 && warna[2] > -1150)){
    Serial.println(" - Black detected!");
    hitam = 1;
    abu = 0;
    putih = 0;
  }
  if((warna[0] < -700 && warna[0] > -1100) && (warna[1] < -1000 && warna[1] > -1700) && (warna[2] < -200 && warna[2] > -750)){
    Serial.println(" - Grey detected!");
    hitam = 0;
    abu = 1;
    putih = 0;
  }
  else if((warna[0] < -100 && warna[0] > -1000) && (warna[1] < -200 && warna[1] > -2000) && (warna[2] < -30 && warna[2] > -800)){
    Serial.println(" - White detected!");
    hitam = 0;
    abu = 0;
    putih = 1;
  }
  /*
  if((redColor < -1100 && redColor > -1950) && (greenColor < -1700 && greenColor > -2550) && (blueColor < -500 && blueColor > -1150)){
    Serial.println(" - Black detected!");
    hitam = 1;
    abu = 0;
    putih = 0;
  }
  if((redColor < -700 && redColor > -1100) && (greenColor < -1000 && greenColor > -1700) && (blueColor < -200 && blueColor > -750)){
    Serial.println(" - Grey detected!");
    hitam = 0;
    abu = 1;
    putih = 0;
  }
  else if((redColor < -100 && redColor > -1000) && (greenColor < -200 && greenColor > -2000) && (blueColor < -30 && blueColor > -800)){
    Serial.println(" - White detected!");
    hitam = 0;
    abu = 0;
    putih = 1;
  }*/
//  Serial.println("");
}


void deteksiJarakCobaAja(){  
  wasakaUltraSonic();  
    while(distanceDepan < 20 && distanceKananDepan > 20 && perjalanan == false){
      
      while(jarak[3]>12){
        wasakaUltraSonic();
        putarKiri(0);
        Serial.println("belok 1");
        perjalanan = true;
        ikutKanan = true;
        if(jarak[3]<20){
          break;
        }
      }
      if(distanceKananDepan<20 || distanceKananBelakang<20){
          break;
      }
      
//        majuJalan(0,5);
    }
    
    if(distanceDepan<US1Sedang && jarak[2]>US3Jauh && jarak[4]<US5Sedang){
      putarKanan(0);
//        geserKanan(0);
    }

    else if(jarak[5]<US6Sedang){
      putarKanan(0);
//      geserKanan(0);
    }
    
    else if(jarak[1]<US2Sedang){
      putarKiri(0);
    }

    else if(jarak[2]>14 && jarak[2]<16 && ikutKanan == true){
      geserKanan(0);
//      puteranKanan++;
    }
    
    else if(jarak[4]<15){
      geserKanan(0);

    }

    else if(jarak[2]<12 && ikutKanan == false){
      geserKiri(0);
    }

    else if(jarak[0]<US1Dekat){
      mundur(0);
    }
    
    else{
      maju(0, 1);
      if(perjalanan == true && rintangan_satu == false){
        Serial.println("Rintangan 1");
//        majuAngkat(0,25,60,5,10); // kodingan sebelumnya
          majuAngkat(0,7,85,5,10); // yang baru
        rintangan_satu = true;
        perjalanan = false;
      }
      if(perjalanan == true && rintangan_satu == true){
        Serial.println("Rintangan 2");
//        majuAngkat(0,20,60,3,11); // kodingan sebelumnya
          majuAngkat(0,13,85,3,12); // yang baru
        perjalanan = false;
      }
    }    
}

void deteksiJarakFix(){
   
  wasakaUltraSonic();   
    while(distanceDepan < 20 && distanceKananDepan > 20 && perjalanan == false){
      wasakaUltraSonic(); 
      while(distanceKananBelakang > 12){
        putarKiri(8);
        Serial.println("belok 1");
        perjalanan = true;
        ikutKanan = true;
        if(distanceKananDepan < 20 && distanceKananBelakang < 20){
          break;
        }
      }
       if(distanceKananDepan < 20 && distanceKananBelakang < 20){
          break;
       }
      
//        majuJalan(0,5);
    }
    
    if(distanceDepan < 20 && distanceKiriDepan < 20 && distanceKiriDepan < 20){
      putarKanan(0);
//        geserKanan(0);
    }

    else if(distanceKiriDepan<US6Sedang){
      putarKanan(5);
//      geserKanan(0);
    }
    
    else if(distanceKananDepan<US2Sedang){
      putarKiri(5);
    }

//    else if(jarak[2]>14 && jarak[2]<16 && ikutKanan == true){
//      geserKanan(0);
////      puteranKanan++;
//    }
    
//    else if(distanceKiriDepan<15){
//      geserKanan(0);
//
//    }

//    else if(jarak[2]<12 && ikutKanan == false){
//      geserKiri(0);
//    }

    else if(distanceDepan < US1Dekat){
      mundur(0);
    }
    
    else{
      maju(0, 1);
      if(perjalanan == true && rintangan_satu == false){
        Serial.println("Rintangan 1");
//        majuAngkat(0,25,60,5,10); // kodingan sebelumnya
          majuAngkat(0,7,85,5,10); // yang baru
        rintangan_satu = true;
        perjalanan = false;
      }
      if(perjalanan == true && rintangan_satu == true){
        Serial.println("Rintangan 2");
//        majuAngkat(0,20,60,3,11); // kodingan sebelumnya
          majuAngkat(0,13,85,3,12); // yang baru
        perjalanan = false;
      }
    }    
}

void deteksiJarak(){    
    while(jarak[0]<US1Sedang && jarak[4]>US5Jauh && perjalanan == false){
      
      while(jarak[3]>12){
        putarKiriSpd(0,8);
        Serial.println("belok 1");
        perjalanan = true;
        ikutKanan = true;
        if(jarak[3]<20){
          break;
        }
      }
      if(jarak[3]<20){
          break;
      }
      
//        majuJalan(0,5);
    }
    
    if(jarak[0]<US1Sedang && jarak[2]>US3Jauh && jarak[4]<US5Sedang){
      putarKanan(0);
//        geserKanan(0);
    }

    else if(jarak[5]<US6Sedang){
      putarKanan(0);
//      geserKanan(0);
    }
    
    else if(jarak[1]<US2Sedang){
      putarKiri(0);
    }

    else if(jarak[2]>14 && jarak[2]<16 && ikutKanan == true){
      geserKanan(0);
//      puteranKanan++;
    }
    
    else if(jarak[4]<15){
      geserKanan(0);

    }

    else if(jarak[2]<12 && ikutKanan == false){
      geserKiri(0);
    }

    else if(jarak[0]<US1Dekat){
      mundur(0);
    }
    
    else{
      maju(0, 1);
      if(perjalanan == true && rintangan_satu == false){
        Serial.println("Rintangan 1");
//        majuAngkat(0,25,60,5,10); // kodingan sebelumnya
          majuAngkat(0,7,85,5,10); // yang baru
        rintangan_satu = true;
        perjalanan = false;
      }
      if(perjalanan == true && rintangan_satu == true){
        Serial.println("Rintangan 2");
//        majuAngkat(0,20,60,3,11); // kodingan sebelumnya
          majuAngkat(0,13,85,3,12); // yang baru
        perjalanan = false;
      }
    }    
}

void deteksiJarakCoba(){
  wasakaUltraSonic();
  
    while(distanceDepan < 20 && distanceKananDepan > 20 && perjalanan == false){
      wasakaUltraSonic();
      while(distanceKananBelakang > 12){
        Serial.println("belok 1");
        putarKiri(0);
        Serial.println("belok 1");
        perjalanan = true;
        ikutKanan = true;
        if(distanceKananDepan < 20 && distanceKananBelakang < 20){
          break;
        }
      }
      if(distanceKananDepan<20 && distanceKananBelakang<20){
          break;
      }
      
//        majuJalan(0,5);
    }
      
      if( distanceKananDepan < 20 && distanceKananBelakang <20 ){
        geserKiri(0);
      }
  
      else if( distanceKiriDepan < 20 && distanceKiriBelakang <20){
        geserKanan(0);
      }

      else if(distanceDepan < 20 && (distanceKananDepan < 20 ||distanceKananBelakang < 20 )) {
        putarKiri(0.5);
      }


      else if(distanceDepan < 40 && (distanceKiriDepan < 30 ||distanceKiriBelakang < 30 )) {
        putarKanan(0.5);
      }

      
      else{
        maju(0, 1);
  //      if(perjalanan == true && rintangan_satu == false){
  //        Serial.println("Rintangan 1");
  ////        majuAngkat(0,25,60,5,10); // kodingan sebelumnya
  //          majuAngkat(0,7,85,5,10); // yang baru
  //        rintangan_satu = true;
  //        perjalanan = false;
  //      }
  //      if(perjalanan == true && rintangan_satu == true){
  //        Serial.println("Rintangan 2");
  ////        majuAngkat(0,20,60,3,11); // kodingan sebelumnya
  //          majuAngkat(0,13,85,3,12); // yang baru
  //        perjalanan = false;
  //      }
      }    
}

void manuverJarak(){    
    if(jarak[0]<US1Sedang && jarak[2]<US3Sedang && jarak[4]<US5Jauh){
      putarKiri(0);
    }
    
    else if(jarak[0]<US1Sedang && jarak[2]>US3Jauh && jarak[4]<US5Sedang){
      putarKanan(0);
    }

    else if(jarak[5]<US6Sedang){
      putarKanan(0);
    }
    
    else if(jarak[1]<US2Sedang){
      putarKiri(0);
    }
    
    else if(jarak[4]<US5Dekat){
    geserKanan(0);
//      follow("L");
    }

    else if(jarak[2]<US3Dekat){
    geserKiri(0);
//      follow("R");
    }

    else if(jarak[0]<US1Dekat){
//      Serial.println("2");
      mundur(0);
    }
    
    else{
      maju(0,mtc);
//      follow("L");
    }
       
}

void deteksiJarakRuangan1(){
    if(jarak[4]<US5Dekat){
    geserKanan(0);
//      follow("L");
    }

//    while(jarak[2]<US3Dekat){
//    geserKiri(0);
////      follow("R");
//    }

    else if(jarak[0]<US1Sedang && jarak[2]<US3Sedang || jarak[0]<US1Sedang && jarak[4]<US5Sedang){
      putarKanan(0);
    }
    
    else if(jarak[5]<US6Sedang){
      putarKanan(0);
    }
    
//    while(jarak[1]<US2Sedang){
//      putarKiri(0.5);
//    }

//    if(jarak[0]<65){
//          delay(500);
//          runSt = 17;
//    }
    
    else{
      mundur1(1);
//      follow("L");
    }

    
}
/* ======================================================================================================================

// runSt = 0;
int jalanBergerak(){
    if(jarak[0]<US1Sedang && jarak[2]<US3Sedang && jarak[3]>US5Jauh){
      runSt = 7; //Putar Kiri
    }
    
    else if(jarak[0]<US1Sedang && jarak[2]>US3Jauh && jarak[3]<US5Sedang){
      runSt = 3; //Putar Kanan
    }
    
    else if(jarak[0]<US1Dekat){
      runSt = 5; //Mundur
    }
    
    else if(jarak[2]<US3Dekat){
//      runSt = 12;//geserKanan(0);
//      runSt=1;
    }
    
    else if(jarak[3]<US5Dekat){
//      runSt = 11;//geserKiri(0);
//      runSt=1;
    }
    
    else if(jarak[4]<US6Sedang){
      runSt = 3;//putarKanan(0);
    }
    else if(jarak[1]<US2Sedang){
      runSt = 7;//putarKiri(0);
    }
    
    else{
      runSt = 1;//maju(0);
    }    
    return runSt;
}
*/

//NEW
void deteksiBerjalanCoba(){

  wasakaUltraSonic();
//  Kondisi 1 = maju
//  Kondisi 2 = serong kanan
//  Kondisi 3 = kanan
//  Kondisi 4 = kiri
//  Kondisi 5 = serong kiri
//  Kondisi 6 = Geser kiri
//  Kondisi 7 = Geser kanan
  if(distanceDepan > 20 && distanceBelakang > 20)
  {
//    Kondisi = 1; //Pertama
    maju(0,mtc);
    
//    if(distanceKananDepan<US3Dekat ){
//    geserKiri(0);
//    }
//
//    else{
//    geserKanan(0);
//    }
  }

  else if(distanceDepan < 15 && (distanceKananDepan < 20 || distanceKananBelakang < 20)) {
    putarKanan(0);
  }
  
  else if(distanceDepan < 15 && (distanceKiriDepan < 20 || distanceKiriBelakang < 20)) {
    putarKiri(0);
  }

  else if(distanceKananDepan < 15 || distanceKananBelakang < 15 ) {
    geserKiri(0);
  }

  else if(distanceKiriDepan < 15 || distanceKiriBelakang < 15 ) {
    geserKanan(0);
  }
}
void deteksiBerjalan(){
//  Kondisi 1 = maju
//  Kondisi 2 = serong kanan
//  Kondisi 3 = kanan
//  Kondisi 4 = kiri
//  Kondisi 5 = serong kiri
//  Kondisi 6 = Geser kiri
//  Kondisi 7 = Geser kanan
  if(jarak[0]>=US1Jauh && jarak[1]>US2Jauh && (jarak[2]>US3Dekat && jarak[2]<US3Jauh) /*&& US4<=US4Dekat*/ && (jarak[3]>US5Dekat && jarak[3]<US5Jauh) && jarak[4]>US6Jauh)
  {
//    Kondisi = 1; //Pertama
    maju(0,mtc);
    
    if(jarak[2]<US3Dekat){
//    Kondisi = 7;
    geserKanan(0);
    }

    else{
//    Kondisi = 6;
    geserKiri(0);
    }
  }

  else if(jarak[0]>=US1Jauh && jarak[1]>US2Jauh && (jarak[2]>US3Dekat && jarak[2]<US3Jauh) && /*US4>=US4Jauh &&*/ (jarak[3]>US5Dekat && jarak[3]<US5Jauh) && jarak[4]>US6Jauh)
  {
//    Kondisi = 1; //Kedua
    maju(0,mtc);
    if(jarak[2]>US3Dekat){
//    Kondisi = 7;
    geserKanan(0);
    }

    else{
//    Kondisi = 6;
    geserKiri(0);
    }
  }

  else if(jarak[0]>=US1Jauh && jarak[1]>US2Jauh && (jarak[2]>US3Dekat && jarak[2]<US3Jauh) && /*US4>=US4Jauh && */jarak[3]<US5Dekat && jarak[4]>US6Jauh)
  {
//    Kondisi = 2; //Ketiga
  }

  else if(jarak[0]>=US1Jauh && jarak[1]>US2Jauh && jarak[2]>US3Jauh && /*US4>=US4Jauh && */jarak[3]<US5Dekat && jarak[4]>US6Jauh)
  {
//    Kondisi = 3; //Ke empat
    putarKanan(0);
  }

  else if(jarak[0]>=US1Jauh && jarak[1]>US2Jauh && jarak[2]<US3Dekat && /*US4>=US4Jauh && */(jarak[3]>US5Dekat && jarak[3]<US5Jauh) && jarak[4]>US6Jauh)
  {
//    Kondisi = 5; //Kelima
  }

  else if(jarak[0]>=US1Jauh && jarak[1]>US2Jauh && jarak[2]<US3Dekat && /*US4>=US4Jauh && */jarak[3]>US5Jauh && jarak[4]>US6Jauh)
  {
//    Kondisi = 4; //Ke enam
    putarKiri(0);
  }

  else if(jarak[0]<=US1Dekat && jarak[1]>US2Jauh && (jarak[2]>US3Dekat && jarak[2]<US3Jauh) && /*US4>=US4Jauh && */(jarak[3]>US5Dekat && jarak[3]<US5Jauh) && jarak[4]>US6Jauh)
  {
//    Kondisi = 3; //Ketujuh
    putarKanan(0);
  }

  else if(jarak[0]<=US1Dekat && jarak[1]>US2Jauh && jarak[2]>US3Jauh && /*US4>=US4Jauh  && */jarak[3]<US5Dekat && jarak[4]>US6Jauh)
  {
//    Kondisi = 3; //Kedelapan
    putarKanan(0);
  }

  else if(jarak[0]<=US1Dekat && jarak[1]>US2Jauh && (jarak[2]>US3Dekat && jarak[2]<US3Jauh) && /*US4>=US4Jauh && */jarak[3]>US5Jauh && jarak[4]>US6Jauh)
  {
//    Kondisi = 4; //Kesembilan
    putarKiri(0);
  }

  else if(jarak[0]<=US1Dekat && jarak[1]>US2Jauh && jarak[2]<US3Dekat && /*US4>=US4Jauh && */jarak[3]>US5Jauh && jarak[4]>US6Jauh)
  {
//    Kondisi = 4; //Kesepuluh
    putarKiri(0);
  }

  else if(jarak[0]<=US1Dekat && jarak[1]>US2Jauh && jarak[2]>US3Jauh && /*US4>=US4Jauh && */(jarak[3]>US5Dekat && jarak[3]<US5Jauh) && jarak[4]>US6Jauh)
  {
//    Kondisi = 3; //Kesebelas
    putarKanan(0);
  }

  else if(jarak[1]<=US2Jauh)
  {
//    Kondisi = 5; //Keduabelas
  }

  else//(US5<=US5Jauh)
  {
//    Kondisi = 2; //Ketigabelas
  }
}

/*
long counterUkur = 0;
void timerUkur() {
  unsigned long t = millis();
  if ((t - last_time) > update_interval) {
    last_time = t;
    counterUkur++;
    if (counterUkur % 2 == 0) {
      ukurJarak();
    }
//    ukurJarak();
  }
}

long counterDeteksi = 0;
void timerDeteksi() {
  unsigned long t = millis();
  if ((t - last_time) > update_interval) {
    last_time = t;
//    counterUkur++;
//    if (counter- % 2 == 0) {
//      ukurJarak();
//    }
    deteksiJarak();
  }
}*/
