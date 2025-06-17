//int USJarakk1(){
//  digitalWrite(trig1Pin, LOW);
//  delayMicroseconds(8);
//  digitalWrite(trig1Pin, HIGH);
//  delayMicroseconds(8);
//  digitalWrite(trig1Pin, LOW);
//  delayMicroseconds(8);
//
//  durasi1 = pulseIn(echo1Pin, HIGH); // menerima suara ultrasonic
//  return jarak1 = (durasi1 / 2) / 29.1;  
//}
//
//int USJarakk2(){
//  digitalWrite(trig2Pin, LOW);
//  delayMicroseconds(8);
//  digitalWrite(trig2Pin, HIGH);
//  delayMicroseconds(8);
//  digitalWrite(trig2Pin, LOW);
//  delayMicroseconds(8);
//
//  durasi2 = pulseIn(echo2Pin, HIGH); 
//  jarak2 = (durasi2 / 2) / 29.1;  
//  return jarak2;
//}
//
//int USJarakk3(){
// 
//  digitalWrite(trig3Pin, LOW);
//  delayMicroseconds(8);
//  digitalWrite(trig3Pin, HIGH);
//  delayMicroseconds(8);
//  digitalWrite(trig3Pin, LOW);
//  delayMicroseconds(8);
//
//  durasi3 = pulseIn(echo3Pin, HIGH); 
//  jarak3 = (durasi3 / 2) / 29.1;  
//  return jarak3;
//}

//void teganganBaterai() {
//  for (int i = 0; i < 5; i++) {
//    voltageReading += analogRead(bat2sSensPin);
//    voltageReading1 += analogRead(bat1sSensPin);
//    delay(1);
//  }
//  //voltageReading = voltageReading / 5 * 0.0082;// * calibFacV;
//  //voltageReading1 = voltageReading1 / 5 * 0.004857;// * calibFacV;
//  voltageReading = voltageReading / 5 * 0.008771;// * calibFacV;
//  //voltageReading1 = voltageReading1 / 5 * 0.0057;// * calibFacV;
//  batVol = voltageReading;
//  batVol1 = voltageReading1;
//  //Serial.print("V ");
//  Serial.println(voltageReading, 2);
//  //Serial.print("V1 ");
//  //Serial.println(voltageReading1, 2);
//  //batPct = mapFloat(batVol, 6.2, 8.4, 0, 100);
//  //batPct = (153 * batVol - 540) / 10;
//  batPct = (78.2 * batVol - 544) * 0.94;
//  //Serial.print("pct ");
//  Serial.println(batPct, 1);
//
//  if (batPct < 0) batPct = 0; if (batPct > 100) batPct = 100;
//  if (voltageReading <= 7.2) batSt = 0;
//  else if (voltageReading <= 7.5) batSt = 1;
//  else batSt = 2;
//
//  sendToAndroidDevice("B", String(batPct)); //delay(10);
//  sendToAndroidDevice("z", String(batVol) + " v"); //delay(10);
//  sendToAndroidDevice("Z", String(batPct) + " %"); //delay(10);
//  byte G = batPct;
//  Serial.print("*LR" + String(250 - (G * 2.5)) + "G" + String(G * 2.5) + "B" + String(0) + "*");
////Serial.print("*LR" + String(250 - (G * 2.5)) + "G" + String(G * 2.5) + "B" + String(0) + "*");
//
//}

//Sensor UltraSonik 1
int echo1;           // membuat variabel echo yang di set ke-pin 2
int trig1;           // membuat varibel trig yang di set ke-pin 3

//Sensor UltraSonik 2
int echo2;           // membuat variabel echo yang di set ke-pin 4
int trig2;           // membuat varibel trig yang di set ke-pin 5

//Sensor UltraSonik 3
int echo3;           // membuat variabel echo yang di set ke-pin 6
int trig3;           // membuat varibel trig yang di set ke-pin 7

int oldJR, oldJL;

//void SensorUltra(){
//  pinMode(echo1Pin, INPUT);     // set pin echo menjadi INPUT
//  pinMode(trig1Pin, OUTPUT);    // set pin trig menjadi OUTPUT
//  pinMode(echo2Pin, INPUT);     // set pin echo menjadi INPUT
//  pinMode(trig2Pin, OUTPUT);    // set pin trig menjadi OUTPUT
//  pinMode(echo3Pin, INPUT);     // set pin echo menjadi INPUT
//  pinMode(trig3Pin, OUTPUT);    // set pin trig menjadi OUTPUT
//}
//
//int USJarak1(){
//    // program dibawah ini agar trigger memancarakan suara ultrasonic
//  digitalWrite(trig1Pin, LOW);
//  delayMicroseconds(8);
//  digitalWrite(trig1Pin, HIGH);
//  delayMicroseconds(8);
//  digitalWrite(trig1Pin, LOW);
//  delayMicroseconds(8);
//
//  durasi1 = pulseIn(echo1Pin, HIGH); // menerima suara ultrasonic
//  jarak1 = (durasi1 / 2) / 29.1;  // mengubah durasi menjadi jarak (cm)
//  return jarak1;
////  Serial.println(jarak1);        // menampilkan jarak pada Serial Monitor
//}
//
//int USJarak2(){
//    // program dibawah ini agar trigger memancarakan suara ultrasonic
//  digitalWrite(trig2Pin, LOW);
//  delayMicroseconds(8);
//  digitalWrite(trig2Pin, HIGH);
//  delayMicroseconds(8);
//  digitalWrite(trig2Pin, LOW);
//  delayMicroseconds(8);
//
//  durasi2 = pulseIn(echo2Pin, HIGH); // menerima suara ultrasonic
//  jarak2 = (durasi2 / 2) / 29.1;  // mengubah durasi menjadi jarak (cm)
//  return jarak2;
////  Serial.println(jarak2);        // menampilkan jarak pada Serial Monitor
//}
//
//int USJarak3(){
//    // program dibawah ini agar trigger memancarakan suara ultrasonic
//  digitalWrite(trig3Pin, LOW);
//  delayMicroseconds(8);
//  digitalWrite(trig3Pin, HIGH);
//  delayMicroseconds(8);
//  digitalWrite(trig3Pin, LOW);
//  delayMicroseconds(8);
//
//  durasi3 = pulseIn(echo3Pin, HIGH); // menerima suara ultrasonic
//  jarak3 = (durasi3 / 2) / 29.1;  // mengubah durasi menjadi jarak (cm)
//  return jarak3;
////  Serial.println(jarak3);        // menampilkan jarak pada Serial Monitor
//}

void ukurWarna(){
  unsigned long t = millis();
  if ((millis() - last_time) > 100) {
    last_time = millis();
//    counter1++;
//    if (counter1 % 4 == 0) {
      int j;
  for (uint8_t i = 0; i < jumlahWarna; i++) {
    //for (uint8_t i = 0; i < 1; i++) {
    delay(2);
    if(i==0){
      digitalWrite(WS21,LOW);
      digitalWrite(WS31,LOW);
      j = pulseIn(WsensorOut, LOW);
      warna[i] = map(j, 39, 103, 255,0);
    }
    else if(i==1){
      digitalWrite(WS21,HIGH);
      digitalWrite(WS31,HIGH);
      j = pulseIn(WsensorOut, LOW);
      warna[i] = map(j, 61, 109, 255, 0);
    }
    else if(i==2){
      digitalWrite(WS21,LOW);
      digitalWrite(WS31,HIGH);  
      j = pulseIn(WsensorOut, LOW);
      warna[i] = map(j, 38, 125, 255, 0);
    }
  }
  
  if((warna[0] < -1100 && warna[0] > -1950) && (warna[1] < -1700 && warna[1] > -2550) && (warna[2] < -500 && warna[2] > -1150)){
    Serial.println(" - Black detected!");
    hitam = 1;
    abu = 0;
    putih = 0;
    majuAngkat(0,10,10,2,1);
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
    majuAngkat(0,25,45,5,10);
  }
    }
//  }
}

void wasakaUltraSonic() {
  //Depan
  DEBUG_SERIAL.print("Sensor Depan: ");
  distanceDepan = depan.ping_cm(); 
  DEBUG_SERIAL.println(distanceDepan);
  
  //Kanan Depan
  DEBUG_SERIAL.print("Sensor Kanan Depan: ");
  distanceKananDepan = kananDepan.ping_cm(); 
  DEBUG_SERIAL.println(distanceKananDepan);

  //Kanan Belakang
  DEBUG_SERIAL.print("Sensor Kanan Belakang: ");
  distanceKananBelakang = kananBelakang.ping_cm();
  DEBUG_SERIAL.println(distanceKananBelakang);

  //Kiri Depan
  DEBUG_SERIAL.print("Sensor Kiri Depan: ");
  distanceKiriDepan = kiriDepan.ping_cm();
  DEBUG_SERIAL.println(distanceKiriDepan);

  //Kiri Belakang
  DEBUG_SERIAL.print("Sensor Kiri Belakang: ");
  distanceKiriBelakang = kiriBelakang.ping_cm();
  DEBUG_SERIAL.println(distanceKiriBelakang);

  //Belakang
  DEBUG_SERIAL.print("Sensor Belakang: ");
  distanceBelakang = belakang.ping_cm();
  DEBUG_SERIAL.println(distanceBelakang);
  
  delay(10);
}

void ukurJarak() {
//  int x = millis();
  float j;
  for (uint8_t i = 0; i < jumlahPing; i++) {
    //for (uint8_t i = 0; i < 1; i++) {
//    delay(2);
    j = pinG[i].ping_cm();
    delay(10);
    if (j == 0) j = jarakMak;
//    j = j;// - 12;
//    if (j < 0) j == 0;
    jarak[i] = j;
  }


  DEBUG_SERIAL.print("Jarak 1 "); DEBUG_SERIAL.println(jarak[0]); // Depan
  DEBUG_SERIAL.print("Jarak 2 "); DEBUG_SERIAL.println(jarak[1]); // Depan Kanan
  DEBUG_SERIAL.print("Jarak 3 "); DEBUG_SERIAL.println(jarak[2]); // Depan Kiri
  DEBUG_SERIAL.print("Jarak 4 "); DEBUG_SERIAL.println(jarak[3]); // Belakang Kanan
  DEBUG_SERIAL.print("Jarak 5 "); DEBUG_SERIAL.println(jarak[4]); // Belakang Kiri
  DEBUG_SERIAL.print("Jarak 6 "); DEBUG_SERIAL.println(jarak[5]); // Belakang
  DEBUG_SERIAL.print("Jarak Capit "); DEBUG_SERIAL.println(jarak[6]); // Sensor Capit 
  DEBUG_SERIAL.println();
  delay(500);
//  int xx = millis();
//  Serial.print("time jarak ");
//  Serial.println(xx - x);
}
/*
void jarakDepan() {
  float j = pinG[0].ping_cm();
//  int x = millis();
  if (j == 0) j = jarakMak;
  j = j - 12;
  if (j < 0) j == 0;
  jarak[0] = j;
  //int xx = millis();
  //Serial.print("time dpn ");
  //Serial.println(xx - x);
}

void jarakSerongKanan() {
  float j = pinG[1].ping_cm();
//  int x = millis();
  if (j == 0) j = jarakMak;
  j = j - 12;
  if (j < 0) j == 0;
  jarak[1] = j;
  //int xx = millis();
  //Serial.print("time dpn ");
  //Serial.println(xx - x);
}

void jarakKanan() {
  //int x = millis();
  float j = pinG[2].ping_cm();
  if (j == 0) j = jarakMak;
  j = j - 12;
  if (j < 0) j == 0;
  jarak[2] = j;
  //jarakDepan();
  //int xx = millis();
  //Serial.print("time kanan ");
  //Serial.println(xx - x);
}

void jarakKiri() {
  float j = pinG[4].ping_cm();
  if (j == 0) j = jarakMak;
  j = j - 12;
  if (j < 0) j == 0;
  jarak[4] = j;
}


void jarakSerongKiri() {
  float j = pinG[4].ping_cm();
  if (j == 0) j = jarakMak;
  j = j - 12;
  if (j < 0) j == 0;
  jarak[4] = j;
}
*/
void hitungJarakLilin() {
  //int x = millis();
  float j = pinG[3].ping_cm();
  if (j == 0) j = jarakMak;
  j = j;
  if (j < 0) j == 0;
  jarak[3] = j;
//  jarakLilin = j;
  //jarakDepan();
  //int xx = millis();
//  Serial.print("jarakLilin ");
//  Serial.println(jarakLilin);
  //Serial.println(xx - x);
}

//int tambahApi=0;
//void bacaApi() {
////fireLimit = 100;
//  int posApi = 0, nilaiApi = 0;
//  api[1] = analogRead(sApi1Pin);
//  api[2] = analogRead(sApi2Pin);
//  api[3] = analogRead(sApi3Pin);
//  api[4] = analogRead(sApi4Pin);
//  api[5] = analogRead(sApi5Pin);
//
//  if ((((api[1] > fireLimit) or (api[2] > fireLimit)) or ((api[3] > fireLimit) or (api[4] > fireLimit))) or (api[5] > fireLimit))  {
//    apiSt = 1;
//  }
//
//  if (apiSt == 1) {
//    for (byte i = 1; i < 6; i++) {
//      if (api[i] > nilaiApi) {
//        nilaiApi = api[i];
//        posApi = i;
//      }
//    }
//    nilaiApi = 0;
//
//    if (posApi != 3) {
//      if (posApi < 3) {
//        //Serial.println("putar kiri");
//        //putarKiriAction(long s, float x, float y, float z, float n, int next)
//        putarKiriAction(0, 80, 50, 0, 10, 99);
//        //if (buzzerApiSt == 1) tone(buzzerPin, 3136, 100);
//      } else {
//        //Serial.println("putar kanan");
//        putarKananAction(0, 80, 50, 0, 10, 99);
//        //putarKananAction(long s, float x, float y, float z, float n, int next);
//        // putarKananCariLilin(0);
//        //if (buzzerApiSt == 1)tone(buzzerPin, 4186, 100);
//      }
//    } else {
//      Serial.println("posisi api oke");
//      runSt = 21;
////      frameRunning();
//    }
//  }
//
//  //==========================
//  if (printOut == 4) {
//    Serial.print("api :");
//    Serial.print("\t");
//    Serial.print(api[1]);
//    Serial.print("\t");
//    Serial.print(api[2]);
//    Serial.print("\t");
//    Serial.print(api[3]);
//    Serial.print("\t");
//    Serial.print(api[4]);
//    Serial.print("\t");
//    Serial.println(api[5]);
//
//    Serial.print("nilaiApi :");
//    Serial.print(nilaiApi);
//    Serial.print(", posApi :");
//    Serial.println(posApi);
//  }
//}

void padamkanLilin() {
////  hitungJarakLilin();
//  if (jarakLilin > 20) {
////    mundurAction(0, 80, 50, 10, 15, 99);
//  } else {
    pompaOn();
    delay(1000);
    jumlahLangkah = 0;
    runSt = 22;
//    frameRunning();
    apiSt = 0;
//  }
}
