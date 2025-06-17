//  ===== Variable Servo ===== //

//const float DXL_PROTOCOL_VERSION = 1.0;
//
//Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

#include "1_Variable.h"
byte printOut = 0;
int P36 = 6;

byte smothtly = 1; //1, 2, 3, 4 //default 3
int JS = 6; //6, 12, 18, 24 smoothly //default 18
//byte smothtly = 2; //1, 2, 3, 4 //default 3
//int JS = 12; //6, 12, 18, 24 smoothly //default 18

byte calSt = 1;
byte dir = 1;
byte pushSt = 1;//BT
byte btSt = 0;
byte activeSt = 0;
byte clrSt = 0;
byte oledSt = 0;
byte kipasSt = 0;
byte apiSt = 0;
byte buzzerApiSt = 0;
int menuSt = 0;
byte runSt = 0;
//EEPROM.writeInt(405, fireLimit);

 int buttonState = 1;
 bool Kondisi = false;
 int startJalan = 0;
 
 int statRobot = 0;
 int statKondisi = 0;
 int mtc = 1;
 bool adaApi = false;

 int atas = 155;
 int tengah = 20;
 int bawah = 60;

void setup() {
  
  CapitAtas.attach(20);
  CapitTengah.attach(19);
  CapitBawah.attach(18);
  
  CapitTengah.write(tengah);
  CapitAtas.write(atas);
  CapitBawah.write(bawah);
  delay(1000);
  

 //Dynamxel=======================================
    // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
//  Serial.begin(115200);
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
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID4, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID5, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID6, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID7, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID8, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID9, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID10,0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID11, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID12, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID13, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID14, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID15, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID16, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID17, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID18, 0);















 //==============================================
  Serial.println();
  Serial.println("Quadruped Robot Program");
//  testOledSt();
//  setupBT();
  //testBTSt();
  pinMode(buzzerPin, OUTPUT);
//  EEPROM_ARR(3);

  if (calSt == 1) {
    gaitsTrajectoryCalculation(smothtly);
    moveToCalculation(dir);
    calSt = 0;
  }
  pinMode(buttonPin, INPUT_PULLUP); //push button
//  setupServo(1);

  kipasOn();
  beep1();
  beep();
//  delay(500);
  pushSt = 0;
  kipasOff();
//  SensorUltra();
//  Serial.print("yAxis ");
//  Serial.println(yAxis);

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

  dxl.setGoalVelocity(DXL_ID6, 200);
  dxl.setGoalPosition(DXL_ID6, 512);

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

      // ===== Goals Servo 3 ===== //

//
//  setupServoDynamixel(1);
//maju(5,1);
//  wasakaUltraSonic(); 

}

void loop1() { // ini untuk mencoba fungsi dari bbrp void

//  normalisasiKapit();
  //oledNavigator();
  //timer();
  action();
  //Serial.print("btSt ");
  //Serial.println(btSt);
  //delay(500);
  //setServoA(45, 100); // Gripper 100 tutup, 30 buka
  //setServoB(46, 180);  //0 UP  180 Down
  //testServo(47); // kipas
  //teganganBaterai();
  kalibrasi();
  //bacaApi();
  //ukurJarak();

}

/*void loop() {
  //if (oledSt == 1) oledNavigator();
  //if (btSt == 1)
  bluetoothControl();

  if (calSt == 1) {
    gaitsTrajectoryCalculation(smothtly);
    moveToCalculation(dir);
    calSt = 0;
  } else {
    runningMainProgram();
    //runningMainProgram1();
  }
  timer();
}
*/
bool case13 = true;
bool case14 = true;
int jangka = 0;
bool done = false;
//int update_interval = 1000; // time interval in ms for updating panel indicators
unsigned long last_time = 0; // time of last update
//int tambah1 = 1;
//int waktu_sebelumnya = 0;
bool ikutKanan = false;
bool tombol = true;
bool startRobot = false;
bool ruanganSatu = false;
bool statAngkat = true;
int puteranKanan = 0;
int puteranKiri = 0;

//Manuver rintangan
bool perjalanan = false;
bool rintangan_satu = false;
bool keluar_kondisi_satu = false;

//int servoCapit = 20;
//int pos = 90; 
//maju(5,1);
int runServo = 1;


void loop() {
// geserKanan(0);
//    while(true){
//      wasakaUltraSonic();
//      berjalanCobaBGTT(); 
//  }

  // Jalankan gerakan geser kiri
  geserKiri(5);  // Parameter 5 berarti akan melakukan 5 langkah geser kiri
  delay(2000);   // Tunggu 2 detik setelah selesai
  
  // Setelah selesai, kembali ke posisi normal
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
  dxl.setGoalVelocity(DXL_ID6, 200);
  dxl.setGoalPosition(DXL_ID6, 512);
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
  
  delay(2000);  // Tunggu 2 detik sebelum mengulang
}

void nyariApi(){
  switch (runSt) {
  case 20: // lilin
//        bacaApi();
        break;
      case 21:
//        padamkanLilin();
        break;
      case 22:
        putarKiriAction(2, 80, 50, 10, 15, 23);
        break;
      case 23:
        putarKananAction(3, 80, 50, 10, 15, 24);
        break;
      case 24:
        pompaOff();
        break;
  }
}

void cariApi(){
  switch(runSt){
  case 20: // lilin
//        bacaApi();
        break;
      case 21:
        putarKananAction(1, 80, 50, 10, 10, 22);
        break;
      case 22:
        putarKiriAction(1, 80, 50, 10, 10, 24);
        break;
      case 23:
        pompaOn();
        delay(2000);
        jumlahLangkah = 0;
        runSt=21;
        break;
      case 24:
        pompaOff();
        break;
      case 30:
//        mundurAction(2, 80, 40, 10, 20, 31);
        break;
      case 99:
        break;
  }
}

void berjalan4(){
  switch(runSt){
      case 0:
        putarKiriAction(20, 70, 50, 10, 8, 3);
//        putarKanan(0);
        while(jarak[2]<20 && jarak[4]<20 && jumlahLangkah > 2 ){ 
          runSt=4;
//          majuAction(8, 70, 45, 0, 5, 4);
        majuAction(0, 60, 45, 10, 5, 4); // Koding yang sebelumnya
//            majuJalan(0,5);
//          deteksiJarak();
        }
        break;
      case 1:
        break;
      case 2:
        putarKananAction(20, 70, 50, 10, 8, 1);
        while(jarak[2]<20 && jarak[4]<20 && jumlahLangkah > 2){
          runSt=4;
          majuAction(0, 60, 45, 10, 5, 4);
//          majuAction(8, 70, 45, 0, 5, 4);
//            majuJalan(0,5);
//          deteksiJarak();
        }
        break;
      case 3:
      majuAngkat(0,25,60,5,10);
        break;
      case 4:
        deteksiJarak();
        break;
      case 5:
        break;
  }
}

void berjalanCoba(){
  switch(runSt){
      case 0:
        // == Terdeteksi Depan == //
        while(jarak[0] < 10 ){
          runSt=4;
          mundur(0);
        }
        break;
      case 1:
        break;
      case 2:
        // == Terdeteksi Depan == //
        while( jarak[5] < 10 ){
          runSt=4;
//          majuAction(8, 70, 45, 0, 4, 4);
            majuJalan(0,5);
//          deteksiJarak();
        }
        break;
      case 3:
        break;
      case 4:
        deteksiJarakCoba();
        break;
      case 5:
        break;
  }
}

void berjalanCobaBGTT(){
  wasakaUltraSonic();
  switch(runSt){
      case 0:
      wasakaUltraSonic();
      
        maju(0,1);
        while(distanceKiriDepan < 12 ){
//          runSt=4;
          putarKanan(10);
          if (distanceKiriDepan > 25 && distanceBelakang > 15) {
            break;
          }
        
        }

         while(distanceKananDepan < 12 ){
//          runSt=4;
          putarKiri(10);
          if (distanceKananDepan > 25 &&  distanceBelakang > 15) {
            break;
          }
        
        }

        while(distanceDepan < 12 ){
//          runSt=4;
          mundur(5);
          if (distanceDepan > 25) {
            break;
          }
        }

        
//        jumlahLangkah = 0;
        break;
      case 1:
        break;
      case 2:
        putarKiri(20);
        while(distanceKiriDepan <20 && distanceKananBelakang<20 && jumlahLangkah > 2){
          runSt=4;
            maju(0,1);
        }
        break;
      case 5:
        break;
  }
}

void berjalanCobaBGTt(){
  switch(runSt){
      case 0:
        putarKanan(20);
        while(distanceDepan > 20 && distanceBelakang > 20 && jumlahLangkah > 3 ){
          runSt=4;
        }

 
        break;
      case 1:
        break;
      case 2:
        putarKiri(20);
        while(distanceKiriDepan <20 && distanceKananBelakang<20 && jumlahLangkah > 2){
          runSt=4;
            maju(0,1);
        }
        break;
      case 3:
        break;
      case 4:
        wasakaUltraSonic();
      
        maju(0,1);
        while(distanceKiriDepan < 13 ){
//          runSt=4;
          putarKanan(10);
          if (distanceKiriDepan > 30 && distanceDepan > 30 && distanceBelakang > 20) {
            break;
          }
        
        }

         while(distanceKananDepan < 13 ){
//          runSt=4;
          putarKiri(10);
          if (distanceKananDepan > 30  && distanceDepan > 30 &&  distanceBelakang > 20) {
            break;
          }
        
        }
//        jumlahLangkah = 0
        break;
      case 5:
        break;
  }
}

void berjalanCobaBGT(){
  switch(runSt){
      case 0:
        putarKanan(20);
        while(distanceDepan > 20 && distanceBelakang > 20 && jumlahLangkah > 3 ){
          runSt=4;
          maju(0,1);
        }
        break;
      case 1:
        break;
      case 2:
        putarKiri(20);
        while(distanceKiriDepan <20 && distanceKananBelakang<20 && jumlahLangkah > 2){
          runSt=4;
            maju(0,1);
        }
        break;
      case 3:
        break;
      case 4:
        deteksiJarakFix();
        break;
      case 5:
        break;
  }
}

void berjalan(){
  switch(runSt){
      case 0:
        putarKananAction(20, 70, 50, 10, 8, 1);
//        putarKanan(0);
        while(jarak[2]<20 && jarak[4]<20 && jumlahLangkah > 2 ){
          runSt=4;
//          majuAction(8, 70, 45, 0, 4, 4);
            majuJalan(0,5);
//          deteksiJarak();
        }
        break;
      case 1:
        break;
      case 2:
        putarKiriAction(20, 70, 50, 10, 8, 3);
        while(jarak[2]<20 && jarak[4]<20 && jumlahLangkah > 2){
          runSt=4;
//          majuAction(8, 70, 45, 0, 4, 4);
            majuJalan(0,5);
//          deteksiJarak();
        }
        break;
      case 3:
        break;
      case 4:
        deteksiJarak();
        break;
      case 5:
        break;
  }
}

void ruangan1Api(){
  switch(runSt){
    case 1:
      break;
  }
}

void ksri() {
  wasakaUltraSonic();
  switch(runSt){
      case 0:
      geserKanan(20);
      wasakaUltraSonic();
        while(distanceKiriDepan < 12 && distanceDepan > 20){
          putarKanan(10);
          if (distanceKiriDepan > 25 && distanceBelakang > 15) {
            break;
          }
        
        }

         while(distanceKananDepan < 12 ){
//          runSt=4;
          putarKiri(10);
          if (distanceKananDepan > 25 &&  distanceBelakang > 15) {
            break;
          }
        
        }

        while(distanceDepan < 12 ){
//          runSt=4;
          mundur(5);
          if (distanceDepan > 25) {
            break;
          }
        }

        
//        jumlahLangkah = 0;
        break;
      case 1:
        break;
      case 2:
        putarKiri(20);
        while(distanceKiriDepan <20 && distanceKananBelakang<20 && jumlahLangkah > 2){
          runSt=4;
            maju(0,1);
        }
        break;
      case 5:
        break;
  }
}

void ruangan1Korban(){
  switch(runSt){
      case 10:
        tarok();
        delay(1000);
        runSt = 11;
        break;
      case 11:
        angkat();
        delay(1000);
        runSt = 12;
        break;
      case 12:      
        putarKanan(1);
        while(case14){
          jangka=jumlahLangkah+1;
          case14=false;
        }
        while(jangka==jumlahLangkah){
          runSt = 13; 
          break; 
        }
        break;
      case 13:
        maju(0,mtc);
//mundur1(20);
          if(jarak[0]<20){
            mundur1(1);
            runSt=14;
          }
      case 14:
        putarKananAction(6, 80, 85, 10, 8, 14);
        if(jarak[0]>60){
          runSt=15;
        }
        break;
      case 15:
      while(jarak[3]<18){
          tarok();
          delay(200);
          runSt = 15;
          break;
      }
        break;
      case 16:
        mundur1(1);
        if(jarak[0]<105 && jarak[0]>90){
          angkat();
          delay(800);
          runSt = 17;
        }
        break;
      case 17:
        deteksiJarakRuangan1();
        break;
      case 18:
        putarKananAction(6, 80, 85, 10, 8, 16);
        while(jarak[0]>70){
          majuAction(5, 70, 45, 0, 2, 17);
          runSt=19;
        }
//        while(case14){
//          jangka=jumlahLangkah+3;
//          case14=false;
//        }
//        while(jangka==jumlahLangkah){
//          runSt = 16;  
//        }
        break;
      case 19:
      maju(0,mtc);
//      putarKananAction(6, 80, 53, 10, 5, 18);
//        while(jarak[0]>70){
//          maju(0);
//          runSt=19;
//        }
//        putarKananAction(9, 80, 85, 10, 5, 12);
//      while(jarak[4]<42 && jumlahLangkah > 6){
//          mundur(0);
//          runSt=13;
//      }
        break;
      case 20:
        putarKanan(0.5);
//        runSt=20;
        break;
      case 21:
//        runSt=20;
        break;
  }
}
/*
void runningMainProgram1() {
  switch (runSt) {
    case 0:
      break;
    case 1:
      maju(0);
      break;
    case 60:
      mundur(2);
      break;
  }
}


void runningBergerak(){
//  runSt = jalanBergerak();
  switch (runSt) {
      case 0:
        break;
      case 1:
        maju(0);
        //majuAction(0, 70, 50, 10, 99);
        //majuAction(0, 80, 30, 0, 99);
        break;
      case 2:
        belokKanan(0.5);
        break;
      case 3:
        putarKanan(0);
        break;
      case 5:
        mundur(0);
        break;
      case 7:
        putarKiri(0);
        break;
      case 8:
        belokKiri(0.5);
        break;
      case 9:
        action();
        break;
      case 10:
        stanBy();
        break;
      case 11:
        geserKiri(2);
        break;
      case 12:
        geserKanan(2);
        break;
      case 13:
        setupServo(1);
        runSt = 0;
        break;
      case 60:
        //mundur1(3);
        break;
      case 61:
        //putarKanan1(6);
        break;
      case 20: // lilin
        bacaApi();
        break;
      case 21:
        padamkanLilin();
        break;
      case 22:
        putarKiriAction(2, 80, 50, 10, 15, 23);
        break;
      case 23:
        putarKananAction(3, 80, 50, 10, 15, 24);
        break;
      case 24:
        kipasOff();
        break;
      case 30:
//        mundurAction(2, 80, 40, 10, 20, 31);
        break;
      case 31:
        putarKananAction(6, 80, 50, 10, 15, 32);
        break;
      case 32:
//        mundurAction(2, 80, 40, 10, 20, 33);
        break;
      case 33:
        gripperClose();
        angkat();
        runSt = 34;
        break;
      case 34:
        putarKiriAction(6, 80, 50, 10, 15, 35);
        break;
      case 35:

        break;
    }
}

void runningMainProgram() {
  if (activeSt == 1) {
    switch (runSt) {
      case 0:
        break;
      case 1:
        maju(0);
        //majuAction(0, 70, 50, 10, 99);
        //majuAction(0, 80, 30, 0, 99);
        break;   
      case 2:
        belokKanan(0.5);
        break;
      case 3:
        putarKanan(0);
        break;
      case 5:
        mundur(0);
        break;
      case 7:
        putarKiri(0);
        break;
      case 8:
        belokKiri(0.5);
        break;
      case 9:
        action();
        break;
      case 10:
        stanBy();
        break;
      case 11:
        geserKiri(2);
        break;
      case 12:
        geserKanan(2);
        break;
      case 13:
        setupServo(1);
        runSt = 0;
        break;
      case 60:
        //mundur1(3);
        break;
      case 61:
        //putarKanan1(6);
        break;
      case 20: // lilin
        bacaApi();
        break;
      case 21:
        padamkanLilin();
        break;
      case 22:
        putarKiriAction(2, 80, 50, 10, 15, 23);
        break;
      case 23:
        putarKananAction(3, 80, 50, 10, 15, 24);
        break;
      case 24:
        kipasOff();
        break;
      case 30:
//        mundurAction(2, 80, 40, 10, 20, 31);
        break;
      case 31:
        putarKananAction(6, 80, 50, 10, 15, 32);
        break;
      case 32:
//        mundurAction(2, 80, 40, 10, 20, 33);
        break;
      case 33:
        gripperClose();
        angkat();
        runSt = 34;
        break;
      case 34:
        putarKiriAction(6, 80, 50, 10, 15, 35);
        break;
      case 35:

        break;
    }


  }
}
*/

void action() {
  if (langkah < addStep)
    langkah++;
  action_1(langkah);
}

void stanBy() {
  if (directionSt != 1) {
    moveToCalculation(2);
    directionSt = 1;
  }
  if (langkah >= addStep / 2) {
    if (langkah < addStep) {
      langkah++;
      moveTo(langkah);
    }
  } else if (langkah > 1) {
    numberStep -= 1;
    langkah--;
    moveTo(langkah);
  }
}
