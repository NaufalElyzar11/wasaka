//int update_interval = 1000; // time interval in ms for updating panel indicators
//unsigned long last_time = 0; // time of last update
//char data_in; // data received from serial link
//int slider_value; // Received Slider Values
//String text; // String for text elements
//int progress; // Progress Bar Value
//
//void testBTSt() {
//  if (digitalRead(bluetoothStatusPin) == 1) {
//    Serial.println("Bluetooth connected ");
//    btSt = 1;
//    //setupBT();
//  } else {
//    btSt = 0;
//    Serial.println("Bluetooth disconnect ");
//  }
//}
//
//void setupBT() {
//  Serial3.begin(9600); //Change baud rate as required!
//  String hh = String(lift / 5, 0);
//  String nn = String(xspd);
//  pinMode(bluetoothStatusPin, INPUT);
//  ///////////// Build panel in app
//  Serial3.println("*.kwl");  delay(10);
//  Serial3.println("clear_panel()");  delay(10);
//  Serial3.println("set_grid_size(18,9)");  delay(10);
//  Serial3.println("add_text(4,0,xlarge,L,HEXAPOD ROBOT CONTROLLER,220,180,220,)");  delay(10);
//
//  Serial3.println("add_text(10,4,large,C,10,200,200,200,n)");  delay(10);
//
//  Serial3.println("add_button(10,3,2,u,0)");  delay(10);
//  Serial3.println("add_button(10,5,3,d,0)");  delay(10);
//  Serial3.println("add_button(9,4,4,L,0)");  delay(10);
//  Serial3.println("add_button(11,4,5,R,0)");  delay(10);
//
//  Serial3.println("add_button(12,7,9,r,0)");  delay(10);
//  Serial3.println("add_button(13,7,8,p,0)");  delay(10);
//  Serial3.println("add_button(14,7,6,x,0)");  delay(10);
//  Serial3.println("add_button(15,7,7,o,0)");  delay(10);
//  Serial3.println("add_button(16,7,20,c,0)");  delay(10);
//
//  //Serial3.println("add_text(1,1,large,C,10,220,220,220,H)");  delay(10);
//  Serial3.println("add_text(1,1,large,C,5,220,220,220,N)");  delay(10);
//
//  Serial3.println("add_text(1,6,large,C,Speed,90,180,245,)");  delay(10);
//  //Serial3.println("add_text(3,6,large,C,G&U,90,180,245,)");  delay(10);
//  //Serial3.println("add_text(5,6,large,C,GR,90,180,245,)");  delay(10);
//  //Serial3.println("add_text(7,6,large,C,UD,90,180,245,)");  delay(10);
//
//  Serial3.println("add_text(14,1,large,C,STOP,100,200,200,R)");  delay(10);
//  Serial3.println("add_text(14,2,large,C,D,200,200,100,D)");  delay(10);
//
//  Serial3.println("add_8way_pad(13,3,1,2,3,4,5,6,7,8,0,100,,)");  delay(10);
//
//  Serial3.println("add_text(1,8,medium,C,10,200,200,200,z)");  delay(10);
//  Serial3.println("add_text(6,8,medium,C,10,200,200,200,Z)");  delay(10);
//
//  Serial3.println("add_gauge(1,7,5,0,100,50,B,,,10,5)");  delay(10);
//  Serial3.println("add_led(8,7,1,L,50,50,50)");  delay(10);
//  Serial3.println("add_button(10,7,15,K,k)");  delay(10);
//
//  //Serial3.println("add_slider(1,2,4,1,10," + hh + ",h,,0)");  delay(10);
//  Serial3.println("add_slider(1,2,4,1,20," + nn + ",n,,0)");  delay(10);
//
//  //Serial3.println("add_switch(3,3,3,Y,y,0,0)");  delay(10);
//  //Serial3.println("add_switch(5,3,3,G,g,0,1)");  delay(10);
//  //Serial3.println("add_switch(7,3,3,A,T,0,1)");  delay(10);
//
//  Serial3.println("set_panel_notes(-,,,)");  delay(10);
//  Serial3.println("run()");  delay(10);
//  Serial3.println("*");
//  delay(20);
//
//  sendToAndroidDevice("N", String(xspd)); delay(10);
//  sendToAndroidDevice("H", String(lift, 0)); delay(10);
//  sendToAndroidDevice("B", String(50)); delay(10);
//  //BTSt = 1;
//}
//
////void led() {
//
////}
//long counter1 = 0;
//void timer() {
//  unsigned long t = millis();
//  if ((t - last_time) > update_interval) {
//    last_time = t;
//    counter1++;
//    if (counter1 % 2 == 0) {
//      teganganBaterai();
//    }
//  }
//}
//
//byte sendStatusSt = 0;
//void bluetoothControl() {
//
//
//  if (Serial3.available()) {
//    data_in = Serial3.read(); //Get next character
//    switch  (data_in) {
//      case '0'://8-way Pad Released
//        pushSt = 0;
//
//        break;
//      case '1'://Up Direction Pressed
//        if (pushSt == 0) {
//          sendToAndroidDevice("D", "Forward"); delay(10);
//          runSt = 1;
//          activeSt = 1;
//          if (oledSt == 1) menuSt = 1001;
//          jumlahLangkah = 0;
//          runtStepNumber = 0;
////          // frameRunning();
//          pushSt = 1;
//        }
//        break;
//
//      case '2'://Diagonal Up Right Direction Pressed
//        if (pushSt == 0) {
//          runSt = 2;
//          activeSt = 1;
//          pushSt = 1;
//          jumlahLangkah = 0;
////          // frameRunning();
//          sendToAndroidDevice("D", "F. Right"); delay(10);
//        }
//        break;
//
//      case '3'://Right Direction Pressed
//        if (pushSt == 0) {
//          pushSt = 1;
//          activeSt = 1;
//          sendToAndroidDevice("D", "Right"); delay(10);
//          runSt = 3;
////          // frameRunning();
//          jumlahLangkah = 0;
//        }
//        break;
//
//      case '4':
//        if (pushSt == 0) {
//          pushSt = 1;
//          sendToAndroidDevice("D", "R. Right"); delay(10);
//        }
//        break;
//
//      case '5'://Down Direction Pressed
//        if (pushSt == 0) {
//          pushSt = 1;
//          activeSt = 1;
//          runSt = 5;
//          jumlahLangkah = 0;
//          sendToAndroidDevice("D", "Reverse"); delay(10);
//          // frameRunning();
//        }
//        break;
//
//      case '6'://Diagonal Down Left Direction Pressed
//        if (pushSt == 0) {
//          pushSt = 1;
//          activeSt = 1;
//          sendToAndroidDevice("D", "R. Left"); delay(10);
//          // frameRunning();
//        }
//        break;
//
//      case '7'://Left Direction Pressed
//        if (pushSt == 0) {
//          pushSt = 1;
//          activeSt = 1;
//          runSt = 7;
//          jumlahLangkah = 0;
//          sendToAndroidDevice("D", "Left"); delay(10);
//          // frameRunning();
//        }
//        break;
//
//      case '8'://Diagonal Up Left Direction Pressed
//        if (pushSt == 0) {
//          runSt = 8;
//          activeSt = 1;
//          pushSt = 1;
//          jumlahLangkah = 0;
//          sendToAndroidDevice("D", "F. Left"); delay(10);
//          // frameRunning();
//        }
//        break;
//
//      case 'c':
//        if (pushSt == 0) {
//          runSt = 9;
//          pushSt = 1;
//          activeSt = 1;
//          directionSt = 9;
//          jumlahLangkah = 0;
//          sendToAndroidDevice("R", "ACTION"); delay(10);
//          // frameRunning();
//        }
//        break;
//
//      case 'o':
//        if (pushSt == 0) {
//          runSt = 10;
//          pushSt = 1;
//          activeSt = 1;
//          jumlahLangkah = 0;
//          directionSt = 10;
//          sendToAndroidDevice("R", "STEP 0"); delay(10);
//          // frameRunning();
//        }
//        break;
//
//      case 'L'://Diagonal Up Left Direction Pressed
//        if (pushSt == 0) {
//          runSt = 11;
//          pushSt = 1;
//          activeSt = 1;
//          Serial.print("runSt ");
//          Serial.println(runSt);
//          jumlahLangkah = 0;
//          sendToAndroidDevice("D", "<<<<<<"); delay(10);
//          // frameRunning();
//        }
//        break;
//
//      case 'R'://Diagonal Up Left Direction Pressed
//        if (pushSt == 0) {
//          runSt = 12;
//          pushSt = 1;
//          activeSt = 1;
//          jumlahLangkah = 0;
//          sendToAndroidDevice("D", ">>>>>>"); delay(10);
//          // frameRunning();
//        }
//        break;
//
//      case 'u':
//        //if (pushSt == 0) {
//        if (langkah < addStep) {
//          langkah++;
//          numberStep += 1;
//        } else {
//          langkah = 1;
//        }
//        moveTo(langkah);
//        //sendToAndroidDevice("n", String(langkah)); delay(10);
//        //}
//        break;
//
//      case 'd'://Diagonal Up Left Direction Pressed
//        //runSt = 13;
//        if (langkah > 1) {
//          numberStep -= 1;
//          langkah--;
//        } else {
//          langkah = addStep;
//        }
//        moveTo(langkah);
//        //sendToAndroidDevice("n", String(langkah)); delay(10);
//        break;
//
//      case 'h':
//        slider_value = Serial3.parseFloat();
//        if (slider_value > 0) {
//          lift = slider_value * 5;
//          sendToAndroidDevice("H", String(lift, 0)); delay(10);
//          calSt = 1;
//          data_in = '-';
//        }
//        break;
//
//      case 'n':
//        slider_value = Serial3.parseFloat();
//        if (slider_value > 0) {
//          xspd = slider_value;
//          spd = 30 - xspd;
//          //spdTemp = spd;
//          sendToAndroidDevice("N", String(xspd)); delay(1);
//          Serial.print("Spd ");
//          Serial.println(spd);
//          data_in = '-';
//        }
//        break;
//      case 'p':
//        if (pushSt == 0) {
//          Oled.clearDisplay();
//          runSt = 99;
//          activeSt = 0;
//          pushSt = 1;
//          sendToAndroidDevice("R", "PAUSE"); delay(10);
//          // frameRunning();
//          //Oled.clearDisplay();
//        }
//        break;
//      case 'r':
//        if (pushSt == 0) {
//          pushSt = 1;
//          activeSt = 1;
//          if (oledSt == 1) menuSt = 1001;
//          //// frameRunning();
//          jumlahLangkah = 0;
//          sendToAndroidDevice("R", "RUN"); delay(10);
//          // frameRunning();
//        }
//        break;
//
//      case 'x':
//        if (pushSt == 0) {
//          pushSt = 1;
//          runSt = 13;
//          activeSt = 0;
//          jumlahLangkah = 0;
//          sendToAndroidDevice("R", "STOP"); delay(10);
//          kipasOff();
//          //// frameRunning();
//          //Oled.clearDisplay();
//        }
//        break;
//
//      case 'K':
//        if (pushSt == 0) {
//          pushSt = 1;
//          activeSt = 1;
//          if (oledSt == 1) menuSt = 1001;
//          jumlahLangkah = 0;
//          runSt = 20;
//          sendToAndroidDevice("R", "FIRE"); delay(10);
//          // frameRunning();
//        }
//        //Serial.println(data_in);
//        break;
//      case 'G':
//        gripperClose();
//        //Serial.println(data_in);
//        break;
//      case 'g':
//        gripperOpen();
//        //Serial.println(data_in);
//        break;
//      case 'A': //naik posisi awal
//        if (gripperSt == 0) gripperClose();
//        angkat();
//        break;
//      case 'T': //turun
//        if (gripperSt == 0) gripperClose();
//        tarok();
//        break;
//
//      case 'Y': //ambil
//        if (UDSt == 0) {
//          for (posGp = 40; posGp <= 90; posGp += 1) {
//            srv_GR.write(posGp);
//            delay(20);
//          }
//          delay(100);
//          srv_UP.write(160);
//          delay(100);
//          srv_GR.write(100);
//          for (posUd = 160; posUd >= 60; posUd -= 1) {
//            srv_UP.write(posUd);
//            delay(25);
//          }
//          gripperSt = 1;
//          UDSt = 2;
//          jumlahLangkah = 0;
//          runSt = 30;
//          // frameRunning();
//          // gripper 95 tutup, 40 buka
//        }
//        break;
//
//      case 'y': // tarok
//        if (UDSt == 2) {
//          for (posUd = 60; posUd <= 170; posUd += 1) {
//            srv_UP.write(posUd);
//            delay(15);
//          }
//        }
//        delay(100);
//        for (posGp = 95; posGp >= 40; posGp -= 1) {
//          srv_GR.write(posGp);
//          delay(15);
//        }
//        UDSt = 0;
//        gripperSt = 0;
//        runSt = 35;
//        if (pushSt == 0) {
//          pushSt = 1;
//          activeSt = 1;
//          jumlahLangkah = 0;
//          if (oledSt == 1) menuSt = 1001;
////          // frameRunning();
//        }
//        break;
//    }
//  }
//}
//
//void sendToAndroidDevice(String d, String t) {
//  //String data
//  Serial.println("*" + d + t + "*");
//  //Serial.println("*" + d + t + "*");
//  delay(10);
//}
