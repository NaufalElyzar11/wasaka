#include <WiFi.h>
#include <WebServer.h>

// Buat wifi nya
const char* ssid = "Zzz";
const char* password = "kandangan";

// Buat instance WebServer di port 80
WebServer server(80);

// Pin untuk Motor Kiri
const int motor1_in1 = 5;
const int motor1_in2 = 18;
const int motor1_ena = 22;

// Pin untuk Motor Kanan
const int motor2_in3 = 19;
const int motor2_in4 = 21;
const int motor2_enb = 23;

// Pengaturan PWM kd dipakai
// const int freq = 5000;
// const int ledcChannel1 = 0;
// const int ledcChannel2 = 1;
// const int resolution = 8;

void setup() {
  Serial.begin(115200);

  // Set semua pin motor sebagai OUTPUT
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor2_in3, OUTPUT);
  pinMode(motor2_in4, OUTPUT);
  pinMode(motor1_ena, OUTPUT); 
  pinMode(motor2_enb, OUTPUT); 

  // Konfigurasi PWM kd dipakai
  // ledcSetup(ledcChannel1, freq, resolution);
  // ledcSetup(ledcChannel2, freq, resolution);
  // ledcAttachPin(motor1_ena, ledcChannel1);
  // ledcAttachPin(motor2_enb, ledcChannel2);

  // Inisialisasi Wi-Fi sebagai Access Point
  Serial.print("Membuat Access Point: ");
  Serial.println(ssid);
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  // Definisi handler untuk URL
  server.on("/control", handleControl);

  // Mulai server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

// Fungsi untuk mengontrol motor (tanpa kecepatan)
void moveMotors(int x, int y) {
  // y > 600 -> Maju
  // y < 400 -> Mundur
  // x > 600 -> Belok Kanan
  // x < 400 -> Belok Kiri

  if (y > 600) { // Perintah Maju
    if (x > 600) {
      belokKanan();
    } else if (x < 400) {
      belokKiri();
    } else {
      maju();
    }
  } else if (y < 400) { // Perintah Mundur
    mundur();
  } else { // Perintah Berhenti
    berhenti();
  }
}

// Fungsi untuk menangani request di "/control"
void handleControl() {
  int joyX = 512;
  int joyY = 512;

  if (server.hasArg("x")) {
    joyX = server.arg("x").toInt();
  }
  if (server.hasArg("y")) {
    joyY = server.arg("y").toInt();
  }

  Serial.print("Menerima data -> X: ");
  Serial.print(joyX);
  Serial.print(" | Y: ");
  Serial.println(joyY);

  moveMotors(joyX, joyY);

  server.send(200, "text/plain", "OK");
}

void maju() {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in3, HIGH);
  digitalWrite(motor2_in4, LOW);
  digitalWrite(motor1_ena, HIGH); 
  digitalWrite(motor2_enb, HIGH); 
}

void mundur() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, HIGH);
  digitalWrite(motor2_in3, LOW);
  digitalWrite(motor2_in4, HIGH);
  digitalWrite(motor1_ena, HIGH); 
  digitalWrite(motor2_enb, HIGH); 
}

void belokKanan() {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in3, LOW); 
  digitalWrite(motor2_in4, HIGH);
  digitalWrite(motor1_ena, HIGH);
  digitalWrite(motor2_enb, HIGH);
}

void belokKiri() {
  digitalWrite(motor1_in1, LOW);  
  digitalWrite(motor1_in2, HIGH);
  digitalWrite(motor2_in3, HIGH);
  digitalWrite(motor2_in4, LOW);
  digitalWrite(motor1_ena, HIGH); 
  digitalWrite(motor2_enb, HIGH); 
}

void berhenti() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in3, LOW);
  digitalWrite(motor2_in4, LOW);
  digitalWrite(motor1_ena, LOW); 
  digitalWrite(motor2_enb, LOW); 
}