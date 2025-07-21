#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

// Konfigurasi Wi-Fi, sesuai akn dgn yg di receivernya
const char* ssid = "Zzz";      
const char* password = "kandangan";  

// Alamat IP server (ESP32). Default IP untuk Soft AP adalah 192.168.4.1
const char* serverIP = "192.168.4.1";

// Pin joystick
const int joyY_pin = A0; // Sumbu Y untuk maju/mundur

void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println();
  Serial.print("Menghubungkan ke ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi terhubung");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Baca nilai joystick Y (0-1023)
  int joyY_val = analogRead(joyY_pin);
  // Disini cuma ngirim Y, jadi X nya normal aja
  int joyX_val = 512; 

  Serial.print("Mengirim Y: ");
  Serial.println(joyY_val);

  // Buat objek HTTP client
  WiFiClient client;
  HTTPClient http;

  // Buat URL dengan data joystick
  String url = "http://";
  url += serverIP;
  url += "/control?x=";
  url += joyX_val;
  url += "&y=";
  url += joyY_val;

  // Kirim HTTP GET request
  if (http.begin(client, url)) {
    int httpCode = http.GET();
    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        // Serial.println(payload); // Optional: cetak balasan server
      }
    } else {
      Serial.printf("[HTTP] GET... gagal, error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
  } else {
    Serial.printf("[HTTP] Tidak dapat terhubung\n");
  }

  delay(100); 
}