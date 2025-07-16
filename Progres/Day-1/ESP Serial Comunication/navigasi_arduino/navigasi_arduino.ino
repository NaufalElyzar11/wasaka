String messageFromPC = "";
String messageFromESP32 = "";

void setup() {
  Serial.begin(115200);    
  Serial1.begin(115200);   
  Serial.println("Mega chat ready!");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    Serial1.write(c);

    if (c == '\n') {
      Serial.print("You (Mega) sent to ESP32: ");
      Serial.println(messageFromPC);
      messageFromPC = "";
    } else {
      messageFromPC += c;
    }
  }

  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      Serial.print("ESP32 replied: ");
      Serial.println(messageFromESP32);
      messageFromESP32 = "";
    } else {
      messageFromESP32 += c;
    }
  }
}
