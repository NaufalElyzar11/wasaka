String messageFromPC = "";
String messageFromMega = "";

void setup() {
  Serial.begin(115200);         
  Serial1.begin(115200, SERIAL_8N1, 16, 17); 

  Serial.println("ESP32 chat ready!");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    Serial1.write(c);

    if (c == '\n') {
      Serial.print("You (ESP32) sent to Mega: ");
      Serial.println(messageFromPC);
      messageFromPC = "";
    } else {
      messageFromPC += c;
    }
  }

  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);

    if (c == '\n') {
      Serial.print("Mega replied: ");
      Serial.println(messageFromMega);
      messageFromMega = "";
    } else {
      messageFromMega += c;
    }
  }
}