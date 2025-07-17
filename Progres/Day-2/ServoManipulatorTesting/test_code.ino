// Library Servo 
#include <Servo.h>

// Inisialisasi Objek Servo
Servo myservo;

// Variabel Derajat Posisi Saat Ini
int currentPos = 90;

// Inisialisasi Servo Derajat 90
void setup() {
  Serial.begin(9600);

  // Menyambungkan myservo dengan pin 9
  myservo.attach(9);
  
  // Mengubah derajat servo menjadi 90 derajat
  myservo.write(currentPos);

  // UI
  Serial.println("--- Servo Control Ready ---");
  Serial.println("Enter a target angle (0-180) and press Send.");
}

void loop() {
  // Cek jika ada nput yang dapat diolah
  if (Serial.available() > 0) {
    // Membaca Input Integer dari User
    int targetPos = Serial.parseInt();

    // Untuk menghilangkan semua input data yang tidak relevan
    while(Serial.available() > 0) {
      Serial.read();
    }

    // UI
    Serial.print("New target received: ");
    Serial.println(targetPos);

    // constrain untuk membatasi input dari user antara 0 dan 180 saja
    targetPos = constrain(targetPos, 0, 180);

    // UI
    Serial.print("Sweeping from ");
    Serial.print(currentPos);
    Serial.print(" degrees to ");
    Serial.print(targetPos);
    Serial.println(" degrees...");

    // Sweep
    if (currentPos < targetPos) {
      for (int pos = currentPos; pos <= targetPos; pos++) {
        myservo.write(pos); 
        delay(15);          
      }
    } 
    else if (currentPos > targetPos) {
      for (int pos = currentPos; pos >= targetPos; pos--) {
        myservo.write(pos); 
        delay(15);          
      }
    }
    
    // Mengubah data variable currentPos(posisi saat ini) menjadi targetPos(posisi yang diinginkan)
    currentPos = targetPos;
    
    // UI
    Serial.println("Sweep complete. Ready for a new command.");
    Serial.println("--------------------------------------");
  }
}
