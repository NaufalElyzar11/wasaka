#include <NewPing.h>

#define SONAR_NUM     4 // Number or sensors.
#define MAX_DISTANCE 300 // Max distance in cm.
#define PING_INTERVAL 100 // Milliseconds between pings.
 
unsigned long pingTimer[SONAR_NUM]; // When each pings.
unsigned int cm[SONAR_NUM]; // Store ping distances.
uint8_t currentSensor = 0; // Which sensor is active.
 
NewPing sonar[SONAR_NUM] = { // Sensor object array.
  NewPing(22, 23, MAX_DISTANCE), //depan
  NewPing(50, 51, MAX_DISTANCE), //kanan depan
  NewPing(24, 25, MAX_DISTANCE), //kiri depan
//  NewPing(48, 49, MAX_DISTANCE), //kanan belakang
//  NewPing(26, 27, MAX_DISTANCE), //kiri belakang
  NewPing(28, 29, MAX_DISTANCE), //belakang
//  NewPing(30, 31, MAX_DISTANCE) //capit
};

void setup() {
  Serial.begin(9600);
  pingTimer[0] = millis() + 75; // First ping start in ms.
  for (uint8_t i = 1; i < SONAR_NUM; i++)
  pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1)
         oneSensorCycle(); // Do something with results.
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
  // The rest of your code would go here.
}
void echoCheck() { // If ping echo, set distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}
