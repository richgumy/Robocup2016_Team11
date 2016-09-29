#include <NewPing.h>
 
#define TRIGGER_PIN  20
#define ECHO_PIN     21
#define MAX_DISTANCE 300
 
 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
 
void setup() {
  Serial.begin(9600);
}
 
void loop() {

  delay(150);
  int uS = sonar.ping();
  if (uS==0)
  {
    Serial.println("MAX: resetting sensor");
    pinMode(ECHO_PIN, OUTPUT);
    delay(150);
    digitalWrite(ECHO_PIN, LOW);
    delay(150);
    pinMode(ECHO_PIN, INPUT);
    delay(150);
  }
  else
  {
  Serial.print(" ");
  Serial.print("Ping: ");
  Serial.print(uS / US_ROUNDTRIP_CM);
  Serial.println("cm");
  }
}
