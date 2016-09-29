#include <Servo.h>

Servo myservoR;      // create servo object to control a servo
Servo myservoL;      // create servo object to control a servo

int ProxSensor = A8;
int On = 0;
int servoR = 4;
int servoL = 5;

void setup()
{ 
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  pinMode(servoR, OUTPUT); 
  pinMode(servoL, OUTPUT);
  pinMode(ProxSensor, INPUT);
  myservoR.attach(servoR);
  myservoL.attach(servoL);
  Serial.begin(9600);
}

void loop() 
{ 
  On = analogRead(ProxSensor);
  Serial.print(On);
  Serial.println();
  myservoR.writeMicroseconds(700);
  myservoL.writeMicroseconds(2300);
  // storage tray is set to a flat gradient
  if (On)
  {
    myservoR.writeMicroseconds(2300);
    myservoL.writeMicroseconds(700);
    // ^storage tray is set to a steep gradient
    for (int wiggles = 0 ; wiggles < 4 ; wiggles++)
    {
      myservoR.writeMicroseconds(2000);
      myservoL.writeMicroseconds(1000);
      delay(200);
      myservoR.writeMicroseconds(1800);
      myservoL.writeMicroseconds(1200);
      delay(200);
    }
    // ^storage tray is then 'wiggled' theatrically
    myservoR.writeMicroseconds(700);
    myservoL.writeMicroseconds(2300);
    // ^storage tray is finally restored to flat gradient
  }
} 



