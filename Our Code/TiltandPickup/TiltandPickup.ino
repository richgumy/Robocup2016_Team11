#include "Herkulex.h"
#include <Servo.h>

int motorA=0xfd; //motor ID - verify your ID !!!!
int motorB=0x02;

const int eMagnet = 33;
const int springSwitch = 34;
const int InductiveProxy = A8;

static int eMagnetState = LOW;
static int SwitchState;             // the current reading from the input pin
static int lastSwitchState = LOW;   // the previous reading from the input pin
int holding = 0;
int tick = 0;

static long lastDebounceTime = 0;  // the last time the output pin was toggled
static long debounceDelay = 50;    // the debounce time; increase if the output flickers

Servo myservoR;      // create servo object to control a servo
Servo myservoL;      // create servo object to control a servo

int servoR = 4;
int servoL = 5;

void setup() {
  // put your setup code here, to run once:
  pinMode(49, OUTPUT);
  digitalWrite(49, 1);
  pinMode(eMagnet, OUTPUT);
  digitalWrite(eMagnet, eMagnetState);
  pinMode(springSwitch, INPUT);
  pinMode(servoR, OUTPUT); 
  pinMode(servoL, OUTPUT);
  myservoR.attach(servoR);
  myservoL.attach(servoL);
//  pinMode(InductiveProxy, INPUT);
  Herkulex.beginSerial2(115200);       //open serial port 2 to talk to the motors
  Herkulex.reboot(motorA);             //reboot first motor
  Herkulex.reboot(motorB);             //reboot second motor
  delay(500);
  Herkulex.initialize();               //initialize motors
  delay(200); 
  Herkulex.moveOneAngle(motorA, -159, 1000, LED_BLUE); //move motorA backward
  Herkulex.moveOneAngle(motorB, 159, 1000, LED_BLUE); //move motorB forward
  delay(1500);
  Serial.begin(9600);
  myservoR.writeMicroseconds(700);
  myservoL.writeMicroseconds(2300);
}

void loop() {

  // put your main code here, to run repeatedly:
  int reading = digitalRead(springSwitch);
  int proxyRead = analogRead(InductiveProxy);
  if (CheckProxy(proxyRead)) // Pull up weight
  {
    Herkulex.initialize();               //initialize motors
    digitalWrite(eMagnet, 1);
    holding = 1;
    delay(20);
    // Lower crane and with eMagnet turned ON
    Herkulex.moveOneAngle(motorA, 159, 1000, LED_GREEN); //move motorA backward
    Herkulex.moveOneAngle(motorB, -159, 1000, LED_GREEN); //move motorB forward
    delay(1500);
    // Pick up weight and with eMagnet turned ON
    Herkulex.moveOneAngle(motorA, -159, 1000, LED_BLUE); //move motorA backward
    Herkulex.moveOneAngle(motorB, 159, 1000, LED_BLUE); //move motorB forward
    delay(2000);
  }
  if (holding)
  {
    delay(500);
    digitalWrite(eMagnet, 0);
    holding = 0;
    Herkulex.moveOneAngle(motorA, 159, 1000, LED_GREEN); //move motorA backward
    Herkulex.moveOneAngle(motorB, -159, 1000, LED_GREEN); //move motorB forward
    delay(1500);
    Herkulex.moveOneAngle(motorA, -159, 1000, LED_CYAN); //move motorA backward
    Herkulex.moveOneAngle(motorB, 159, 1000, LED_CYAN); //move motorB forward
    delay(2000);
    wiggle();
    //exit  
  }
  Serial.print(reading);
//  Serial.print(" -> ");
//  Serial.print(eMagnetState);
//  Serial.print(" -> ");
//  Serial.print(proxyRead);
//  Serial.print(" -> ");
//  Serial.print(holding);
  Serial.println();
}

int CheckProxy(int proxyRead)
{
  int detected;
  if (proxyRead > 500)
  {
    detected = 1;
  }
  else
  {
    detected = 0;
  }
  return (detected);
}

void wiggle(void)
{
    myservoR.writeMicroseconds(2300);
    myservoL.writeMicroseconds(700);
    // ^storage tray is set to a steep gradient
    for (int wiggles = 0 ; wiggles < 8 ; wiggles++)
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

