#include "Herkulex.h"

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

void setup() {
  // put your setup code here, to run once:
  pinMode(49, OUTPUT);
  digitalWrite(49, 1);
  pinMode(eMagnet, OUTPUT);
  digitalWrite(eMagnet, eMagnetState);
  pinMode(springSwitch, INPUT);
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
}

void loop() {
  // put your main code here, to run repeatedly:
  int reading = digitalRead(springSwitch);
  int proxyRead = analogRead(InductiveProxy);
  if (CheckProxy(proxyRead)) // Pull up weight
  {
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
  if (holding && !reading)
  {
    digitalWrite(eMagnet, 0);
    holding = 0;
    Herkulex.moveOneAngle(motorA, 159, 1000, LED_GREEN); //move motorA backward
    Herkulex.moveOneAngle(motorB, -159, 1000, LED_GREEN); //move motorB forward
    delay(1500);
    Herkulex.moveOneAngle(motorA, -159, 1000, LED_CYAN); //move motorA backward
    Herkulex.moveOneAngle(motorB, 159, 1000, LED_CYAN); //move motorB forward
    delay(1500);    
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

