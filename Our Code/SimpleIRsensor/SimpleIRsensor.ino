
#define ALPHA 0.9

// Calibrating the four IR sensors to give an analog read out in mm

int irPinRightHigh = 0; //analog pin 0
int irPinLeftHigh = 2; //analog pin 2
int irPinRightLow = 4; //analog pin 4
int irPinLeftLow = 6; //analog pin 6

int distLeftHigh;
int distRightHigh;
int distLeftLow;
int distRightLow;

void setup(){
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  Serial.begin(9600);
}

void loop(){
  distLeftHigh =  analogRead(irPinLeftHigh);
  distRightHigh = analogRead(irPinRightHigh);
 // distLeftLow = first_order_recursive_filter(analogRead(irPinLeftLow),distLeftLow);
//  distRightLow = first_order_recursive_filter(analogRead(irPinRightLow), distRightLow);

  Serial.print(50000/distLeftHigh);
  Serial.print(" ");
  Serial.print(50000/distRightHigh);
//  Serial.print(" ");
//  Serial.print(23000/distLeftLow);
//  Serial.print(" ");
//  Serial.print(50000/distRightLow);
  Serial.println();
  
  //just to slow down the output - remove if trying to catch an object passing by
  delay(20);

}

double first_order_recursive_filter(int value, int previous) {
  return previous * ALPHA + value * (1 - ALPHA);
}

