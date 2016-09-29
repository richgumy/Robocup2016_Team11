
#include <NewPing.h>

int maxDistance = 200;

#define ALPHA 0.9

// sonar(trigPin, echoPin, maxDistance)
NewPing sonarTopLeft(35, 34, maxDistance);
NewPing sonarTopRight(36, 37, maxDistance);
NewPing sonarBottomLeft(39, 38, maxDistance);
NewPing sonarBottomRight(40, 41, maxDistance);

int distanceTopLeft;
int distanceTopRight;
int distanceBottomLeft;
int distanceBottomRight;

void setup() {
  // put your setup code here, to run once:
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  CalculateDistances();
  
  Serial.print(distanceTopLeft);
  Serial.print(" ");
  Serial.print(distanceBottomLeft);
  Serial.print(" | ");
  Serial.print(distanceTopRight);
  Serial.print(" ");
  Serial.print(distanceBottomRight);
  Serial.println();

  delay(20);
}


void CalculateDistances() {
  
  distanceTopLeft = first_order_recursive_filter(SonarSense(sonarTopLeft), distanceTopLeft);
  distanceTopRight = first_order_recursive_filter(SonarSense(sonarTopRight), distanceTopRight);
  distanceBottomLeft = first_order_recursive_filter(SonarSense(sonarBottomLeft), distanceBottomLeft);
  distanceBottomRight = first_order_recursive_filter(SonarSense(sonarBottomRight), distanceBottomRight);

//  distanceLeft = IRSense(leftIRPin, leftIRConstant);
//  distanceRight = IRSense(rightIRPin, rightIRConstant);
//  
//  distanceFrontLeft = IRSense(frontRightIRPin, frontRightIRConstant);
//  distanceFrontRight = IRSense(frontLeftIRPin, frontLeftIRConstant);
}

double first_order_recursive_filter(int value, int previous) {
  return previous * ALPHA + value * (1 - ALPHA);
}

int SonarSense(NewPing sonar) {
  int value = sonar.convert_cm(sonar.ping());
  if (value == 0) {
    //value = maxDistance;
  }
  return (value);
}

