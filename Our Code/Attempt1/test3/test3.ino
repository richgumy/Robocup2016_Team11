#include <Servo.h>
#include <NewPing.h>
#include "Herkulex.h"// include the library code:
#include <LiquidCrystal.h>



#define NOTE_G3  196
#define ULONG_MAX 4294967295

#define ALPHA 0.9

enum Direction {Left, Right};

enum GlobalState {NoState, Searching, Gathering, PickUp};
enum MovementState {NoMove, MovingStraight, TurningLeft, TurningRight, FollowingLeftWall, FollowingRightWall, Reversing};
enum GatheringState {NoGather, Turning, Approaching, Honing, Scouting, Collecting, Lost};

const char* globalStateNames[] = {"N", "S", "G", "P"};
const char* movementStateNames[] = {"N ", "MS", "FW", "TL", "TR", "FL", "FR", "R "};
const char* gatheringStateNames[] = {"N ", "T ", "A ", "H ", "S ", "C ", "L "};

GlobalState globalState = Searching;
MovementState movementState = NoMove;
GatheringState gatheringState = NoGather;

Servo motorRight;      // create servo object to control the right DC motor
Servo motorLeft;      // create servo object to control the left DC motor

// create servo objects to control the right & left storage tray tilting servo
Servo StorageServoR;
Servo StorageServoL;

// Set storage tray servo pins
int servoR = 4;
int servoL = 5;

int speaker = 10;

int debugDisplayDelay = 2000;

int moveSpeed = 150;
int motorStopValue = 1500;

int maxDistance = 60;

int distanceTopLeft;
int distanceTopRight;
int distanceBottomLeft;
int distanceBottomRight;

int distanceLeft;
int distanceRight;

int distanceFrontLeft;
int distanceFrontRight;

int leftIRPin = 0;
int rightIRPin = 2;
int frontLeftIRPin = 3;
int frontRightIRPin = 1;

int leftIRConstant = 3000;
int rightIRConstant = 3000;
int frontLeftIRConstant = 14000;
int frontRightIRConstant = 14000;

float wallFollowDistance = 20;
int stoppingDistance = 45;
int wallProximityDistance = 30;

unsigned long turnTimer;
bool turnCompleted;
int turnConstant = 10;

unsigned long moveTimer;
bool moveCompleted;

int checkStopLeft1;
int checkStopRight1;
int checkStopLeft2;
int checkStopRight2;

int stopReverseDistance = 55;

int reverseCount;
bool canReverseCount = true;

int sensorNoiseOverride = 2;

Direction packageDirection;
int packageDistance;

int packageCheckingTolerance = 15;

float packageAngleDistanceRatio = 0.5;

int packageCollectMoveTime = 800;
int packageApproachConstant = 50;
int packageCloseDistance = 15;

int checkPackageLeftNoiseOverride;
int checkPackageRightNoiseOverride;

unsigned long gatherWatchDogTimer;
int gatherWatchDogPeriod = 10000;

unsigned long displayTimer;
int displayTimerPeriod = 300;

// sonar(trigPin, echoPin, maxDistance)
NewPing sonarTopLeft(44, 45, maxDistance);
NewPing sonarTopRight(39, 38, maxDistance);
NewPing sonarBottomLeft(43, 42, maxDistance);
NewPing sonarBottomRight(40, 41, maxDistance);

/////////////////////////////////


int motorA=0xfd; //motor ID - verify your ID !!!!
int motorB=0x02;

const int eMagnet = 33;
const int springSwitch = 34;
const int InductiveProxy = 8;

static int eMagnetState = LOW;
static int SwitchState;             // the current reading from the input pin
static int lastSwitchState = LOW;   // the previous reading from the input pin
int holding = 0;
int tick = 0;

static long lastDebounceTime = 0;  // the last time the output pin was toggled
static long debounceDelay = 50;    // the debounce time; increase if the output flickers

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(31, 32, 35, 36, 37, 30);


void setup(){
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  Serial.begin(9600);
  //attach the servo pin 3 & 2 to the right & left DC drive motors
  motorRight.attach(3);
  motorLeft.attach(2);
  //attach the servo pin 4 & 5 to the right & left storage tray servos
  StorageServoR.attach(servoR);  // attaches the servo  to the servo object useing pin 33
  StorageServoL.attach(servoL);  // attaches the servo  to the servo object useing pin 32
  
  lcd.begin(16, 2);
//  lcd.print("init");
  
  ///////////////////////////////
  //// Pickup stuff
  setPwmFrequency(speaker, 512);
  pinMode(eMagnet, OUTPUT);
  digitalWrite(eMagnet, LOW);
  pinMode(springSwitch, INPUT);
  pinMode(InductiveProxy, INPUT);
//  Herkulex.beginSerial2(115200);       //open serial port 2 to talk to the motors
  Herkulex.reboot(motorA);             //reboot first motor
  Herkulex.reboot(motorB);             //reboot second motor
  delay(500);
  Herkulex.initialize();               //initialize motors
  delay(200); 
  Herkulex.moveOneAngle(motorA, 159, 1000, LED_GREEN); //move motorA backward
  Herkulex.moveOneAngle(motorB, -159, 1000, LED_GREEN); //move motorB forward
  delay(2000);
  Herkulex.moveOneAngle(motorA, -100, 1000, LED_BLUE); //move motorA backward
  Herkulex.moveOneAngle(motorB, 100, 1000, LED_BLUE); //move motorB forward
  delay(1500);

  /////////////////////////////

  ResumeDefaultState();
}

bool test = true;

void TestTurn() {
  

  int proxyRead = analogRead(InductiveProxy);
  lcd.setCursor(0, 0);
  lcd.print("    ");
  lcd.setCursor(0, 0);
  lcd.print(turnCompleted);
  if (CheckProxy(proxyRead)) {
    TurnByAngle(Right, 90);
  }
  Serial.print(millis());
  Serial.println(turnCompleted);
  if (turnCompleted) {
    if (movementState == TurningRight) {
      InterruptTurn();
      delay(1000);
      TurnByAngle(Left,90);
    } else {
      InterruptTurn();
    }
  }
}

void TestMove() {
  
  int proxyRead = analogRead(InductiveProxy);

  if (CheckProxy(proxyRead)) {
    Serial.println("touch");
    MoveForTime(20 * packageApproachConstant);
  }

  if (moveCompleted) {
    InterruptMove();
  }
}

void loop(){
  
  CalculateDistances();

  //TestTurn();
  //TestMove();

//  motorRight.writeMicroseconds(motorStopValue + -1 * moveSpeed);
//  motorLeft.writeMicroseconds(motorStopValue + -1 * moveSpeed);
  
  
  if (globalState == Searching) {
    Search();
  }
  if (globalState == Gathering) {
    Gather();
  }
  
  CheckTurn();
  CheckMove();
  

  PickUpStuff();
  
//  Serial.print(distanceTopLeft);
//  Serial.print(" ");
//  Serial.print(distanceBottomLeft);
//  Serial.print(" | ");
//  Serial.print(distanceTopRight);
//  Serial.print(" ");
//  Serial.print(distanceBottomRight);
//  Serial.println();
//  Serial.print(" | ");
//  Serial.print(" L ");
//  Serial.print(distanceLeft);
//  Serial.print(" R ");
//  Serial.print(distanceRight);
//  Serial.println();
//  Serial.print(" FL ");
//  Serial.print(distanceFrontLeft);
//  Serial.print(" FR ");
//  Serial.print(distanceFrontRight);
//  Serial.println();


  UpdateDisplayCounter();

  
  delay(20);
}

void Search() {

  if (CheckPackage(Left)) {
    PackageFound(Left);
  }
  if (CheckPackage(Right)) {
    PackageFound(Right);
  }

  
  if (movementState == MovingStraight) {
    MoveStraight();
  }

  if (movementState != Reversing) {
    if (distanceLeft < stoppingDistance) {
      checkStopLeft1++;
      Serial.print(checkStopLeft1);
      Serial.println();
      if (checkStopLeft1 > sensorNoiseOverride) {
        movementState = FollowingLeftWall;
        checkStopLeft1 = 0;
        Serial.print("Following left wall");
        Serial.println();
      }
    } else {
      checkStopLeft1 = 0;
    }
      
    if (distanceRight < stoppingDistance) {
      checkStopRight1++;
      if (checkStopRight1 > sensorNoiseOverride) {
        movementState = FollowingRightWall;
        checkStopRight1 = 0;
        Serial.print("Following right wall");
        Serial.println();
      }
    } else {
      checkStopRight1 = 0; 
    }
  }

  if (movementState == FollowingLeftWall) {
    FollowWall(Left);
  }

  if (movementState == FollowingRightWall) {
    FollowWall(Right);
  }

  if (movementState != TurningLeft && movementState != TurningRight) {
    if (distanceFrontRight < startTurnDistance) {
      checkStopLeft2++;
      if (checkStopLeft2 > sensorNoiseOverride) {
        if (distanceLeft > wallProximityDistance) {
          TurnByAngle(Left, min((distanceFrontLeft - distanceFrontRight) * 150, 90));
        } else {
          TurnByAngle(Right, 90);
        }
        Serial.print("Turn left");
        Serial.println();
        checkStopLeft2 = 0;
      }
    } else {
      checkStopLeft2 = 0;
    }
    
    if (distanceFrontLeft < startTurnDistance) {
      checkStopRight2++;
      if (checkStopRight2 > sensorNoiseOverride) {
        if (distanceRight > wallProximityDistance) {
          TurnByAngle(Right, min((distanceFrontRight - distanceFrontLeft) * 150, 90));
        } else {
          TurnByAngle(Left, 90);
        }
        Serial.print("Turn right");
        Serial.println();
        checkStopRight2 = 0;
      }
    } else {
      checkStopRight2 = 0;
    }
  }

  if (turnCompleted) {
    
    Serial.print("Turn complete");
    Serial.println();
    
    movementState = MovingStraight;
    //Finding wall
    turnCompleted = false;
  }

  CheckTurn();
  //CheckReverse();
}

void PackageFound(Direction sensor) {
  globalState = Gathering;

  if (sensor == Left) {
    packageDirection = Right;
    packageDistance = distanceBottomLeft;
    if (packageDistance < packageCanApproachDistance) {
      MoveForTime (packageCollectMoveTime);
      gatheringState = Approaching;
    } else {
      TurnByAngle(Right, 45);
      gatheringState = Turning;
    }

  Serial.print("Package found (left sensor)");
  Serial.println();
  } else {
    packageDirection = Left;
    packageDistance = distanceBottomRight;
    if (packageDistance < packageCanApproachDistance) {
      MoveForTime (packageCollectMoveTime);
      gatheringState = Approaching;
    } else {
      TurnByAngle(Left, 45);
      gatheringState = Turning;
    }

  Serial.print("Package found (right sensor)");
  Serial.println();
  }


  gatherWatchDogTimer = millis() + gatherWatchDogPeriod;

  UpdateDisplay();

  delay(1000);
}

void Gather() {

  motorRight.writeMicroseconds(motorStopValue);
  motorLeft.writeMicroseconds(motorStopValue);

  
  if (millis() > gatherWatchDogTimer) {
    ResumeDefaultState();
    gatherWatchDogTimer = ULONG_MAX;
  }

  if (gatheringState == Turning) {
    if (CheckPackage(Left) && CheckPackage(Right)) {
      InterruptTurn();
      
      MoveForTime (packageCollectMoveTime);
      gatheringState = Collecting;

      UpdateDisplay();
      lcd.setCursor(3, 1);
      lcd.print("       ");
      lcd.setCursor(3, 1);
      lcd.print("trnt PM");
      delay(debugDisplayDelay);
      
      Serial.print("Turnt, package in middle");
      Serial.println();

    } else if (movementState == TurningLeft && CheckPackage(Left)) {
      if (distanceBottomLeft < packageCloseDistance) {
        InterruptTurn();
        
        float angle = (float)distanceBottomLeft * packageAngleDistanceRatio;
        TurnByAngle(Left, angle);
        gatheringState = Honing;
  
        UpdateDisplay();
        lcd.setCursor(3, 1);
        lcd.print("       ");
        lcd.setCursor(3, 1);
        lcd.print("trnt PL");
        delay(debugDisplayDelay);
  
        Serial.print("Turnt, package is to the left, now honing");
        Serial.println();
  
        // Turnt, package is to the left, now honing
      }
      
    } else if (movementState == TurningRight && CheckPackage(Right)) {
      if (distanceBottomright < packageCloseDistance) {
        InterruptTurn();
        
        float angle = (float)distanceBottomRight * packageAngleDistanceRatio;
        TurnByAngle(Right, angle);
        gatheringState = Honing;
  
        UpdateDisplay();
        lcd.setCursor(3, 1);
        lcd.print("       ");
        lcd.setCursor(3, 1);
        lcd.print("trnt PR");
        delay(debugDisplayDelay);
  
  
        Serial.print("Turnt, package is to the right, now honing");
        Serial.println();
  
        // Turnt, package is to the right, now honing
      }
    } else if (turnCompleted) {
      InterruptMove();
      gatheringState = Approaching;

      MoveForTime (packageDistance * packageApproachConstant);

      turnCompleted = false;

      UpdateDisplay();
      lcd.setCursor(3, 1);
      lcd.print("       ");
      lcd.setCursor(3, 1);
      lcd.print("trnt A");
      delay(debugDisplayDelay);

      Serial.print("Turnt, now approaching");
      Serial.println();

      // Turnt, now approaching
    }
  }

  if (gatheringState == Approaching) {
    if (CheckPackage(Left) && CheckPackage(Right)) {
      InterruptMove();
      
      MoveForTime(packageCollectMoveTime);
      gatheringState = Collecting;

      UpdateDisplay();
      lcd.setCursor(3, 1);
      lcd.print("       ");
      lcd.setCursor(3, 1);
      lcd.print("aprd PR");
      delay(debugDisplayDelay);


      Serial.print("Approached, package is in the middle");
      Serial.println();
      
      // Approached, package is in the middle
    }
  } else if (CheckPackage(Left)) {
    if (distanceBottomLeft < packageCloseDistance) {
      InterruptMove();

      packageDistance = distanceBottomLeft;
      float angle = (float)distanceBottomLeft * packageAngleDistanceRatio;
      TurnByAngle(Left, angle);
      gatheringState = Honing;

      UpdateDisplay();
      lcd.setCursor(3, 1);
      lcd.print("       ");
      lcd.setCursor(3, 1);
      lcd.print("aprd PL");
      delay(debugDisplayDelay);


      Serial.print("Approached, package is to the left");
      Serial.println();

      //Approached, package is to the left
    }
  } else if (CheckPackage(Right)) {
    if (distanceBottomRight < packageCloseDistance) {
      InterruptMove();
      
      packageDistance = distanceBottomRight;
      float angle = (float)distanceBottomRight * packageAngleDistanceRatio;
      TurnByAngle(Right, angle);
      gatheringState = Honing;

      UpdateDisplay();
      lcd.setCursor(3, 1);
      lcd.print("       ");
      lcd.setCursor(3, 1);
      lcd.print("aprd PR");
      delay(debugDisplayDelay);


      Serial.print("Approached, package is to the right");
      Serial.println();
      
      //Approached, package is to the right
    }
  } else if (moveCompleted) {
      gatheringState = Lost;
      moveCompleted = false;
      
      UpdateDisplay();
      lcd.setCursor(3, 1);
      lcd.print("       ");
      lcd.setCursor(3, 1);
      lcd.print("aprd er");
      delay(debugDisplayDelay);

      Serial.print("Tried approaching, now lost");
      Serial.println();

    // Tried approaching, now lost
  }

  if (gatheringState == Honing) {
    if (CheckPackage(Left) && CheckPackage(Right)) {
      InterruptTurn();
      
      MoveForTime(packageCollectMoveTime);
      gatheringState = Collecting;
      
      UpdateDisplay();
      lcd.setCursor(3, 1);
      lcd.print("       ");
      lcd.setCursor(3, 1);
      lcd.print("hnd PM");
      delay(debugDisplayDelay);


      Serial.print("Honed, package is in the middle");
      Serial.println();

      // Honed, package is in the middle
      
    } else if (CheckPackage(Left) && movementState != TurningLeft) {
      if (distanceBottomLeft < packageCollectDistance) {
        InterruptTurn();
        
        MoveForTime(packageCollectMoveTime);
        gatheringState = Collecting;
      } else {
        InterruptMove();
        
        packageDistance = distanceBottomLeft;
        float angle = (float)packageDistance * packageAngleDistanceRatio;
        TurnByAngle(Left, angle);
        
        UpdateDisplay();
        lcd.setCursor(3, 1);
        lcd.print("       ");
        lcd.setCursor(3, 1);
        lcd.print("hnd PL");
        delay(debugDisplayDelay);
  
        Serial.print("Honed, package is to the left");
        Serial.println();
  
        // Honed, package is to the left
      }
      
    } else if (CheckPackage(Right) && movementState != TurningRight) {
      if (distanceBottomRight < packageCollectDistance) {
        InterruptTurn();
        
        MoveForTime(packageCollectMoveTime);
        gatheringState = Collecting;
      } else {
        InterruptMove();
        
        packageDistance = distanceBottomRight;
        float angle = (float)packageDistance * packageAngleDistanceRatio;
        TurnByAngle(Right, angle);
        
        UpdateDisplay();
        lcd.setCursor(3, 1);
        lcd.print("       ");
        lcd.setCursor(3, 1);
        lcd.print("hnd PR");
        delay(debugDisplayDelay);
  
  
        Serial.print("Honed, package is to the right");
        Serial.println();
  
        // Honed, package is to the right
      }
    } else if (turnCompleted) {
      MoveForTime(packageDistance * packageApproachConstant);
      turnCompleted = false;

      
      UpdateDisplay();
      lcd.setCursor(3, 1);
      lcd.print("       ");
      lcd.setCursor(3, 1);
      lcd.print("h t af");
      delay(debugDisplayDelay);

      
      Serial.print("Honed, turnt, approaching further");
      Serial.println();

      // Honed, turnt, approaching further
    } else if (moveCompleted) {
      gatheringState = Lost;
      moveCompleted = false;
    }
  }

  if (gatheringState == Collecting) {
    if (moveCompleted) {
      gatheringState = Lost;

      moveCompleted = false;
      
      UpdateDisplay();
      lcd.setCursor(3, 1);
      lcd.print("       ");
      lcd.setCursor(3, 1);
      lcd.print("clt er");
      delay(debugDisplayDelay);

      Serial.print("Tried to collect, couldn't find anything, lost");
      Serial.println();

      // Tried to collect, couldn't find anything, lost
    }
  }

  if (gatheringState == Scouting) {
    if (moveCompleted) {
      if (packageDirection == Left) {
        TurnByAngle(Right, 45);

        // Scout right
      } else {
        TurnByAngle(Left, 45);

        // Scout left
      }

      moveCompleted = false;
    } else if (turnCompleted) {
      if (packageDirection == Left) {
        if (movementState == TurningRight) {
          TurnByAngle(Left, 90);
          // Scout left
        } else {
          gatheringState = Lost;
          // Scouted, lost
        }
      } else {
        if (movementState == TurningLeft) {
          TurnByAngle(Right, 90);
        // Scout right
        } else {
          gatheringState = Lost;
          // Scouted, lost
        }
      }
      turnCompleted = false;
    }
  }

  if (gatheringState == Lost) {
    Turn(packageDirection);

    if (CheckPackage(Left)) {
      PackageFound(Left);
      // I was lost, but now I am found (left sensor)
      return;
    }
    if (CheckPackage(Right)) {
      PackageFound(Right);
      // I was lost, but now I am found (right sensor)
      return;
    }
  }
}

void CheckReverse() {
  if (distanceFrontLeft < stoppingDistance && distanceFrontRight < stoppingDistance) {
    movementState = Reversing;
    Serial.print("Reversing");
    Serial.println();
    if (canReverseCount) {
      reverseCount++;
      canReverseCount = false;
    }
  }

  if (movementState == Reversing) {
    if (distanceFrontpLeft > stopReverseDistance && distanceFrontRight > stopReverseDistance) {
      movementState = MovingStraight;
      // Finding wall
      canReverseCount = true;

      if (reverseCount > 0) {
        if (distanceLeft > distanceRight) {
          TurnByAngle(Right, 90);
          Serial.print("Reversed too many times, turning right");
          Serial.println();
        } else {
          TurnByAngle(Left, 90);
          Serial.print("Reversed too many times, turning left");
        }
        reverseCount = 0;
      }
    } else {
      Reverse();
    }
  }
}

void Reverse () {
  motorRight.writeMicroseconds(motorStopValue - moveSpeed);
  motorLeft.writeMicroseconds(motorStopValue - moveSpeed);
}


void ResumeDefaultState() {
  InterruptMove();
  InterruptTurn();
  globalState = Searching;
  movementState = MovingStraight; // Finding wall
  gatheringState = NoGather;
}

void TurnByAngle(Direction dir, int angle) {
  turnTimer = millis() + angle * turnConstant;

  if (dir == Left) {
    movementState = TurningLeft;
  } else {
    movementState = TurningRight;
  }
}

void CheckTurn() {
  if (movementState == TurningLeft || movementState == TurningRight) {
    if (millis() < turnTimer) {
      if (movementState == TurningLeft) {
        Turn(Left);
      } else {
        Turn(Right);
      }
    }
    else {
      turnTimer = 0;
      turnCompleted = true;
      motorRight.writeMicroseconds(motorStopValue);
      motorLeft.writeMicroseconds(motorStopValue);
    }
  }
}

void MoveForTime(int time) {
  moveTimer = millis() + time;
  movementState = MovingStraight;
}

void CheckMove() {
  if (movementState == MovingStraight) {
    if (millis() < moveTimer) {
      MoveStraight();
    } else {
      if (moveCompleted == false) {
        moveTimer = 0;
        moveCompleted = true;
  
        motorRight.writeMicroseconds(motorStopValue);
        motorLeft.writeMicroseconds(motorStopValue);
      }
    }
  }
}

void MoveStraight() {
  float rightDrive;
  float leftDrive;

  //rightDrive = constrain(((float)distanceFrontLeft - 10) / ((float)stopReverseDistance * 1), 0.5, 1);
  //leftDrive = constrain(((float)distanceFrontRight - 10) / ((float)stopReverseDistance * 1), 0.5, 1);

  rightDrive = 1;
  leftDrive = 1;

  
  motorRight.writeMicroseconds(motorStopValue + (float)moveSpeed * rightDrive);
  motorLeft.writeMicroseconds(motorStopValue + (float)moveSpeed * leftDrive);
}

int SonarSense(NewPing sonar) {
  int value = sonar.convert_cm(sonar.ping());
  if (value == 0) {
    value = maxDistance;
  }
  return (value);
}

int IRSense(int pin, int constant) {
  return (constant/analogRead(pin));
}

void FollowWall(Direction dir) {
  float rightDrive;
  float leftDrive;
  if (dir == Left) {
    rightDrive = constrain(2 * (float)distanceLeft / wallFollowDistance, 0, 2) - 1;
    leftDrive = constrain(2 * wallFollowDistance / (float)distanceLeft, 0, 2) - 1;
  } else {
    rightDrive = constrain(2 * wallFollowDistance / (float)distanceRight, 0, 2) - 1;
    leftDrive = constrain(2 * (float)distanceRight / wallFollowDistance, 0, 2) - 1;
  }

  motorRight.writeMicroseconds(motorStopValue + rightDrive * (float)moveSpeed);
  motorLeft.writeMicroseconds(motorStopValue + leftDrive * (float)moveSpeed);
}

void Turn(Direction dir) {
  signed int rightDrive;
  signed int leftDrive;

  if (dir == Left) {
    rightDrive = 1;
    leftDrive = -1;
  } else {
    rightDrive = -1;
    leftDrive = 1;
  }

  motorRight.writeMicroseconds(motorStopValue + rightDrive * moveSpeed);
  motorLeft.writeMicroseconds(motorStopValue + leftDrive * moveSpeed);
}

bool CheckPackage(Direction sensor) {
  if (sensor == Left) {
    if (distanceTopLeft - distanceBottomLeft > packageCheckingTolerance) {
      checkPackageLeftNoiseOverride++;
    } else {
      checkPackageLeftNoiseOverride = 0;
    }
    if (checkPackageLeftNoiseOverride > 1) {
      checkPackageLeftNoiseOverride = 0;
      return true;
    } else {
      return false;
    }
  } else {
    if  (distanceTopRight - distanceBottomRight > packageCheckingTolerance) {
      checkPackageRightNoiseOverride++;
    } else {
      checkPackageRightNoiseOverride = 0;
    }
    if (checkPackageRightNoiseOverride > 1) {
      checkPackageRightNoiseOverride = 0;
      return true;
    } else {
      return false;
    }
  }
}

void InterruptTurn() {
  movementState = NoMove;
  turnTimer = 0;
  turnCompleted = false;
}

void InterruptMove() {
  movementState = NoMove;
  moveTimer = 0;
  moveCompleted = false;
}

void CalculateDistances() {
  
  distanceTopLeft = first_order_recursive_filter(SonarSense(sonarTopLeft), distanceTopLeft);
  distanceTopRight = first_order_recursive_filter(SonarSense(sonarTopRight), distanceTopRight);
  distanceBottomLeft = first_order_recursive_filter(SonarSense(sonarBottomLeft), distanceBottomLeft);
  distanceBottomRight = first_order_recursive_filter(SonarSense(sonarBottomRight), distanceBottomRight);

  distanceLeft = IRSense(leftIRPin, leftIRConstant);
  distanceRight = IRSense(rightIRPin, rightIRConstant);
  
  distanceFrontLeft = IRSense(frontRightIRPin, frontRightIRConstant);
  distanceFrontRight = IRSense(frontLeftIRPin, frontLeftIRConstant);
}

double first_order_recursive_filter(int value, int previous) {
  return previous * ALPHA + value * (1 - ALPHA);
}

void UpdateDisplayCounter() {
  if (millis() > displayTimer) {
    UpdateDisplay();

    displayTimer = millis() + displayTimerPeriod;
  }
}

void UpdateDisplay() {
  lcd.setCursor(0, 0);
  lcd.print(globalStateNames[globalState]);
  lcd.setCursor(0, 1);
  lcd.print(movementStateNames[movementState]);

  if (globalState == Gathering) {
    unsigned long timer;

    timer = min(9999, gatherWatchDogTimer - millis());
        
    lcd.setCursor(1, 0);
    lcd.print(": ");
    lcd.print(gatheringStateNames[gatheringState]);
    lcd.print(" ");
    lcd.print(timer);
  } else {
    lcd.setCursor(1, 0);
    lcd.print("    ");
  }

  
  lcd.setCursor(11, 0);
  lcd.print(distanceTopLeft);
  lcd.setCursor(11, 1);
  lcd.print(distanceBottomLeft);
  lcd.setCursor(14, 0);
  lcd.print(distanceTopRight);
  lcd.setCursor(14, 1);
  lcd.print(distanceBottomRight);
}

////////////////////////////
/// Pickup stuff

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

int CheckSwitch (int reading)
{
    if (reading != lastSwitchState)
  {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != SwitchState) {
      SwitchState = reading;

      // only toggle the eMagnet if the new button state is HIGH
      if (SwitchState == HIGH) {
        eMagnetState = HIGH;
      }
      else
      {
        eMagnetState = LOW;
      }
    }
  }
  lastSwitchState = reading;
  return (eMagnetState);
}

void setPwmFrequency(int pin, int divisor) {
   byte mode;
   if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
     switch(divisor) {
       case 1: mode = 0x01; break;
       case 8: mode = 0x02; break;
       case 64: mode = 0x03; break;
       case 256: mode = 0x04; break;
       case 1024: mode = 0x05; break;
       default: return;
     }
     if(pin == 5 || pin == 6) {
       TCCR0B = TCCR0B & 0b11111000 | mode;
     } else {
       TCCR1B = TCCR1B & 0b11111000 | mode;
     }
   } else if(pin == 3 || pin == 11) {
     switch(divisor) {
       case 1: mode = 0x01; break;
       case 8: mode = 0x02; break;
       case 32: mode = 0x03; break;
       case 64: mode = 0x04; break;
       case 128: mode = 0x05; break;
       case 256: mode = 0x06; break;
       case 1024: mode = 0x7; break;
       default: return;
     }
     TCCR2B = TCCR2B & 0b11111000 | mode;
   }
}

void PickUpStuff() {
  int reading = digitalRead(springSwitch);
  int proxyRead = analogRead(InductiveProxy);
  

//  Serial.print(CheckProxy(proxyRead));
//  Serial.println();
  
  //Serial.print(CheckSwitch (reading));

  if (CheckProxy(proxyRead)) // Pull up weight
  {
    globalState = PickUp;
    UpdateDisplay();
    motorRight.writeMicroseconds(motorStopValue);
    motorLeft.writeMicroseconds(motorStopValue);
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
    delay(2500);
  
    
  }
  if (holding) //holding && !reading
  {
    digitalWrite(eMagnet, 0);
    holding = 0;
    Herkulex.moveOneAngle(motorA, 159, 1000, LED_GREEN); //move motorA backward
    Herkulex.moveOneAngle(motorB, -159, 1000, LED_GREEN); //move motorB forward
    delay(1500);
    Herkulex.moveOneAngle(motorA, -100, 1000, LED_CYAN); //move motorA backward
    Herkulex.moveOneAngle(motorB, 100, 1000, LED_CYAN); //move motorB forward
    delay(1500);    
    ResumeDefaultState();

    //PlaySong();
  }
}

/*
void StorageTrayTilt(int state)
{
  if (state = searching) 
  {
    myservoR.writeMicroseconds(700);
    myservoL.writeMicroseconds(2300);
    // storage tray is set to a flat gradient
  }
    if (state = atBase) 
  {
    myservoR.writeMicroseconds(2300);
    myservoL.writeMicroseconds(700);
    // ^storage tray is set to a steep gradient
    delay(1000);
    for (int wiggles = 0 ; wiggles < 4 ; wiggles++)
    {
      myservoR.writeMicroseconds(2000);
      myservoL.writeMicroseconds(1000);
      delay(300);
      myservoR.writeMicroseconds(2300);
      myservoL.writeMicroseconds(700);
      delay(300);
    }
    // ^storage tray is then 'wiggled' theatrically
    myservoR.writeMicroseconds(2300);
    myservoL.writeMicroseconds(700);
    // ^storage tray is finally restored to flat gradient
  }
}
*/

//void PlaySong() {
//  
//    setPwmFrequency(speaker, 1); //set beep frequency
//    analogWrite(speaker, 127);
//    delay(500);
//    analogWrite(speaker,0);
//    delay(20);
//  
//    setPwmFrequency(speaker, 8); //set beep frequency
//    analogWrite(speaker, 127);
//    delay(300);
//    analogWrite(speaker,0);
//    delay(10);
//  
//    setPwmFrequency(speaker, 64); //set beep frequency
//    analogWrite(speaker, 127);
//    delay(200);
//    analogWrite(speaker,0);
//    delay(30);
//  
//    setPwmFrequency(speaker, 256); //set beep frequency
//    analogWrite(speaker, 127);
//    delay(400);
//    analogWrite(speaker,0);
//    delay(5);
//  
//    setPwmFrequency(speaker, 256); //set beep frequency
//    analogWrite(speaker, 127);
//    delay(300);
//    analogWrite(speaker,0);
//    delay(10);
//  
//    setPwmFrequency(speaker, 1); //set beep frequency
//    analogWrite(speaker, 127);
//    delay(500);
//    analogWrite(speaker,0);
//    delay(20);
//  
//    setPwmFrequency(speaker, 8); //set beep frequency
//    analogWrite(speaker, 127);
//    delay(300);
//    analogWrite(speaker,0);
//    delay(10);
//  
//    setPwmFrequency(speaker, 64); //set beep frequency
//    analogWrite(speaker, 127);
//    delay(200);
//    analogWrite(speaker,0);
//    delay(30);
//  
//    setPwmFrequency(speaker, 256); //set beep frequency
//    analogWrite(speaker, 127);
//    delay(400);
//    analogWrite(speaker,0);
//    delay(5);
//  
//    setPwmFrequency(speaker, 256); //set beep frequency
//    analogWrite(speaker, 127);
//    delay(300);
//    analogWrite(speaker,0);
//    delay(10);
//}

