 #include <NewPing.h>
 
// Integrate the ultrasonic sensor

int trigPin = 36;
int echoPin = 37;
int maxDistance = 200;

NewPing sonarBottomLeft(trigPin, echoPin, maxDistance);
NewPing sonarTopLeft(35, 34, maxDistance);

int distanceBottomLeft;
int distanceTopLeft;

void setup()
{ 
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  pinMode(35, OUTPUT);                 //Pin 49 is used to enable IO power
  pinMode(34, INPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  digitalWrite(35, 1);                 //Enable IO power on main CPU board
  digitalWrite(34, 0);                 //Enable IO power on main CPU board

  Serial.begin(9600);                  // initialize serial communication:
}

void loop()
{
  unsigned int pingBottomLeft = sonarBottomLeft.ping();
  unsigned int pingTopLeft = sonarTopLeft.ping();
 
  // convert the time into a distance
  distanceBottomLeft = sonarBottomLeft.convert_cm(pingBottomLeft);
  distanceTopLeft = sonarTopLeft.convert_cm(pingTopLeft);
  Serial.print(distanceBottomLeft);
  Serial.print(" ");
  Serial.print(distanceTopLeft);
  Serial.println();
  delay(50);
}
 
 
long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
} 
