int eMagnet = 34;

void setup() {
  // put your setup code here, to run once:
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  pinMode(eMagnet, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(eMagnet, HIGH);
  delay(6000);
  digitalWrite(eMagnet, HIGH);
  delay(6000);
}
