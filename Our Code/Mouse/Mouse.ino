/*
USB Pinout (Left to Right, USB symbol up)
4: GND
3: Clk
2: Data
1: Vcc
*/

#include "ps2.h"

double positionX;
double positionY;

PS2 mouse(20,21);

void setup(){
  Serial.begin(9600);
  while(!Serial); 
  Serial.println("Setup");
  mouse.mouse_init();
  Serial.println("Mouse Ready");
}

void loop(){
  char stat,x,y;
  mouse.mouse_pos(stat,x,y);

  positionX += (double)x;
  positionY += (double)y;
  
//  Serial.print(stat, BIN);
//  Serial.print("\tx=");
//  Serial.print(x, DEC);
//  Serial.print("\ty=");
//  Serial.println(y, DEC);

  Serial.print(positionX);
  Serial.print(" ");
  Serial.print(positionY);
  Serial.println();
  
  //delay(500);
}
