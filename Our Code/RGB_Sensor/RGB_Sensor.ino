#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <LiquidCrystal.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(31, 32, 35, 36, 37, 30);

void setup() 
{
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  
  lcd.begin(16, 2);                   // Set columns and rows for LCD
  lcd.print("RGB ON");

  if (tcs.begin()) 
  {
    lcd.clear();
    lcd.print("Found RGB");
  } 
  else 
  {
    lcd.clear();
    lcd.print("Not Found");
    while (1); // halt!
  }
}


void loop() 
{
  uint8_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on LED

  delay(60);  // takes 50ms to read 
  
  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED
  
  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float g, b;
  g = green; g /= sum;
  b = blue; b /= sum;
  g *= 256; b *= 256;
  
  lcd.clear();
  lcd.print("G:");
  lcd.setCursor(3,0);
  lcd.print((int)g,HEX);
  lcd.setCursor(0,1);
  lcd.print("B:");
  lcd.setCursor(3,1);
  lcd.print((int)b,HEX);
  
//  Serial.print("C:\t"); Serial.print(clear);
//  Serial.print("\tR:\t"); Serial.print(red);
//  Serial.print("\tG:\t"); Serial.print(green);
//  Serial.print("\tB:\t"); Serial.print(blue);
}

