/**
*
* SimpleTest.ino
*
* Written by Christian Paul, 2014
*
* Demonstration of the BMP180 library.
* Measure the temperature and pressure in a 'automatic way'.
*
*/
#include <Wire.h>
#include <BMP180.h>

BMP180 bmp180;

void setup()
{
  // init
  bmp180.init();

  // welcome
  Serial.begin(9600);
  Serial.println("BMP180 Demo");
  if (!bmp180.hasValidID())
    Serial.println("Error - please check the BMP180 board!");
}

void loop() {

  Serial.print("Temperature: ");
  Serial.print(bmp180.getTemperature());
  Serial.println(" C");
  
  Serial.print("Pressure: ");
  Serial.print(bmp180.getPressure());
  Serial.println(" hPa");
  
  Serial.println();
  delay(2000);
}






