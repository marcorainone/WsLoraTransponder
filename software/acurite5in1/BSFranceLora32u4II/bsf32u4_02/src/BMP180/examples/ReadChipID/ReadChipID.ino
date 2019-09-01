/**
*
* ReadChipID.ino
*
* Written by Christian Paul, 2014
*
* Reads the Chip-ID Register.
* This muste be 0x55.
*
*/
#include <Wire.h>

void setup()
{

  // welcome
  Serial.begin(9600);
  Serial.println("Read BMP180's ID");
  Serial.println("must be 0x55 ...");
  Serial.println();
  
  // select register
  Wire.begin();
  Wire.beginTransmission(0x77);
  Wire.write(0xD0);
  Wire.endTransmission();
  
  // read value
  Wire.beginTransmission(0x77);
  Wire.requestFrom(0x77, 1);
  byte b = Wire.read();
  Wire.endTransmission();

  // output
  Serial.print("ID is: 0x");
  Serial.println(b, HEX);   
}

void loop() {}






