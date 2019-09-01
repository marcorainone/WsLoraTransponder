/**
*
* ReadAndCalc.ino
*
* Written by Christian Paul, 2014
*
* Demonstration of the BMP180 library.
* Measure the temperature and pressure in a 'manually way'.
* First measure both values and than compensated it.
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
  Serial.print("The ID of the chip is 0x");
  Serial.print(bmp180.getID(), HEX);
  Serial.print(" - that's ");
  if (bmp180.hasValidID())
    Serial.println("OK!");
  else
    Serial.println("not OK!");
}

void loop() {

  // measure
  byte samplingMode = BMP180_OVERSAMPLING_ULTRA_HIGH_RESOLUTION;
  long ut = bmp180.measureTemperature();
  long t = bmp180.compensateTemperature(ut);
  long up = bmp180.measurePressure(samplingMode);
  long p = bmp180.compensatePressure(up, samplingMode);
  
  // it's possible to do more than one measurement of the pressure
  // and measure the temperature only once
  // long p = 0;
  // for (int i=0; i<10; i++) {
  //   up = bmp180.measurePressure(samplingMode);
  //   p+ = bmp180.compensatePressure(up, samplingMode);
  // }
  // p = p / 10;
  
  // output
  Serial.println();
  Serial.print("Uncompensated temperature UT = ");
  Serial.println(ut);
  Serial.print("Compensated temperature    T = ");
  Serial.print(t);
  Serial.print(" (");
  Serial.print(bmp180.formatTemperature(t));
  Serial.println(" C)");
  Serial.print("Uncompensated pressure    UP = ");
  Serial.println(up);
  Serial.print("Compensated pressure       P = ");
  Serial.print(p);
  Serial.print(" (");
  Serial.print(bmp180.formatPressure(p));
  Serial.println(" hPa)");

  // wait
  delay(3000);

}






