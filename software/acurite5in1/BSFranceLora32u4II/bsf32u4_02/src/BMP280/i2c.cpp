/*
	I2C compatible base class library.
	
	Copyright (C) Martin Lindupp 2019
	
	V1.0.0 -- Initial release
	V1.0.1 -- Added ESP32 HSPI support	
	V1.0.2 -- Modification to allow external creation of HSPI object on ESP32
	
	The MIT License (MIT)
	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#include "i2c.h"

////////////////////////////////////////////////////////////////////////////////
// i2c Class Constructors
////////////////////////////////////////////////////////////////////////////////

i2c::i2c() : comms(I2C_COMMS) {}														// Initialise constructor for I2C communications

////////////////////////////////////////////////////////////////////////////////
// i2c Public Member Function
////////////////////////////////////////////////////////////////////////////////

void i2c::setClock(uint32_t clockSpeed)													// Set the I2C or SPI clock speed
{
	Wire.setClock(clockSpeed);
}

////////////////////////////////////////////////////////////////////////////////
// i2c I2C & SPI Wrapper (Protected) Member Functions
////////////////////////////////////////////////////////////////////////////////

void i2c::initialise()																	// Initialise device communications
{
	Wire.begin();																		// Initialise I2C communication
	Wire.setClock(400000);																// Set the SCL clock to default of 400kHz
}

void i2c::setI2CAddress(uint8_t addr)													// Set the i2c's I2C address
{	
	address = addr;
}

void i2c::writeByte(uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  													// Write a byte to the sub-address using I2C
	Wire.write(subAddress);          
	Wire.write(data);                 
	Wire.endTransmission();          
}

uint8_t i2c::readByte(uint8_t subAddress)												// Read a byte from the sub-address using I2C
{
	uint8_t data;
	Wire.beginTransmission(address);         
	Wire.write(subAddress);                  
	Wire.endTransmission(false);             
	Wire.requestFrom(address, (uint8_t)1);	 
	data = Wire.read();                      
	return data;                             											// Return data read from sub-address register
}

void i2c::readBytes(uint8_t subAddress, uint8_t* dest, uint8_t count)
{  
	Wire.beginTransmission(address);          
	Wire.write(subAddress);                   
	Wire.endTransmission(false);              
	uint8_t i = 0;
	Wire.requestFrom(address, count);  
	while (Wire.available()) 
	{
		dest[i++] = Wire.read();          
	}
}
