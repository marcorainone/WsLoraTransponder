/*
BMP280 is an I2C/SPI compatible library for the Bosch BMP280 barometer.
	
	Copyright (C) Martin Lindupp 2019
	
	V1.0.0 -- Initial release 	
	V1.0.1 -- Added ESP32 HSPI support and change library to unique name
	V1.0.2 -- Modification to allow external creation of HSPI object on ESP32
	V1.0.3 -- Change library name in the library.properties file
	
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

#include "BMP280_I2C.h"

////////////////////////////////////////////////////////////////////////////////
// BMP280_I2C Class Constructors
////////////////////////////////////////////////////////////////////////////////

BMP280_I2C::BMP280_I2C() : writeMask(0xFF) { setI2CAddress(BMP280_I2C_ADDR); }	// Constructor for I2C communications																							// Constructor for I2C communications
////////////////////////////////////////////////////////////////////////////////
// BMP280_I2C Public Member Functions
////////////////////////////////////////////////////////////////////////////////

uint8_t BMP280_I2C::begin(Mode mode, 															// Initialise BMP280 device settings
Oversampling presOversampling, 
Oversampling tempOversampling,
IIRFilter iirFilter,
TimeStandby timeStandby)
{
	initialise();																											// Call the Device base class "initialise" function
	reset();                                                          // Reset the BMP280 barometer
	if (readByte(BMP280_DEVICE_ID) != DEVICE_ID)              				// Check the device ID
	{
		return 0;                                                     	// If the ID is incorrect return 0
	}	
	readBytes(BMP280_TRIM_PARAMS, (uint8_t*)&params, sizeof(params)); // Read the trim parameters into the params structure
	setConfigRegister(iirFilter, timeStandby); 												// Initialise the BMP280 configuration register
	setCtrlMeasRegister(mode, presOversampling, tempOversampling);		// Initialise the BMP280 control and measurement register	
	return 1;																													// Report successful initialisation
}

uint8_t BMP280_I2C::begin(Mode mode, uint8_t addr)									// Initialise BMP280 with default settings, but selected mode and
{																																		// I2C address
	setI2CAddress(addr);
	begin(mode);
}

uint8_t BMP280_I2C::begin(Mode mode)																// Initialise BMP280 with default settings and selected mode
{
	return begin(mode, OVERSAMPLING_X16, OVERSAMPLING_X2, IIR_FILTER_OFF, TIME_STANDBY_05MS);
}

uint8_t BMP280_I2C::begin(uint8_t addr)															// Initialise BMP280 with default settings and selected I2C address
{
	setI2CAddress(addr);
	return begin(SLEEP_MODE, OVERSAMPLING_X16, OVERSAMPLING_X2, IIR_FILTER_OFF, TIME_STANDBY_05MS);
}

uint8_t BMP280_I2C::begin()																					// Initialise BMP280 with default settings (place in SLEEP_MODE)
{
	return begin(SLEEP_MODE, OVERSAMPLING_X16, OVERSAMPLING_X2, IIR_FILTER_OFF, TIME_STANDBY_05MS);
}

void BMP280_I2C::reset()																						// Reset the BMP280 barometer
{
	writeByte(BMP280_RESET & writeMask, RESET_CODE);                     									
	delay(10);                                                                
}

void BMP280_I2C::startNormalConversion() { setMode(NORMAL_MODE); }	// Start continuous measurement in NORMAL_MODE

void BMP280_I2C::startForcedConversion() 														// Start a one shot measurement in FORCED_MODE
{ 
	if (ctrl_meas.bit.mode == SLEEP_MODE)															// Only set FORCED_MODE if we're already in SLEEP_MODE
	{
		setMode(FORCED_MODE);
	}	
}			

void BMP280_I2C::stopConversion() { setMode(SLEEP_MODE); }					// Stop the conversion and return to SLEEP_MODE

void BMP280_I2C::setPresOversampling(Oversampling presOversampling)	// Set the pressure oversampling rate
{
	ctrl_meas.bit.osrs_p = presOversampling;
	writeByte(BMP280_CTRL_MEAS & writeMask, ctrl_meas.reg);
}

void BMP280_I2C::setTempOversampling(Oversampling tempOversampling)	// Set the temperature oversampling rate
{
	ctrl_meas.bit.osrs_t = tempOversampling;
	writeByte(BMP280_CTRL_MEAS & writeMask, ctrl_meas.reg);
}

void BMP280_I2C::setIIRFilter(IIRFilter iirFilter)									// Set the IIR filter setting
{
	config.bit.filter = iirFilter;
	writeByte(BMP280_CONFIG & writeMask, config.reg);
}

void BMP280_I2C::setTimeStandby(TimeStandby timeStandby)						// Set the time standby measurement interval
{
	config.bit.t_sb = timeStandby;
	writeByte(BMP280_CONFIG & writeMask, config.reg);
}

#ifdef	ENABLE_FLOAT_RESULTS
// -----------------------------------------------
// temperature & pressure in floating point

uint8_t BMP280_I2C::getTemperature(float &temperature)							// Get the temperature
{
	if (!checkMode())																									// Check if a measurement is ready
	{
		return 0;
	}
	uint8_t data[3];                                                  // Create a data buffer
	readBytes(BMP280_TEMP_MSB, &data[0], 3);             							// Read the temperature and pressure data
	int32_t adcTemp = (int32_t)data[3] << 12 | (int32_t)data[4] << 4 | (int32_t)data[5] >> 4;  // Copy the temperature and pressure data into the adc variables
	int32_t temp = bmp280_compensate_T_int32(adcTemp);                // Temperature compensation (function from BMP280 datasheet)
	temperature = (float)temp / 100.0f;                               // Calculate the temperature in degrees celcius
	return 1;
}

uint8_t BMP280_I2C::getPressure(float &pressure)										// Get the pressure
{
	float temperature;
	return getTempPres(temperature, pressure);
}

uint8_t BMP280_I2C::getTempPres(float &temperature, float &pressure)	// Get the temperature and pressure
{
	if (!checkMode())																									// Check if a measurement is ready
	{
		return 0;
	}
	uint8_t data[6];                                                  // Create a data buffer
	readBytes(BMP280_PRES_MSB, &data[0], 6);             							// Read the temperature and pressure data
	int32_t adcTemp = (int32_t)data[3] << 12 | (int32_t)data[4] << 4 | (int32_t)data[5] >> 4;  // Copy the temperature and pressure data into the adc variables
	int32_t adcPres = (int32_t)data[0] << 12 | (int32_t)data[1] << 4 | (int32_t)data[2] >> 4;
	int32_t temp = bmp280_compensate_T_int32(adcTemp);                // Temperature compensation (function from BMP280 datasheet)
	uint32_t pres = bmp280_compensate_P_int64(adcPres);               // Pressure compensation (function from BMP280 datasheet)
	temperature = (float)temp / 100.0f;                               // Calculate the temperature in degrees celcius
	pressure = (float)pres / 256.0f / 100.0f;                         // Calculate the pressure in millibar
	return 1;
}

#else
// -----------------------------------------------
// temperature and pressure in long

uint8_t BMP280_I2C::getTemperature(long &temperature)							// Get the temperature
{
	if (!checkMode())																									// Check if a measurement is ready
	{
		return 0;
	}
	uint8_t data[3];                                                  // Create a data buffer
	readBytes(BMP280_TEMP_MSB, &data[0], 3);             							// Read the temperature and pressure data
	int32_t adcTemp = (int32_t)data[3] << 12 | (int32_t)data[4] << 4 | (int32_t)data[5] >> 4;  // Copy the temperature and pressure data into the adc variables
	int32_t temp = bmp280_compensate_T_int32(adcTemp);                // Temperature compensation (function from BMP280 datasheet)
	temperature = temp / 10L;                               // Calculate the temperature in degrees celcius * 10
	return 1;
}

uint8_t BMP280_I2C::getPressure(long &pressure)										// Get the pressure
{
	long temperature;
	return getTempPres(temperature, pressure);
}

uint8_t BMP280_I2C::getTempPres(long &temperature, long &pressure)	// Get the temperature and pressure
{
	if (!checkMode())																									// Check if a measurement is ready
	{
		return 0;
	}
	uint8_t data[6];                                                  // Create a data buffer
	readBytes(BMP280_PRES_MSB, &data[0], 6);             							// Read the temperature and pressure data
	int32_t adcTemp = (int32_t)data[3] << 12 | (int32_t)data[4] << 4 | (int32_t)data[5] >> 4;  // Copy the temperature and pressure data into the adc variables
	int32_t adcPres = (int32_t)data[0] << 12 | (int32_t)data[1] << 4 | (int32_t)data[2] >> 4;
	int32_t temp = bmp280_compensate_T_int32(adcTemp);                // Temperature compensation (function from BMP280 datasheet)
	uint32_t pres = bmp280_compensate_P_int64(adcPres);               // Pressure compensation (function from BMP280 datasheet)
	temperature = temp / 10L;                               // Calculate the temperature in degrees celcius
	// the pressure value is in Pascal
	pressure = pres / 256L;                         		// Calculate the pressure in pascal
	return 1;
}

// -----------------------------------------------
#endif					// #ifdef ENABLE_FLOAT_RESULTS

#ifdef	EN_CALC_ALTITUDE
// -----------------------------------------------
uint8_t BMP280_I2C::getAltitude(float &altitude)										// Get the altitude
{
	float temperature, pressure;
	return getMeasurements(temperature, pressure, altitude);
}

uint8_t BMP280_I2C::getMeasurements(float &temperature, float &pressure, float &altitude)		// Get all measurements temperature, pressue and altitude
{  
	const float SEA_LEVEL_PRESSURE = 1013.23f;
	if (getTempPres(temperature, pressure))
	{
		altitude = ((float)powf(SEA_LEVEL_PRESSURE / pressure, 0.190223f) - 1.0f) * (temperature + 273.15f) / 0.0065f; // Calculate the altitude in metres 
		return 1;
	}
	return 0;
}
// -----------------------------------------------
#endif					// #ifdef	EN_CALC_ALTITUDE

////////////////////////////////////////////////////////////////////////////////
// BMP280_I2C Private Member Functions
////////////////////////////////////////////////////////////////////////////////

void BMP280_I2C::setMode(Mode mode)																	// Set the BMP280's mode
{
	ctrl_meas.bit.mode = mode;
	writeByte(BMP280_CTRL_MEAS & writeMask, ctrl_meas.reg);
}

// Set the BMP280 control and measurement register 
void BMP280_I2C::setCtrlMeasRegister(Mode mode, Oversampling presOversampling, Oversampling tempOversampling)
{
	ctrl_meas.reg = (uint8_t)tempOversampling << 5 | (uint8_t)presOversampling << 2 | (uint8_t)mode;
	writeByte(BMP280_CTRL_MEAS & writeMask, ctrl_meas.reg);                              
}

// Set the BMP280 configuration register
void BMP280_I2C::setConfigRegister(IIRFilter iirFilter, TimeStandby timeStandby)
{
	config.reg = (uint8_t)timeStandby << 5 | (uint8_t)iirFilter << 2;
	writeByte(BMP280_CONFIG & writeMask, config.reg);                              
}

uint8_t BMP280_I2C::checkMode()																			// Check the device mode
{
	static bool pending = false;
	if (ctrl_meas.bit.mode == SLEEP_MODE)															// If in SLEEP_MODE return immediately
	{		
		pending = false;
		return 0;
	}
	else 																															// Otherwise we're in NORMAL or FORCED mode
	{
		status.reg = readByte(BMP280_STATUS);														// Check the STATUS register
		if (status.bit.measuring && !pending)														// If a measurement is taking place return immediately
		{							
			pending = true;
			return 0;																										
		}
		else if (!status.bit.measuring && pending)
		{					
			if (ctrl_meas.bit.mode == FORCED_MODE)												// A measurement is ready, if we're in FORCED_MODE switch back to SLEEP_MODE
			{		
				ctrl_meas.bit.mode = SLEEP_MODE;	
			}
			pending = false;
			return 1;
		}		
	}
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Bosch BMP280_I2C (Private) Member Functions
////////////////////////////////////////////////////////////////////////////////

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t BMP280_I2C::bmp280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t)params.dig_T1 << 1))) * ((int32_t)params.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)params.dig_T1)) * ((adc_T >> 4) - ((int32_t)params.dig_T1))) >> 12) *
	((int32_t)params.dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BMP280_I2C::bmp280_compensate_P_int64(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)params.dig_P6;
	var2 = var2 + ((var1 * (int64_t)params.dig_P5) << 17);
	var2 = var2 + (((int64_t)params.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)params.dig_P3) >> 8) + ((var1 * (int64_t)params.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)params.dig_P1) >> 33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)params.dig_P9) * (p >> 13) * (p>>13)) >> 25;
	var2 = (((int64_t)params.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)params.dig_P7) << 4);
	return (uint32_t)p;
}
