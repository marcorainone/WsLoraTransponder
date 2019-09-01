// **********************************************************************************
// Project:    LaCrosse WS-2300 Weather Station lora tranceiver
//             v1 2019-03-01, ICTP Wireless Lab
//             Version for BsFrance LORA32U4II unit
// Programmer: Marco Rainone - ICTP Wireless Lab
// Specifications, revisions and verifications:   
//             Marco Zennaro, Ermanno Pietrosemoli, Marco Rainone - ICTP Wireless Lab
// **********************************************************************************
//
// The project is released with Mit License
// https://opensource.org/licenses/MIT
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// **********************************************************************************

//
// Info
// 20/06 test power etherodina with digital output
// 11/07: added another interrupt for data 
//
// Note 16/07:
// the  module .ino is compiled with arduino hourly builds
// https://www.arduino.cc/en/Main/Software?
// LAST UPDATE 11 July 2018 17:13:23 GMT
// https://www.arduino.cc/download.php?f=/arduino-nightly-windows.zip
// La versione 1.8.5 ha un bug del compilatore gcc che impedisce la compilazione
//
// -----------------------------------------------------------------
// https://github.com/fcgdam/TTN32u4ABP/blob/master/src/main.cpp
// https://github.com/kersing/node-workshop/blob/master/lora32u4.md#connecting-dio1-to-d6
// reported problem:
// https://github.com/matthijskooijman/arduino-lmic/issues/162
// LoRa32u4 II never gets EV_TXCOMPLETE #162
// Problem solved. I had to solder DIO1 to D1. For reference, DIO1 is located on the back side of the board. See screen shot.
// Here’s the LMIC pin mapping:
// const lmic_pinmap lmic_pins = {
//     .nss = 8,
//     .rxtx = LMIC_UNUSED_PIN,
//     .rst = 4,
//     .dio = {7, 1, 2},
// };

//
// As another example, use:
// https://github.com/kersing/node-workshop/blob/master/lora32u4.md
// https://lorenzadriaensen.com/cheap-ttn-node-rfm95w-arduino-pro-mini/
// https://primalcortex.wordpress.com/2017/11/10/using-the-bsfrance-lora32u4-board-to-connect-to-the-things-network-lorawan/
//
// This module use arduino-lmic library
// Open Arduino IDE. 
// Now we are going to include necessary libraries to build firmware for our LoRa node. 
// First of all, download this ZIP file. 
// https://github.com/matthijskooijman/arduino-lmic
// https://github.com/matthijskooijman/arduino-lmic/archive/master.zip
// Then in Arduino IDE, go to Sketch -> Include Library -> Add .ZIP library, 
// select the downloaded ZIP file then open.
// 
// -----------------------------------------------------------------
// to reduce space of lmic library
// Edit lmic configuration file C:\Users\<user>\Documents\Arduino\libraries\arduino-lmic-master\src\lmic\config.h
// Uncomment these rows to enable
// DISABLE_JOIN, DISABLE_PING, DISABLE_BEACONS
//         // Uncomment this to disable all code related to joining
//         #define DISABLE_JOIN
//         // Uncomment this to disable all code related to ping
//         #define DISABLE_PING
//         // Uncomment this to disable all code related to beacon tracking.
//         // Requires ping to be disabled too
//         #define DISABLE_BEACONS
//
// Arduino code decode several Acurite devices OTA data stream.
// The arduino uses RXB6 etherodina interface:
// https://www.amazon.it/WINGONEER-Superheterodyne-Wireless-Receiver-Arduino/dp/B06XHJMC82
// http://aitendo3.sakura.ne.jp/aitendo_data/product_img/wireless/315MHz-2012/ASK-RXB6/RXB6_aitendo.pdf
// -----------------------------------------------------------
// the module is able to read data from these devices:
// 1) AcuRite 06014 PRO+ 5-in-1 Weather Sensor with Rain Gauge, Wind Speed, Wind Direction, Temperature and Humidity (vn1txca2)
//    https://www.amazon.com/AcuRite-Weather-Direction-Temperature-Humidity/dp/B00SN1WHEU/ref=sr_1_7?s=home-garden&ie=UTF8&qid=1528101292&sr=1-7&keywords=acurite+5+in+1+weather+station
// 2) AcuRite 06045M Lightning Detector Sensor with Temperature & Humidity
//    https://www.amazon.com/AcuRite-06045M-Lightning-Detector-Temperature/dp/B01LNALL6C
// 3) AcuRite 06044M 06044M Wireless Temperature and Humidity Monitor Sensor
//    https://www.amazon.com/AcuRite-06044M-Wireless-Temperature-Humidity/dp/B01G7BE9WK/ref=cm_cr_arp_d_product_top?ie=UTF8
//

#include <Arduino.h>
// use these local libraries
#include "src/arduino-lmic/lmic.h"
#include "src/arduino-lmic/hal/hal.h"
#include <SPI.h>
#include <avr/power.h>

#ifndef __AVR_ATmega32U4__
#define __AVR_ATmega32U4__
#endif
#include "src/LowPower/LowPower.h"
#include <Wire.h>
#include <avr/interrupt.h>		// Needed to use interrupts
#include "src/BMP180/BMP180.h"
#include <LoRa.h>				// for utility set low-power chip lora

#include "bsf32u4_01.h"

// ---------------------------------------------------------------
// defines used for configuration
// ---------------------------------------------------------------

// enable SERIAL_MSG if you want to print msg to serial
// #define SERIAL_MSG

#define BSFRANCEBOARD 1
#define LOWPOWER 1

// ---------------------------------------------------
// WATCHDOG
// define ENABLE_WATCHDOG if you want to manage watchdog
// #define ENABLE_WATCHDOG

// ---------------------------------------------------
// define USE_I2C_SENSPRE if you want to read data from I2C pressure sensor
#define USE_I2C_SENSPRE

// ---------------------------------------------------
// define USE_BMP180 if you want to use Bosh BMP180 temp / press sensor
// otherwise the sensor used is BMP280 
#define  USE_BMP180

// ---------------------------------------------------
// define RECEIVE_ONLY_1_ID if you want to receive messages only from a 5n1 weather station, 
// with ID specified in idWeatherStation
// IMPORTANT:
// The ID of Lacrosse WS2300 changes randomly each time it powers up or after a reset of the receiver.
// For this weather station model it is best to disable this control during normal transponder operation
// CHECK ID DISABLED: #define RECEIVE_ONLY_1_ID

// ---------------------------------------------------------------
// ID Lacrosse WS2300: default value
// ---------------------------------------------------------------

// id of Lacrosse ws2300 weather station connected to receiver.
// IMPORTANT:
// see note above about Lacrosse WS2300
const int idWeatherStation = 0x73;

volatile bool joined = false;
volatile bool sleeping = false;
int CountLowPower = 0;

// mod 16/07:
// if DataMsgWsFull = 0x03, you have received the two types os 5n1 msg
int  DataMsgWsFull = 0x00;
//-----------------------------------------------------------------
// hardware configuration
//-----------------------------------------------------------------
//
// -------------------------------------
// use data pin PD5 as output power for etherodina 
#define RXB6_POWER		(5)             // PD5 for power etherodina
// -------------------------------------

//-----------------------------------------------------------------
// clock prescaler
//-----------------------------------------------------------------
// Switch to RC Clock:
// ClockDiv e' clock_div_2 ... clock_div_128
void ClockRc(int ClockDiv)
{
	UDINT  &= ~(1 << SUSPI); // UDINT.SUSPI = 0; Usb_ack_suspend
	USBCON |= ( 1 <<FRZCLK); // USBCON.FRZCLK = 1; Usb_freeze_clock
	PLLCSR &= ~(1 << PLLE); // PLLCSR.PLLE = 0; Disable_pll
	
	CLKSEL0 |= (1 << RCE); // CLKSEL0.RCE = 1; Enable_RC_clock()
	while ( (CLKSTA & (1 << RCON)) == 0){}    // while (CLKSTA.RCON != 1);  while (!RC_clock_ready())
	CLKSEL0 &= ~(1 << CLKS);  // CLKSEL0.CLKS = 0; Select_RC_clock()
	CLKSEL0 &= ~(1 << EXTE);  // CLKSEL0.EXTE = 0; Disable_external_clock

	// clock_prescale_set(clock_div_2);
	// clock_prescale_set(clock_div_128);
	clock_prescale_set(ClockDiv);
}

// Enable External Clock
void ClockExternal()
{
	clock_prescale_set(clock_div_1);
	CLKSEL0 |= (1 << EXTE);   // CKSEL0.EXTE = 1; // Enable_external_clock(); 
	while ( (CLKSTA & (1 << EXTON)) == 0 ){} // while (CLKSTA.EXTON != 1);    // while (!External_clock_ready()); 
	CLKSEL0 |= (1 << CLKS);   // CLKSEL0.CLKS = 1;    //  Select_external_clock(); 
	PLLCSR |= (1 << PLLE);    // PLLCSR.PLLE = 1; // Enable_pll(); 
	CLKSEL0 &= ~(1 << RCE);   // CLKSEL0.RCE = 0; // Disable_RC_clock(); 
	while ( (PLLCSR & (1 << PLOCK)) == 0){}   // while (PLLCSR.PLOCK != 1);   // while (!Pll_ready()); 
	USBCON &= ~(1 << FRZCLK); // USBCON.FRZCLK = 0;   // Usb_unfreeze_clock(); 
}

//-----------------------------------------------------------------
// manage usb
//-----------------------------------------------------------------
// see:
// https://arduino.stackexchange.com/questions/10408/starting-usb-on-pro-micro-after-deep-sleep-atmega32u4

// switch off usb use before send module to sleep
void UsbOff()
{
	// Disable the USB interface 
	USBCON &= ~(1 << USBE); 
	
	// Disable the VBUS transition enable bit 
	USBCON &= ~(1 << VBUSTE); 
	
	// Disable the VUSB pad 
	USBCON &= ~(1 << OTGPADE); 
	
	// Freeze the USB clock 
	USBCON &= ~(1 << FRZCLK); 
	
	// Disable USB pad regulator 
	UHWCON &= ~(1 << UVREGE); 
	
	// Clear the IVBUS Transition Interrupt flag 
	USBINT &= ~(1 << VBUSTI); 
	
	// Physically detact USB (by disconnecting internal pull-ups on D+ and D-) 
	UDCON |= (1 << DETACH); 

	power_usb_disable();  // Keep it here, after the USB power down
}

// to re-enable usb after exit from sleep state
void UsbOn()
{
	power_usb_enable();
	delay(100);
	USBDevice.attach(); // keep this
	delay(100);
#ifdef SERIAL_MSG
	Serial.begin(9600);
	delay(100);	
#endif					// #ifdef SERIAL_MSG
}

//-----------------------------------------------------------------
// lora in sleep
//-----------------------------------------------------------------

// --------------------------------------------------
// These instructions are taken from this example: LoRaReceiver.ino
// C:\Users\marco\Documents\Arduino\Hardware\BSFrance-avr\avr\libraries\LoRa\examples\LoRaReceiver\LoRaReceiver.ino
// il modulo fa parte della libreria fornita col micro BsFrance

//LoR32u4II 868MHz or 915MHz (black board)
#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    868E6  // 915E6
#define PABOOST true 
#define REG_OP_MODE              0x01
#define REG_VERSION              0x42
#define MODE_SLEEP               0x00
#define RF_OPMODE_MASK           0xF8

// --------------------------------------------------

// from lora.cpp
#define _spiSettings		SPISettings(8E6, MSBFIRST, SPI_MODE0)
uint8_t singleTransfer(uint8_t address, uint8_t value)
{
	uint8_t response;
	digitalWrite(SS, LOW);
	SPI.beginTransaction(_spiSettings);
	SPI.transfer(address);
	response = SPI.transfer(value);
	SPI.endTransaction();
	digitalWrite(SS, HIGH);
	return response;
}

uint8_t readRegister(uint8_t address)
{
	return singleTransfer(address & 0x7f, 0x00);
}

void writeRegister(uint8_t address, uint8_t value)
{
	singleTransfer(address | 0x80, value);
}

void LoRaSleep()
{
	int value;
	// SetAntSwLowPower( true );
	value = readRegister(REG_OP_MODE) & RF_OPMODE_MASK;
	writeRegister(REG_OP_MODE, value | MODE_SLEEP);
}

//-----------------------------------------------------------------
// Power management of functional blocks
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// RXB6 433Mhz Superheterodyne Wireless Receiver Module
//-----------------------------------------------------------------
// 
// https://www.ebay.it/itm/RXB6-433Mhz-Superheterodyne-Wireless-Receiver-Module-for-Arduino-ARM-AVR-/301953017682
// https://www.amazon.it/WINGONEER-Superheterodyne-Wireless-Receiver-Arduino/dp/B06XHJMC82
//
// power on/off etherodina module RXB6
void Rxb6Power(int value)
{
	// modify 20/06: on power etherodina
	pinMode(RXB6_POWER, OUTPUT);				// output pin power on/off
	if(value>0)
	{
		digitalWrite(RXB6_POWER, HIGH);		// sets the digital pin RXB6_POWER on
	}
	else
	{
		digitalWrite(RXB6_POWER, LOW);		// sets the digital pin RXB6_POWER off
	}
}


// --------------------------------------------
// ws2300 data
// --------------------------------------------


//-----------------------------------------------------------------
// rx Ws2300
//-----------------------------------------------------------------
//
// The pulse durations are the measured time in micro seconds between
// pulse edges.

byte 		dataRx = 0x00;		// byte received from etherodina
byte 		checksum = 0x00;	//
bool errMsg = false;
unsigned int countEndOfMsg = 0;	// n. of end of msg

int nMsg = 0;								// n. msg rx					
unsigned long timeRxb6LastMsg = 0L;			// time rx last msg in usec
unsigned long Rxb6IntLastTime = 0L;

bool         received  = false; // true if sync plus enough bits found
unsigned int pulseCount = 0;	// n. of pulses received
unsigned int bytesReceived = 0;

// ---------------------------------
// data read from bmp180 pressure sensor

BMP180 bmp180;

long Bmp180temp, Bmp180press;

// ---------------------------------
// lacrosse Ws2300 msg. All data are nibble (4bit)
// see rtl_433, module lacrossews.c
// Packet Format is 52 bits/ 13 nibbles
//  bits    nibble
//  0- 3    0 - 0000
//  4- 7    1 - 1001
//  8-11    2 - Type  GPTT  G=0, P=Parity, Gust=Gust, TT=Type  GTT 000=Temp, 001=Humidity, 010=Rain, 011=Wind, 111-Gust
// 12-15    3 - ID High
// 16-19    4 - ID Low
// 20-23    5 - Data Types  GWRH  G=Gust Sent, W=Wind Sent, R=Rain Sent, H=Humidity Sent
// 24-27    6 - Parity TUU? T=Temp Sent, UU=Next Update, 00=8 seconds, 01=32 seconds, 10=?, 11=128 seconds, ?=?
// 28-31    7 - Value1
// 32-35    8 - Value2
// 36-39    9 - Value3
// 40-43    10 - ~Value1
// 44-47    11 - ~Value2
// 48-51    12 - Check Sum = Nibble sum of nibbles 0-11

// 
#define LGPTT  0     //  8-11	2 - Type  GPTT  G=0, P=Parity, Gust=Gust, TT=Type  GTT 000=Temp, 001=Humidity, 010=Rain, 011=Wind, 111-Gust
#define LID    1     // 12-15, 16-19	3 - ID High, 4 - ID Low
#define LGWRH  2     // 20-23	5 - Data Types  GWRH  G=Gust Sent, W=Wind Sent, R=Rain Sent, H=Humidity Sent
#define LTUU   3     // 24-27	6 - Parity TUU? T=Temp Sent, UU=Next Update, 00=8 seconds, 01=32 seconds, 10=?, 11=128 seconds, ?=?
#define LV1	   4     // 28-31	7 - Value1
#define LV2	   5     // 32-35	8 - Value2
#define LV3	   6     // 36-39	9 - Value3
byte msg[7];

// message received
byte msg_type;
byte sensor_id;
byte msg_data;
byte msg_unknown;
int msg_value;
int wind_dir;
int wind_spd;
unsigned long tmMsg;			// duration of msg

unsigned int BatteryVoltage;	// used to store battery voltage

// lacrosse ws2300 data
typedef struct {
	unsigned long idMsg;		// message count to transmit
	unsigned long time;			// time of update variables
	unsigned long tmMsg;		// duration of msg
	int msg_type;				// type of msg
	uint16_t SensorId;			// id of lacrosse ws2300 sensor unit
	int WindSpeed;				// wind speed
	int WindDirection;			// wind direction
	int GustSpeed;				// gust speed
	int GustDirection;			// gust direction
	int Rainfall; 				// rainfall
	int Temp;					// temperature in celsius degrees
	int Hum;					// % humidity
	// ---------------------------------
} data_WsLacro2300;

data_WsLacro2300 dWs2300;

data_WsLacro2300 dWsRx[10];		// array of data msg rx

void init_dWs2300()
{
	dWs2300.idMsg = 0L			;		// message count to transmit
	dWs2300.time = 0L			;		// time of update variables
	dWs2300.tmMsg = 0L			;		// duration of msg
	dWs2300.msg_type = 0		;
	dWs2300.SensorId = 0		;
	dWs2300.WindSpeed = 0		;
	dWs2300.WindDirection = 0	;
	dWs2300.GustSpeed = 0		;
	dWs2300.GustDirection = 0	;
	dWs2300.Rainfall = 0		; 
	dWs2300.Temp = 0			;
	dWs2300.Hum = 	0 			;
}

// increment id msg
void incIdMsg()
{
	dWs2300.idMsg++;
}

void ulong2bytes(uint8_t *ptr, unsigned long val)
{
	int i;
	ptr = ptr + 3;
	for(i=0;i<4;i++)
	{
		*ptr = (uint8_t)(val & 0xFF);
		ptr--;
		val = val >> 8;
	}
}
void word2bytes(uint8_t *ptr, uint16_t val)
{
	int i;
	ptr = ptr + 1;
	for(i=0;i<2;i++)
	{
		*ptr = (uint8_t)(val & 0xFF);
		ptr--;
		val = val >> 8;
	}
}
// convert ws data struct in string message to send via lora
void str_dWs2300(uint8_t *str)
{
	uint8_t *ptr;
	uint16_t dtWord;
	int dtInt;
	
	ptr = str;
	//	unsigned long idMsg;		[00][01][02][03]    	// message count to transmit
	ulong2bytes(ptr, dWs2300.idMsg);
	ptr = ptr + 4;
	//	unsigned long time;			[04][05][06][07]    	// time of update variables
	ulong2bytes(ptr, dWs2300.time);
	ptr = ptr + 4;
	//	uint16_t SensorId;			[08][09]            	// id of lacrosse ws2300 sensor
	word2bytes(ptr, dWs2300.SensorId);
	ptr = ptr + 2;
	// int WindSpeed;				[10][11]            	// wind speed
	word2bytes(ptr, (uint16_t)dWs2300.WindSpeed);
	ptr = ptr + 2;
	// int WindDirection;			[12]               		// wind direction
	*ptr = dWs2300.WindDirection & 0x0F;
	ptr++;
	// int GustSpeed;				[13][14]            	// Gust speed
	word2bytes(ptr, (uint16_t)dWs2300.GustSpeed);
	ptr = ptr + 2;
	// int GustDirection;			[15]               		// Gust direction
	*ptr = dWs2300.GustDirection & 0x0F;
	ptr++;
	//	float Rainfall;				[16][17]            	// rainfall
	word2bytes(ptr, (uint16_t)dWs2300.Rainfall);
	ptr = ptr + 2;

	//	int Temp;					[18][19]            	// (03) temperature in celsius. Range: 4,44 to 70 C
	word2bytes(ptr, (uint16_t)dWs2300.Temp);
	ptr = ptr + 2;
	//	int Hum;					[20][21]                // % humidity. Range: 1 to 99 %RH
	dtWord = dWs2300.Hum & 0xFF;
	*ptr = dWs2300.Hum & 0xFF;
	if(dtWord > 99)
	dtWord = 99;
	word2bytes(ptr, dtWord);
	ptr = ptr + 2;
	*ptr = BatteryVoltage & 0xFF;
	ptr = ptr + 1;
	// tot: 23 bytes
	dtInt = (int)(Bmp180temp);
	word2bytes(ptr, (uint16_t)dtInt);
	ptr = ptr + 2;
	// tot: 25 bytes
	dtInt = (int)(Bmp180press/10);
	word2bytes(ptr, (uint16_t)dtInt);
	// tot: 27 bytes
}

//-----------------------------------------------------------------
// TTN CONNECTION
//-----------------------------------------------------------------
//
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// https://tutorial.cytron.io/2017/09/15/lesson-1-build-simple-arduino-lora-node-10-minutes/
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
//
// -----------------------------------------------------------------
// registering node to ttn:
// 1) define application
// 2) define node to connect to application
// 3) define Device ID, ex:  bsf0001
// 4) write Description, ex: bsfrance 0001
// 5) set Activation Method: ABP
// 6) get Device Address, 4 bytes, ex: 26011824. 
// 7) set variable DEVADDR with Device Address: static u4_t DEVADDR = 0x26011824;
// 8) get Network session key, 16 bytes, ex: { 0x57, 0xE0, 0xB7, 0xB4, 0x1C, 0x0A, 0x4E, 0xF8, 0x7C, 0xA1, 0xB0, 0x97, 0x8E, 0x9A, 0x69, 0xF6 }
// 9) set array static u1_t NWKSKEY[16] with Network session key
// 10) get App Session Key, 16 bytes, ex: { 0xF3, 0x37, 0xB0, 0x4E, 0x2E, 0x36, 0xAE, 0x0F, 0x4D, 0x42, 0x8B, 0xB8, 0xBF, 0x22, 0x67, 0x34 }
// 11) set static u1_t APPSKEY[16] with App Session Key
// -----------------------------------------------------------------

// format of msg transmitted
//	unsigned long idMsg;		[00][01][02][03]    	// message count to transmit
//	unsigned long time;			[04][05][06][07]    	// time of update variables
//	uint16_t SensorId;			[08][09]            	// id of lacrosse ws2300 sensor
//	float WindSpeed;		[10][11]            	// (01) windspeed in km/hour. Range: 0.0 to 159.9 kph
//	int WindDirection;		[12]                	// windspeed direction. Range: 0..15
//	float Rainfall; 			[13][14]            	// (02) rainfall. Range: -327.68 (0) to 327.67
//	int ActiveRain;				[15]                	// flag for active rainfall
//	unsigned long RainTmSpan;	[16][17][18][19]    	// time start rain
//	float Temp;					[20][21]            	// (03) temperature in celsius. Range: 4,44 to 70 C
//	int Hum;					[22]                	// % humidity. Range: 1 to 99 %RH
//	int BattLow;				[23]                	// flag battery low

// (01) Range: 0.0 to 159.9 kph. the float is multiplied by 10 and converted into an integer
// (02) Range: 0 to 327.67. the float is multiplied by 100 and converted into an integer
// (03) Range: 4,44 to 70 C. the float is multiplied by 100 and converted into an integer

#define SIZE_MSGWS		27

uint8_t msgStr_dWs[SIZE_MSGWS];

/*************************************
* TODO: Change the following keys
* NwkSKey: network session key, AppSKey: application session key, and DevAddr: end-device address
*************************************/
// follow the tutorial in
// https://tutorial.cytron.io/2017/09/15/lesson-1-build-simple-arduino-lora-node-10-minutes/

// --------------------------------------------
// TTN configuration keys
// see:
// https://primalcortex.wordpress.com/2017/10/31/ttnlorawan32u4node/
// https://www.thethingsnetwork.org/docs/devices/registration.html#personalize-device-for-abp
// https://github.com/fcgdam/TTN32u4ABP
//
// 
// Define TTN_ACCESS_OTAA if you wanto to access using OTAA method else use ABP if undefined
// #define		TTN_ACCESS_OTAA


#ifdef TTN_ACCESS_OTAA
// -------------------------------------------------------------
// KEYS FOR OTAA ACCESS
// LoRaWAN NwkSKey, network session key
static u1_t NWKSKEY[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D  0x0E, 0x0F };

// LoRaWAN AppSKey, application session key
static u1_t APPSKEY[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D  0x0E, 0x0F };

// LoRaWAN end-device address (DevAddr)
static u4_t DEVADDR = 0x12345678;   // Put here the device id in hexadecimal form.

#else
// -------------------------------------------------------------
// KEYS FOR ABP ACCESS

// Application ID: demo_ttn
// LoRaWAN NwkSKey, network session key
static u1_t NWKSKEY[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D  0x0E, 0x0F };

// LoRaWAN AppSKey, application session key
static u1_t APPSKEY[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D  0x0E, 0x0F };

// LoRaWAN end-device address (DevAddr)
static u4_t DEVADDR = 0x12345678;   // Put here the device id in hexadecimal form.

#endif			// #ifdef TTN_ACCESS_OTAA

// --------------------------------------------

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { };
void os_getDevEui (u1_t* buf) { };
void os_getDevKey (u1_t* buf) { };

static osjob_t sendjob;
static osjob_t initjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 300;

// Pin mapping
// These settings are for the user made board with the Atmega32u4 shield.

// see:
// https://github.com/matthijskooijman/arduino-lmic/issues/162
// In Atmega32u4 I had to solder DIO1 to D1. 
// For reference, DIO1 is located on the back side of the board.
// Here’s the LMIC pin mapping:
// 
const lmic_pinmap lmic_pins = {
	.nss = 8,
	.rxtx = LMIC_UNUSED_PIN,
	.rst = 4,
	.dio = {7, 1, 2},
};

// send message to lorawan
void do_send(osjob_t* j)
{
	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND) 
	{
		// Serial.println(F("OP_TXRXPEND, not sending"));
		Serial.println("OP_TXRXPEND, not sending");
	} 
	else 
	{
		// Prepare upstream data transmission at the next possible time.
		
		str_dWs2300(msgStr_dWs);				// form lora message
		// send lora message
		LMIC_setTxData2(1, msgStr_dWs, SIZE_MSGWS, 0);
		// print to serial terminal message
#ifdef SERIAL_MSG
		// transform in hex string and send to serial terminal
		PrintHex8(&msgStr_dWs[0], SIZE_MSG5N1);
#endif
		
		incIdMsg();							// increment msgId
		
#ifdef SERIAL_MSG
		Serial.println();
		
		Serial.println("Sending uplink packet...");
#endif					// #ifdef SERIAL_MSG

		// Lit up the led to signal transmission . Should not be used if aiming to save power...
	}
	// Next TX is scheduled after TX_COMPLETE event.
}

// from: https://github.com/DiederikRhee/ProMini_Lora_Otaa_temperture
// initial job
static void initfunc (osjob_t* j) {
	// reset MAC state
	LMIC_reset();
}

// from: https://github.com/DiederikRhee/ProMini_Lora_Otaa_temperture
void onEvent (ev_t ev) {
	switch (ev) {
	case EV_SCAN_TIMEOUT:
		break;
	case EV_BEACON_FOUND:
		break;
	case EV_BEACON_MISSED:
		break;
	case EV_BEACON_TRACKED:
		break;
	case EV_JOINING: 
		break;
	case EV_JOINED:
		do_send(&sendjob);
		joined = true;
		break;
	case EV_RFU1:
		break;
	case EV_JOIN_FAILED:
		break;
	case EV_REJOIN_FAILED:
		// Re-init
		os_setCallback(&initjob, initfunc);      
		break;
	case EV_TXCOMPLETE:
		sleeping = true;
		break;
	case EV_LOST_TSYNC:
		break;
	case EV_RESET:
		break;
	case EV_RXCOMPLETE:
		break;
	case EV_LINK_DEAD:
		break;
	case EV_LINK_ALIVE:
		break;
	default:
		break;
	}
}

// start connection to TTN. This function must call by setup()
void InitTTNConnection()
{
	// LMIC init
	os_init();
	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

	
	// Set up the channels used by the Things Network, which corresponds
	// to the defaults of most gateways. Without this, only three base
	// channels from the LoRaWAN specification are used, which certainly
	// works, so it is good for debugging, but can overload those
	// frequencies, so be sure to configure the full frequency range of
	// your network here (unless your network autoconfigures them).
	// Setting up channels should happen after LMIC_setSession, as that
	// configures the minimal channel set.
	// NA-US channels 0-71 are configured automatically
	LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
	LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
	// TTN defines an additional channel at 869.525Mhz using SF9 for class B
	// devices' ping slots. LMIC does not have an easy way to define set this
	// frequency and support for class B is spotty and untested, so this
	// frequency is not configured here.

	// Set static session parameters.
	LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// TTN uses SF9 for its RX2 window.
	LMIC.dn2Dr = DR_SF9;

	// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(DR_SF9, 14);

	// Start job
	do_send(&sendjob);
}

//-----------------------------------------------------------------
// end ttn utilities
//-----------------------------------------------------------------

state_program StMain = SP_START;			// state program
unsigned long tmDelay;
unsigned long tmTimeout;

#define LEDPIN 13
#if BSFRANCEBOARD
// note on:
// https://docs.bsfrance.fr/documentation/11355_LORA32U4II/LoRa32u4II_pinout_diagram.pdf
// lipo battery voltage: D9 (ADC12)
// nota: se si vede pins_arduino.h:
// static const uint8_t A9 = 27;	// D9
#define VBATPIN A9
#endif				// #if BSFRANCEBOARD


// The BSFrance Lora32U4 board has a voltage divider from the LiPo connector that allows
// to measure battery voltage.
// note on:
// https://docs.bsfrance.fr/documentation/11355_LORA32U4II/LoRa32u4II_pinout_diagram.pdf
// lipo battery voltage: D9 (ADC12)
//
#if BSFRANCEBOARD
float Original_getBatVoltage() {
	float measuredvbat = analogRead(VBATPIN);
	measuredvbat *= 2;    // Voltage is devided by two by bridge resistor so multiply back
	measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
	measuredvbat /= 1024; // convert to voltage
	Serial.print("VBat: " ); Serial.println(measuredvbat);
	return measuredvbat;
}

// new version 
unsigned int getBatVoltage() {
	unsigned int measuredvbat;
	measuredvbat = analogRead(VBATPIN);		// analogread return int(0 to 1023)
	measuredvbat *= 33;  					// Multiply by 3.3V, our reference voltage
	// Voltage is devided by two by bridge resistor so multiply * 2 and later divide per 1024.
	// Join the 2 operations, divide per 512 (shift by 9)
	// limits measuredvbat: 0 .. 33791
	// measuredvbat /= 1024; // convert to voltage
	measuredvbat = measuredvbat >> 9;		// convert to voltage, divide for 512 (moltiply for 2 and divide for 1024)
	// Serial.print("VBat: " ); Serial.println(measuredvbat);
	return measuredvbat;
}
#endif			// #if BSFRANCEBOARD

//-----------------------------------------------------------------
// watchdog management
//-----------------------------------------------------------------

#ifdef ENABLE_WATCHDOG
// ----------------------------------------------
// WATCHDOG ENABLED
//
// active watchdog with 8sec timing
void wdtEnable(void)
{
	wdt_enable(WDTO_8S);	// enable watchdog with 8sec timer
}
// deactive watchdog timer
void wdtDisable(void)
{
	wdt_disable();			// disable watchdog
}
// reset watchdog	
void wdtReset()
{
	wdt_reset();			// reset watchdog
}

#else
// ----------------------------------------------
// WATCHDOG DISABLED
// active watchdog with 8sec timing
void wdtEnable(void)
{
}
// deactive watchdog timer
void wdtDisable(void)
{
}
// reset watchdog	
void wdtReset()
{
}

#endif						// #ifdef ENABLE_WATCHDOG

// use watchdog to restart arduino
void ResetSystem()
{
	wdt_enable(WDTO_15MS);	// enable watchdog with 15msec timer
	while(1);
}

//-----------------------------------------------------------------
// led commands
//-----------------------------------------------------------------
//
// Blink LED
void BlinkLed(int DELAY_MS) 
{
	pinMode(LEDPIN, OUTPUT);
	digitalWrite(LEDPIN, HIGH);
	delay(DELAY_MS);
	digitalWrite(LEDPIN, LOW);
	delay(DELAY_MS);
}

//-----------------------------------------------------------------
// rx Ws2300
//-----------------------------------------------------------------
//

// Arduino code decode several Acurite devices OTA data stream.
// The arduino uses RXB6 etherodina interface:
// https://www.amazon.it/WINGONEER-Superheterodyne-Wireless-Receiver-Arduino/dp/B06XHJMC82
// http://aitendo3.sakura.ne.jp/aitendo_data/product_img/wireless/315MHz-2012/ASK-RXB6/RXB6_aitendo.pdf
// -----------------------------------------------------------
//
// module configuration
//
// define PRTDATA if you want to print data
// #define PRTDATA

// defines for lacrosse modules managed by software
//
// types of arduino provided by the module
#define ARD_UNO				0				// arduino UNO
#define ARD_BSFLORA32U4		1				// bsfrance lora f32u4

// define module type used
#define	MOD_ARDUINO			ARD_BSFLORA32U4

// for lacrosse. see lacrossews.c, r_device lacrossews
#define PULSE_LONG      3000
#define PULSE_SHORT     950

#define PULSE_RANGE     100

// 1 bit ~0.390 msec high followed by ~1.320 msec low
// 0 bit ~1.550 msec high followed by ~1.280 msec low

//BIT1: ~0.350 msec high followed by ~1.320 msec low
#define BIT1_HIGH_MIN  (300)
#define BIT1_HIGH_MAX  (600)
// for lacrosse only interested in counting the duration of positive edges.
// After sending the front of the last bit of the message, the signal remains at 0 until it is received
// of the next message (or up to the first completely random bit)
// #define BIT1_LOW_MIN   (1300)
// #define BIT1_LOW_MAX   (1400)

//BIT0: ~1.550 msec high followed by ~1.280 msec low
#define BIT0_HIGH_MIN  (1300)
#define BIT0_HIGH_MAX  (1600)

// limits
#define BIT_HIGH_MIN  (300)
#define BIT_HIGH_MAX  (1600)

#define BIT_LOW_MIN   (1100)
#define BIT_LOW_MAX   (1400)

// bit low that signal end of msg, in usec
// original
// #define BIT0_ENDOFMSG (140000)
// test
#define BIT0_ENDOFMSG (110000)

// bit low end of sequence messages
#define BIT0_ENDSEQMSG (350000)


//-----------------------------------------
// pinout:
// http://bsfrance.fr/documentation/11355_LORA32U4II/LoRa32u4II_pinout_diagram.pdf
// blue column: number used to identify pin with pinMode
// int column: interrupt associated with the pin
//-----------------------------------------
//
// --------------------------------------------------------------------------------------
// INTERRUPT CONFIGURATION
//
// define USE_EXTERNAL_INTERRUPT if you want to use external pin interrupt
// If not defined, is used pin change interrupt
// https://playground.arduino.cc/Code/Interrupts
// Arduino has two different kinds of interrupts: external, and "pin change". 
// There are only two external interrupt pins, INT0 and INT1, and they are mapped to Arduino pins 2 and 3. 
// These interrupts can be set to trigger on RISING or FALLING signal edges, or on low level.
// https://playground.arduino.cc/Main/PinChangeInterrupt
// The interrupt can be enabled for each pin individually (analog and digital!), 
// but there are only 3 interrupt vectors, so 6-8 pins share one service routine:
// ISR (PCINT0_vect) pin change interrupt for D8 to D13
// ISR (PCINT1_vect) pin change interrupt for A0 to A5
// ISR (PCINT2_vect) pin change interrupt for D0 to D7
// The interrupt fires on a change of the pin state (rising and falling). 
// For more information please refer to the excellent tutorials http://gammon.com.au/interrupts.
// Interrupt Pins should be set as INPUT, pullup resistors can be enabled to be able to detect simple switches
//   pinMode(i,INPUT);   // set Pin as Input (default)
//   digitalWrite(i,HIGH);  // enable pullup resistor
// --------------------------------------------------------------------------------------
// #define USE_EXTERNAL_INTERRUPT

// --------------------------------------------------------------------------------------

#if (defined(USE_EXTERNAL_INTERRUPT))

// On arduino connect the data pin, the pin that will be 
// toggling with the incoming data from the RXB6 RF module, to
// digital pin RXB6_DATA. 
// INT_RXB6_DATA is the interrupt of digital pin RXB6_DATA
// It can be configured for interrupt on change, change to high or low.
// The squelch pin in an input to the radio that squelches, or
// blanks the incoming data stream. 
// Use the squelch pin to stop the data stream and prevent interrupts 
// between the data packets if desired.
//
#if (defined(MOD_ARDUINO) && MOD_ARDUINO == ARD_UNO)
// ------------------------------------
// specific defines for module ARDUINO UNO
// RXB6 datapin
#define RXB6_DATA			(3)             // D3 is interrupt 1
#define INT_RXB6_DATA		(1)				// interrupt 1 for D3

#elif (defined(MOD_ARDUINO) && MOD_ARDUINO == ARD_BSFLORA32U4)
// ------------------------------------
// specific defines for module BSFRANCE
// RXB6 datapin
#define RXB6_DATA			(0)				// This arduino pin is connected to rtc INT/SQW (pin3 rtc)
#define INT_RXB6_DATA		(2)				// interrupt 2 for RXB6_DATA
// #define RXB6_DATA		(1)             // D3 is interrupt 3
// #define INT_RXB6_DATA	(3)				// interrupt 3 for D3

#endif						// #if (defined(MOD_ARDUINO) && MOD_ARDUINO == ARD_UNO)

// -------------------------------------------
#else			// #define USE_EXTERNAL_INTERRUPT
// see:
// https://docs.bsfrance.fr/documentation/11355_LORA32U4II/LoRa32u4II_pinout_diagram.pdf
// for pin change interrupt
// original
// #define RXB6_DATA			(9)				// pin connected to PB5 PCINT5
// modified 02/12/2018:
// to use lipo battery monitor, we changed the interrupt from pin 9 to 10
#define RXB6_DATA			(10)				// pin connected to PB6 PCINT6

#endif			// #define USE_EXTERNAL_INTERRUPT

#define SYNCPULSECNT    (4)             // 4 pulses (8 edges)
#define SYNCPULSEEDGES  (SYNCPULSECNT*2)

#define DATABYTESCNT_MIN (7) // Minimum number of data bytes
#define DATABITSCNT_MIN     (DATABYTESCNT_MIN*8)// 7 bytes * 8 bits
#define DATABITSEDGES_MIN   (DATABITSCNT_MIN*2)

#define DATABYTESCNT_MID 128 	// 8 Bytes

#define DATABYTESCNT_MAX (9) 	// 9 Bytes
#define DATABITSCNT_MAX     (DATABYTESCNT_MAX*8)// 7 bytes * 8 bits
#define DATABITSEDGES_MAX   (DATABITSCNT_MAX*2)

// Ws2300 Tower Message Types
#define  MT_WS_WD_RF  49    // wind speed, wind direction, rainfall
#define  MT_WS_T_RH   56    // wind speed, temp, RH

// macros from DateTime.h 
// Useful Constants 
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

// Useful Macros for getting elapsed time
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)  

// ===================================================================================================
// aux utils
// ===================================================================================================
// 
// helper code to print formatted hex 
// 
void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex
{
	char tmp[length*2+1];
	byte first;
	int j = 0;
	for (uint8_t i = 0; i < length; i++) 
	{
		first = (data[i] >> 4) | 48;
		if (first > 57) tmp[j] = first + (byte)39;
		else tmp[j] = first ;
		j++;

		first = (data[i] & 0x0F) | 48;
		if (first > 57) tmp[j] = first + (byte)39; 
		else tmp[j] = first;
		j++;
	}
	tmp[length*2] = 0;
	Serial.print(tmp);
}

// ===================================================================================================
// INTERRUPT MANAGEMENT OF RXB6 ETHERODINA DATA
// ===================================================================================================

// 16/07/2018: 
// initialize acquisition ws messages
void Rxb6InitializeMsgAcquisition()
{
	dataRx = 0x00;
	pulseCount = 0;  		// restart looking for data bits
	received = false;
}


// in base of msg_type decode the values
void DecodeLacrosseData(int state)
{
	switch (msg_type) 
	{
		// Temperature: 3 nibble bcd
	case 0:
		// msg_value
		if(state == 0)
		{
			msg_value = ((int)msg[LV1])*10;
		}
		else if(state == 1)
		{
			msg_value = (msg_value + ((int)msg[LV2])) * 10;
		}
		else if(state == 2)
		{
			msg_value = (msg_value + ((int)msg[LV3])) - 300;
		}
		break;
		// Humidity: 2 nibble bcd
	case 1:
		// msg_value
		if(state == 0)
		{
			msg_value = ((int)msg[LV1])*10;
		}
		else if(state == 1)
		{
			msg_value = (msg_value + ((int)msg[LV2]));
		}
		else if(state == 2)
		{
			if(msg_value >= 110)
			{
				// Humidity Error
				msg_value = 0xAA;
			}	
		}
		break;
		// Rain: 2 nibble bin
	case 2:
		// msg_value
		if(state == 0)
		{
			msg_value = ((int)msg[LV1]) << 4;
		}
		else if(state == 1)
		{
			msg_value = (msg_value + ((int)msg[LV2])) << 4;
		}
		else if(state == 2)
		{
			msg_value = (msg_value + ((int)msg[LV3]));
		}
		// to obtain rain_mm:
		// rain_mm = 0.5180 * msg_value;
		break;
		// Wind speed: 2 nibble bin
	case 3:
		// Gust speed: 2 nibble bin
	case 7:
		// wind dir, (for msg=3 -> wind speed, msg=7 -> gust speed)
		if(state == 0)
		{
			wind_spd = ((int)msg[LV1]) << 4;
		}
		else if(state == 1)
		{
			wind_spd = (wind_spd + ((int)msg[LV2]));
			// if wind_spd 0xFE, LaCrosse WS not Connected
		}
		else if(state == 2)
		{
			// wind_dir in degrees: wind_dir * 22.5;
			wind_dir = (int)msg[LV3];
		}
		break;
	}
}

bool StoreLacrosseData(void)
{
#ifdef RECEIVE_ONLY_1_ID
	if(sensor_id != idWeatherStation)
	{
		// msg received from a different weather station
		return false;
	}
#endif
	
	// dWs2300.time = millis();
	dWs2300.time = micros();
	dWs2300.SensorId = sensor_id;
	dWs2300.msg_type = msg_type;
	dWs2300.tmMsg = tmMsg;
	switch (msg_type) 
	{
	case 0:				// Temperature: 3 nibble bcd
		DataMsgWsFull = DataMsgWsFull | 0x01;
		dWs2300.Temp = msg_value;
		break;
	case 1:				// Humidity: 2 nibble bcd
		// msg_value
		DataMsgWsFull = DataMsgWsFull | 0x02;
		dWs2300.Hum = msg_value;
		break;
	case 2:				// Rain: 2 nibble bin
		// msg_value
		// to obtain rain_mm:
		// rain_mm = 0.5180 * msg_value;
		DataMsgWsFull = DataMsgWsFull | 0x04;
		// OK !!!!! dWs2300.Rainfall = ((float)msg_value) * 0.5180;
		// TEST PER VEDERE SE CAMBIA QUALCOSA
		dWs2300.Rainfall = msg_value;
		break;
	case 3:				// Wind speed: 2 nibble bin
		DataMsgWsFull = DataMsgWsFull | 0x08;
		dWs2300.WindSpeed = wind_spd          ;
		// dWs2300.WindDirection = windDir2Deg[wind_dir & 0xF]      ;
		dWs2300.WindDirection = wind_dir      ;
		break;
	case 7:				// Gust speed: 2 nibble bin
		// wind dir, (for msg=3 -> wind speed, msg=7 -> gust speed)
		DataMsgWsFull = DataMsgWsFull | 0x10;
		dWs2300.GustSpeed = wind_spd          ;
		// dWs2300.GustDirection = windDir2Deg[wind_dir & 0xF]      ;
		dWs2300.GustDirection = wind_dir      ;
		break;
	default:
		return false;
	}
	return true;
}

// update dWs2300 data with dWsRx
bool UpdateLacrosseData(int idx)
{
#ifdef RECEIVE_ONLY_1_ID
	if(dWsRx[idx].SensorId != idWeatherStation)
	{
		// msg received from a different weather station
		return false;
	}
#endif
	
	dWs2300.time = dWsRx[idx].time;
	dWs2300.SensorId = dWsRx[idx].SensorId;
	dWs2300.msg_type = dWsRx[idx].msg_type;
	dWs2300.tmMsg = dWsRx[idx].tmMsg;
	switch (dWsRx[idx].msg_type) 
	{
	case 0:				// Temperature: 3 nibble bcd
		DataMsgWsFull = DataMsgWsFull | 0x01;
		dWs2300.Temp = dWsRx[idx].Temp;
		break;
	case 1:				// Humidity: 2 nibble bcd
		DataMsgWsFull = DataMsgWsFull | 0x02;
		dWs2300.Hum = dWsRx[idx].Hum;
		break;
	case 2:				// Rain: 2 nibble bin
		// to obtain rain_mm:
		// rain_mm = 0.5180 * msg_value;
		DataMsgWsFull = DataMsgWsFull | 0x04;
		dWs2300.Rainfall = dWsRx[idx].Rainfall;
		break;
	case 3:				// Wind speed: 2 nibble bin
		DataMsgWsFull = DataMsgWsFull | 0x08;
		dWs2300.WindSpeed = dWsRx[idx].WindSpeed          ;
		dWs2300.WindDirection = dWsRx[idx].WindDirection      ;
		break;
	case 7:				// Gust speed: 2 nibble bin
		// wind dir, (for msg=3 -> wind speed, msg=7 -> gust speed)
		DataMsgWsFull = DataMsgWsFull | 0x10;
		dWs2300.GustSpeed = dWsRx[idx].GustSpeed          ;
		dWs2300.GustDirection = dWsRx[idx].GustDirection      ;
		break;
	default:
		return false;
	}
	return true;
}

// RXB6 Data Interrupt handler 
// Tied to RXB6_DATA arduino digital input.
// Set to interrupt on edge (level change) high or low transition.
// Change the state of the Arduino LED (pin 13) on each interrupt. 
// This allows scoping pin 13 to see the interrupt / data pulse train.
// 
void Rxb6InterruptHandler() 
{
	static unsigned long Rxb6BitDuration = 0;
	static unsigned int bitState  = 0;

	bitState = digitalRead(RXB6_DATA);

	// ignore if we haven't finished processing the previous 
	// received signal in the main loop.
	if( received == true )
	{
		// the main loop work on the message buffer has not yet completed
		return;
	}

	// calculating timing since last change
	unsigned long time = micros();
	
	if(Rxb6IntLastTime == 0L)
	{
		// initialize the bit timing acquisition
		Rxb6IntLastTime = time;
		return;
	}
	
	Rxb6BitDuration = time - Rxb6IntLastTime;
	Rxb6IntLastTime = time;

	if( bitState == LOW )
	{
		// ---------------------------------------------------------
		// signal transition: 0 -> 1 -> 0
		// 1   +--------+   1
		//     |        |
		// 0 --+        +-- 0
		// check duration of 0 -> 1 -> 0
		// 
		// Known error in bit stream is runt/short pulses.
		// If we ever get a really short, or really long pulse, we know there is an error in the bit stream
		// and should start over.
		// check if errMsg was previously set. In this case, wait a low pulse for end msg
		if(errMsg)
		{
			// wait a low pulse with duration >= 148msec for end msg
			return;
		}
		// errMsg = false. Check duration of positive transition
		if( (Rxb6BitDuration >= BIT0_HIGH_MIN) && (Rxb6BitDuration <= BIT0_HIGH_MAX) )
		{
			// set bit in dataRx
			dataRx = dataRx << 1;
		}
		else if( (Rxb6BitDuration >= BIT1_HIGH_MIN) && (Rxb6BitDuration <= BIT1_HIGH_MAX) )
		{
			// set bit in dataRx
			dataRx = dataRx << 1;
			dataRx = dataRx | 1;
		}
		else
		{
			// the pulse duration is out of range
			// if(pulseCount>0)
			if(pulseCount>3)
			{
				errMsg = true;
				Rxb6InitializeMsgAcquisition();				// initialize acquisition ws messages
			}
			return;
		}
		
		pulseCount++; // found another edge
		
		// if not enough bits yet, no message received yet
		// if( pulseCount < DATABITSCNT_MAX )
		// {
		// 	received = false;
		// }
		// errMsg = false;
		// runtime decode msg, in base of value of pulseCount
		switch(pulseCount)
		{
		case 1:
			tmMsg = Rxb6IntLastTime;
			break;
			// -------------------------------------
			// per test perdita primo messaggio
		case 4:
		case 5:
		case 6:
		case 7:
			if(dataRx != 0x09)
			{
				break;
			}
			// e' molto probabile che siano stati persi i primi bit
			pulseCount = 8;
			// ... continua con lo stato successivo
			// -------------------------------------
		case 8:
			// nibble 0,1
			if(dataRx != 0x09)
			{
				errMsg = true;
				break;
			}
			// msg[L01] = dataRx;
			checksum = 0x09;
			dataRx = 0x00;
			break;
		case 12:
			// nibble 2: GPTT
			msg[LGPTT] = dataRx;
			checksum += dataRx;
			msg_type = ((dataRx >> 1) & 0x4) + (dataRx & 0x3);
			dataRx = 0x00;
			break;
		case 16:
			// nibble 3: Id hi
			checksum += dataRx;
			break;
		case 20:
			// nibble 4: Id low
			msg[LID] = dataRx;
			sensor_id = dataRx; 
			checksum += (dataRx & 0x0F);
			dataRx = 0x00;
			break;
		case 24:
			// 20-23	5 - Data Types  GWRH  G=Gust Sent, W=Wind Sent, R=Rain Sent, H=Humidity Sent
			msg[LGWRH] = dataRx;
			checksum += dataRx;
			dataRx = 0x00;
			break;
		case 28:
			// 24-27	6 - Parity TUU? T=Temp Sent, UU=Next Update, 00=8 seconds, 01=32 seconds, 10=?, 11=128 seconds, ?=?
			msg[LTUU] = dataRx;
			msg_data = (msg[LGWRH] << 1) + (msg[LTUU] >> 3);
			msg_unknown = msg[LTUU] & 0x01;
			checksum += dataRx;
			dataRx = 0x00;
			break;
		case 32:
			// 28-31	7 - Value1
			msg[LV1] = dataRx;
			checksum += dataRx;
			dataRx = 0x00;
			break;
		case 36:
			// 32-35	8 - Value2
			msg[LV2] = dataRx;
			checksum += dataRx;
			dataRx = 0x00;
			break;
		case 40:
			// 36-39	9 - Value3
			msg[LV3] = dataRx;
			checksum += dataRx;
			dataRx = 0x00;
			break;
		case 44:
			// 40-43	10 - ~Value1
			if(dataRx != (~msg[LV1] & 0xF))
			{
				errMsg = true;
				break;
			}
			// msg[LCV1] = dataRx;
			checksum += dataRx;
			dataRx = 0x00;
			break;
		case 48:
			// 44-47	11 - ~Value2
			if(dataRx != (~msg[LV2] & 0xF))
			{
				errMsg = true;
				break;
			}
			// msg[LCV2] = dataRx;
			checksum += dataRx;
			dataRx = 0x00;
			break;
		case 52:
			// 48-51	12 - Check Sum = Nibble sum of nibbles 0-11
			tmMsg = Rxb6IntLastTime - tmMsg;
			if(dataRx != (checksum & 0xF))
			{
				errMsg = true;
				break;
			}
			// msg[LCHK] = dataRx;
			received = true;
			if(StoreLacrosseData() == false)
			{
				errMsg = true;
			}
			break;
		default:
			received = false;
			if(pulseCount>52)
			{
				errMsg = true;
			}
			break;
		}
		if(errMsg == true)
		{
			// error msg
			Rxb6InitializeMsgAcquisition();				// initialize acquisition ws messages
			return;
		}

		//
		// END signal transition: 0 -> 1 -> 0
		// ---------------------------------------------------------
	}
	else
	{
		// signal transition: 1 -> 0 -> 1
		// 1 --+        +-- 1
		//     |        |
		// 0   +--------+   0
		// check duration of 1 -> 0 -> 1
		// 

		// In this case, check if the low pulse is greater than 148msec
		// low pulse with duration >= 148msec for end msg
		if(Rxb6BitDuration > BIT0_ENDOFMSG)
		{
			// 
			countEndOfMsg++;
			errMsg = false;
			Rxb6InitializeMsgAcquisition();
			return;
		}
		// NO END OF MSG.
		// check if errMsg was previously set. 
		if( (Rxb6BitDuration < BIT_LOW_MIN) || (Rxb6BitDuration > BIT_LOW_MAX) )
		{
			if(pulseCount>3)
			{
				errMsg = true;
				Rxb6InitializeMsgAcquisition();				// initialize acquisition ws messages
				return;
			}
			
		}
		// manage the second part of state
		switch(pulseCount)
		{
		case 32:
			// 28-31	7 - Value1
			DecodeLacrosseData(0);
			break;
		case 36:
			// 32-35	8 - Value2
			DecodeLacrosseData(1);
			break;
		case 40:
			// 36-39	9 - Value3
			DecodeLacrosseData(2);
			msg[LV3] = dataRx;
			checksum += dataRx;
			dataRx = 0x00;
			break;
		}
		
		return;
		//
		// END signal transition: 1 -> 0 -> 1
		// ---------------------------------------------------------
	}
}

#if (defined(USE_EXTERNAL_INTERRUPT))
// ----------------------------------------------------------
// use external pin interrupt int2 (pin0) or int3 (pin1)

void Rxb6AttachInterrupt()
{
	cli();						// disable interrupt
	attachInterrupt(INT_RXB6_DATA, Rxb6InterruptHandler, CHANGE);
	sei();						// enable interrupt
}
// disable interrupt to avoid new data
void Rxb6DetachInterrupt()
{
	cli();						// disable interrupt
	detachInterrupt(INT_RXB6_DATA);
	sei();						// enable interrupt
}

#else
// ---------------------------------------------------------
// CURRENTLY, THE SYSTEM USES THE PIN CHANGE INTERRUPT
// ----------------------------------------------------------
// Pin Change Interrupt management
// https://playground.arduino.cc/Main/PinChangeInterrupt
// The interrupt can be enabled for each pin individually, but there are only 3 interrupt vectors, so 6-8 pins share one service routine:
// ISR (PCINT0_vect) pin change interrupt for D8 to D13
// ISR (PCINT1_vect) pin change interrupt for A0 to A5
// ISR (PCINT2_vect) pin change interrupt for D0 to D7

// Install Pin change interrupt for a pin, can be called multiple times
void attachPinChangeInterrupt(byte pin)
{
	cli();						// disable interrupt
	*digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
	PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
	PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
	sei();						// enable interrupt
}

// https://arduino.stackexchange.com/questions/20667/detaching-interrupts
// detach Pin change interrupt for a pin
void detachPinChangeInterrupt(byte pin)
{
	cli();						// disable interrupt
	PCICR  &= ~bit (digitalPinToPCICRbit(pin));
	sei();						// enable interrupt
}

// ------------------------------------------------------------
// interrupt vectors for pin change
// https://playground.arduino.cc/Main/PinChangeInterrupt

// interrupt vector for Port B, PCINT0 - PCINT7
// handle pin change interrupt for D8 to D13 here
ISR(PCINT0_vect)
{
	Rxb6InterruptHandler();
}    // end of isr pin change Port B, PCINT0 - PCINT7

// interrupt vector for Port C, PCINT8 - PCINT14
// handle pin change interrupt for A0 to A5 here
ISR(PCINT1_vect)
{
}    // end of isr pin change Port C, PCINT8 - PCINT14

// interrupt vector for Port D, PCINT16 - PCINT23
// handle pin change interrupt for D0 to D7 here
ISR(PCINT2_vect)
{
}    // end of isr pin change Port D, PCINT16 - PCINT23

void Rxb6AttachInterrupt()
{
	attachPinChangeInterrupt(RXB6_DATA);
}
// disable interrupt to avoid new data
void Rxb6DetachInterrupt()
{
	detachPinChangeInterrupt(RXB6_DATA);
}

#endif				// #ifndef USE_EXTERNAL_INTERRUPT

// END INTERRUPT MANAGEMENT
// ===================================================================================================

//-----------------------------------------------------------------

bool DecodeEthManager()
{
	// ---------------------------------------------------
	wdtReset();				// reset watchdog	
	// ---------------------------------------------------

	if( received == true )
	{
		// disable interrupt to avoid new data corrupting the buffer
		// pinMode(RXB6_DATA, INPUT);          	// data interrupt input

		// Rxb6Power(0);								// off power rxb6 433Mhz Receiver
		// disable interrupt to avoid new data corrupting the buffer
		// --------------------------------------------------------------------------------------
		// Rxb6DetachInterrupt();
		// --------------------------------------------------------------------------------------

		// #define DISPLAY_BIT_TIMING 
#ifdef DISPLAY_BIT_TIMING
		displayBitTiming();
#endif // DISPLAY_BIT_TIMING

		// set time in ws data struct
		dWs2300.time = millis();

		// delay for 1 second to avoid repetitions
		// delay(1000);
		Rxb6InitializeMsgAcquisition();				// initialize acquisition ws messages

		// re-enable interrupt
		// pinMode(RXB6_DATA, INPUT);          	// data interrupt input
		// --------------------------------------------------------------------------------------
		// Rxb6AttachInterrupt();
		// --------------------------------------------------------------------------------------
		// Rxb6Power(1);							// ON power rxb6 433Mhz Receiver
		return true;
	}
	Rxb6Power(1);								// ON power rxb6 433Mhz Receiver
	return false;
}


void StateManager()
{
	unsigned long time;
	int i;
	
	// ---------------------------------------------------
	wdtReset();				// reset watchdog	
	// ---------------------------------------------------
	
	switch(StMain)
	{
	case SP_START			:
		Rxb6Power(0)			;		// off etherodina
		// disable interrupt to avoid new data corrupting the buffer
		pinMode(RXB6_DATA, INPUT);          	// data interrupt input
		// disable interrupt to avoid new data corrupting the buffer
		// --------------------------------------------------------------------------------------
		Rxb6DetachInterrupt();
		// --------------------------------------------------------------------------------------
		StMain = SP_ON_ETH			;
		// disabled StMain = SP_JOINED_TTN			;
		break;
	case SP_JOINED_TTN		:
		// state disabled
		// ---------------------------------------------------
		wdtReset();				// reset watchdog	
		// ---------------------------------------------------
		if (joined==false)
		{
			os_runloop_once();
			break;
		}
		StMain = SP_ON_ETH			;
		break;
		
	case SP_ON_ETH			:
		// switch on etherodina and wait 500msec
		Rxb6Power(1)			;		// on etherodina
		tmDelay = millis();
		StMain = SP_WAIT01			;
		break;
	case SP_WAIT01			:
		// ---------------------------------------------------
		wdtReset();				// reset watchdog	
		// ---------------------------------------------------
		time = millis();
		// wait
		if((time - tmDelay) < 100L)
		break;
		// --------------------------------------
		// start etherodina rx
		// initialize acquisition messages
		nMsg = 0;								// n. msg rx
		timeRxb6LastMsg = 0L;					// time rx last msg in usec
		Rxb6IntLastTime = 0L;					// initialize the bit timing acquisition
		
		Rxb6InitializeMsgAcquisition();
		DataMsgWsFull = 0x00;
		// limitTimeout = 10000L;		// initialize timeout limit for etherodina		
		// enable interrupt etherodina
		pinMode(RXB6_DATA, INPUT);          	// data interrupt input
		// --------------------------------------------------------------------------------------
		Rxb6AttachInterrupt();
		// --------------------------------------------------------------------------------------
		timeRxb6LastMsg = time;

		tmTimeout = millis();				// timeout rx
		StMain = SP_DECODE_ETH		;
		break;
	case SP_DECODE_ETH		:
		// wait until a message is received 
		

		time = micros();
		if( received == true )
		{
			cli();						// disable interrupt
			timeRxb6LastMsg = time;

			dWsRx[nMsg] = dWs2300;
			nMsg++;
			if(nMsg>=10)
			nMsg = 0;
			dataRx = 0x00;
			pulseCount = 0;  // restart looking for data bits
			received = false;

			// --------------------------------------------------------------------------------------
			Rxb6IntLastTime = 0L;					// initialize the bit timing acquisition
			
			sei();						// enable interrupt
			break;
		}
		else if(nMsg)
		{		
			// time rx last msg in usec
			if((time - timeRxb6LastMsg)>500000L)
			{
				DataMsgWsFull = 0;				
				// join data
				for(i=0;i<nMsg;i++)
				{
					UpdateLacrosseData(i);
				}
				nMsg = 0;
				timeRxb6LastMsg = time;
				
				// Switch off etherodina
				StMain = SP_OFF_ETH			;
			}
		}
		// remain in the same state
		break;

	case SP_OFF_ETH			:
		Rxb6Power(0)			;		// off etherodina
		// all different types msg are received. Send message to TTN and wait

		// disable interrupt to avoid new data corrupting the buffer
		pinMode(RXB6_DATA, INPUT);          	// data interrupt input
		// disable interrupt to avoid new data corrupting the buffer
		// --------------------------------------------------------------------------------------
		Rxb6DetachInterrupt();
		// --------------------------------------------------------------------------------------
		// read battery voltage
		BatteryVoltage = getBatVoltage();
		// --------------------------------------------------------------------------------------
		// read temp/press from bmp180 sensor
		Bmp180temp = bmp180.getTemperature();
		Bmp180press = bmp180.getPressure();
		
		// --------------------------------------------------------------------------------------
		StMain = SP_SEND_TTN		;
		break;

	case SP_SEND_TTN		:
		// send message to ttn
		// ---------------------------------------------------
		wdtReset();				// reset watchdog	
		// ---------------------------------------------------
		do_send(&sendjob);
		tmTimeout = millis();				// start timeout send ttn
		StMain = SP_SEND_TTN1		;
		break;
		
	case SP_SEND_TTN1		:
		// send message to ttn
		// ---------------------------------------------------
		wdtReset();				// reset watchdog	
		// ---------------------------------------------------
		// do_send(&sendjob);
		if(sleeping == false)
		{
			os_runloop_once();
			break;
		}
		sleeping = false;
		CountLowPower = 63;				// 63 * 8 = 504sec (8.4 min)
		
		
		tmDelay = millis();

		// put lora in sleep mode
		LoRaSleep();
		UsbOff();			// off usb

		ClockRc(clock_div_128);			// Switch to RC Clock 
		
		StMain = SP_LOWPOWER       ;
		
		break;

	case SP_LOWPOWER		:

		wdtReset();					// reset watchdog	
		if(CountLowPower<=0)
		{
			ClockExternal();			// Enable External Clock

			UsbOn();			// on usb
			
			StMain = SP_ON_ETH			;
			break;
		}

		
		// powerdown for 8sec
		wdtDisable();							// deactive watchdog timer
		// lowpower
		// LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
		LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
		// TEST
		// LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
		wdtEnable();							// active watchdog with 8sec timing
		
		CountLowPower--;
		if(CountLowPower<=0)
		{
			ClockExternal();			// Enable External Clock

			UsbOn();			// on usb
			StMain = SP_ON_ETH			;
			break;
		}
		break;
		
	case SP_RESETSYSTEM		:
		// execute reset system
		ResetSystem();				// use watchdog to restart arduino
		break;
		
		default					:
		// for every other states, do reset
		StMain = SP_START			;
		break;
	}
}

void setup() {
	
	wdtDisable();							// deactive watchdog timer

	BlinkLed(250);
	wdtEnable();							// active watchdog with 8sec timing

	// start SPI
	SPI.begin();

#ifdef SERIAL_MSG
	Serial.begin(115200);
#endif				// #ifdef SERIAL_MSG

	// Serial.println(F("Starting..."));
#ifdef SERIAL_MSG	
	Serial.println("Starting...");
#endif				// #ifdef SERIAL_MSG

	// init bmp180 sensor
	bmp180.init();

	//---------------------------------------------
	StMain = SP_START;						// reset state machine
	// power off etherodina
	Rxb6Power(0);
	StateManager();
	
	//---------------------------------------------
	// init 5n1

	// initialize d5n1
	init_dWs2300();
	
	// end init 5n1
	//---------------------------------------------
	
	// --------------------------------------------
	// start connection to TTN. This function must call by setup()
	InitTTNConnection();
	// mod 18/07
	while(sleeping == false)
	{
		os_runloop_once();
	}
	sleeping = false;
}

// ================================================================
// loop: 
//
void loop() {

	// ---------------------------------------------------
	// normal operation
	// loop acquisition on lacrosse ws2300
	StateManager();
	// end loop lacrosse
	// ---------------------------------------------------
}



// =======================================================================
// =======================================================================
#ifdef NOT_USED
// =======================================================================
// =======================================================================



#endif // NOT_USED

// =======================================================================
