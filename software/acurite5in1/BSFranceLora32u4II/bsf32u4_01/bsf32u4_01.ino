// **********************************************************************************
// Project:    Acurite 5n1 weather station lora tranceiver
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
// 10/12/2018
// inserted pressure reading management
//
// 19/07/2018
// inserted activation from OTAA to ABP
// see:
// https://primalcortex.wordpress.com/2017/10/31/ttnlorawan32u4node/
// https://www.thethingsnetwork.org/docs/devices/registration.html#personalize-device-for-abp
// https://github.com/fcgdam/TTN32u4ABP
//
// 16/07/2018
// sleep test with lmic library
// see:
// https://www.thethingsnetwork.org/forum/t/lmics-tx-complete-event-takes-20-30-seconds-to-fire/2639
// https://github.com/DiederikRhee/ProMini_Lora_Otaa_temperture
// https://github.com/DiederikRhee/ProMini_Lora_Otaa_temperture/blob/master/Lora_temp_hum/Lora_temp_hum.ino
// -----------------------------------------------------------------
// acurite 5n1 acquisition program with bsfrance lora32u4 module 
// https://bsfrance.fr/lora-long-range/1345-LoRa32u4-II-Lora-LiPo-Atmega32u4-SX1276-HPD13-868MHZ-EU-Antenna.html
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
// the superheterodyne module is able to read data from these devices:
// 1) AcuRite 06014 PRO+ 5-in-1 Weather Sensor with Rain Gauge, Wind Speed, Wind Direction, Temperature and Humidity (vn1txca2)
//    https://www.amazon.com/AcuRite-Weather-Direction-Temperature-Humidity/dp/B00SN1WHEU/ref=sr_1_7?s=home-garden&ie=UTF8&qid=1528101292&sr=1-7&keywords=acurite+5+in+1+weather+station
// 2) AcuRite 06045M Lightning Detector Sensor with Temperature & Humidity
//    https://www.amazon.com/AcuRite-06045M-Lightning-Detector-Temperature/dp/B01LNALL6C
// 3) AcuRite 06044M 06044M Wireless Temperature and Humidity Monitor Sensor
//    https://www.amazon.com/AcuRite-06044M-Wireless-Temperature-Humidity/dp/B01G7BE9WK/ref=cm_cr_arp_d_product_top?ie=UTF8
//

#define BSFRANCEBOARD 1
#define LOWPOWER 1

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
#include "src/BMP180/BMP180.h"

#include <LoRa.h>			// per basso consumo lora

// ---------------------------------------------------------------
// defines used for configuration
// ---------------------------------------------------------------

// enable SERIAL_MSG if you want to print msg to serial
// #define SERIAL_MSG

// ------------------------------- 1) #define CHECK_5N1ID
// define CHECK_5N1ID if you want to receive messages only from a 5n1 weather station, 
// with ID specified in id5n1WeatherStation
#define CHECK_5N1ID

// ------------------------------- 2) #define CHECK_CHANNEL
// define CHECK_CHANNEL if you want to check the transmission channel
// #define CHECK_CHANNEL

// ------------------------------- 3) #define ID_5N1
// id address of 5n1 weather station
#define ID_5N1		0x0103

// ------------------------------- 4) #define T_CYCLE_TIME_MSEC
// cycle time in msec
// #define T_CYCLE_TIME_MSEC	71500L
#define T_CYCLE_TIME_MSEC	359500L			// 6min - 0.5sec

// ---------------------------------------------------------------
// ID 5n1: default value
// ---------------------------------------------------------------

// id weather station connected to receiver.
const uint16_t id5n1WeatherStation = ID_5N1;

// id weather station connected to receiver splitted.
// these variables are used to recognize the 5n1 ID in the interrupt function
const uint8_t HiId5n1 = (id5n1WeatherStation >> 8) & 0x0F;
const uint8_t LoId5n1 = (id5n1WeatherStation & 0xFF);

//-----------------------------------------------------------------
// hardware configuration
//-----------------------------------------------------------------
//
//LoR32u4II 868MHz or 915MHz (black board)
#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    868E6  // 915E6
#define PABOOST true 

#include "bsf32u4_01.h"

volatile bool joined = false;
volatile bool sleeping = false;

//------------------------------------------------------

// mod 16/07:
// if DataMsg5n1Full = 0x03, you have received the two types os 5n1 msg
int  DataMsg5n1Full = 0x00;
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
//	uint16_t SensorId;			[08][09]            	// id of acurite 5n1 sensor
//	float WindSpeed_kph;		[10][11]            	// (01) windspeed in km/hour. Range: 0.0 to 159.9 kph
//	int WindDirection_Pos;		[12]                	// windspeed direction. Range: 0..15
//	float Rainfall; 			[13][14]            	// (02) rainfall. Range: -327.68 (0) to 327.67
//	int ActiveRain;				[15]                	// flag for active rainfall
//	unsigned long RainTmSpan;	[16][17][18][19]    	// time start rain
//	float Temp;					[20][21]            	// (03) temperature in celsius. Range: 4,44 to 70 C
//	int Hum;					[22]                	// % humidity. Range: 1 to 99 %RH
//	int BattLow;				[23]                	// flag battery low

// (01) Range: 0.0 to 159.9 kph. the float is multiplied by 10 and converted into an integer
// (02) Range: 0 to 327.67. the float is multiplied by 100 and converted into an integer
// (03) Range: 4,44 to 70 C. the float is multiplied by 100 and converted into an integer

#define SIZE_MSG5N1		29

uint8_t msgStr_d5n1[SIZE_MSG5N1];

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
static u1_t NWKSKEY[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

// LoRaWAN AppSKey, application session key
static u1_t APPSKEY[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

// LoRaWAN end-device address (DevAddr)
static u4_t DEVADDR = 0x12345678;   // Put here the device id in hexadecimal form.

#else
// -------------------------------------------------------------
// KEYS FOR ABP ACCESS

// Application ID: demo_ttn
// LoRaWAN NwkSKey, network session key
static u1_t NWKSKEY[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
// LoRaWAN AppSKey, application session key
static u1_t APPSKEY[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
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
		str_d5n1(msgStr_d5n1);				// form lora message
		// send lora message
		LMIC_setTxData2(1, msgStr_d5n1, SIZE_MSG5N1, 0);
		// print to serial terminal message
#ifdef SERIAL_MSG
		// transform in hex string and send to serial terminal
		PrintHex8(&msgStr_d5n1[0], SIZE_MSG5N1);
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
		// Unknown event
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

#include <avr/interrupt.h>    // Needed to use interrupts

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
// rx 5n1
//-----------------------------------------------------------------
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
// module configuration
//
// define PRTDATA if you want to print data
// #define PRTDATA

// defines for acurite modules managed by software
//
// types of arduino provided by the module
#define ARD_UNO				0				// arduino UNO
#define ARD_BSFLORA32U4		1				// bsfrance lora f32u4

// define module type used
#define	MOD_ARDUINO			ARD_BSFLORA32U4

// ring buffer size has to be large enough to fit
// data and sync signal, at least 120
// round up to 128 for now
#define RING_BUFFER_SIZE  152

#define SYNC_HIGH       600
#define SYNC_LOW        600

#define PULSE_LONG      400
#define PULSE_SHORT     220

// original:
#define PULSE_RANGE     100
// test 07/04: verified that statistically the duration of the message is approximately 43msec.
// The error delta for each pulse is 10usec. 20usec is used as a pulse range
// #define PULSE_RANGE     20

//#define BIT1_HIGH       PULSE_LONG
//#define BIT1_LOW        PULSE_SHORT
//#define BIT0_HIGH       PULSE_SHORT
//#define BIT0_LOW        PULSE_LONG

//BIT1
#define BIT1_HIGH_MIN  (PULSE_LONG-PULSE_RANGE)
#define BIT1_HIGH_MAX  (PULSE_LONG+PULSE_RANGE)
#define BIT1_LOW_MIN   (PULSE_SHORT-PULSE_RANGE)
#define BIT1_LOW_MAX   (PULSE_SHORT+PULSE_RANGE)

//BIT0
#define BIT0_HIGH_MIN  (PULSE_SHORT-PULSE_RANGE)
#define BIT0_HIGH_MAX  (PULSE_SHORT+PULSE_RANGE)
#define BIT0_LOW_MIN   (PULSE_LONG-PULSE_RANGE)
#define BIT0_LOW_MAX   (PULSE_LONG+PULSE_RANGE)

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
// Arduino has two different kinds of interrupts: external, and “pin change”. 
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

// 5n1 Tower Message Types
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


// The pulse durations are the measured time in micro seconds between pulse edges.
unsigned int bytesReceived = 0;

// ------------------------------------------------
// for the new interrupt procedure
// channel codes used by the control unit
#define	CHAN_A	0x03
#define	CHAN_B	0x02
#define	CHAN_C	0x00

volatile uint8_t ChannelWS5n1 = CHAN_C;			// select the channel used by the ws


// variables used for message receive
volatile unsigned long TmrMsgBegin, deltaTmrMsg;		// is used to calculate the duration of the message in microsecond 
volatile unsigned long TmrNextMsg;						// timer start of the next message
volatile unsigned long TmrStart, TmrStop, delta;
volatile unsigned long StartSeqMsec, StopSeqMsec, deltaSeqMsec;
volatile unsigned long msecStartAcqMsg;	// start time acq. msg 
volatile unsigned long uSecNextMsg;			// wait in microseconds before receiving the next message
volatile int nPulses;

volatile uint8_t nMsgReceived = 0; 		// n. messages actually in vector msg5n1
volatile uint8_t msg5n1[9];			// 5n1 acurite message (8 bytes + crc calculated)
volatile uint8_t crcOk;


// --------------------- DATI PER CALCOLO RAINFALL

bool activeRain = false;

int strikeTot = 0;
int strikeWrapOffset = 0;
int lastStrikeCount = 0;
bool activeStrikes = false;
unsigned long strikeLast = 0;

const unsigned int tempOffset10th = 0;  			// offset in 10th degrees C

// ---------------------------------
// data read from bmp180 pressure sensor

BMP180 bmp180;

long Bmp180temp, Bmp180press;

// ---------------------------------

// --------------------------------------------
// 5n1 data
// --------------------------------------------

unsigned int BatteryVoltage;		// used to store battery voltage

// acurite 5n1 data
typedef struct {
	unsigned long idMsg;			// message count to transmit
	unsigned long time;				// time of update variables
	uint16_t SensorId;				// id of acurite 5n1 sensor
	float WindSpeed_kph;			// windspeed in km/hour
	int WindDirection_Pos;			// windspeed direction
	float Rainfall; 				// rainfall
	int ActiveRain;					// flag for active rainfall
	unsigned long RainTmSpan;		// time start rain
	float Temp;						// temperature in celsius
	int Hum;						// % humidity
	int BattLow;					// flag battery low
} data_5n1;

data_5n1 d5n1;

void init_d5n1()
{
	d5n1.idMsg = 0L				;		// message count to transmit
	d5n1.time = 0L				;		// time of update variables
	d5n1.SensorId = 0			;
	d5n1.WindSpeed_kph = 0.0	;
	d5n1.WindDirection_Pos = 0	;
	d5n1.Rainfall = 0.0			; 
	d5n1.ActiveRain = 0			;
	d5n1.RainTmSpan = 0L		;		// tm Last rain event
	d5n1.Temp = 0.0				;
	d5n1.Hum = 	0 				;
	d5n1.BattLow = 0			;			
}

// increment id msg
void incIdMsg()
{
	d5n1.idMsg++;
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
// convert d5n1 struct in string message
void str_d5n1(uint8_t *str)
{
	uint8_t *ptr;
	uint16_t dtWord;
	int dtInt;
	
	ptr = str;
	//	unsigned long idMsg;		[00][01][02][03]    	// message count to transmit
	ulong2bytes(ptr, d5n1.idMsg);
	ptr = ptr + 4;
	//	unsigned long time;			[04][05][06][07]    	// time of update variables
	ulong2bytes(ptr, d5n1.time);
	ptr = ptr + 4;
	//	uint16_t SensorId;			[08][09]            	// id of acurite 5n1 sensor
	word2bytes(ptr, d5n1.SensorId);
	ptr = ptr + 2;
	//	float WindSpeed_kph;		[10][11]            	// (01) windspeed in km/hour. Range: 0.0 to 159.9 kph
	// (01) Range: 0.0 to 159.9 kph. the float is multiplied by 10 and converted into an integer
	dtWord = (uint16_t)(d5n1.WindSpeed_kph * 10.0);
	word2bytes(ptr, dtWord);
	ptr = ptr + 2;
	//	int WindDirection_Pos;		[12]                	// windspeed direction. Range: 0..15
	*ptr = d5n1.WindDirection_Pos & 0x0F;
	ptr++;
	//	float Rainfall; 			[13][14]            	// (02) rainfall. Range: -327.68 (0) to 327.67
	// (02) Range: 0 to 327.67. the float is multiplied by 100 and converted into an integer
	dtWord = (uint16_t)(d5n1.Rainfall * 100.0);
	word2bytes(ptr, dtWord);
	ptr = ptr + 2;
	//	int ActiveRain;				[15]                	// flag for active rainfall
	*ptr = 0x00;
	if(d5n1.ActiveRain)
	*ptr = 0x01;
	ptr++;
	//	unsigned long RainTmSpan;	[16][17][18][19]    	// time start rain
	ulong2bytes(ptr, d5n1.RainTmSpan);
	ptr = ptr + 4;
	//	float Temp;					[20][21]            	// (03) temperature in celsius. Range: 4,44 to 70 C
	// (03) Range: 4,44 to 70 C. the float is multiplied by 100 and converted into an integer
	dtInt = (int)(d5n1.Temp * 10);
	word2bytes(ptr, (uint16_t)dtInt);
	ptr = ptr + 2;
	//	int Hum;					[22]                	// % humidity. Range: 1 to 99 %RH
	*ptr = d5n1.Hum & 0xFF;
	if(*ptr > 99)
	*ptr = 99;
	ptr++;
	//	int BattLow;				[23]                	// flag battery low
	*ptr = 0x00;
	if(d5n1.BattLow)
	*ptr = 0x01;
	ptr = ptr + 1;
	*ptr = BatteryVoltage & 0xFF;
	ptr = ptr + 1;
	// tot: 25 bytes
	dtInt = (int)(Bmp180temp);
	word2bytes(ptr, (uint16_t)dtInt);
	ptr = ptr + 2;
	// tot: 27 bytes
	dtInt = (int)(Bmp180press/10);
	word2bytes(ptr, (uint16_t)dtInt);
	// tot: 29 bytes
}

// table to convert from idx to position
char acurite_5n1_winddirection_pos[] =
{
	//  position     str   idx degrees     
	14,       // "NW",  0  315	  
	11,       // "WSW", 1  247.5  
	13,       // "WNW", 2  292.5  
	12,       // "W",   3  270    
	15,       // "NNW", 4  337.5  
	10,       // "SW",  5  225    
	0 ,       // "N",   6  0      
	9 ,       // "SSW", 7  202.5  
	3 ,       // "ENE", 8  67.5   
	6 ,       // "SE",  9  135    
	4 ,       // "E",   10 90     
	5 ,       // "ESE", 11 112.5  
	2 ,       // "NE",  12 45     
	7 ,       // "SSE", 13 157.5  
	1 ,       // "NNE", 14 22.5   
	8         // "S"    15 180    
};

// 
// helper code to print formatted hex 
// 
// const char HexTblChr[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
const char HexTblChr[16] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};

void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex
{
	char hex8buf[length*2+1];
	char *ptr;
	uint8_t val;

	ptr = &hex8buf[0];
	for (uint8_t i = 0; i < length; i++) 
	{   
		val = *data;
		*ptr =  HexTblChr[val >> 4];
		ptr++;
		*ptr =  HexTblChr[val & 0x0F];
		ptr++;
		data++;
	}
	*ptr = 0;
	Serial.print(hex8buf);
}

// ===================================================================================================
// INTERRUPT MANAGEMENT
// ===================================================================================================

// initialize acquisition 5n1 messages
void Rxb6InitializeMsgAcquisition()
{
	nMsgReceived = 0;			// reinitialize the reception of the message
}

//-------------------------------------------------------
// RXB6 INTERRUPT
//-------------------------------------------------------

// acurite 5n1 msg fields
// data specific for type1 msg
typedef struct {
	// MT_WS_WD_RF='1': msg Wind Speed, Direction and Rainfall messaggio '1'
	uint8_t Direction;
	uint16_t Rainfall;
} data_message_5n1_type1;

// data specific for type8 msg
typedef struct {
	// MT_WS_T_RH ='8': msg Wind speed, Temperature, Humidity messaggio '8'
	uint16_t Temperature;		
	uint8_t Humidity;
} data_message_5n1_type8;

// data type with message fields
typedef struct {
	uint8_t ch;						// field channel
	uint8_t rpt;					// field Rpt
	uint16_t id5n1;					// id weather station
	uint8_t batt;					// battery status
	uint8_t idm;					// id message
	uint8_t crc[2];					// crc sent & calculated
	uint16_t WindSpeed;				// set always the last windspeed
	union {
		data_message_5n1_type1 t1;
		data_message_5n1_type8 t8;
	} fld;
} data_message_5n1;

volatile data_message_5n1 Fld5n1;	

void InitFld5n1(void)
{
	Fld5n1.ch = 0;					// field channel
	Fld5n1.rpt = 0;					// field Rpt
	Fld5n1.id5n1 = 0;				// id weather station
	Fld5n1.batt = 0;				// battery status
	Fld5n1.idm = 0;
	Fld5n1.crc[0] = 0;
	Fld5n1.crc[1] = 0;
	Fld5n1.WindSpeed = 0;
	Fld5n1.fld.t1.Direction = 0;
	Fld5n1.fld.t1.Rainfall = 0;
}

// decode all message fields
// return true if crc is OK
bool decodeFld5n1(uint8_t msg[])
{
	uint16_t highbits, lowbits;
	
	Fld5n1.crc[0] = msg[7];
	Fld5n1.crc[1] = msg[8];
	
	lowbits = msg[0] >> 4;
	Fld5n1.rpt = lowbits & 0x03;				// field Rpt
	Fld5n1.ch = lowbits >> 2;					// field channel
	Fld5n1.id5n1 = (((uint16_t)msg[0] & 0x0F) << 8) | msg[1];
	Fld5n1.batt = msg[2] >> 6;					// battery status
	Fld5n1.idm = msg[2] & 0x3F;					// id message
	highbits = ((uint16_t)msg[3] & 0x1F) << 3;
	lowbits = ((uint16_t)msg[4] & 0x70 ) >> 4;
	Fld5n1.WindSpeed	= highbits | lowbits;
	if (Fld5n1.idm == '1')
	{
		// MT_WS_WD_RF='1': msg Wind Speed, Direction and Rainfall messaggio '1'
		Fld5n1.fld.t1.Direction	= msg[4] & 0x0F;
		highbits = ((uint16_t)msg[5] & 0x7f) << 7;
		lowbits = ((uint16_t)msg[6] & 0x7F);
		Fld5n1.fld.t1.Rainfall		= highbits | lowbits;
	}
	else
	{
		// MT_WS_T_RH ='8': msg Wind speed, Temperature, Humidity messaggio '8'
		highbits = ((uint16_t)msg[4] & 0x0F) << 7 ;
		lowbits = ((uint16_t)msg[5] & 0x7F);
		Fld5n1.fld.t8.Temperature	= highbits | lowbits;	
		Fld5n1.fld.t8.Humidity		= msg[6] & 0x7F;
	}
	// check crc
	if(msg[7] != msg[8])
	{
		// crc error
		return false;
	}
	return true;
}

// Note:
// rx msg intervals in msec: you receive 2 messages at intervals of 18 seconds between them
// 10:15:23.006|[1c2471000fb4f569]|[69]|14359		msg '1'
// 10:15:23.053|[2c2471000fb4f579]|[79]|45
// 10:15:40.980|[1c24780087504edd]|[dd]|17914		msg '8'
// 10:15:41.027|[2c24780087504eed]|[ed]|45
// 10:15:58.979|[1c2471000fb4f569]|[69]|17916		msg '1'
// 10:15:59.059|[2c2471000fb4f579]|[79]|45   
// 10:16:16.978|[1c24780087504edd]|[dd]|17914		msg '8'
// 10:16:17.025|[2c24780087504eed]|[ed]|45
// 10:16:34.997|[1c2471000fb4f569]|[69]|17915		msg '1'
// 10:16:35.043|[2c2471000fb4f579]|[79]|46   
// 10:16:52.980|[1c24780087504edd]|[dd]|17913		msg '8'
// 10:16:53.027|[2c24780087504eed]|[ed]|45   
// 10:17:11.002|[1c2471000fb4f569]|[69]|17916		msg '1'
// 10:17:11.037|[2c2471000fb4f579]|[79]|45   
// 10:17:29.004|[1c24780087504edd]|[dd]|17914		msg '8'
// 10:17:29.039|[2c24780087504eed]|[ed]|45
// 10:17:46.989|[1c2471000fb4f569]|[69]|17916		msg '1'
// 10:17:47.023|[2c2471000fb4f579]|[79]|45   
// 10:18:04.973|[1c24780087504edd]|[dd]|17914		msg '8'
// 10:18:05.047|[2c24780087504eed]|[ed]|45   
// 10:18:23.013|[1c2471000fb4f569]|[69]|17916		msg '1'
// 10:18:23.047|[2c2471000fb4f579]|[79]|45   
// 10:18:40.984|[1c24780087504edd]|[dd]|17916		msg '8'
// 10:18:41.031|[2c24780087504eed]|[ed]|45

// RXB6 Data Interrupt handler 
// Tied to RXB6_DATA arduino digital input.
// Set to interrupt on edge (level change) high or low transition.
// Change the state of the Arduino LED (pin 13) on each interrupt. 
// This allows scoping pin 13 to see the interrupt / data pulse train.
// 
// RX OF A SEQUENCE OF LEVEL CHANGES:
// - 8 level changes for IDLE pulses
// - 8-byte message reception:
// nbit = 8 * 8 = 64bit
// nlivels = nbit * 2 = 128
// In total for each message, you have these level changes
// Tot Levels = 128 + 8 = 136 levels
// 
void Rxb6InterruptHandler() 
{
	static unsigned int bitState  = 0;
	static uint8_t wsByteMsg;			// stato del byte ricevuto dalla weather station
	static uint8_t crc5n1;				// 
	static uint8_t fErrMsg;				// true if error msg

	int idx, nbit;
	uint8_t value, bitStatus;
	unsigned long svTmrStart;

	svTmrStart = TmrStart;
	TmrStop = micros();
	delta = TmrStop - TmrStart;			// time diff from pulse fronts
	TmrStart = TmrStop;
	bitState = digitalRead(RXB6_DATA);

	// ignore if we haven't finished processing the previous 
	// received signal in the main loop.
	if( nMsgReceived != 0)
	{
		// the main loop work on the message buffer has not yet completed
		return;
	}
	if(bitState == HIGH)
	{
		// received low bit zone
		// --+     +--
		//   +-----+
		// do not manage this area
		return;
	}
	
	// if(bitState == LOW)
	// received hi bit area
	//   +-----+
	// --+     +--

	fErrMsg = false;
	if(nPulses<0)
	{
		// rx of 4 sync pulses
		if( delta>=(SYNC_HIGH-PULSE_RANGE) && delta<=(SYNC_HIGH+PULSE_RANGE) )
		{
			// check if this is the first sync. 
			// In this case it initializes the timer that estimates the beginning of the next message
			if(nPulses == -4)
			{
				uSecNextMsg = svTmrStart;
				TmrMsgBegin = svTmrStart;
			}
		}
		else if( delta>=(BIT0_HIGH_MIN) && delta<=(BIT0_HIGH_MAX) )
		{
			// Maybe you have lost several sync bits. Try to retrieve the analysis
			nPulses = 0;				// il bit e' 0.
		}
		else if( delta>=(BIT1_HIGH_MIN) && delta<=(BIT1_HIGH_MAX) )
		{
			// Maybe you have lost several sync bits. Try to retrieve the analysis
			nPulses = 0;				// il bit e' 1.
		}
		else
		{
			fErrMsg = true;
		}
		if(nPulses>=0)
		{
			// initialize the variables used for message recognition
			crc5n1 = 0;					// initialize the calculated crc of the message
			msecStartAcqMsg = millis();			// start time acquisition msg
		}	
	}

	if(nPulses>=0)
	{
		// decodifica parte hi del bit ricevuto
		idx = nPulses >> 3;
		nbit = nPulses & 0x07;
		if(nbit == 0)
		{
			// bisogna memorizzare il primo bit.
			wsByteMsg = 0;				// initialize the message byte
		}
		wsByteMsg = wsByteMsg << 1;
		if( delta>=(BIT0_HIGH_MIN) && delta<=(BIT0_HIGH_MAX) )
		{
			bitStatus = 0x00;				// bit is 0.
		}
		else if( delta>=(BIT1_HIGH_MIN) && delta<=(BIT1_HIGH_MAX) )
		{
			bitStatus = 0x01;				// bit is 1.
			wsByteMsg = wsByteMsg | 0x01;
		}
		else
		{
			// received pulse with incorrect duration
			fErrMsg = true;
		}
		if(fErrMsg == false)		
		{
			// the pulse has a correct duration (0 or 1).
			// store bit in byte
			
			// Check the status of some bit fields already acquired
			if(idx==0)
			{
				// checks on byte0 of the message
				if(nbit == 1)
				{
// define CHECK_CHANNEL if you want to check the transmission channel
#ifdef CHECK_CHANNEL
					
					// received the first 2 meaningful bits: channel A, B, C
					if(wsByteMsg != ChannelWS5n1)
					{
						// received the data of an unit on a different channel.
						fErrMsg = true;
					}
#endif				// #ifdef CHECK_CHANNEL
				}
				else if(nbit == 3)
				{
					// received 2 bits indicating the n. of repeating the message
					value = wsByteMsg & 0x03;
					if(value == 1)
					{
						// this is the first copy of the message.
						// The first copy of the next message will be received in 18 seconds
						// uSecNextMsg = uSecNextMsg + 18000000L;
						uSecNextMsg = 17980000L;
						// calculates the start time of the next message
						TmrNextMsg = TmrMsgBegin + uSecNextMsg;
					}
					else if(value == 2)
					{
						// this is the second copy of the message.
						// The first copy of the next message will be received in about 18 seconds - around 45msec
						// uSecNextMsg = uSecNextMsg + 17900000L;
						uSecNextMsg = 17900000L;
						// calculates the start time of the next message
						TmrNextMsg = TmrMsgBegin + uSecNextMsg;
					}
					else
					{
						// repetition id different than 1 or 2
						fErrMsg = true;
					}
				}
				else if(nbit == 7)
				{
// define CHECK_5N1ID if you want check the control unit id
#ifdef CHECK_5N1ID
					// received the 4 bits that express the n. HiId5n1
					if(HiId5n1 != (wsByteMsg & 0x0F))
					{
						// received data from a different unit.
						fErrMsg = true;
					}
#endif			// #ifdef CHECK_5N1ID
				}
			}
			else if(idx==1)
			{
				// checks on message byte1
				if(nbit == 7)
				{
// define CHECK_5N1ID if you want check the control unit id
#ifdef CHECK_5N1ID
					// received 8 bits that express the n. LoId5n1
					if(LoId5n1 != (wsByteMsg))
					{
						// received data from a different unit.
						fErrMsg = true;
					}
#endif			// #ifdef CHECK_5N1ID
				}
			}
			else if(idx==2)
			{
				// checks on message byte2
				if(nbit == 7)
				{
					// received the 6 bits of the code of message
					value = wsByteMsg & 0x3F;
					if( (value != '1') && (value != '8') ) 
					{
						// wrong message code
						fErrMsg = true;
					}
				}
			}
		}
		if(fErrMsg == false)		
		{
			// check if the crc needs to be calculated
			if(nbit == 7)
			{
				// It's the most significant bit of the byte.
				// store the new byte
				msg5n1[idx] = wsByteMsg;
				// calculate the crc
				if(idx<7)
				{
					// continue to calculate the crc
					crc5n1 = crc5n1 + wsByteMsg;
				}
				else
				{
					// last byte of the received message
					// store the calculated crc
					msg5n1[8] = crc5n1;
					// decode the message and check crc
					crcOk = decodeFld5n1(msg5n1);
				}
			}
		}
	}

	if(fErrMsg == false)
	{
		// increase the number of pulses
		nPulses++;					// index of the next impulse
		if(nPulses>=64)
		{
			nPulses = -4;				// sync reception starts again
			StopSeqMsec = millis();
			deltaSeqMsec = StopSeqMsec - StartSeqMsec;
			// Serial.print("Delta: " ); Serial.println(deltaSeqMsec);
			StartSeqMsec = millis();
			// calculate how long the message reception phase has lasted
			deltaTmrMsg = micros() - TmrMsgBegin;
			nMsgReceived = 1;			// 1 messaggio ricevuto
			
			// =============================
			// modify 21/05:
			// added off interrupt and etherodine power here
			// serves to reduce consumption
			// disable interrupt to avoid new data corrupting the buffer
			// --------------------------------------------------------------------------------------
			// Rxb6DetachInterrupt();
			// detachInterrupt(INT_RXB6_DATA);
			// the project uses the pin change interrupt
			PCICR  &= ~bit ( digitalPinToPCICRbit( RXB6_DATA ) );
			Rxb6Power(0);								// off power rxb6 433Mhz Receiver
			// =============================
		}
	}
	else
	{
		// message error
		nMsgReceived = 0;			// reinitialize the reception of the message
		nPulses = -4;				// sync reception starts again
		crc5n1 = 0;
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

// used in acurite 5n1, 6045, 6044
int acurite_getHumidity (uint8_t byte) 
{
	// range: 1 to 99 %RH
	int humidity = byte & 0x7F;
	return humidity;
}

// return wind direction to position
char getWindDirection_Position(byte b) 
{
	int direction = b & 0x0F;
	return (int)acurite_5n1_winddirection_pos[direction];
}

float acurite_getTemp_5n1(int rawtemp) 
{
	// range fahrenheit: -40 to 158 F
	// range celsius:     4,44 to 70 C
	//
	// temperature in fahrenheit
	// float temp_F = (rawtemp - 400) / 10.0;
	// return temp_F;
	//
	// temperature in celsius
	// T(C) = (T(F) - 32)*5/9		// standard conversion
	// T(C)*9/5 = (T(F) - 32)
	// T(C)*9/5 + 32 = T(F)
	// T(C)*9/5 + 32 = (rawtemp - 400) / 10.0
	// ( (T(C)*18) + 320 ) / 10.0 = (rawtemp - 400) / 10.0
	// (T(C)*18) + 320 = (rawtemp - 400)
	// (T(C)*18) = rawtemp - 400 - 320
	// T(C)*18 = rawtemp - 720
	// T(C) = (rawtemp - 720) / 18
	// T(C) = (rawtemp/18) - 40
	float temp_C = (rawtemp/18.0) - 40.0;
	return temp_C;
}

// --------------------------------------------------------------------------
// NEW ALGORITHM RAINFALL

// IMPORTANT:
// EXPLANATION HOW THE BUCKET TIPS COUNTER WORKS IN THE MESSAGE
// 
// https://github.com/merbanan/rtl_433/issues/42
// 
// The rain gauge data is a counter of bucket tips that ranges from 0 to 16383 (14 bits, 0x0000 .. 0x3FFF). 
// Each bucket tip corresponds to 0.01 in. of rain. 
// Data is accumulated starting with power-up. 
// When the counter reaches 16383 (0x3FFF) it resets to 0.
// 
// http://iw4blg.info/2014/12/01/pluviometro-con-arduino-rke05-2012/
// There are two commonly used measurement methods:
//
// total quantity: is the total accumulation of precipitation in a time period.
// It can be for example annual from January 1st to December 31st, or for farmers, the agricultural year from September 1st to August 31st
// The intensity of the phenomenon, that is how "strong it rains":
// rainfall is measured in millimeters over a specified period of time.
// The measurement of x millimeters of rain corresponds to the fall of x liters of rainwater, equivalent to x cubic decimetres,
// over an area of 1 square meter.
// According to scholars of the subject it is possible to catalog the precipitations as follows:
// Drizzle (<1 mm every hour)
// Light rain (1-2 mm / h)
// Moderate rain (2-6 mm / h)
// Heavy rain (> 6 mm / h)
// Reverse (> 10 mm / h but limited in duration)
// Cloudburst (> 30 mm / h)

// ---------------------------------------------[ nuovo algoritmo rain]

// queue of bucket tips values.
// The controller cycles every 5 minutes, so 12 elements store the bucket tips distant 1 hour ego: 
// 5 * 12 = 60 
#define 	TIME_CYCLE_RDRAIN		5							// every 5 minutes we read the bycket tips
#define		NEL_QUE_RAIN_COUNTER	(60 / TIME_CYCLE_RDRAIN)	// n. elements of rain counter
uint16_t queueRainCounter[NEL_QUE_RAIN_COUNTER-1];				// queue of rain counter data
uint8_t idxRainCounter = 0;

void InitRainCalc(void)
{
	int i;
	idxRainCounter = 0;
	for(i=0;i<NEL_QUE_RAIN_COUNTER;i++)
	{
		queueRainCounter[i] = 0;
	}
}

float acurite_get5n1Rainfall (uint16_t raincounter) 
{
	uint16_t delta;
	
	if(raincounter >= queueRainCounter[idxRainCounter])
	{
		// calc n. bucket tips per hour
		delta = raincounter - queueRainCounter[idxRainCounter];
	}
	else
	{
		// The rain gauge data is a counter of bucket tips that ranges from 0 to 16383 (14 bits, 0x0000 .. 0x3FFF). 
		delta = (0x4000 - queueRainCounter[idxRainCounter]) + raincounter;
	}
	// save the actual value of raincounter
	queueRainCounter[idxRainCounter] = raincounter;
	idxRainCounter++;				// increment queue index to pass to the oldest rain counter stored in queue
	if(idxRainCounter>=(NEL_QUE_RAIN_COUNTER-1))
	{
		idxRainCounter=0;
	}
	// every bucket tips is 0.01" = 1/100 inch (0.254mm )
	return ((float)delta * 0.254);
}

// end algoritmo rain

// end rx 5n1
//-----------------------------------------------------------------

bool DecodeEthManager()
{
	int MsgType = 0x00;
	
	// ---------------------------------------------------
	wdtReset();				// reset watchdog	
	// ---------------------------------------------------

	if(nMsgReceived)
	{
		// disable interrupt to avoid new data corrupting the buffer
		pinMode(RXB6_DATA, INPUT);          	// data interrupt input

		Rxb6Power(0);								// off power rxb6 433Mhz Receiver
		// disable interrupt to avoid new data corrupting the buffer
		// --------------------------------------------------------------------------------------
		Rxb6DetachInterrupt();
		// --------------------------------------------------------------------------------------

		// extract temperature value
		unsigned int startIndex, stopIndex, ringIndex;
		unsigned long temperature = 0;
		bool fail = false;

		//Decode to Hex Bytes
		byte dataBytes[bytesReceived];
		fail = false; // reset bit decode error flag

		// set time in d5n1 struct
		d5n1.time = millis();

		// -----------------------------------------------------
		// received full message of a 5n1 tower sensor
		// -----------------------------------------------------
		// -------------------------------- start
		d5n1.SensorId = Fld5n1.id5n1;
		float speed_kph = 0;
		if (Fld5n1.WindSpeed > 0) 
		{
			speed_kph = Fld5n1.WindSpeed * 0.8278 + 1.0;
		}
		d5n1.WindSpeed_kph = speed_kph ;
		if (Fld5n1.idm == '1')
		{
			// MT_WS_WD_RF='1': msg Wind Speed, Direction and Rainfall messaggio '1'
			// Wind Speed, Direction and Rainfall
			d5n1.WindDirection_Pos = getWindDirection_Position(Fld5n1.fld.t1.Direction) ;
			
			// Serial.print(",Rainfall,");
			// Serial.print(acurite_get5n1Rainfall (dataBytes[5], dataBytes[6]));
			d5n1.Rainfall = acurite_get5n1Rainfall(Fld5n1.fld.t1.Rainfall);
			d5n1.ActiveRain = activeRain;
			/****
			if (activeRain)
			{
				// NOTA: DISABILITATO. activeRaub serve per indicare da quanto tempo sta piovendo.
				// Usare eventualmente un altro metodo
				// Serial.print(",Last rain event,");
				// Serial.print(getTimeSpan(rainLast, millis()));
				d5n1.RainTmSpan = (millis() - rainLast) / 1000;
			}
			****/
			// Serial.print(",");
			MsgType = 0x01;			// 0x01: msg Wind Speed, Direction and Rainfall
		} 
		else 
		{
			// MT_WS_T_RH ='8': msg Wind speed, Temperature, Humidity messaggio '8'
			// Wind speed, Temp, Humidity
			d5n1.Temp = acurite_getTemp_5n1(Fld5n1.fld.t8.Temperature);
			
			// Serial.print(",Humidity,");
			// Serial.print(acurite_getHumidity(dataBytes[6]));
			d5n1.Hum = acurite_getHumidity(Fld5n1.fld.t8.Humidity);
			// Serial.print("%");
			MsgType = 0x02;			// 0x02: msg Wind speed, Temp, Humidity
		}
		
		// -------------------------------- end

		// ----------------------------
		// modify 18/07/2018
		// if enabled CHECK_5N1ID store messages only when
		// d5n1.SensorId == id5n1WeatherStation
#ifdef CHECK_5N1ID
		if(d5n1.SensorId != id5n1WeatherStation)
		{
			// the message received from a different weather station
			// initialize acquisition 5n1 messages
			nMsgReceived = 0;

			// re-enable interrupt
			pinMode(RXB6_DATA, INPUT);          	// data interrupt input
			// --------------------------------------------------------------------------------------
			Rxb6AttachInterrupt();
			// --------------------------------------------------------------------------------------
			Rxb6Power(1);							// ON power rxb6 433Mhz Receiver
			return false;
		}
#endif				// #ifdef CHECK_5N1ID	
		// ----------------------------
		// continue ...
		MsgType = MsgType & 0x03;
		DataMsg5n1Full = DataMsg5n1Full | MsgType;

		// delay for 1 second to avoid repetitions
		delay(1000);
		nMsgReceived = 0;
		// syncFound = false;

		// re-enable interrupt
		pinMode(RXB6_DATA, INPUT);          	// data interrupt input
		// --------------------------------------------------------------------------------------
		Rxb6AttachInterrupt();
		// --------------------------------------------------------------------------------------
		Rxb6Power(1);							// ON power rxb6 433Mhz Receiver
		return true;
	}
	Rxb6Power(1);								// ON power rxb6 433Mhz Receiver
	return false;
}

// https://www.avrfreaks.net/forum/watchdog-timer-discrepancy-time-sleep-mode-longer-8-seconds


// -------------------------------------------------------------
// autotuning timer watchdog utilizzando timer1 o timer3 come free running timer
// Nota: le funzioni vanno eseguite a interrupt disabilitati.
// La funzione di LowPower.idle viene risvegliata anche dagli interrupt.
// Il micro lora32u4 lavora ad una frequenza di clock di 8Mhz.
// Il timer scelto viene impostato in questo modo:
// 1) il timer non genera interrupt
// 2) la frequenza utilizzata per il clock del timer e' pari a: f_micro / 8 = 1Mhz
// 3) Il timer viene azzerato 
// a questo punto viene eseguita LowPower.idle per il tempo minimo, corrispondente a SLEEP_15MS
// Alla fine vengono letti i conteggi del timer
// Il tempo dei conteggi e' pari a:
// t = counts * (8 / f_micro) = counts * 1e-6sec
// quindi ogni incremento in counts corrisponde ad 1uS.
// Dato che i contatori timer1 e timer3 sono a 16 bit, i conteggi vanno da 0..65535 uS
// A questo punto si calcola il tempo corrispondente a SLEEP_8S: 
// SLEEP_8S risulta 512 volte il tempo SLEEP_15MS
// Se SLEEP_5S e' in usec, il n. di msec corrispondente a SLEEP_8S viene calcolato come:
// SLEEP_8S = SLEEP_5S * 512/1000 = SLEEP_5S * 64/125

// autotuning durata 15msec
volatile unsigned long timeTune8S;

// definire USE_TIMER3_TUNING se si vuole usare il timer3 per il tuning del watchdog
#define USE_TIMER3_TUNING

#ifdef USE_TIMER3_TUNING
//------------------------------------------------------
// use timer3 for watchdog timer tuning
// execute the function at interrupt disabled.

void tune15MS(void)
{
	uint16_t counts;
	unsigned long deltaT;
	//--------------------------------------------------
	// for tuning use idle, as it does not block the usb serial port
	// low power sleep 
	// ok. idle works. ALSO WORKS USB
	// average consumption during message recognition and idle enabled: 14.8mA
	//------------------------------
	// init timer3
	// cli();					// disable interrupt

	// Bit 7:6 – COMnA1:0: 0 0 Compare Output Mode for Channel A: OCnA/OCnB/OCnC port disconnected
	// Bit 5:4 – COMnB1:0: 0 0 Compare Output Mode for Channel B: OCnA/OCnB/OCnC port disconnected
	// Bit 3:2 – COMnC1:0: 0 0 Compare Output Mode for Channel C: OCnA/OCnB/OCnC port disconnected
	// Bit 3:2 - COMnx1:0: 0 0 Normal port operation, OCnA/OCnB/OCnC disconnected
	TCCR3A = 0x00;			// timer/counter 3 control register A
	// Timer/Counter3 Control Register B – TCCR3B
	// last 3 bits specify clock source: 
	// 0 = no clock source (stopped); 1 = no prescale; 2 = clock / 8; 3 = clock / 64
	// 4 = clock / 256; 5 = clock / 1024; 6 = ext clock on T0 pin falling edge; 7 = ext clock on T0 pin rising edge
	TCCR3B = 0x02; 			// 0b00000010: set clock / 8
	// Timer/Counter3 Control Register C – TCCR3C
	// Bit 7 – FOCnA: Force Output Compare for Channel A
	TCCR3C = 0x00; 			// not forcing output compare
	// Timer/Counter3 Interrupt Mask Register – TIMSK3       
	TIMSK3 = 0x00; 			// disabled all timer3 interrupt  
	TCNT3 = 0; 				// set timer counter initial value (16 bit value)

	// sei(); 					// enable interrupts	
	//-------------------------------
	// start lowpower idle
	LowPower.idle(SLEEP_15MS, ADC_OFF,
					TIMER4_OFF, TIMER3_ON,
					TIMER1_OFF, TIMER0_OFF,
					SPI_OFF, USART1_OFF, TWI_OFF, USB_ON);
	// read timer3 counts
	counts = TCNT3;
	// lora32u4 has 8Mhz clock
	// convert counts to msec
	deltaT = (unsigned long)counts;
	// deltaT stores the n. of microseconds corresponding to SLEEP_15MS
	// calculate the n. of msec corresponding to SLEEP_8S: it results 512 times the n. of msec corresponding to SLEEP_15MS
	/***
	deltaT = deltaT << 9;				// multiply by 512
	deltaT = deltaT / 1000;
	***/
	deltaT = deltaT << 6;				// multiply by 64
	// note: to get the value in msec, the data must be divided by 125
	timeTune8S = deltaT;
}

#else		// #define USE_TIMER3_TUNING
//------------------------------------------------------
// usa timer1 per il tuning del timer watchdog
void tune15MS(void)
{
	uint16_t counts;
	unsigned long deltaT;
	//--------------------------------------------------
	// for tuning use idle, as it does not block the usb serial port
	// low power sleep 
	// ok. idle works. ALSO WORKS USB
	// average consumption during message recognition and idle enabled: 14.8mA
	//------------------------------
	// init timer1
	// cli();					// disable interrupt

	// Bit 7:6 – COMnA1:0: 0 0 Compare Output Mode for Channel A: OCnA/OCnB/OCnC port disconnected
	// Bit 5:4 – COMnB1:0: 0 0 Compare Output Mode for Channel B: OCnA/OCnB/OCnC port disconnected
	// Bit 3:2 – COMnC1:0: 0 0 Compare Output Mode for Channel C: OCnA/OCnB/OCnC port disconnected
	// Bit 3:2 - COMnx1:0: 0 0 Normal port operation, OCnA/OCnB/OCnC disconnected
	TCCR1A = 0x00;			// timer/counter 3 control register A
	// Timer/Counter3 Control Register B – TCCR3B
	// last 3 bits specify clock source: 
	// 0 = no clock source (stopped); 1 = no prescale; 2 = clock / 8; 3 = clock / 64
	// 4 = clock / 256; 5 = clock / 1024; 6 = ext clock on T0 pin falling edge; 7 = ext clock on T0 pin rising edge
	TCCR1B = 0x02; 			// 0b00000010: set clock / 8
	// Timer/Counter3 Control Register C – TCCR3C
	// Bit 7 – FOCnA: Force Output Compare for Channel A
	TCCR1C = 0x00; 			// not forcing output compare
	// Timer/Counter3 Interrupt Mask Register – TIMSK3       
	TIMSK1 = 0x00; 			// disabled all timer3 interrupt  
	TCNT1 = 0; 				// set timer counter initial value (16 bit value)

	// sei(); 					// enable interrupts	
	//-------------------------------
	// start lowpower idle
	LowPower.idle(SLEEP_15MS, ADC_OFF,
					TIMER4_OFF, TIMER3_OFF,
					TIMER1_ON, TIMER0_OFF,
					SPI_OFF, USART1_OFF, TWI_OFF, USB_ON);
	// read timer1 counts
	counts = TCNT1;
	// lora32u4 has 8Mhz clock
	// convert counts to msec
	deltaT = (unsigned long)counts;
	// deltaT stores the n. of microseconds corresponding to SLEEP_15MS
	// calculate the n. of msec corresponding to SLEEP_8S: it results 512 times the n. of msec corresponding to SLEEP_15MS
	/***
	deltaT = deltaT << 9;				// multiply by 512
	deltaT = deltaT / 1000;
	***/
	deltaT = deltaT << 6;				// multiply by 64
	// note: to get the value in msec, the data must be divided by 125
	timeTune8S = deltaT;
}

#endif 		// #ifdef USE_TIMER3_TUNING

// version that uses timeTune8S to divide by 125 to get the msecs.
// It saves an operation by multiplying by 125 msec and also increases the precision of the operations
void PowerDown(unsigned long msec)
{
	int i;
	unsigned long deltaT;
	unsigned tmSleep = 0L;

	msec = msec * 125;					// esegui questo per utilizzare timeTune8S moltiplicato per 125
	deltaT = timeTune8S;
	// loop di esecuzione ritardo
	for(i=0;i<10;i++)
	{
		while(msec >= deltaT)
		{
			// LowPower.powerDown(tblSleep[i].tmSleepCode, ADC_OFF, BOD_OFF);
			LowPower.powerDown( (SLEEP_8S - i), ADC_OFF, BOD_OFF);
			msec = msec - deltaT;
			tmSleep = tmSleep + deltaT;
		}
		deltaT = deltaT >> 1;			// dividi per 2 deltaT
	}
}

// volatile unsigned long msecSleep = 17000L;		// tempo in msec

// execute a delay low power
void DelayLowPower(unsigned long delay_msec)
{
	unsigned long msecSleep;	// tempo in msec

	// ---------------------------------------------------
	msecSleep = millis() - msecStartAcqMsg;
	msecSleep = delay_msec - msecSleep;
	wdtReset();					// reset watchdog	
	wdtDisable();				// deactive watchdog timer
	UsbOff();					// off usb
	// ClockRc(clock_div_128);		// Switch to RC Clock 
	ClockRc(clock_div_1);		// Switch to RC Clock 
	PowerDown(msecSleep);
	ClockExternal();			// Enable External Clock
	UsbOn();					// on usb
	wdtEnable();				// active watchdog with 8sec timing
	// ---------------------------------------------------
}

void StateManager()
{
	unsigned long time;
	
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

		// ----------------
		// tune timer watchdog 15msec
		cli();						// disable interrupt
		tune15MS();
		sei();						// enable interrupt
		// ----------------

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
		// initialize acquisition 5n1 messages
		Rxb6InitializeMsgAcquisition();
		
		// enable interrupt etherodina
		pinMode(RXB6_DATA, INPUT);          	// data interrupt input
		// --------------------------------------------------------------------------------------
		Rxb6AttachInterrupt();
		// --------------------------------------------------------------------------------------

		tmTimeout = millis();				// timeout rx
		StMain = SP_DECODE_ETH		;
		break;
	case SP_DECODE_ETH		:
		// wait until a message is received 
		if(DecodeEthManager()==false)
		{
			// check timeout rx
			time = millis();
			if((time - tmTimeout) >= 20000L)
			{
				// in 20sec you have not received a message.
				StMain = SP_ON_ETH			;
				break;
			}
			break;
		}
		// received one message. 
		// Switch off etherodina
		StMain = SP_OFF_ETH			;
		break;
	case SP_OFF_ETH			:
		Rxb6Power(0)			;		// off etherodina
		// verify DataMsg5n1Full if you received 2 different types of message
		// If you received only 1 type of msg, wait some seconds and repeat the acquisition
		switch(DataMsg5n1Full)
		{
		case 0x01:
		case 0x02:
				// wait some seconds before to repeat the acquisition
				// initialize acquisition 5n1 messages
				Rxb6InitializeMsgAcquisition();
				StMain = SP_WAIT02          ;
				break;
		case 0x03:					
			// all different types msg are received. Send message to TTN and wait
			// mr: ripristinato il 13/08
			DataMsg5n1Full = 0x00;
			// disable interrupt to avoid new data corrupting the buffer
			pinMode(RXB6_DATA, INPUT);          	// data interrupt input
			// disable interrupt to avoid new data corrupting the buffer
			// --------------------------------------------------------------------------------------
			Rxb6DetachInterrupt();

			// these instructions are performed in 4sec
			// --------------------------------------------------------------------------------------
			// read battery voltage
			BatteryVoltage = getBatVoltage();
			// --------------------------------------------------------------------------------------
			// read temp/press from bmpX80 sensor
			Bmp180temp = bmp180.getTemperature();
			Bmp180press = bmp180.getPressure();
			// --------------------------------------------------------------------------------------
			StMain = SP_SEND_TTN		;
			break;
		default:
			StMain = SP_ON_ETH			;
			break;
		}
		break;
		/**
		**/
	case SP_WAIT02          :
		// wait some seconds (15sec) before to repeat the acquisition

		// ---------------------------------------------------
		DelayLowPower(17000L);

		// end delay. Return to read etherodina
		StMain = SP_ON_ETH			;
		break;
		
	case SP_SEND_TTN		:
		// send message to ttn
		// ---------------------------------------------------
		wdtReset();				// reset watchdog	
		// ---------------------------------------------------
		// send job
		os_setCallback(&sendjob, do_send);

		// ---------------------------------
		// modify 21/05/2019
		while(sleeping == false)
		{
			os_runloop_once();
		}
		sleeping = false;

		// ---------------------------------------------------
		time = (T_CYCLE_TIME_MSEC) - 18000L;
		DelayLowPower(time);
		
		// ----------------
		// tune timer watchdog 15msec
		cli();						// disable interrupt
		tune15MS();
		sei();						// enable interrupt
		// ----------------
		
		tmDelay = millis();

		StMain = SP_ON_ETH			;
		// -----------------------------------------------------
		
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

	InitRainCalc();							// init variables to calc rain fall

	
	// power off etherodina
	Rxb6Power(0);
	StateManager();
	
	//---------------------------------------------
	// init 5n1

	// initialize d5n1
	init_d5n1();
	
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
	// loop acquisition on acurite 5n1
	StateManager();
	// end loop 5n1
	// ---------------------------------------------------
}



// =======================================================================
// =======================================================================
#ifdef NOT_USED
// =======================================================================
// =======================================================================



#endif // NOT_USED

// =======================================================================
