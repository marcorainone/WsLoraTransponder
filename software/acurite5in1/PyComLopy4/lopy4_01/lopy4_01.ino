// **********************************************************************************
// Project:    Acurite 5n1 weather station lora tranceiver
//             v1 2019-03-01, ICTP Wireless Lab
//             Version for Arduino LOPY4 
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
#define LOWPOWER 1

#include <Arduino.h>
#include <soc/rtc.h>
#include <rom/rtc.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <driver/adc.h>
#include <esp_timer.h>			// original pos: esp_common/include/esp_timer.h

#include <SPI.h>
#include <Wire.h>
// use these local libraries
#include "src/arduino-lmic/lmic.h"
#include "src/arduino-lmic/hal/hal.h"
#include "src/BMP180/BMP180.h"

#include "lopy4_01.h"

// connection pin lopy4, BMP180 sensor, 
// Ver. 1.1 10/07/2019
//
// PIN SENSOR   | color  | Wiring to ESP32   | Pin Lopy4  | Note
//              | red    | Vin (3.5 .. 5.5V) | 28         | to +3.7V lithium battery
// VbatPin      | white  | ADC1 CH7 (GPIO35) | 19         | Vbatt analog input
// RXB6 GND     | black  | GND               | 27         | GND lithium battery
// BMP180 GND   | black  | GND               | 27         | GND lithium battery
// BMP180 POWER | red    | GPIO33            | 22         | 
// BMP180 SCL   | yellow | SCL GPIO13        | 12         | 
// BMP180 SDA   | white  | SDA GPIO12        | 11         | 
// RXB6 POWER   | red    | GPIO32            | 21         | 
// RXB6 DATA    | yellow | GPIO34            | 20         | 
// RXB6 antenna | green  |                   |            | 

// ---------------------------------------------------------------
// defines used for configuration
// ---------------------------------------------------------------

// mr 17/08:
// enable SERIAL_MSG if you want to print msg to serial
#define SERIAL_MSG

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
// default cycle time: 5min (300sec)
// #define TIME_TO_SLEEP  300000L			// Time ESP32 will go to sleep (in seconds)
#define T_CYCLE_TIME_MSEC	359500L			// 6min - 0.5sec
// modify for debug: reduced cycle for more frequent reception
// #define T_CYCLE_TIME_MSEC  72000L   		// Time ESP32 will go to sleep (in seconds)

// ---------------------------------------------------------------
// ID 5n1: default value
// ---------------------------------------------------------------

// id weather station connected to receiver.
const uint16_t id5n1WeatherStation = ID_5N1;

// id weather station connected to receiver splitted.
const uint8_t HiId5n1 = (id5n1WeatherStation >> 8) & 0x0F;
const uint8_t LoId5n1 = (id5n1WeatherStation & 0xFF);

// --------------------------------------------------------------------------------
// https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/deep-sleep-stub.html
// https://gist.github.com/igrr/54f7fbe0513ac14e1aea3fd7fbecfeab
// http://www.lucadentella.it/en/2018/01/22/esp32-29-deep-sleep/
// in deep sleep mode the content of the RTC fast and RTC slow memories is preserved. 
// You can therefore use those memory segments to store data that must be retained during the sleep.
// To ask the compiler to store a variable in the RTC slow memory, you can use: 
// - the RTC_DATA_ATTR attribute, or 
// - the RTC_RODATA_ATTR one if the variable is read only:
// example: RTC_DATA_ATTR static time_t last;
//
// RTC_DATA_ATTR bool FirstBoot = true;

// band 868MHz or 915MHz
#define BAND    868E6  					// 915E6

int sleepcycles = 0;
bool joined = false;
bool sleeping = false;
int CountLowPower = 0;

// if DataMsg5n1Full = 0x03, you have received the two types os 5n1 msg
RTC_DATA_ATTR int  DataMsg5n1Full = 0x00;

//-----------------------------------------------------------------
// timer utilities
//-----------------------------------------------------------------
// use High Resolution Timer esp_timer_get_time as base timing functions
// https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/system/esp_timer.html
unsigned long my_micros() 
{
    uint64_t usec = esp_timer_get_time();
    return (unsigned long)usec;
}
unsigned long my_millis() 
{
	unsigned long msec = (unsigned long)(esp_timer_get_time() / 1000L);
    return msec;
}
//-----------------------------------------------------------------
// hardware configuration
//-----------------------------------------------------------------
//
// -------------------------------------
// use data pin PD5 as output power for etherodina 
#define BMP180_POWER	(33)				// ESP32 GPIO n.33 (pin22 lopy4) for bmp180

// -------------------------------------
// use data pin PD5 as output power for etherodina 
#define RXB6_POWER		(32)				// ESP32 GPIO n.32 (pin21 lopy4) for power etherodina
// -------------------------------------

//-----------------------------------------------------------------
// bmp180 sensor
//-----------------------------------------------------------------
// 
// power on/off etherodina module bmp180
// note: do not use! Consumption increases
void Bmp180Power(int value)
{
	// modify 20/06: on power etherodina
	pinMode(BMP180_POWER, OUTPUT);			// output pin power on/off
	if(value>0)
	{
		digitalWrite(BMP180_POWER, HIGH);	// sets the digital pin BMP180_POWER on
	}
	else
	{
		digitalWrite(BMP180_POWER, LOW);	// sets the digital pin BMP180_POWER off
	}
}

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
	pinMode(RXB6_POWER, OUTPUT);				// output pin power on/off
	if(value>0)
	{
		digitalWrite(RXB6_POWER, HIGH);			// sets the digital pin RXB6_POWER on
	}
	else
	{
		digitalWrite(RXB6_POWER, LOW);			// sets the digital pin RXB6_POWER off
	}
}

//-----------------------------------------------------------------
// TTN CONNECTION
//-----------------------------------------------------------------
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
static u1_t NWKSKEY[16] = { 
	0x00, 0x01, 0x02, 0x03, 
	0x04, 0x05, 0x06, 0x07, 
	0x08, 0x09, 0x0A, 0x0B,
	0x0C, 0x0D  0x0E, 0x0F 
};

// LoRaWAN AppSKey, application session key
static u1_t APPSKEY[16] = { 
	0x00, 0x01, 0x02, 0x03, 
	0x04, 0x05, 0x06, 0x07, 
	0x08, 0x09, 0x0A, 0x0B,
	0x0C, 0x0D  0x0E, 0x0F 
};

// LoRaWAN end-device address (DevAddr)
static u4_t DEVADDR = 0x12345678;   // Put here the device id in hexadecimal form.

#else
// -------------------------------------------------------------
// KEYS FOR ABP ACCESS
// LoRaWAN NwkSKey, network session key

// for pycom lpyabp0002
static const PROGMEM u1_t NWKSKEY[16] = { 
	0x35, 0xD4, 0xD5, 0x88, 
	0x14, 0x8D, 0x2D, 0x98, 
	0xB6, 0xD6, 0xA0, 0x7D, 
	0x3D, 0x06, 0xC8, 0x2B 
};

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 
	0xD1, 0x9F, 0xF1, 0x05, 
	0x7D, 0x95, 0x85, 0x75, 
	0x08, 0xF6, 0x48, 0xDA, 
	0x45, 0x8E, 0x09, 0x8C 
};

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x12345678;   // Put here the device id in hexadecimal form.

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { };
void os_getDevEui (u1_t* buf) { };
void os_getDevKey (u1_t* buf) { };

#endif			// #ifdef TTN_ACCESS_OTAA

// --------------------------------------------

static osjob_t sendjob;
static osjob_t initjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 300;

// Pin mapping lopy4:
// see header: 
// Arduino\Hardware\espressif\esp32\variants\lopy4\pins_arduino.h
const lmic_pinmap lmic_pins = {
	.mosi = 27,
	.miso = 19,
	.sck = 5,
	.nss = 18,
	.rxtx = LMIC_UNUSED_PIN,
	.rst = LMIC_UNUSED_PIN,
	.dio = {23, 23, 23}, //workaround to use 1 pin for all 3 radio dio pins
};

// send message to lorawan
void do_send(osjob_t* j)
{
	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND) 
	{
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
		// Unknown event
		break;
	}
}

// start connection to TTN. This function must call by setup()
void InitTTNConnection()
{
	// LMIC init
	Serial.println("InitTTNConnection os-init...");
	os_init();

	Serial.println("InitTTNConnection lmic-reset...");
	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

	// Set static session parameters.
	#ifdef PROGMEM
	// On AVR, these values are stored in flash and only copied to RAM
	// once. Copy them to a temporary buffer here, LMIC_setSession will
	// copy them into a buffer of its own again.
	uint8_t appskey[sizeof(APPSKEY)];
	uint8_t nwkskey[sizeof(NWKSKEY)];
	memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
	memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
	LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
	#else
	// If not running an AVR with PROGMEM, just use the arrays directly
	LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
	#endif

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

	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// TTN uses SF9 for its RX2 window.
	LMIC.dn2Dr = DR_SF9;

	// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(DR_SF9,14);

	Serial.println("InitTTNConnection inizio do_send...");
	// Start job
	do_send(&sendjob);
	Serial.println("Exit InitTTNConnection...");
}

//-----------------------------------------------------------------
// end ttn utilities
//-----------------------------------------------------------------

RTC_DATA_ATTR state_program StMain = SP_START;			// state program
unsigned long tmDelay;
unsigned long tmTimeout;

const uint8_t vbatPin = 35;

// Measure supply voltage through resistors divider
// 
// define K_VBATT     (1.1 * 4.9) / 4095.0
#define K_VBATT     3450

// calc vbatt floating point version
#define K_VBATT_FLOAT_MUL_10		(11 * 4.9) / 4095.0 

// The ESP32 has 18 x 12 bits ADC input channels.
// These are the GPIOs that can be used as ADC and respective channels:
// GPIO Pin  label        I/O        acd channel
// 35   R 19 GPIO_NUM_35  ONLY INPUT ADC1_CH7

unsigned int getBatVoltage() {
	float ris;
	
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_0);
	int val = adc1_get_raw(ADC1_CHANNEL_7);
	
	ris = ((float)(val)) * K_VBATT_FLOAT_MUL_10; 		// battery voltage in volts
	return (unsigned int)ris;
}

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
	// predisposition. Todo for lopy4
}
// deactive watchdog timer
void wdtDisable(void)
{
	// predisposition. Todo for lopy4
}
// reset watchdog	
void wdtReset()
{
	// predisposition. Todo for lopy4
}

// use watchdog to restart module
void ResetSystem()
{
	// predisposition. Todo for lopy4
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

// use watchdog to restart arduino
void ResetSystem()
{
}

#endif						// #ifdef ENABLE_WATCHDOG

//-----------------------------------------------------------------
// rx 5n1
//-----------------------------------------------------------------
//
// Functions to decode 5n1 Acurite device:
// 1) AcuRite 06014 PRO+ 5-in-1 Weather Sensor with Rain Gauge, Wind Speed, Wind Direction, Temperature and Humidity (vn1txca2)
//    https://www.amazon.com/AcuRite-Weather-Direction-Temperature-Humidity/dp/B00SN1WHEU/ref=sr_1_7?s=home-garden&ie=UTF8&qid=1528101292&sr=1-7&keywords=acurite+5+in+1+weather+station
//
// defines for acurite modules managed by software

#define SYNC_HIGH       600
#define SYNC_LOW        600

#define PULSE_LONG      400
#define PULSE_SHORT     220

// original:
#define PULSE_RANGE     100
// test 07/04: verified that statistically the duration of the message is approximately 43msec.
// The error delta for each pulse is 10usec. 20usec is used as a pulse range
// #define PULSE_RANGE     20

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

// --------------------------------------------------------------------------------------
// INTERRUPT CONFIGURATION
// --------------------------------------------------------------------------------------
// Connect to the controller the RXB6 data pin (RXB6_DATA). This pin will be 
// toggling with the incoming data from the RXB6 RF module.
// It can be configured for interrupt on change, change to high or low.
//
// specific defines for module lopy4
// RXB6 datapin
#define RXB6_DATA			(34)             		// ESP32 GPIO n.34 (pin20 lopy4) pin change interrupt

#define SYNCPULSECNT    (4)             			// 4 pulses (8 edges)
#define SYNCPULSEEDGES  (SYNCPULSECNT*2)

#define DATABYTESCNT_MIN (7) 						// Minimum number of data bytes
#define DATABITSCNT_MIN     (DATABYTESCNT_MIN*8)	// 7 bytes * 8 bits
#define DATABITSEDGES_MIN   (DATABITSCNT_MIN*2)

#define DATABYTESCNT_MID 128 						// 8 Bytes

#define DATABYTESCNT_MAX (9) 						// 9 Bytes
#define DATABITSCNT_MAX     (DATABYTESCNT_MAX*8)	// 7 bytes * 8 bits
#define DATABITSEDGES_MAX   (DATABITSCNT_MAX*2)

// 5n1 Tower Message Types
#define  MT_WS_WD_RF  49    						// wind speed, wind direction, rainfall
#define  MT_WS_T_RH   56    						// wind speed, temp, RH

#define eventTimeoutms 7200000  					// Timeout in miliseconds before an event is over

// macros from DateTime.h 
// Useful Constants 
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

// Useful Macros for getting elapsed time
#define numberOfSeconds(_time_) 	(_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) 	((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) 		(( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) 		( _time_ / SECS_PER_DAY)  


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
volatile unsigned long StartSeqUsec, StopSeqUsec, deltaSeqUsec;
volatile unsigned long usecStartAcqMsg;	// start time acq. msg in useconds
volatile unsigned long uSecNextMsg;			// microsecondi di attesa prima di ricevere prossimo messaggio
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

RTC_DATA_ATTR data_5n1 d5n1;

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
	TmrStop = my_micros();
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
			crc5n1 = 0;					// inizializza il crc calcolato del messaggio
			usecStartAcqMsg = my_micros();			// start time acquisition msg in microseconds
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
			wsByteMsg = 0;				// inizializza il byte del messaggio
		}
		wsByteMsg = wsByteMsg << 1;
		if( delta>=(BIT0_HIGH_MIN) && delta<=(BIT0_HIGH_MAX) )
		{
			bitStatus = 0x00;				// il bit e' 0.
		}
		else if( delta>=(BIT1_HIGH_MIN) && delta<=(BIT1_HIGH_MAX) )
		{
			bitStatus = 0x01;				// il bit e' 1.
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
					crcOk = decodeFld5n1((uint8_t *)msg5n1);
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
			StopSeqUsec = my_micros();
			deltaSeqUsec = StopSeqUsec - StartSeqUsec;
			// Serial.print("Delta: " ); Serial.println(deltaSeqMsec);
			StartSeqUsec = my_micros();
			// calculate how long the message reception phase has lasted
			deltaTmrMsg = my_micros() - TmrMsgBegin;
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
			detachInterrupt(digitalPinToInterrupt(RXB6_DATA));
			Rxb6Power(0);								// off power rxb6 433Mhz Receiver
			// =============================
		}
	}
	else
	{
		// errore nel messaggio
		nMsgReceived = 0;			// reinizializza la ricezione del messaggio
		nPulses = -4;				// ricomincia la ricezione sync
		crc5n1 = 0;
	}
}

// ----------------------------------------------------------
// use external pin interrupt int2 (pin0) or int3 (pin1)

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR Rxb6AttachInterrupt()
{
	portENTER_CRITICAL_ISR(&mux);						// disable interrupt
	attachInterrupt(digitalPinToInterrupt(RXB6_DATA), Rxb6InterruptHandler, CHANGE);
	portEXIT_CRITICAL_ISR(&mux);;						// enable interrupt
}
// disable interrupt to avoid new data
void IRAM_ATTR Rxb6DetachInterrupt()
{
	portENTER_CRITICAL_ISR(&mux);						// disable interrupt
	detachInterrupt(digitalPinToInterrupt(RXB6_DATA));
	portEXIT_CRITICAL_ISR(&mux);;						// enable interrupt
}

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
		d5n1.time = my_millis();

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
				// Serial.print(getTimeSpan(rainLast, my_millis()));
				d5n1.RainTmSpan = (my_millis() - rainLast) / 1000;
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

// set sleep delay taking note start of message
void SetDelaySleep(unsigned long delay_usec)
{
	unsigned long timeSleep;	// sleep time

	// ---------------------------------------------------
	// calc sleep time in microseconds
	timeSleep = my_micros() - usecStartAcqMsg;
	timeSleep = delay_usec - timeSleep;		// timesleep in microseconds
	// nota: tieni conto del tempo di start 1.5sec
	timeSleep = timeSleep - 1500000L;		// timesleep in microseconds
	
	wdtReset();					// reset watchdog	
	wdtDisable();				// deactive watchdog timer
	// cpu frequency settings
	// https://github.com/espressif/esp-idf/blob/master/components/soc/esp32/include/soc/rtc.h
	// typedef enum {
	//     RTC_CPU_FREQ_XTAL = 0,      //!< Main XTAL frequency
	//     RTC_CPU_FREQ_80M = 1,       //!< 80 MHz
	//     RTC_CPU_FREQ_160M = 2,      //!< 160 MHz
	//     RTC_CPU_FREQ_240M = 3,      //!< 240 MHz
	//     RTC_CPU_FREQ_2M = 4,        //!< 2 MHz
	// } rtc_cpu_freq_t;
	// imposta 2Mhz
	// verified: in this state the frequency change does not lead to consumption reductions
	// rtc_clk_cpu_freq_set(RTC_CPU_FREQ_2M);
	// light sleep
	// originale 
	esp_sleep_enable_timer_wakeup(timeSleep); 		// sleep in usec
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
		tmDelay = my_micros();
		StMain = SP_WAIT01			;
		break;
	case SP_WAIT01			:
		// ---------------------------------------------------
		wdtReset();				// reset watchdog	
		// ---------------------------------------------------
		time = my_micros();
		// wait 100msec
		if((time - tmDelay) < 100000L)
		{
			break;
		}
		// --------------------------------------
		// start etherodina rx
		// initialize acquisition 5n1 messages
		Rxb6InitializeMsgAcquisition();
		
		// enable interrupt etherodina
		pinMode(RXB6_DATA, INPUT);          	// data interrupt input
		// --------------------------------------------------------------------------------------
		Rxb6AttachInterrupt();
		// --------------------------------------------------------------------------------------

		tmTimeout = my_micros();				// timeout rx
		StMain = SP_DECODE_ETH		;
		break;
	case SP_DECODE_ETH		:
		// wait until a message is received 
		if(DecodeEthManager()==false)
		{
			// check timeout rx
			time = my_micros();
			if((time - tmTimeout) >= 20000000L)
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
			// --------------------------------------------------------------------------------------
			// read battery voltage
			// BatteryVoltage = getBatVoltage();
			// --------------------------------------------------------------------------------------
			// read temp/press from bmp180 sensor
			// Bmp180temp = bmp180.getTemperature();
			// Bmp180press = bmp180.getPressure();
			
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
		// wait some seconds (17sec) before to repeat the acquisition
		// ---------------------------------------------------
		SetDelaySleep(17000000L);			// 17 sec
		// esp_light_sleep_start();
		esp_deep_sleep_start();

		// end delay. Return to read etherodina
		// StMain = SP_ON_ETH			;
		break;
	case SP_SEND_TTN		:
		// send message to ttn
		// --------------------------------------------
		// start connection to TTN. This function must call by setup()
		Serial.println("InitTTNConnection...");
		InitTTNConnection();
		delay(500);
		Serial.println("End InitTTNConnection exit setup...");
		sleeping = false;
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
		
		// -----------------------------------------
		// deep sleep
		time = (T_CYCLE_TIME_MSEC - 18000L) * 1000L;			// time in microseconds
		SetDelaySleep(time);
		esp_deep_sleep_start();
		break;
		
	case SP_RESETSYSTEM		:
		ResetSystem();				// use watchdog to restart arduino
		break;
		
		default					:
		// for every other states, do reset
		StMain = SP_START			;
		break;
	}
}

//-----------------------------------------------------------------
// setup function
//-----------------------------------------------------------------
void setup() {
	int i, j;

    // Set the wake stub function
    // esp_set_deep_sleep_wake_stub(&wake_stub);

	// cpu frequency settings
	// https://github.com/espressif/esp-idf/blob/master/components/soc/esp32/include/soc/rtc.h
	// typedef enum {
	//     RTC_CPU_FREQ_XTAL = 0,      //!< Main XTAL frequency
	//     RTC_CPU_FREQ_80M = 1,       //!< 80 MHz
	//     RTC_CPU_FREQ_160M = 2,      //!< 160 MHz
	//     RTC_CPU_FREQ_240M = 3,      //!< 240 MHz
	//     RTC_CPU_FREQ_2M = 4,        //!< 2 MHz
	// } rtc_cpu_freq_t;
	// set frequency to 80Mhz
	rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
	// attention: in lopy the 2MHz frequency should not be set
	// rtc_clk_cpu_freq_set(RTC_CPU_FREQ_2M);
	yield();
	Rxb6Power(0)			;		// off etherodina

	// off wifi e bluetooth
	// In deep sleep and light sleep modes, wireless peripherals are powered down. 
	// Before entering deep sleep or light sleep modes, applications must disable WiFi and BT using appropriate calls.
	// WiFi and BT connections will not be maintained in deep sleep or light sleep, even if these functions are not called.
	// originale
	esp_bluedroid_disable(); 
	esp_bt_controller_disable(); 
	esp_wifi_stop();
	/***
	// note: consumption are the same
	// mr 25/06
	// https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiBlueToothSwitch/WiFiBlueToothSwitch.ino
	 WiFi.mode(WIFI_OFF);
     btStop();
	***/

	// http://esp32.info/docs/esp_idf/html/db/dc1/group__rtc__apis.html
	// https://gist.github.com/igrr/54f7fbe0513ac14e1aea3fd7fbecfeab
    // get the reset cause
	if(rtc_get_reset_reason(0) != DEEPSLEEP_RESET) 
	{
		// possible cause of reset:
		// NO_MEAN                :  0  
		// POWERON_RESET          :  1, Vbat power on reset
		// 2                      :  2, ???
		// SW_RESET               :  3, Software reset digital core
		// OWDT_RESET             :  4, Legacy watch dog reset digital core
		// DEEPSLEEP_RESET        :  5, Wake up from deep sleep
		// SDIO_RESET             :  6, Reset by SLC module, reset digital core
		// TG0WDT_SYS_RESET       :  7, Timer Group0 Watch dog reset digital core
		// TG1WDT_SYS_RESET       :  8, Timer Group1 Watch dog reset digital core
		// RTCWDT_SYS_RESET       :  9, RTC Watch dog Reset digital core
		// INTRUSION_RESET        :  10, Instrusion tested to reset CPU
		// TGWDT_CPU_RESET        :  11, Time Group reset CPU
		// SW_CPU_RESET           :  12, Software reset CPU
		// RTCWDT_CPU_RESET       :  13, RTC Watch dog Reset CPU
		// EXT_CPU_RESET          :  14, for APP CPU, reseted by PRO CPU
		// RTCWDT_BROWN_OUT_RESET :  15, Reset when the vdd voltage is not stable
		// RTCWDT_RTC_RESET       :  16, RTC Watch dog reset digital core and rtc module		
		//
		// Wake up cause different from deep sleep
		//---------------------------------------------
		// Initialize non volatile data
		init_d5n1();						// init 5n1
		DataMsg5n1Full = 0x00;
		StMain = SP_START;					// reset state machine
		StateManager();
		//---------------------------------------------
		Serial.begin(115200);
		Serial.println("Reset lpacu01 ...");
		// --------------------------------------------
    }
	else
	{
		// ------------------------------------------------
		// deepsleep reset
		Serial.begin(115200);
		Serial.println("Deepsleep reset lpacu01 ...");
		// operations after deepsleep
		switch(StMain)
		{
		case SP_WAIT02          :
		case SP_SEND_TTN		:
			// after send message to ttn
			// ------------------------------------------
			wdtEnable();							// active watchdog with 8sec timing
			tmDelay = my_micros();
			StMain = SP_ON_ETH			;
			break;
		}
	}

	// Do not manage on / off the bmp180 board. Increases consumption in deepsleep
	// Bmp180Power(1);			// power on bmp180
	// delay(500);
	
	// init bmp180 sensor
	bmp180.init();
	if (!bmp180.hasValidID())
	{
#ifdef SERIAL_MSG
		Serial.println("Error - please check the BMP180 board!");
#endif
		for(i=0;i<3;i++)
		{
			bmp180.reset();
			delay(100);
		}
	}

	// --------------------------------------------------------------------------------------
	// mod 01/12: read battery voltage
	BatteryVoltage = getBatVoltage();
	// --------------------------------------------------------------------------------------
	// mod 10/12: read temp/press from bmp180 sensor
	for(i=0;i<3;i++)
	{
		Bmp180temp = 0L;
		for(j=0;j<5;j++)
		{
			// calc average 
			Bmp180temp = Bmp180temp + bmp180.getTemperature();
			delay(10);
		}
		Bmp180temp = Bmp180temp / 5;
		// temp limits * 10
		if((Bmp180temp>-400) && (Bmp180temp<1000))
		{
			// value is on limits
			break;
		}
		// wrong value. Reinitialize sensor
		Bmp180temp = 0L;
		bmp180.init();
		delay(100);
	};
	// limits pressure:
	// https://sciencing.com/range-barometric-pressure-5505227.html
	// The highest air pressure recorded was 1084 hPa in Siberia. 
	// The lowest air pressure, 870 hPa, was recorded in a typhoon in the Pacific Ocean.
	// Atmospheric pressure on Mount Everest summit: 337hPa
	for(i=0;i<3;i++)
	{
		Bmp180press = 0L;
		for(j=0;j<5;j++)
		{
			// calc average 
			Bmp180press = Bmp180press + bmp180.getPressure();
			delay(10);
		}
		Bmp180press = Bmp180press / 5;
		if((Bmp180press>33700) && (Bmp180press<110000))
		{
			// value is on limits
			break;
		}
		// wrong value. Reinitialize sensor
		Bmp180press = 0L;
		bmp180.init();
		delay(100);
	};
	// ------------------------------------
	// check:
	// Serial.print("Temperature: ");
	// Serial.print(Bmp180temp);
	// Serial.println(" C * 10");

	// Serial.print("Pressure: ");
	// Serial.print(Bmp180press);
	// Serial.println(" hPa * 100");
	
	// Bmp180Power(0);			// power off bmp180
	// delay(500);
	// ------------------------------------
	
	while(1)
	{
		StateManager();
		yield();
	}
}

// ================================================================
// loop: 
// the deep sleep in esp32 cause a reset, so the loop function is not executed 
void loop() {

	// ---------------------------------------------------
	// loop 5n1
	// end loop 5n1
	// ---------------------------------------------------
}

// =======================================================================
