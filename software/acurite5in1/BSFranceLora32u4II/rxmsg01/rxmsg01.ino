// **********************************************************************************
// Project:    RX test program for Acurite 5n1 weather station
//             v1 2018-12-01, ICTP Wireless Lab
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
#define LOWPOWER 1

#include <Arduino.h>
#include <SPI.h>
#include <avr/power.h>

#ifndef __AVR_ATmega32U4__
#define __AVR_ATmega32U4__
#endif
#include <Wire.h>
#include <avr/interrupt.h>    // Needed to use interrupts
#include "src/LowPower/LowPower.h"
#include "rxmsg01.h"

// ---------------------------------------------------------------
// defines used for configuration
// ---------------------------------------------------------------

// enable SERIAL_MSG if you want to print msg to serial
#define SERIAL_MSG

// CONFIGURATION RECEIVING MESSAGES ACURITE 5N1
//
// ------------------------------- 1) #define CHECK_5N1ID
// if disabled it allows to accept any message from any control unit.
// If CHECK_5N1ID is enabled, the interrupt only recognizes the ECU messages with ID provided in id5n1WeatherStation
// currently CHECK_5N1ID is disabled to be able to read messages from any control unit
// #define CHECK_5N1ID
//
// ------------------------------- 2) #define CHECK_CHANNEL
// define CHECK_CHANNEL if you want to check the transmission channel
// #define CHECK_CHANNEL

// ---------------------------------------------------------------
// ID 5n1: default value
// ---------------------------------------------------------------

// id weather station connected to receiver.
const uint16_t id5n1WeatherStation = 0x0C24;

// id weather station connected to receiver splitted.
// these variables are used to recognize the 5n1 ID in the interrupt function
const uint8_t HiId5n1 = (id5n1WeatherStation >> 8) & 0x0F;
const uint8_t LoId5n1 = (id5n1WeatherStation & 0xFF);

//-----------------------------------------------------------------
// hardware configuration
//-----------------------------------------------------------------
//
// -------------------------------------
// see:
// https://docs.bsfrance.fr/documentation/11355_LORA32U4II/LoRa32u4II_pinout_diagram.pdf

#define RXB6_POWER		(5)             // Pin5 PC6 for power etherodina
#define LEDPIN 			(13)

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

//-----------------------------------------------------------------
// RXB6 433Mhz Superheterodyne Wireless Receiver Module
//-----------------------------------------------------------------
// 
// https://www.ebay.it/itm/RXB6-433Mhz-Superheterodyne-Wireless-Receiver-Module-for-Arduino-ARM-AVR-/301953017682
// https://www.amazon.it/WINGONEER-Superheterodyne-Wireless-Receiver-Arduino/dp/B06XHJMC82
//
// https://docs.bsfrance.fr/documentation/11355_LORA32U4II/LoRa32u4II_pinout_diagram.pdf
// POWER PIN5 PC6
// power on/off etherodina module RXB6
void Rxb6Power(int value)
{
	// modify 20/06: on power etherodina
	// pinMode(RXB6_POWER, OUTPUT);				// output pin power on/off
	DDRC |= 0b01000000;
	if(value>0)
	{
		PORTC |= 0b01000000;
		// digitalWrite(RXB6_POWER, HIGH);		// sets the digital pin RXB6_POWER on
	}
	else
	{
		PORTC &= 0b10111111;
		// digitalWrite(RXB6_POWER, LOW);		// sets the digital pin RXB6_POWER off
	}
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

//
// types of arduino provided by the module
#define ARD_UNO				0				// arduino UNO
#define ARD_BSFLORA32U4		1				// bsfrance lora f32u4

// define module type used
#define	MOD_ARDUINO			ARD_BSFLORA32U4

#define SYNC_HIGH       600
#define SYNC_LOW        600

#define PULSE_LONG      400
#define PULSE_SHORT     220

// originale:
// #define PULSE_RANGE     100
// test 07/04: a livello statistico la durata del messaggio e' di 43msec circa.
// Il delta di errore per ogni impulso e' 10usec. Come pulse range metto 20usec
#define PULSE_RANGE     20

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

#endif

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

// 5n1 Tower Message Types
#define  MT_WS_WD_RF  49    // wind speed, wind direction, rainfall
#define  MT_WS_T_RH   56    // wind speed, temp, RH

// channel codes used by acurite 5n1
#define	CHAN_A	0x03
#define	CHAN_B	0x02
#define	CHAN_C	0x00

// if defined CHECK_CHANNEL, this variable store the channel to verify
volatile uint8_t ChannelWS5n1 = CHAN_C;			// select only this channel

// ---------------------------------

// --------------------------------------------
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
	int highbits, lowbits;
	
	Fld5n1.crc[0] = msg[7];
	Fld5n1.crc[1] = msg[8];
	
	lowbits = msg[0] >> 4;
	Fld5n1.rpt = lowbits & 0x03;				// field Rpt
	Fld5n1.ch = lowbits >> 2;					// field channel
	Fld5n1.id5n1 = ((msg[0] & 0x0F) << 8) | msg[1];
	Fld5n1.batt = msg[2] >> 6;					// battery status
	Fld5n1.idm = msg[2] & 0x3F;					// id message
	highbits = ( msg[3] & 0x1F) << 3;
	lowbits = ( msg[4] & 0x70 ) >> 4;
	Fld5n1.WindSpeed	= highbits | lowbits;
	if (Fld5n1.idm == '1')
	{
		// MT_WS_WD_RF='1': msg Wind Speed, Direction and Rainfall messaggio '1'
		Fld5n1.fld.t1.Direction	= msg[4] & 0x0F;
		highbits = ( msg[5] & 0x7f) << 7;
		lowbits = ( msg[6] & 0x7F);
		Fld5n1.fld.t1.Rainfall		= highbits | lowbits;
	}
	else
	{
		// MT_WS_T_RH ='8': msg Wind speed, Temperature, Humidity messaggio '8'
		highbits = ( msg[4] & 0x0F) << 7 ;
		lowbits = ( msg[5] & 0x7F);
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

// var used to message rx
volatile unsigned long TmrMsgBegin, deltaTmrMsg;		// used to calculate the duration in microseconds of the message recognition time
volatile unsigned long TmrStart, TmrStop, delta;
volatile unsigned long StartSeqMsec, StopSeqMsec, deltaSeqMsec;
volatile unsigned long uSecNextMsg;						// microseconds next message
volatile int nPulses;

volatile uint8_t nMsgReceived = 0; 						// n. messages actually stored in msg5n1 vector
volatile uint8_t msg5n1[9];								// message acurite 5n1 (8 byte + crc calcolated)
volatile uint8_t crcOk;
// note:
// intervals rx msg in msec: you receive 2 messages at intervals of 18 seconds between them
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
// SEQUENCE RX OF LEVEL CHANGES:
// - 8 level changes for IDLE pulses
// - 8-byte message reception:
// nbit = 8 * 8 = 64bit
// nlivels = nbit * 2 = 128
// In total for each message, you have these level changes
// TotLivelli = 128 + 8 = 136 levels
// 
void Rxb6InterruptHandler() 
{
	static unsigned int bitState  = 0;
	static uint8_t wsByteMsg;			// status of the byte received from the weather station
	static uint8_t crc5n1;				// 
	static uint8_t fErrMsg;				// true if error msg

	int idx, nbit;
	uint8_t value, bitStatus;
	unsigned long svTmrStart;

	svTmrStart = TmrStart;
	TmrStop = micros();
	delta = TmrStop - TmrStart;
	TmrStart = TmrStop;
	bitState = digitalRead(RXB6_DATA);

	// ignore if we haven't finished processing the previous 
	// received signal in the main loop.
	if( nMsgReceived != 0)
	{
		// il main loop non ha ancora completato il lavoro sul buffer messaggi
		return;
	}
	if(bitState == HIGH)
	{
		// signal transition: 1 -> 0 -> 1
		// 1 --+        +-- 1
		//     |        |
		// 0   +--------+   0
		// do not manage this area
		return;
	}
	
	// (bitState == LOW)
	// ---------------------------------------------------------
	// signal transition: 0 -> 1 -> 0
	// 1   +--------+   1
	//     |        |
	// 0 --+        +-- 0
	// check duration of 0 -> 1 -> 0

	fErrMsg = false;
	if(nPulses<0)
	{
		// phase of receiving 4 sync pulses
		if( delta>=(SYNC_HIGH-PULSE_RANGE) && delta<=(SYNC_HIGH+PULSE_RANGE) )
		{
			// check if this is the first sync 
			// In this case initialize the timer that estimates the beginning of the next message
			if(nPulses == -4)
			{
				uSecNextMsg = svTmrStart;
				TmrMsgBegin = svTmrStart;
			}
		}
		else if( delta>=(BIT0_HIGH_MIN) && delta<=(BIT0_HIGH_MAX) )
		{
			// Maybe you have lost several sync bits. Try to retrieve the analysis
			nPulses = 0;				// bit is 0.
		}
		else if( delta>=(BIT1_HIGH_MIN) && delta<=(BIT1_HIGH_MAX) )
		{
			// Maybe you have lost several sync bits. Try to retrieve the analysis
			nPulses = 0;				// bit is 1.
		}
		else
		{
			fErrMsg = true;
		}
		if(nPulses>=0)
		{
			// initialize the variables used for message recognition
			crc5n1 = 0;					// initialize the calculated crc of the message
		}	
	}

	if(nPulses>=0)
	{
		// decode the hi part of the received bit
		idx = nPulses >> 3;
		nbit = nPulses & 0x07;
		if(nbit == 0)
		{
			// store the first bit.
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
			// the impulse is with correct duration (0 or 1).
			// store the bit
			
			// Check the status of some bit fields already acquired
			if(idx==0)
			{
				// check the byte0 of the message
				if(nbit == 1)
				{
					// define CHECK_CHANNEL if you want to check the transmission channel
#ifdef CHECK_CHANNEL
					
					// received the first 2 bits identifying channel A, B, C
					if(wsByteMsg != ChannelWS5n1)
					{
						// received the data of a control unit on a different channel.
						fErrMsg = true;
					}
#endif
				}
				else if(nbit == 3)
				{
					// received the 2 bits indicating the n. of repetition
					value = wsByteMsg & 0x03;
					if(value == 1)
					{
						// this is the first copy of the message.
						// The first copy of the next message will be received in 18 seconds
						// uSecNextMsg = uSecNextMsg + 18000000L;
						uSecNextMsg = 17980000L;
					}
					else if(value == 2)
					{
						// this is the second copy of the message.
						// The first copy of the next message will be received in about (18 seconds - 45msec) around 
						// uSecNextMsg = uSecNextMsg + 17900000L;
						uSecNextMsg = 17900000L;
					}
					else
					{
						// n. repetition other than 1 or 2
						fErrMsg = true;
					}
				}
				else if(nbit == 7)
				{
					// define CHECK_5N1ID if you want to check the id of the control unit
#ifdef CHECK_5N1ID
					// received the 4 bits that express the n. HiId5n1
					if(HiId5n1 != (wsByteMsg & 0x0F))
					{
						// received the data of a different unit.
						fErrMsg = true;
					}
#endif
				}
			}
			else if(idx==1)
			{
				// byte1 Message
				if(nbit == 7)
				{
					// define CHECK_5N1ID if you want to check the id of the weather station
#ifdef CHECK_5N1ID
					// received 8 bits expressing the value LoId5n1
					if(LoId5n1 != (wsByteMsg))
					{
						// received the data of a different unit.
						fErrMsg = true;
					}
#endif
				}
			}
			else if(idx==2)
			{
				// checks on byte2 of the message
				if(nbit == 7)
				{
					// received the 6 bits that express the message code
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
				// calc crc
				if(idx<7)
				{
					// continue crc calc
					crc5n1 = crc5n1 + wsByteMsg;
				}
				else
				{
					// last byte of the received message
					// store the calculated crc
					msg5n1[8] = crc5n1;
					// message decode and verify crc
					crcOk = decodeFld5n1(msg5n1);
				}
			}
		}
	}

	if(fErrMsg == false)
	{
		// inc the number of pulses received
		nPulses++;					// index of next pulse
		if(nPulses>=64)
		{
			nPulses = -4;				// restart to receive the sync pulses
			StopSeqMsec = millis();
			deltaSeqMsec = StopSeqMsec - StartSeqMsec;
			// Serial.print("Delta: " ); Serial.println(deltaSeqMsec);
			StartSeqMsec = millis();
			// calcola la durata di ricezione del messaggio
			deltaTmrMsg = micros() - TmrMsgBegin;
			nMsgReceived = 1;			// 1 message received
		}
	}
	else
	{
		// message error
		nMsgReceived = 0;			// restart the message rx
		nPulses = -4;				// restart to receive the sync pulses
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

// end rx 5n1
//-----------------------------------------------------------------

// ================================================================
// setup

void setup() {
	
	wdtDisable();							// deactive watchdog timer
	BlinkLed(250);
	wdtEnable();							// active watchdog with 8sec timing

	// start SPI
	SPI.begin();

#ifdef SERIAL_MSG
	Serial.begin(115200);
#endif

#ifdef SERIAL_MSG	
	Serial.println("Starting...");
#endif

	//---------------------------------------------
	// power on etherodina
	TmrStart = micros();
	nPulses = -4;
	crcOk = false;
	nMsgReceived = 0;
	StartSeqMsec = millis();
	TmrStart = micros();

	Rxb6Power(1);
	Rxb6AttachInterrupt();
	
}

// define DBG_WITH_MONITOR_USB if you want to debug with serial messages activated via usb
#define		DBG_WITH_MONITOR_USB

// function used to call the sleep mode during the recognition of the message pulses.
// Note: during idle the USB port works, so it can be used in the debug phase
void LPwaitMsg(period_t period)
{
#ifdef	DBG_WITH_MONITOR_USB
	//--------------------------------------------------
	// use idle, as it does not block the usb serial port
	// low power sleep
	// ok. idle works. ALSO WORKS USB
	// average consumption of message recognition and idle enabled: 14.8mA
	LowPower.idle(period, ADC_OFF,
	TIMER4_OFF, TIMER3_OFF,
	TIMER1_OFF, TIMER0_ON,
	SPI_OFF, USART1_OFF, TWI_OFF, USB_ON);

#else
	//--------------------------------------------------
	// use standby, it consumes less than idle but blocks the usb serial port
	// low power with standby
	// StandBy: IT WORKS, BUT THE USB IS LOCKED
	// average message recognition consumption and standby enabled: 9.07mA
	LowPower.powerStandby(period, ADC_OFF, BOD_OFF);

	//--------------------------------------------------
#endif 					// #ifdef	DBG_WITH_MONITOR_USB
}

volatile uint8_t msg1[9];

// ================================================================
// loop acurite message acquisition
//
void loop() {
	int i;

	// ---------------------------------------------------
	// loop 5n1
	static unsigned int bitState  = 0;
	uint8_t *ptr;

	//-------------------------------------------------
	if(nMsgReceived == 0)
	{
		// sleep until interrupt wake up
		LPwaitMsg(SLEEP_FOREVER);
	}
	else
	{
		// copy the message received
		// to save time, re-enable reception
		nPulses = -4;
		for(i=0;i<9;i++)
		{
			msg1[i] = msg5n1[i];
		}
		nMsgReceived = 0;
		
		// ---------------------------------------------
		// print array 9 bytes with rx msg:
		// byte 0..6: 7 bytes of 5n1 message
		// byte 7:    message crc sent by 5n1
		// byte 8:    crc calculated. Message OK if <crc message> = <crc calculated>
		Serial.print("[" ); 
		PrintHex8(msg1, 9);
		Serial.print("]" ); 

		// print deltaTmrMsg: time duration in microseconds of the message recognition
		Serial.println(deltaTmrMsg);
		// ---------------------------------------------

		TmrStart = micros();
	}
}

// =======================================================================
