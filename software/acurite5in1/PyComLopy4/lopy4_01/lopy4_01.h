#ifndef LOPY4_01_H   
#define LOPY4_01_H

#ifndef byte
	typedef unsigned char byte;    
#endif

enum state_program
{
	SP_START			,
	SP_JOINED_TTN		,
	SP_ON_ETH			,
	SP_WAIT01			,
	SP_READ_ETH			,
	SP_DECODE_ETH		,
	SP_OFF_ETH			,
	SP_SEND_TTN			,
	SP_SEND_TTN1		,
	SP_WAIT02			,
	SP_LOWPOWER			,
	SP_RESETSYSTEM
};

// configuration

// ---------------------------------------------------
// WATCHDOG
// define ENABLE_WATCHDOG if you want to manage watchdog
// #define ENABLE_WATCHDOG

#ifdef ENABLE_WATCHDOG
// ----------------------------------------------
// WATCHDOG ENABLED

#include <avr/wdt.h>

// active watchdog with 8sec timing
void wdtEnable(void);
// deactive watchdog timer
void wdtDisable(void);
// reset watchdog	
void wdtReset(void);


#endif						// #ifdef ENABLE_WATCHDOG

#endif		// LOPY4_01_H