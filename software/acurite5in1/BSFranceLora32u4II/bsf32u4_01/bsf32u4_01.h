#ifndef BSF32U4_01_H   
#define BSF32U4_01_H

#ifndef byte
	typedef unsigned char byte;    
#endif

enum state_program
{
	SP_START			,
	SP_JOINED_TTN		,
	SP_ON_ETH			,
	SP_WAIT01			,
	SP_DECODE_ETH		,
	SP_OFF_ETH			,
	SP_SEND_TTN			,
	SP_WAIT02			,
	SP_RESETSYSTEM
};

// configuration

// ---------------------------------------------------
// WATCHDOG
// define ENABLE_WATCHDOG if you want to manage watchdog
// #define ENABLE_WATCHDOG

#include <avr/wdt.h>

// active watchdog with 8sec timing
void wdtEnable(void);
// deactive watchdog timer
void wdtDisable(void);
// reset watchdog	
void wdtReset(void);

#endif		// BSF32U4_01_H