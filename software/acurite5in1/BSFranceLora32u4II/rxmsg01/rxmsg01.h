#ifndef RXMSG01_H   
#define RXMSG01_H

#ifndef byte
	typedef unsigned char byte;    
#endif

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

#endif		// RXMSG01_H