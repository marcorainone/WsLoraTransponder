#ifndef BSF32U4_02_H   
#define BSF32U4_02_H

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

typedef enum {
	stm_RDCMD,							// read command 'W' or 'R'
	stm_RD_ADDRESS_CFG,					// read start address cfg
	// write data
	// =========================== bsf32u4 lorawan parameters
	stm_NWKSKEY,						// LoRaWAN Network Session Keys
	stm_APPSKEY,						// LoRaWAN App Session Keys
	stm_DEVADDR,						// LoRaWAN end-device 4 byte hex address (DevAddr)
	stm_TX_INTERVAL,					// LoRaWAN tx data interval (sec)
	// =========================== weather station parameters
	stm_IdWs,							// Id weather station
	// =========================== bsf32u4 micro parameters
	stm_PVarFWdog,						// +/- max tolerance percentage of the watchdog frequency
	stm_WChkSum							// get write checksum
} state_msgserial;

// configuration
typedef struct
{
	// =========================== bsf32u4 lorawan parameters
	uint8_t NWKSKEY[16];						// LoRaWAN Network Session Keys
	uint8_t APPSKEY[16];						// LoRaWAN App Session Keys
	uint32_t DEVADDR;							// LoRaWAN end-device 4 byte hex address (DevAddr)
	uint16_t TX_INTERVAL;						// LoRaWAN tx data interval (sec)
	// =========================== weather station parameters
	uint16_t IdWs;								// Id weather station
	// =========================== bsf32u4 micro parameters
	uint8_t PVarFWdog;							// +/- max tolerance percentage of the watchdog frequency
	uint8_t chksum;								// store the cfg checksum
}cfg_data;

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

#endif		// BSF32U4_02_H