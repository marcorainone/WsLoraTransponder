// **********************************************************************************
// Project:    serial configurator for Acurite transponder with BsFrance Lora32u4II 
//             v1.0 2019-06-01, ICTP Wireless Lab
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
// ---------------------------------------------------
// the program was compiled and tested using:
//        gcc (Ubuntu 7.4.0-1ubuntu1~18.04.1) 7.4.0
// To compile use these commands:
//            make clean
//            make
// ---------------------------------------------------
// 

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>   					//  File Control Definitions           
#include <termios.h> 					//  POSIX Terminal Control Definitions 
#include <unistd.h>  					//  UNIX Standard Definitions 	    
#include <errno.h>   					//  ERROR Number Definitions           

// iniparser library v. 3.0
// http://ndevilla.free.fr/iniparser/
// stand-alone ini parser library in ANSI C
#include "iniparser.h"

// =======================================================================
// DATA
// =======================================================================

#define		PROGRAM_VERSION		1.0

// ---------------------------------------------------------
// ini file configuration data
const char default_ini_filename[256] = "bsf32u4.ini";
char ini_filename[256];
dictionary	*iniDict ;

// ---------------------------------------------------------
// serial port data
char serial_port_name[256] = "/dev/";
int fdSerial;									// serial File Descriptor
int baudRate;									// baudrate serial porty
long uSecByte;									// sleep time to tx 1 byte

struct termios SerialPortSettings;				// serial data structure                          

char SerialTxBuf[1024];							//  serial tx Buffer
char SerialRxBuf[1024];							//  serial rx Buffer
int  bytes_written  = 0;  						//  n. of bytes tx to serial port  
int  bytes_read  = 0;  							//  n. of bytes rx from serial port  

// =======================================================================
// UTILITIES
// =======================================================================

// array used to convert binary data to ascii hex and calc checksum
// 
const char HexTblChr[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

// -----------------------------------------------------------------------
// convert sHex to data

unsigned char *ULong2sHex(char *hex, unsigned long data)
{
	int i;
	unsigned char val;
	
    for(i=7;i>=0;i--)
	{
		val = (unsigned char)data & 0x0F;
		data = data >> 4;
		if(val<=9)	val = '0' + val;
		else		val = 'a' + val - 10;
		*(hex+i) = val;
    }
	hex = hex + 8;
    return (hex);
}
unsigned char *UInt2sHex(char *hex, unsigned int data)
{
	int i;
	unsigned char val;
	
    for(i=3;i>=0;i--)
	{
		val = (unsigned char)data & 0x0F;
		data = data >> 4;
		if(val<=9)	val = '0' + val;
		else		val = 'a' + val - 10;
		*(hex+i) = val;
    }
	hex = hex + 4;
    return (hex);
}
unsigned char *Byte2sHex(char *hex, unsigned char data)
{
	int i;
	unsigned char val;
	
    for(i=1;i>=0;i--)
	{
		val = data & 0x0F;
		data = data >> 4;
		if(val<=9)	val = '0' + val;
		else		val = 'a' + val - 10;
		*(hex+i) = val;
    }
	hex = hex + 2;
    return (hex);
}

// -----------------------------------------------------------------------
// convert data in sHex

char *Array2sHex(char *startPtr, void *data, int length)
{
	int i;
	char *ptr;
	unsigned char val;

	ptr = startPtr;
	for (i = 0; i < length; i++) 
	{   
		val = *(unsigned char *)data;
		
		*ptr =  HexTblChr[val >> 4];
		ptr++;
		*ptr =  HexTblChr[val & 0x0F];
		ptr++;
		data++;
	}
	// *ptr = 0;
	return(ptr);
}

// convert an hex string to unsigned long
unsigned long sHex2ULong(char *str)
{
	int i;
	unsigned long ris = 0L;
	
	unsigned char val;
	for(i=0;i<8;i++)
	{
		val = *str;
		str++;
		
		if((val>='0') && (val<='9'))
			val = val - '0';
		else if((val>='A') && (val<='F'))
			val = val - 'A' + 10;
		else if((val>='a') && (val<='f'))
			val = val - 'a' + 10;
		else
		{
			// error
			return 0L;
		}
		ris = ris << 4;
		ris = ris | (unsigned long)val;
	}
	return(ris);
}
// convert an hex string to unsigned int
unsigned int sHex2UInt(char *str)
{
	int i;
	unsigned int ris = 0L;
	unsigned char val;

	for(i=0;i<4;i++)
	{
		val = *str;
		str++;
		
		if((val>='0') && (val<='9'))
			val = val - '0';
		else if((val>='A') && (val<='F'))
			val = val - 'A' + 10;
		else if((val>='a') && (val<='f'))
			val = val - 'a' + 10;
		else
		{
			// error
			return 0;
		}
		ris = ris << 4;
		ris = ris | (unsigned int)val;
	}
	return(ris);
}
// convert an hex string to unsigned char
unsigned char sHex2Byte(char *str)
{
	int i;
	unsigned char ris = 0L;
	unsigned char val;

	for(i=0;i<2;i++)
	{
		val = *str;
		str++;
		
		if((val>='0') && (val<='9'))
			val = val - '0';
		else if((val>='A') && (val<='F'))
			val = val - 'A' + 10;
		else if((val>='a') && (val<='f'))
			val = val - 'a' + 10;
		else
		{
			// error
			return 0;
		}
		ris = ris << 4;
		ris = ris | val;
	}
	return(ris);
}

// -----------------------------------------------------------------------
// crc utilities

const unsigned long crc_table[16] = {
	0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
	0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
	0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
	0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

unsigned long calc_crc( void *data, int length )
{
	unsigned char value;
	
	unsigned long crc = ~0L;

	for( unsigned int index = 0 ; index < length  ; ++index )
	{
		value = *(unsigned char *)data;
		data++;
		// calc crc
		crc = crc_table[( crc ^ value ) & 0x0f] ^ (crc >> 4);
		crc = crc_table[( crc ^ ( value >> 4 )) & 0x0f] ^ (crc >> 4);
		crc = ~crc;
	}
	return crc;
}

// calc checksum
unsigned char calc_chksum( void *data, int length )
{
	unsigned char crc = 0x00;

	for( unsigned int index = 0 ; index < length  ; ++index )
	{
		crc += *(unsigned char *)data;
		data++;
	}
	crc = crc ^ 0xFF;				// complement bits
	return crc;
}

// =======================================================================
// INI FILE MANAGEMENT
// =======================================================================

void CreateIniFile(char *iniFileName)
{
	FILE	*ini_file ;

	ini_file = fopen(iniFileName, "w");
	fprintf(ini_file,
	"#\n"
	"# This is the base ini file for bsf32u4 lora controller\n"
	"#\n"
	"[Lorawan]\n"
	"# =========================== bsf32u4 lorawan parameters\n"
	"\n"
	"# LoRaWAN Network Session Keys\n"
	"NWKSKEY      = \"A5EC135CBA37A2C55266E77B0B948982\" ;\n"
	"# LoRaWAN Network Session Keys\n"
	"APPSKEY      = \"7D5821DB815148C2A3139BE3BB0678FA\" ;\n"
	"# LoRaWAN end-device 4 byte hex address (DevAddr)\n"
	"DEVADDR      = \"26011824\" ;\n"
	"# LoRaWAN tx data interval (sec)\n"
	"TX_INTERVAL  = 300 ;\n"
	"\n"
	"[Ws]\n"
	"# =========================== weather station parameters\n"
	"\n"
	"# Id weather station\n"
	"IdWs	     = \"0C24\" ;\n"
	"\n"
	"[Micro]\n"
	"# =========================== bsf32u4 micro parameters\n"
	"\n"
	"# +/- max tolerance percentage of the watchdog frequency\n"
	"PVarFWdog   = 15 ;\n"
	"\n");
	fclose(ini_file);
}

int ParseIniFile(char *iniFileName)
{
	// Some temporary variables to hold query results
	int			b ;
	int			i ;
	double		d ;
	char		*s ;

	iniDict = iniparser_load(iniFileName);
	if (iniDict==NULL) 
	{
		fprintf(stderr, "cannot parse file: %s\n", iniFileName);
		return -1 ;
	}
	iniparser_dump(iniDict, stderr);

	// Get Lorawan attributes
	printf("Lorawan:\n");

	s = iniparser_getstring(iniDict, "Lorawan:NWKSKEY", NULL);
	printf("NWKSKEY:       [%s]\n", s ? s : "UNDEF");
	s = iniparser_getstring(iniDict, "Lorawan:APPSKEY", NULL);
	printf("APPSKEY:       [%s]\n", s ? s : "UNDEF");
	s = iniparser_getstring(iniDict, "Lorawan:DEVADDR", NULL);
	printf("DEVADDR:       [%s]\n", s ? s : "UNDEF");
	i = iniparser_getint(iniDict, "Lorawan:TX_INTERVAL", -1);
	printf("TX_INTERVAL:   [%d]\n", i);

	// Get Weather station attributes
	printf("Ws:\n");
	s = iniparser_getstring(iniDict, "Ws:IdWs", NULL);
	printf("IdWs:          [%s]\n", s ? s : "UNDEF");

	// Get Micro attributes
	printf("Micro:\n");
	i = iniparser_getint(iniDict, "Micro:PVarFWdog", -1);
	printf("PVarFWdog:     [%d]\n", i);

	iniparser_freedict(iniDict);
	return 0 ;
}

dictionary	*openIniDictionary(char *iniFileName)
{
	iniDict = iniparser_load(iniFileName);
	if (iniDict==NULL) 
	{
		fprintf(stderr, "cannot parse file: %s\n", iniFileName);
	}
	return iniDict ;
}

void freeIniDictionary(dictionary	*dict)
{
	iniparser_freedict(dict);
}

// =======================================================================
// SERIAL PORT PROGRAMMING
// =======================================================================
// see:
// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

int OpenSerialPort(void)
{
	fdSerial = open(serial_port_name,O_RDWR | O_NOCTTY | O_NDELAY);		// linux serial port (ex: ttyUSB0)
	//  O_RDWR Read/Write access to serial port           
	//  O_NOCTTY - No terminal will control the process   
	//  O_NDELAY -Non Blocking Mode,Does not care about-  
	//  -the status of DCD line,Open() returns immediatly                                         
	if(fdSerial == -1)
	{
		printf("\n  Error! in Opening %s  ", serial_port_name);
		return(fdSerial);
	}
	else
	{
		printf("\n  %s Opened Successfully ", serial_port_name);
	}
	return(fdSerial);
}

void CloseSerialPort(void)
{
	close(fdSerial);			//  Close the Serial port 
}

// ---------- Set the Attributes of the serial port using termios structure --------- 

// n. usec per byte tx (N,8,1, 10 bit per byte tx)
long uSecByteTx(long baud)
{
	return(10000000L / baud);
}

int setSerialPortSettings(long baud)
{
	int result;
	
	baudRate = B9600;
	switch(baud)
	{
		case 19200L  :  baudRate = B19200  ; uSecByte = uSecByteTx( 19200L  ) ; break;
		case 38400L  :  baudRate = B38400  ; uSecByte = uSecByteTx( 38400L  ) ; break;
		case 57600L  :  baudRate = B57600  ; uSecByte = uSecByteTx( 57600L  ) ; break;
		case 115200L :  baudRate = B115200 ; uSecByte = uSecByteTx( 115200L ) ; break;
		case 230400L :  baudRate = B230400 ; uSecByte = uSecByteTx( 230400L ) ; break;
		case 460800L :  baudRate = B460800 ; uSecByte = uSecByteTx( 460800L ) ; break;
		case 500000L :  baudRate = B500000 ; uSecByte = uSecByteTx( 500000L ) ; break;
		case 576000L :  baudRate = B576000 ; uSecByte = uSecByteTx( 576000L ) ; break;
		case 921600L :  baudRate = B921600 ; uSecByte = uSecByteTx( 921600L ) ; break;
		default:
		case 9600L   :  baudRate = B9600   ; uSecByte = uSecByteTx( 9600L   ) ; break;
	}
	
	tcgetattr(fdSerial, &SerialPortSettings);				//  Get the current attributes of the Serial port 
	
	cfsetispeed(&SerialPortSettings,baudRate); 				//  Set Read Speed as baud                     
	cfsetospeed(&SerialPortSettings,baudRate); 				//  Set Write Speed as baud                     
		
	SerialPortSettings.c_cflag &= ~PARENB;   				//  Disables the Parity Enable bit(PARENB),So No Parity   
	SerialPortSettings.c_cflag &= ~CSTOPB;   				//  CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit 
	SerialPortSettings.c_cflag &= ~CSIZE;	 				//  Clears the mask for setting the data size             
	SerialPortSettings.c_cflag |=  CS8;      				//  Set the data bits = 8                                 
		
	SerialPortSettings.c_cflag &= ~CRTSCTS;       			//  No Hardware flow Control                         
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; 			//  Enable receiver,Ignore Modem Control lines        
	
	
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          //  Disable XON/XOFF flow control both i/p and o/p 
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  //  Non Cannonical mode                            

	SerialPortSettings.c_oflag &= ~OPOST;			// No Output Processing

	result = tcsetattr(fdSerial,TCSANOW,&SerialPortSettings);
	if(result != 0) //  Set the attributes to the termios structure
	{
		printf("\n  ERROR ! in Setting attributes");
	}
	else
	{
		printf("\n  BaudRate = %li \n  StopBits = 1 \n  Parity   = none", baud);
	}
	return(result);
}

// "Blocking" sets whether a read() on the port waits for the specified number of characters to arrive. 
// Setting no blocking means that a read() returns however many characters are available without waiting for more, up to the buffer limit.
void set_blocking(int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fdSerial, &tty) != 0)
	{
		printf("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fdSerial, TCSANOW, &tty) != 0)
	{
		printf("error %d setting term attributes", errno);
	}
}

// ----------------------------------------------------------------
// PROTOCOL MESSAGES
//

// form configuration message to send
int FormCfgSerial(int addr, dictionary *dict)
{
	unsigned char sv;
	int 		len, n;
	// unsigned long chkSum = 0x00;
	unsigned char chkSum = 0x00;

	// temporary variables to hold query results
	int			b ;
	int			i ;
	double		d ;
	char		*s ;

	SerialTxBuf[0] = 'W';
	len = 1;
	// start addr to write data
	n = sprintf(&SerialTxBuf[len], "%04X", addr); 
	len = len + n;
	// NWKSKEY
	s = iniparser_getstring(dict, "Lorawan:NWKSKEY", NULL);
	n = sprintf(&SerialTxBuf[len], "%s", s ? s : "00000000000000000000000000000000"); 
	len = len + n;
	// APPSKEY
	s = iniparser_getstring(dict, "Lorawan:APPSKEY", NULL);
	n = sprintf(&SerialTxBuf[len], "%s", s ? s : "00000000000000000000000000000000"); 
	len = len + n;
	// DEVADDR
	s = iniparser_getstring(dict, "Lorawan:DEVADDR", NULL);
	n = sprintf(&SerialTxBuf[len], "%s", s ? s : "00000000"); 
	len = len + n;
	// TX_INTERVAL
	i = iniparser_getint(dict, "Lorawan:TX_INTERVAL", 30);
	n = sprintf(&SerialTxBuf[len], "%04X", i); 
	len = len + n;
	// Get weather station attributes
	s = iniparser_getstring(dict, "Ws:IdWs", NULL);
	n = sprintf(&SerialTxBuf[len], "%s", s ? s : "0000"); 
	len = len + n;
	// Get Micro attributes
	i = iniparser_getint(dict, "Micro:PVarFWdog", 0);
	n = sprintf(&SerialTxBuf[len], "%02X", i); 
	len = len + n;
	
	// calc checksum
	chkSum = calc_chksum( &SerialTxBuf[0], len );
	n = sprintf(&SerialTxBuf[len], "%02X", chkSum); 
	len = len + n;
	// terminator
	SerialTxBuf[len] = '\n';
	len++;
	
	printf("Message: [%s] len %i (checksum %02X)\n", SerialTxBuf, len, chkSum);
	
	return(len);
}

// form read message
int FormRdMessage(int addr, int nbytes)
{
	int 		len, n;
	// unsigned long chkSum = 0x00;
	unsigned char chkSum = 0x00;

	// temporary variables to hold query results
	int			b ;
	int			i ;
	double		d ;
	char		*s ;

	SerialTxBuf[0] = 'R';
	len = 1;
	// addr
	n = sprintf(&SerialTxBuf[len], "%04X", addr); 
	len = len + n;
	// n bytes
	n = sprintf(&SerialTxBuf[len], "%04X", nbytes); 
	len = len + n;
	// calc checksum
	chkSum = calc_chksum( &SerialTxBuf[0], len );
	n = sprintf(&SerialTxBuf[len], "%02X", chkSum); 
	len = len + n;
	// terminator
	SerialTxBuf[len] = '\n';
	len++;

	printf("Message: [%s] len %i (checksum %02X)\n", SerialTxBuf, len, chkSum);
	
	return(len);
}


// =======================================================================
// MAIN
// =======================================================================

int main(int argc, char * argv[])
{
	int tWait;				// waiting time in usec
	int	status ;
	int lenMsg;

	if( argc < 2 ) 
	{
		// no parameters. Print help
		printf(
			"====================================================================================================\n"
			"Linux configuration tool for Acurite transponder with BsFrance Lora32u4II.\n"
			"Version:%.2f\n"
			"Use:\n"
			"%s <serial port name> <configuration filename>\n" 
			"       OR\n"
			"%s <serial port name>\n"
			"       in this case a default configuration file bsf32u4.ini is automatically generated.\n"
			"Example:\n"
			"%s ttyUSB0 ./bsf32u4.ini\n" 
			"where:\n"
			"ttyUSB0:     USB serial port where the transponder Lora32u4II is connected\n"
			"bsf32u4.ini: configuration file name\n"
			"\n"
			"Run this tool in Windows 10 (using WSL, Windows Subsystem for Linux compatibility layer):\n"
			"If the program run in a bash shell of windows10, use the WSL name\n"
			"of the serial to which the Lora32u4II module is connected.\n"
			"In WSL, the serial port COM<n> port is available at /dev/ttyS<n>\n"
			"For example, if the board is connected to the windows serial port COM3,\n"
			"use ttyS3 as serial port name\n"
			"In bash shell launch the program with these parameters:\n"
			"%s ttyS3 ./bsf32u4.ini\n" 
			"For info, see:\n"
			"https://blogs.msdn.microsoft.com/wsl/2017/04/14/serial-support-on-the-windows-subsystem-for-linux/\n"
			"====================================================================================================\n"
			"#\n",
			PROGRAM_VERSION,
			argv[0],
			argv[0],
			argv[0],
			argv[0]
		);
		return(0);
	}
	if( argc >= 2 ) 
	{
		printf("Serial port name %s\n", argv[1]);
		strcat(serial_port_name, argv[1]);
	}
	if (argc<3) 
	{
		strcpy(ini_filename, (char *)default_ini_filename);
		CreateIniFile(ini_filename);
	} 
	else 
	{
		strcpy(ini_filename, argv[2]);
	}
	status = ParseIniFile(ini_filename);
	
	// form the config msg
	iniDict = openIniDictionary(ini_filename);
	if(iniDict == NULL)
	{
		return(0);
	}
	lenMsg = FormCfgSerial(0, iniDict);
	
	// open serial port
	if( OpenSerialPort() < 0 )
	{
		return(0);
	}
	if( setSerialPortSettings(9600L) != 0)
	{
		return(0);
	}

	set_blocking(0);						// set serial read no blocking
	
	// https://www.linuxquestions.org/questions/programming-9/serial-port-write-wait-for-transmission-complete-with-the-function-tcdrain-4175502564/
	
	tcflush(fdSerial, TCOFLUSH);
	// send message
	bytes_written = write(fdSerial, SerialTxBuf, lenMsg);				//  use write() to send data to 
	tcdrain(fdSerial);
	//
	// wait and read answer
	// note: after sent the configuration, the program must wait:
	// 1) time to write the configuration bytes in eeprom
	// 2) time to receive the eeprom data written
	// typ EEPROM Programming Time for atmega32u4 is 3.3usec.
	// See table 5.3 pag. 23 datasheet
	// https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/ATMega32U4.pdf
	//
	// estimate the waiting time:
	tWait = (int)(3300 * (bytes_written));
	tWait += (2* bytes_written) * uSecByte;
	usleep(tWait);								// sleep time: sleep enough to transmit & receive bytes_written
	memset(SerialRxBuf,0x00, sizeof(SerialRxBuf) );						// clear serial rx buffer
	bytes_read = read(fdSerial, SerialRxBuf, sizeof(SerialRxBuf));  	// read up to sizeof(SerialRxBuf) if ready to read
	printf("\nMessageRX: [%s] len %i\n", SerialRxBuf, bytes_read);
	//
	// clear data and exit
	freeIniDictionary(iniDict);
	CloseSerialPort();
	return status ;
}
