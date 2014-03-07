/********************************************************************/
/********************************************************************/
/*																	*/
/*					Program Flash Memory Module						*/
/*																	*/
/********************************************************************/
/********************************************************************/

// xmodem function based on a free Unix program downloaded from the
// Internet, which explains the unusual programming style

// code in this module located in default code segment ".text", along
// with the Flash_API code from TI, and is not intended to be field programmable

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
2008-04-21	enable SCI FIFO
2008-08-22	increase baud rate from 9600 to 115200
			increase hex buffer from 256 to 1024 words
2008-05-06	move sector_table and messages to econst segment
*/

#include "2812reg.h"
#include "literals.h"
#include "io_def.h"
#include "struct.h"
//#include "hardware.h"
#include "Flash2812_API_V210\include\Flash281x_API_Library.h"

#define SOH  0x01	// start character for XMODEM
#define STX  0x02	// start character for XMODEM-1K
#define EOT  0x04	// end of file character for XMODEM
#define ACK  0x06
#define NAK  0x15
#define CAN  0x18

#define DLY_1S 1000
#define MAXRETRANS 25
#define FLASH_BLOCK_SIZE	1024
#define CHAR_BUFFER_SIZE	1030	/* 1024 + 3 header chars + 2 crc + nul */
#define FPGA_RUN_LED		0x0008	// run LED
#define FPGA_FAULT_LED		0x0010	// fault LED
/********************************************************************/
/*		Global variables											*/
/********************************************************************/

FLASH_ST EraseStatus,ProgStatus,VerifyStatus;


#pragma DATA_SECTION(char_buffer,".xm_data")
#pragma DATA_SECTION(hex_buffer,".xm_data")
#pragma DATA_SECTION(flash_pntr,".xm_data")
#pragma DATA_SECTION(flash_length,".xm_data")
#pragma DATA_SECTION(sector_mask,".xm_data")
#pragma DATA_SECTION(sector_erased,".xm_data")
#pragma DATA_SECTION(status,".xm_data")
#pragma DATA_SECTION(run_led_output,".xm_data")
unsigned char char_buffer[CHAR_BUFFER_SIZE];
unsigned hex_buffer[FLASH_BLOCK_SIZE];
unsigned far *flash_pntr;
unsigned long flash_length;
unsigned sector_mask,sector_erased;
int	status,run_led_output;

/********************************************************************/
/*		Constants													*/
/********************************************************************/

#pragma DATA_SECTION(sector_table,".econst")
#pragma DATA_SECTION(messages,".econst")
const unsigned long sector_table[10]={0x3D8000,0x3DA000,0x3DC000,0x3E0000,0x3E4000,0x3E8000,0x3EC000,0x3F0000,0x3F4000,0x3F6000};
const char messages[4][17]={ ' ','F','i','l','e',' ','r','e','c','e','i','v','e','d',' ', 0 ,0,
						     ' ','C','a','n','c','e','l','l','e','d',' ', 0 , 0 , 0 , 0 , 0 ,0,
						  	 ' ','S','y','n','c',' ','e','r','r','o','r',' ', 0 , 0 , 0 , 0 ,0,
						  	 ' ','T','o','o',' ','m','a','n','y',' ','r','e','t','r','y',' ',0};

void flash_run_led (void)
{
	FPGA_OUTPUT=(run_led_output^=FPGA_RUN_LED);
}

void program_flash (void)
{
	register int temp;
	register const char *message_pntr;

/********************************************************************/
/*	Initialize SCI-A Registers (RS-232 PC interface)				*/
/********************************************************************/
	PF2.SCICCR		=0x07;	// one stop bit,no parity,idle mode,8 data bits
	PF2.SCICTL1		=0x03;	// disable ERR INT,SW RESET,enable TX,enable RX
	PF2.SCICTL2		=0x00;	// disable RX interrupt, disable TX interrupt
	PF2.SCIPRI		=0x40;	// SCI FREE=1
	temp			=(int)((LSPCLK/8.0)/115200.0 - 1.);
	PF2.SCILBAUD	=temp;
	PF2.SCIHBAUD	=temp>>8;
	PF2.SCIFFTX		=0x0000;	// reset transmit FIFO
	PF2.SCIFFRX		=0x0000;	// reset receive FIFO
	PF2.SCIFFTX		=0xE040;
	PF2.SCIFFRX		=0x201F;
	PF2.SCIFFCT		=0x0000;
	PF2.SCICTL1		=0x23;	// Renable SCI by setting SW Reset, TX & RX enabled

/********************************************************************/
/*	Initialize variables											*/
/********************************************************************/
	Flash_CPUScaleFactor=SCALE_FACTOR;
	Flash_CallbackPtr=&flash_run_led;
	run_led_output=temp=0;
	flash_pntr=(unsigned far*)START_OF_FLASH;

	while (1)
	{
		temp=xmodemReceive();
		if (temp!=0)
		{
			message_pntr=&messages[0][0];
			if (temp < 0)
				message_pntr=&messages[abs(temp)][0];
			while ((temp=*message_pntr++)!=0)		// send message
			{
				if ((PF2.SCIFFTX & 0x1000)==0);		// transmit FIFO not full
		       		PF2.SCITXBUF=temp;				// transmit next character
			}
		}
	}		// bottom of while loop
}	// end program_flash

/********************************************************************/
/*	Serial input function											*/
/********************************************************************/
static int inbyte(int msec_delay)
{
	unsigned counter;

    while((PF2.SCIFFRX & 0x1F00)==0)		// receive FIFO empty
    {										// no new data
		counter=(int)(SYSCLKOUT/9.0e3);		// 1 ms delay
		while (counter!=0)
		{
			counter--;
		}
        if (msec_delay-- <= 0)
		{
        	return -1;
		}
    }
    return (PF2.SCIRXBUF & 0xFF);
}

static unsigned crc16_ccitt(unsigned char *buf, int size)
{
    register int i;
    register unsigned crc = 0;

    while (--size >= 0)
    {
        crc ^= (unsigned) *buf++ << 8;
        for (i = 0; i < 8; i++)
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc <<= 1;
    }
    return crc;
}

static int check_crc(int crc, unsigned char *buf, int size)
{
    if (crc!=0)
    {											// 16-bit crc
        crc = crc16_ccitt(buf, size);
        if (crc == ((buf[size]<<8)+buf[size+1]))
        	return 1;							// passed
    }
    else
    {											// 8-bit checksum
        register int i=0;
        register unsigned char chksum = 0;
        while (i < size)
        	chksum += buf[i++];
        if (chksum == buf[size])
       		return 1;							// passed
    }

    return 0;									// failed
}

static void flushinput(void)
{
    while (inbyte((DLY_1S*3)>>1) >= 0)
        	;
}

/********************************************************************/
/*	Xmodem Receive function											*/
/********************************************************************/
int xmodemReceive(void)
{
    register int temp,i,hex_word;
	register long long_temp;
    unsigned char start_char,packet_number;
    unsigned char *char_buffer_pntr;
	unsigned hex_buffer_index;
    int crc,indata,packet_size;
    int retry, retrans;

/********************************************************************/
/*	Initialize variables											*/
/********************************************************************/
    char_buffer_pntr=&char_buffer[0];
    while (char_buffer_pntr < &char_buffer[CHAR_BUFFER_SIZE])
		*char_buffer_pntr++=0;
	crc=0;						// 8-bit checksum
	start_char = 'C';			// start character for XMODEM-CRC
	packet_number = 1;
	retrans = MAXRETRANS;
	sector_erased=sector_mask=0;
	flash_length=hex_buffer_index=0;

/********************************************************************/
/*	Request first data packet										*/
/********************************************************************/
    while(1)
    {
        for( retry = 0; retry < 32767; ++retry)
        {
			PF2.GPATOGGLE=TP48_OUTPUT;
			FPGA_OUTPUT=(run_led_output^=FPGA_RUN_LED);
            if (start_char)					// not null character
            	PF2.SCITXBUF=start_char;	// request data
            if ((indata = inbyte(DLY_1S*2)) >= 0)
            {							// character received
                switch (indata)
                {
                case SOH:				// original XMODEM
                	packet_size = 128;
                    goto start_recv;
                case STX:				// XMODEM-1K
                    packet_size = 1024;
                    goto start_recv;
                case EOT:				// end of file
                    flushinput();
                    PF2.SCITXBUF=ACK;	// acknowledge end of file
                    return 1;		 /* normal end */
                case CAN:				// cancel file transfer
                    if ((indata = inbyte(DLY_1S)) == CAN)
                    {
                        flushinput();
                        PF2.SCITXBUF=ACK;	// acknowledge cancel
                        return -1; 		/* canceled by remote */
                    }
                    break;
                default:
                    break;
        	}
    	}	// end for loop
    }		// end while loop

    if (start_char == 'C')
    {							// no response for 'C'
    	start_char = NAK;		// change start character to NAK
    	continue;
    }
    flushinput();
	while ((PF2.SCIFFTX & 0x1F00)!=0);	// wait for FIFO to empty
    PF2.SCITXBUF=CAN;					// cancel transfer
	while ((PF2.SCIFFTX & 0x1F00)!=0);	// wait for FIFO to empty
    PF2.SCITXBUF=CAN;
	while ((PF2.SCIFFTX & 0x1F00)!=0);	// wait for FIFO to empty
    PF2.SCITXBUF=CAN;
    return -2; /* sync error */

/********************************************************************/
/*	Receive next data packet										*/
/********************************************************************/
    start_recv:
        if (start_char == 'C')		// 'C' indicates XMODEM-1K
        	crc = 1;				// select 16-bit crc
        start_char=0;
        char_buffer[0]= indata;
        for (i = 1;  i < (packet_size+crc+4); i++)
        {
            if ((indata = inbyte(DLY_1S*7)) < 0)
            	goto reject;
            char_buffer[i]= indata;
        }

/********************************************************************/
/*	Process next data packet (hex buffer holds up to 4 packets)		*/
/********************************************************************/
        if (((255-char_buffer[1]) == char_buffer[2]) &&	// block number & ~block number
        	(char_buffer[1] == packet_number || char_buffer[1] == (unsigned char)packet_number-1) &&
            check_crc(crc,(unsigned char*)&char_buffer[3], packet_size))
            {
            	if (char_buffer[1] == packet_number)
            	{									// correct packet number
					flash_length+=(packet_size>>2);	// 4 ASCII characters per word
					char_buffer_pntr=&char_buffer[3];	// first character

/********************************************************************/
/*	Determine if packet begins with address field					*/
/********************************************************************/
					if (((*char_buffer_pntr)=='$') && ((*(char_buffer_pntr+1))=='A'))
					{					// '$A' indicates start of address field
						char_buffer_pntr+=2;	// skip over first "$A'
						long_temp=0;
						for (i=6;i>0;i--)
						{				// convert 6 character address to hex
							temp=*char_buffer_pntr++;
							if ((temp >= '0') && (temp <= '9'))
								temp-='0';
							else if ((temp >= 'A') && (temp <= 'F'))
								temp-=('A'-0x0a);
							else if ((temp >= 'a') && (temp <= 'f'))
								temp-=('a'-0x0a);
							long_temp=(long_temp<<4)|temp;
						}
						flash_pntr=(unsigned far*)long_temp;
						flash_length-=(8/4);	// subtract length of address field
					}

/********************************************************************/
/*	Convert ASCII data in char_buffer to hex 						*/
/********************************************************************/
					while (hex_buffer_index < flash_length)
					{
						for (i=0;i<4;i++)
						{					// 4 characters per word
							temp=*char_buffer_pntr++;
							if ((temp >= '0') && (temp <= '9'))
								temp-='0';
							else if ((temp >= 'A') && (temp <= 'F'))
								temp-=('A'-0x0a);
							else if ((temp >= 'a') && (temp <= 'f'))
								temp-=('a'-0x0a);
							hex_word=(hex_word<<4)|temp;
						}
						hex_buffer[hex_buffer_index++]=hex_word;
					}
							
/********************************************************************/
/*	Determine if flash sector needs to be erased					*/
/********************************************************************/
					if (flash_length > (FLASH_BLOCK_SIZE-(packet_size>>2)))
					{										// hex buffer full
						long_temp=(unsigned long)flash_pntr;
						for (i=0;i<(sizeof(sector_table)-1);i++)
						{							// search table to find sector
							if ((long_temp>=sector_table[i]) && (long_temp<sector_table[i+1]))
								break;
						}
						sector_mask=(SECTORJ>>i)&~sector_erased;
						if (sector_mask==0)			// this sector already erased
						{		// determine if data block runs into next sector
							long_temp+=flash_length;	// end of data packet
							for (i=0;i<(sizeof(sector_table)-1);i++)
							{							// search table to find sector
								if ((long_temp>=sector_table[i]) && (long_temp<sector_table[i+1]))
									break;
							}
							sector_mask=(SECTORJ>>i)&~sector_erased;
						}
						sector_mask&=~SECTORA;			// do not erase sector A
						if (sector_mask!=0)
						{								// sector not erased yet
							sector_erased|=sector_mask;	// mark sector as erased
							status=Flash_Erase(sector_mask,&EraseStatus);
						}

/********************************************************************/
/*	Program next block of flash	(up to FLASH_BLOCK_SIZE words)		*/
/********************************************************************/
						if (flash_pntr < (unsigned far*)END_OF_FLASH)
						{								// program next block
							status=Flash_Program(flash_pntr,(unsigned far*)&hex_buffer,flash_length,&ProgStatus);
							if (status==STATUS_SUCCESS)
								status=Flash_Verify(flash_pntr,(unsigned far*)&hex_buffer,flash_length,&VerifyStatus);
							flash_pntr+=flash_length;
							flash_length=hex_buffer_index=0;
						}
					}	// end hex buffer full

                    packet_number=(packet_number+1)&0xFF;	// increment packet number
                    retrans = MAXRETRANS+1;
                }	// end correct packet number
                if (--retrans <= 0)
                {
                    flushinput();
					while ((PF2.SCIFFTX & 0x1F00)!=0);	// wait for FIFO to empty
                    PF2.SCITXBUF=CAN;					// cancel transfer
					while ((PF2.SCIFFTX & 0x1F00)!=0);	// wait for FIFO to empty
                    PF2.SCITXBUF=CAN;
					while ((PF2.SCIFFTX & 0x1F00)!=0);	// wait for FIFO to empty
                    PF2.SCITXBUF=CAN;
                    return -3; /* too many retry error */
                }
                PF2.SCITXBUF=ACK;			// request next packet
                continue;
            }	// end process next data packet
        reject:
            flushinput();
			if (PF2.SCIRXST & 0x80)  				// check RX ERROR
				PF2.SCIFFTX&=~0x8000;				// clear SCIRST
			else if ((PF2.SCIFFTX & 0x8000)==0)
				PF2.SCIFFTX|=0x8000;;				// set SCIRST

            PF2.SCITXBUF=NAK;			// request resend last packet
    }	// end start_recv
}			// end xmodemReceive
