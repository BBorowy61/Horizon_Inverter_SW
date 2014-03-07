/********************************************************************/
/********************************************************************/
/*																	*/
/*							Main Module								*/
/*																	*/
/********************************************************************/
/********************************************************************/

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
2008-03-00	add nvram test
2008-03-00	add fpga interface test
2008-03-00	move background code to separate module
2008-03-00	add provision for programming flash memory
2008-04-01	add loading of FPGA at startup
2008-04-04	expand ROM checksum to include FPGA code
			add separate checksum for flash programming code
2008-04-07	remove initialization code from main and put in its own
			segment
2008-04-22	enable SCI FIFO's
2008-04-29	include checksums for old versions of .text code
2008-09-16	change McBSP configuration to SPI slave
1 MW version
2008-10-27	correct errors in last change
2008-11-10	add 4th checksum for .text code
2008-11-21	allow McBSP to operate in either SPI master or slave mode
2009-01-08	delete use of J12 to set SCI baud rate
2009-01-14	initialize XTIMING0 instead of XTIMING1 (wrong) for FPGA
2009-01-20	remove carrier phase shift in gating for 2nd bridge
2009-01-26	add external interrupt 1
2009-02-11	delete timer 0 interrupt
2009-03-18	move call to flash programming function to background
2009-04-01	use real SPI instead of McBSP for SPI communications
2009-04-29	delete watchdog reset
2009-05-22	use 2nd set of Modbus parameters for 1 MW
2009-06-10	add constant chksum_adjustment
Solstice version
2009-07-20	delete alternate Modbus settings for 1 MW
			delete PWM synchronization for 1 MW
2009-08-13	add Jon Juniman code for his version of Modbus
2009-08-25	delete SPI interrupts
2009-10-20	delete waiting for FPGA to initialize
			initialize digital outputs after loading 1st block of code
*/

#include "2812reg.h"
#include "literals.h"
#include "struct.h"
#include "io_def.h"
#include "faults.h"
#include "fpga_1303.h"

/********************************************************************/
/*		Global variables											*/
/********************************************************************/
#pragma DATA_SECTION(hw_stack,".stack")
unsigned hw_stack[STACK_SIZE];
struct CHKSUMS checksums;
int save_command;
int fpga_version,software_version;

/********************************************************************/
/*		2812 I/O structures (defined in 2812reg.h)					*/
/********************************************************************/
#pragma DATA_SECTION(PF0,".p_frame0")
struct P_FRAME0 PF0;
#pragma DATA_SECTION(PF2,".p_frame2")
struct P_FRAME2 PF2;

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern struct PARAMS parameters;
extern struct SAVED_VARS saved_variables;
extern struct FAULT_QUEUE fault_queue;
extern struct NV_RAM saved_data,saved_data_2;
extern int status_flags;
extern int pwm_period;
extern int junk;
extern int faults[FAULT_WORDS];
extern int debugIndex,debug[],debug_control,dv[];

extern int fdbk_pu[FDBK_SIZE];

/********************************************************************/
/*		External functions											*/
/********************************************************************/
extern void pwm_isr (void);
extern void parameter_calc (void);
extern void program_flash_232 (void);
extern void program_flash_485 (void);
extern void read_flash_status (void);
extern void background (void);
//extern void mppt_ini (void);
extern void initialize_modbus(void);
extern void modbusPacketTimerPreIsr(void);
extern void uartRxCharPreIsr(void);
extern void uartTxCharPreIsr(void);

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
struct ROMCHKSUM {unsigned code; unsigned text; unsigned text2; unsigned text3;};
#pragma DATA_SECTION(rom_chksum,".chksum")
//const far struct ROMCHKSUM rom_chksum={65535,65535,65535,65535};
const far struct ROMCHKSUM rom_chksum={45677,65535,65535,37032};
#pragma DATA_SECTION(chksum_adjustment,".rom_const")
const far unsigned chksum_adjustment=0;

/********************************************************************/
/*		Interrupt vectors											*/
/********************************************************************/
#pragma DATA_SECTION(int_vector_table,".vectors")
void far *int_vector_table[128];

/********************************************************************/
/********************************************************************/
/*		Calculate checksum function									*/
/********************************************************************/
#pragma CODE_SECTION(simple_checksum,".rom_code")
unsigned simple_checksum(register unsigned far *pntr,register unsigned long words)  
{
	register unsigned checksum=0;
	while (words > 0)
	{
		checksum+=*pntr++;
		words--;
	}
	return checksum;
}
				
/********************************************************************/
/********************************************************************/
/*		Configure Flash function (must be executed from RAM)		*/
/********************************************************************/
// defaults to .text segment
void configure_flash(int wait_states)
{ 
	PF0.FBANKWAIT	=wait_states|(wait_states<<8);
	PF0.FOPT		=0x0001;			// enable flash pipeline
	PF0.FSTDBYWAIT	=511;				// set to maximum
	PF0.FACTIVEWAIT	=511;				// set to maximum
	asm(" RPT #7 || NOP");				// flush instruction queue
}

/********************************************************************/
/********************************************************************/
/*		Dummy interrupt function									*/
/********************************************************************/
#pragma CODE_SECTION(dummy_isr,".rom_code")

interrupt void dummy_isr (void)
{
	PF0.PIEACK|=0x1;				// acknowledge interrupt
}

/********************************************************************/
/********************************************************************/
/*		Initialize function											*/
/********************************************************************/
// located in its own code segment, and should not be moved because
// the call to this function is in "main" in non-programmable sector A
#pragma CODE_SECTION(initialize,".init_code")

void initialize (void)
{
	register int index,temp;
	register unsigned temp_unsign,spi_data;
	register int far *int_pointer;
	register long far *long_dest_pntr,*long_src_pntr;
	register char far *char_pointer;

	PF2.GPACLEAR=	DIO_18_GPA|DIO_19_GPA; //rs added (moved from main_code)
	PF2.GPASET =	DIO_17_GPA;  // GJK, shut off AC contactor 2 //rs moved from main to initialize()
/********************************************************************/
/*	Initialize ADC registers										*/
/********************************************************************/
	PF2.ADCTRL3=	0x010E;				// external reference
	PF2.ADCTRL1=	0x0010;				// cascaded mode
	PF2.ADCTRL3=	0x010E + 0xC0;		// power up bandgap/ref
										// 7 ms delay req'd
/********************************************************************/
/*	Initialize external memory interface							*/
/********************************************************************/

//	XINTF Registers
//	--------------- 
	PF0.XINTCNF2=	0x08L;			// XCLKOUT disabled


// Zone 1 - FPGA
	PF0.XTIMING0=	((3L<<16)		// XSIZE
					+(1<<12)		// XRDLEAD
					+(7<<9)			// XRDACTIVE
					+(1<<7)			// XRDTRAIL
					+(1<<5)			// XWRLEAD
					+(7<<2)			// XWRACTIVE
					+1);			// XWTRAIL
// Zone 2 - NVRAM
	PF0.XTIMING2=	((3L<<16)		// XSIZE
					+(1<<12)		// XRDLEAD
					+(7<<9)			// XRDACTIVE
					+(1<<7)			// XRDTRAIL
					+(1<<5)			// XWRLEAD
					+(5<<2)			// XWRACTIVE
					+1);			// XWTRAIL

/********************************************************************/
/*	Set up SPI-A for loading FPGA									*/
/********************************************************************/
	PF2.SPICCR		=0x07;	// SW Reset, Clock Polarity=0, 8 bit character
	PF2.SPICTL		=0x0E;	// SPICLK delayed, Master, Transmit Enabled, SPI Int disabled
	PF2.SPIBRR		=0x03;	// Baud rate=LSPCLK/4 (9.375 MHz)
	PF2.SPICCR		=0x87;	// Clear SW Reset

/********************************************************************/
/*	Load FPGA (approx 45 ms)										*/
/*	J34 must be removed to select Passive Serial mode				*/
/********************************************************************/
	PF2.GPASET=TP48_OUTPUT;	
	PF2.GPBCLEAR=0x4000;						// clear FPGA_NCONFIG
	while ((PF2.GPBDAT & 0x2000)!=0) {};		// wait for nSTATUS to go low
	PF2.GPBSET=0x4000;		// set FPGA_NCONFIG to enter configuration mode
	while ((PF2.GPBDAT & 0x2000)==0) {};	// wait for nSTATUS to go high
	char_pointer=(char far*)&fpga_table;	// point to start of fpga_table
	// use SPI to send fpga_table to device
	for (temp_unsign=0;temp_unsign<=sizeof(fpga_table)+1;temp_unsign++)
	{
		while (PF2.SPISTS & 0x20) {};		// wait for TX BUF to empty
		spi_data=*char_pointer++;			// get next byte
		spi_data=	((spi_data&0x01)<<15)|	// swap bit order
					((spi_data&0x02)<<13)|	// and move to high byte
					((spi_data&0x04)<<11)|
					((spi_data&0x08)<<9)|
					((spi_data&0x10)<<7)|
					((spi_data&0x20)<<5)|
					((spi_data&0x40)<<3)|
					((spi_data&0x80)<<1);
		PF2.SPITXBUF=spi_data&0xFF00;		// mask low byte & send
	}
	while (((PF2.GPADAT & 0x0200)==0) && (++temp_unsign < 65535U))
	{	// while FPGA_CONF_DONE is low continue to send 'FF's
		while (PF2.SPISTS & 0x20) {};	// wait for TX BUF to empty
		PF2.SPITXBUF=0xFF00;
	}	// FPGA_CONF_DONE going high indicates configuration complete
	PF2.GPACLEAR=TP48_OUTPUT;
	PF2.GPFMUX=	0x3F3F;				// enable SPI_CS output

/********************************************************************/
/*	Copy application code to L1_RAM (4K)							*/
/*	Overwrites .text code segment which is no longer required		*/
/********************************************************************/
	long_src_pntr=(long far*)L1_FLASH;		// load address
	long_dest_pntr=(long far*)L1_RAM;		// run address
	for (index=0; index<L1_RAM_SIZE; index+=2)
		*(long_dest_pntr++)=*(long_src_pntr++);
	EXT_OUTPUT=0;

/********************************************************************/
/*	Copy application code to H0_RAM (8K)							*/
/********************************************************************/
	long_src_pntr=(long far*)H0_FLASH;		// load address
	long_dest_pntr=(long far*)H0_RAM;		// run address
	for (index=0; index<H0_RAM_SIZE; index+=2)
		*(long_dest_pntr++)=*(long_src_pntr++);

/********************************************************************/
/*	Fill stack with a pattern that makes the hi water mark visible	*/
/********************************************************************/
	for(index=32; index<STACK_SIZE; index++)  // leave 32 words for active stack
		hw_stack[index] = 0xDEAD;

/********************************************************************/
/*	Clear M0_RAM & M1_RAM data memory								*/
/********************************************************************/
	long_dest_pntr=(long far*)STACK_SIZE;
	for (index=(0x0800-STACK_SIZE); index>0; index-=2)
		*(long_dest_pntr++)=0;

/********************************************************************/
/*	Clear L0_RAM data memory										*/
/********************************************************************/
	long_dest_pntr=(long far*)L0_RAM;
	for (index=(L1_RAM-L0_RAM); index>0; index-=2)
		*(long_dest_pntr++)=0;

/********************************************************************/
/*	Check flash memory for checksum errors (approx 25 ms)			*/
/********************************************************************/
	PF2.GPASET=TP48_OUTPUT;	
	checksums.text=simple_checksum((unsigned far*)TEXT_FLASH,(END_OF_FLASH-TEXT_FLASH));
	checksums.code=simple_checksum((unsigned far*)START_OF_FLASH,(CHKSUM_FLASH-START_OF_FLASH));
	if (((checksums.code!=rom_chksum.code) && (rom_chksum.code!=0xFFFF))||
	  	((rom_chksum.text!=checksums.text) && (rom_chksum.text!=0xFFFF) &&
	  	(rom_chksum.text2!=checksums.text) && (rom_chksum.text3!=checksums.text)))
		SET_FAULT(FLASH_CHKSUM_FLT)

/********************************************************************/
/*	Test NVRAM (byte wide) (approx 35 ms)							*/
/********************************************************************/
	PF2.GPACLEAR=TP48_OUTPUT;	
	int_pointer=(int far*)NVRAM;
	temp_unsign=0;
	while (int_pointer < (int far*)(NVRAM+0x10000-0x10))
	{
		temp=~(*int_pointer)&0xFF;
		*int_pointer=temp;
		if( (((*int_pointer)&0xFF)!=temp)&&(temp_unsign==0))
			temp_unsign=1;
		*int_pointer++=~temp;
	}
	if (temp_unsign!=0)
	{								// failure detected
		SET_FAULT(NVRAM_FLT)
	}

/********************************************************************/
/*	Copy saved parameters from NVRAM								*/
/********************************************************************/
	PF2.GPASET=TP48_OUTPUT;	
	// recall first copy
	read_nvram ((unsigned far*)&saved_data.parameters,(unsigned far*)&parameters,sizeof(parameters));
	checksums.params=simple_checksum((unsigned far*)&parameters,(long)sizeof(parameters)-1);
	if (checksums.params!=parameters.chksum)
	{	// if checksum error recall second copy
		SET_FAULT(PARAM_CHKSUM_FLT_A1)
		read_nvram ((unsigned far*)&saved_data_2.parameters,(unsigned far*)&parameters,sizeof(parameters));
		checksums.params=simple_checksum((unsigned far*)&parameters,(long)sizeof(parameters)-1);
		if (checksums.params!=parameters.chksum)
			SET_FAULT(PARAM_CHKSUM_FLT_A2)
	}

	parameter_calc();

/********************************************************************/
/*	Copy saved variables from NVRAM									*/
/********************************************************************/
	// recall first copy
	read_nvram ((unsigned far*)&saved_data.saved_variables,(unsigned far*)&saved_variables,SAVED_VARS_SIZE);
	checksums.saved_vars=simple_checksum((unsigned far*)&saved_variables+1,(long)(SAVED_VARS_SIZE-1));
	if (checksums.saved_vars!=saved_variables.chksum)
	{	// if checksum error recall second copy
		SET_FAULT(DATA_CHKSUM_FLT1)
		read_nvram ((unsigned far*)&saved_data_2.saved_variables,(unsigned far*)&saved_variables,SAVED_VARS_SIZE);
		checksums.saved_vars=simple_checksum((unsigned far*)&saved_variables+1,(long)(SAVED_VARS_SIZE-1));
		if (checksums.saved_vars!=saved_variables.chksum)
			SET_FAULT(DATA_CHKSUM_FLT2)
	}

/********************************************************************/
/*	Copy fault queue from NVRAM										*/
/********************************************************************/
	read_nvram ((unsigned far*)&saved_data.fault_queue,(unsigned far*)&fault_queue,sizeof(fault_queue));

/********************************************************************/
/*	Wait for parameter initialization to finish						*/
/********************************************************************/
	while (!(status_flags & STATUS_INITIALIZED)) {}

/********************************************************************/
/*	Initialize PERIPHERAL FRAME 2 (16-bit peripheral bus)			*/
/********************************************************************/
//	SCI-A Registers (RS-232 PC interface)
//	---------------
	PF2.SCICCR		=0x07;	// one stop bit,no parity,idle mode,8 data bits
	PF2.SCICTL1		=0x03;	// disable ERR INT,SW RESET,enable TX,enable RX
	PF2.SCICTL2		=0x00;	// disable RX interrupt, disable TX interrupt
	PF2.SCIPRI		=0x40;	// SCI FREE=1
	temp_unsign		=(unsigned)(LSPCLK/(8.0*100.0))/(unsigned)(192)-1;
	PF2.SCILBAUD	=temp_unsign;
	PF2.SCIHBAUD	=temp_unsign>>8;
	PF2.SCIFFTX		=0x0000;	// reset transmit FIFO
	PF2.SCIFFRX		=0x0000;	// reset receive FIFO
	PF2.SCIFFTX		=0xE040;
	PF2.SCIFFRX		=0x201F;
	PF2.SCIFFCT		=0x0000;
	PF2.SCICTL1		=0x23;	// Renable SCI by setting SW Reset, TX & RX enabled

//	SCI-B Registers (RS-485 Modbus interface)
//	---------------
	temp=((parameters.modbus_data_bits-1)&0x07)|
		 ((parameters.modbus_parity&0x03)<<5)|
		 ((parameters.modbus_stop_bits-1)<<8);
	temp_unsign	=(unsigned)(LSPCLK/(8.0*100.0))/(unsigned)(parameters.modbus_baud_rate)-1;
	PF2.SCICCRB		=temp;
	PF2.SCICTL1B	=0x03;	// disable ERR INT,SW RESET,enable TX,enable RX
	PF2.SCICTL2B	=0x00;	// disable RX interrupt, disable TX interrupt
	PF2.SCIPRIB		=0x40;	// SCI FREE=1
	PF2.SCILBAUDB	=temp_unsign;
	PF2.SCIHBAUDB	=temp_unsign>>8;
	PF2.SCICTL1B	=0x23;	// Renable SCI by setting SW Reset, TX & RX enabled

//	SPI Registers
//	-------------
	PF2.SPIFFTX	=0x0040;	// reset transmit FIFO
	PF2.SPIFFRX	=0x405F;	// reset receive FIFO
//	PF2.SPIFFCT	=2;			// transmit delay
	PF2.SPICCR	=0x07;		// SW Reset, Clock Polarity=0, 8 bit character
	if (parameters.modbus_slave_id==0)
		PF2.SPICTL	=0x07;		// Master, Transmit Enabled, SPI Int enabled
	else
		PF2.SPICTL	=0x03;		// Slave, Transmit Enabled, SPI Int enabled
	PF2.SPIBRR	=75;		// Baud rate=LSPCLK/38=987 kHz=12.3 char per 100 us
	PF2.SPICCR	=0x87;		// Clear SW Reset
	PF2.SPIFFTX	=0xE008;	// enable transmit FIFO
	PF2.SPIFFRX	=0x2008;	// enable receive FIFO

//	ADC Registers
//	-------------
	PF2.ADCTRL3=		0x010E +0xC0 + 0x20;	// power up ADC
	PF2.ADCMAXCONV=		0x000F;					// 16 conversions
	PF2.ADCCHSELSEQ1=	0x3210;
	PF2.ADCCHSELSEQ2=	0x7654;
	PF2.ADCCHSELSEQ3=	0xBA98;
	PF2.ADCCHSELSEQ4=	0xFEDC;

//	EV Registers
//	--------------
// GPT Control register
	PF2.GPTCONA=PF2.GPTCONB=(0x0002		// T1PIN output active high
							+0x0004		// T2PIN output active high
							+0x0070);	// TCMPOE enable compare outputs
// Action Control registers
	PF2.ACTRA=PF2.ACTRB=	0x666;
// Deadband Timer Control registers
	temp=MULQ(12,parameters.dead_time,(int)(4096.0*(HSPCLK/1.0e6)/32.0));
	LIMIT_MAX_MIN(temp,12,2)	// 0.85 to 5.12 us
	PF2.DBTCONA=PF2.DBTCONB=((temp<<8)	// DBT period
							+0x0014		// DBTPS prescale /32 (HSPCLK /32 = 2.344 MHz)
							+0x00E0);	// EDBTx enable all
// Compare Control registers
	PF2.COMCONA=PF2.COMCONB=(0x2000		// CLD reload compare on period match
							+0x0400		// ACTRLD reload action on period match
							+0x0200		// FCMPOE enable full compare output
							+0x00E0);	// FCMPxOE enable full compare outputs 1,2,3
// Timer Count register
	PF2.T1CNT=				0;
	PF2.T3CNT=				0;
// Timer Period register
	PF2.T1PR=PF2.T3PR=		pwm_period;
// Timer Control register
	PF2.T1CON=PF2.T3CON=	(0x0800		// TMODE continuous up/down count mode
							+0x0000		// TPS prescale HSPCLK/1
							+0x0040		// TENABLE enable timer
							+0x0004		// TCLD reload on period match
							+0x0002);	// TECMPR enable compare
	PF2.EXTCONA|=0x0008;				// enable EVASOC for ADC
// EVA Interrupt Mask register
	PF2.EVAIMRA=0x0280;					// enable T1PINT & T1UFINT
	PF2.COMCONA|=0x8000;				// CENABLE enable compare
	PF2.COMCONB|=0x8000;

//Config DACs' System Control Reg and Channel Control Reg
//mapped to lower 10bits of word
//to use as 'signed' must sign extend with>>6	
	DAC1_CNTL=(DAC_MC_POWERUP|DAC_MC_BINARY); //write DAC System Control Registerchannel a is also sys control	 
	DAC1_CNTL=(DAC_CH_VREF_EXT|DAC_NOT_STBY|DAC_CH_SELECT);
	DAC2_CNTL=(DAC_CH_VREF_EXT|DAC_NOT_STBY|DAC_CH_SELECT);
	DAC3_CNTL=(DAC_CH_VREF_EXT|DAC_NOT_STBY|DAC_CH_SELECT);
	DAC4_CNTL=(DAC_CH_VREF_EXT|DAC_NOT_STBY|DAC_CH_SELECT);		
	
	DAC1_DATA = DAC2_DATA = DAC3_DATA = 0;	
	DAC4_DATA = 0x3FF;	// full scale for hardware overcurrent trip

/********************************************************************/
/*	Initialize Peripheral Interrupt Expansion (PIE) Controller		*/
/********************************************************************/
	for (index=0;index<128;index++)		// load interrupt vector table
		int_vector_table[index]=&dummy_isr;	// default for unused ints

	int_vector_table[43]=&pwm_isr;			// T1PINT
	int_vector_table[45]=&pwm_isr;			// T1UFINT
	// Communication functions
	int_vector_table[98]=uartRxCharPreIsr;
	int_vector_table[99]=uartTxCharPreIsr;
	int_vector_table[14]=modbusPacketTimerPreIsr;

	PF0.PIECTRL|=0x0001;		// ENPIE enable fetching from PIE block
	PF0.PIEIER2|=0x0028;		// enable INT2.4 (T1PINT) & 2.6 (T1UFINT)
	PF0.PIEIER9|=0x000C;		// enable INT9.3 (SCI B RX ) & 9.4 (SCI B TX )
	PF0.PIEACK|=0x0103;			// acknowledge INT1 & INT2 & INT9

/********************************************************************/
/*	Setup CPU Timer 2 for JJ version								*/
/********************************************************************/
	PF0.TIMER2.TIMERTPR=(int)(SYSCLKOUT/1.0e6)-1;	// 1 us
	PF0.TIMER2.TIMERPRD=3646;				// = 3.5 chars @ 9600 baud

	/* disable DIN filter on fan tachometer signals */
	DIN_FILTER_CNFG = 0x8100;

junk = 0;

//	mppt_ini();
	WATCHDOG_RESET=0;
	asm("	EDIS");			// disable protected access

/********************************************************************/
/*	Enable interrupts for normal operation							*/
/********************************************************************/
	asm("	AND IFR,#0");	// clear interrupt flags
	asm("	OR IER,#2102h");// enable CPU INT2, 9, 14
	asm("	EINT");			// enable global interrupts




	initialize_modbus();
	PF2.GPACLEAR=TP48_OUTPUT;
	background();			// does not return

}		// end of initialize function

/********************************************************************/
/********************************************************************/
/*		Main function (called by hardware reset)					*/
/********************************************************************/
// located in its own code segment which is not reprogrammable
// cannot be part of .text segment because it is not copied to RAM
// performs the absolute minimum initialization required to run the 
// flash programming code
#pragma CODE_SECTION(main,".main_code")

void main (void)
{
	register int index,temp;
	register int far *int_pointer;
	register long far *long_dest_pntr,*long_src_pntr;

	asm("	C28OBJ");			// select C28 mode
	asm("	C28ADDR");			// select C28 mode
	asm("	C28MAP");			// select C28 mode
	asm("	CLRC OVM");
	asm("	EALLOW");			// enable protected access
	asm("	MOV @SP,#00H");		// initialize stack pointer


/********************************************************************/
/*	Initialize 	System Control Registers							*/
/********************************************************************/
	PF2.HISPCP=	0x0001;			// divide by 2 (75 MHz)
	PF2.LOSPCP=	0x0002;			// divide by 4 (37.5 MHz)
	PF2.PCLKCR=	0x1D0B;			// EV-A,EV-B,SCI-A,SCI-B,McBSP,ADC enabled
	PF2.PLLCR=	0x000A;			// multiply CLKIN by 10/2=5

	PF2.WDCR=	0x00E8;			// disable watchdog XCLKIN/512/256/1->4369us

/********************************************************************/
/*	Initialize GPIO Mux Registers									*/
/********************************************************************/
	PF2.GPAMUX	=	0x003F;	// PWM outputs 1-6
	PF2.GPADIR	=	0xF8FF;	// 15-11,7-0 output
	PF2.GPACLEAR=	TP48_OUTPUT|DIO_20_GPA;

//	PF2.GPACLEAR=	AC_CONTACT23_OUTPUT | FAN3_RELAY | FAN3_PWM_COMMAND;
	PF2.GPASET=	AC_CONTACT23_OUTPUT | FAN3_RELAY | FAN3_PWM_COMMAND;

	PF2.GPBMUX	=	0x003F;	// PWM outputs 7-12
	PF2.GPBDIR	=	0x5C3F;	// 14,12-10,5-0 output
	PF2.GPBSET	=	0x0800|0x4000;
	PF2.GPBCLEAR=	RS485_TXEN_OUTPUT|0x1000;

	PF2.GPDMUX=	0x0063;
	PF2.GPFMUX=	0x35F7;	// Serial I/O
	PF2.GPEMUX=	0x0007;		// Interrupt inputs
	PF2.GPFDIR=	SPI_CS;
	PF2.GPGMUX=	(SCI_B_TX|SCI_B_RX);
	PF2.GPFSET=	SPI_CS;
	
/********************************************************************/
/*	Unlock memory 													*/
/********************************************************************/

	int_pointer=(int far*)FLASH_PASSWORD;;
	for (index=0;index<8;index++)
		temp=*int_pointer++;

/********************************************************************/
/*	Copy .text code segment to L1_RAM (4K)							*/
/********************************************************************/
	long_src_pntr=(long far*)TEXT_FLASH;	// load address
	long_dest_pntr=(long far*)L1_RAM;		// run address
	for (index=0; index<L1_RAM_SIZE; index+=2)
		*(long_dest_pntr++)=*(long_src_pntr++);
	configure_flash(5);

/********************************************************************/
/*	If jumper J13 installed go to flash programming mode			*/
/********************************************************************/
	if (!(PF2.GPBDAT & 0x0200))
		program_flash();
	else

/********************************************************************/
/*	If initialization code is present go to normal operating mode	*/
/********************************************************************/
	{
		int_pointer=(int far*)&initialize;

		for (index=0;index<8;index++)
		{
			temp=*int_pointer++;
			if (temp!=0xFFFF)				// flash not blank
				initialize();				// normal operating mode
		}
		program_flash();					// flash programming mode
	}

}	// end main

