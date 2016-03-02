/********************************************************************/
/*  Program version                                                 */
/********************************************************************/
#define MAJOR_VERSION_NUMBER (18)
#define MINOR_VERSION_NUMBER (0)
#define BUILD_NUMBER         (1)


/********************************************************************/
/*	External functions												*/
/********************************************************************/
extern int div_q12 (register int x,register int y);
extern int int_sqrt (int input);
extern int long_sqrt (long input);
extern void read_nvram(register unsigned far *src_pointer,register unsigned far *dest_pointer,register unsigned length);
extern void write_nvram(register unsigned far *src_pointer,register unsigned far *dest_pointer,register unsigned length);

/********************************************************************/
/*	Constants														*/
/********************************************************************/
#define XCLKIN			30.0e6			// external clock
#define SYSCLKOUT		(XCLKIN*5.0)	// cpu clock
#define HSPCLK			(SYSCLKOUT/2.0)	// high speed peripheral clock
#define LSPCLK			(SYSCLKOUT/4.0)	// low speed peripheral clock
//	memory blocks
#define L0_RAM			0x8000			// used for data
#define L1_RAM			0x9000			// used to run flash programming & application code
#define L1_RAM_SIZE		0x1000			// 4K
#define FPGA_FLASH		0x3DC000L		// FPGA code (64K)
#define CODE_FLASH		0x3EC000L		// 2812 code and constants
#define L1_FLASH		0x3F3000L		// application code to be copied to ram (4K)
#define H0_FLASH		0x3F4000L		// application code to be copied to ram (8K)
#define CHKSUM_FLASH	0x3F5FF0L		// rom checksums
#define TEXT_FLASH		0x3F6000L		// flash programming code (sector A)
#define FLASH_PASSWORD	0x3F7FF8L
#define START_OF_FLASH	FPGA_FLASH		// start of reprogrammable flash
#define END_OF_FLASH	0x3F8000L		// end of reprogrammable flash
#define H0_RAM			0x3F8000L		// used to run application code
#define H0_RAM_SIZE		0x2000			// 8K
#define NVRAM			0x80000L

#define VALID_DATA_ADDRESS(addr)((addr>=STACK_SIZE)&&(addr<0x800))||((addr>=0x8000)&&(addr<0xA000))
#define	RD_ACCESS_SHIFT	6
#define	WR_ACCESS_SHIFT	4
#define ACCESS_MASK		0x03

#define RD_ONLY			(0x02)
#define NOWR_RUN		(0x04)

#define	WR0				(0x00)
#define WR1				(0x10)
#define WR2				(0x20)
#define WR3				(0x30)

#define	RD0				(0x00)
#define RD1				(0x40)
#define RD2				(0x80)
#define RD3				(0xc0)

#define AC_CONTACT23_OUTPUT	0x0800 //rs added
#define FAN3_RELAY			0x2000
#define FAN3_PWM_COMMAND	0x4000
#define SPI_CS				0x0008  //rs added
// array sizes
#define STACK_SIZE			0x180
#define FAULT_QUEUE_SIZE	100		// fault queue size
#define SAVED_VARS_SIZE		128
#define	DIAG_SIZE			1000	// number of samples
#define DATA_CHANNELS		8
#define STRING_FDBK_SIZE	32		// maximum number of strings
#define PWR_CURVE_SIZE		50
#define	CYCLES_PER_SAMPLE	10
#define RATINGS_SIZE		7

#define PARAM_MIN			300		// first writable parameter
#define PARAM_MAX			550		// last writable parameter (PARAMETER NUMBERS AND ADDRESSES MUST NOT OVERLAP)

#define NUM_STATUS_MSGS		13
#define SIZE_STATUS_MSG		13
#define NUM_FAULT_MSGS		103
#define SIZE_FAULT_MSG		16
#define LANGUAGES			2
#define FLT_MSG_MIN			1000
#define FLT_MSG_MAX			(FLT_MSG_MIN+(NUM_FAULT_MSGS*SIZE_FAULT_MSG)*LANGUAGES/2)

#define DAC_FULL_SCALE		1023
#define DAC_HALF_SCALE		512

#define SLOW_TASK_RATE	10		// per second
#define I_CMD_MAX		(int)(1.1*PER_UNIT_F)

#define	PER_UNIT			1000
#define	PER_UNIT_F			1000.0
#define DECIMAL_TO_PU_Q12	(int)(4096.0*(4096.0/PER_UNIT_F)+0.5)
#define PERCENT_TO_PU		(int)(PER_UNIT_F/100.0+0.5)
#define FREQ_SHIFT			11	// right shift required to convert frequency from 32 to 16 bits
#define ANGLE_SHIFT			4	// right shift required to convert 16-bit angle to sine table index
#define THREE_SIXTY_DEGREES	(unsigned)(65536>>ANGLE_SHIFT)	// for tables only
#define	THIRTY_DEGREES		((THREE_SIXTY_DEGREES+6)/12)
#define	NINETY_DEGREES		((THREE_SIXTY_DEGREES+2)/4)
#define	ONE_TWENTY_DEGREES	((THREE_SIXTY_DEGREES+1)/3)
#define	ONE_EIGHTY_DEGREES	((THREE_SIXTY_DEGREES+1)/2)
#define TWO_SEVENTY_DEGREES	((THREE_SIXTY_DEGREES*3+2)/4)
#define ANGLE_MASK			(THREE_SIXTY_DEGREES-1)
#define PI				3.1415926
#define SQRT2			1.414214
#define SQRT2_Q12		5796
#define SQRT3			1.732051
#define ONE_THIRD_Q16	21846
#define INVERSE_MIN		512

#define IDLE			0
#define RQST			1
#define BUSY			2

#define ADCTRL1_WORD	ADC2EN+ADC1EN+ADCINTFLAG+ADCEOC

#define NA				0	// North American region
#define EU				1	// European region

#define NEGATIVE_GND	0
#define POSITIVE_GND	1
#define	FLOATING_GND	2

/********************************************************************/
/*	Macro definitions												*/
/********************************************************************/
#define HIGH(xlong) ((int)((xlong)>>16))
#define LONG(x) 	((long)(x)<<16)
#define LIMIT_MAX_MIN(x,max,min) do{if(x>max) x=max; else if(x<min) x=min;}while(0);
#define MULQ(n,x,y) ((long)x*y>>n)					// x*y/2^n
#define MULQ_RND(n,x,y) (((long)x*y+(1L<<(n-1)))>>n)	// (x*y+2048)/4096
#define MULQ_RND_UP(n,x,y) (((long)x*y+(1L<<(n-1))+(1L<<(n-2)))>>n)	// (x*y+2048+1024)/4096

#define MAGNITUDE(m,d,q,s) m=(long_sqrt((long)(d<<s)*(d<<s)+(long)(q<<s)*(q<<s)))>>s;

#define FAULT_BIT(n) (faults[n>>4])&(1<<(n&0x0F))

#define SET_FAULT(n) (faults[n>>4])|=(1<<(n&0x0F));
#define CLEAR_FAULT(n) (faults[n>>4])&=~(1<<(n&0x0F));


//	alpha=(2*a-b-c)/3
//	beta=(c-b)/SQRT3
//	d=alpha*cos - beta*sin
//	q=alpha*sin + beta*cos		
#define ABC_TO_DQ(a,b,c,angle,sin,cos,alpha,beta,d,q)\
	do{sin=sin_table[angle];\
	cos=sin_table[(angle+NINETY_DEGREES)&ANGLE_MASK];\
	alpha=(long)((a<<1)-b-c)*(int)(4096/3.0)>>12;\
	beta=(long)(c-b)*(int)(4096/SQRT3)>>12;\
	d=((long)alpha*cos-(long)beta*sin)>>15;\
	q=((long)alpha*sin+(long)beta*cos)>>15;}while(0);
				
#define DQ_TO_ABC(a,b,c,sin,cos,d,q)\
	do{a=((long)d*cos + (long)q*sin)>>15;\
	c=((long)q*cos - (long)d*sin)>>15;\
	c=(long)c*(int)(4096*SQRT3)>>12;\
	b=(-a-c)>>1;c=(-a+c)>>1;}while(0);

#define FLT_TMR_LATCHING(cond,flt,n,setflt)\
	do{if(cond)\
	{if (flt.timer<flt.delay)\
		flt.timer++;\
	else {SET_FAULT(n)\
		if(setflt) status_flags=(status_flags&~(STATUS_READY|STATUS_RUN))|STATUS_FAULT;}}\
	else {if (flt.timer > 0)\
		flt.timer--;}}while(0);
 
#define PI_REG(cmd,fbk,err,out,kp,ki,max,min)\
	do{temp=cmd-fbk;\
	out=out+(long)ki*temp+(long)kp*(temp-err);\
	err=temp;\
	if(out>max) out=max;\
	else if (out<min) out=min;}while(0);

#define PI_REG_CS(cmd,fbk,err,i_term,out,kp,ki,max,min)\
	do{temp=cmd-fbk;\
	err=temp;\
	i_term+=(long)ki*temp;\
	LIMIT_MAX_MIN(i_term,max,min)\
	out=(long)kp*temp+i_term;\
	LIMIT_MAX_MIN(out,max,min)}while(0);

#define PI_REG_PF(cmd,fbk,err,i_term,out,kp,ki,max,min)\
	{\
		err=cmd-fbk;\
		i_term+=(long)ki*err;\
		LIMIT_MAX_MIN(i_term,max,min)\
		out=(long)kp*err+i_term;\
		LIMIT_MAX_MIN(out,max,min)\
	}

#define RAMP(in,out,up,down)\
	do{if (out<in) {if ((out+=up)>in) out=in;}\
	else if (out>in) {if ((out-=down)<in) out=in;}}while(0);

#define CONVERT_TIME(t_dec,t_bcd)\
	temp=t_bcd;\
	t_dec=(temp)&0xF;\
	t_dec+=((temp>>4)&0xF)*10;\
	t_dec+=((temp>>8)&0xF)*100;\
	t_dec+=(temp>>12)*1000;	

//	Modbus
#define WRITE_EXCEPTION(comm_data,code){\
	comm_data.buffer[FUNCTION_CODE]=comm_data.function_code|0x80;\
	comm_data.buffer[NUMBER_HIGH]=0;\
	comm_data.buffer[NUMBER_LOW]=code;}

#define READ_EXCEPTION(comm_data,code){\
	comm_data.buffer[FUNCTION_CODE]=comm_data.function_code|0x80;\
	comm_data.buffer[READ_DATA]=0;\
	comm_data.buffer[READ_DATA+1]=code;\
	comm_data.last_data_byte=READ_DATA+1;}

/********************************************************************/
/* Define command values */
/********************************************************************/
#define CMD_START		1
#define CMD_STOP		2
#define CMD_RESET		3
#define CMD_SHUTDOWN	4
#define CMD_LOCAL		6
#define CMD_REMOTE		7
#define CMD_ANALOG		8

/********************************************************************/
/*	Define power control modes										*/
/********************************************************************/

#define PCM_CONST_I		0
#define PCM_CONST_P		1
#define PCM_MPPT		2
#define PCM_CONST_VDC	3
#define PCM_DC_POWER	4
#define PCM_AC_VOLTAGE  5

/********************************************************************/
/*	Define serial communication literals							*/
/********************************************************************/

// for receive_state
#define	NO_COMM			0
#define	GET_FUN			1                   
#define	GET_ADD_NUM		2
#define	GET_BYTE_CNT	3
#define	GET_DATA		4
#define	GET_CHKLO		5
#define	GET_CHKHI		6
#define	WAIT_COMM		7
// for transmit_state
#define	REQUEST_RESP	1
#define	PREPARE_RESP	2
#define	SEND_RESP		3
#define	SEND_REQUEST	4

#define	MAXLEN			266	// size of communications buffer
#define THE_TX_BUF_SIZE (MAXLEN+2)
#define THE_RX_BUF_SIZE (MAXLEN+2)
#define	DATALEN			126	// size of address and data buffers
#define MODBUS_IDLE_DELAY	20	// 2 seconds

#define SLAVE_ADDR			0
#define FUNCTION_CODE		1
#define START_ADDR_HIGH		2
#define START_ADDR_LOW		3
#define NUMBER_HIGH			4
#define NUMBER_LOW			5
#define CHKSUM_LOW			6
#define CHKSUM_HIGH			7
#define READ_BYTE_COUNT		2
#define WRITE_BYTE_COUNT	6
#define READ_DATA			3
#define WRITE_DATA			7
#define	INVALID_FUNCTION	1	// exception codes
#define	INVALID_ADDRESS		2
#define	INVALID_DATA		3

/********************************************************************/
/* Define TP48_OUTPUT											 	*/ 
/********************************************************************/
#define TP48_PWM	0
#define TP48_SYN	1	
#define TP48_1MS	2
#define TP48_10MS	3
#define TP48_100MS	4
#define TP48_SLW	5
#define TP48_QUE	6
#define TP48_VAR	7
#define TP48_FGND	8
#define TP48_BKGND	9


/********************************************************************/
/* Define Test types */ 
/********************************************************************/
#define GATE_TEST			1
#define OPEN_CCT_TEST		2
#define SHORT_CCT_TEST		3
#define UL_TEST				4
#define SIMULATE			5
#define ANALOG_IO_TEST		6
#define DIGITAL_IO_TEST		7
#define GATING_IO_TEST		8
#define SERIAL_IO_TEST		9
#define COMPLETE_IO_TEST	10

/********************************************************************/
/* Define bits of remote_command									*/
/********************************************************************/
#define REMOTE_ENABLE	0x0001
#define REMOTE_DISABLE	0x0002
#define REMOTE_RESET	0x0004
#define REMOTE_ESTOP	0x0008
#define REMOTE_LOCAL	0x0010
#define REMOTE_REMOTE	0x0020
#define REMOTE_ANALOG	0x0040

/********************************************************************/
/* Define operating states											 */
/********************************************************************/
#define STATE_POWERUP				0
#define STATE_SHUTDOWN 				1
#define STATE_STOP		 			2
#define STATE_PRECHG_CLOSED			3
#define STATE_1ST_CONTACTOR_CLOSED	4
#define STATE_MATCH_VOLTAGE			5
#define STATE_2ND_CONTACTOR_CLOSED	6
#define STATE_RUN 					7
#define STATE_GATE_TEST				8
#define STATE_OPEN_CCT_TEST			9
#define STATE_SHORT_CCT_TEST		10
#define STATE_UL_TEST 				11
#define STATE_BOARD_TEST 			12

/********************************************************************/
/* Define status flags												*/
/********************************************************************/
#define STATUS_INITIALIZED		0x0001
#define STATUS_PLL_ENABLED		0x0002
#define STATUS_PLL_LOCKED		0x0004
#define STATUS_RVS				0x0008
#define STATUS_LINE_OK			0x0010
#define STATUS_VDC_OK			0x0020
#define STATUS_READY			0x0040
#define STATUS_FAULT			0x0080
#define STATUS_SHUTDOWN			0x0100
#define STATUS_RUN				0x0200
#define STATUS_GENERATING		0x0400
#define STATUS_LINE_LINKED		0x0800
#define STATUS_GATE_TEST		0x1000
#define STATUS_OPEN_CCT_TEST	0x2000
#define STATUS_SHORT_CCT_TEST	0x4000
#define STATUS_SIMULATE			0x8000
#define STATUS_TEST	(STATUS_GATE_TEST|STATUS_OPEN_CCT_TEST|STATUS_SHORT_CCT_TEST)
#define STATUS_TEST_MASK	(STATUS_TEST|STATUS_SIMULATE)

/********************************************************************/
/* Define global_flags												*/
/********************************************************************/
#define SAVE_QUEUE			0x0001
#define SAVE_VARIABLES		0x0002
#define SAVE_PARAMETERS		0x0004
// 500 kW analog inputs
#define AC_CONTACTOR_INPUT2		0x0100
#define FUSE_FLT_INPUT2			0x0200
#define INV_TEMP_FLT_INPUT2		0x0400
#define REAC_TEMP_FLT_INPUT2	0x0800

#define	MATCH_AC_VOLTAGE		0x1000


/* advance controls flags */
#define ADV_CON_DIS_5_HARM		0x0001
#define ADV_CON_IGN_FANFLT		0x0002
#define ADV_CON_SKIP_VMATCH		0x0004
#define ADV_CON_DIS_3_HARM      0x0008
#define ADV_CON_DIS_LOW_POWER	0x0010
#define ADV_CON_QCLOSE_DC_CON	0x0020
#define ADV_CON_PTRB_LINE_ID	0x0040
#define ADV_CON_PTRB_INV_ID     0x0080
#define ADV_CON_UNUSED_0100		0x0100
#define ADV_CON_UNUSED_0200		0x0200
#define ADV_CON_UNUSED_0400		0x0400
#define ADV_CON_UNUSED_0800		0x0800
#define ADV_CON_UNUSED_1000		0x1000
#define ADV_CON_UNUSED_2000		0x2000
#define ADV_CON_UNUSED_4000		0x4000
#define ADV_CON_UNUSED_8000		0x8000

#define POWER_SOURCE_CURRENT	0
#define POWER_SOURCE_VOLTAGE	1

/********************************************************************/
/* Define feedback array											*/
/********************************************************************/
#define FDBK_SIZE		43
#define AC_FDBK_SIZE	16
#define RAW_FDBK_SIZE	32
// bipolar measured inputs
#define VLA		0
#define VLB		1
#define VLC		2
#define VIBC	3
#define ILA		4
#define ILB		5
#define ILC		6
#define ILN		7
#define IIA		8
#define IIB		9
#define IIC		10
#define IIA2	11
#define IIB2	12
#define IIC2	13
#define GND		14
#define DCIMB	15
#define FUSEV	15
// unipolar measured inputs
#define CAL1	16
#define CAL2	17
#define VDCIN	18
#define VDC		19
#define SPARE0	20
#define BENDER	21
#define TMP		22
#define IDC		23
#define CAL3	24
#define CAL4	25
#define EX1		26
#define EX2		27
#define EX3		28
#define EX4		29
// calculated values
#define ID		30
#define IQ		31
#define VL		32
#define VLUB	33
#define IL		34
#define ILUB	35
#define II		36
#define FRQ		37
#define FRQERR	38
#define PDC		39
#define KW		40
#define KVAR	41
#define KVA		42


