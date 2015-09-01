/********************************************************************/
/*	FPGA Input Registers 											*/
/********************************************************************/
//												FPGA decoder #  XA[4..1]
#define	ADC_START		(*((volatile int *)0x2000))	//0
#define	ADC_RD1			(*((volatile int *)0x2002))	//1
#define	ADC_RD2			(*((volatile int *)0x2004))	//2
#define	ADC_RD3			(*((volatile int *)0x2006))	//3
#define	ADC_RD4			(*((volatile int *)0x2008))	//4
#define	EXT_INPUT		(*((volatile int *)0x200A))	//5
#define	FPGA_INPUT		(*((volatile int *)0x200C))	//6
#define	PWM_FLT_INPUT	(*((volatile int *)0x200E))	//7
#define	WATCHDOG_INPUT	(*((volatile int *)0x2010))	//8
#define	VERSION_INPUT	(*((volatile int *)0x2012))	//9
#define FPGA_TEST_INPUT	(*((volatile int *)0x2014))	//10
#define DIN_FILTER_CNFG	(*((volatile int *)0x2016))	//11

//parallel dac chip select		    				    M,A1,A0
#define	DAC1_CNTL		(*((volatile int *)0x2018))//12	(1,1),0,0,0	
#define	DAC2_CNTL		(*((volatile int *)0x2019))//12	(1,1),0,0,1	XA[0]=1
#define	DAC3_CNTL		(*((volatile int *)0x201A))//13 (1,1),0,1,0
#define	DAC4_CNTL		(*((volatile int *)0x201B))//13	(1,1),0,1,1	XA[0]=1
#define	DAC1_DATA		(*((volatile int *)0x201C))//14 (1,1),1,0,0
#define	DAC2_DATA		(*((volatile int *)0x201D))//14	(1,1),1,0,1	XA[0]=1
#define	DAC3_DATA		(*((volatile int *)0x201E))//15 (1,1),1,1,0
#define	DAC4_DATA		(*((volatile int *)0x201F))//15	(1,1),1,1,1	XA[0]=1

/********************************************************************/
/*	FPGA Output Registers 											*/   
/********************************************************************/
#define	EXT_OUTPUT			(*((volatile int *)0x2000))
#define	PWM_EN_OUTPUT		(*((volatile int *)0x2002))
#define	FAN_SPD_OUTPUT		(*((volatile int *)0x2004))
#define	FPGA_OUTPUT			(*((volatile int *)0x2006))
#define	FAN_SPD_OUTPUT_2	(*((volatile int *)0x2008))
#define	WATCHDOG_RESET		(*((volatile int *)0x2010))
#define	FPGA_TEST_OUTPUT	(*((volatile int *)0x2014))

/********************************************************************/
/*	Digital input bits												*/
/********************************************************************/
// EXT_INPUT
#define DISCONNECT_CLOSED		!(ext_inputs&0x0001)
#define DOOR_SW_INPUT			0x0002
#define DC_CONTACTOR_INPUT		0x0004
#define GNDFLT_INPUT			0x0008
#define SURGE_SUPP_INPUT		0x0010
#define REAC_TEMP_FLT_INPUT		0x0020
#define XFMR_TEMP_FLT_INPUT		0x0040
#define FUSE_FLT_INPUT1			0x0080

#define FAN_FAULT_INPUT_1		0x0100
#define AC_CONTACTOR_INPUT		0x0200
#define AC_SURGE_SUPP_INPUT		0x0400
#define AC_BREAKER_INPUT		0x0800	// not used
#define ON_OFF_INPUT			0x1000
#define ESTOP_INPUT				0x2000
#define INV_TEMP_FLT_INPUT1		0x4000
#define FAN_FAULT_INPUT_2		0x8000

// FPGA_INPUT
#define PS_FLT_INPUTS			0x000F
#define	WATCHDOG_FLT_INPUT		0x0010
#define HWOC_FLT_INPUT1			0x0020
#define HWOC_FLT_INPUT2			0x0040
#define PWM_FB_FLT_INPUT		0x0080	// not used
#define RX_FPGA					0x0100
//#define 						0x0200
#define HW_GATE_ENABLE			0x0400	// not used
#define NVRAM_INT				0x0800	// not used
#define ADC_BUSY				0xF000

//	GPG
#define	SCI_B_TX				0x0010
#define	SCI_B_RX				0x0020

/********************************************************************/
/*	Digital output bits												*/
/********************************************************************/
// EXT_OUTPUT
#define GD_RESET_OUTPUT			0x0001	// DOUT 1
#define AC_CONTACTOR_OUTPUT		0x0002	// DOUT 2
#define PRECHARGE_OUTPUT		0x0004	// DOUT 3
#define DC_CONTACTOR_OUTPUT		0x0008	// DOUT 4
#define GENERATING_OUTPUT		0x0010	// DOUT 5
#define	SW_FAULT_OUTPUT			0x0020	// DOUT 6
#define AC_BREAKER_OUTPUT		0x0040	// DOUT 7
#define FAN_OUTPUT				0x0080	// DOUT 8

#define	FAN_PWM_OUTPUT_1		0x0100	// DOUT 9
#define	FAN_PWM_OUTPUT_2		0x0200	// DOUT 10
#define RUN_OUTPUT				0x0400	// DOUT 11
// 500 kW only
#define PRECHARGE_OUTPUT_2		0x8000	// DOUT 16
#define AC_CONTACTOR_OUTPUT_2	0x0800	// DIO_17 thru PF2 is used which is 0x0800

//FPGA_OUTPUT
#define RUN_LED_OUTPUT		0x0001
#define FAULT_LED_OUTPUT	0x0002
#define INHIBIT_FAULT_INDUCTOR_TRIP	0x0004
#define START_OUTPUT		0x0008
#define FLT_RESET_OUTPUT	0x0010
#define INVERT_GATE_FB		0x0020
#define ENA_PWMFB_A_OUTPUT	0x0040
#define ENA_PWMFB_B_OUTPUT	0x0080
#define FAN_PWM_DISABLE_1	0x0100	// 0 for pwm, 1 for normal output
#define FAN_PWM_DISABLE_2	0x0200	// 0 for pwm, 1 for normal output
#define TX_FPGA				0x0400

// GPA
#define DIO_17_GPA			0x0800
#define TP48_OUTPUT			0x1000
#define DIO_18_GPA			0x2000
#define DIO_19_GPA			0x4000
#define DIO_20_GPA			0x8000

// GPB
#define RS485_TXEN_OUTPUT	0x0400

/********************************************************************/
/*	Real Time Clock Registers										*/
/********************************************************************/

#define RTC_YEAR		(*((volatile far int *)0x9FFFF))
#define RTC_MONTH		(*((volatile far int *)0x9FFFE))
#define RTC_DATE		(*((volatile far int *)0x9FFFD))
#define RTC_DAY			(*((volatile far int *)0x9FFFC))
#define RTC_HOUR		(*((volatile far int *)0x9FFFB))
#define RTC_MINUTE		(*((volatile far int *)0x9FFFA))
#define RTC_SECOND		(*((volatile far int *)0x9FFF9))
#define RTC_CALIBRATE	(*((volatile far int *)0x9FFF8))
#define RTC_WATCHDOG	(*((volatile far int *)0x9FFF7))
#define RTC_CENTURY		(*((volatile far int *)0x9FFF1))
#define RTC_FLAGS		(*((volatile far int *)0x9FFF0))



//7805  DAC

//dac main control reg		
#define DAC_MC_SCLR			0x08
#define DAC_MC_STANDBY		0x10
#define DAC_MC_POWERUP		0x20
#define DAC_MC_BINARY		0x40
#define DAC_MC_8BIT			0x80
#define DAC_MC_BIPOLAR		0x00	//dummy

//dac channel control reg
#define DAC_CH_SELECT		0x001
#define DAC_CH_CLR			0x008
#define DAC_NOT_STBY		0x010
#define DAC_CH_SUB_NOT_MAIN	0x080
#define DAC_CH_MX0			0x100
#define DAC_CH_MX1			0x200

#define DAC_CH_VREF_INT		DAC_CH_MX0
#define DAC_CH_VREF_EXT		DAC_CH_MX1
