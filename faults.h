/********************************************************************/
/* Define fault words												*/
/********************************************************************/
#define FAULT_WORDS			7

/********************************************************************/
/* Define bits of faults[0]											*/
/********************************************************************/
#define DCIN_FLT			0
#define VOLT_MATCHING_FLT	1
#define	STOP_FLT			2
#define	SHUTDOWN_FLT		3
#define ESTOP_FLT			4
#define LOW_POWER_FLT		5
#define LOW_CURRENT_FLT		6
#define	ILLEGAL_MODE_CHANGE 7
#define DOOR_SW_FLT			8
#define DISCONNECT_OPEN_FLT	9
#define BREAKER_OPEN_FLT	10
#define DPCB_FAULT			11
#define HARDWARE_FAULT		12
#define INVERTER_FAULT		13
#define TEMP_FAULT			14
#define	I_LEAKAGE_FLT		15

/********************************************************************/
/* Define bits of faults[1]											*/
/********************************************************************/
#define DCIN_OV_FLT			16	// 1h
#define DCIN_UV_FLT			17
#define DC_OV_FLT			18
#define DC_UV_FLT			19
#define DC_GNDFLT			20	// 10h
#define LINE_OV_FLT_SLOW	21
#define LINE_OV_FLT_FAST	22
#define LINE_UV_FLT_SLOW	23
#define LINE_UV_FLT_FAST	24	// 100h
#define LINE_VUB_FLT		25
#define OVERFREQ_FLT		26
#define UNDERFREQ_FLT		27
#define UNDERFREQ_FLT_FAST	28	// 1000h
#define LINE_OCN_FLT		29
#define	LINE_OV_FLT_INST	30
#define	PWM_SYNC_FAULT		31
#define UV_MASK				0x1F8A
#define RECONNECT_MASK		0x1FE0

/********************************************************************/
/* Define bits of faults[2]											*/
/********************************************************************/
#define FLASH_CHKSUM_FLT	32
#define WRONG_RATINGS_FLT	33 //rs added
#define DATA_CHKSUM_FLT1	34
#define DATA_CHKSUM_FLT2	35
#define PARAM_CHKSUM_FLT_A1	36
#define PARAM_CHKSUM_FLT_A2	37
#define PARAM_CHKSUM_FLT_B1	38
#define PARAM_CHKSUM_FLT_B2	39
#define V_SCALING_FLT		40
#define I_SCALING_FLT		41
#define	IINV_DIFF_FLT		42
#define PARAM_CHANGE_FAULT	43
#define STACK_FLT			44
#define ADC_FAULT			45
#define NVRAM_FLT			46
#define FPGA_FLT			47
#define DPCB_FLT_MASK		0x0400	// IINV_DIFF_FLT

/********************************************************************/
/* Define bits of faults[3]											*/
/********************************************************************/
#define ISO_P5V_FLT			48
#define P5V_FLT				49
#define P15V_FLT			50
#define N15V_FLT			51
#define WATCHDOG_FLT		52
#define SURGE_SUPP_FLT		53
#define INV_FUSE_FLT1		54
#define INV_FUSE_FLT2		55
#define INV_TEMP_FLT1		56
#define INV_TEMP_FLT2		57
#define XFMR_TEMP_FLT		58
#define REAC_TEMP_FLT		59
#define PRECHARGE_FLT		60
#define TEST_MODE_FLT		61
#define OPEN_CCT_TEST_FLT	62
#define SHORT_CCT_TEST_FLT	63

/********************************************************************/
/* Define bits of faults[4] 										*/
/********************************************************************/
#define PWM_FLT_MASK_1		0x0007
#define DCIN_OC_FLT			70
#define DCIN_OC_INST_FLT	71
#define DC_UV_INST_FLT		72
#define DC_OV_INST_FLT		73
#define INV_OC_FLT			74
#define INV_HW_OC_FLT1		75
#define INV_HW_OC_FLT2		76
#define LINE_OC_FLT			77
#define LINE_CUB_FLT		78
#define	INV_UV_FLT			79

/********************************************************************/
/* Define bits of faults[5]											*/
/********************************************************************/
#define HIGH_TEMP_FLT_1		80
#define HIGH_TEMP_FLT_2		81
#define HIGH_TEMP_FLT_3		82
#define HIGH_TEMP_FLT_4		83
#define HIGH_TEMP_FLT_5		84
#define HIGH_TEMP_FLT_6		85
#define HIGH_TEMP_FLT_7		86
#define HIGH_TEMP_FLT_8		87
#define LOW_TEMP_FLT_1		88
#define LOW_TEMP_FLT_2		89
#define LOW_TEMP_FLT_3		90
#define LOW_TEMP_FLT_4		91
#define LOW_TEMP_FLT_5		92
#define LOW_TEMP_FLT_6		93
#define LOW_TEMP_FLT_7		94
#define LOW_TEMP_FLT_8		95

/********************************************************************/
/* Define bits of faults[6]											*/
/********************************************************************/
#define FAN_FLT_1			96
#define FAN_FLT_2			97
#define	INPUT_OPEN_FLT		98
#define INPUT_CLOSED_FLT	99
#define	OUTPUT_OPEN_FLT		100
#define OUTPUT_CLOSED_FLT	101
//#define	102
//#define 	103
//#define 	104
//#define 	105
//#define 	106
//#define 	107
//#define 	108
//#define 	109
//#define 	110
//#define 	111

/********************************************************************/
/* Define bits of gating_faults										*/
/********************************************************************/
#define DO_FLT_17_20		0x1000
#define DAC_FLT_1			0x2000
#define DAC_FLT_2			0x4000
#define DAC_FLT_3			0x8000

/********************************************************************/
/* Define bits of serial_comm_faults								*/
/********************************************************************/
#define	RX_C_TX_B			0x01
#define	RX_B_TX_C			0x02


