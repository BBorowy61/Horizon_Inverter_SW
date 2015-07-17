/********************************************************************/
/********************************************************************/
/*																	*/
/*							Fault Module							*/
/*																	*/
/********************************************************************/
/********************************************************************/

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
2008-03-00	add second fan pwm output
2008-03-27	delete fault_masks
			add SET_FAULT and CLEAR_FAULT macros
2008-07-04	add powerup delay timer
2008-08-05	increase powerup delay from 0.3 to 0.4 sec
2008-08-18	add on-off switch input for euro version
2008-08-19	add ground impedance fault for euro version
2008-08-21	use dc contactor cmd instead of feedback in dc input uv fault
			add current difference fault to fault reset
2008-10-10	add inverter undervoltage fault
2008-11-11	disable inverter undervoltage fault in all test modes
1 MW version
2009-01-23	add line instantaneous overvoltage fault as requested by
			Pat McGinn
2009-01-30	move current magnitude calculations to regulator
2009-02-12	modify gate feedback faults for 1 MW (5 inputs)
2009-03-03	add ground leakage current faults for 1 MW
2009-03-04	move 1 MW analog flag inputs to avg_fbk module
2009-04-09	add maximum ground impedance fault
2009-04-24	clear auto_reset_attempts during fault reset
2009-05-07	move line instantaneous overvoltage fault to regulator module
Solstice version
2009-07-20	delete 1 MW features
			delete ground leakage fault
			delete ground fault reset output
2009-08-14	correct error in inverter undervoltage fault
2009-10-15	change number of temp feedbacks for 2 bridges from 3 per bridge to 2 per bridge
*/
#include "literals.h"
#include "2812reg.h"
#include "io_def.h"
#include "struct.h"
#include "faults.h"

/********************************************************************/
/*	Literal definitions												*/
/********************************************************************/
#define POWERUP_DELAY_10MS		40			// 0.4 sec
#define CONTACTOR_DELAY_SEC		5.0
#define HYSTERESIS				(int)(PER_UNIT_F*0.05)
#define ADC_MAX_ERROR_MV		100			// millivolts
#define GNDFLT_RESET_DELAY_10MS	100			// 1 second
#define FAN_SET_DELAY_SEC		2.0
#define FAN_CLR_DELAY_SEC		10.0
#define SET_FLT_STS				0
#define TEMP_FBK_MIN			-20

#define FAN_TIMER_START_VALUE	(10 * 1000) // 10 sec x 1000/sec


/********************************************************************/
/*	Macro definitions												*/
/********************************************************************/
#define FLT_TMR_CLEARING(setcond,clrcond,flt,n,setflt)\
	do{if(!(FAULT_BIT(n)))\
	{if (setcond)\
	{if (flt.timer<flt.delay)\
		flt.timer++;\
	else {SET_FAULT(n)\
		if(setflt) status_flags=(status_flags&~(STATUS_READY|STATUS_RUN))|STATUS_FAULT;}}}\
	else if (clrcond)\
	{if (flt.timer > 0)\
		flt.timer--;\
	else CLEAR_FAULT(n)}}while(0);

	//for faster clear on slow faults
	#define FLT_TMR_FAST_CLEARING(setcond,clrcond,flt,n,setflt)\
	do{if(!(FAULT_BIT(n)))\
	{if (setcond)\
	{if (flt.timer<flt.delay)\
		flt.timer++;\
	else {SET_FAULT(n)\
		if(setflt) status_flags=(status_flags&~(STATUS_READY|STATUS_RUN))|STATUS_FAULT;}}}\
	else if (clrcond)\
	{if (flt.timer > 0)\
		flt.timer-=100;\
	else {CLEAR_FAULT(n) flt.timer=0;}}}while(0);
 
/********************************************************************/
/*		Global variables											*/
/********************************************************************/
#pragma DATA_SECTION(flts,".flt_data")
struct FLTS flts;
struct FLT pll_enable;
int faults[FAULT_WORDS];
int temp_fbk[8],temp_fbk_mv[8];
int heatsink_temp_max[2],number_temp_fbk;
int line_voltage_timer_sec;
int ext_inputs,ext_input_mask,ext_output_mask,ac_contactor_input,contactor_inputs;
int a2d_offset_mv,hw_oc_trip;
int fault_queue_index,number_faults;

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
int heatsink_temp_old;
int fault_reset_timer_ms,pwm_flt_inputs,line_voltage_timer_10ms;
int dc_contactor_timer_10ms,ac_contactor_timer_10ms;
int door_switch_timer_10ms;
int temp_index,fan_flt_timer_10ms[2];
int a2d_offset_counts,a2d_gain_q12;
unsigned powerup_timer_10ms;
unsigned utemp,fan_tach_bit[2],fan_tach_bit_mask[2];
int fan_fault_bit[2],fan_timer[2],fan_reset_count[2];
int enable_uv_tests;

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern unsigned hw_stack[STACK_SIZE];
extern int fdbk_pu[FDBK_SIZE];
extern int fdbk_display[FDBK_SIZE];
extern int status_flags,pc_cmd,gnd_impedance;
extern int fpga_inputs,fpga_outputs,ext_outputs,ac_contactor_output;
extern int fdbk_avg[FDBK_SIZE];	
extern int timer_1ms,test_mode,auto_reset_attempts,global_flags;
extern int ac_contactor_input,contactor_outputs,dc_bus_imbalance_ma_x10,operating_state;
extern struct PARAMS parameters;
extern int fan_spd_cmd_out[];
extern int debugIndex,debug[],debug_control,dv[];

#pragma CODE_SECTION(find_faults,".h0_code")

/********************************************************************/
/********************************************************************/
/*		Fault function	(called at 1 ms rate)						*/
/********************************************************************/

void find_faults(void)
{
	register int i,temp,trip_level,clear_level,fbk_mv;
	register int *src_pntr;

/********************************************************************/
/* Reset faults if required 										*/
/********************************************************************/
	if (fpga_outputs&FLT_RESET_OUTPUT)
	{
		if (fault_reset_timer_ms > 0)
			fault_reset_timer_ms--;
		else
		{
			faults[0]&=(1 << ILLEGAL_MODE_CHANGE); // clears all the faults except the ILLEGAL_MODE_CHANGE fault
			faults[1]&=RECONNECT_MASK;
			faults[2]&=~DPCB_FLT_MASK;
			faults[3]=faults[4]=faults[5]=faults[6]=0;
			fpga_outputs&=~FLT_RESET_OUTPUT;
			ext_outputs&=~GD_RESET_OUTPUT;
			fan_reset_count[0] = fan_reset_count[1] = 0;
			fan_timer[0] = fan_timer[1] = 0;
		}
	}

   	else if ((pc_cmd==CMD_RESET)||!(fpga_outputs&RUN_LED_OUTPUT))
	{
		// RUN_LED_OUTPUT is zero only after hardware reset
		// this forces a fault reset after a hardware reset
		fpga_outputs|=(RUN_LED_OUTPUT|FLT_RESET_OUTPUT);
		if (operating_state <= STATE_STOP)
			ext_outputs|=GD_RESET_OUTPUT;
		WATCHDOG_RESET=0;
		pc_cmd=0;
		fault_reset_timer_ms=1000;

		fan_tach_bit_mask[0] = FAN_FAULT_INPUT_1;
		fan_tach_bit_mask[1] = FAN_FAULT_INPUT_2;
		fan_fault_bit[0] = FAN_FLT_1;
		fan_fault_bit[1] = FAN_FLT_2;
	}

/********************************************************************/
/*		Read external inputs										*/
/********************************************************************/
	ext_output_mask=0xFFFF;
	temp=~(EXT_INPUT | ext_input_mask);
	if (status_flags & STATUS_SIMULATE)
	{
		ext_output_mask&=~contactor_outputs;
		temp=contactor_inputs|ON_OFF_INPUT;
		if (ext_outputs & DC_CONTACTOR_OUTPUT)
			temp&=~DC_CONTACTOR_INPUT;
		if (ext_outputs & ac_contactor_output)
			temp&=~ac_contactor_input;
	}
	ext_inputs=temp;

	// PFP enable dc uv faults formerly controlled by DC_CONTACTOR_OUTPUT status
	enable_uv_tests = (ext_outputs & DC_CONTACTOR_OUTPUT);
	if(ADV_CON_QCLOSE_DC_CON & parameters.adv_control_configuration) { // DC Con closes early
			if(operating_state <= STATE_MATCH_VOLTAGE)
					enable_uv_tests = 0;
	}

/********************************************************************/
/* Check fast faults at 1 ms intervals 								*/
/********************************************************************/
/********************************************************************/
/* 	Detect stack overflow 											*/
/********************************************************************/
	src_pntr=(int*)&hw_stack[STACK_SIZE];
	for (temp=10;temp>0;temp--)
	{
		if (*(--src_pntr)!=0xDEAD)
			SET_FAULT(STACK_FLT)
	}

/********************************************************************/
/* DC input Over Voltage */
/********************************************************************/
	FLT_TMR_LATCHING((fdbk_avg[VDCIN]>flts.dcin_ov.trip),flts.dcin_ov,DCIN_OV_FLT,SET_FLT_STS)

/********************************************************************/
/*	DC input instantaneous over current 		 					*/
/********************************************************************/
	FLT_TMR_LATCHING((fdbk_pu[IDC]>=flts.dcin_oc_inst.trip),flts.dcin_oc_inst,DCIN_OC_INST_FLT,SET_FLT_STS)

/********************************************************************/
/* DC input Over Current */
/********************************************************************/
	FLT_TMR_LATCHING((fdbk_avg[IDC]>flts.dcin_oc.trip),flts.dcin_oc,DCIN_OC_FLT,SET_FLT_STS)

/********************************************************************/
/* DC link instantaneous over voltage 								*/
/********************************************************************/
	FLT_TMR_LATCHING((fdbk_pu[VDC]>=flts.dc_ov_inst.trip),flts.dc_ov_inst,DC_OV_INST_FLT,SET_FLT_STS)
		
/********************************************************************/
/* DC link Over Voltage */
/********************************************************************/
	FLT_TMR_LATCHING((fdbk_avg[VDC]>flts.dc_ov.trip),flts.dc_ov,DC_OV_FLT,SET_FLT_STS)

/********************************************************************/
/* Inverter over current  */
/********************************************************************/
	if (!(status_flags & STATUS_SIMULATE))
		FLT_TMR_LATCHING((fdbk_pu[II]>flts.inv_oc.trip),flts.inv_oc,INV_OC_FLT,SET_FLT_STS)

/********************************************************************/
/*	DC ground fault													*/
/********************************************************************/
	if (parameters.gnd_config==NEGATIVE_GND) {
		FLT_TMR_LATCHING((abs(fdbk_avg[GND])>flts.gnd_flt.trip),flts.gnd_flt,DC_GNDFLT,SET_FLT_STS)
		FLT_TMR_LATCHING((abs(fdbk_avg[FUSEV])>flts.fuse_voltage.trip),flts.fuse_voltage,DC_GNDFLT,SET_FLT_STS)
	}
	else if(parameters.gnd_config==POSITIVE_GND) {
	}
	else {

		if (status_flags & STATUS_RUN) {
			FLT_TMR_LATCHING((abs(fdbk_pu[DCIMB])>parameters.dc_bus_imbalance_trip),flts.gnd_flt,DC_GNDFLT,SET_FLT_STS)
		}
		else {
			FLT_TMR_LATCHING((gnd_impedance<flts.gnd_impedance.trip),flts.gnd_flt,DC_GNDFLT,SET_FLT_STS)
		}
	}

/********************************************************************/
/*	Line neutral overcurrent fault (scaled 10=1.0A)					*/
/********************************************************************/
	 FLT_TMR_LATCHING((fdbk_avg[ILN]>flts.line_oc_neutral.trip),\
	 				flts.line_oc_neutral,LINE_OCN_FLT,SET_FLT_STS) 
	
/********************************************************************/
/* Inverter under voltage  */
/********************************************************************/
	if (((ext_inputs & ac_contactor_input)!=ac_contactor_input) && (test_mode==0))
		FLT_TMR_LATCHING((fdbk_avg[VIBC]<flts.inv_uv.trip),flts.inv_uv,INV_UV_FLT,SET_FLT_STS)

/********************************************************************/
/*	Line overcurrent fault 											*/
/********************************************************************/
	FLT_TMR_LATCHING((fdbk_pu[IL]>flts.line_oc.trip),\
				flts.line_oc,LINE_OC_FLT,SET_FLT_STS)

/********************************************************************/
/*	Current difference faults 										*/
/********************************************************************/
	temp=abs(fdbk_pu[IL]-fdbk_pu[II]);
	FLT_TMR_LATCHING((temp>flts.iinv_diff.trip),flts.iinv_diff,IINV_DIFF_FLT,SET_FLT_STS)
	
/********************************************************************/
/*	Line overvoltage fault fast										*/
/********************************************************************/
	trip_level=flts.line_ov_fast.trip;
	clear_level=flts.line_ov_fast.trip-HYSTERESIS;
	FLT_TMR_CLEARING(((fdbk_avg[VLA]>trip_level)||(fdbk_avg[VLB]>trip_level)||(fdbk_avg[VLC]>trip_level)),\
				((fdbk_avg[VLA]<clear_level)&&(fdbk_avg[VLB]<clear_level)&&(fdbk_avg[VLC]<clear_level)),\
				flts.line_ov_fast,LINE_OV_FLT_FAST,1)

/********************************************************************/
/*	Line overvoltage fault slow	 									*/
/********************************************************************/
	trip_level=flts.line_ov_slow.trip;
	clear_level=flts.line_ov_slow.trip-HYSTERESIS;
	FLT_TMR_CLEARING(((fdbk_avg[VLA]>trip_level)||(fdbk_avg[VLB]>trip_level)||(fdbk_avg[VLC]>trip_level)),\
				(fdbk_avg[VLA]<clear_level&&fdbk_avg[VLB]<clear_level&&fdbk_avg[VLC]<clear_level),\
				flts.line_ov_slow,LINE_OV_FLT_SLOW,1)           

/********************************************************************/
/* Disable under voltage faults if pll disabled						*/
/********************************************************************/
	if (!(status_flags&STATUS_PLL_ENABLED))
	{
		flts.line_uv_fast.timer=flts.line_uv_slow.timer=0;
		flts.overfreq.timer=flts.underfreq.timer=flts.underfreq_inst.timer=0;
		flts.dcin_uv.timer=flts.dc_uv.timer=0;
		faults[1]&=~UV_MASK;
	}
	else
	{

/********************************************************************/
/* DC input Under Voltage */
/********************************************************************/
		FLT_TMR_LATCHING(((fdbk_avg[VDCIN]<flts.dcin_uv.trip)&&(enable_uv_tests)),\
							flts.dcin_uv,DCIN_UV_FLT,SET_FLT_STS)

/********************************************************************/
/* DC link instantaneous under voltage 								*/
/********************************************************************/
		FLT_TMR_LATCHING((fdbk_pu[VDC]<flts.dc_uv_inst.trip)&&(status_flags&STATUS_RUN)&&\
			(enable_uv_tests),flts.dc_uv_inst,DC_UV_INST_FLT,SET_FLT_STS)

/********************************************************************/
/* DC link Under Voltage */
/********************************************************************/
		FLT_TMR_LATCHING(((status_flags&STATUS_RUN)&&(fdbk_avg[VDC]<flts.dc_uv.trip)),flts.dc_uv,DC_UV_FLT,SET_FLT_STS)

/********************************************************************/
/*	Line undervoltage fault fast		 							*/
/********************************************************************/
		trip_level=flts.line_uv_fast.trip;
		clear_level=flts.line_uv_fast.trip+HYSTERESIS;
		FLT_TMR_CLEARING(((fdbk_avg[VLA]<trip_level)||(fdbk_avg[VLB]<trip_level)||(fdbk_avg[VLC]<trip_level)),\
				((fdbk_avg[VLA]>clear_level)&&(fdbk_avg[VLB]>clear_level)&&(fdbk_avg[VLC]>clear_level)),\
				flts.line_uv_fast,LINE_UV_FLT_FAST,SET_FLT_STS)

/********************************************************************/
/*	Line undervoltage fault slow		 							*/
/********************************************************************/
		trip_level=flts.line_uv_slow.trip;
		clear_level=flts.line_uv_slow.trip+HYSTERESIS;
		FLT_TMR_FAST_CLEARING(((fdbk_avg[VLA]<trip_level)||(fdbk_avg[VLB]<trip_level)||(fdbk_avg[VLC]<trip_level)),\
				((fdbk_avg[VLA]>clear_level)&&(fdbk_avg[VLB]>clear_level)&&(fdbk_avg[VLC]>clear_level)),\
				flts.line_uv_slow,LINE_UV_FLT_SLOW,SET_FLT_STS)

/********************************************************************/
/* Line over frequency fault										*/
/********************************************************************/
		FLT_TMR_CLEARING(((status_flags&STATUS_PLL_ENABLED)&&(fdbk_avg[FRQERR]>flts.overfreq.trip)),\
					fdbk_avg[FRQERR]<50,flts.overfreq,OVERFREQ_FLT,SET_FLT_STS)

/********************************************************************/
/* Line under frequency fault fast									*/
/********************************************************************/
		FLT_TMR_CLEARING((fdbk_avg[FRQERR]<flts.underfreq_inst.trip),(fdbk_avg[FRQERR]>-70),\
		flts.underfreq_inst,UNDERFREQ_FLT_FAST,SET_FLT_STS)

	}	

	if(!(ADV_CON_IGN_FANFLT & parameters.adv_control_configuration)) {

		if(parameters.adjustable_spd_fan) {

			for(i=0;i<2;i++) {

				if(FAULT_BIT(fan_fault_bit[i])) {
					if(fan_timer[i]-- && !fan_timer[i] && fan_reset_count[i]++ < 3) {
						CLEAR_FAULT(fan_fault_bit[i])
					}
				}
				if(fan_spd_cmd_out[i] > 150) {
					utemp =((unsigned)ext_inputs & fan_tach_bit_mask[i]);
					if(fan_tach_bit[i] ^ temp) {
						fan_timer[i] = FAN_TIMER_START_VALUE;
					}
					else if(fan_timer[i] > 0) {
						fan_timer[i]--;
					}
					else {
						SET_FAULT(fan_fault_bit[i])
						fan_timer[i] = FAN_TIMER_START_VALUE;
					}
					fan_tach_bit[i] = utemp;
				}
				else {
					fan_timer[i] = FAN_TIMER_START_VALUE;
				}
			}
		}
	}

/********************************************************************/
/* Check slow faults at 10 ms intervals 							*/
/********************************************************************/
	switch (timer_1ms)
	{
		
/********************************************************************/
/*	External ground fault and door switch fault						*/
/********************************************************************/
	case(0):
	if (ext_inputs&ESTOP_INPUT)
	{										// estop pulled out
	// PFP Test shutdown trigger
	// CLEAR_FAULT(ESTOP_FLT)
		
		/* digital fuse fault only on grounded systems */
		if (parameters.gnd_config!=FLOATING_GND)	{

			if ((ext_inputs & GNDFLT_INPUT) && (powerup_timer_10ms >= POWERUP_DELAY_10MS)) {
				SET_FAULT(DC_GNDFLT)
			}
		}

		if (ext_inputs & DOOR_SW_INPUT)
		{									// door open
			if (door_switch_timer_10ms < POWERUP_DELAY_10MS)
				door_switch_timer_10ms++;
			else
				SET_FAULT(DOOR_SW_FLT)
		}
		else if (door_switch_timer_10ms > 0)
			door_switch_timer_10ms--;
	}
	else
	{										// estop pushed in
		SET_FAULT(ESTOP_FLT)
	}
	break;

/********************************************************************/
/* hardware faults 													*/
/********************************************************************/
	case(1):
	temp=0;
	if (powerup_timer_10ms < POWERUP_DELAY_10MS)
		powerup_timer_10ms++;
	else
	{
		faults[3]|=((~fpga_inputs)&PS_FLT_INPUTS)|(fpga_inputs&WATCHDOG_FLT_INPUT);
		if (ext_inputs & SURGE_SUPP_INPUT)		SET_FAULT(SURGE_SUPP_FLT)
		if (ext_inputs & REAC_TEMP_FLT_INPUT)	SET_FAULT(REAC_TEMP_FLT)
		if (ext_inputs & XFMR_TEMP_FLT_INPUT)	SET_FAULT(XFMR_TEMP_FLT)
		if (ext_inputs & FUSE_FLT_INPUT1)		SET_FAULT(INV_FUSE_FLT1)
		if (ext_inputs & AC_SURGE_SUPP_INPUT)	SET_FAULT(SURGE_SUPP_FLT)
		if (ext_inputs & INV_TEMP_FLT_INPUT1)	SET_FAULT(INV_TEMP_FLT1)
		if (fpga_inputs & HWOC_FLT_INPUT1)		SET_FAULT(INV_HW_OC_FLT1)
		if (fpga_inputs & HWOC_FLT_INPUT2)		SET_FAULT(INV_HW_OC_FLT2)
		temp=ENA_PWMFB_A_OUTPUT|FAN_PWM_DISABLE_2;
		clear_level=PWM_FLT_MASK_1;

		if (parameters.inv_bridges==2)
		{
			if (global_flags & FUSE_FLT_INPUT2)			SET_FAULT(INV_FUSE_FLT2)
			if (global_flags & INV_TEMP_FLT_INPUT2)		SET_FAULT(INV_TEMP_FLT2)
			if (global_flags & REAC_TEMP_FLT_INPUT2)	SET_FAULT(REAC_TEMP_FLT)
			temp=(ENA_PWMFB_A_OUTPUT|ENA_PWMFB_B_OUTPUT);
			clear_level|=(PWM_FLT_MASK_1<<3);
		}
	}
	if (status_flags & (STATUS_GATE_TEST|STATUS_SIMULATE))
	{
		temp=clear_level=0;								// disable all fdbk
		if (test_mode==GATING_IO_TEST)					// for gating i/o test only
			temp=(ENA_PWMFB_A_OUTPUT|ENA_PWMFB_B_OUTPUT);	// enable all fdbk
	}
	fpga_outputs=(fpga_outputs&~(ENA_PWMFB_A_OUTPUT|ENA_PWMFB_B_OUTPUT))|temp;
	pwm_flt_inputs=PWM_FLT_INPUT;
	faults[4]=(faults[4]&~clear_level)|(pwm_flt_inputs & clear_level);
	break;

/********************************************************************/
/* A/D converter fault												*/
/********************************************************************/
	case(2):
	if ((abs(fdbk_pu[CAL1]-2048) > ADC_MAX_ERROR_MV) ||
		(abs(fdbk_pu[CAL2]-1048) > ADC_MAX_ERROR_MV) ||
		(abs(fdbk_pu[CAL3]-2048) > ADC_MAX_ERROR_MV) ||
		(abs(fdbk_pu[CAL4]-1048) > ADC_MAX_ERROR_MV))
	SET_FAULT(ADC_FAULT)
	else
	{
		temp=(fdbk_display[CAL4]+fdbk_display[CAL3]+fdbk_display[CAL2]+
				fdbk_display[CAL1]-(2048+1048+2048+1048))>>2;
		LIMIT_MAX_MIN(temp,ADC_MAX_ERROR_MV,-ADC_MAX_ERROR_MV)
		temp=(temp+a2d_offset_mv)>>1;
		a2d_offset_mv=temp;
		a2d_offset_counts=temp*(32760/3000);
		a2d_gain_q12=(fdbk_display[CAL1]-fdbk_display[CAL2]+fdbk_display[CAL3]-fdbk_display[CAL4])/2;
	}
	break;

/********************************************************************/
/*	Fan fault				 	 									*/
/********************************************************************/
	case(3):

#if 0
	if(!(ADV_CON_IGN_FANFLT & parameters.adv_control_configuration)) {

		if(parameters.adjustable_spd_fan) {

			for(i=0;i<2;i++) {

				if(FAULT_BIT(fan_fault_bit[i])) {
					if(fan_timer[i]-- && !fan_timer[i] && fan_reset_count[i]++ < 3) {
						CLEAR_FAULT(fan_fault_bit[i])
					}
				}
				else if(fan_spd_cmd_out[i] > 5) {
					if(fan_tach_bit[i] ^ (temp =(ext_inputs & fan_tach_bit_mask[i]))) {
						fan_timer[i] = (int)(10.0*100);
					}
					else if(fan_timer[i]) {
						fan_timer[i]--;
					}
					else {
						SET_FAULT(fan_fault_bit[i])
						fan_timer[i] = (int)(10.0*100);
					}
					fan_tach_bit[i] = temp;
				}
				else {
					fan_timer[i] = (int)(10.0*100);
				}
			}
		}
	}
#endif

	break;

/********************************************************************/
/*	Line underfrequency fault			 	 						*/
/********************************************************************/
	case(4):
	FLT_TMR_FAST_CLEARING(((status_flags&STATUS_PLL_ENABLED)&&(fdbk_avg[FRQERR]<flts.underfreq.trip)),\
				(fdbk_avg[FRQERR]>-70),flts.underfreq,UNDERFREQ_FLT,SET_FLT_STS)

/********************************************************************/
/*	Line unbalance faults				 	 						*/
/********************************************************************/
	FLT_TMR_CLEARING((fdbk_avg[VLUB]>flts.v_unbalance.trip),(fdbk_avg[VLUB]<flts.v_unbalance.trip),flts.v_unbalance,LINE_VUB_FLT,SET_FLT_STS)
	FLT_TMR_CLEARING((fdbk_avg[ILUB]>flts.i_unbalance.trip),(fdbk_avg[ILUB]<flts.i_unbalance.trip),flts.i_unbalance,LINE_CUB_FLT,SET_FLT_STS)

	break;

/********************************************************************/
/*	ground impedance fault 											*/
/********************************************************************/
	case(5):

	break;

/********************************************************************/
/*	Temperature feedback fault			 	 						*/
/********************************************************************/
	case(6):
	fbk_mv=temp_fbk_mv[temp_index];

/********************************************************************/
/*	Convert one temperature feedback from millivolts to degrees C	*/
/*	5V reference 909 ohm resistor									*/
/*																	*/
/*	use 4th order curve fit below 1000 mv							*/
/*	temp_fbk=(((((-99.478*fbk_mv/1000)+326.68)*fbk_mv/1000)			*/
/*			-392.24)*fbk_mv/1000)+242.62)*fbk_mv/1000 -27.695		*/
/*	=(((((-28001*fbk_mv/4096)+22449)*fbk_mv/4096)-6581)*fbk_mv/4096)+994)*fbk_mv/4096 -28	*/
/*																	*/
/*	use 1st order curve fit above 1000 mv							*/
/*	temp_fbk=26.381*fbk_mv/1000 + 24.496							*/
/*			=108*fbk_mv/4096 + 24									*/
/********************************************************************/
	if (fbk_mv > 0)							// temp_fbk in millivolts
	{
		if (fbk_mv < 1000)
		{										// low temperatures
			temp=MULQ(12,-28001,fbk_mv);		// 4th order curve fit
			temp=MULQ(12,(temp+22449),fbk_mv);
			temp=MULQ(12,(temp-6581),fbk_mv);
			temp=MULQ(12,(temp+994),fbk_mv);
			temp=temp-28;
		}
		else
		{										// high temperatures
			temp=MULQ(12,108,fbk_mv);			// 1st order curve fit
			temp=temp+24;
		}
		if (temp < TEMP_FBK_MIN)
			temp=TEMP_FBK_MIN;

			temp_fbk[temp_index]=temp;
	}

/********************************************************************/
/* Check one high temperature fault									*/
/********************************************************************/
	temp=temp_fbk[temp_index];
	if ((temp_index > 1) && (temp > heatsink_temp_old))
		heatsink_temp_old=temp;
	FLT_TMR_LATCHING((temp>flts.high_temp[temp_index].trip),flts.high_temp[temp_index],(HIGH_TEMP_FLT_1+temp_index),1)
	temp=temp_index+1;
	if (parameters.inv_bridges==2)
	{
		if (temp==4)
		{						// 1st fdbk in 2nd bridge
			heatsink_temp_max[0]=heatsink_temp_old;
			heatsink_temp_old=TEMP_FBK_MIN;
		}
		else if (temp >= number_temp_fbk)
		{
			heatsink_temp_max[1]=heatsink_temp_old;
			heatsink_temp_old=TEMP_FBK_MIN;
			temp=0;
		}
	}
	else
	{
		if (temp >= number_temp_fbk)
		{
			heatsink_temp_max[0]=heatsink_temp_old;
			heatsink_temp_old=TEMP_FBK_MIN;
			temp=0;
		}
	}
	temp_index=temp;
	break;

/********************************************************************/
/*	Input contactor faults 											*/
/********************************************************************/
	case(7):

	if (ext_outputs & DC_CONTACTOR_OUTPUT)
	{											// should be closed
		if (!(DISCONNECT_CLOSED))
			SET_FAULT(DISCONNECT_OPEN_FLT)
		if ((ext_inputs & DC_CONTACTOR_INPUT)==0)
			dc_contactor_timer_10ms=0;
		else if (++dc_contactor_timer_10ms >= (int)(CONTACTOR_DELAY_SEC*100.0))
		{
			dc_contactor_timer_10ms=0;
			SET_FAULT(INPUT_OPEN_FLT)
		}

	}
	else
	{											// should be open
		if ((ext_inputs & DC_CONTACTOR_INPUT)==0)
		{										// closed
			if (++dc_contactor_timer_10ms >= (int)(CONTACTOR_DELAY_SEC*100.0))
			{
				dc_contactor_timer_10ms=0;
				SET_FAULT(INPUT_CLOSED_FLT)
			}
		}
		else									// open
			dc_contactor_timer_10ms=0;
	}

/********************************************************************/
/*	Output contactor faults 										*/
/********************************************************************/
	case(8):

	/* confirm closed state */
	if(ext_outputs & ac_contactor_output) {

		if ((ext_inputs & ac_contactor_input)==0) {
			ac_contactor_timer_10ms=0;
		}
		else if (++ac_contactor_timer_10ms >= (int)(CONTACTOR_DELAY_SEC*100.0)) {
			ac_contactor_timer_10ms=0;
			SET_FAULT(OUTPUT_OPEN_FLT)
		}
	}

	/* confirm opened state */
	else {
		if ((ext_inputs & ac_contactor_input)==0) {
			if (++ac_contactor_timer_10ms >= (int)(CONTACTOR_DELAY_SEC*100.0)) {
				SET_FAULT(OUTPUT_CLOSED_FLT)
			}
		}
		else {
			ac_contactor_timer_10ms=0;
		}
	}


	break;
		
/********************************************************************/
/*	Determine line voltage status									*/
/********************************************************************/
	case(9):
	temp=parameters.reconnect_delay_6sec*(6000/10);
	if (line_voltage_timer_10ms > temp)
		line_voltage_timer_10ms=temp; 
	if ((status_flags & STATUS_PLL_ENABLED) &&
		((faults[1] & RECONNECT_MASK)==0) &&
		!(status_flags & STATUS_GATE_TEST))
	{ 
		if(line_voltage_timer_10ms > 0 )
			line_voltage_timer_10ms--;
		else
			status_flags|=(STATUS_LINE_OK|STATUS_LINE_LINKED);
    }
    else
    {
        status_flags&=~(STATUS_LINE_OK|STATUS_READY);
		line_voltage_timer_10ms=temp; 
    }
	line_voltage_timer_sec=MULQ(12,(line_voltage_timer_10ms+20),((4096+50)/100));
	break;
    }

/********************************************************************/
/*	Set master fault flags to indicate faults in hidden words		*/
/********************************************************************/

	if ((faults[2]&~DPCB_FLT_MASK)!=0)
		SET_FAULT(DPCB_FAULT)
	else
		CLEAR_FAULT(DPCB_FAULT)
	if (faults[3]!=0)
		SET_FAULT(HARDWARE_FAULT)
	if (faults[4]!=0 || faults[6]!=0)
		SET_FAULT(INVERTER_FAULT)
	if (faults[5]!=0)
		SET_FAULT(TEMP_FAULT)

}
