/********************************************************************/
/********************************************************************/
/*																	*/
/*							Slow Tasks Module						*/
/*																	*/
/*		each function in this module called at 100 ms intervals		*/
/********************************************************************/
/********************************************************************/

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
2008-06-26	if only one bridge set second fan speed command to zero
2008-08-08	delay two minutes before clearing low power faults
2008-08-26	use fdbk_display[IDC] instead of fdbk_pu in low current fault
			initialize vdc_in_timer_100ms when low power fault occurs
2008-09-09	set and clear stop fault according to state of run_enable
2008-09-17	add fan_speed_min_pu
2008-10-14	delete two minute delay before clearing low power faults
2008-10-16	clear low power timer when not running
1 MW version
2008-12-03	add ground fault to ready conditions
2008-12-08	add array paralleling contactor control for 1 MW unit
2008-12-09	eliminate rounding in conversion from pu to kW
2009-01-07	delete ratings change fault
2009-02-04	rewrite determination of ready flag for 1 MW
2009-02-13	add special scaling for 1 MW ground impedance
2009-02-20	clear paralleling contactor fault when stopped
2009-02-24	split into 9 separate functions called by pwm_isr
2009-03-09	use short dc input voltage delay in test mode
2009-03-10	invert fan control outputs for 1 MW
2009-03-20	add k_i_leakage_q8 parameter to leakage current calculation
2009-04-14	calculate ground impedance even when not running
			add dc_ground_amps variable for display
			filter calculated ground impedance
2009-05-13	add ac breaker control
2009-06-16	check validity of kwh date data
2009-07-03	calculate 7 day total if sufficient data available
Solstice version
2009-07-20	delete 1 MW code
			delete paralleling contactor control used for 1 MW
			delete ground leakage current calculation
			add on/off input
2009-08-04	add fan control output for adjustable speed fan
2009-08-14	delete ac breaker control
*/
#include "literals.h"
#include "2812reg.h"
#include "io_def.h"
#include "struct.h"
#include "faults.h"

#define SLOW_TASK_RATE	10		// per second

/********************************************************************/
/*		Global variables											*/
/********************************************************************/
struct SAVED_VARS saved_variables;

struct AC_PWR output_power;
struct ABCM line_volts,line_amps,inverter_amps;
struct ABC inverter2_amps;
int fan_spd_cmd_out[2],vdc_in_timer_6sec;
int string_amps_avg,string_kwh_avg;
int input_kw,gnd_impedance,dc_ground_amps;
int inverter_volts_bc,global_flags;
int dc_link_amps,dc_link_volts,dc_input_volts;
int wh_total,kwh_yesterday,kwh_data,kwh_month_day,kwh_index;
int kwh_total_7days,kwh_avg_7days,kwh_total_30days,kwh_avg_30days;
int total_dc_power,gnd_impedance_norm,dc_bus_imbalance_ma_x10;
// constants from parameter_calc
int pu_to_line_volts_q12,pu_to_line_amps_q12,pu_to_inv_volts_q12,pu_to_inv_amps_q12;
int pu_to_dc_volts_q12,pu_to_dc_amps_q12,pu_to_kva_q12;
int kp_fan_q16,ki_fan_q16,thermal_limit,i_cmd_max,kp_overtemp_q16,ki_overtemp_q16;
int fuse_volts;
int rs1,rs2;  //rs added debug variables

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
long cooling_cmd[2],i_cmd_max_q12,kwh_total,string_amps_total;
int amp_sec_index,low_power_timer_100ms,low_idc_timer_100ms,vdc_in_timer_100ms;
int fan_off_temp,fan_on_temp,fan_spd_cmd_in,fan_speed_min_pu;
int heatsink_temp_error[2],high_temp_error;
int auto_reset_timer_sec,auto_reset_lockout_timer_sec,auto_reset_attempts;
unsigned string_watt_sec[STRING_FDBK_SIZE];
int paralleling_timer_100ms;
int fan_spd_cmd_out[2];
int impedance_timer;
/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern int faults[FAULT_WORDS];
extern int ext_inputs,ext_outputs,fpga_outputs,test_mode;
extern int operating_state,status_flags,pc_cmd,kw_rated_param,v_inv_rated_param;
extern int v_rated,rated_Hz,dc_volts_to_pu_q12,rated_dc_volts;
extern int number_strings,real_power_cmd_pu;
extern int fdbk_pu[FDBK_SIZE],fdbk_avg[FDBK_SIZE],fdbk_display[FDBK_SIZE];
extern int string_amps[STRING_FDBK_SIZE];
extern int heatsink_temp_max[2];
extern int v_fdbk_config,inverter_type,ac_contactor_input;
extern int temp_fbk[8];
extern int dc_contactor_flag;
extern struct DQ_LONG i_ref_q16;
extern struct PARAMS parameters;
extern struct RTC_PACKED bcd_time;
extern int dv[4]; //rs added
extern int v0,v1,v2;
/********************************************************************/
/********************************************************************/
/*	Check dc input voltage function									*/
/********************************************************************/


#pragma CODE_SECTION(check_dc_input_voltage,".l1_code")

void check_dc_input_voltage(void)
{
register int temp,vdcin_delay_100ms;

	vdcin_delay_100ms=6000/100;
	if (test_mode==0)
		vdcin_delay_100ms=parameters.vdcin_delay_6sec*(6000/100);
    if (vdc_in_timer_100ms > vdcin_delay_100ms)
     	vdc_in_timer_100ms=vdcin_delay_100ms;
    if (FAULT_BIT(DCIN_UV_FLT)||FAULT_BIT(LOW_POWER_FLT)||FAULT_BIT(LOW_CURRENT_FLT))
	{
     	vdc_in_timer_100ms=vdcin_delay_100ms;
		status_flags&=~STATUS_VDC_OK;
	}
	if ((temp=((dc_input_volts-parameters.vdcin_threshold)>>3)) > 0)
	{										// above threshold
		if (temp > 3)
			temp=3;
		if ((vdc_in_timer_100ms=vdc_in_timer_100ms-temp) <= 0)
		{
			vdc_in_timer_100ms=0;
			status_flags|=STATUS_VDC_OK;
		}
	}
	else if ((status_flags & STATUS_RUN)==0)
	{										// below threshold
		if ((vdc_in_timer_100ms=vdc_in_timer_100ms-temp) > vdcin_delay_100ms)
		{
	     	vdc_in_timer_100ms=vdcin_delay_100ms;
			status_flags&=~STATUS_VDC_OK;
		}
	}
	vdc_in_timer_6sec=MULQ(16,(vdc_in_timer_100ms+30),(int)(65536.0*100.0/6000.0+0.5));
}

/********************************************************************/
/********************************************************************/
/*		Low power fault function									*/
/********************************************************************/
#pragma CODE_SECTION(low_power_fault,".l1_code")

void low_power_fault(void)
{
#define	COUNTS_6SEC		(6*SLOW_TASK_RATE)

register int temp;

/********************************************************************/
/*		Low power fault												*/
/********************************************************************/
	temp=GENERATING_OUTPUT;
	if(operating_state==STATE_RUN)
	{
		if( (parameters.power_control_mode!=PCM_DC_POWER) //no need to check the low power or low current faults during Rectifier mode
			&& (!(ADV_CON_DIS_LOW_POWER & parameters.adv_control_configuration)))
		{
			if (output_power.kw > parameters.low_power_trip_pct*PERCENT_TO_PU)
			{
				low_power_timer_100ms=0;
				ext_outputs|=temp;
				status_flags|=STATUS_GENERATING;
			}								
			else
			{
				ext_outputs&=~temp;
				status_flags&=~STATUS_GENERATING;
				if (++low_power_timer_100ms > parameters.low_power_delay_6sec*COUNTS_6SEC)
				{
					SET_FAULT(LOW_POWER_FLT)
				}
			}

		/********************************************************************/
		/*		Low current	fault											*/
		/********************************************************************/
			if (fdbk_display[IDC] > parameters.low_power_trip_pct*PERCENT_TO_PU)
				low_idc_timer_100ms=0;
			else if(++low_idc_timer_100ms>parameters.low_power_delay_6sec*COUNTS_6SEC)
			{
				SET_FAULT(LOW_CURRENT_FLT)
			}
		}
/********************************************************************/
/*		Auto reset													*/
/********************************************************************/
    	if(auto_reset_attempts > 0)
    	{
			if(++auto_reset_lockout_timer_sec > parameters.auto_reset_lockout_6sec*COUNTS_6SEC)
				auto_reset_lockout_timer_sec=auto_reset_attempts=0;
    	}
	}

	else
	{
		ext_outputs&=~temp;
		status_flags&=~STATUS_GENERATING;
		low_power_timer_100ms=low_idc_timer_100ms=auto_reset_lockout_timer_sec=0;
		CLEAR_FAULT(LOW_POWER_FLT)
		CLEAR_FAULT(LOW_CURRENT_FLT)
	}


	if((status_flags & STATUS_FAULT)==0)
		auto_reset_timer_sec=0;
	else
	{
		if(auto_reset_attempts < parameters.auto_reset_max_attempts)
		{
			if(++auto_reset_timer_sec>parameters.auto_reset_interval_6sec*COUNTS_6SEC)
			{
				auto_reset_timer_sec=0;
				auto_reset_attempts++;
				pc_cmd=CMD_RESET;
			}
		}
		if(parameters.testVar[0]==52)
		dv[3]++;
	}

	if(parameters.testVar[0]==52){
	dv[0]=auto_reset_lockout_timer_sec;
	dv[1]=auto_reset_attempts;
	dv[2]=auto_reset_timer_sec;
	v0=auto_reset_timer_sec >> parameters.testVar[1];
	v1=auto_reset_lockout_timer_sec>> parameters.testVar[1];
	v2=auto_reset_attempts>>parameters.testVar[1];
	}

}

/********************************************************************/
/********************************************************************/
/*		Fan speed control function									*/
/********************************************************************/
#pragma CODE_SECTION(fan_speed_control,".l1_code")

void fan_speed_control(void)
{
	register int temp;
	register long temp_long;

#define FAN_CMD(x)		(kw_rated_param==500 ? (x) : (PER_UNIT - x))

/********************************************************************/
/*		Fixed speed fan control (cycles off and on)					*/
/********************************************************************/
	if (parameters.adjustable_spd_fan==0)
	{
		if (ext_outputs & FAN_OUTPUT)
		{
			fan_spd_cmd_out[0]=fan_spd_cmd_out[1]=PER_UNIT;
			if (heatsink_temp_max[0] < fan_off_temp)
				ext_outputs&=~FAN_OUTPUT;
		}
		else
		{
			fan_spd_cmd_out[0]=fan_spd_cmd_out[1]=PER_UNIT;
			if (heatsink_temp_max[0] > fan_on_temp)
				ext_outputs|=FAN_OUTPUT;
		}
	}

/********************************************************************/
/*		Adjustable speed fan control								*/
/********************************************************************/
	else
	{
		ext_outputs|=(FAN_PWM_OUTPUT_1|FAN_PWM_OUTPUT_2);

		if (fan_spd_cmd_in > 0)			// open loop fan control
			fan_spd_cmd_out[0]=fan_spd_cmd_out[1]=fan_spd_cmd_in;
		else
		{								// closed loop fan control
			static long iTerm0 = 0;
			static long iTerm1 = 0;

			// fan control is PU; fan speed limit is 100; 10*10 = 100 */
			if(heatsink_temp_max[0] < parameters.fan_extended_temp_threshold) {
				temp_long = (long)(parameters.fan_normal_speed_limit*parameters.fan_normal_speed_limit);
				temp_long *= 100;
			}
			else {
				temp_long = (long)(parameters.fan_extended_speed_limit*parameters.fan_extended_speed_limit);
				temp_long *= 100;
			}

			PI_REG_CS(heatsink_temp_max[0],fan_on_temp,heatsink_temp_error[0],iTerm0,\
			cooling_cmd[0],kp_fan_q16,ki_fan_q16,temp_long,0)
		    
			fan_spd_cmd_out[0]=long_sqrt(cooling_cmd[0]);


			if (parameters.inv_bridges==2) {

				if(heatsink_temp_max[1] < parameters.fan_extended_temp_threshold) {
					temp_long = (long)(parameters.fan_normal_speed_limit*parameters.fan_normal_speed_limit);
					temp_long *= 100;
				}
				else {
					temp_long = (long)(parameters.fan_extended_speed_limit*parameters.fan_extended_speed_limit);
					temp_long *= 100;
				}
							
				PI_REG_CS(heatsink_temp_max[1],fan_on_temp,heatsink_temp_error[1],iTerm1,\
				cooling_cmd[1],kp_fan_q16,ki_fan_q16,temp_long,0) 
				
				fan_spd_cmd_out[1]=long_sqrt(cooling_cmd[1]);
			}
			else {
				fan_spd_cmd_out[1]=cooling_cmd[1]=0;
			}
		}
	}

	FAN_SPD_OUTPUT   = FAN_CMD(fan_spd_cmd_out[0]);
	FAN_SPD_OUTPUT_2 = FAN_CMD(fan_spd_cmd_out[1]);

	if ((fan_spd_cmd_out[0]==0) && (fan_spd_cmd_out[1]==0))
   		ext_outputs&=~FAN_OUTPUT;
	else
		ext_outputs|=FAN_OUTPUT;

}

/********************************************************************/
/********************************************************************/
/*		Overtemperature regulator function							*/
/********************************************************************/
#pragma CODE_SECTION(overtemp_regulator,".l1_code")

void overtemp_regulator(void)
{
register int temp,temp2;
register long temp_long;

/********************************************************************/
/*		Overtemperature regulator									*/
/********************************************************************/
	temp_long=(long)i_cmd_max<<12;
	if ((fan_spd_cmd_out[0]>(int)(0.9*PER_UNIT_F))||
		(fan_spd_cmd_out[1]>(int)(0.9*PER_UNIT_F)))
	{
		temp=heatsink_temp_max[0];	// determine highest heatsink temp
		temp2=heatsink_temp_max[1];
		if (temp2 > temp)
			temp=temp2;
		// activate regulator when temp is 5 degrees below trip level
		temp2=parameters.high_temp_trip[2]-5;
		PI_REG(temp2,temp,high_temp_error,i_cmd_max_q12,kp_overtemp_q16,ki_overtemp_q16,temp_long,0)
	}
	else
		i_cmd_max_q12=temp_long;
	thermal_limit=i_cmd_max_q12>>12;
}

/********************************************************************/
/********************************************************************/
/*		Convert to display units function							*/
/********************************************************************/
#pragma CODE_SECTION(convert_to_display_units,".l1_code")


void convert_to_display_units(void)
{
register int temp,*fdbk_disp_pntr;

/********************************************************************/
/*		Ready flags													*/
/********************************************************************/
	if ((status_flags & (STATUS_GATE_TEST|STATUS_OPEN_CCT_TEST))==0)
	{
		if(!parameters.run_enable)
		{
			if((operating_state!=STATE_RUN) || ((i_ref_q16.d==0) && (i_ref_q16.q==0))) {
				SET_FAULT(STOP_FLT)
				status_flags&=~STATUS_READY;
			}
		}
		else
		{
			CLEAR_FAULT(STOP_FLT)
			if(
				((status_flags & (STATUS_FAULT|STATUS_LINE_OK|STATUS_VDC_OK))!=(STATUS_LINE_OK|STATUS_VDC_OK)) ||
				((ext_inputs & ESTOP_INPUT)==0) ||
				(!(ext_inputs & ON_OFF_INPUT) && (i_ref_q16.d==0) && (i_ref_q16.q==0))
			)
				status_flags&=~STATUS_READY;
			else
				status_flags|=STATUS_READY;
		}
	}

/********************************************************************/
/*	Convert voltage and current to engineering units for display	*/
/********************************************************************/
	fdbk_disp_pntr	=&fdbk_display[VLA];
	line_volts.a	=MULQ(12,(*fdbk_disp_pntr++),pu_to_line_volts_q12);
	line_volts.b	=MULQ(12,(*fdbk_disp_pntr++),pu_to_line_volts_q12);
	line_volts.c	=MULQ(12,(*fdbk_disp_pntr++),pu_to_line_volts_q12);
	inverter_volts_bc=MULQ(12,(*fdbk_disp_pntr++),pu_to_inv_volts_q12);
	line_amps.a		=MULQ(12,(*fdbk_disp_pntr++),pu_to_line_amps_q12);
	line_amps.b		=MULQ(12,(*fdbk_disp_pntr++),pu_to_line_amps_q12);
	line_amps.c		=MULQ(12,(*fdbk_disp_pntr++),pu_to_line_amps_q12);
	fdbk_disp_pntr	=&fdbk_display[IIA];
	inverter_amps.a	=MULQ(12,(*fdbk_disp_pntr++),pu_to_inv_amps_q12);
	inverter_amps.b	=MULQ(12,(*fdbk_disp_pntr++),pu_to_inv_amps_q12);
	inverter_amps.c	=MULQ(12,(*fdbk_disp_pntr++),pu_to_inv_amps_q12);
	temp			=inverter_amps.a+inverter_amps.b+inverter_amps.c;
	if (parameters.inv_bridges==2)
	{
		inverter2_amps.a=MULQ(12,(*fdbk_disp_pntr++),pu_to_inv_amps_q12);
		inverter2_amps.b=MULQ(12,(*fdbk_disp_pntr++),pu_to_inv_amps_q12);
		inverter2_amps.c=MULQ(12,(*fdbk_disp_pntr++),pu_to_inv_amps_q12);
		temp=temp+inverter2_amps.a+inverter2_amps.b+inverter2_amps.c;
	}
	inverter_amps.m	=MULQ(16,temp,ONE_THIRD_Q16);	
	line_volts.m	=MULQ(16,(line_volts.a+line_volts.b+line_volts.c),ONE_THIRD_Q16);
	line_amps.m		=MULQ(16,(line_amps.a+line_amps.b+line_amps.c),ONE_THIRD_Q16);

	fdbk_disp_pntr	=&fdbk_display[VDCIN];
	dc_input_volts	=MULQ(12,(*fdbk_disp_pntr++),pu_to_dc_volts_q12);
	dc_link_volts	=MULQ(12,(*fdbk_disp_pntr++),pu_to_dc_volts_q12);

	fdbk_disp_pntr=&fdbk_display[IDC];
	dc_link_amps	=MULQ(12,(*fdbk_disp_pntr),pu_to_dc_amps_q12);

	fuse_volts		=MULQ(12,fdbk_display[FUSEV],pu_to_dc_volts_q12);


/********************************************************************/
/*	Convert input and output power to display units of 0.1 kw		*/
/********************************************************************/
	fdbk_disp_pntr=&fdbk_display[PDC];
	temp				=pu_to_kva_q12*10;
	input_kw			=MULQ_RND_UP(12,(*fdbk_disp_pntr++),temp);
	output_power.kw		=MULQ_RND_UP(12,(*fdbk_disp_pntr++),temp);
	output_power.kvar	=MULQ_RND_UP(12,(*fdbk_disp_pntr++),temp);
	output_power.kva	=MULQ_RND_UP(12,(*fdbk_disp_pntr++),temp);

/********************************************************************/
/*	Calculate line power factor										*/
/********************************************************************/
	if (output_power.kva > (INVERSE_MIN>>4))
	{	// power factor = (KW/KVA)*1000
		temp=div_q12(abs(fdbk_display[KW]),fdbk_display[KVA]);
		temp=MULQ(12,temp,1000);
		if (temp > 1000)
			temp=1000;
		output_power.pf=temp;
	}
	else
		output_power.pf=0;
}

/********************************************************************/
/********************************************************************/
/*		Calculate ground impedance function							*/
/********************************************************************/


#pragma CODE_SECTION(calculate_ground_impedance,".l1_code")

void calculate_ground_impedance(void)
{
	register int temp;
#if 0
	register unsigned long temp_ulong,ul0,ul1,ul2;
#endif

	if (parameters.gnd_config==FLOATING_GND)	{

#if 0
		temp = fdbk_avg[BENDER];	// millivolts

		if(temp > 2900) {
			ul0 = ul1 = ul2 = 0;
		}
		else if(temp > 933) {
			ul0 = (unsigned long)( 608.578*256) << 10;
			ul1 = (unsigned long)( 418.756*256)*temp;
			ul2 = (unsigned long)(  72.673*256)*(((unsigned long)temp*temp)>>10);
		}
		else {
			ul0 = (unsigned long)(2413.486*256) << 10;
			ul1 = (unsigned long)(4765.045*256)*temp;
			ul2 = (unsigned long)(2690.800*256)*(((unsigned long)temp*temp)>>10);
		}

		temp_ulong = (ul2 - ul1) + ul0;

		gnd_impedance = (int)(temp_ulong >> 18);
#endif

		dc_ground_amps=0;
		dc_bus_imbalance_ma_x10 = MULQ_RND(12,fdbk_display[DCIMB],((4096*5)/100)); // 1000PU = 5ma
	}

	else
	{
		dc_ground_amps=fdbk_display[GND];
		dc_bus_imbalance_ma_x10 = 0;
	}

/********************************************************************/
/*	Calculate ground impedance										*/
/********************************************************************/
	//	parameters.inverter_type=EU;
	if(parameters.inverter_type==NA) 
	{
		gnd_impedance=2000;
	}

	else //rs commented
	{
			//dc_contactor_flag=parameters.testVar[0];
		//dv[0]=	fdbk_display[DCIMB]; //rs debug
		//fdbk_display[DCIMB]=parameters.testVar[1]; //rs debug
		//dc_input_volts=710;//rs debug
		temp = abs(fdbk_display[DCIMB] );
		if(dc_input_volts>400 && temp>2)
		{
			//impedance_timer=0;
		   // temp=fdbk_display[DCIMB]=4;
			if(fdbk_display[DCIMB]<0)
			{	
				if(dc_contactor_flag==0)
				//if(parameters.testVar[0]==0)
				{
					//dv[1]=impedance_timer; //rs debug
					impedance_timer--;
					if(impedance_timer<=0)
					{
						temp=40*temp-543;
						gnd_impedance=div_q12((dc_input_volts>>1),temp)-100;
						impedance_timer++;
					}
					//impedance_timer2=0;
				}
				else
				{
					//dv[2]=impedance_timer; //rs debug
					//temp=85;
					impedance_timer++;
					if(impedance_timer>40)
					{
						//temp=56*temp-937;
						temp=61*temp-1003;
						gnd_impedance=div_q12((dc_input_volts>>1),temp)-100;
						impedance_timer--;
					}
					//impedance_timer1=0;
				}		
			}
			else
			{
				if(dc_contactor_flag==0)
				//if(parameters.testVar[0]==0)
				{
					//dv[1]=impedance_timer; //rs debug
					impedance_timer--;
					if(impedance_timer<=0)
					{
						temp=38*temp+892;
						gnd_impedance=div_q12((dc_input_volts>>1),temp)-100;
						impedance_timer++;
					}
					//impedance_timer2=0;
				}
				else
				{
					//dv[2]=impedance_timer; //rs debug
					impedance_timer++;
					if(impedance_timer>40)
					{
						temp=61*temp+1251;
						gnd_impedance=div_q12((dc_input_volts>>1),temp)-100;
						impedance_timer--;
					}
					//impedance_timer1=0;
				}		
			}
			if(gnd_impedance<0) 
			gnd_impedance=0;
		}
		else {
			gnd_impedance = 2000;
		}
	
	}

}

/********************************************************************/
/********************************************************************/
/*	Calculate kw hours function										*/
/********************************************************************/
#pragma CODE_SECTION(calculate_kw_hrs,".l1_code")

void calculate_kw_hrs(void)
{
	#define KW_HR_INC 	(long)(60.0*60.0*10.0*SLOW_TASK_RATE)
	#define INVALID_TOTAL			0x80000000L

	register int temp,temp2,index;
	register long kwh_total,*long_pntr;

/********************************************************************/
/*	kWh meter														*/
/********************************************************************/
	kwh_total=saved_variables.kws_total+output_power.kw;
	if (kwh_total > KW_HR_INC)
	{
		kwh_total-=KW_HR_INC;
		if (kwh_total > KW_HR_INC)
			kwh_total=0;
		saved_variables.kwh_today++;
		temp2=saved_variables.kwh_total+1;
		if (temp2 >= 1000)
		{
			temp2-=1000;
			saved_variables.mwh_total++;
		}
		saved_variables.kwh_total=temp2;
	}
	else if (kwh_total < 0)
	{	
		kwh_total+=KW_HR_INC;
		if (kwh_total < 0)
			kwh_total=0;
		saved_variables.kwh_today--;
		temp2=saved_variables.kwh_total-1;
		if (temp2 < 0)
		{
			temp2+=1000;
			temp=saved_variables.mwh_total-1;
			if (temp < 0)
				temp2=temp=0;
			saved_variables.mwh_total=temp;
		}
		saved_variables.kwh_total=temp2;
	}
	saved_variables.kws_total=kwh_total;
	wh_total=MULQ(12,(kwh_total>>2),(int)(4.0*1000.0*4096.0/KW_HR_INC));

/********************************************************************/
/*	Check initialization request									*/
/********************************************************************/
	if ((temp=parameters.initialize_cmd) > 3)
	{
		if ((status_flags&STATUS_RUN)||(fpga_outputs&START_OUTPUT))
			parameters.initialize_cmd=0;
		else if (temp==4)
			global_flags|=SAVE_PARAMETERS;
	}

/********************************************************************/
/*	Clear kw hour totals if requested								*/
/********************************************************************/
	else if (temp==3)
	{
		kwh_total=0;
		long_pntr=(long*)&saved_variables.kwh_per_day[0];
		for (index=0;index<32;index++)
			*long_pntr++=kwh_total;
		saved_variables.kws_total=0;
		saved_variables.kwh_total=saved_variables.mwh_total=0;
		saved_variables.kwh_today=saved_variables.kwh_index=0;
		parameters.initialize_cmd=0;
	}

/********************************************************************/
/*	Save kwh per day data											*/
/********************************************************************/
	index=saved_variables.kwh_index&31;
	saved_variables.kwh_per_day[index].date=saved_variables.today;
	saved_variables.kwh_per_day[index].kwh=saved_variables.kwh_today;

/********************************************************************/
/*	Check for start of new day										*/
/********************************************************************/
	if (bcd_time.month_day!=saved_variables.today)
	{
		saved_variables.today=bcd_time.month_day;
		saved_variables.kwh_today=0;
		saved_variables.kwh_index=index=(index+1)&31;
		index=(index+1)&31;
		saved_variables.kwh_per_day[index].date=0;
		saved_variables.kwh_per_day[index].kwh=0;
	}

/********************************************************************/
/*	Get yesterday's total											*/
/********************************************************************/
	index=(saved_variables.kwh_index-1)&31;
	if (saved_variables.kwh_per_day[index].date==0)
		kwh_yesterday=0;
	else
		kwh_yesterday=saved_variables.kwh_per_day[index].kwh;
	if (kwh_yesterday < 0)
		kwh_yesterday=saved_variables.kwh_per_day[index].kwh=0;

/********************************************************************/
/*	Get total for selected date										*/
/********************************************************************/
	temp2=kwh_index;
	if ((temp2 < 0) || (temp2 > 31))
		kwh_index=temp2=0;
	CONVERT_TIME(kwh_month_day,saved_variables.kwh_per_day[temp2].date)
	if ( (kwh_month_day < 101) || (kwh_month_day > 1231)
	|| ((kwh_data=saved_variables.kwh_per_day[temp2].kwh) < 0) )
		kwh_data=kwh_month_day=0;			// invalid total or date

/********************************************************************/
/*	Calculate 7-day kwh total										*/
/********************************************************************/
	kwh_total=0;
	for (temp=0;temp<7;temp++)
	{
		temp2=saved_variables.kwh_per_day[index].date;
		if( (temp2 >= 0x0101) && (temp2 <= 0x1231) )
			kwh_total+=saved_variables.kwh_per_day[index].kwh;
		else	// indicate insufficient data for valid total
			kwh_total|=INVALID_TOTAL;
		index=(index-1)&31;
	}
	if ((kwh_total & INVALID_TOTAL)!=0)
		kwh_total_30days=kwh_total_7days=kwh_avg_30days=kwh_avg_7days=0;
	else
	{													// valid total
		kwh_total_7days=kwh_total;
		kwh_avg_7days=MULQ(16,kwh_total,(65535U/7));	// divide by 7

/********************************************************************/
/*	Calculate 30-day kwh total										*/
/********************************************************************/
		for (temp=7;temp<30;temp++)
		{
			temp2=saved_variables.kwh_per_day[index].date;
			if( (temp2 >= 0x0101) && (temp2 <= 0x1231) )
				kwh_total+=saved_variables.kwh_per_day[index].kwh;
			else	// indicate insufficient data for valid total
				kwh_total|=INVALID_TOTAL;
			index=(index-1)&31;
		}
		if ((kwh_total & INVALID_TOTAL)==0)
		{													// valid total
			kwh_avg_30days=MULQ(16,kwh_total,(65535U/30));
			kwh_total_30days=MULQ(16,kwh_total,(65535U/10));// divide by 10
		}
		else											// invalid totals
			kwh_total_30days=kwh_avg_30days=0;
	}

}


/********************************************************************/
/********************************************************************/
/*	Calculate string kwh function									*/
/********************************************************************/
#pragma CODE_SECTION(calculate_string_kwh,".l1_code")

void calculate_string_kwh(void)
{
	#define KWH_SHIFT	9		// shift product right to fit in 16 bits
	#define STRING_KWH	(int)(60.0*60.0*1000.0*10.0/((float)(STRING_FDBK_SIZE<<KWH_SHIFT)/SLOW_TASK_RATE))
	register int temp;
	register unsigned *pntr_1,*pntr_2;

/********************************************************************/
/*	Clear kwh totals if requested									*/
/********************************************************************/
	if (parameters.initialize_cmd==2)
	{
		pntr_1=&saved_variables.string_kw_hrs[0];
		pntr_2=&string_watt_sec[0];
		for (temp=0;temp<STRING_FDBK_SIZE;temp++)
		{
			*pntr_1++=0;		// clear string_kw_hrs
			*pntr_2++=0;		// clear string_watt_sec
		}
		parameters.initialize_cmd=string_kwh_avg=0;
	}

/********************************************************************/
/*	Calculate string averages every STRING_FDBK_SIZE passes			*/
/*	string_amps_avg=4096*(string_amps_total/8)/(number_strings*512)	*/
/*	string_kwh_avg=4096*(kwh_total/16)/(number_strings*256)			*/
/********************************************************************/
	amp_sec_index=(amp_sec_index+1)&(STRING_FDBK_SIZE-1);
	if (amp_sec_index==(STRING_FDBK_SIZE-1))
	{
		temp=(string_amps_total+4L)>>3;
		if (temp < 0)
			temp=0;
		string_amps_total=0;
		string_amps_avg=div_q12(temp,(number_strings<<9))+1;
		temp=(kwh_total+8L)>>4;
		if (temp < 0)
			temp=0;
		kwh_total=0;
		string_kwh_avg=div_q12(temp,(number_strings<<8))+1;
	}

/********************************************************************/
/*	Calculate amps and kwh for one string each pass					*/
/********************************************************************/
	else if (amp_sec_index < number_strings)
	{
		kwh_total+=saved_variables.string_kw_hrs[amp_sec_index];
		temp=string_amps[amp_sec_index];
		string_amps_total+=(long)temp;
		temp=(long)temp*dc_input_volts>>KWH_SHIFT;
		temp+=string_watt_sec[amp_sec_index];
		if (temp < 0)
			temp=0;
		if (temp > STRING_KWH)
		{
			temp-=STRING_KWH;
			string_watt_sec[amp_sec_index]=temp;
			saved_variables.string_kw_hrs[amp_sec_index]++;
		}
		else
			string_watt_sec[amp_sec_index]=temp;
	}		
	global_flags|=SAVE_VARIABLES;
}
