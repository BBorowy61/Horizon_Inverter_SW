
/*******************************************************************
 *
 *    DESCRIPTION:  Maximum Power Point Tracking
 *    AUTHOR:  		Xiaorong (Bert) Xia
 *
 *******************************************************************/

#include "literals.h"
#include "2812reg.h"
#include "io_def.h"
#include "struct.h"

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
2008-07-18	correct error in scaling of min power change parameter
2008-07-28	set current command to rated in dc voltage control mode
2008-08-07	add new MPPT mode
2008-09-03	use filtered dc power for mppt (mppt_power_pu)
2008-09-04	use ac output power instead of dc input power
2008-09-26	use calculated dc power as input
2008-09-29	insert Gen I MPPT function
2008-10-16	replace VDC_CMD_MIN literal with vdc_cmd_min variable
2008-10-28	add parameter vdc_step_time_100ms
2009-06-10	replace constant used to convert dc voltage between Gen
			and Gen II scaling by dc_volts_to_pu_q12 & pu_to_dc_volts_q12
2009-07-20	delete power curve function
			add on/off input
*/

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
int i_cmd_const_pwr,vdc_cmd_max;

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern int vdc_cmd_pu,thermal_limit,dc_volts_to_pu_q12;
extern int operating_state,status_flags,ext_inputs;
extern int fdbk_display[FDBK_SIZE],fdbk_avg[FDBK_SIZE];
extern int inverse_vl_q12,pu_to_dc_volts_q12;
extern int pu_to_kva_q12,rated_Hz,vdc_cmd_min;
extern int *real_power_cmd_pntr;
extern long vdc_ref_q16;
extern struct DQ i_cmd_in;
extern struct PARAMS parameters;
extern far struct PWR_CURVE power_curve;
extern int real_power_rate_q12;
extern struct DQ i_cmd_line;
extern struct DQ_LONG i_ref_q16;

#pragma CODE_SECTION(track_maximum_power,".h0_code")

void track_maximum_power(void)
{ 
	register int temp;

/********************************************************************/
/*	Calculate current command for constant power mode				*/
/********************************************************************/
	temp=*real_power_cmd_pntr;

	temp=MULQ(12,temp,inverse_vl_q12);
   	LIMIT_MAX_MIN(temp,thermal_limit,0)
	i_cmd_const_pwr=temp;

    {
		i_cmd_in.d=PER_UNIT;
    	vdc_cmd_pu=fdbk_avg[VDC];
		if (parameters.power_control_mode==PCM_CONST_I)
	    	temp=parameters.real_current_cmd_pct*PERCENT_TO_PU;
		else if (parameters.power_control_mode==PCM_CONST_P)
			temp=i_cmd_const_pwr;
		else if (parameters.power_control_mode==PCM_CONST_VDC || parameters.power_control_mode==PCM_DC_POWER)
		{
			temp=MULQ(12,parameters.dc_voltage_cmd_volts,dc_volts_to_pu_q12);
			if (temp > vdc_cmd_max)
				temp=vdc_cmd_max;
			vdc_cmd_pu=temp;
			temp=i_cmd_const_pwr;
		}
    	LIMIT_MAX_MIN(temp,thermal_limit,0)

   		i_cmd_in.d=temp;
    }
}





