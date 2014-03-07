/********************************************************************/
/********************************************************************/
/*																	*/
/*					Average Feedback Module							*/
/*																	*/
/********************************************************************/
/********************************************************************/

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
1 MW version
2009-01-22	add variable temp_fbk_mv_display
2009-03-04	move 1 MW analog flag inputs from avg_fbk module
2009-04-23	add slave_run_timer
2009-05-22	always calculate inverse_vl_q12
2009-07-20	delete reading of analog flag inputs for 1 MW unit
Solstice version
2009-10-14	read analog input flags for 500 kW only
*/

#include "literals.h"
#include "2812reg.h"
#include "struct.h"
#include "io_def.h"

#define AVERAGING_CYCLES	1
#define	ANALOG_LOW				(int)(1000.0*0.25)
#define	ANALOG_HIGH				(int)(1000.0*0.75)

/********************************************************************/
/*		Global variables											*/
/********************************************************************/
int fdbk_display[FDBK_SIZE],string_amps[STRING_FDBK_SIZE];
//	BECAUSE OF POSSIBLE CONFLICT WITH REAL ADDRESSES, THE VARIABLES IN
//	.fbk_data SHOULD NOT BE ASSIGNED PARAMETER NUMBERS
#pragma DATA_SECTION(fdbk_avg,".fbk_data")
#pragma DATA_SECTION(k_disp_fltr_q16,".fbk_data")
#pragma DATA_SECTION(inverse_vl_q12,".fbk_data")
#pragma DATA_SECTION(string_index,".fbk_data")
#pragma DATA_SECTION(new_cycle,".fbk_data")
#pragma DATA_SECTION(sample_count_max,".fbk_data")
int fdbk_avg[FDBK_SIZE];
int k_disp_fltr_q16,inverse_vl_q12,string_index,new_cycle,sample_count_max;

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
long fdbk_display_q16[FDBK_SIZE];
#pragma DATA_SECTION(one_cycle_sum,".fbk_data")
#pragma DATA_SECTION(temp_fbk_long,".fbk_data")
#pragma DATA_SECTION(string_amps_0_long,".fbk_data")
#pragma DATA_SECTION(string_amps_1_long,".fbk_data")
#pragma DATA_SECTION(inactive_pntr_long,".fbk_data")
#pragma DATA_SECTION(sample_count,".fbk_data")
#pragma DATA_SECTION(k_average,".fbk_data")
#pragma DATA_SECTION(offset,".fbk_data")
#pragma DATA_SECTION(string_index_timer,".fbk_data")
#pragma DATA_SECTION(mux_channel,".fbk_data")
#pragma DATA_SECTION(temp_fbk_mv_display,".fbk_data")
#pragma DATA_SECTION(slave_run_timer,".fbk_data")
long one_cycle_sum[2*FDBK_SIZE];	// 32-bit q0
long temp_fbk_long,string_amps_0_long,string_amps_1_long,*inactive_pntr_long;
int sample_count,k_average,offset,string_index_timer;
int mux_channel,temp_fbk_mv_display,slave_run_timer;

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern int fdbk_pu[FDBK_SIZE],fdbk_raw[RAW_FDBK_SIZE],temp_fbk_mv[8];
extern int status_flags,vdc_cmd_max,ext_inputs,global_flags;
extern int number_strings,phase_rotation,test_mode,kwh_index,rated_Hz;
extern struct DQM i_line_neg;
extern struct PARAMS parameters;
extern struct FLTS flts;

extern void track_maximum_power(void); 

#pragma CODE_SECTION(accumulate_feedback,".h0_code")
#pragma CODE_SECTION(average_ac_feedback,".h0_code")

/********************************************************************/
/********************************************************************/
/*	Accumulate feedback function (called by pwm_isr)				*/
/********************************************************************/

void accumulate_feedback(void) 
{	// inputs are 16-bit instantaneous values in per unit
	register int temp,index,*fdbk_pntr;
	register long *active_pntr_long;
	
	sample_count++;

/********************************************************************/
/* 	Calculate dc input power in per unit							*/
/*	fdbk_pu[PDC]= (fdbk_avg[VDC]*4096/PER_UNIT)*fdbk_avg[IDC]/4096	*/
/*				= fdbk_avg[VDC]*fdbk_avg[IDC]/PER_UNIT				*/
/********************************************************************/
	temp=MULQ(12,fdbk_avg[VDC],(int)(4096.0*4096.0/PER_UNIT_F));
	if (temp < 0)
		temp=0;
	fdbk_pu[PDC]=MULQ(12,temp,fdbk_pu[IDC]);

/********************************************************************/
/* 	Calculate ac output power										*/
/********************************************************************/
	temp			=MULQ_RND(12,fdbk_pu[VL],(int)(4096.0*4096.0/PER_UNIT_F));
	fdbk_pu[KW]		=MULQ_RND(12,temp,fdbk_pu[ID]);
	fdbk_pu[KVAR]	=MULQ_RND(12,temp,(fdbk_pu[IQ]*phase_rotation));
	fdbk_pu[KVA]	=MULQ_RND(12,temp,fdbk_pu[IL]);

/********************************************************************/
/*	Accumulate one cycle sum of all feedback signals				*/
/********************************************************************/
	fdbk_pntr=&fdbk_pu[0];
	active_pntr_long=&one_cycle_sum[offset];
	while (fdbk_pntr < &fdbk_pu[GND])			// ac fdbk is rectified
		(*active_pntr_long++)+=MULQ(12,abs(*fdbk_pntr++),(int)(4096.0*PI/2.0));
	while (fdbk_pntr < &fdbk_pu[FDBK_SIZE])		// dc fdbk is not rectified
		(*active_pntr_long++)+=(*fdbk_pntr++);
                      
/********************************************************************/
/*	Calculate average temperature feedback 							*/
/********************************************************************/
	if ((string_index_timer=(string_index_timer+1)&511)==0)
	{
		if ((temp=temp_fbk_long>>9) < 0)
			temp=0;
		temp_fbk_long=0;
		index=string_index;
		temp_fbk_mv[index&7]=temp;
		temp_fbk_mv_display=temp_fbk_mv[kwh_index&7];

/********************************************************************/
/*	Calculate average string current feedback 						*/
/********************************************************************/
		if (test_mode==ANALOG_IO_TEST)
		{	// convert raw fdbk to millivolts & save in string_amps
			string_amps[index]=MULQ(16,fdbk_raw[index],10000);	// bipolar
			index+=(RAW_FDBK_SIZE/2);
			string_amps[index]=MULQ(16,fdbk_raw[index],6000);	// unipolar
		}
		else
		{

#if 0 //pjf
			if (number_strings <= (STRING_FDBK_SIZE/2))
			{								// one multiplexer borad
				if (((temp=(string_amps_0_long>>9)) < 0)||(index >= number_strings))
					temp=0;
				string_amps[index]=temp;
			}
			else
			{				// two multiplexer boards (number_strings could be odd)
				if ((temp=(string_amps_0_long>>9)) < 0) 
					temp=0;
				if (index < ((number_strings+1)>>1))
					string_amps[index]=temp;
				index+=((number_strings+1)>>1);
				if (((temp=(string_amps_1_long>>9)) < 0) || (index >= number_strings))
					temp=0;
				string_amps[index]=temp;
			}
			string_amps_0_long=string_amps_1_long=0;

#else

string_amps[index]=temp;

#endif
		}

		string_index=(string_index+1)&((STRING_FDBK_SIZE-1)/2);
	}

/********************************************************************/
/*	Accumulate temperature & string current feedback for averaging	*/
/********************************************************************/
	else
	{
		temp_fbk_long+=fdbk_pu[TMP];		// in millvolts

		string_amps_0_long = 0;
		string_amps_1_long = 0;
	}

/********************************************************************/
/*	Prepare for start of new cycle									*/
/*	pll detects start of new cycle from rollover of reference angle	*/
/********************************************************************/
	if ((new_cycle >= AVERAGING_CYCLES)||(sample_count > sample_count_max))
	{
		new_cycle=0;
		k_average=div_q12(16,sample_count);
		sample_count=0;
		inactive_pntr_long=&one_cycle_sum[offset];
		if (offset==0)
			offset=FDBK_SIZE;
		else
			offset=0;
	
	}
}

/********************************************************************/
/********************************************************************/
/*	Average ac feedback function									*/
/*	called by pwm_isr every 1 ms but only runs once per cycle		*/
/********************************************************************/

void average_ac_feedback(void) 
{
	register int temp,deviation,avg,k_fltr;
	register int *fdbk_avg_pntr,*fdbk_display_pntr;
	register long *one_cycle_sum_pntr,*fdbk_display_q16_pntr;
	
/********************************************************************/
/*	Calculate one cycle average of all feedback signals 			*/
/********************************************************************/
	if((avg=k_average)!=0)
	{
		k_fltr=k_disp_fltr_q16;
		one_cycle_sum_pntr=inactive_pntr_long;
		fdbk_avg_pntr=&fdbk_avg[0];
		fdbk_display_pntr=&fdbk_display[0];
		fdbk_display_q16_pntr=&fdbk_display_q16[0];
		while (fdbk_avg_pntr < &fdbk_avg[FDBK_SIZE])
		{	// fdbk_avg = one_cycle_sum*(65536/sample_count)/65536
			*fdbk_avg_pntr=(*one_cycle_sum_pntr)*avg>>16;
			// apply display filter to one cycle average
			(*fdbk_display_q16_pntr)+=(long)k_fltr*(*fdbk_avg_pntr-(int)(*fdbk_display_q16_pntr>>16));
			(*fdbk_display_pntr++)=(*fdbk_display_q16_pntr)>>16;
			fdbk_display_q16_pntr++;
			*one_cycle_sum_pntr=0;
			one_cycle_sum_pntr++;
			fdbk_avg_pntr++;
		}
		k_average=0;

/********************************************************************/
/*	Calculate line voltage unbalance (NEMA definition)				*/
/*	Normalized to actual line voltage not rated voltage				*/
/********************************************************************/
		temp=avg=fdbk_avg[VL]=MULQ(16,(fdbk_avg[VLC]+fdbk_avg[VLB]+fdbk_avg[VLA]),ONE_THIRD_Q16);
		if (temp < INVERSE_MIN)
			temp=INVERSE_MIN;				// to avoid divide overflow
		inverse_vl_q12=div_q12(PER_UNIT,temp)-1;	// 4096*PER_UNIT/avg
    	if (avg < flts.v_unbalance.trip)
    		fdbk_pu[VLUB]=0;		// too low for reliable measurement
		else
		{
	    	deviation=0;		// find largest deviation from average
	    	if ((temp=abs(fdbk_avg[VLA]-avg)) > deviation)
	    		deviation=temp;
	    	if ((temp=abs(fdbk_avg[VLB]-avg)) > deviation)
	    		deviation=temp;
	    	if ((temp=abs(fdbk_avg[VLC]-avg)) > deviation)
	    		deviation=temp;
	    	fdbk_pu[VLUB]=MULQ(12,deviation,inverse_vl_q12);
			if (fdbk_pu[VLUB] < 0)
				fdbk_pu[VLUB]=0;
		}
	
/********************************************************************/
/*	Calculate line current unbalance								*/
/*	Normalized to actual line current not rated current				*/
/********************************************************************/
		avg=fdbk_avg[IL]=MULQ(16,(fdbk_avg[ILC]+fdbk_avg[ILB]+fdbk_avg[ILA]+1),ONE_THIRD_Q16);
    	
		if(parameters.power_control_mode==PCM_DC_POWER) {
			avg=fdbk_display[IL]=((long)(fdbk_display[ILC]+fdbk_display[ILB]+fdbk_display[ILA]+1)*(65536/3))>>16;
    	
		/* For rectifier mode, we need to have more filtered values*/
			fdbk_avg[ILC]= fdbk_display[ILC]; 
			fdbk_avg[ILB]= fdbk_display[ILB];
			fdbk_avg[ILA]= fdbk_display[ILA];
		/***********************************************************/
    	}
		
		if (avg < flts.i_unbalance.trip)
    		fdbk_pu[ILUB]=0;		// too low for reliable measurement
		else
		{
			if (parameters.inverter_type==EU)
			{									// IEC definition
				temp=div_q12(i_line_neg.m,avg);
				fdbk_pu[ILUB]=MULQ(12,temp,PER_UNIT);
			}
			else
			{									// NEMA definition
	    		deviation=0;
		    	if ((temp=abs(fdbk_avg[ILA]-avg)) > deviation)
		    		deviation=temp;
		    	if ((temp=abs(fdbk_avg[ILB]-avg)) > deviation)
		    		deviation=temp;
		    	if ((temp=abs(fdbk_avg[ILC]-avg)) > deviation)
		    		deviation=temp;
				temp=div_q12(PER_UNIT,avg);
		    	fdbk_pu[ILUB]=MULQ(12,deviation,temp);
				if (fdbk_pu[ILUB] < 0)
					fdbk_pu[ILUB]=0;
			}
		}

/********************************************************************/
/*		Read analog flag inputs for units with 2 inverter bridges	*/
/********************************************************************/
		if (parameters.inv_bridges==2)
		{
			if (test_mode!=0)
				global_flags&=~(AC_CONTACTOR_INPUT2|FUSE_FLT_INPUT2|INV_TEMP_FLT_INPUT2|REAC_TEMP_FLT_INPUT2);
			else
			{
				fdbk_avg_pntr=&fdbk_avg[EX1];
				fdbk_display_pntr=&global_flags;

				temp=*fdbk_avg_pntr++;
				if (temp < ANALOG_LOW)
					(*fdbk_display_pntr)|=AC_CONTACTOR_INPUT2;
				else if (temp > ANALOG_HIGH)
					(*fdbk_display_pntr)&=~AC_CONTACTOR_INPUT2;

				temp=*fdbk_avg_pntr++;
				if (temp < ANALOG_LOW)
					(*fdbk_display_pntr)|=FUSE_FLT_INPUT2;
				else if (temp > ANALOG_HIGH)
					(*fdbk_display_pntr)&=~FUSE_FLT_INPUT2;

				temp=*fdbk_avg_pntr++;
				if (temp < ANALOG_LOW)
					(*fdbk_display_pntr)|=INV_TEMP_FLT_INPUT2;
				else if (temp > ANALOG_HIGH)
					(*fdbk_display_pntr)&=~INV_TEMP_FLT_INPUT2;

				temp=*fdbk_avg_pntr++;
				if (temp < ANALOG_LOW)
					(*fdbk_display_pntr)|=REAC_TEMP_FLT_INPUT2;
				else if (temp > ANALOG_HIGH)
					(*fdbk_display_pntr)&=~REAC_TEMP_FLT_INPUT2;
			}
		}
		track_maximum_power();
	}
}
