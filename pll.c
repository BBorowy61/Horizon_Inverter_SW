/********************************************************************/
/********************************************************************/
/*																	*/
/*						PLL module									*/
/*																	*/
/********************************************************************/
/********************************************************************/

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
2008-03-19	add check for pll_bw==0
2008-03-31	add inverter_phase_shift
2008-04-25	delete inverter_phase_shift
			add alpha-beta calculation for delta feedback
2008-04-28	add gating_phase_shift parameter
2008-05-02	move gating_phase_shift from line to inverter
2008-07-16	move sampling_shift from inverter angle to pll error
			move inverter angle calculations to regulator
2008-08-19	add line_angle variable
1 MW version
2009-02-23	run PI regulator every pass
*/
#include "literals.h"
#include "2812reg.h"
#include "io_def.h"
#include "struct.h"

#define	STEP_TIME		5000
#define PLL_DEGREE		(65536.0/360.0)
#define ROTN_DELAY		500
#define PLL_DELAY		100
#define FILTER_SHIFT	6			// 64*0.2 ms = 12.8 ms

/********************************************************************/
/*		Global variables											*/
/********************************************************************/
#pragma DATA_SECTION(k_ac_fltr_q16,".bss")
#pragma DATA_SECTION(ac_fltr_gain_comp,".bss")
#pragma DATA_SECTION(ac_fltr_phase_shift,".bss")
#pragma DATA_SECTION(pll_output_angle,".bss")
#pragma DATA_SECTION(phase_rotation,".bss")
#pragma DATA_SECTION(gating_phase_shift,".bss")
#pragma DATA_SECTION(sampling_shift,".bss")
#pragma DATA_SECTION(pll_freq_error_Hz,".bss")
#pragma DATA_SECTION(pll_freq_to_Hz_q27,".bss")
#pragma DATA_SECTION(line_angle,".bss")
int k_ac_fltr_q16,ac_fltr_gain_comp,ac_fltr_phase_shift;
int pll_output_angle,phase_rotation,gating_phase_shift,sampling_shift;
int pll_freq_error_Hz,pll_freq_to_Hz_q27,line_angle;

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
#pragma DATA_SECTION(pll_step_timer,".bss")
#pragma DATA_SECTION(pll_error_timer,".bss")
#pragma DATA_SECTION(pll_error,".bss")
#pragma DATA_SECTION(pll_error_old,".bss")
#pragma DATA_SECTION(pll_step,".bss")
#pragma DATA_SECTION(pll_input_angle,".bss")
#pragma DATA_SECTION(pll_input_angle_old,".bss")
#pragma DATA_SECTION(rotn_timer,".bss")
#pragma DATA_SECTION(pll_input_angle_deg,".bss")
#pragma DATA_SECTION(pll_output_angle_deg,".bss")
#pragma DATA_SECTION(pll_error_deg,".bss")
#pragma DATA_SECTION(pll_freq_Hz,".bss")
#pragma DATA_SECTION(pll_bw,".bss")
#pragma DATA_SECTION(pll_kp,".bss")
#pragma DATA_SECTION(pll_ki,".bss")
#pragma DATA_SECTION(v_line_alpha,".bss")
#pragma DATA_SECTION(v_beta_long,".bss")
#pragma DATA_SECTION(pll_angle_long,".bss")
#pragma DATA_SECTION(freq_error_long,".bss")
#pragma DATA_SECTION(pll_freq_err_max_long,".bss")
int pll_step_timer,pll_error_timer,pll_error,pll_error_old;
int pll_step,pll_input_angle,pll_input_angle_old,rotn_timer;
int pll_input_angle_deg,pll_output_angle_deg,pll_error_deg,pll_freq_Hz;
int pll_bw,pll_kp,pll_ki;
int v_line_alpha,v_line_beta,v_line_magn;
long line_freq_long,line_freq_rated;
long v_alpha_long,v_beta_long;
long pll_angle_long,freq_error_long;
long pll_freq_err_max_long;

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern far int asin_table[4096];
extern int fdbk_pu[FDBK_SIZE],fdbk_avg[FDBK_SIZE],fdbk_display[FDBK_SIZE];
extern int status_flags,new_cycle,rated_Hz;
extern int ext_outputs,v_fdbk_config,operating_state;
extern long pll_Hz_long;
extern struct FLT pll_enable;
extern struct PARAMS parameters;

#pragma CODE_SECTION(phase_lock_loop,".h0_code")

/********************************************************************/
/********************************************************************/
/*		PLL function	(called by pwm_isr)							*/
/********************************************************************/

void phase_lock_loop (void)
{
	register int temp1,temp2;

/********************************************************************/
/*		Initialize pll gains										*/
/********************************************************************/
	if ((pll_bw==0)||(pll_bw!=parameters.pll_bw))
	{
		pll_bw=parameters.pll_bw;
		if ((pll_bw < 1) || (pll_bw > 100))
			parameters.pll_bw=pll_bw=20;;
		pll_kp=2*pll_bw*pll_bw;
		pll_ki=pll_bw;
		pll_freq_err_max_long=pll_Hz_long*(pll_bw>>1);
		pll_enable.trip=(int)(PER_UNIT/10);
		pll_enable.delay=500;							// 0.5 sec
	}

/********************************************************************/
/*		Calculate new PLL and line voltage angles					*/
/*		scaling is 65536^2 = 360 degrees so masking not required	*/
/********************************************************************/
	temp1=pll_output_angle;
	if (status_flags & STATUS_RVS)
	{
		phase_rotation=-1;
		pll_angle_long+=(freq_error_long-line_freq_rated);
		pll_output_angle=pll_angle_long>>16;
		if ((pll_output_angle > 0) && (temp1 < 0))
			new_cycle++;
	}
	else
	{
		phase_rotation=1;
		pll_angle_long+=(freq_error_long+line_freq_rated);
		pll_output_angle=pll_angle_long>>16;
		if ((pll_output_angle < 0) && (temp1 > 0))
		new_cycle++;
	}

/********************************************************************/
/*		Calculate and filter alpha-beta line voltages				*/
/*		v_alpha=(2*v_an - v_bn - v_cn)/3 or 	-(2*v_ca + v_bc)/3	*/
/*		v_beta=(v_cn - v_bn)/sqrt(3)	 or 	-v_bc/sqrt(3)		*/
/********************************************************************/
	if (v_fdbk_config==0)
	{								// line to neutral feedback
		temp1=(long)((fdbk_pu[VLA]<<1)-fdbk_pu[VLB]-fdbk_pu[VLC])*(int)(4096/3.0)>>12;
		temp2=(long)(fdbk_pu[VLC]-fdbk_pu[VLB])*(int)(4096/SQRT3)>>12;
	}
	else
	{								// line to line feedback
		temp1=(long)-((fdbk_pu[VLC]<<1)+fdbk_pu[VLB])*(int)(4096/SQRT3)>>12;
		temp2=-fdbk_pu[VLB];
	}
	temp1=MULQ(12,temp1,ac_fltr_gain_comp);	// compensate for filter gain
	v_alpha_long+=(long)(temp1-(int)(v_alpha_long>>16))*k_ac_fltr_q16;	// LP filter
	v_line_alpha=v_alpha_long>>16;

	temp2=MULQ(12,temp2,ac_fltr_gain_comp);	// compensate for filter gain
	v_beta_long+=(long)(temp2-(int)(v_beta_long>>16))*k_ac_fltr_q16;	// LP filter
	v_line_beta=v_beta_long>>16;

	MAGNITUDE(v_line_magn,v_line_alpha,v_line_beta,0)

/********************************************************************/
/*		Determine PLL enable flag									*/
/********************************************************************/
	if ((fdbk_avg[VL] < pll_enable.trip)||
		(status_flags & STATUS_GATE_TEST)||
		((status_flags & STATUS_OPEN_CCT_TEST)&&(ext_outputs & AC_CONTACTOR_OUTPUT)))
	{
		if (pll_enable.timer < pll_enable.delay)
			pll_enable.timer++;
		else
		{
			pll_error_timer=PLL_DELAY;
			status_flags&=~(STATUS_PLL_ENABLED|STATUS_PLL_LOCKED);
			freq_error_long=pll_error=0;
		}
	}
	else
	{
		if (pll_enable.timer > 0)
			pll_enable.timer--;
		else
			status_flags|=STATUS_PLL_ENABLED;
	}
			
/********************************************************************/
/*		Calculate phase angle of line voltage from alpha-beta 		*/
/*		Scaling is 65536=360 degrees								*/
/*		pll_input_angle=asin(v_line_alpha/v_line_magn)				*/
/********************************************************************/
	if (status_flags & STATUS_PLL_ENABLED)
	{
		pll_input_angle_old=pll_input_angle;
		temp1=abs(v_line_alpha);
		temp2=abs(v_line_beta);
		if (temp1 < temp2)
		{												// alpha smaller
			temp2=div_q12(temp1,v_line_magn);
			if ((unsigned)temp2 < THREE_SIXTY_DEGREES)
				pll_input_angle=asin_table[temp2];
			else				// get 1st quadrant angle from table
				pll_input_angle=0;
		}
		else
		{												// beta smaller
			temp1=div_q12(temp2,v_line_magn);
			if ((unsigned)temp1 < THREE_SIXTY_DEGREES)
				pll_input_angle=16384-asin_table[temp1];
			else				// get 1st quadrant angle from table
				pll_input_angle=0;
		}
		if (v_line_alpha>=0)
		{												// alpha positive
			if (v_line_beta<0)							// beta negative
				pll_input_angle=32768-pll_input_angle;	// 2nd quadrant
		}
		else
		{												// alpha negative
			if (v_line_beta<0)							// beta negative
				pll_input_angle=32768+pll_input_angle;	// 3rd quadrant
			else										// beta positive
				pll_input_angle=-pll_input_angle;		// 4th quadrant
		}

/********************************************************************/
/*		Calculate and filter line frequency							*/
/********************************************************************/
		line_freq_long+=(((long)(pll_input_angle-pll_input_angle_old)<<16)-line_freq_long)>>FILTER_SHIFT;
		
/********************************************************************/
/*		Determine phase rotation if not running						*/
/********************************************************************/
		if (!(status_flags&STATUS_RUN))
		{
			if (line_freq_long >= 0)
			{
				if (rotn_timer < ROTN_DELAY)
					rotn_timer++;
				else 
					status_flags&=~STATUS_RVS;
			}
			else
			{
				if (rotn_timer > 0)
					rotn_timer--;
				else 
					status_flags|=STATUS_RVS;
			}
		}

/********************************************************************/
/*		Calculate pll error including filter phase shift			*/
/********************************************************************/
		pll_error=pll_input_angle-pll_output_angle+
			((ac_fltr_phase_shift+sampling_shift)*phase_rotation)-
			(int)(90.0*PLL_DEGREE);
			
/********************************************************************/
/*		Add step to PLL error for testing response					*/
/*		pll_step is adjustable step size in degrees					*/
/********************************************************************/
		if (pll_step!=0)
		{
			if (++pll_step_timer < (STEP_TIME>>1))
				pll_error+=(pll_step*(int)PLL_DEGREE);
			else if (pll_step_timer < STEP_TIME)
				pll_error-=(pll_step*(int)PLL_DEGREE);
			else
				pll_step_timer=0;
		}
		pll_error_deg=MULQ(16,pll_error,3600);	// 65536=360.0 deg
			
/********************************************************************/
/*		PI regulator												*/
/********************************************************************/
		freq_error_long+=(long)pll_error*pll_ki+(long)(pll_error-pll_error_old)*pll_kp;
		LIMIT_MAX_MIN(freq_error_long,pll_freq_err_max_long,-pll_freq_err_max_long)
		pll_error_old=pll_error;

/********************************************************************/
/*		Determine if pll is locked									*/
/********************************************************************/
		if (abs(pll_error_deg) > 100)			// 10.0 degrees
		{
			if (pll_error_timer < PLL_DELAY)
				pll_error_timer++;
			else 
				status_flags&=~STATUS_PLL_LOCKED;
		}
		else
		{
			if (pll_error_timer > 0)
				pll_error_timer--;
			else 
				status_flags|=STATUS_PLL_LOCKED;
		}
	}

/********************************************************************/
/*		Set pll frequency to rated if pll disabled					*/
/********************************************************************/
	else
	{
		if (status_flags & STATUS_OPEN_CCT_TEST)
		{
			if (parameters.freq_simulate < 0)
				status_flags|=STATUS_RVS;
			else 
				status_flags&=~STATUS_RVS;
		}
		line_freq_long=line_freq_rated;
		freq_error_long=0;
	}
	
/********************************************************************/
/*		Convert angles to units of 0.1 degrees for display			*/
/********************************************************************/
	temp2=360*10;								// 65536=360.0 deg
	if ((temp1=MULQ(16,pll_input_angle,temp2)) < 0)
		temp1+=temp2;
	pll_input_angle_deg=temp1;
	if ((temp1=MULQ(16,pll_output_angle,temp2)) < 0)
		temp1+=temp2;
	pll_output_angle_deg=temp1;
	line_angle=(pll_output_angle>>ANGLE_SHIFT) & ANGLE_MASK;
	
/********************************************************************/
/*		Convert frequencies to units of 0.01 Hz for display			*/
/********************************************************************/
	temp2=pll_freq_to_Hz_q27;
	fdbk_pu[FRQ]=MULQ(16,abs(line_freq_long>>FREQ_SHIFT),temp2);
	fdbk_pu[FRQERR]=fdbk_pu[FRQ]-rated_Hz*100;
	pll_freq_error_Hz=MULQ(16,(int)(freq_error_long>>FREQ_SHIFT),temp2);
	pll_freq_Hz=MULQ(16,(int)((freq_error_long+line_freq_rated)>>FREQ_SHIFT),temp2);
}



