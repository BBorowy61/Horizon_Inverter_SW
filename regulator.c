/********************************************************************/
/********************************************************************/
/*																	*/
/*						Regulator module							*/
/*																	*/
/********************************************************************/
/********************************************************************/

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
Solstice version
2009-07-20	delete calculation of line current feedback for 1 MW
			add on/off input
2009-08-04	change dc voltage regulator from Q16 to Q12
			initialize dc voltage regulator integral term when not running
2009-08-14	correct error in change from Q16 to Q12 scaling
2009-09-14	correct error in determining q-axis current in MPPT mode
2009-10-22	add ac voltage matching
*/
#include "literals.h"
#include "2812reg.h"
#include "io_def.h"
#include "struct.h"
#include "faults.h"

/********************************************************************/
/*		Literals													*/
/********************************************************************/
#define V_CMD_MAX	(int)(1.15*PER_UNIT_F)
#define I_TRIM_MAX	(int)(0.25*PER_UNIT_F)

/********************************************************************/
/*		Global variables											*/
/********************************************************************/
struct DQ i_cmd_line;
struct DQM i_line_neg;
struct ABCDQM v_cmd_pu,i_inv_pu;
long vdc_ref_q16;
int ki_vdc_q12,kp_vdc_q12,ki_inv_q12,kp_inv_q12,kp_line_q12,ki_line_q12;
int vdc_cmd_pu,k_anti_island_q12,transformer_shift;
int cos_angle,sin_angle;

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
struct DQ i_cmd_inv,inv_error,prop_term,int_term,v_fdfwd;
struct DQ line_error,v_line_pu;
struct DQ_LONG inv_ref_q12,v_ref_q12,v_fdfwd_q12,i_ref_q16;
struct DQ_LONG line_int_term_q12,inv_int_term_q12,inv_prop_term_q12;
long ramp_rate_pu_q16,fast_ramp_pu_q16,vdc_int_term;
int iq_cmd_anti_island,vdc_error_pu;
int vdc_cmd_min,vdc_comp_q12,k_vffd_q12;
struct DQ comp;
int debugIndex,debug[4],debug_control,dv[4];

int shut_down_flag;
int real_power_rate_q12,reactive_power_rate_q12;
long maximum_d_q16,maximum_q_q16;
long ramped_i_ref_d_q16,ramped_i_ref_q_q16;
long reg_max,reg_min;
int vdcTarget;

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern far int sin_table[THREE_SIXTY_DEGREES];
extern int fdbk_pu[FDBK_SIZE],fdbk_avg[FDBK_SIZE],fdbk_display[FDBK_SIZE];
extern int pll_output_angle,phase_rotation,gating_phase_shift,sampling_shift;
extern int line_angle,v_fdbk_config,inverse_vl_q12,k_pf_q12,global_flags;
extern int status_flags,operating_state,fpga_outputs,pwm_cmd_scale,ext_inputs;
extern int pll_freq_error_Hz,k_l_q16,k_cac_q16;
extern int faults[FAULT_WORDS];
extern struct DQ i_cmd_in;
extern struct PARAMS parameters;
extern struct FLTS flts;

extern int fdbk_raw[],kwh_index;
extern int gnd_impedance;
extern int a2d_offset_counts,k_fdbk_q15[];
extern int pu_to_inv_amps_q12;
extern int power_control_mode_at_start;
extern struct ABCM line_volts,line_amps,inverter_amps;

/*vvvvvvv 5th harmonic control variables - to be cleaned up some day vvvvvvv*/
int inverter_current_command_vector = 0;

int applyHarm = 0;
int junk;

int v0 = 0;
int v1 = 0;
int v2 = 0;

int d0 = 0;
int d1 = 0;
int d2 = 0;

int harmAngle5;

int dHarm5,qHarm5;

long dHarm5_q12 = 0;
long qHarm5_q12 = 0;
long dHarm5Filtered_q12 = 0;
long qHarm5Filtered_q12 = 0;

long dHarm5Integral_q22 = 0;
long qHarm5Integral_q22 = 0;
long dHarm5Control_q22 = 0;
long qHarm5Control_q22 = 0;
long dHarm5Control_q8 = 0;
long qHarm5Control_q8 = 0;
long dHarm5Error_q12 = 0;
long qHarm5Error_q12 = 0;

long harmMaxVoltage_q22,harmMinVoltage_q22;

long kp_q10,ki_q10;

long harm5Vref_q8[3] = {0,0,0};
int harm5Vcmd[3] = {0,0,0};
int iRef[3] = {0,0,0};
int iNoFun[3] = {0,0,0};

/*^^^^^^^ 5th harmonic control variables - to be cleaned up some day ^^^^^^^*/

#pragma CODE_SECTION(regulators,".h0_code")

/********************************************************************/
/********************************************************************/
/*		Regulator function (called by pwm_isr)						*/
/********************************************************************/

void regulators (void)
{

	register int temp,temp2,temp3;
	register long long_temp_1,long_temp_2;
		
/********************************************************************/
/*		Calculate minimum dc voltage command in per unit			*/
/*		300 Vdc for nominal 200 Vac inverter voltage				*/
/********************************************************************/
	vdc_cmd_min=long_temp_2=MULQ(12,fdbk_display[VL],(int)(1.06*4096.0));

/********************************************************************/
/*		Ramp dc voltage reference to command value					*/
/********************************************************************/
	long_temp_1=(long)vdc_cmd_pu<<16;
	if (long_temp_1 < long_temp_2)
		long_temp_1=long_temp_2;				// clamp to low limit
	RAMP(long_temp_1,vdc_ref_q16,ramp_rate_pu_q16,ramp_rate_pu_q16)
            	           	
/********************************************************************/
/*		Transform line voltage feedback from abc to dq frame		*/
/*		Used for voltage feedforward in this module					*/
/********************************************************************/

	temp=(line_angle-transformer_shift+THIRTY_DEGREES) & ANGLE_MASK;

	ABC_TO_DQ(fdbk_pu[VLA],fdbk_pu[VLB],fdbk_pu[VLC],temp,sin_angle,cos_angle,temp2,temp3,v_line_pu.d,v_line_pu.q)
	MAGNITUDE(fdbk_pu[VL],v_line_pu.d,v_line_pu.q,0)

/********************************************************************/
/*	Line undervoltage instantaneous fault  							*/
/********************************************************************/
	FLT_TMR_LATCHING((fdbk_pu[VL]<flts.line_uv_inst.trip)&&(status_flags&STATUS_PLL_ENABLED),
						flts.line_uv_inst,LINE_UV_FLT_FAST,1)

/********************************************************************/
/*	Line overvoltage instantaneous fault  							*/
/********************************************************************/
	FLT_TMR_LATCHING(fdbk_pu[VL]>flts.line_ov_inst.trip,flts.line_ov_inst,LINE_OV_FLT_INST,0)

/********************************************************************/
/*		Calculate compensation factor for dc voltage variation		*/
/********************************************************************/
	if ((temp=fdbk_pu[VDC]) < INVERSE_MIN)
		temp=INVERSE_MIN;
	vdc_comp_q12=div_q12(V_CMD_MAX,temp);

/********************************************************************/
/*		Transform line current feedback from abc to dq frame		*/
/*		Used for line current regulators in this module	and			*/
/*		ac power calculation in fbk_avg module						*/
/********************************************************************/
	ABC_TO_DQ(fdbk_pu[ILA],fdbk_pu[ILB],fdbk_pu[ILC],line_angle,sin_angle,cos_angle,temp2,temp3,fdbk_pu[ID],fdbk_pu[IQ])
	MAGNITUDE(fdbk_pu[IL],fdbk_pu[ID],fdbk_pu[IQ],0)

/********************************************************************/
/*		Calculate negative sequence component of line current 		*/
/*		Used for IEC current unbalance calculation in fbk_avg		*/
/********************************************************************/
	comp.d=fdbk_pu[ID]-fdbk_avg[ID];	// subtract dc component
	comp.q=fdbk_pu[IQ]-fdbk_avg[IQ];
	temp=(-line_angle<<1)&ANGLE_MASK;	// negative sequence angle
	temp2=sin_table[temp];
	temp3=sin_table[(temp+NINETY_DEGREES) & ANGLE_MASK];
	i_line_neg.d=((long)comp.d*temp3-(long)comp.q*temp2)>>15;
	i_line_neg.q=((long)comp.d*temp2+(long)comp.q*temp3)>>15;
	MAGNITUDE(i_line_neg.m,i_line_neg.d,i_line_neg.q,0);	// magnitude of i_line_neg

/********************************************************************/
/*		Transform inverter current feedback from abc to dq frame	*/
/*		Used for inverter current regulators in this module			*/
/********************************************************************/ 
	temp=(line_angle-transformer_shift)&ANGLE_MASK;
	i_inv_pu.a=fdbk_pu[IIA]+fdbk_pu[IIA2];	// add inverter currents
	i_inv_pu.b=fdbk_pu[IIB]+fdbk_pu[IIB2];
	i_inv_pu.c=fdbk_pu[IIC]+fdbk_pu[IIC2];
	ABC_TO_DQ(i_inv_pu.a,i_inv_pu.b,i_inv_pu.c,temp,sin_angle,cos_angle,temp2,temp3,comp.d,comp.q)

	if ((status_flags & STATUS_SIMULATE)==0)
	{
		i_inv_pu.d=comp.d;
		i_inv_pu.q=comp.q;
	}
	MAGNITUDE(fdbk_pu[II],i_inv_pu.d,i_inv_pu.q,0)

if(parameters.testVar[0]==499){
	
	dv[0]=v0=i_ref_q16.d>>16;
	dv[1]=v1=i_ref_q16.q>>16;
	dv[2]++;
}

/********************************************************************/
/*		Match voltages for closing contactor						*/
/********************************************************************/
	if (operating_state==STATE_MATCH_VOLTAGE || operating_state==STATE_2ND_CONTACTOR_CLOSED || operating_state==STATE_RUN) 
	{		
		if (operating_state==STATE_RUN)  
		{

			/* set shut down flag if either on/off switch or run_enable indicates off state */
			if(!(ext_inputs & ON_OFF_INPUT) || !parameters.run_enable) {
				shut_down_flag = 1;
			}
			else {
				shut_down_flag = 0;
			}

			/* ramp down D axis current; must be before VDC regulators as a means to limit its power */
			if(shut_down_flag) {
				RAMP(0,ramped_i_ref_d_q16,(long)real_power_rate_q12<<4,(long)real_power_rate_q12<<4)
				i_cmd_in.d = ramped_i_ref_d_q16 >> 16;
			}
			else {
				ramped_i_ref_d_q16 = i_ref_q16.d;
			}

			/* maximum D axis current */
			{
				
				if(parameters.power_control_mode == PCM_DC_POWER) // no need to ramp currents in Rectifier mode
					maximum_d_q16 = (long)i_cmd_in.d<<16;
				else	
				RAMP(((long)i_cmd_in.d<<16),maximum_d_q16,(long)real_power_rate_q12<<4,(long)real_power_rate_q12<<4)

				LIMIT_MAX_MIN(maximum_d_q16,LONG(parameters.maximum_dq_current),0);
			}

			if((parameters.power_control_mode==PCM_CONST_I)||(parameters.power_control_mode==PCM_CONST_P))	{
				i_ref_q16.d = maximum_d_q16;
			}
			else if(parameters.power_control_mode>=PCM_MPPT) {

				temp=HIGH(vdc_ref_q16);		// from vdc_cmd_pu		
				
				if(parameters.power_control_mode==PCM_DC_POWER) {
					reg_max = (long)(0.05*PER_UNIT_F) << 16;
					reg_min = -(long)maximum_d_q16;				
				}
				else {
					reg_max = (long)maximum_d_q16;
					reg_min = (long)(-0.05*PER_UNIT_F)<<16;	
				}
				
				PI_REG_PF(fdbk_pu[VDC],temp,vdc_error_pu,vdc_int_term,
					i_ref_q16.d,((long)kp_vdc_q12<<4),((long)ki_vdc_q12<<4),reg_max,reg_min);
			}

			/* fixed, user specified reactive power */
			if(parameters.reactive_power_control_mode == 1) {

				/* calculate maximum Q current available */
				temp = HIGH(i_ref_q16.d);
				temp = long_sqrt(((long)parameters.maximum_dq_current*parameters.maximum_dq_current)-((long)temp*temp));
				maximum_q_q16 = LONG(temp);

				/* convert reactive power to reactive current */
				temp = MULQ(12,parameters.reactive_power_cmd,inverse_vl_q12);

				/* scale to Q16 and limit reactive current to maximum Q current */
				long_temp_1 = (long)temp << 16;
				LIMIT_MAX_MIN(long_temp_1,maximum_q_q16,-maximum_q_q16);

				/* calculate final Q axis current with phase rotation */
				i_ref_q16.q = phase_rotation * long_temp_1;
			}

		/* reactive power calculated using PF */
			else {

				i_ref_q16.q = ((long)k_pf_q12 * (i_ref_q16.d>>7)) >> 5;

				MAGNITUDE(temp,HIGH(i_ref_q16.d),HIGH(i_ref_q16.q),0);

				if (temp > parameters.maximum_dq_current) {
					temp=div_q12(parameters.maximum_dq_current,temp);
					i_ref_q16.d=(i_ref_q16.d>>12)*temp;
					i_ref_q16.q=(i_ref_q16.q>>12)*temp;
				}
			}

		/* ramp down Q axis current; must be after Q axis calculations to override them */
			if(shut_down_flag) {
				RAMP(0,ramped_i_ref_q_q16,(long)reactive_power_rate_q12<<4,(long)reactive_power_rate_q12<<4)
			}

		/* normal Q axis ramping */
			else {

				if (parameters.power_control_mode != PCM_DC_POWER){
					RAMP(i_ref_q16.q,ramped_i_ref_q_q16,(long)reactive_power_rate_q12<<4,(long)reactive_power_rate_q12<<4)
					i_ref_q16.q = ramped_i_ref_q_q16;
				}
			}
			

		}  //end of state_run (adv reactive power control features)
		
			/* voltage match during: */
			/*  - voltage match state */
			/*  - and DC contactor if connected to current source (NOT voltage source) */
	
		else if(parameters.power_source==POWER_SOURCE_CURRENT || operating_state==STATE_MATCH_VOLTAGE)
		{
			if(operating_state==STATE_MATCH_VOLTAGE) {
				vdcTarget = fdbk_pu[VDCIN];
			}

			PI_REG_PF(fdbk_pu[VDC],vdcTarget,vdc_error_pu,vdc_int_term,
						i_ref_q16.d,((long)kp_vdc_q12<<4),((long)ki_vdc_q12<<4),((long)(0.25*PER_UNIT_F)<<16),((long)(-0.25*PER_UNIT_F)<<16))

			i_ref_q16.q = 0;
			
		}
		else 
		{
			i_ref_q16.d = 0;
			i_ref_q16.q = 0;
			
			if(parameters.power_control_mode==PCM_DC_POWER) {
				vdc_ref_q16 = (long)vdc_cmd_min << 16;
			}
		}

/********************************************************************/
/*	Determine line current references								*/
/********************************************************************/
		if(operating_state==STATE_RUN) {
			iq_cmd_anti_island=temp=MULQ(12,pll_freq_error_Hz,k_anti_island_q12);
		}
		else {
			iq_cmd_anti_island=temp=0;
		}
		i_cmd_line.q=(int)(i_ref_q16.q>>16)-temp;
		i_cmd_line.d=(int)(i_ref_q16.d>>16);
		
/********************************************************************/
/*		Line current regulators	(2-axis)							*/
/*		These regulators add a correction term to the dc current	*/
/*		commands to compensate for the ac filter and transformer	*/
/*		Separate rectangular limits for these regulators			*/
/*		Input is i_cmd_line.  Output is inv_ref_q12.				*/
/********************************************************************/
		long_temp_1=0;
		if ((status_flags & STATUS_SIMULATE)==0)	// for normal operation
			long_temp_1=(long)I_TRIM_MAX<<12;		// set upper limit
		long_temp_2=-long_temp_1;					// set lower limit
		temp2=kp_line_q12;							// copy gains to registers
		temp3=ki_line_q12;
		PI_REG_CS(i_cmd_line.d,fdbk_pu[ID],line_error.d,line_int_term_q12.d,inv_ref_q12.d,temp2,temp3,long_temp_1,long_temp_2)
		PI_REG_CS(i_cmd_line.q,fdbk_pu[IQ],line_error.q,line_int_term_q12.q,inv_ref_q12.q,temp2,temp3,long_temp_1,long_temp_2)

/********************************************************************/
/*		Calculate inverter current commands							*/
/********************************************************************/
		temp=fdbk_avg[VIBC];
		i_cmd_inv.d=i_cmd_line.d+(inv_ref_q12.d>>12);
		i_cmd_inv.q=i_cmd_line.q+(inv_ref_q12.q>>12)-MULQ(16,temp,k_cac_q16);

/********************************************************************/
/*		Limit inverter current commands								*/
/********************************************************************/
		MAGNITUDE(temp,i_cmd_inv.d,i_cmd_inv.q,0);
		inverter_current_command_vector = temp;
		if (temp > (int)(1.20*PER_UNIT_F))
		{
			temp=div_q12((int)(1.20*PER_UNIT_F),temp);
			i_cmd_inv.d=MULQ(12,i_cmd_inv.d,temp);
			i_cmd_inv.q=MULQ(12,i_cmd_inv.q,temp);
		}

/********************************************************************/
/*		Inverter current regulators	(2-axis)						*/
/********************************************************************/
		// calculate d-axis terms
		inv_error.d=temp=i_cmd_inv.d-i_inv_pu.d;
		inv_int_term_q12.d+=(long)ki_inv_q12*temp;
		inv_prop_term_q12.d=(long)kp_inv_q12*temp;
		v_fdfwd_q12.d=((long)i_cmd_inv.q*k_l_q16>>4)+(long)v_line_pu.d*k_vffd_q12;

		// calculate q-axis terms
		inv_error.q=temp=i_cmd_inv.q-i_inv_pu.q;
		inv_int_term_q12.q+=(long)ki_inv_q12*temp;
		inv_prop_term_q12.q=(long)kp_inv_q12*temp;
		if (status_flags & STATUS_RVS)
			v_fdfwd_q12.q=((long)i_cmd_inv.d*k_l_q16>>4)+(long)v_line_pu.q*k_vffd_q12;
		else
			v_fdfwd_q12.q=-((long)i_cmd_inv.d*k_l_q16>>4)+(long)v_line_pu.q*k_vffd_q12;

		temp2=inv_int_term_q12.d>>12;
		temp3=inv_int_term_q12.q>>12;
		MAGNITUDE(temp,temp2,temp3,0);
		if (temp > V_CMD_MAX)
		{	temp=div_q12(V_CMD_MAX,temp);
			inv_int_term_q12.d=MULQ(12,inv_int_term_q12.d,temp);
			inv_int_term_q12.q=MULQ(12,inv_int_term_q12.q,temp);
		}

		// add proportional, integral, and feedforward terms to get output
		long_temp_1=inv_prop_term_q12.d+inv_int_term_q12.d+v_fdfwd_q12.d;		
		long_temp_2=inv_prop_term_q12.q+inv_int_term_q12.q+v_fdfwd_q12.q;

		//	compensate regulator output for dc voltage variation
		v_ref_q12.d=(long_temp_1>>12)*vdc_comp_q12;
		v_ref_q12.q=(long_temp_2>>12)*vdc_comp_q12;

		/* 5th harmonic regulation */
		
		if (operating_state==STATE_RUN) //5th harmonic regulation is active during STATE_RUN
		{
			kp_q10 = 1;
			ki_q10 = 1;

			temp=ANGLE_MASK & (line_angle - transformer_shift);
			sin_angle=sin_table[temp];
			cos_angle=sin_table[(temp+NINETY_DEGREES) & ANGLE_MASK];
			DQ_TO_ABC(iRef[0],iRef[1],iRef[2],sin_angle,cos_angle,i_cmd_inv.d,i_cmd_inv.q)

			iNoFun[0] = fdbk_pu[IIA] - iRef[0];
			iNoFun[1] = fdbk_pu[IIB] - iRef[1];
			iNoFun[2] = fdbk_pu[IIC] - iRef[2];

if(parameters.testVar[0]==14){
v0 = iRef[0]>>parameters.testVar[1];
v2 = iNoFun[0]>>parameters.testVar[1];
v1 = fdbk_pu[IIA]>>parameters.testVar[1];
}		
			harmAngle5 = (5*line_angle) & ANGLE_MASK;
			ABC_TO_DQ(iNoFun[0],iNoFun[2],iNoFun[1],harmAngle5,sin_angle,cos_angle,temp2,temp3,dHarm5,qHarm5)

			{
				dHarm5_q12 = (long)dHarm5 << 12;
				dHarm5Filtered_q12 = (dHarm5_q12 + 255*dHarm5Filtered_q12) >> 8;

				qHarm5_q12 = (long)qHarm5 << 12;
				qHarm5Filtered_q12 = (qHarm5_q12 + 255*qHarm5Filtered_q12) >> 8;
			}


#if 1

			harmMaxVoltage_q22=((long)(0.1*V_CMD_MAX))<<22;
			harmMinVoltage_q22=-harmMaxVoltage_q22;

			if(inverter_current_command_vector > (PER_UNIT/10)) {

				PI_REG_PF(0,dHarm5Filtered_q12,dHarm5Error_q12,dHarm5Integral_q22,dHarm5Control_q22,kp_q10,ki_q10,harmMaxVoltage_q22,harmMinVoltage_q22)
				PI_REG_PF(0,qHarm5Filtered_q12,qHarm5Error_q12,qHarm5Integral_q22,qHarm5Control_q22,kp_q10,ki_q10,harmMaxVoltage_q22,harmMinVoltage_q22)

				dHarm5Control_q8 = -(dHarm5Control_q22 >> 14);
				qHarm5Control_q8 = -(qHarm5Control_q22 >> 14);

			}
			else {
				dHarm5Control_q8 = 0;
				qHarm5Control_q8 = 0;
				dHarm5Integral_q22 = 0;
				qHarm5Integral_q22 = 0;
			}

#else

			/* 2^22 = 0x400,000 */
			if(i_cmd_line.d < (2*PER_UNIT/10)) {
				harmMaxVoltage_q22=0L;
			}
			else if(i_cmd_line.d < (3*PER_UNIT/10)) {
				temp = i_cmd_line.d - (2*PER_UNIT/10);
				harmMaxVoltage_q22=temp*(long)(0.1*PER_UNIT*0x400000/100);
			}
			else {
				harmMaxVoltage_q22=(long)(0.1*PER_UNIT*0x400000);
			}

			harmMinVoltage_q22=-harmMaxVoltage_q22;

			PI_REG_PF(0,dHarm5Filtered_q12,dHarm5Error_q12,dHarm5Integral_q22,dHarm5Control_q22,kp_q10,ki_q10,harmMaxVoltage_q22,harmMinVoltage_q22)
			PI_REG_PF(0,qHarm5Filtered_q12,qHarm5Error_q12,qHarm5Integral_q22,qHarm5Control_q22,kp_q10,ki_q10,harmMaxVoltage_q22,harmMinVoltage_q22)

			dHarm5Control_q8 = -(dHarm5Control_q22 >> 14);
			qHarm5Control_q8 = -(qHarm5Control_q22 >> 14);
#endif


			temp=ANGLE_MASK &	(
									5*((line_angle+(phase_rotation*(sampling_shift+gating_phase_shift))) - transformer_shift)
								);
			sin_angle=sin_table[temp];
			cos_angle=sin_table[(temp+NINETY_DEGREES) & ANGLE_MASK];

			DQ_TO_ABC(harm5Vref_q8[0],harm5Vref_q8[2],harm5Vref_q8[1],sin_angle,cos_angle,dHarm5Control_q8,qHarm5Control_q8)
			
			//if(parameters.testVar[3]==1)
			{
				harm5Vcmd[0] = harm5Vref_q8[0] >> 8;
				harm5Vcmd[1] = harm5Vref_q8[1] >> 8;
				harm5Vcmd[2] = harm5Vref_q8[2] >> 8;
			}
			/*else{
				harm5Vcmd[0] = 0;
				harm5Vcmd[1] = 0;
				harm5Vcmd[2] = 0;
				dHarm5Integral_q22=0; //rs added
				qHarm5Integral_q22=0; //rs added
			}*/


		} //end of 5th harmonic regulation 

	}		// end STATE_MATCH_VOLTAGE or STATE_RUN or STATE_2ND_CONTACTOR_CLOSED

/********************************************************************/
/*		For other operating states initialize current regulators	*/
/********************************************************************/
	else
	{
		shut_down_flag = 0;

		line_int_term_q12.d=line_int_term_q12.q=0;
		inv_prop_term_q12.d=inv_prop_term_q12.q=0;
		inv_int_term_q12.d=inv_int_term_q12.q=vdc_int_term=0;

		maximum_d_q16 = maximum_q_q16 = 0;

		dHarm5Integral_q22 = qHarm5Integral_q22 = 0;
		dHarm5Filtered_q12 = qHarm5Filtered_q12 = 0;
	}

/********************************************************************/
/*		Set voltage command for open circuit test					*/
/********************************************************************/
	if (status_flags & STATUS_OPEN_CCT_TEST)
	{
		long_temp_1=(long)parameters.v_cmd_open_cct_pct*(long)(1.15*PER_UNIT_F*4096.0/100.0);
		RAMP(long_temp_1,v_ref_q12.d,ramp_rate_pu_q16,ramp_rate_pu_q16);
		v_ref_q12.q=0;
	}

/* if DC POWER mode - changing control modes is illegal (must reinit) */
	if(power_control_mode_at_start != parameters.power_control_mode) {
		if(power_control_mode_at_start < PCM_DC_POWER && parameters.power_control_mode < PCM_DC_POWER) {
		}
		else {
			SET_FAULT(ILLEGAL_MODE_CHANGE);
	    	status_flags&=~(STATUS_RUN|STATUS_SHUTDOWN);
		}
	}

/********************************************************************/
/*		Initialize regulators if inverter off						*/
/********************************************************************/
	if ((status_flags & STATUS_RUN)==0)
	{
		shut_down_flag = 0;
		i_cmd_line.d=i_cmd_line.q=0;
		i_ref_q16.d=i_ref_q16.q=vdc_int_term=0;
		vdc_error_pu=inv_error.d=inv_error.q=0;
		v_ref_q12.d=v_ref_q12.q=0;
		maximum_d_q16 = maximum_q_q16 = 0;
	}
	v_cmd_pu.d=v_ref_q12.d>>12;
	v_cmd_pu.q=v_ref_q12.q>>12;

/********************************************************************/
/*		Apply vector limiting to voltage command					*/		
/********************************************************************/
	MAGNITUDE(temp,v_cmd_pu.d,v_cmd_pu.q,0);
	if (temp > V_CMD_MAX)
	{	temp=div_q12(V_CMD_MAX,temp);
		v_cmd_pu.d=MULQ(12,v_cmd_pu.d,temp);
		v_cmd_pu.q=MULQ(12,v_cmd_pu.q,temp);
	}

/********************************************************************/
/*		Save 16-bit versions for display							*/		
/********************************************************************/
	int_term.d	=inv_int_term_q12.d>>12;
	int_term.q	=inv_int_term_q12.q>>12;
	prop_term.d	=inv_prop_term_q12.d>>12;
	prop_term.q	=inv_prop_term_q12.q>>12;
	v_fdfwd.d	=v_fdfwd_q12.d>>12;
	v_fdfwd.q	=v_fdfwd_q12.q>>12;

/********************************************************************/
/*		Transform voltage commands from dq to abc and rescale		*/
/********************************************************************/
	// calculate inverter angle
	temp=((line_angle+(sampling_shift+gating_phase_shift)*phase_rotation)-transformer_shift)&ANGLE_MASK;
	sin_angle=sin_table[temp];
	cos_angle=sin_table[(temp+NINETY_DEGREES) & ANGLE_MASK];
	DQ_TO_ABC(temp,temp2,temp3,sin_angle,cos_angle,v_cmd_pu.d,v_cmd_pu.q)

	if(!(ADV_CON_DIS_5_HARM & parameters.adv_control_configuration)) {
		temp  += harm5Vcmd[0];
		temp2 += harm5Vcmd[1];
		temp3 += harm5Vcmd[2];
	}
	else {
		dHarm5Integral_q22=0; //rs added
		qHarm5Integral_q22=0; //rs added 
	}

	v_cmd_pu.a=MULQ(12,temp,pwm_cmd_scale);
	v_cmd_pu.b=MULQ(12,temp2,pwm_cmd_scale);
	v_cmd_pu.c=MULQ(12,temp3,pwm_cmd_scale);

/********************************************************************/
/*		Determine voltage command offset							*/
/*		Looks like a triangular wave at 3rd harmonic frequency		*/
/********************************************************************/
	temp2=temp3=v_cmd_pu.a;
	if (v_cmd_pu.b > temp2)
		temp2=v_cmd_pu.b;
	else if (v_cmd_pu.b < temp3)
		temp3=v_cmd_pu.b;							// positive peak
	if (v_cmd_pu.c > temp2)
		temp2=v_cmd_pu.c;
	else if (v_cmd_pu.c < temp3)
		temp3=v_cmd_pu.c;							// negative peak
	temp=-(temp2+temp3)>>1;	// average of positive & negative peak

/********************************************************************/
/*		Subtract offset (3rd harmonic) from voltage commands		*/
/********************************************************************/
	v_cmd_pu.a+=temp;
	v_cmd_pu.b+=temp;
	v_cmd_pu.c+=temp;

/********************************************************************/
/*		Set voltage command for gate test mode						*/
/********************************************************************/
	if (status_flags & STATUS_GATE_TEST)
	{
		v_cmd_pu.a=v_cmd_pu.b=v_cmd_pu.c=
			MULQ(12,(parameters.v_cmd_gate_test_pct*PERCENT_TO_PU),pwm_cmd_scale);
	}

#if 1
{
	if(debugIndex==0) {
		static unsigned int counter = 0;
		counter++;
		v0 = (int)(0x3ff & (counter + 700)) - 0x1ff;
		v1 = (int)(0x3ff & (counter + 500)) - 0x1ff;
		v2 = (int)(0x3ff & (counter + 300)) - 0x1ff;
	}
	else if(debugIndex==1) {
		v0 = (v_cmd_pu.a>>3);
		v1 = (v_cmd_pu.b>>3);
		v2 = (cos_angle>>6);
	}
	else if(debugIndex==2) {
		v0 = (v_cmd_pu.d>>1);
		v1 = (v_cmd_pu.q>>1);
		v2 = (cos_angle>>6);
	}
	else if(debugIndex==3) {
		v0 = (v_ref_q12.d>>12);
		v1 = (v_ref_q12.q>>12);
		v2 = (cos_angle>>6);
	}
	else if(debugIndex==4) {
		v0 = (i_ref_q16.d>>16);
		v1 = (i_ref_q16.q>>16);
		v2 = (cos_angle>>6);
	}
	else if(debugIndex==5) {
		v0 = (fdbk_pu[VLA]>>1);
		v1 = (fdbk_pu[VLB]>>1);
		v2 = (cos_angle>>6);
	}
	else if(debugIndex==6) {
		v0 = (v_line_pu.d>>1);
		v1 = (v_line_pu.q>>1);
		v2 = (cos_angle>>6);
	}
	else if(debugIndex==7) {
		v0 = (fdbk_pu[ILA]>>1);
		v1 = (fdbk_pu[ILB]>>1);
		v2 = (cos_angle>>6);
	}

	else if(debugIndex==8) {
		v0 = (i_inv_pu.a<<2);
		v1 = (i_inv_pu.b<<2);
		v2 = (cos_angle>>6);
	}
	else if(debugIndex==9) {
		v0 = (i_inv_pu.d<<2);
		v1 = (i_inv_pu.q<<2);
		v2 = (cos_angle>>6);
	}

	else if(debugIndex==10) {
		v0 = (fdbk_pu[VLA]>>1);
		v1 = (fdbk_pu[ILA]>>1);
		v2 = (cos_angle>>6);
	}

	else if(debugIndex==11) {
		v0 = (fdbk_pu[VLA]>>1);
		v1 = (fdbk_pu[VLB]>>1);
		v2 = (fdbk_pu[VLC]>>1);
	}
	else if(debugIndex==12) {
		v0 = (fdbk_pu[ILN]>>1);
		v1 = (fdbk_pu[VDC]>>1);
		v2 = (fdbk_pu[VDCIN]>>1);
	}
	else if(debugIndex==15) {
		v0 = (fdbk_pu[VDC]>>2);
		v1 = (fdbk_pu[VDCIN]>>2);
		v2 = (int)(vdc_int_term>>13);
	}

	d0 = v0 + 0x1ff;
	d1 = v1 + 0x1ff;
	d2 = v2 + 0x1ff;
}

#endif

#if 1
if(debugIndex < 40) {

	extern int k_fdbk_q15[];

	debug[0] = fdbk_raw[debugIndex];
	debug[1] = fdbk_pu[debugIndex];
//	debug[2] = k_fdbk_q15[debugIndex];
	debug[2] = fdbk_avg[debugIndex];
	debug[3] = fdbk_display[debugIndex];
}
else if(debugIndex == 40) {
	debug[0] = dv[0];
	debug[1] = dv[1];
	debug[2] = dv[2];
	debug[3] = dv[3];
}
#endif
}
