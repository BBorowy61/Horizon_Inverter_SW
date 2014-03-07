/********************************************************************/
/********************************************************************/
/*																	*/
/*					Simulate Feedback Module						*/
/*																	*/
/********************************************************************/
/********************************************************************/

#include "literals.h"
#include "2812reg.h"
#include "struct.h"
#include "io_def.h"

#define R_DC_SHIFT			5		// 0.031 pu
#define R_PRECHARGE_SHIFT	3
#define K_DQ_FILTER_Q16	(65536U/4)

#define MUL_1000(x,y) (((((long)x*y)>>8)*16778)>>16)	// x*y/1000
#define DQ_FILTER(out_q16,in)\
	do{out_q16.d+=(long)(in.d-HIGH(out_q16.d))*K_DQ_FILTER_Q16;\
	out_q16.q+=(long)(in.q-HIGH(out_q16.q))*K_DQ_FILTER_Q16;}while(0);

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
#pragma DATA_SECTION(v_inv_q16,".test_data")
#pragma DATA_SECTION(i_inv_q16,".test_data")
#pragma DATA_SECTION(test_angle_q16,".test_data")
#pragma DATA_SECTION(v_inv_dc_q16,".test_data")
#pragma DATA_SECTION(test_input,".test_data")
#pragma DATA_SECTION(comp,".test_data")
#pragma DATA_SECTION(idc_in_pu,".test_data")
#pragma DATA_SECTION(test_angle,".test_data")
struct DQ_LONG v_inv_q16,i_inv_q16;
long test_angle_q16,v_inv_dc_q16;
struct ABC test_input;
static struct DQ comp;
int idc_in_pu,test_angle;

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern far int sin_table[THREE_SIXTY_DEGREES];
extern struct PARAMS parameters;
extern struct ABCDQM v_cmd_pu,i_inv_pu;
extern int fdbk_pu[FDBK_SIZE];
extern int pll_output_angle,line_angle,status_flags,string_index,ext_outputs;
extern int ac_contactor_output,precharge_output;

extern int k_l_q16,k_cdc_q16;
extern int cos_angle,sin_angle,transformer_shift;
extern int number_strings,i_string_rated_q4;
extern long f_test_long;

#pragma CODE_SECTION(simulate_feedback,".rom_code")


/********************************************************************/
/********************************************************************/
/*		Feedback Simulation function								*/
/********************************************************************/

void simulate_feedback(void)
{
	#define TEST3	0//(-4096/10)
	#define TEST5	0//(4096/20)
	#define TEST11	0//(4096/22)
	#define TEST1	(4096-TEST5-TEST11)
		
	register int temp,temp2;
	register long pos_limit,neg_limit,long_temp;

	pos_limit=(long)2*PER_UNIT<<16;
	neg_limit=-pos_limit;
		
/********************************************************************/
/*		Generate simulated voltage waveforms						*/
/********************************************************************/
	fdbk_pu[VDCIN]=(int)(1.4*PER_UNIT);
	test_angle_q16+=f_test_long;
	test_angle=(test_angle_q16>>(16+ANGLE_SHIFT))&ANGLE_MASK;
	temp=test_angle;
	test_input.a=MULQ(15,TEST1,sin_table[temp]);
	if (TEST3!=0)
		test_input.a+=MULQ(15,TEST3,sin_table[(temp*3)&ANGLE_MASK]);
	if (TEST5!=0)
		test_input.a+=MULQ(15,TEST5,sin_table[(temp*5)&ANGLE_MASK]);
	if (TEST11!=0)
		test_input.a+=MULQ(15,TEST11,sin_table[(temp*11)&ANGLE_MASK]);
	temp=(test_angle-ONE_TWENTY_DEGREES)&ANGLE_MASK;
	test_input.b=MULQ(15,TEST1,sin_table[temp]);
	if (TEST3!=0)
		test_input.b+=MULQ(15,TEST3,sin_table[(temp*3)&ANGLE_MASK]);
	if (TEST5!=0)
		test_input.b+=MULQ(15,TEST5,sin_table[(temp*5)&ANGLE_MASK]);
	if (TEST11!=0)
		test_input.b+=MULQ(15,TEST11,sin_table[(temp*11)&ANGLE_MASK]);
	temp=(test_angle+ONE_TWENTY_DEGREES)&ANGLE_MASK;
	test_input.c=MULQ(15,TEST1,sin_table[temp]);
	if (TEST3!=0)
		test_input.c+=MULQ(15,TEST3,sin_table[(temp*3)&ANGLE_MASK]);
	if (TEST5!=0)
		test_input.c+=MULQ(15,TEST5,sin_table[(temp*5)&ANGLE_MASK]);
	if (TEST11!=0)
		test_input.c+=MULQ(15,TEST11,sin_table[(temp*11)&ANGLE_MASK]);

	temp2=parameters.v_cmd_simulate_pct*PERCENT_TO_PU;
	fdbk_pu[VLA]=MULQ(12,test_input.a,temp2);
	fdbk_pu[VLB]=MULQ(12,test_input.b,temp2);
	fdbk_pu[VLC]=MULQ(12,test_input.c,temp2);
	if (ext_outputs & (precharge_output|ac_contactor_output))
		fdbk_pu[VIBC]=MULQ(12,test_input.b,temp2);
	else
		fdbk_pu[VIBC]=0;

/********************************************************************/
/*	Inverter running												*/
/********************************************************************/
	
	if (status_flags & STATUS_RUN)
	{

/********************************************************************/
/*		Calculate inverter dc voltage and current					*/
/*		dc contactor closed											*/
/********************************************************************/
		if (ext_outputs & DC_CONTACTOR_OUTPUT)
		{
			temp=(idc_in_pu-(int)(0.4*PER_UNIT_F))>>1;
			if (temp < 0)
				temp=0;
			temp+=(int)(0.2*PER_UNIT_F);	// input resistance
			idc_in_pu=div_q12((fdbk_pu[VDCIN]-fdbk_pu[VDC]),temp);
			idc_in_pu=MULQ(12,idc_in_pu,PER_UNIT);
			v_inv_dc_q16+=(long)(idc_in_pu-fdbk_pu[IDC])*k_cdc_q16;
			LIMIT_MAX_MIN(v_inv_dc_q16,pos_limit,0)
			fdbk_pu[VDCIN]=HIGH(v_inv_dc_q16);
		}

/********************************************************************/
/*		dc contactor open											*/
/********************************************************************/
		else
		{
			idc_in_pu=0;
			v_inv_dc_q16-=(long)fdbk_pu[IDC]*k_cdc_q16;
		}

/********************************************************************/
/*		Transform line voltages from abc to dq frame				*/
/********************************************************************/
		ABC_TO_DQ(fdbk_pu[VLA],fdbk_pu[VLB],fdbk_pu[VLC],line_angle,sin_angle,cos_angle,temp,temp2,comp.d,comp.q)
		DQ_FILTER(v_inv_q16,comp)

/********************************************************************/
/*		Calculate inverter q-axis voltage and current				*/
/*		i_inv_q16.q+=((v_inv_dc_q16*v_cmd_pu.q/1000)-i_inv_q16.q>>R_DC_SHIFT)*k_l_q16/65536	*/
/********************************************************************/
		temp=MUL_1000(HIGH(v_inv_dc_q16),v_cmd_pu.q);
		long_temp=i_inv_q16.q+(long)(temp-HIGH(i_inv_q16.q>>R_DC_SHIFT))*k_l_q16;
		LIMIT_MAX_MIN(long_temp,pos_limit,neg_limit)
		i_inv_q16.q=long_temp;

/********************************************************************/
/*		Calculate inverter d-axis voltage and current				*/
/*		i_inv_q16.d+=((v_inv_dc_q16*v_cmd_pu.d/1000)-i_inv_q16.d>>R_DC_SHIFT)*k_l_q16/65536	*/
/********************************************************************/
		temp=MUL_1000(HIGH(v_inv_dc_q16),v_cmd_pu.d);
		long_temp=i_inv_q16.d+(long)(temp-HIGH(v_inv_q16.d+(i_inv_q16.d>>R_DC_SHIFT)))*k_l_q16;
		LIMIT_MAX_MIN(long_temp,pos_limit,neg_limit)
		i_inv_q16.d=long_temp;

/********************************************************************/
/*		Calculate dc link current									*/
/********************************************************************/
		MAGNITUDE(temp,HIGH(i_inv_q16.d),HIGH(i_inv_q16.q),0)
		if (i_inv_q16.d < 0)
			temp=-temp;
		fdbk_pu[IDC]=MUL_1000(temp,v_cmd_pu.d);
	}	// end STATUS_RUN

/********************************************************************/
/*		Inverter not running										*/
/********************************************************************/
	else
	{											// gating off
		i_inv_q16.q=idc_in_pu=0;
		if (ext_outputs & (precharge_output|ac_contactor_output))
		{							// dc link capacitor charging
			v_inv_q16.d=(long)fdbk_pu[VIBC]<<16;
			// calculate voltage across resistor
			if (ext_outputs & ac_contactor_output)	// precharge resistor shorted
				long_temp=i_inv_q16.d>>R_DC_SHIFT;
			else									// precharge resistor in circuit
				long_temp=i_inv_q16.d<<R_PRECHARGE_SHIFT;
			// calculate voltage across inductor
			temp=(v_inv_dc_q16-v_inv_q16.d-long_temp)>>16;
			//calculate capacitor charging current
			long_temp=i_inv_q16.d+(long)temp*k_l_q16;
			// charging current must be negative
			LIMIT_MAX_MIN(long_temp,0,neg_limit)
			i_inv_q16.d=long_temp;
			fdbk_pu[IDC]=HIGH(i_inv_q16.d);
			// calculate capacitor voltage
			if ((v_inv_dc_q16=v_inv_dc_q16-(long)fdbk_pu[IDC]*k_cdc_q16) < 0)
				v_inv_dc_q16=0;
		}
		else
		{							// dc link capacitor discharging
			i_inv_q16.d=0;
			fdbk_pu[IDC]=HIGH(i_inv_q16.d);
			if ((v_inv_dc_q16-=10000) < 0)
				v_inv_dc_q16=0;
		}
	}											// end gating off
			
/********************************************************************/
/*		Determine feedbacks											*/
/********************************************************************/
	fdbk_pu[VDC]=HIGH(v_inv_dc_q16);
	fdbk_pu[ID]=i_inv_pu.d=HIGH(i_inv_q16.d);
	fdbk_pu[IQ]=i_inv_pu.q=HIGH(i_inv_q16.q);
	DQ_TO_ABC(fdbk_pu[ILA],fdbk_pu[ILB],fdbk_pu[ILC],sin_angle,cos_angle,fdbk_pu[ID],fdbk_pu[IQ])

	temp=(line_angle-transformer_shift)&ANGLE_MASK;
	sin_angle=sin_table[temp];
	cos_angle=sin_table[(temp+NINETY_DEGREES) & ANGLE_MASK];
	DQ_TO_ABC(fdbk_pu[IIA],fdbk_pu[IIB],fdbk_pu[IIC],sin_angle,cos_angle,i_inv_pu.d,i_inv_pu.q)
	if (parameters.inv_bridges==2)
	{
		fdbk_pu[IIA]=fdbk_pu[IIA2]=fdbk_pu[IIA]>>1;
		fdbk_pu[IIB]=fdbk_pu[IIB2]=fdbk_pu[IIB]>>1;
		fdbk_pu[IIC]=fdbk_pu[IIC2]=fdbk_pu[IIC]>>1;
	}
	else
		fdbk_pu[IIA2]=fdbk_pu[IIB2]=fdbk_pu[IIC2]=0;
		

	fdbk_pu[TMP]=1000;

}											// end simulate_feedback
