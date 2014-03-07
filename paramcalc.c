/********************************************************************/
/********************************************************************/
/*																	*/
/*					Parameter Calculation module					*/
/*																	*/
/********************************************************************/
/********************************************************************/

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
1 MW version
2008-11-10	ignore gate_test_param in gating i/o test mode
2008-12-17	add 1 MW rating
2008-12-18	add contactor_inputs,contactor_outputs,ac_contactor_output,precharge_output
2009-01-12	correct error in feedback scale factor for ground impedance
			add special default parameter settings for 250 kw 60 Hz units
2009-01-14	add calculation of voltage feedforward gain k_vffd_q12
2009-01-19	clear scaling faults before scaling calculations
2009-01-23	calculate parameters for line instantaneous undervoltage fault
2009-02-04	correct scaling factor for VDCIN2
2009-02-17	delete low temperature fault parameters
2009-02-18	add new ground fault parameters
2009-02-24	modify calculation of gain and phase shift compensation of ac filter
2009-02-24	move slow_param_update function to separate file
2009-02-25	increase minimum allowable pwm frequency from 2000 to 4000 Hz
2009-03-09	add special dc voltage feedback scaling for 1 MW
2009-04-16	change instantaneous trip delays from 2 to 1
2009-02-04	correct another scaling error for VDCIN2
2009-05-06	divide hardware overcurrent trip by 2 for 1 MW
2009-05-07	add line instantaneous undervoltage fault
2009-05-19	if ground leakage gain parameter not zero enable protection
2009-05-26	add special value of voltage feedback scaling for 1 MW
2009-05-27	normalize integral gains to nominal PWM frequency
2009-06-01	move power factor calculation to oper.c
2009-07-16	change minimum line voltage from 200V to 100V
2009-07-20	delete 1 MW code
			delete ground leakage fault
2009-08-04	change maximum value of regulator gains from 5.000 to 7.999
2009-10-15	delete 600 kW rating
			add second ac contactor
2009-10-26	add parameter precharge_config
*/
#include "literals.h"
#include "2812reg.h"
#include "io_def.h"
#include "struct.h"
#include "faults.h"

#define FULL_SCALE_BP	(5*PER_UNIT)
#define FULL_SCALE_UP	(3*PER_UNIT)
#define KW_RATED_MIN	30
#define KW_RATED_MAX	1000
#define V_RATED_MIN		100			// formerly 208
#define V_RATED_MAX		600
#define V_INV_NOM		200
#define PWM_HZ_NOMINAL	5000.0

#define KIL		0
#define KII		1
#define KIN		2
#define KIDC	3
#define KIGND	4
#define KISTR	5

/********************************************************************/
/*		Global variables											*/
/********************************************************************/
struct PARAMS parameters;

#pragma DATA_SECTION(saved_data,".nv_ram")
#pragma DATA_SECTION(saved_data_2,".nv_ram2")
struct NV_RAM saved_data,saved_data_2;

long pll_Hz_long,f_test_long;
int kw_rated_param,v_inv_rated_param;
int rated_Hz,v_rated,rated_inv_volts,rated_dc_volts,v_fdbk_config;
int rated_line_amps,rated_inv_amps,rated_dc_amps,voltage_derating_q12;
int pwm_Hz,pwm_period;
int k_l_q16,k_cac_q16,k_cdc_q16;
int number_strings,i_string_rated_q4,modbus_access_level;

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
int rating_code,dc_volts_to_pu_q12,modbus_access_code;
int update_case,channel,gate_test_param;
int kva_rated,l_pu;
int ac_voltage_fdbk_ratio,dc_voltage_fdbk_ratio;
int power_control_mode_at_start;

#pragma DATA_SECTION(ifbk_ratio_table,".rom_const")
#pragma DATA_SECTION(ifbk_burden_table,".rom_const")

const far int ifbk_ratio_table[RATINGS_SIZE][KISTR+1];
const far int ifbk_burden_table[RATINGS_SIZE][KISTR+1];

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern far int asin_table[4096];

extern int faults[FAULT_WORDS];
extern int k_fdbk_q15[RAW_FDBK_SIZE];
extern int status_flags,i_cmd_max,test_mode,operating_state,global_flags;
extern int vdc_step_min_pu,vdc_step_max_pu;
extern int k_disp_fltr_q16,k_ac_fltr_q16,ac_fltr_gain_comp,ac_fltr_phase_shift;
extern int ki_vdc_q12,kp_vdc_q12,ki_inv_q12,kp_inv_q12,kp_line_q12,ki_line_q12;
extern int kp_fan_q16,ki_fan_q16,kp_overtemp_q16,ki_overtemp_q16;
extern int fan_off_temp,fan_on_temp,fan_speed_min_pu;
extern int pwm_int_us,pll_freq_to_Hz_q27,sample_count_max,gating_phase_shift,sampling_shift;
extern long ramp_rate_pu_q16,fast_ramp_pu_q16,line_freq_rated,analog_input_faults;
extern int number_temp_fbk,string_index,gate_test_select;
extern int pwm_cmd_scale,hw_oc_trip,vdc_cmd_max;
extern int pu_to_line_volts_q12,pu_to_line_amps_q12,pu_to_inv_volts_q12,pu_to_inv_amps_q12;
extern int pu_to_dc_volts_q12,pu_to_dc_amps_q12,pu_to_kva_q12;
extern int k_anti_island_q12,transformer_shift,k_vffd_q12;
extern int fdbk_offset_vdc_pu,fdbk_offset_idc_pu;
extern int real_power_cmd_pu,reactive_power_cmd_pu;
extern int *real_power_cmd_pntr,*reactive_power_cmd_pntr;
extern int vdc_in_timer_100ms;
extern int ext_input_faults,ext_output_faults,gating_faults,serial_comm_faults;
extern int ac_contactor_input,contactor_inputs;
extern int contactor_outputs,ac_contactor_output,precharge_output,fpga_outputs,sw_fault_output;
extern struct FLTS flts;
extern far struct PARAM_ENTRY param_table[PARAM_MAX+1];
extern int phase_rotation;
extern int real_power_rate_q12,reactive_power_rate_q12;
extern int k_pf_q12,phase_rotation;
extern int debugIndex,debug[],debug_control,dv[];


#pragma CODE_SECTION(parameter_calc,".rom_code")
#pragma CODE_SECTION(slow_param_update,".l1_code")

/********************************************************************/
/*		Local functions												*/
/********************************************************************/
void slow_param_update(void);

/********************************************************************/
/*		External functions											*/
/********************************************************************/
extern void setup_analog_out(int channel);
extern void setup_data_channel(int channel);
extern void setup_trigger_channel(void);
extern void setup_trigger_condition(void);
extern void setup_trigger_level(void);
extern void setup_trigger_delay(void);
extern void setup_trigger_interval(void);

//	temp1=(ratio*FS)/(burden*rated/10)
//		 =4096*(ratio*FS/4096)/(burden*rated/10)
#define CALC_K_IFDBK(ratio,burden,FS,rated,K){\
		temp1=ratio;\
		if (temp1==0)\
			ratio=temp1=*(ratio_pntr+K);\
		temp1=MULQ_RND(12,temp1,(int)(FS*10.0/16.0));\
		temp2=burden;\
		if (temp2==0)\
			burden=temp2=*(burden_pntr+K);\
		temp2=(long)temp2*rated>>4;\
		temp1=div_q12(temp1,temp2);}


#define VDC_TRIP_TO_PU(param,fbk,flt,min)\
	temp1=MULQ_RND(12,k_fdbk_q15[fbk],pu_to_dc_volts_q12);\
	if (parameters.param > temp1) parameters.param=temp1;\
   	flts.flt.trip=MULQ_RND(12,parameters.param,dc_volts_to_pu_q12);\
	if (flts.flt.trip < min) flts.flt.trip=min;

#define PCT_TRIP_TO_PU(param,fbk,flt,min)\
	temp1=MULQ(15,k_fdbk_q15[fbk],(32767/PERCENT_TO_PU));\
	if (parameters.param > temp1) parameters.param=temp1;\
   	flts.flt.trip=parameters.param*PERCENT_TO_PU;\
	if (flts.flt.trip < min) flts.flt.trip=min;

#define CALC_AC_FAULT_DELAY(flt,param)\
	temp1=param-parameters.contactor_delay_ms;\
	if (temp1 < 0) temp1=0;\
	flts.flt.delay=temp1;


// Note that code in this module is NOT executed from RAM

/********************************************************************/
/********************************************************************/
/*		Parameter Calculation function 								*/
/********************************************************************/
                 
void parameter_calc(void)
{
   	register int temp1,temp2,i_peak,*int_pntr;
	register far int *ratio_pntr,*burden_pntr;

	power_control_mode_at_start = parameters.power_control_mode;

	if (operating_state <= STATE_STOP)
	{

/********************************************************************/
/*	Initialize parameters if requested								*/
/********************************************************************/
		if (parameters.initialize_cmd==4)
		{			// initialize rating parameters if out of range
			kw_rated_param=parameters.kw_rated;
			if ((kw_rated_param < KW_RATED_MIN)||(kw_rated_param > KW_RATED_MAX))
				kw_rated_param=KW_RATED_MIN;

			v_rated=parameters.v_line_rated;
			LIMIT_MAX_MIN(v_rated,V_RATED_MAX,V_RATED_MIN)

			v_fdbk_config=parameters.v_fdbk_config;
			if (v_fdbk_config > 2)
				v_fdbk_config=0;

			v_inv_rated_param=parameters.v_inv_rated;
			if ((v_inv_rated_param < V_RATED_MIN)||(v_inv_rated_param > V_RATED_MAX))
				v_inv_rated_param=V_INV_NOM;

			rated_Hz=parameters.freq_rated;
			if (rated_Hz!=50)
				parameters.freq_rated=rated_Hz=60;


			// initialize parameters to default values
			for (temp1=PARAM_MIN+1;temp1<PARAM_MAX;temp1++)
			{
				int_pntr=param_table[temp1].addr;
				*int_pntr=param_table[temp1].def;
			}
			parameters.kw_rated=kw_rated_param;		// restore rating parameters
			parameters.v_line_rated=v_rated;
			parameters.v_inv_rated=v_inv_rated_param;
			parameters.freq_rated=rated_Hz;
			parameters.v_fdbk_config=v_fdbk_config;
			parameters.power_control_mode = power_control_mode_at_start;

			if (parameters.kw_rated > 50)
				parameters.adjustable_spd_fan=1;
			else
				parameters.adjustable_spd_fan=0;

			ac_voltage_fdbk_ratio = 2500;
			dc_voltage_fdbk_ratio = 2500;

			// 100kW
			if(parameters.kw_rated==80 || parameters.kw_rated==100) {

				parameters.pwm_Hz = 7000;
				parameters.dead_time = 3;
				parameters.inv_bridges = 1;
				parameters.filter_inductor_uh = 300;
				parameters.filter_cap_uf = 17;
				parameters.dc_cap_uf = 960;
				parameters.number_temp_fbk = 3;
				parameters.number_strings=6;
				parameters.gnd_fault_trip_amps_x10=60;
			}

			// 125kW
			else if(parameters.kw_rated==125) {

				parameters.pwm_Hz = 7000;
				parameters.dead_time = 3;
				parameters.inv_bridges = 1;
				parameters.filter_inductor_uh = 300;
				parameters.filter_cap_uf = 48;
				parameters.dc_cap_uf = 960;
				parameters.number_temp_fbk = 3;
				parameters.dc_overvolt_inst_trip_volts=800;
				parameters.dcin_overvolt_trip_volts=775;
				parameters.dc_overvolt_trip_volts=775;

				parameters.dc_undervolt_inst_trip_volts=500;
				parameters.dcin_undervolt_trip_volts=600;
				parameters.dc_undervolt_trip_volts=600;

				parameters.dc_voltage_cmd_volts = 715;

				parameters.precharge_volts_min = 420;

				parameters.vdcin_threshold = 650;

				parameters.gnd_config = 2;

				dc_voltage_fdbk_ratio = 3300;
				parameters.number_strings=6; 
				parameters.gnd_fault_trip_amps_x10=200; // was at 60 before changed to 200

				parameters.line_undervolt_trip_fast_pct= 80;
				parameters.line_undervolt_trip_slow_pct= 80;
				parameters.line_undervolt_delay_slow_ms = 160;
				parameters.line_overvolt_trip_fast_pct=115;

				parameters.under_frequency_trip_Hz_x10=250;
				parameters.over_frequency_trip_Hz_x10= 20;

			}

			// 500kW
			else if(parameters.kw_rated==400 || parameters.kw_rated==500) {

				parameters.pwm_Hz = 5000;
				parameters.dead_time = 4;
				parameters.inv_bridges = 2;
				parameters.filter_inductor_uh = 80;// was at 300 before changed to 80
				parameters.filter_cap_uf = 133; // was at 17 before changed to 133
				parameters.dc_cap_uf = 4800;// was at 960 before changed to 4800
				parameters.number_temp_fbk = 3;
				parameters.number_strings=30;
				parameters.gnd_fault_trip_amps_x10=80;
			}

			else {
				SET_FAULT(PARAM_CHANGE_FAULT)
			}


			if(parameters.power_control_mode==PCM_DC_POWER) {
				parameters.dcin_overvolt_trip_volts=600;
				parameters.dc_overvolt_inst_trip_volts=600;
				parameters.dc_overvolt_trip_volts=600;
				parameters.precharge_volts_min=0;
				parameters.dc_voltage_cmd_volts=500;
				parameters.vdcin_threshold=0;
				parameters.vdcin_delay_6sec = 0;
				parameters.k_anti_island = 0;
				parameters.auto_reset_max_attempts = 0;
				parameters.dcin_undervolt_trip_volts = 50;
				parameters.dc_undervolt_inst_trip_volts = 50;
				parameters.dc_undervolt_trip_volts = 50;
			}
			

		}	// end parameters.initialize_cmd==4

/********************************************************************/
/*	Clear parameters if requested									*/
/********************************************************************/
		else if (parameters.initialize_cmd==5)
		{
			for (temp1=PARAM_MIN;temp1<PARAM_MAX;temp1++)
			{
				int_pntr=param_table[temp1].addr;
				*int_pntr=0;
			}
			SET_FAULT(PARAM_CHANGE_FAULT)
		}
		parameters.initialize_cmd=0;

		if(parameters.sci_slave_id<1 || parameters.sci_slave_id>255)
			parameters.sci_slave_id = 1;

/********************************************************************/
/*	Clear scaling faults											*/
/********************************************************************/
		CLEAR_FAULT(V_SCALING_FLT)
		CLEAR_FAULT(I_SCALING_FLT)
		
/********************************************************************/
/*	Determine kva rating											*/
/********************************************************************/
		LIMIT_MAX_MIN(parameters.kw_rated,KW_RATED_MAX,KW_RATED_MIN)
		kw_rated_param=parameters.kw_rated;
		voltage_derating_q12=4096+parameters.v_inv_tap_pct*(int)(4096.0/100.0+0.5);
		kva_rated=MULQ_RND(12,parameters.kw_rated,voltage_derating_q12);
		if (parameters.kw_rated==1000)		// 1 MW unit
			kva_rated=kva_rated>>1;			// actually 2x500 kva
		pu_to_kva_q12=MULQ_RND(12,kva_rated,DECIMAL_TO_PU_Q12);		

/********************************************************************/
/*	Determine line side voltage ratings								*/
/*	1 per unit defined as peak line to neutral voltage				*/
/*	v_fdbk_config=	0	line-neutral wye-delta transformer			*/
/*					1	line-line delta-delta transformer			*/
/*					2	line-line wye-delta transformer				*/
/********************************************************************/
		LIMIT_MAX_MIN(parameters.v_line_rated,V_RATED_MAX,V_RATED_MIN)
		v_rated=parameters.v_line_rated;
		if ((v_fdbk_config=parameters.v_fdbk_config)==2)
		{
			transformer_shift=THIRTY_DEGREES;
			pu_to_line_volts_q12=MULQ_RND(12,v_rated,(int)((0.5+4096.0)*(4096.0/1000)));	// 2
		}
		else if (v_fdbk_config==1) {
			transformer_shift = 0;  // GJK, 11/30/09
			pu_to_line_volts_q12=MULQ_RND(12,v_rated,(int)((0.5+4096.0)*(4096.0/1000)));	// 1
		} else
		{
			v_fdbk_config=parameters.v_fdbk_config=0;
			transformer_shift=THIRTY_DEGREES;
			pu_to_line_volts_q12=MULQ_RND(12,v_rated,(int)((0.5+4096.0/SQRT3)*(4096.0/1000)));	// 0
		}

/********************************************************************/
/*	Determine inverter side voltage ratings							*/
/*	1 per unit defined as peak line to line voltage					*/
/********************************************************************/
		LIMIT_MAX_MIN(parameters.v_inv_rated,V_RATED_MAX,V_RATED_MIN)
		v_inv_rated_param=parameters.v_inv_rated;
		rated_inv_volts=MULQ_RND(12,v_inv_rated_param,voltage_derating_q12);
		rated_dc_volts=MULQ_RND(12,rated_inv_volts,SQRT2_Q12);
		pu_to_inv_volts_q12=MULQ_RND(12,rated_inv_volts,DECIMAL_TO_PU_Q12);		
		pu_to_dc_volts_q12=MULQ_RND(12,rated_dc_volts,DECIMAL_TO_PU_Q12);
		dc_volts_to_pu_q12=div_q12(PER_UNIT,rated_dc_volts);
	
/********************************************************************/
/*	Calculate rated line current									*/
/*																	*/
/*	rated_line_amps=(kva_rated*1000/SQRT3)/v_rated			 		*/
/********************************************************************/
		temp1=MULQ_RND(12,kva_rated,(int)(0.5+1000.0*32.0/SQRT3));
		rated_line_amps=div_q12(temp1,(v_rated<<5));	// amps rms

/* 208VAC - override calculated rated line amps */
//rated_line_amps = 120;

		pu_to_line_amps_q12=MULQ_RND(12,rated_line_amps,DECIMAL_TO_PU_Q12);		
		
/********************************************************************/
/*	Calculate rated inverter current								*/
/*																	*/
/*	rated_inv_amps=(kva_rated*1000/SQRT3)/rated_inv_volts			*/
/********************************************************************/
		rated_inv_amps=div_q12(temp1,(rated_inv_volts<<5));	// rms
		pu_to_inv_amps_q12=MULQ_RND(12,rated_inv_amps,DECIMAL_TO_PU_Q12);		

/********************************************************************/
/*	Calculate rated dc link current									*/
/*																	*/
/*	i_dc_rated=kva_rated*1000/rated_dc_volts						*/
/********************************************************************/
		temp1=MULQ_RND(12,kva_rated,(1000<<5));
		rated_dc_amps=div_q12(temp1,(rated_dc_volts<<5));
		pu_to_dc_amps_q12=MULQ_RND(12,rated_dc_amps,DECIMAL_TO_PU_Q12);		
			
/********************************************************************/
/*		Determine rating code										*/
/********************************************************************/
		rating_code=0;
	
		if (kw_rated_param==80 && v_rated==265)
				rating_code=1;
		else if (kw_rated_param==100 && (v_rated==480 || v_rated==320 ))
				rating_code=2;
		else if (kw_rated_param==100 &&  (v_rated==240 || v_rated==208 ))
				rating_code=3;
		else if (kw_rated_param==125 && v_rated==400)
				rating_code=4;
		else if (kw_rated_param==400 && v_rated==265)
				rating_code=5;
		else if (kw_rated_param==500 && (v_rated==480 || v_rated==320))
				rating_code=6;	
		else 
			rating_code=0;
	
		{
			if (kw_rated_param>=500)	// 500 kw and above
			{
				ac_contactor_output	=AC_CONTACTOR_OUTPUT_2;
				precharge_output	=PRECHARGE_OUTPUT_2;
				
			}
		}
	

		number_temp_fbk = parameters.number_temp_fbk;
		number_strings=parameters.number_strings;

		if(rating_code==0)
			SET_FAULT(WRONG_RATINGS_FLT)
		else
			CLEAR_FAULT(WRONG_RATINGS_FLT)
							
/********************************************************************/
/*		Initialize digital i/o masks								*/
/********************************************************************/
		ac_contactor_input	|=AC_CONTACTOR_INPUT;
		contactor_inputs	=ac_contactor_input|DC_CONTACTOR_INPUT|ESTOP_INPUT;
		ac_contactor_output	|=AC_CONTACTOR_OUTPUT;
		precharge_output	|=PRECHARGE_OUTPUT;
		contactor_outputs	=ac_contactor_output|precharge_output|DC_CONTACTOR_OUTPUT;
		sw_fault_output		=SW_FAULT_OUTPUT;

//PJF precharge_output	|=PRECHARGE_OUTPUT_2;


/********************************************************************/
/*		Setup pointers to default parameters						*/
/********************************************************************/
		ratio_pntr=(far int*)&ifbk_ratio_table[rating_code][0];
		burden_pntr=(far int*)&ifbk_burden_table[rating_code][0];

/********************************************************************/
/*	Calculate rated string current for simulation mode				*/
/*	i_string_rated_q4=16*rated_dc_amps/number_strings				*/
/********************************************************************/
		i_string_rated_q4=div_q12(rated_dc_amps,(number_strings<<8));

/********************************************************************/
/*	Set default scaling for all channels to millivolts				*/
/********************************************************************/
		int_pntr=&k_fdbk_q15[0];
		temp1=FULL_SCALE_BP;						// bipolar inputs
		while (int_pntr < &k_fdbk_q15[CAL1])
			*int_pntr++=temp1;
		temp1=FULL_SCALE_UP;						// unipolar inputs
		while (int_pntr < &k_fdbk_q15[RAW_FDBK_SIZE])
			*int_pntr++=temp1;

/********************************************************************/
/*	line ac voltage feedback scaling in per unit					*/
/*	k_fdbk_q15=fdbk_ratio_vline_x10*FULL_SCALE_BP/(v_rated*SQRT2/SQRT3)	*/
/********************************************************************/
		if ((temp1=parameters.fdbk_ratio_vline_x10)==0)
			parameters.fdbk_ratio_vline_x10=temp1=ac_voltage_fdbk_ratio;
		if (v_fdbk_config!=0)
			temp1=MULQ_RND(12,temp1,(int)(FULL_SCALE_BP/SQRT2));
		else
			temp1=MULQ_RND(12,temp1,(int)(FULL_SCALE_BP*SQRT3/SQRT2));
		temp1=div_q12(temp1,(v_rated*10));
		k_fdbk_q15[VLC]=k_fdbk_q15[VLB]=k_fdbk_q15[VLA]=temp1;
		if (temp1 < PER_UNIT)
			SET_FAULT(V_SCALING_FLT)
		
/********************************************************************/
/*	inverter voltage feedback scaling in per unit					*/
/*	k_fdbk_q15=fdbk_ratio_vinv_x10*FULL_SCALE_BP/rated_dc_volts		*/
/********************************************************************/
		if ((temp1=parameters.fdbk_ratio_vinv_x10)==0)
			parameters.fdbk_ratio_vinv_x10=temp1=ac_voltage_fdbk_ratio;
		temp1=MULQ_RND(12,temp1,FULL_SCALE_BP);
		k_fdbk_q15[VIBC]=div_q12(temp1,(rated_dc_volts*10));
		if (k_fdbk_q15[VIBC] < PER_UNIT)
			SET_FAULT(V_SCALING_FLT)

/********************************************************************/
/*	line current feedback scaling in per unit						*/
/*	for 1 MW units ILC & ILN are used for ILA2 & ILB2, so default 	*/
/*	scaling for ILN is same as line current feedback				*/
/********************************************************************/
		i_peak=MULQ_RND(12,rated_line_amps,SQRT2_Q12);
		CALC_K_IFDBK(parameters.fdbk_ratio_iline,parameters.fdbk_burden_iline_x10,FULL_SCALE_BP,i_peak,KIL)
		k_fdbk_q15[ILN]=k_fdbk_q15[ILC]=k_fdbk_q15[ILB]=k_fdbk_q15[ILA]=temp1;
		if (temp1 < PER_UNIT)
			SET_FAULT(I_SCALING_FLT);

/********************************************************************/
/*	inverter current feedback scaling in per unit					*/
/********************************************************************/
		i_peak=MULQ_RND(12,rated_inv_amps,SQRT2_Q12);
		CALC_K_IFDBK(parameters.fdbk_ratio_iinv,parameters.fdbk_burden_iinv_x10,FULL_SCALE_BP,i_peak,KII)
		k_fdbk_q15[IIC]=k_fdbk_q15[IIB]=k_fdbk_q15[IIA]=temp1;
		if (temp1 < PER_UNIT)
			SET_FAULT(I_SCALING_FLT);
		if (parameters.inv_bridges!=2)
			temp1=0;
		k_fdbk_q15[IIC2]=k_fdbk_q15[IIB2]=k_fdbk_q15[IIA2]=temp1;

/********************************************************************/
/*	neutral current feedback scaling in amps x10					*/
/********************************************************************/
		CALC_K_IFDBK(parameters.fdbk_ratio_ineutral,parameters.fdbk_burden_ineutral_x10,FULL_SCALE_BP,(int)(PER_UNIT_F*SQRT2/10.0),KIN)
		k_fdbk_q15[ILN]=temp1;

/********************************************************************/
/*	dc link current feedback scaling in per unit					*/
/********************************************************************/
		CALC_K_IFDBK(parameters.fdbk_ratio_idc,parameters.fdbk_burden_idc_x10,FULL_SCALE_UP,rated_dc_amps,KIDC)
		k_fdbk_q15[IDC]=temp1;
		if (temp1 < PER_UNIT)
			SET_FAULT(I_SCALING_FLT);

/********************************************************************/
/*	dc link voltage feedback scaling in per unit					*/
/*	k_fdbk_q15=fdbk_ratio_vdc_x10*FULL_SCALE_UP/rated_dc_volts		*/
/********************************************************************/
		if ((temp1=parameters.fdbk_ratio_vdc_x10)==0)
		{
			temp1=dc_voltage_fdbk_ratio;
			parameters.fdbk_ratio_vdc_x10=temp1;
		}
		temp1=MULQ_RND(12,temp1,FULL_SCALE_UP);
		k_fdbk_q15[VDC]=div_q12(temp1,(rated_dc_volts*10));
		if (k_fdbk_q15[VDC] < PER_UNIT)
			SET_FAULT(V_SCALING_FLT)

/********************************************************************/
/*	dc input voltage feedback scaling in per unit					*/
/*	k_fdbk_q15=fdbk_ratio_vdcin_x10*FULL_SCALE_UP/rated_dc_volts	*/
/********************************************************************/
		if ((temp1=parameters.fdbk_ratio_vdcin_x10)==0)
		{
			temp1=dc_voltage_fdbk_ratio;
			parameters.fdbk_ratio_vdcin_x10=temp1;
		}
		temp1=MULQ_RND(12,temp1,FULL_SCALE_UP);
		k_fdbk_q15[VDCIN]=div_q12(temp1,(rated_dc_volts*10));
		if (k_fdbk_q15[VDCIN] < PER_UNIT)
			SET_FAULT(V_SCALING_FLT)


	/********************************************************************/
	/*	ground current feedback scaling	in amps x10						*/
	/********************************************************************/
	{
		CALC_K_IFDBK(parameters.fdbk_ratio_ignd,parameters.fdbk_burden_ignd_x10,FULL_SCALE_BP,(PER_UNIT/10),KIGND)
		k_fdbk_q15[GND]=temp1;
	}

	/* ADC B15 is either FUSEV or DCIMB depending on system configuration  */
	if(parameters.gnd_config==NEGATIVE_GND || parameters.gnd_config==POSITIVE_GND) {
		temp1=MULQ_RND(12,parameters.fdbk_ratio_vdcin_x10,FULL_SCALE_BP);
		k_fdbk_q15[FUSEV]=div_q12(temp1,(rated_dc_volts*10));
	}
	else {
		/* 5ma = 1000PU */
		/* 5mA = 31500 raw ADC (with 2 turns) */
		/* k_q15 = 2^15 * 1000 / 30200 = 1085 */
		k_fdbk_q15[DCIMB]=1085;
	}

	/* ADC U21 - Bender impedance monitoring - millivolts */
	{
		k_fdbk_q15[BENDER] = FULL_SCALE_UP;
	}
	
/********************************************************************/
/*	Determine PWM frequency											*/
/********************************************************************/
		LIMIT_MAX_MIN(parameters.pwm_Hz,8000,4000)
		temp1=parameters.pwm_Hz;
		pwm_period=(unsigned long)(LSPCLK)/temp1;
		parameters.pwm_Hz=pwm_Hz=(unsigned long)(LSPCLK)/pwm_period;
		pwm_int_us=MULQ(15,(pwm_period>>1),(int)((32768.0E6+LSPCLK/2.0)/LSPCLK));
		
		pll_freq_to_Hz_q27=MULQ_RND(12,pwm_Hz,(int)((1<<(FREQ_SHIFT-4))*100));
		pll_Hz_long=0xFFFFFFFF/(unsigned)pwm_Hz;

/********************************************************************/
/*	Factor to convert from per unit to pwm modulator scaling		*/
/********************************************************************/
		pwm_cmd_scale=MULQ_RND(12,pwm_period,(int)(4096.0*4096.0/(2.0*PER_UNIT_F)));

/********************************************************************/
/*	Determine nominal frequency (50 or 60 Hz only)					*/
/********************************************************************/
		rated_Hz=parameters.freq_rated;
		if (rated_Hz!=50)
			parameters.freq_rated=rated_Hz=60;
		line_freq_rated=rated_Hz*pll_Hz_long;
		sample_count_max=div_q12((pwm_Hz>>3),((rated_Hz-10)<<9));
		sampling_shift=div_q12(rated_Hz,pwm_Hz);

/********************************************************************/
/*	Calculate ac filter constants									*/
/********************************************************************/
		#define H_AC_FLTR		10	// harmonic of rated frequency
		temp1=(4096+div_q12(pwm_Hz,((int)(H_AC_FLTR*2.0*PI+0.5)*rated_Hz)))>>1;
		k_ac_fltr_q16=div_q12(32767,temp1);
		temp1=4096+div_q12(1,(H_AC_FLTR*H_AC_FLTR));
		ac_fltr_gain_comp=long_sqrt((long)temp1<<12);
		// asin=atan for small angles
		ac_fltr_phase_shift=asin_table[div_q12(1,(1+H_AC_FLTR))];

/********************************************************************/
/*	Calculate per unit filter inductance							*/
/*	pu_uh=1e6*(rated_inv_volts/SQRT3)/(rated_inv_amps*rated_Hz*2*PI)*/
/*	l_pu=PER_UNIT*filter_inductor_uh/pu_uh							*/
/*	k_l_q16=65536*l_pu/PER_UNIT										*/
/********************************************************************/

		temp2 = parameters.filter_inductor_uh;
		
//		if (temp2==0)
//			parameters.filter_inductor_uh=temp2=filter_uh_table[rating_code];
		temp1=(long)rated_inv_amps*rated_Hz>>6;
		temp1=(long)temp1*temp2>>6;
		temp2=rated_inv_volts*(int)(1e6/(PER_UNIT_F*2.0*PI*SQRT3));
		l_pu=div_q12(temp1,temp2);
		
		k_l_q16=l_pu*(int)(65536.0/PER_UNIT_F+0.5);

/********************************************************************/
/*	Calculate per unit ac filter capacitance						*/
/*	pu_uf=1e6*(rated_inv_amps)/((rated_inv_volts/SQRT3)*rated_Hz*2*PI)	*/
/*	cac_pu=PER_UNIT*filter_cap_uf/pu_uf								*/
/*	k_cac_q16=65536*cdc_pu/PER_UNIT									*/
/********************************************************************/

		temp2=(parameters.filter_cap_uf)*3;

		temp1=(long)(rated_inv_volts*rated_Hz)>>6;
		temp1=MULQ_RND(12,temp1,temp2);	// (rated_inv_volts*rated_Hz*filter_cap_uf)>>18
		temp2=MULQ_RND(12,rated_inv_amps,(int)(4.096*1e6/(64.0*2.0*PI/SQRT3)));
		temp2=div_q12(temp1,temp2);

		k_cac_q16=temp2*(int)(65536.0/PER_UNIT_F+0.5);

/********************************************************************/
/*	Calculate per unit dc link capacitance							*/
/*	pu_uf=1e6*(rated_inv_amps)/((rated_inv_volts/SQRT3)*rated_Hz*2*PI)	*/
/*	cdc_pu=PER_UNIT*dc_cap_uf/pu_uf									*/
/*	k_cdc_q16=65536*rated_Hz/(pwm_Hz*cdc_pu)						*/
/********************************************************************/
		temp2=parameters.dc_cap_uf;
	//	if (temp2==0)
	//		parameters.dc_cap_uf=temp2=dc_cap_uf_table[rating_code];
		temp1=(long)(rated_inv_volts*rated_Hz)>>6;
		temp1=MULQ_RND(12,temp1,temp2);	// (rated_inv_volts*rated_Hz*dc_cap_uf)>>18
		temp2=MULQ_RND(12,rated_inv_amps,(int)(4.096*1e6/(64.0*2.0*PI/SQRT3)));
		temp2=div_q12(temp1,temp2);
		
		temp1=div_q12(rated_Hz*(PER_UNIT/4),pwm_Hz);
		k_cdc_q16=div_q12(temp1,(temp2*64));

/********************************************************************/
/*	Initialize miscellaneous variables								*/
/********************************************************************/
   		real_power_cmd_pntr=&real_power_cmd_pu;
   		reactive_power_cmd_pntr=&reactive_power_cmd_pu;
		vdc_in_timer_100ms=(10*60*10);		// 10 minutes
		ext_input_faults=ext_output_faults=gating_faults=serial_comm_faults=~0;
		analog_input_faults=~0;
	}
   
/********************************************************************/
/*	Initialize remaining parameters if not done yet					*/
/********************************************************************/
	while (!(status_flags & STATUS_INITIALIZED))
		slow_param_update();
}


#include "slow_update.c"
#include "init_tables.h"
