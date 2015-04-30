/********************************************************************/
/********************************************************************/
/*																	*/
/*						Parameter Table module						*/
/*																	*/
/********************************************************************/
/********************************************************************/

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
2008-03-00	add ext_input_faults,ext_output_faults,gating_faults,analog_input_faults
2008-03-25	correct error in declaration of i_inv_pu
2008-03-27	change a number of modbus access levels from 3 to 2
2008-05-08	add serial_comm_faults
2008-08-19	add i_line_neg
2008-08-21	replace low_power_timer_sec with mppt_timer_cycles
2008-09-03	replace old_dc_power_pu with mppt_power_q16
2009-01-15	add variable vdc_comp_q12
2009-01-19	add variable global_flags
2009-02-26	add variables gnd_impedance_norm and i_leakage_ma
2009-04-17	delete voltage regulator step
2009-06-22	delete variable mppt_power_q16
2009-07-20	delete variable global_flags
2009-07-22	add separate access levels for read and write
			change read access levels of many parameters
2009-08-17	change read access levels of input_kw from 2 to 0
*/
#include "literals.h"
#include "struct.h"
#include "faults.h"

#define DUMMY_VARIABLE	&dummy_variable,RD3|RD_ONLY,0,0,0,

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern struct ABCDQM v_cmd_pu,i_inv_pu;
extern struct DQ i_cmd_inv,inv_error,int_term,prop_term,v_fdfwd;
extern struct DQ i_cmd_line,line_error,v_line_pu;
extern struct DQM i_line_neg;
extern int playback[8],playback_index;
extern int pc_cmd,remote_cmd_param,operating_state,mppt_timer_cycles;
extern int ext_inputs,fpga_inputs,ext_outputs_display,fpga_outputs,pwm_flt_inputs;
extern int faults[FAULT_WORDS],number_faults,a2d_offset_mv,a2d_gain_q12;
extern int fdbk_pu[FDBK_SIZE],fdbk_display[FDBK_SIZE],fdbk_avg[FDBK_SIZE];
extern int string_amps[32],temp_fbk_mv_display;
extern int v_line_alpha,v_line_beta;
extern int vdc_cmd_pu,vdc_error_pu,voltage_derating_q12;
extern long vdc_ref_q16;
extern int output_test,ext_input_mask,gate_test_select;
extern int pll_output_angle_deg,pll_input_angle_deg;
extern int pll_freq_error_Hz,pll_error_deg,status_flags;
extern int id_step,iq_step,id_ref_max;
extern int trig_state,trig_mode,trig_index;
extern int iq_cmd_anti_island,iq_ref,thermal_limit,vdc_comp_q12;
extern int vdc_in_timer_6sec,line_voltage_timer_sec;
extern int testmode_param,dummy_variable;
extern int save_command,fan_spd_cmd_out[2];
extern int rated_line_amps,rated_inv_amps,rated_dc_volts,rated_dc_amps;
extern int temp_fbk[8],heatsink_temp_max[2];
extern struct PARAMS parameters;
extern struct RTC_PACKED display_time;
extern struct PCS_OP comm;
extern struct ABC test_input,v_cmd_test;
extern struct SAVED_VARS saved_variables;
extern struct CHKSUMS checksums;
extern struct QUEUE_ENTRY display_fault;
extern struct PWR_CURVE power_curve;
extern int set_clock,calibrate_clock,fpga_version,software_version,build_number;
extern int fault_queue_index,fan_spd_cmd_in,gate_test_param;
extern int pv_curve_data_volts,pv_curve_data_power;
extern int ext_input_faults,ext_output_faults,gating_faults,serial_comm_faults;
extern int total_dc_power,gnd_impedance_norm,dc_bus_imbalance_ma_x10,dc_ground_amps;
extern long analog_input_faults;
extern int debug[4],fdbk_raw_mv;
// variables in engineering units from slow_task
extern int string_amps_avg,string_kwh_avg;
extern int input_kw,gnd_impedance;
extern struct AC_PWR output_power;
extern struct ABCM line_volts,line_amps,inverter_amps;
extern struct ABC inverter2_amps;
extern int inverter_volts_bc,power_change_timer_cycles;
extern int dc_link_amps,dc_link_volts,dc_input_volts,vdc_r_avg,idc_r_avg;
extern int wh_total,kwh_yesterday,kwh_data,kwh_month_day,kwh_index;
extern int vdc_step;
extern int kwh_total_7days,kwh_avg_7days,kwh_total_30days,kwh_avg_30days;
extern int fuse_volts;
extern int debug_control;
extern int junk;

/********************************************************************/
/********************************************************************/
/*	Parameter Table													*/
/********************************************************************/
#pragma DATA_SECTION(param_table,".rom_const")


/********************************************************************/
/*	Read only variables (0-299)										*/
/*	Modbus register number is 1 higher than parameter number		*/
/********************************************************************/
const far struct PARAM_ENTRY param_table[PARAM_MAX+1]={
		DUMMY_VARIABLE
		&playback[0],						RD3|RD_ONLY,14,0,0,
		&playback[1],						RD3|RD_ONLY,0,0,0,
		&playback[2],						RD3|RD_ONLY,0,0,0,
		&playback[3],						RD3|RD_ONLY,0,0,0,
		&playback[4],						RD3|RD_ONLY,0,0,0,
		&playback[5],						RD3|RD_ONLY,0,0,0,
		&playback[6],						RD3|RD_ONLY,0,0,0,
		&playback[7],						RD3|RD_ONLY,0,0,0,
		&software_version,					RD0|RD_ONLY,0,0,0,
// faults (10-19)
		&faults[0],							RD0|RD_ONLY,0,0,0,
		&faults[1],							RD0|RD_ONLY,0,0,0,
		&faults[2],							RD0|RD_ONLY,0,0,0,
		&faults[3],							RD0|RD_ONLY,0,0,0,
		&faults[4],							RD0|RD_ONLY,0,0,0,
		&faults[5],							RD0|RD_ONLY,0,0,0,
		&faults[6],							RD0|RD_ONLY,0,0,0,
		&number_faults,						RD0|RD_ONLY,0,0,0,
		(int*)&checksums.code,				RD0|RD_ONLY,0,0,0,
		(int*)&checksums.params,			RD0|RD_ONLY,0,0,0,
// meters (20-49)
		&dc_input_volts,					RD0|RD_ONLY,0,0,0,
		&dc_link_volts,     				RD0|RD_ONLY,0,0,0,
		&dc_link_amps,      				RD0|RD_ONLY,0,0,0,
		&dc_ground_amps,					RD0|RD_ONLY,0,0,0,
		&inverter_amps.a,   				RD3|RD_ONLY,0,0,0,
		&inverter_amps.b,   				RD3|RD_ONLY,0,0,0,
		&inverter_amps.c,   				RD3|RD_ONLY,0,0,0,
		&inverter2_amps.a,  				RD3|RD_ONLY,0,0,0,
		&inverter2_amps.b,  				RD3|RD_ONLY,0,0,0,
		&inverter2_amps.c,  				RD3|RD_ONLY,0,0,0,

		&inverter_amps.m,   				RD3|RD_ONLY,0,0,0,
		&inverter_volts_bc,					RD3|RD_ONLY,0,0,0,
		&line_amps.a,       				RD0|RD_ONLY,0,0,0,
		&line_amps.b,       				RD0|RD_ONLY,0,0,0,
		&line_amps.c,       				RD0|RD_ONLY,0,0,0,
		&line_amps.m,       				RD0|RD_ONLY,0,0,0,
		&fdbk_display[ILN], 				RD0|RD_ONLY,0,0,0,
		&line_volts.a,      				RD0|RD_ONLY,0,0,0,
		&line_volts.b,      				RD0|RD_ONLY,0,0,0,
		&line_volts.c,      				RD0|RD_ONLY,0,0,0,

		&line_volts.m,      				RD0|RD_ONLY,0,0,0,
		&fdbk_display[VLUB],				RD0|RD_ONLY,0,0,0,
		&fdbk_display[ILUB],				RD0|RD_ONLY,0,0,0,
		&input_kw,							RD0|RD_ONLY,0,0,0,
		&output_power.kw,					RD0|RD_ONLY,0,0,0,
		&output_power.kvar,					RD0|RD_ONLY,0,0,0,
		&output_power.kva,					RD0|RD_ONLY,0,0,0,
		&output_power.pf,					RD0|RD_ONLY,0,0,0,
		&fdbk_raw_mv,						RD3|RD_ONLY,0,0,0,
		&gnd_impedance,						RD0|RD_ONLY,0,0,0,

// string currents (50-89)
		&string_amps[0],					RD0|RD_ONLY,0,0,0,
		&string_amps[1],					RD0|RD_ONLY,0,0,0,
		&string_amps[2],					RD0|RD_ONLY,0,0,0,
		&string_amps[3],					RD0|RD_ONLY,0,0,0,
		&string_amps[4],					RD0|RD_ONLY,0,0,0,
		&string_amps[5],					RD0|RD_ONLY,0,0,0,
		&string_amps[6],					RD0|RD_ONLY,0,0,0,
		&string_amps[7],					RD0|RD_ONLY,0,0,0,
		&string_amps[8],					RD0|RD_ONLY,0,0,0,
		&string_amps[9],					RD0|RD_ONLY,0,0,0,
		&string_amps[10],					RD0|RD_ONLY,0,0,0,
		&string_amps[11],					RD0|RD_ONLY,0,0,0,
		&string_amps[12],					RD0|RD_ONLY,0,0,0,
		&string_amps[13],					RD0|RD_ONLY,0,0,0,
		&string_amps[14],					RD0|RD_ONLY,0,0,0,
		&string_amps[15],					RD0|RD_ONLY,0,0,0,
		&string_amps[16],					RD0|RD_ONLY,0,0,0,
		&string_amps[17],					RD0|RD_ONLY,0,0,0,
		&string_amps[18],					RD0|RD_ONLY,0,0,0,
		&string_amps[19],					RD0|RD_ONLY,0,0,0,
		&string_amps[20],					RD0|RD_ONLY,0,0,0,
		&string_amps[21],					RD0|RD_ONLY,0,0,0,
		&string_amps[22],					RD0|RD_ONLY,0,0,0,
		&string_amps[23],					RD0|RD_ONLY,0,0,0,
		&string_amps[24],					RD0|RD_ONLY,0,0,0,
		&string_amps[25],					RD0|RD_ONLY,0,0,0,
		&string_amps[26],					RD0|RD_ONLY,0,0,0,
		&string_amps[27],					RD0|RD_ONLY,0,0,0,
		&string_amps[28],					RD0|RD_ONLY,0,0,0,
		&string_amps[29],					RD0|RD_ONLY,0,0,0,
		&string_amps[30],					RD0|RD_ONLY,0,0,0,
		&string_amps[31],					RD0|RD_ONLY,0,0,0,
		DUMMY_VARIABLE
		DUMMY_VARIABLE
		&fdbk_pu[TMP],						RD3|RD_ONLY,0,0,0,
//		&mppt_timer_cycles,					RD3|RD_ONLY,0,0,0,
		DUMMY_VARIABLE
		DUMMY_VARIABLE
		&vdc_cmd_pu,						RD3|RD_ONLY,0,0,0,
//		&power_change_timer_cycles,			RD3|RD_ONLY,0,0,0,
		DUMMY_VARIABLE
		&string_amps_avg,					RD0|RD_ONLY,0,0,0,

// string kw hours (90-129)
		(int*)&saved_variables.string_kw_hrs[0],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[1],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[2],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[3],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[4],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[5],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[6],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[7],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[8],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[9],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[10],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[11],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[12],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[13],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[14],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[15],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[16],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[17],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[18],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[19],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[20],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[21],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[22],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[23],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[24],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[25],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[26],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[27],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[28],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[29],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[30],		RD0|RD_ONLY,0,0,0,
		(int*)&saved_variables.string_kw_hrs[31],		RD0|RD_ONLY,0,0,0,
		(int*)&power_curve.date[0],			RD0|RD_ONLY,0,0,0,
		(int*)&power_curve.date[1],			RD0|RD_ONLY,0,0,0,
		(int*)&power_curve.date[2],			RD0|RD_ONLY,0,0,0,
//		&pv_curve_data_volts,				RD0|RD_ONLY,0,0,0,
//		&pv_curve_data_power,				RD0|RD_ONLY,0,0,0,
		DUMMY_VARIABLE
		DUMMY_VARIABLE
		&kwh_month_day,						RD0|RD_ONLY,0101,0101,1231,
		&kwh_data,							RD0|RD_ONLY,0,0,0,
		&string_kwh_avg,					RD0|RD_ONLY,0,0,0,

// energy (130-139)
		&wh_total,	 						RD0|RD_ONLY,0,0,999,
		(int*)&saved_variables.kwh_total,	RD0|RD_ONLY,0,0,999,
		(int*)&saved_variables.mwh_total,	RD0|RD_ONLY,0,0,32767,
		(int*)&saved_variables.kwh_today,	RD0|RD_ONLY,0,0,32767,
		&kwh_yesterday,						RD0|RD_ONLY,0,0,32767,
		&kwh_total_7days,					RD0|RD_ONLY,0,0,0,
		&kwh_total_30days,					RD0|RD_ONLY,0,0,0,
		&kwh_avg_7days,						RD0|RD_ONLY,0,0,0,
		&kwh_avg_30days,					RD0|RD_ONLY,0,0,0,
		DUMMY_VARIABLE

// inverter feedback average (140-159)
		&fdbk_display[VDCIN],				RD3|RD_ONLY,0,0,0,
		&fdbk_display[VDC],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[IDC],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[II],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[IIA],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[IIB],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[IIC],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[IIA2],				RD3|RD_ONLY,0,0,0,
		&fdbk_display[IIB2],				RD3|RD_ONLY,0,0,0,
		&fdbk_display[IIC2],				RD3|RD_ONLY,0,0,0,
		&fdbk_display[VIBC],				RD3|RD_ONLY,0,0,0,
		&debug[0],							RD3|RD_ONLY,0,0,0,
		&debug[1],							RD3|RD_ONLY,0,0,0,
		&debug[2],							RD3|RD_ONLY,0,0,0,
		&debug[3],							RD3|RD_ONLY,0,0,0,
		&fdbk_display[PDC],					RD3|RD_ONLY,0,0,0,
		DUMMY_VARIABLE
		DUMMY_VARIABLE
		DUMMY_VARIABLE
		&fuse_volts,						RD0|RD_ONLY,0,0,0,

// line feedback average (160-179)
		&fdbk_display[ILA],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[ILB],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[ILC],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[IL],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[ILN],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[VLA],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[VLB],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[VLC],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[VL],					RD3|RD_ONLY,0,0,0,
		&i_line_neg.m,						RD3|RD_ONLY,0,0,0,

		&total_dc_power,					RD3|RD_ONLY,0,0,0,
		&gnd_impedance_norm,				RD0|RD_ONLY,0,0,0,
		&dc_bus_imbalance_ma_x10,			RD0|RD_ONLY,0,0,0,
		&fdbk_display[ID],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[IQ],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[FRQ],					RD0|RD_ONLY,0,0,0,
		&fdbk_display[FRQERR],				RD0|RD_ONLY,0,0,0,
		&fdbk_display[KW],					RD3|RD_ONLY,0,0,0,
		&fdbk_display[KVAR],				RD3|RD_ONLY,0,0,0,
		&fdbk_display[KVA],					RD3|RD_ONLY,0,0,0,

// inverter feedback instantaneous (180-199)
		&fdbk_pu[VDCIN],					RD3|RD_ONLY,0,0,0,
		&fdbk_pu[VDC],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[IDC],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[II],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[IIA],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[IIB],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[IIC],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[IIA2],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[IIB2],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[IIC2],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[VIBC],						RD3|RD_ONLY,0,0,0,
		&voltage_derating_q12,				RD3|RD_ONLY,0,0,0,
		DUMMY_VARIABLE
		&i_inv_pu.d,						RD3|RD_ONLY,0,0,0,
		&i_inv_pu.q,						RD3|RD_ONLY,0,0,0,
		&fdbk_avg[PDC],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[EX1],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[EX2],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[EX3],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[EX4],						RD3|RD_ONLY,0,0,0,

// line feedback instantaneous (200-219)
		&fdbk_pu[ILA],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[ILB],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[ILC],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[IL],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[ILN],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[VLA],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[VLB],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[VLC],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[VL],						RD3|RD_ONLY,0,0,0,
		&v_line_alpha,						RD3|RD_ONLY,0,0,0,
		&v_line_beta,						RD3|RD_ONLY,0,0,0,
		&v_line_pu.d,						RD3|RD_ONLY,0,0,0,
		&v_line_pu.q,						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[ID],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[IQ],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[FRQ],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[FRQERR],					RD3|RD_ONLY,0,0,0,
		&fdbk_pu[KW],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[KVAR],						RD3|RD_ONLY,0,0,0,
		&fdbk_pu[KVA],						RD3|RD_ONLY,0,0,0,

// voltage regulator (220-229)
		(int*)&vdc_ref_q16+1,				RD3|RD_ONLY,0,0,0,
		&fdbk_pu[VDC],						RD3|RD_ONLY,0,0,0,
		&vdc_error_pu,						RD3|RD_ONLY,0,0,0,
//		&vdc_step,							RD3|RD_ONLY,0,0,0,
		DUMMY_VARIABLE
		DUMMY_VARIABLE
		&vdc_comp_q12,						RD3|RD_ONLY,0,0,0,
		&pll_input_angle_deg,				RD3|RD_ONLY,0,0,0,
		&pll_output_angle_deg,				RD3|RD_ONLY,0,0,0,
		&iq_cmd_anti_island,				RD3|RD_ONLY,0,0,0,
		&thermal_limit,						RD3|RD_ONLY,0,0,0,

// inverter current regulator (230-239)
		&i_cmd_inv.d,						RD3|RD_ONLY,0,0,0,
		&i_cmd_inv.q,						RD3|RD_ONLY,0,0,0,
		&i_inv_pu.d,						RD3|RD_ONLY,0,0,0,
		&i_inv_pu.q,						RD3|RD_ONLY,0,0,0,
		&inv_error.d,						RD3|RD_ONLY,0,0,0,
		&inv_error.q,						RD3|RD_ONLY,0,0,0,
		&prop_term.d,						RD3|RD_ONLY,0,0,0,
		&prop_term.q,						RD3|RD_ONLY,0,0,0,
		&int_term.d,						RD3|RD_ONLY,0,0,0,
		&int_term.q,						RD3|RD_ONLY,0,0,0,

// line current regulator (240-249)
		&i_cmd_line.d,						RD3|RD_ONLY,0,0,0,
		&i_cmd_line.q,						RD3|RD_ONLY,0,0,0,
		&fdbk_avg[ID],						RD3|RD_ONLY,0,0,0,
		&fdbk_avg[IQ],						RD3|RD_ONLY,0,0,0,
		&line_error.d,						RD3|RD_ONLY,0,0,0,
		&line_error.q,						RD3|RD_ONLY,0,0,0,
		(int*)&analog_input_faults,			RD3|RD_ONLY,0,0,0,
		(int*)&analog_input_faults+1,		RD3|RD_ONLY,0,0,0,
		&ext_input_faults,					RD3|RD_ONLY,0,0,0,
		&ext_output_faults,					RD3|RD_ONLY,0,0,0,

// voltage commands (250-259)
		&v_fdfwd.d,							RD3|RD_ONLY,0,0,0,
		&v_fdfwd.q,							RD3|RD_ONLY,0,0,0,
		&v_cmd_pu.d,						RD3|RD_ONLY,0,0,0,
		&v_cmd_pu.q,						RD3|RD_ONLY,0,0,0,
		&serial_comm_faults,				RD3|RD_ONLY,0,0,0,
		&gating_faults,						RD3|RD_ONLY,0,0,0,
		&v_cmd_pu.a,						RD3|RD_ONLY,0,0,0,
		&v_cmd_pu.b,						RD3|RD_ONLY,0,0,0,
		&v_cmd_pu.c,						RD3|RD_ONLY,0,0,0,
		DUMMY_VARIABLE

// ratings (260-265)
		&rated_line_amps,					RD3|RD_ONLY,0,0,0,
		&rated_inv_amps,					RD3|RD_ONLY,0,0,0,
		&rated_dc_amps,						RD3|RD_ONLY,0,0,0,
		&rated_dc_volts,					RD3|RD_ONLY,0,0,0,
		&build_number,						RD0|RD_ONLY,0,0,0,
		&fpga_version,						RD0|RD_ONLY,0,0,0,

// fault_queue (266-269)
		(int*)&display_fault.number,		RD1|RD_ONLY,0,0,0,
		(int*)&display_fault.time[0],		RD1|RD_ONLY,0,0,0,
		(int*)&display_fault.time[1],		RD1|RD_ONLY,0,0,0,
		(int*)&display_fault.time[2],		RD1|RD_ONLY,0,0,0,

// input/output (270-279)
		&ext_inputs,						RD3|RD_ONLY,0,0,0,
		DUMMY_VARIABLE
		&ext_outputs_display,				RD3|RD_ONLY,0,0,0,
		&fpga_inputs,						RD3|RD_ONLY,0,0,0,
		&fpga_outputs,						RD3|RD_ONLY,0,0,0,
		&vdc_in_timer_6sec,					RD0|RD_ONLY,0,0,0,
		&line_voltage_timer_sec,			RD0|RD_ONLY,0,0,0,
		&pwm_flt_inputs,					RD3|RD_ONLY,0,0,0,
		&status_flags,						RD3|RD_ONLY,0,0,0,
		&operating_state,					RD0|RD_ONLY,0,0,0,

// temperature feedback (280-289)
		&temp_fbk[0],						RD0|RD_ONLY,0,0,0,
		&temp_fbk[1],						RD0|RD_ONLY,0,0,0,
		&temp_fbk[2],						RD0|RD_ONLY,0,0,0,
		&temp_fbk[3],						RD0|RD_ONLY,0,0,0,
		&temp_fbk[4],						RD0|RD_ONLY,0,0,0,
		&temp_fbk[5],						RD0|RD_ONLY,0,0,0,
		&temp_fbk[6],						RD0|RD_ONLY,0,0,0,
		&temp_fbk[7],						RD0|RD_ONLY,0,0,0,
		&heatsink_temp_max[0],				RD0|RD_ONLY,0,0,0,
		&fan_spd_cmd_out[0],				RD0|RD_ONLY,0,0,0,

//	misc (290-299)		
		&heatsink_temp_max[1],				RD0|RD_ONLY,0,0,0,
		&fan_spd_cmd_out[1],				RD0|RD_ONLY,0,0,0,
		&pll_error_deg,						RD3|RD_ONLY,0,0,0,
		&pll_freq_error_Hz,					RD3|RD_ONLY,0,0,0,
		&fdbk_display[CAL1],				RD3|RD_ONLY,0,0,0,
		&fdbk_display[CAL2],				RD3|RD_ONLY,0,0,0,
		&fdbk_display[CAL3],				RD3|RD_ONLY,0,0,0,
		&fdbk_display[CAL4],				RD3|RD_ONLY,0,0,0,
		&a2d_offset_mv,						RD3|RD_ONLY,0,0,0,
		&a2d_gain_q12,						RD3|RD_ONLY,0,0,0,

#include "write_params.c"
