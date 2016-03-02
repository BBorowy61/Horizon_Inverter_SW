/********************************************************************/
/********************************************************************/
/*																	*/
/*						Structure Declarations						*/
/*																	*/
/********************************************************************/
/********************************************************************/
 
/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
2008-03-00	add parameters a2d_error_mv,v_fdbk_delta
2008-05-30	add language parameter
2008-06-27	change fdbk_offset_vdcin_volts to fdbk_offset_idc_amps
2008-08-07	change power_change_delay_100ms to mppt_interval_6sec
2008-08-18	add inverter_type parameter
2008-09-17	replace heat_xchg_temp_ref with fan_speed _min
2008-10-10	add inverter undervoltage fault
2008-10-28	replace mppt_factor with vdc_step_time_100ms
2008-11-21	add slave_id parameters for other serial ports
2008-12-03	add inverter_id parameter
2008-12-10	add comm_master_id parameter
2009-01-14	add k_voltffd parameter
2009-01-23	add line instantaneous overvoltage fault and parameters
2009-02-17	add 9 new ground fault parameters
			comment out low temperature faults to conserve memory
2009-03-06	add parameters vdcin_threshold_2 and vdcin_delay_2_6sec
2009-03-20	add parameter k_i_leakage_q8
2009-03-23	delete parameter regulator_flags
2009-05-06	add Solstice power control parameters
2009-05-07	add parameter gnd_config
2009-06-17	add 50 parameters for future use
2009-10-26	add parameter precharge_config
*/
/********************************************************************/
/* Structure definitions											*/
/********************************************************************/

struct FLT {int trip; int delay; int timer;};
struct FLTS	{	struct FLT dcin_ov;
				struct FLT dcin_uv;
				struct FLT dcin_oc;
				struct FLT dcin_oc_inst;
				struct FLT dc_ov;
				struct FLT dc_ov_inst;
				struct FLT dc_uv;
				struct FLT dc_uv_inst;
				struct FLT gnd_flt;
				struct FLT inv_uv;
				struct FLT inv_oc;
				struct FLT overfreq;
				struct FLT underfreq;
				struct FLT underfreq_inst;
				struct FLT line_oc;
				struct FLT line_oc_neutral;
				struct FLT line_ov_inst;
				struct FLT line_ov_fast;
				struct FLT line_ov_slow;
				struct FLT line_uv_inst;
				struct FLT line_uv_fast;
				struct FLT line_uv_slow;
				struct FLT v_unbalance;
				struct FLT i_unbalance;
				struct FLT idc_diff;
				struct FLT iinv_diff;
				struct FLT temp_fbk;
				struct FLT high_temp[8];
				struct FLT gnd_impedance;
				struct FLT dc_bus_imbalance;
				struct FLT fuse_voltage;
			};
struct ABC {int a; int b; int c;};
struct ABCM {int a; int b; int c; int m;};
struct ABCDQM {int a; int b; int c; int d; int q; int m;};
struct DQ {int d; int q;};
struct DQ_LONG {long d; long q;};
struct DQM {int d; int q; int m;};
struct PARAM_ENTRY {int *addr; unsigned attrib; int def; int min; int max;};
struct QUEUE_ENTRY {unsigned number; unsigned time[3];};
struct FAULT_QUEUE {unsigned int index; struct QUEUE_ENTRY entry[FAULT_QUEUE_SIZE];};
struct RTC_PACKED {unsigned second_hundredth; unsigned hour_minute; unsigned month_day; unsigned year;};
struct AC_PWR {int kw; int kvar; int kva; int pf;};
struct CHKSUMS {unsigned code; unsigned saved_vars; unsigned params;unsigned text;};
struct PWR_CURVE_DATA {int vdc; int power;};
struct PWR_CURVE {unsigned date[3]; struct PWR_CURVE_DATA data[PWR_CURVE_SIZE+1];};

struct DATA_SETUP	{int channel[DATA_CHANNELS];
					int interval;
					int trig_channel;
					int pre_trigger;
					int condition;
					int level;
					int mode;
					int spare7;
					int spare8;};
struct DATA_RECORD 	{int data[DIAG_SIZE][DATA_CHANNELS];};

struct PCS_OP		{unsigned data[128];
					unsigned *addr[126];
					unsigned save_command;
					unsigned commnum;};
struct SCI_DATA		{unsigned buffer[MAXLEN+2];
					unsigned indata;
					int tx_state,rx_state,access_level;
					unsigned index,last_data_byte,function_code,wait_timer;};

struct STATUS_MESSAGE
{
	unsigned char text[SIZE_STATUS_MSG];
};
struct FAULT_MESSAGE
{
	unsigned char text[SIZE_FAULT_MSG];
};

/********************************************************************/
/*	Saved Parameters (256 words)									*/
/********************************************************************/
// declared as a structure because compiler will not rearrange 
// variables within a structure

struct PARAMS {
/********************************************************************/
/*		serial number (300-309)	[000-009]							*/
/********************************************************************/
int param_start;			// not used
int serial_number[9];

/********************************************************************/
/*		ratings (310-319) [010-019]									*/
/********************************************************************/
int kw_rated;
int freq_rated;
int v_line_rated;
int v_inv_rated;
int	v_inv_tap_pct;
int	v_fdbk_config;
int inverter_type;
int gnd_config;
int filler18; 
int filler19;

/********************************************************************/
/*		components (320-329) [020-029]								*/
/********************************************************************/
int filter_inductor_uh;
int filter_cap_uf;
int filler22;
int number_temp_fbk;
int inv_bridges;
int pwm_Hz;
int dc_cap_uf;
int number_strings;
int dead_time;
int IGBT_current_max;

/********************************************************************/
/*		dc feedback scaling (330-339) [030-039]						*/
/********************************************************************/
int fdbk_ratio_istring;
int fdbk_burden_istring_x10;
int fdbk_ratio_idc;
int fdbk_burden_idc_x10;
int fdbk_ratio_ignd;
int fdbk_burden_ignd_x10;
int fdbk_ratio_vdcin_x10;
int fdbk_ratio_vdc_x10;
int fdbk_offset_idc_amps;
int fdbk_offset_vdc_volts;

/********************************************************************/
/*		ac feedback scaling (340-349) [040-049]						*/
/********************************************************************/
int fdbk_ratio_iinv;
int fdbk_burden_iinv_x10;
int fdbk_ratio_iline;
int fdbk_burden_iline_x10;
int fdbk_ratio_ineutral;
int fdbk_burden_ineutral_x10;
int fdbk_ratio_vinv_x10;
int fdbk_ratio_vline_x10;
int gnd_impedance_trip;
int gnd_impedance_delay_ms;

/********************************************************************/
/*		dc protection settings (350-369) [050-069]					*/
/********************************************************************/
int dcin_overvolt_trip_volts;
int dcin_overvolt_delay_ms;
int dcin_undervolt_trip_volts;
int dcin_undervolt_delay_ms;
int dcin_overcurrent_trip_pct;
int dc_overvolt_inst_trip_volts;
int dc_overvolt_trip_volts;
int dc_overvolt_delay_ms;
int dc_undervolt_inst_trip_volts;
int dc_undervolt_trip_volts;
int dc_undervolt_delay_ms;
int dcin_overcurrent_inst_trip_pct;
int dcin_overcurrent_delay_ms;
int gnd_fault_trip_amps_x10;
int gnd_fault_delay_ms;
int current_difference_trip_pct;
int current_difference_delay_ms;
int inv_overcurrent_trip_pct;
int inv_overcurrent_delay_ms;
int hw_overcurrent_trip_pct;

/********************************************************************/
/*		ac protection settings (370-389) [070-089]					*/
/********************************************************************/
int line_overvolt_trip_slow_pct;
int line_overvolt_delay_slow_ms;
int line_overvolt_trip_fast_pct;
int line_overvolt_delay_fast_ms;
int line_undervolt_trip_slow_pct;
int line_undervolt_delay_slow_ms;
int line_undervolt_trip_fast_pct;
int line_undervolt_delay_fast_ms;
int line_overcurrent_trip_pct;
int line_overcurrent_delay_ms;
int neutral_overcurrent_trip_amps_x10;
int neutral_overcurrent_delay_ms;
int over_frequency_trip_Hz_x10;
int over_frequency_delay_ms;
int under_frequency_trip_Hz_x10;
int under_frequency_delay_10ms;
int voltage_unbalance_trip_pct_x10;
int voltage_unbalance_delay_10ms;
int current_unbalance_trip_pct_x10;
int current_unbalance_delay_10ms;

/********************************************************************/
/*		features (390-399) [090-099]								*/
/********************************************************************/
int estop_reset;
int auto_reset_interval_6sec;
int auto_reset_max_attempts;
int auto_reset_lockout_6sec;
int t_disp_fltr_100ms;
int pll_bw;
int reconnect_delay_6sec;
int contactor_delay_ms;
int gating_phase_shift;
int inverter_id;

/********************************************************************/
/*		PI regulators (400-409)	[100-109]							*/
/********************************************************************/
int kp_dc_volt_reg;
int ki_dc_volt_reg;
int kp_inv_cur_reg;
int ki_inv_cur_reg;
int kp_line_cur_reg;
int ki_line_cur_reg;
int kp_temp_reg;
int ki_temp_reg;
int kp_overtemp_reg;
int ki_overtemp_reg;

/********************************************************************/
/*		PV settings (410-419) [110-119]								*/
/********************************************************************/
int vdcin_threshold;
int kp_ac_volt_reg;
int vdcin_delay_6sec;
int precharge_volts_min;
int ki_ac_volt_reg;
int power_source;
int k_anti_island;
int mppt_interval_6sec;
int inhibit_flt_inductor;
int kwh_index;

/********************************************************************/
/*		MPPT (420-429) [120-129]									*/
/********************************************************************/
int dc_voltage_cmd_volts;
int min_power_change_pct_x10;
int power_change_delay_100ms;
int vdc_step_time_100ms;
int vdc_step_min_volts_x10;
int vdc_step_max_volts_x10;
int low_power_trip_pct;
int low_power_delay_6sec;
int preset_kwh_total;
int preset_mwh_total;

/********************************************************************/
/*		ac control (430-439) [130-139]								*/
/********************************************************************/
int power_control_mode;
int real_power_cmd_pct;
int real_current_cmd_pct;
int pc_cmd_param;		// not saved
int reactive_power_cmd_pct;
int reactive_current_cmd_pct;
int power_factor_cmd_pct;
int run_enable;
int ramp_time_ms;
int remote_cmd_param;	// not saved

/********************************************************************/
/*		test mode (440-449) [140-149]								*/
/********************************************************************/
int test_mode;			// not saved
int output_test;		// not saved
int ext_input_mask;		// not saved
int fan_spd_cmd_in;		// not saved
int gate_test_select;	// not saved
int freq_simulate;
int v_cmd_simulate_pct;
int v_cmd_open_cct_pct;
int i_cmd_short_cct_pct;
int v_cmd_gate_test_pct;

/********************************************************************/
/*		data collection parameters (450-459) [150-159]				*/
/********************************************************************/
int trigger_channel;
int trigger_delay;
int trigger_condition;
int trigger_level;
int trigger_mode;
int trigger_state;
int sample_interval;
int analog_out[3];

/********************************************************************/
/*		data channel parameters (460-469) [160-169]					*/
/********************************************************************/
int channel[DATA_CHANNELS];
int playback_index;	// not saved
int testpoint_select;

/********************************************************************/
/*		communications settings	(470-479) [170-179]					*/
/********************************************************************/
int modbus_baud_rate;
int modbus_parity;
int modbus_data_bits;
int modbus_stop_bits;
int modbus_slave_id;
int modbus_access_code;
int access_code_1;
int access_code_2;
int access_code_3;
int fault_queue_index;

/********************************************************************/
/*		real time clock (480-489) [180-189]							*/
/********************************************************************/
int save_command;		// not saved
int initialize_cmd;
int calibrate_clock;	// not saved
int	set_clock;			// not saved
int display_time[4];	// not saved
int filler188;
int language;

/********************************************************************/
/*		thermal protection (490-499) [190-199]						*/
/********************************************************************/
int low_temp_trip[3];
int high_temp_trip[3];
int fan_on_temp;
int fan_off_temp;
int fan_speed_min_pct;
int adjustable_spd_fan;

/********************************************************************/
/*		external input scaling (500-509) [200-209]					*/
/********************************************************************/
int ext_input_scale_1;
int ext_input_scale_2;
int ext_input_scale_3;
int ext_input_scale_4;
int ext_input_offset_1;
int ext_input_offset_2;
int ext_input_offset_3;
int ext_input_offset_4;
int a2d_error_mv;
int param_209;

/********************************************************************/
/*		more protection (510-519) [210-219]							*/
/********************************************************************/
int inv_undervolt_trip_pct;
int inv_undervolt_delay_ms;
int sci_slave_id;
int k_voltffd;
int filler214;
int filler215;
int line_overvolt_trip_inst_pct;
int line_overvolt_delay_inst_ms;
int line_undervolt_trip_inst_pct;
int line_undervolt_delay_inst;

/********************************************************************/
/*		ground fault protection (520-529) [220-229]					*/
/********************************************************************/
int filler220;
int filler221;
int filler222;
int filler223;
int filler224;
int filler225;
int dc_bus_imbalance_trip;
int dc_bus_imbalance_delay_ms;
int fuse_volt_fault_trip;
int fuse_volt_fault_delay_ms;

/********************************************************************/
/*		unused (530-539) [230-239]									*/
/********************************************************************/
int adv_control_configuration;
int fan_normal_speed_limit;
int fan_extended_speed_limit;
int fan_extended_temp_threshold;
int testVar[6];


/********************************************************************/
/*		reactive power (540 - 550) [240-249]						*/
/********************************************************************/
int filler240;
int real_power_rate;
int reactive_power_control_mode;
int reactive_power_cmd;
int reactive_power_rate;
int neg_limit_power_factor_cmd_pct;
int pos_limit_power_factor_cmd_pct;
int maximum_dq_current;
int filler248;
int filler249;

/*	(550-554)  [250-255]							*/
int param_250;
int param_251;
int param_252;
int param_253;
int param_254;

int chksum;
};

/********************************************************************/
/*	Saved Variables (128 words)										*/
/********************************************************************/
struct PERDAY {int date;int kwh;};

struct SAVED_VARS {
unsigned chksum;
long kws_total;            
int kwh_total;            
int mwh_total;
int today;
int kwh_today;
unsigned saved_debug[4];
unsigned scia_error_count,scib_error_count;
int filler[3];         
int kwh_index;
struct PERDAY kwh_per_day[32];
unsigned string_kw_hrs[32];
unsigned old_faults[16];
};

/********************************************************************/
/*	NV RAM (128Kx8)													*/
/* DO NOT ATTEMPT TO ACCESS VARIABLES WITHIN THESE STRUCTURES		*/
/********************************************************************/
struct NV_RAM	{
				struct PARAMS parameters;
				struct PARAMS filler1;
				struct PARAMS parameters_2;
				struct PARAMS filler2;
				struct SAVED_VARS saved_variables;
				struct SAVED_VARS filler3;
				struct FAULT_QUEUE fault_queue;
				struct FAULT_QUEUE filler4;
				struct DATA_SETUP data_setup;
				struct DATA_SETUP filler5;
				struct DATA_RECORD data_record;
				struct DATA_RECORD filler6;
				struct PWR_CURVE power_curve;
				struct PWR_CURVE filler7;
				};


extern struct P_FRAME0 PF0;
extern struct P_FRAME2 PF2;
