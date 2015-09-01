/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
2008-03-00	add parameters a2d_error_mv,v_fdbk_config
2008-04-28	add parameter gating_phase_shift
			change default values for vdcin_threshold, vdcin_delay_6sec,
			low_power_trip_pct, low_power_delay_6sec
2008-05-12	add parameter gnd_impedance_trip
2008-05-13	change to Modbus access level 0 on serial_number & number_strings
2008-05-27	change default value of line overcurrent trip from 200 to 120%
2008-06-24	increase maximum value of gate_test_param from 12 to 16
			reduce maximum value of 6 sec delay parameters from 1000 to 500
2008-06-27	change fdbk_offset_vdcin_volts to fdbk_offset_idc_amps
2008-07-03	increase maximum value of high_temp_trip[2] from 90 to 105
2008-07-04	change default value of number_strings from 0 to 1
2008-07-22	increase maximum value of high_temp_trip[2] from 105 to 120
2008-08-15	change v_fdbk_delta to v_fdbk_config and increase max value to 2
2008-08-20	change default values of dc voltage parameters from volts to percent
			increase default value of dc voltage regulator gains to 1.0 & 0.1
2008-08-21	increase maximum value of power_control mode from 3 to 4
2008-09-02	increase default value of ki_dc_volt_reg from 0.1 to 0.2
2008-09-03	change default values of dc voltage parameters back to volts
2008-09-08	change region parameter to inverter_type and increase maximum to 2
2008-09-17	replace heat_xchg_temp_ref with fan_speed_min
2008-10-03	move mppt_interval_6sec and replace with mppt_factor
2008-10-10	change default value of vdcin_delay_6sec from 10 to 30 minutes
2008-10-17	change default value of min_power_change_pct_x10 from 0 to 1
			change default value of kp_dc_volt_reg from 1.0 to 2.0
			change default value of ki_dc_volt_reg from 0.2 to 0.5
2008-10-28	replace mppt_factor with vdc_step_time_100ms
1 MW version
2008-12-12	add 4 new Modbus ID parameters
2009-01-07	add parameter change fault bit (0x10) to attribute word
2009-01-21	add reg_limit_type parameter
2009-01-23	add line instantaneous overvoltage parameters
2009-02-18	add 9 new ground fault parameters
2009-03-03	delete parameter gate_driver_fdbk_polarity
2009-03-06	add parameters vdcin_threshold_2 and vdcin_delay_2_6sec
2009-03-20	add parameter k_i_leakage_q8
2009-03-23	delete parameter regulator_flags
2009-03-25	delete parameter spi_master
			change parameter comm_slave_id to spi_slave_id
			change parameter comm_master_id to comm_board_id
2009-04-23	change comm_board_id access level from 3 to 0
2009-05-06	add Solstice power control parameters
2009-05-07	add line instantaneous undervoltage parameters
2009-05-07	add parameter gnd_config
2009-05-26	update default settings for 1 MW
2009-06-05	add variable saved_debug
Solstice version
2009-07-22	add separate access levels for read and write
			change read access levels of many parameters
2009-07-23	change parameter change fault bit in attribute word to 0x08
2009-08-04	change default value of voltage feedforward gain from 0.6 to 1.0
			increase maximum value of regulator gains from 5.000 to 7.999
2009-10-15	delete variable saved_debug
2009-10-26	add parameter precharge_config

*/
/********************************************************************/
/*	Read/write parameters (300-529)									*/
/*	Modbus register number is 1 higher than parameter number		*/
/********************************************************************/

// serial number (300-309)
		&junk,											RD0|WR3,95,0,32767,
		&parameters.serial_number[0],					RD0|WR3,95,0,32767,
		&parameters.serial_number[1],					RD0|WR3,0,0,32767,
		&parameters.serial_number[2],					RD0|WR3,1,0,32767,
		&parameters.serial_number[3],					RD0|WR3,1,0,32767,
		&parameters.serial_number[4],					RD0|WR3,0,0,32767,
		&parameters.serial_number[5],					RD0|WR3,0,0,32767,
		&parameters.serial_number[6],					RD0|WR3,0,0,32767,
		&parameters.serial_number[7],					RD0|WR3,0,0,32767,
		&parameters.serial_number[8],					RD0|WR3,0,0,32767,

// ratings (310-319)
		&parameters.kw_rated,							RD0|WR3|NOWR_RUN,100,0,500,
		&parameters.freq_rated,							RD0|WR3|NOWR_RUN,60,0,60,
		&parameters.v_line_rated,						RD0|WR3|NOWR_RUN,320,0,1000,
		&parameters.v_inv_rated,						RD3|WR3|NOWR_RUN,320,0,1000,
		&parameters.v_inv_tap_pct,						RD0|WR3|NOWR_RUN,0,-20,10,
		&parameters.v_fdbk_config,						RD3|WR3|NOWR_RUN,0,0,2,
		&parameters.inverter_type,						RD3|WR3|NOWR_RUN,0,0,2,
		&parameters.gnd_config,							RD0|WR3|NOWR_RUN,0,0,2,
		//&parameters.precharge_config,					RD3|WR3|NOWR_RUN,0,0,1,
		DUMMY_VARIABLE
		DUMMY_VARIABLE
// components (320-329)
		&parameters.filter_inductor_uh,					RD3|WR3|NOWR_RUN,0,0,32767,
		&parameters.filter_cap_uf,						RD3|WR3|NOWR_RUN,0,0,32767,
		DUMMY_VARIABLE
		&parameters.number_temp_fbk,					RD0|WR3|NOWR_RUN,0,0,8,
		&parameters.inv_bridges,						RD3|WR3|NOWR_RUN,0,0,2,
		&parameters.pwm_Hz,								RD3|WR3|NOWR_RUN,5000,4000,8000,
		&parameters.dc_cap_uf,							RD3|WR3|NOWR_RUN,0,0,32767,
		&parameters.number_strings,						RD0|WR3|NOWR_RUN,1,0,30,
		&parameters.dead_time,							RD3|WR3|NOWR_RUN,4,0,5,
		&parameters.IGBT_current_max,					RD3|WR3|NOWR_RUN,0,0,1,

// dc feedback scaling (330-339)	default value zero to force table lookup
		&parameters.fdbk_ratio_istring,					RD3|WR3|NOWR_RUN,2000,0,10000,
		&parameters.fdbk_burden_istring_x10,			RD3|WR3|NOWR_RUN,330,0,10000,
		&parameters.fdbk_ratio_idc,						RD3|WR3|NOWR_RUN,0,0,10000,
		&parameters.fdbk_burden_idc_x10,				RD3|WR3|NOWR_RUN,0,0,10000,
		&parameters.fdbk_ratio_ignd,					RD3|WR3|NOWR_RUN,667,0,10000,
		&parameters.fdbk_burden_ignd_x10,				RD3|WR3|NOWR_RUN,600,0,10000,
		&parameters.fdbk_ratio_vdcin_x10,				RD3|WR3|NOWR_RUN,0,0,10000,
		&parameters.fdbk_ratio_vdc_x10,					RD3|WR3|NOWR_RUN,0,0,10000,
		&parameters.fdbk_offset_idc_amps,				RD3|WR3|NOWR_RUN,0,-1000,1000,
		&parameters.fdbk_offset_vdc_volts,				RD3|WR3|NOWR_RUN,0,-1000,1000,

// ac feedback scaling (340-349)	default value zero to force table lookup
		&parameters.fdbk_ratio_iinv,					RD3|WR3|NOWR_RUN,0,0,10000,
		&parameters.fdbk_burden_iinv_x10,				RD3|WR3|NOWR_RUN,0,0,10000,
		&parameters.fdbk_ratio_iline,					RD3|WR3|NOWR_RUN,0,0,10000,
		&parameters.fdbk_burden_iline_x10,				RD3|WR3|NOWR_RUN,0,0,10000,
		&parameters.fdbk_ratio_ineutral,				RD3|WR3|NOWR_RUN,33,0,10000,
		&parameters.fdbk_burden_ineutral_x10,			RD3|WR3|NOWR_RUN,100,0,10000,
		&parameters.fdbk_ratio_vinv_x10,				RD3|WR3|NOWR_RUN,2500,0,10000,
		&parameters.fdbk_ratio_vline_x10,				RD3|WR3|NOWR_RUN,2500,0,10000,
		&parameters.gnd_impedance_trip,					RD0|WR3,100,0,10000,
		&parameters.gnd_impedance_delay_ms,				RD0|WR3,100,0,10000,

// dc protection (350-369)
		&parameters.dcin_overvolt_trip_volts,			RD0|WR3,660,0,1000,	
		&parameters.dcin_overvolt_delay_ms,				RD0|WR3,100,0,1000,	
		&parameters.dcin_undervolt_trip_volts,			RD0|WR3,400,0,1000,	
		&parameters.dcin_undervolt_delay_ms,			RD0|WR3,100,0,1000,	
		&parameters.dc_overvolt_inst_trip_volts,		RD0|WR3,700,0,1000,
		&parameters.dc_overvolt_trip_volts,				RD0|WR3,660,0,1000,	
		&parameters.dc_overvolt_delay_ms,				RD0|WR3,100,0,1000,	
		&parameters.dc_undervolt_inst_trip_volts,		RD0|WR3,320,0,1000,
		&parameters.dc_undervolt_trip_volts,			RD0|WR3,400,0,1000,
		&parameters.dc_undervolt_delay_ms,				RD0|WR3,1000,0,1000,

		&parameters.dcin_overcurrent_inst_trip_pct,		RD0|WR3,140,0,300,
		&parameters.dcin_overcurrent_trip_pct,			RD0|WR3,120,0,300,
		&parameters.dcin_overcurrent_delay_ms,			RD0|WR3,100,0,1000,
		&parameters.gnd_fault_trip_amps_x10,			RD0|WR3,0,0,10000,
		&parameters.gnd_fault_delay_ms,					RD0|WR3,10,0,10000,
		&parameters.current_difference_trip_pct,		RD3|WR3,20,0,100,
		&parameters.current_difference_delay_ms,		RD3|WR3,1000,0,10000,
		&parameters.inv_overcurrent_trip_pct,			RD3|WR3,140,0,300,
		&parameters.inv_overcurrent_delay_ms,			RD3|WR3,100,0,1000,
		&parameters.hw_overcurrent_trip_pct,			RD3|WR3,180,0,500,

// ac protection (370-389)
		&parameters.line_overvolt_trip_fast_pct,		RD0|WR3,120,0,200,
		&parameters.line_overvolt_delay_fast_ms,		RD0|WR3,160,0,1000,
		&parameters.line_overvolt_trip_slow_pct,		RD0|WR3,110,0,200,
		&parameters.line_overvolt_delay_slow_ms,		RD0|WR3,1000,0,2000,
		&parameters.line_undervolt_trip_fast_pct,		RD0|WR3,50,0,200,
		&parameters.line_undervolt_delay_fast_ms,		RD0|WR3,160,0,1000,
		&parameters.line_undervolt_trip_slow_pct,		RD0|WR3,88,0,200,
		&parameters.line_undervolt_delay_slow_ms,		RD0|WR3,1000,0,2000,
		&parameters.line_overcurrent_trip_pct,			RD0|WR3,120,0,200,
		&parameters.line_overcurrent_delay_ms,			RD0|WR3,100,0,1000,

		&parameters.neutral_overcurrent_trip_amps_x10,	RD0|WR3,30,0,1000,
		&parameters.neutral_overcurrent_delay_ms,		RD0|WR3,1000,0,10000,
		&parameters.over_frequency_trip_Hz_x10,			RD0|WR3,50,0,500,
		&parameters.over_frequency_delay_ms,			RD0|WR3,160,0,30000,
		&parameters.under_frequency_trip_Hz_x10,		RD0|WR3,70,0,500,
		&parameters.under_frequency_delay_10ms,			RD0|WR3,16,0,30000,
		&parameters.voltage_unbalance_trip_pct_x10,		RD0|WR3,100,0,1000,
		&parameters.voltage_unbalance_delay_10ms,		RD0|WR3,1000,0,30000,
		&parameters.current_unbalance_trip_pct_x10,		RD0|WR3,100,0,1000,
		&parameters.current_unbalance_delay_10ms,		RD0|WR3,1000,0,30000,

// features (390-399)
		&parameters.estop_reset,						RD3|WR3,1,0,1,
		&parameters.auto_reset_interval_6sec,			RD3|WR3,10,0,500,
		&parameters.auto_reset_max_attempts,			RD3|WR3,5,0,5,
		&parameters.auto_reset_lockout_6sec,			RD3|WR3,500,0,500,
		&parameters.t_disp_fltr_100ms,					RD3|WR3,5,0,100,
		&parameters.pll_bw,								RD3|WR3,20,0,100,
		&parameters.reconnect_delay_6sec,				RD3|WR3,50,0,50,
		&parameters.contactor_delay_ms,					RD3|WR3,30,0,1000,
		&parameters.gating_phase_shift,					RD3|WR3,0,-450,450,
		&parameters.inverter_id,						RD3|WR3,1,1,2,

// regulators (400-409)
		&parameters.kp_dc_volt_reg,						RD3|WR3,5000,0,7999,
		&parameters.ki_dc_volt_reg,						RD3|WR3,125,0,7999,
		&parameters.kp_inv_cur_reg,						RD3|WR3,400,0,7999,
		&parameters.ki_inv_cur_reg,						RD3|WR3,5,0,7999,
		&parameters.kp_line_cur_reg,					RD3|WR3,100,0,7999,
		&parameters.ki_line_cur_reg,					RD3|WR3,10,0,7999,
		&parameters.kp_temp_reg,						RD3|WR3,5000,0,5000,
		&parameters.ki_temp_reg,						RD3|WR3,10,0,5000,
		&parameters.kp_overtemp_reg,					RD3|WR3,500,0,500,
		&parameters.ki_overtemp_reg,					RD3|WR3,5,0,500,

// dc input (410-419)
		&parameters.vdcin_threshold,					RD3|WR3,550,0,1000,
		DUMMY_VARIABLE
		&parameters.vdcin_delay_6sec,					RD3|WR3,300,0,500,
		&parameters.precharge_volts_min,				RD3|WR3,320,0,1000,
		DUMMY_VARIABLE
		&parameters.power_source,						RD3|WR3,0,0,1,
		&parameters.k_anti_island,						RD3|WR3,4,0,20,
		&parameters.mppt_interval_6sec,					RD3|WR3,50,10,90,
		// PFP 
		&parameters.inhibit_flt_inductor,				RD3|WR3,0,0,3600,
		&kwh_index,										RD3|WR3,0,0,31,

// mppt (420-429)
		&parameters.dc_voltage_cmd_volts,				RD3|WR3,570,0,1000,
		&parameters.min_power_change_pct_x10,			RD3|WR3,0,0,1000,
		&parameters.power_change_delay_100ms,			RD3|WR3,0,0,100,
		&parameters.vdc_step_time_100ms,				RD3|WR3,5,0,100,
		&parameters.vdc_step_min_volts_x10,				RD3|WR3,1,0,100,
		&parameters.vdc_step_max_volts_x10,				RD3|WR3,20,10,100,
		&parameters.low_power_trip_pct,					RD3|WR3,0,0,100,
		&parameters.low_power_delay_6sec,				RD3|WR3,100,0,500,
		&saved_variables.kwh_total,						RD3|WR3,0,0,999,
		&saved_variables.mwh_total,						RD3|WR3,0,0,9999,

// ac control (430-439)
		&parameters.power_control_mode,					RD3|WR3,3,0,4,
		&parameters.real_power_cmd_pct,					RD3|WR3,100,0,110,
		&parameters.real_current_cmd_pct,				RD3|WR3,0,0,110,
		&pc_cmd,										RD3|WR3,0,0,9,
		&parameters.reactive_power_cmd_pct,				RD3|WR3,0,-60,60,
		&parameters.reactive_current_cmd_pct,			RD3|WR3,0,-60,60,
		&remote_cmd_param,								RD2|WR2,0,0,255,
		&parameters.run_enable,							RD1|WR1,1,0,1,
		&parameters.power_factor_cmd_pct,				RD2|WR2,1000,-1000,1000,
		&parameters.ramp_time_ms,						RD3|WR3,10000,0,20000,

// test mode (440-449)
		&testmode_param,								RD3|WR3,0,0,9,
		&output_test,									RD3|WR3,0,0,20,
		&ext_input_mask,								RD3|WR3,0,0,32767,
		&fan_spd_cmd_in,								RD3|WR3,0,0,1000,
		&gate_test_param,								RD3|WR3,0,0,16,
		&parameters.freq_simulate,						RD3|WR3,600,-700,700,
		&parameters.v_cmd_gate_test_pct,				RD3|WR3,0,-100,100,
		&parameters.v_cmd_open_cct_pct,					RD3|WR3,50,0,100,
		&parameters.i_cmd_short_cct_pct,				RD3|WR3,10,0,100,
		&parameters.v_cmd_simulate_pct,					RD3|WR3,100,0,150,

// trigger settings (450-459)
		&parameters.trigger_channel,					RD3|WR3,1,1,8,
		&parameters.trigger_delay,						RD3|WR3,50,0,100,
		&parameters.trigger_condition,					RD3|WR3,0,0,5,
		&parameters.trigger_level,						RD3|WR3,0,0,32767,
		&trig_mode,										RD3|WR3,0,0,2,
		&trig_state,									RD3|WR3,0,0,3,
		&parameters.sample_interval,					RD3|WR3,10,2,32767,
		&parameters.analog_out[0],						RD3|WR3,227,0,299,
		&parameters.analog_out[1],						RD3|WR3,227,0,299,
		&parameters.analog_out[2],						RD3|WR3,227,0,299,

// data channels (460-469)
		&parameters.channel[0],							RD3|WR3,1,0,299,
		&parameters.channel[1],							RD3|WR3,2,0,299,
		&parameters.channel[2],							RD3|WR3,3,0,299,
		&parameters.channel[3],							RD3|WR3,4,0,299,
		&parameters.channel[4],							RD3|WR3,5,0,299,
		&parameters.channel[5],							RD3|WR3,6,0,299,
		&parameters.channel[6],							RD3|WR3,7,0,299,
		&parameters.channel[7],							RD3|WR3,8,0,299,
		&playback_index,								RD3|WR3,0,0,1000,
		&parameters.testpoint_select,					RD3|WR3,0,0,19,

// modbus (470-479)
		&parameters.modbus_baud_rate,					RD0|WR0,96,48,1152,
		&parameters.modbus_parity,						RD0|WR0,0,0,2,
		&parameters.modbus_data_bits,					RD0|WR0,8,7,8,
		&parameters.modbus_stop_bits,					RD0|WR0,1,1,2,
		&parameters.modbus_slave_id,					RD0|WR0,1,0,250,
		&parameters.modbus_access_code,					RD0|WR0,0,0,9999,
		&parameters.access_code_1,						RD1|WR1,9367,0,9999,
		&parameters.access_code_2,						RD2|WR2,5204,0,9999,
		&parameters.access_code_3,						RD3|WR3,1581,0,9999,
		&fault_queue_index,								RD1|WR1,0,0,FAULT_QUEUE_SIZE,

// real time clock (480-489)
		&save_command,									RD1|WR1,0,0,4,
		&parameters.initialize_cmd,						RD3|WR3,0,0,9,
		&parameters.calibrate_clock,					RD3|WR3,0,-59,59,
		&set_clock,										RD3|WR3,0,0,1,
		(int*)&display_time.second_hundredth,			RD0|WR3,0,0,5999,
		(int*)&display_time.hour_minute,				RD0|WR3,0,0,2359,
		(int*)&display_time.month_day,					RD0|WR3,0101,0101,1231,
		(int*)&display_time.year,						RD0|WR3,2000,2000,2099,
		(int*)&saved_variables.scib_error_count,		RD3|WR3,0,0,32767,
		&parameters.language,							RD0|WR0,0,0,1,

// thermal protection (490-499)
		&parameters.low_temp_trip[0],					RD3|WR3,-25,-40,90,
		&parameters.low_temp_trip[1],					RD3|WR3,-25,-40,90,
		&parameters.low_temp_trip[2],					RD3|WR3,-25,-40,90,
		&parameters.high_temp_trip[0],					RD3|WR3,75,0,80,
		&parameters.high_temp_trip[1],					RD3|WR3,70,0,71,
		&parameters.high_temp_trip[2],					RD3|WR3,85,0,120,
		&parameters.fan_speed_min_pct,					RD3|WR3,0,0,100,
		&parameters.fan_on_temp,						RD3|WR3,60,0,70,
		&parameters.fan_off_temp,						RD3|WR3,45,0,70,
		&parameters.adjustable_spd_fan,					RD3|WR3,1,0,1,

// external input scaling (500-509)
		&parameters.ext_input_scale_1,					RD3|WR3,1000,0,10000,
		&parameters.ext_input_scale_2,					RD3|WR3,1000,0,10000,
		&parameters.ext_input_scale_3,					RD3|WR3,1000,0,10000,
		&parameters.ext_input_scale_4,					RD3|WR3,1000,0,10000,
		&parameters.ext_input_offset_1,					RD3|WR3,0,-1000,1000,
		&parameters.ext_input_offset_2,					RD3|WR3,0,-1000,1000,
		&parameters.ext_input_offset_3,					RD3|WR3,0,-1000,1000,
		&parameters.ext_input_offset_4,					RD3|WR3,0,-1000,1000,
		&debug_control,									RD3|WR3,40,0,10000,
		&parameters.a2d_error_mv,						RD3|WR3,250,0,1000,

// more protection (510-519)		
		&parameters.inv_undervolt_trip_pct,				RD3|WR3,80,0,200,
		&parameters.inv_undervolt_delay_ms,				RD3|WR3,1000,0,30000,
		&parameters.sci_slave_id,						RD3|WR3,1,0,250,
		&parameters.k_voltffd,							RD3|WR3,100,0,100,
		DUMMY_VARIABLE
		DUMMY_VARIABLE
		&parameters.line_overvolt_trip_inst_pct,		RD3|WR3,115,0,200,
		&parameters.line_overvolt_delay_inst_ms,		RD3|WR3,16,0,1000,
		&parameters.line_undervolt_trip_inst_pct,		RD3|WR3,30,0,200,
		&parameters.line_undervolt_delay_inst,			RD3|WR3,2,0,100,

// ground fault protection (520-529)
		DUMMY_VARIABLE
		DUMMY_VARIABLE
		DUMMY_VARIABLE
		DUMMY_VARIABLE
		DUMMY_VARIABLE
		DUMMY_VARIABLE
		&parameters.dc_bus_imbalance_trip,				RD3|WR3,800,0,1000,
		&parameters.dc_bus_imbalance_delay_ms,			RD3|WR3,100,0,30000,
		&parameters.fuse_volt_fault_trip,				RD3|WR3,50,0,1000,
		&parameters.fuse_volt_fault_delay_ms,			RD3|WR3,100,0,9999,

// odds & ends (530-539)
		&parameters.adv_control_configuration,			RD3|WR3,0,0,32767,
		&parameters.fan_normal_speed_limit,				RD3|WR3,75,0,100,
		&parameters.fan_extended_speed_limit,			RD3|WR3,100,0,100,
		&parameters.fan_extended_temp_threshold,		RD3|WR3,70,0,100,
		&parameters.testVar[0],							RD3|WR3,0,-10000,10000,
		&parameters.testVar[1],							RD3|WR3,0,-10000,10000,
		&parameters.testVar[2],							RD3|WR3,0,-10000,10000,
		&parameters.testVar[3],							RD3|WR3,0,-10000,10000,
		&parameters.testVar[4],							RD3|WR3,0,-10000,10000,
		&parameters.testVar[5],							RD3|WR3,0,-10000,10000,

// advanced power management (SEGIS) (540-549)
		DUMMY_VARIABLE
		&parameters.real_power_rate,					RD1|WR1,100,1,1000,
		&parameters.reactive_power_control_mode,		RD2|WR2,0,0,1,
		&parameters.reactive_power_cmd,					RD2|WR2,0,-1000,1000,
		&parameters.reactive_power_rate,				RD2|WR2,100,1,1000,
		&parameters.neg_limit_power_factor_cmd_pct,		RD2|WR2,-800,-1000,0,
		&parameters.pos_limit_power_factor_cmd_pct,		RD2|WR2,800,0,1000,
		&parameters.maximum_dq_current,					RD2|WR2,1100,0,1250,
		DUMMY_VARIABLE
		DUMMY_VARIABLE	
};

