/********************************************************************/
/********************************************************************/
/*		Parameter Slow Update function								*/
/*		updates one or two parameters (more or less) each call		*/
/********************************************************************/
extern int vac_cmd_pu, kp_vac_q12, ki_vac_q12;

void slow_param_update(void)
{
	register int temp1,temp2;

	if (update_case > 61)
		update_case=0;
	switch (update_case++)
	{

	case(0):
		real_power_cmd_pu=parameters.real_power_cmd_pct*PERCENT_TO_PU;
		reactive_power_cmd_pu=parameters.reactive_power_cmd_pct*PERCENT_TO_PU;
	break;

/********************************************************************/
/*	dc voltage faults												*/
/********************************************************************/
	case(1):
	VDC_TRIP_TO_PU(dcin_overvolt_trip_volts,VDCIN,dcin_ov,0)
	vdc_cmd_max=flts.dcin_ov.trip-(PER_UNIT/20);
	flts.dcin_ov.delay=parameters.dcin_overvolt_delay_ms;
	break;

	case(2):
	VDC_TRIP_TO_PU(dcin_undervolt_trip_volts,VDCIN,dcin_uv,(PER_UNIT/100))
	flts.dcin_uv.delay=parameters.dcin_undervolt_delay_ms;
	break;

	case(3):
	VDC_TRIP_TO_PU(dc_overvolt_trip_volts,VDC,dc_ov,0)
	flts.dc_ov.delay=parameters.dc_undervolt_delay_ms;
	break;

	case(4):
	VDC_TRIP_TO_PU(dc_undervolt_trip_volts,VDC,dc_uv,0)
	flts.dc_uv.delay=parameters.dc_undervolt_delay_ms;
	break;

	case(5):
	VDC_TRIP_TO_PU(dc_overvolt_inst_trip_volts,VDC,dc_ov_inst,0)
	flts.dc_ov_inst.delay=1;
	break;

	case(6):
	VDC_TRIP_TO_PU(dc_undervolt_inst_trip_volts,VDC,dc_uv_inst,(PER_UNIT/100))
	flts.dc_uv_inst.delay=1;
	break;
	
/********************************************************************/
/*	external analog input scaling									*/
/********************************************************************/
	case(7):
	k_fdbk_q15[EX1]=parameters.ext_input_scale_1;
	k_fdbk_q15[EX2]=parameters.ext_input_scale_2;
	break;

	case(8):
	k_fdbk_q15[EX3]=parameters.ext_input_scale_3;
	k_fdbk_q15[EX4]=parameters.ext_input_scale_4;
	break;

	case(9):
	temp1=div_q12(PER_UNIT,rated_dc_amps);
  	fdbk_offset_idc_pu=	MULQ_RND(12,parameters.fdbk_offset_idc_amps,temp1);
   	fdbk_offset_vdc_pu=	MULQ_RND(12,parameters.fdbk_offset_vdc_volts,dc_volts_to_pu_q12);
	break;

/********************************************************************/
/*	dc current faults												*/
/********************************************************************/
	case(10):
	PCT_TRIP_TO_PU(dcin_overcurrent_trip_pct,IDC,dcin_oc,0)
	flts.dcin_oc.delay=parameters.dcin_overcurrent_delay_ms;
	break;

	case(11):
	PCT_TRIP_TO_PU(dcin_overcurrent_inst_trip_pct,IDC,dcin_oc_inst,0)
	flts.dcin_oc_inst.delay=1;
	break;

	case(12):
	if (status_flags & STATUS_GATE_TEST)
		hw_oc_trip=DAC_FULL_SCALE<<3;
	else
	{
		temp1=MULQ(15,k_fdbk_q15[IIA],(65536L/PERCENT_TO_PU));	// 10V max
		if (parameters.hw_overcurrent_trip_pct > temp1)
			parameters.hw_overcurrent_trip_pct=temp1;
		hw_oc_trip=div_q12(parameters.hw_overcurrent_trip_pct*PERCENT_TO_PU,k_fdbk_q15[IIA]);
	}
	break;

	case(13):
	if (test_mode!=GATING_IO_TEST)
	{
		temp1=gate_test_param;		// 1-6 & 11-16
		if (temp1 > 6)
		{
			if (temp1 > 10)
				temp1-=4;
			else
				temp1=0;
		}
		gate_test_select=temp1;		// 1-6 & 7-12
	}
	break;

	case(14):

		flts.gnd_flt.trip=parameters.gnd_fault_trip_amps_x10;
		flts.gnd_flt.delay=parameters.gnd_fault_delay_ms;

	break;

/********************************************************************/
/*	ac voltage faults												*/
/********************************************************************/
	case(15):
	PCT_TRIP_TO_PU(line_overvolt_trip_slow_pct,VLA,line_ov_slow,0)
	CALC_AC_FAULT_DELAY(line_ov_slow,parameters.line_overvolt_delay_slow_ms)
	break;

	case(16):
	PCT_TRIP_TO_PU(line_undervolt_trip_slow_pct,VLA,line_uv_slow,0)
	CALC_AC_FAULT_DELAY(line_uv_slow,parameters.line_undervolt_delay_slow_ms)
	break;

	case(17):
	PCT_TRIP_TO_PU(line_overvolt_trip_fast_pct,VLA,line_ov_fast,0)
	CALC_AC_FAULT_DELAY(line_ov_fast,parameters.line_overvolt_delay_fast_ms)
	break;

	case(18):
	PCT_TRIP_TO_PU(line_undervolt_trip_fast_pct,VLA,line_uv_fast,0)
	CALC_AC_FAULT_DELAY(line_uv_fast,parameters.line_undervolt_delay_fast_ms)
	break;

	case(19):
	flts.v_unbalance.trip=parameters.voltage_unbalance_trip_pct_x10;
	flts.v_unbalance.delay=parameters.voltage_unbalance_delay_10ms;
	break;

/********************************************************************/
/*	ac current faults												*/
/********************************************************************/
	case(20):
	PCT_TRIP_TO_PU(line_overcurrent_trip_pct,ILA,line_oc,0)
	flts.line_oc.delay=parameters.line_overcurrent_delay_ms;
	break;

	case(21):
	PCT_TRIP_TO_PU(inv_overcurrent_trip_pct,IIA,inv_oc,0)
	flts.inv_oc.delay=parameters.inv_overcurrent_delay_ms;
	break;

	case(22):
	flts.i_unbalance.trip=parameters.current_unbalance_trip_pct_x10;
	flts.i_unbalance.delay=parameters.current_unbalance_delay_10ms;
	break;

	case(23):
	flts.idc_diff.trip=parameters.current_difference_trip_pct*PERCENT_TO_PU;
	flts.idc_diff.delay=parameters.current_difference_delay_ms;
	break;

	case(24):
	flts.iinv_diff.trip=parameters.current_difference_trip_pct*PERCENT_TO_PU;
	flts.iinv_diff.delay=parameters.current_difference_delay_ms;
	break;

	case(25):
	PCT_TRIP_TO_PU(inv_undervolt_trip_pct,VIBC,inv_uv,0)
	CALC_AC_FAULT_DELAY(inv_uv,parameters.inv_undervolt_delay_ms)
	break;

/********************************************************************/
/*	line instantaneous voltage faults and line voltage command		*/
/********************************************************************/
	case(26):
	PCT_TRIP_TO_PU(line_overvolt_trip_inst_pct,VLA,line_ov_inst,0)
	flts.line_ov_inst.delay=parameters.line_overvolt_delay_inst_ms*5;
	PCT_TRIP_TO_PU(line_undervolt_trip_inst_pct,VLA,line_uv_inst,0)
	flts.line_uv_inst.delay=parameters.line_undervolt_delay_inst;
	vac_cmd_pu = 100 * PERCENT_TO_PU;
	break;

/********************************************************************/
/*	ground faults													*/
/********************************************************************/
	case(27):
	flts.fuse_voltage.trip=parameters.fuse_volt_fault_trip;
	flts.fuse_voltage.delay=parameters.fuse_volt_fault_delay_ms;
	break;

	case(28):
		flts.gnd_impedance.trip=parameters.gnd_impedance_trip;
		flts.gnd_impedance.delay=parameters.gnd_impedance_delay_ms;
		flts.dc_bus_imbalance.trip=parameters.dc_bus_imbalance_trip;
		flts.dc_bus_imbalance.delay=parameters.dc_bus_imbalance_delay_ms;
	break;

/********************************************************************/
/*	line frequency fault delays										*/
/********************************************************************/
	case(29):
	CALC_AC_FAULT_DELAY(overfreq,parameters.over_frequency_delay_ms)
	break;

	case(30):
	temp1=parameters.under_frequency_delay_10ms-(parameters.contactor_delay_ms>>3);
	if (temp1 < 0)
		temp1=0;
	flts.underfreq.delay=temp1;
	CALC_AC_FAULT_DELAY(underfreq_inst,160)
//	flts.underfreq_inst.trip = -parameters.under_frequency_trip_Hz_x10;
	flts.underfreq_inst.trip=-300;		// 3.00 Hz
//	flts.underfreq_inst.trip=-500;		// 5.00 Hz

	break;

/********************************************************************/
/*	neutral overcurrent fault										*/
/********************************************************************/
	case(31):
	flts.line_oc_neutral.trip=parameters.neutral_overcurrent_trip_amps_x10;
	flts.line_oc_neutral.delay=parameters.neutral_overcurrent_delay_ms;
	break;

/********************************************************************/
/*	copy frequency fault trip levels								*/
/********************************************************************/
	case(32):
	flts.overfreq.trip=parameters.over_frequency_trip_Hz_x10;
	break;

	case(33):
	flts.underfreq.trip=-parameters.under_frequency_trip_Hz_x10;
	break;
	
/********************************************************************/
/*	calculate maximum d-axis current limit							*/
/********************************************************************/
	case(34):
	temp1=MULQ_RND(12,(int)(4096.0*PI/2.0),flts.line_oc.trip);	// convert average to peak
	temp1-=(PER_UNIT/20);
	LIMIT_MAX_MIN(temp1,I_CMD_MAX,0)
	i_cmd_max=temp1;
	break;

/********************************************************************/
/*	Convert mppt parameters to per unit								*/
/********************************************************************/
	case(35):
#if 0
	temp1=MULQ_RND(12,parameters.vdc_step_min_volts_x10,dc_volts_to_pu_q12);
	// extra factor of 4 gives step sizes similar to Gen I
	vdc_step_min_pu=MULQ(12,temp1,(4*4096/10));
	if (vdc_step_min_pu==0)
		vdc_step_min_pu=1;
	temp1=MULQ_RND(12,parameters.vdc_step_max_volts_x10,dc_volts_to_pu_q12);
	vdc_step_max_pu=MULQ(12,temp1,(4*4096/10));
	if (vdc_step_max_pu < vdc_step_min_pu)
		vdc_step_max_pu=vdc_step_min_pu+(PERCENT_TO_PU/10);
	
#endif
	break;

	case(36):

	k_anti_island_q12=div_q12(parameters.k_anti_island*(PER_UNIT/100),rated_Hz);

	break;

	case(37):
	gating_phase_shift=MULQ_RND(12,parameters.gating_phase_shift,(int)(4096.0*(float)THREE_SIXTY_DEGREES/3600.0));
	break;

	case(38):
	LIMIT_MAX_MIN(parameters.k_voltffd,100,0)
	k_vffd_q12=MULQ(9,parameters.k_voltffd,(int)(512.0*4096.0/100.0));
	break;

	case(39):

		real_power_rate_q12 = div_q12(parameters.real_power_rate,pwm_Hz);
		reactive_power_rate_q12 = div_q12(parameters.reactive_power_rate,pwm_Hz);

		break;

	case(40):


		temp1 = parameters.power_factor_cmd_pct;

		// limit power factor commands
		{
			if((temp1 > 0) && (temp1 < parameters.pos_limit_power_factor_cmd_pct)) {
				parameters.power_factor_cmd_pct = temp1 = parameters.pos_limit_power_factor_cmd_pct;
			}

			if((temp1 < 0) && (temp1 > parameters.neg_limit_power_factor_cmd_pct)) {
				parameters.power_factor_cmd_pct = temp1 = parameters.neg_limit_power_factor_cmd_pct;
			}

			temp2=div_q12(long_sqrt((long)(PER_UNIT_F*PER_UNIT_F)-(long)temp1*temp1),temp1);

			k_pf_q12 = temp2*phase_rotation;
		}

	break;

	case(41):

	{
		debugIndex = debug_control;
	}

	break;

/********************************************************************/
/*	Initialize temperature faults									*/
/********************************************************************/
	case(42):
	if ((temp1=string_index&7) < 2)
	{								// air temperatures
		flts.high_temp[temp1].trip=parameters.high_temp_trip[temp1];
	}
	else
	{								// heatsink temperatures
		flts.high_temp[temp1].trip=parameters.high_temp_trip[2];
	}
	flts.high_temp[temp1].delay=1000/10;
	flts.temp_fbk.delay=10*100;
	break;
	
/********************************************************************/
/*	Voltage regulator gains											*/
/********************************************************************/
	case(43):
	LIMIT_MAX_MIN(parameters.kp_dc_volt_reg,7999,0)
	kp_vdc_q12=parameters.kp_dc_volt_reg*(int)(4096.0/1000.0);
	LIMIT_MAX_MIN(parameters.ki_dc_volt_reg,7999,0)
	temp1=MULQ(12,parameters.ki_dc_volt_reg,(int)(4096.0*PWM_HZ_NOMINAL/1000.0));
	ki_vdc_q12=div_q12(temp1,pwm_Hz);
	break;

/********************************************************************/
/*	Inverter current regulator gains								*/
/********************************************************************/
	case(44):
	LIMIT_MAX_MIN(parameters.kp_inv_cur_reg,7999,0)
	kp_inv_q12=parameters.kp_inv_cur_reg*(int)(4096.0/1000.0);
	LIMIT_MAX_MIN(parameters.ki_inv_cur_reg,7999,0)
	temp1=MULQ(12,parameters.ki_inv_cur_reg,(int)(4096.0*PWM_HZ_NOMINAL/1000.0));
	ki_inv_q12=div_q12(temp1,pwm_Hz);
	break;

/********************************************************************/
/*	Line current regulator gains									*/
/********************************************************************/
	case(45):
	LIMIT_MAX_MIN(parameters.kp_line_cur_reg,7999,0)
	kp_line_q12=parameters.kp_line_cur_reg*(int)(4096.0/1000.0);
	LIMIT_MAX_MIN(parameters.ki_line_cur_reg,7999,0)
	temp1=MULQ(12,parameters.ki_line_cur_reg,(int)(4096.0*PWM_HZ_NOMINAL/1000.0));
	ki_line_q12=div_q12(temp1,pwm_Hz);
	break;

/********************************************************************/
/*	Temperature regulator gains	for adjustable speed fan			*/
/********************************************************************/
	case(46):
	LIMIT_MAX_MIN(parameters.kp_temp_reg,5000,0)
	kp_fan_q16=parameters.kp_temp_reg*(int)(65535.0/10000.0);
	LIMIT_MAX_MIN(parameters.ki_temp_reg,5000,0)
	ki_fan_q16=parameters.ki_temp_reg*(int)(65535.0/10000.0);
	LIMIT_MAX_MIN(parameters.fan_speed_min_pct,100,0)
	fan_speed_min_pu=parameters.fan_speed_min_pct*PERCENT_TO_PU;
	break;
/********************************************************************/
/*	Overtemperature regulator gains									*/
/********************************************************************/
	case(47):
	LIMIT_MAX_MIN(parameters.kp_overtemp_reg,500,0)
	kp_overtemp_q16=parameters.kp_overtemp_reg*(int)(65535.0/1000.0);
	LIMIT_MAX_MIN(parameters.ki_overtemp_reg,500,0)
	ki_overtemp_q16=parameters.ki_overtemp_reg*(int)(65535.0/1000.0);
	break;
	
/********************************************************************/
/*	On/off temperatures for fixed speed fan							*/
/********************************************************************/
	case(48):
	fan_on_temp=parameters.fan_on_temp;
	if (fan_on_temp > (temp1=parameters.high_temp_trip[2]-2))
		fan_on_temp=temp1;
	fan_off_temp=parameters.fan_off_temp;
	if (fan_off_temp > (temp1=fan_on_temp-2))
		fan_off_temp=temp1;
	break;

/********************************************************************/
/*	Setup data capture												*/
/********************************************************************/
	case(49):
	if (++channel >= DATA_CHANNELS)
		channel=0;
	setup_data_channel(channel);								break;
	case(50):if (test_mode!=ANALOG_IO_TEST)
			 setup_analog_out(channel);							break;
	case(51):setup_trigger_channel();							break;
	case(52):setup_trigger_condition();							break;
	case(53):setup_trigger_level();								break;
	case(54):setup_trigger_delay();								break;
	case(55):setup_trigger_interval();							break;
	
/********************************************************************/
/*		Convert test frequency to internal units					*/
/*		f_test_long=(freq_simulate/10)*(pll_Hz_long/2)/65536		*/
/*			  =(freq_simulate*2)*(pll_Hz_long/40)/65536				*/
/********************************************************************/
	case(56):
	f_test_long=(long)(parameters.freq_simulate<<1)*(int)((pll_Hz_long*1638)>>16);
	break;

/********************************************************************/
/*		Calculate display filter gain								*/
/********************************************************************/
	case(57):
    LIMIT_MAX_MIN(parameters.t_disp_fltr_100ms,50,1)
	temp1=parameters.t_disp_fltr_100ms*rated_Hz;
    k_disp_fltr_q16=div_q12((int)((65536.0/4096.0)*(1000.0/100.0)),temp1);
	if (k_disp_fltr_q16==0)
		k_disp_fltr_q16=1;
    break;

/********************************************************************/
/*		Determine Modbus access level								*/
/********************************************************************/
	case(58):
	temp1=parameters.modbus_access_code;
	if (temp1==parameters.access_code_3)
		modbus_access_level=3;
	else if (temp1==parameters.access_code_2)
		modbus_access_level=2;
	else if (temp1==parameters.access_code_1)
		modbus_access_level=1;
	else
		modbus_access_level=0;
	break;

/********************************************************************/
/*		Calculate ramp rates										*/
/********************************************************************/
	case(59):
	temp1=MULQ(12,pwm_Hz,parameters.ramp_time_ms);
	ramp_rate_pu_q16=(long)div_q12((int)(PER_UNIT_F*1000.0*65536.0/(4096.0*4096.0)),temp1);
	fast_ramp_pu_q16=(long)(65536/(5*PWR_CURVE_SIZE*CYCLES_PER_SAMPLE))*rated_Hz;
	break;

/********************************************************************/
/*	Voltage regulator gains											*/
/********************************************************************/
	case(60):
	LIMIT_MAX_MIN(parameters.kp_ac_volt_reg,7999,0)
	kp_vac_q12=parameters.kp_ac_volt_reg*(int)(4096.0/1000.0);
	LIMIT_MAX_MIN(parameters.ki_ac_volt_reg,7999,0)
	temp1=MULQ(12,parameters.ki_ac_volt_reg,(int)(4096.0*PWM_HZ_NOMINAL/1000.0));
	ki_vac_q12=div_q12(temp1,pwm_Hz);
	break;

	case(61):
	if (!(status_flags&STATUS_INITIALIZED))
		status_flags|=STATUS_INITIALIZED;
	break;
	}	// end switch
}	// end slow _update
