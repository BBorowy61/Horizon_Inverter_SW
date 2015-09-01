/********************************************************************/
/********************************************************************/
/*																	*/
/*						PWM Interrupt Module						*/
/*																	*/
/********************************************************************/
/********************************************************************/

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
1 MW version
2008-10-06	add call to spi_slave function
2009-01-06	expand digital outputs from 16 to 20 outputs
2009-02-10	add external interrupt 1 function
2009-02-11	move 1 ms tasks from timer 0 interrupt
2009-02-25	call 100 ms tasks from this module
2009-03-09	disable pwm sync fault in all test modes
2009-05-27	add variable fdbk_raw_mv
Solstice version
2009-07-20	delete 1 MW code
			delete external interrupt 1 function
			delete digital output 17-20
2009-08-18	delete calls to Modbus functions
2009-01-14	add digital outputs 17-20
			delete averaging of analog feedback samples
*/
#include "literals.h"
#include "2812reg.h"
#include "io_def.h"
#include "struct.h"
#include "faults.h"

/********************************************************************/
/*		Global variables											*/
/********************************************************************/
int fdbk_pu[FDBK_SIZE];
int fpga_inputs,ext_outputs,fpga_outputs,ext_outputs_display;
int tp48,output_test,pwm_int_us;
int *dac_addr[3],dac_shift[3];
int gate_test_select,sw_fault_output;
int fdbk_offset_idc_pu,fdbk_offset_vdc_pu;
int fdbk_raw_mv,modbus_idle_timer;
#pragma DATA_SECTION(k_fdbk_q15,".fbk_data")
int k_fdbk_q15[RAW_FDBK_SIZE];

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
#pragma DATA_SECTION(fdbk_raw,".fbk_data")
#pragma DATA_SECTION(timer_us,".fbk_data")
#pragma DATA_SECTION(timer_10ms,".fbk_data")
#pragma DATA_SECTION(timer_1ms,".fbk_data")
#pragma DATA_SECTION(fast_task,".fbk_data")
#pragma DATA_SECTION(pwm_cmd_scale,".fbk_data")
#pragma DATA_SECTION(int_register,".fbk_data")
int fdbk_raw[RAW_FDBK_SIZE];
int pwm_cmd_scale;
int timer_us,timer_10ms,timer_1ms,fast_task;
int int_register;


#pragma DATA_SECTION(test_gating_table,".rom_const")
const far int test_gating_table[13]={0,1,1<<5,1<<2,1<<1,1<<4,1<<3,1<<6,1<<11,1<<8,1<<7,1<<10,1<<9};

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern int status_flags,string_index,test_mode;
extern int faults[FAULT_WORDS],fdbk_avg[FDBK_SIZE];
extern int hw_oc_trip,pwm_period,debug[4];
extern int a2d_offset_counts;
extern int ext_input_mask,ext_inputs,ext_output_mask,kwh_index;
extern struct ABCDQM v_cmd_pu;
extern struct PARAMS parameters;
extern struct SCI_DATA sci_data_a;
extern int debugIndex,debug[],debug_control,dv[];

/********************************************************************/
/*		External functions											*/
/********************************************************************/
extern void sci_a(void);
extern void phase_lock_loop(void);
extern void regulators(void);
extern void accumulate_feedback(void);
extern void capture_data(void);
extern void simulate_feedback(void);
// 1 ms tasks
extern void operating_sequence(void);
extern void find_faults(void);
extern void average_ac_feedback(void);  
extern void manage_queue(void);
extern void slow_param_update(void);
extern void sci_respond(void);
// 100 ms tasks
extern void real_time_clock(void);
extern void check_dc_input_voltage(void);
extern void low_power_fault(void);
extern void overtemp_regulator(void);
extern void fan_speed_control(void);
extern void convert_to_display_units(void);
extern void calculate_ground_impedance(void);
extern void calculate_kw_hrs(void);
extern void calculate_string_kwh(void);

#pragma CODE_SECTION(pwm_isr,".h0_code")
#pragma CODE_SECTION(simulate_feedback,".h0_code")


/********************************************************************/
/********************************************************************/
/*		PWM Interrupt function										*/
/*		Called by EVA interrupt at 2*pwm_Hz							*/
/********************************************************************/

interrupt void pwm_isr (void)
{   
	register int temp,temp1;
	register int *src_pntr,*dest_pntr,*k_fdbk_pntr;

	int_register=PF2.EVAIFRA;		// save EVA interrupt register
	PF2.EVAIFRA=0x0280;				// clear T1PINT & T1UFINT flags
	PF0.PIEACK|=0x2;				// acknowledge INT2
	tp48=0;
	if ((parameters.testpoint_select==TP48_PWM)
	|| (parameters.testpoint_select==TP48_FGND))
		PF2.GPASET=tp48=TP48_OUTPUT;
	temp=ADC_START;					// start external A/D conversion
	PF2.ADCTRL2|=0x2000;			// start internal A/D conversion

/********************************************************************/
/* 		Reset Watchdog counter 										*/
/********************************************************************/
	temp=WATCHDOG_INPUT;

/********************************************************************/
/*	Service SCI communication ports on alternate passes			 	*/
/********************************************************************/
	if(!(int_register & 0x80)) {
		sci_a();
	}

/********************************************************************/
/*	Get data for bipolar analog inputs 								*/
/*	Data is left-justified and signed (5V=0x7FFF)					*/
/********************************************************************/
	src_pntr=(int*)&ADC_RD1;
	dest_pntr=&fdbk_raw[0];
	PF2.GPATOGGLE=tp48;						// indicate waiting
	while (((fpga_inputs=FPGA_INPUT) & ADC_BUSY)!=0) {}
	PF2.GPATOGGLE=tp48;						// indicate data ready
	while (src_pntr <=(int*)&ADC_RD4)
	{
	    *dest_pntr++	=*src_pntr;
	    *dest_pntr++	=*src_pntr;
	    *dest_pntr++	=*src_pntr;
	    *dest_pntr++	=*src_pntr;
		src_pntr+=2;
	}
 
/********************************************************************/
/*	Get data for unipolar analog inputs 							*/
/*	Data is left-justified and unsigned (3V=0xFFF0)					*/
/*	Shift right one to convert to signed (3V=0x7FF8)				*/
/********************************************************************/
#if 0
	temp=-a2d_offset_counts;
	PF2.GPATOGGLE=tp48;							// indicate waiting
	while ((PF2.ADCST & 0x000C)!=0) {}
	PF2.GPATOGGLE=tp48;							// indicate data ready
    *(dest_pntr++)	=(PF2.ADCRESULT0>>1);
    *(dest_pntr++)	=(PF2.ADCRESULT1>>1);
    *(dest_pntr++)	=(PF2.ADCRESULT2>>1)+temp;
    *(dest_pntr++)	=(PF2.ADCRESULT3>>1)+temp;
	*(dest_pntr++)	=(PF2.ADCRESULT4>>1)+temp;
	*(dest_pntr++)	=(PF2.ADCRESULT5>>1)+temp;
    *(dest_pntr++)	=(PF2.ADCRESULT6>>1)+temp;
    *(dest_pntr++)	=(PF2.ADCRESULT7>>1)+temp;
   	*(dest_pntr++)	=(PF2.ADCRESULT8>>1);
    *(dest_pntr++)	=(PF2.ADCRESULT9>>1);
    *(dest_pntr++)	=(PF2.ADCRESULT10>>1)+temp;
    *(dest_pntr++)	=(PF2.ADCRESULT11>>1)+temp;
    *(dest_pntr++)	=(PF2.ADCRESULT12>>1)+temp;
    *(dest_pntr++)	=(PF2.ADCRESULT13>>1)+temp;
    *(dest_pntr++)	=(PF2.ADCRESULT14>>1)+temp;
    *(dest_pntr++)	=(PF2.ADCRESULT15>>1)+temp;

#else

	temp = -a2d_offset_counts;

	PF2.GPATOGGLE=tp48;							// indicate waiting
	while ((PF2.ADCST & 0x000C)!=0) {}
	PF2.GPATOGGLE=tp48;							// indicate data ready

	if(temp > 0) {
		temp1 = 0x7ff8 - temp;
	}
	else {
		temp1 = 0x7ff8;
	}

    *(dest_pntr++)	= (PF2.ADCRESULT0>>1);
    *(dest_pntr++)	= (PF2.ADCRESULT1>>1);
    *(dest_pntr++)	= ((temp = (PF2.ADCRESULT2>>1)) < temp1) ? (temp - a2d_offset_counts) : 0x7ff8;
    *(dest_pntr++)	= ((temp = (PF2.ADCRESULT3>>1)) < temp1) ? (temp - a2d_offset_counts) : 0x7ff8;
	*(dest_pntr++)	= ((temp = (PF2.ADCRESULT4>>1)) < temp1) ? (temp - a2d_offset_counts) : 0x7ff8;
	*(dest_pntr++)	= ((temp = (PF2.ADCRESULT5>>1)) < temp1) ? (temp - a2d_offset_counts) : 0x7ff8;
    *(dest_pntr++)	= ((temp = (PF2.ADCRESULT6>>1)) < temp1) ? (temp - a2d_offset_counts) : 0x7ff8;
    *(dest_pntr++)	= ((temp = (PF2.ADCRESULT7>>1)) < temp1) ? (temp - a2d_offset_counts) : 0x7ff8;
   	*(dest_pntr++)	= (PF2.ADCRESULT8>>1);
    *(dest_pntr++)	= (PF2.ADCRESULT9>>1);
    *(dest_pntr++)	= ((temp = (PF2.ADCRESULT10>>1)) < temp1) ? (temp - a2d_offset_counts) : 0x7ff8;
    *(dest_pntr++)	= ((temp = (PF2.ADCRESULT11>>1)) < temp1) ? (temp - a2d_offset_counts) : 0x7ff8;
    *(dest_pntr++)	= ((temp = (PF2.ADCRESULT12>>1)) < temp1) ? (temp - a2d_offset_counts) : 0x7ff8;
    *(dest_pntr++)	= ((temp = (PF2.ADCRESULT13>>1)) < temp1) ? (temp - a2d_offset_counts) : 0x7ff8;
    *(dest_pntr++)	= ((temp = (PF2.ADCRESULT14>>1)) < temp1) ? (temp - a2d_offset_counts) : 0x7ff8;
    *(dest_pntr++)	= ((temp = (PF2.ADCRESULT15>>1)) < temp1) ? (temp - a2d_offset_counts) : 0x7ff8;
#endif

/********************************************************************/
/*	Convert raw analog feedback data to per unit					*/
/********************************************************************/
	if (status_flags & STATUS_SIMULATE)
		simulate_feedback();
	else
    {
		fdbk_raw_mv=MULQ(15,fdbk_raw[kwh_index&(RAW_FDBK_SIZE-1)],3000);

		src_pntr=&fdbk_raw[0];
		dest_pntr=&fdbk_pu[0];
		k_fdbk_pntr=&k_fdbk_q15[0];

		/* on 2nd pass, average signals */
		if((int_register & 0x80) && (1)) {
			while (src_pntr < &fdbk_raw[RAW_FDBK_SIZE-1]) {	// fdbk_pu=fdbk_raw*k_fdbk_q15/32768
				*dest_pntr++=(*dest_pntr+MULQ(15,(*src_pntr++),(*k_fdbk_pntr++)))>>1;
				*dest_pntr++=(*dest_pntr+MULQ(15,(*src_pntr++),(*k_fdbk_pntr++)))>>1;
				*dest_pntr++=(*dest_pntr+MULQ(15,(*src_pntr++),(*k_fdbk_pntr++)))>>1;
				*dest_pntr++=(*dest_pntr+MULQ(15,(*src_pntr++),(*k_fdbk_pntr++)))>>1;
			}
		}
		else {
			while (src_pntr < &fdbk_raw[RAW_FDBK_SIZE-1]) {	// fdbk_pu=fdbk_raw*k_fdbk_q15/32768
				*dest_pntr++=MULQ(15,(*src_pntr++),(*k_fdbk_pntr++));
				*dest_pntr++=MULQ(15,(*src_pntr++),(*k_fdbk_pntr++));
				*dest_pntr++=MULQ(15,(*src_pntr++),(*k_fdbk_pntr++));
				*dest_pntr++=MULQ(15,(*src_pntr++),(*k_fdbk_pntr++));
			}
		}

		/********************************************************************/
		/*	Subtract offset from external analog inputs						*/
		/********************************************************************/
		dest_pntr=&fdbk_pu[EX1-1];
		src_pntr=&parameters.ext_input_offset_1;
		if (((*++dest_pntr)-=*src_pntr++) < 0)
			*dest_pntr=0;
		if (((*++dest_pntr)-=*src_pntr++) < 0)
			*dest_pntr=0;
		if (((*++dest_pntr)-=*src_pntr++) < 0)
			*dest_pntr=0;
		if (((*++dest_pntr)-=*src_pntr++) < 0)
			*dest_pntr=0;

		/********************************************************************/
		/*	Subtract offset from dc voltage feedback inputs					*/
		/********************************************************************/
		fdbk_pu[IDC]+=fdbk_offset_idc_pu;
		fdbk_pu[VDCIN]+=fdbk_offset_vdc_pu;
		fdbk_pu[VDC]+=fdbk_offset_vdc_pu;
    }

/********************************************************************/
/* 	Everything from here on is done every second pass				*/
/********************************************************************/
	if (int_register & 0x80)
	{										// TIPINT interrupt

/********************************************************************/
/* 	Call 200 us functions											*/
/********************************************************************/
		phase_lock_loop();
		regulators();
		accumulate_feedback();
		capture_data();

/********************************************************************/
/* 	Check fault flags 												*/
/********************************************************************/
		if (faults[0]|faults[1])
		{
			status_flags=(status_flags&~(STATUS_READY|STATUS_RUN))|STATUS_FAULT;
			ext_outputs&=~sw_fault_output;
		}
		else
		{
			status_flags&=~STATUS_FAULT;
			ext_outputs|=sw_fault_output;			
		}

/********************************************************************/
/*		Write data to parallel DAC									*/
/********************************************************************/
		if(1) { // pjf

			extern int d0,d1,d2;

			DAC1_DATA= d0;
			DAC2_DATA= d1;
			DAC3_DATA= d2;
		}
		else {
			DAC1_DATA= (((*dac_addr[0])>>dac_shift[0])+DAC_HALF_SCALE)&DAC_FULL_SCALE;
			DAC2_DATA= (((*dac_addr[1])>>dac_shift[1])+DAC_HALF_SCALE)&DAC_FULL_SCALE;
			DAC3_DATA= (((*dac_addr[2])>>dac_shift[2])+DAC_HALF_SCALE)&DAC_FULL_SCALE;
		}

		// -34 compensates for the 0.128 volt shift inherent to the DAC
		// 1024 * (0.128 / (3.968 - 0.128)) = 34
		if((temp=(hw_oc_trip>>3)-34) < 0)
			temp = 0;
		DAC4_DATA = temp & DAC_FULL_SCALE;

/********************************************************************/
/*	Create 3-phase voltage references for inverter bridges			*/
/********************************************************************/
		temp=MULQ(12,PER_UNIT,pwm_cmd_scale);
		PF2.CMPR4=PF2.CMPR1=temp-v_cmd_pu.a;
		PF2.CMPR5=PF2.CMPR2=temp-v_cmd_pu.b;
		PF2.CMPR6=PF2.CMPR3=temp-v_cmd_pu.c;
	}    									//end T1PINT interrupt

/********************************************************************/
/*		Test gating mode											*/
/********************************************************************/
	if(status_flags & STATUS_GATE_TEST)
	{
		temp=test_gating_table[gate_test_select];
		PWM_EN_OUTPUT=temp;
		ext_outputs|=RUN_OUTPUT;
		fpga_outputs|=START_OUTPUT;
	}

/********************************************************************/
/*		Normal gating mode											*/
/********************************************************************/
	else
	{
		PWM_EN_OUTPUT=0x0FFF;
		if (status_flags & STATUS_RUN)
		{
			ext_outputs|=RUN_OUTPUT;
			fpga_outputs|=START_OUTPUT;
		}
		else
		{
			ext_outputs&=~RUN_OUTPUT;
			fpga_outputs&=~START_OUTPUT;
		}
	}

/********************************************************************/
/*		Write digital outputs										*/
/********************************************************************/
	if ((status_flags &(STATUS_RUN|STATUS_READY))==0 && output_test!=0)
	{
		if (output_test <= 16)
		{				// outputs 1-16 are written as a 16-bit word to the FPGA
			ext_outputs_display=1<<((output_test-1)&0x0F);
			PF2.GPASET=DIO_17_GPA|DIO_18_GPA|DIO_19_GPA|DIO_20_GPA;	// off
		}
		else {
			ext_outputs_display = 0;
			if(output_test == 17) {
				PF2.GPASET=DIO_18_GPA|DIO_19_GPA|DIO_20_GPA;	// off
				PF2.GPACLEAR=DIO_17_GPA;						// on
			}
			else if(output_test == 18) {
				PF2.GPASET=DIO_17_GPA|DIO_19_GPA|DIO_20_GPA;	// off
				PF2.GPACLEAR=DIO_18_GPA;						// on
			}
			else if(output_test == 19) {
				PF2.GPASET=DIO_17_GPA|DIO_18_GPA|DIO_20_GPA;	// off
				PF2.GPACLEAR=DIO_19_GPA;						// on
			}
			else if(output_test == 20) {
				PF2.GPASET=DIO_17_GPA|DIO_18_GPA|DIO_19_GPA;	// off
				PF2.GPACLEAR=DIO_20_GPA;						// on
			}
		}

		EXT_OUTPUT=ext_outputs_display;
	}
	else
	{											// normal operation
		fpga_outputs&=~(FAN_PWM_DISABLE_1|FAN_PWM_DISABLE_2);	// enable fan pwm
		output_test=0;

		ext_outputs_display=ext_outputs;
		//outputs 1-16,  clear 4 mux control bits and or in string_index
		EXT_OUTPUT=temp=( 
							(ext_outputs_display & ~0x7800) | //(DIO_17_GPA|DIO_18_GPA|DIO_19_GPA|DIO_20_GPA))|
							((0xf & string_index)<<11)
						) &	ext_output_mask;

		// outputs 17-20 are GPIO bits mapped to bits 12-15 of ext_outputs_display
		if(ext_outputs_display & AC_CONTACTOR_OUTPUT_2)
			PF2.GPACLEAR = AC_CONTACTOR_OUTPUT_2;
		else
			PF2.GPASET = AC_CONTACTOR_OUTPUT_2;
		PF2.GPACLEAR=DIO_18_GPA|DIO_19_GPA|DIO_20_GPA;
	}
	// PFP set fault inductor inhibit
	if(parameters.inhibit_flt_inductor)
		fpga_outputs |= INHIBIT_FAULT_INDUCTOR_TRIP;
	else
		fpga_outputs &= ~(INHIBIT_FAULT_INDUCTOR_TRIP);

	FPGA_OUTPUT=fpga_outputs;

/********************************************************************/
/*		Accumulate PWM interrupt times until they reach 1 ms		*/
/********************************************************************/
	if ((timer_us+=pwm_int_us) >= 1000)
	{
		timer_us-=1000;
		fast_task=0;
	}

	fast_task++;

	if ((fast_task < 9) && ((parameters.testpoint_select==TP48_1MS)	|| (parameters.testpoint_select==TP48_FGND)))
		PF2.GPASET=tp48=TP48_OUTPUT;

	if (fast_task==1)
		operating_sequence();
	else if (fast_task==2)
    	find_faults();
	else if (fast_task==3)
  		average_ac_feedback();  
	else if (fast_task==4)
		manage_queue();
	else if (fast_task==5)
		slow_param_update();
	else if (fast_task==6)
	{
		if (sci_data_a.tx_state==REQUEST_RESP)
			sci_respond();
	}

	else if (fast_task==7)
	{
		if (++timer_1ms > 9)
		{									// perform every 10 ms
			timer_1ms=0;
			if ((parameters.testpoint_select==TP48_10MS)
			|| (parameters.testpoint_select==TP48_FGND))
				PF2.GPASET=tp48=TP48_OUTPUT;
			real_time_clock();
			if (++timer_10ms > 9) {
				timer_10ms=0;				// reset every 100 ms
			}
			if ((parameters.testpoint_select==TP48_100MS)
			|| (parameters.testpoint_select==TP48_FGND))
				PF2.GPASET=tp48=TP48_OUTPUT;
			switch (timer_10ms)
			{
			case(0):	check_dc_input_voltage();			break;
			case(1):	low_power_fault();					break;
		  	case(2):	fan_speed_control();				break;  
		    case(3):	overtemp_regulator();				break;
			case(4):										break;
			case(5):	convert_to_display_units();			break;
			case(6):	calculate_ground_impedance();		break;
			case(7):	calculate_kw_hrs();					break;
			case(8):	calculate_string_kwh();				break;
			case(9):	if (modbus_idle_timer < MODBUS_IDLE_DELAY)
							modbus_idle_timer++;			break;
			}
		}
	}
	else if (fast_task > 9)
		fast_task=9;
	PF2.GPACLEAR=tp48;
}											// end pwm_isr function


	

