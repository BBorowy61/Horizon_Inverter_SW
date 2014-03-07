/********************************************************************/
/********************************************************************/
/*																	*/
/*					Control Board Test Module						*/
/*																	*/
/********************************************************************/
/********************************************************************/

#include "literals.h"
#include "2812reg.h"
#include "io_def.h"
#include "faults.h"
#include "struct.h"

#define SHORT_DELAY		100
#define LONG_DELAY		500

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
#pragma DATA_SECTION(ext_input_faults,".test_data")		
#pragma DATA_SECTION(ext_output_faults,".test_data")		
#pragma DATA_SECTION(io_test_timer,".test_data")		
#pragma DATA_SECTION(test_case,".test_data")		
#pragma DATA_SECTION(analog_input_faults,".test_data")		
#pragma DATA_SECTION(analog_faults_temp,".test_data")		
#pragma DATA_SECTION(bipolar_test_data,".test_data")		
#pragma DATA_SECTION(unipolar_test_data,".test_data")		
#pragma DATA_SECTION(error_count,".test_data")		
#pragma DATA_SECTION(gating_faults,".test_data")		
#pragma DATA_SECTION(gating_mask,".test_data")		
#pragma DATA_SECTION(serial_comm_faults,".test_data")		
int ext_input_faults,ext_output_faults,io_test_timer,test_case;
long analog_input_faults,analog_faults_temp;
int bipolar_test_data,unipolar_test_data,error_count;
int gating_faults,gating_mask,serial_comm_faults;

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern int ext_inputs,ext_outputs_display,output_test,testmode_param;
extern int *dac_addr[3],dac_shift[3];
extern int gate_test_select,fpga_outputs,fpga_inputs;
extern int string_amps[STRING_FDBK_SIZE];
extern struct PARAMS parameters;

const far int pwm_fdbk_shift_table[13]={0,10-0,12-1,11-2,10-3,12-4,11-5,13-6,15-7,14-8,13-9,15-10,14-11};
#pragma DATA_SECTION(pwm_fdbk_shift_table,".rom_code")
#pragma CODE_SECTION(analog_io_test,".rom_code")
#pragma CODE_SECTION(external_io_test,".rom_code")
#pragma CODE_SECTION(gating_io_test,".rom_code")
#pragma CODE_SECTION(serial_comm_test,".rom_code")

/********************************************************************/
/********************************************************************/
/*		Analog I/O test function									*/
/********************************************************************/

void analog_io_test(void)
{
	register int temp,index,error_max;

/********************************************************************/
/*	Set analog outputs to low level									*/
/********************************************************************/
	if (test_case==0)
	{
		bipolar_test_data=unipolar_test_data=(int)(-0.80*DAC_HALF_SCALE);	// 0.5V
		dac_addr[0]=&bipolar_test_data;	// ch 1-4	-5 to +5V
		dac_addr[2]=&bipolar_test_data;
		dac_addr[1]=&unipolar_test_data;
		dac_shift[0]=dac_shift[1]=dac_shift[2]=0;
		analog_input_faults=0;
		gating_faults&=~(DAC_FLT_1|DAC_FLT_2|DAC_FLT_3);
		io_test_timer=256*16;
		test_case=1;
	}

/********************************************************************/
/*	Check analog inputs												*/
/********************************************************************/
	else if (test_case==1)
	{
		if (io_test_timer > 0)
		{
			io_test_timer--;
			if ((io_test_timer & 255)==0)
				analog_input_faults=(analog_input_faults<<1)+1;
		}
		else
		{
			error_max=parameters.a2d_error_mv;
			analog_faults_temp=0L;		// start with all faults cleared
			for (index=0;index<16;index++)
			{									// bipolar inputs
				temp=string_amps[index];
				if ((temp < (500-error_max)) || (temp > (500+error_max)))
					analog_faults_temp|=(1L<<index);
			}
			index=16;							// calibration inputs
			temp=string_amps[index];
			if ((temp < (2048-100)) || (temp > (2048+100)))
				analog_faults_temp|=(1L<<index);	
			index=17;
			temp=string_amps[index];
			if ((temp < (1048-100)) || (temp > (1048+100)))
				analog_faults_temp|=(1L<<index);	
			for (index=18;index<24;index++)
			{									// unipolar inputs
				temp=string_amps[index];
				if ((temp < (500-error_max)) || (temp > (500+error_max)))
					analog_faults_temp|=(1L<<index);
			}
			index=24;							// calibration inputs
			temp=string_amps[index];
			if ((temp < (2048-100)) || (temp > (2048+100)))
				analog_faults_temp|=(1L<<index);	
			index=25;
			temp=string_amps[index];
			if ((temp < (1048-100)) || (temp > (1048+100)))
				analog_faults_temp|=(1L<<index);	
			for (index=26;index<32;index++)
			{									// unipolar inputs
				temp=string_amps[index];
				if ((temp < (500-error_max)) || (temp > (500+error_max)))
					analog_faults_temp|=(1L<<index);
			}

/********************************************************************/
/*	Set analog outputs to high level								*/
/********************************************************************/
			bipolar_test_data=(int)(0.80*DAC_HALF_SCALE);	// 4.5 V
			unipolar_test_data=0;							// 2.5 V
			io_test_timer=256*16;
			test_case=2;
		}
	}	// end test_case==1

/********************************************************************/
/*	Check analog inputs again										*/
/********************************************************************/
	else if (test_case==2)
	{
		if (io_test_timer > 0)
		{
			io_test_timer--;
			if ((io_test_timer & 255)==0)
				analog_input_faults=(analog_input_faults<<1)+1;
		}
		else
		{
			error_max=parameters.a2d_error_mv;
			for (index=0;index<16;index++)
			{									// bipolar inputs
				temp=string_amps[index];
				if ((temp < (4500-error_max)) || (temp > (4500+error_max)))
					analog_faults_temp|=(1L<<index);
			}
			for (index=18;index<24;index++)
			{									// unipolar inputs
				temp=string_amps[index];
				if ((temp < (2500-error_max)) || (temp > (2500+error_max)))
					analog_faults_temp|=(1L<<index);
			}
			for (index=26;index<32;index++)
			{
				temp=string_amps[index];
				if ((temp < (2500-error_max)) || (temp > (2500+error_max)))
					analog_faults_temp|=(1L<<index);
			}

			temp=analog_faults_temp;
			if ( ((temp&0x0F)==0x0F) &&
				((temp&0xFFF0)!=0xFFF0) )
			{
				analog_faults_temp&=0xFFF0;
				gating_faults|=DAC_FLT_1;
			}
			temp=analog_faults_temp;
			if (temp==0xFFF0)
			{
				analog_faults_temp&=0x000F;
				gating_faults|=DAC_FLT_3;
			}
			temp=analog_faults_temp>>16;
			if (temp==0xFCFC)
			{
				analog_faults_temp&=0x0000FFFF;
				gating_faults|=DAC_FLT_2;
			}

			analog_input_faults=analog_faults_temp;
			test_case=testmode_param=0;
		}
	}	// end test_case==2
}		// end analog_io_test function

/********************************************************************/
/********************************************************************/
/*		Digital I/O test function									*/
/********************************************************************/

void external_io_test(void)
{

/********************************************************************/
/*	Connect external inputs to external outputs						*/
/********************************************************************/
	if (test_case==0)
	{
		fpga_outputs|=(FAN_PWM_DISABLE_1|FAN_PWM_DISABLE_2);
		PF2.GPACLEAR=DIO_17_GPA|DIO_18_GPA|DIO_19_GPA|DIO_20_GPA;
		ext_input_faults=ext_output_faults=~0;	// set fault bits
		io_test_timer=LONG_DELAY;
		output_test=1;						// select first output
		test_case=1;
	}

/********************************************************************/
/*	Wait for outputs to settle										*/
/********************************************************************/
	else if (test_case==1)
	{
		if (io_test_timer > 0)
			io_test_timer--;
		else
		{
			ext_output_faults&=~ext_outputs_display;
			io_test_timer=SHORT_DELAY;
			test_case=2;
		}
	}

/********************************************************************/
/*	Check external outputs											*/
/********************************************************************/
	else if (test_case==2)
	{				// clear fault bit if input and output match
		ext_output_faults|=((~ext_inputs) ^ ext_outputs_display);
		if (io_test_timer > 0)
			io_test_timer--;
		else if (output_test++ < 16)
		{										// not last output
			io_test_timer=LONG_DELAY;
			test_case=1;
		}
		else
			test_case=3;
	}

/********************************************************************/
/*	Force external inputs high										*/
/********************************************************************/
	else if (test_case==3)
		{
			PF2.GPASET=DIO_17_GPA|DIO_18_GPA|DIO_19_GPA|DIO_20_GPA;
			ext_input_faults=0;				// clear input faults
			io_test_timer=LONG_DELAY;
			test_case=4;
		}

/********************************************************************/
/*	Wait for outputs to settle										*/
/********************************************************************/
	else if (test_case==4)
	{
		if (io_test_timer > 0)
			io_test_timer--;
		else
		{
			io_test_timer=LONG_DELAY;
			test_case=5;
		}
	}

/********************************************************************/
/*	Check external inputs (should be zero)							*/
/********************************************************************/
	else if (test_case==5)
	{								// set fault bit if input high
		ext_input_faults|=ext_inputs;
		if (io_test_timer > 0)
			io_test_timer--;

/********************************************************************/
/*	Clean up and exit												*/
/********************************************************************/
		else
		{
			fpga_outputs&=~(FAN_PWM_DISABLE_1|FAN_PWM_DISABLE_2);
			PF2.GPACLEAR=DIO_17_GPA|DIO_18_GPA|DIO_19_GPA|DIO_20_GPA;
			ext_output_faults&=~ext_input_faults;
			if (ext_output_faults==0xFFFF)
				gating_faults|=DO_FLT_17_20;
			test_case=output_test=testmode_param=0;
		}
	}
}		// end external_io_test function

/********************************************************************/
/********************************************************************/
/*		Gating I/O test function									*/
/********************************************************************/

void gating_io_test(void)
{
	if (test_case==0)
	{								// initialize and start test
		io_test_timer=LONG_DELAY;
		gate_test_select=gating_mask=gating_faults=0;
		test_case=1;
	}

	else if (test_case==1)
	{								// select and gate next device
		if (io_test_timer > 0)
			io_test_timer--;
		else if (gate_test_select < 12)
		{
			gating_mask=1<<gate_test_select++;
			gating_faults|=gating_mask;	// set fault for this device
			io_test_timer=LONG_DELAY;
			test_case=2;
		}
		else
			test_case=gate_test_select=testmode_param=0;
	}

	else if (test_case==2)
	{					// wait LONG_DELAY before checking feedback
		if (io_test_timer > 0)
			io_test_timer--;
		else
		{
			io_test_timer=SHORT_DELAY;
			test_case=3;
		}
	}

	else if (test_case==3)
	{				// if feedback correct clear fault bit for selected device
		gating_faults&=((PWM_FLT_INPUT>>(pwm_fdbk_shift_table[gate_test_select]))|~gating_mask);
		if (io_test_timer > 0)
			io_test_timer--;
		else 
			test_case=1;		// select next device
	}
}

/********************************************************************/
/********************************************************************/
/*		Serial communication test function							*/
/********************************************************************/

void serial_comm_test(void)
{
	switch (test_case)
	{
		case(0):						// initialize and start test
		asm("	EALLOW");				// enable protected access
		PF2.GPBSET=RS485_TXEN_OUTPUT;	// transmit mode
		serial_comm_faults=(RX_C_TX_B|RX_B_TX_C);
		PF2.GPGMUX=0;					// configure as GPIO
		io_test_timer=LONG_DELAY;
		PF2.GPGDIR=SCI_B_TX;			// configure SCI_B_TX as output
		test_case++;
		PF2.GPGSET=SCI_B_TX;			// set SCI_B_TX high
		error_count=0;
		fpga_outputs&=~TX_FPGA;			// set TX_FPGA low
		break;

		case(1):						// check channel C RX
		if ((io_test_timer--) < 10)
		{
			if (io_test_timer > 0)
			{
				if ((fpga_inputs & 0x0100)!=0)	// should be low
					error_count++;
			}
			else
			{
				PF2.GPGCLEAR=SCI_B_TX;		// set SCI_B_TX low
				io_test_timer=LONG_DELAY;
				test_case++;
			}
		}
		break;

		case(2):						// check channel C RX
		if ((io_test_timer--) < 10)
		{
			if (io_test_timer > 0)
			{
				if ((fpga_inputs & 0x0100)==0)	// should be high
					error_count++;
			}
			else
			{
				if (error_count < 3)
					serial_comm_faults&=~RX_C_TX_B;
				error_count=0;
				fpga_outputs|=TX_FPGA;		// set TX_FPGA high
				io_test_timer=LONG_DELAY;
				test_case++;
			}
		}
		break;

		case(3):						// check channel B RX
		if ((io_test_timer--) < 10)
		{
			if (io_test_timer > 0)
			{
				if ((PF2.GPGDAT & SCI_B_RX)!=0)	// should be low
					error_count++;
			}
			else
			{
				fpga_outputs&=~TX_FPGA;		// set TX_FPGA low
				io_test_timer=LONG_DELAY;
				test_case++;
			}
		}
		break;

		case(4):						// check channel B RX
		if ((io_test_timer--) < 10)
		{
			if (io_test_timer > 0)
			{
				if ((PF2.GPGDAT & SCI_B_RX)==0)	// should be high
					error_count++;
			}
			else
			{
				if (error_count < 3)
					serial_comm_faults&=~RX_B_TX_C;
				PF2.GPGMUX=(SCI_B_TX|SCI_B_RX);	// configure as SCI-B
				PF2.GPBCLEAR=RS485_TXEN_OUTPUT;	// receive mode
				test_case=testmode_param=0;		// test complete
				asm("	EDIS");			// disable protected access
			}
		}
		break;
	}
	FPGA_OUTPUT=fpga_outputs;
}
