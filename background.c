/********************************************************************/
/********************************************************************/
/*																	*/
/*						Background Module							*/
/*																	*/
/********************************************************************/
/********************************************************************/

#include "2812reg.h"
#include "literals.h"
#include "struct.h"
#include "io_def.h"
#include "faults.h"

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
1 MW version
2009-01-19	call parameter_calc function after saving parameters
			with parameter change fault active (eliminates the 
			processor reset previously required)
2009-03-09	delete FPGA version fault
2009-03-23	add call to flash programming function
2009-08-18	add call to modbus_background
*/
/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern int global_flags,save_command;
extern int fpga_version,software_version;
extern int status_flags,nvram_busy,operating_state,modbus_idle_timer;
extern int faults[FAULT_WORDS];
extern struct PARAMS parameters;
extern struct SAVED_VARS saved_variables;
extern struct FAULT_QUEUE fault_queue;
extern struct NV_RAM saved_data,saved_data_2;
extern struct PCS_OP comm_a;
extern struct CHKSUMS checksums;
extern struct SCI_DATA sci_data_b;

/********************************************************************/
/*		External functions											*/
/********************************************************************/
extern unsigned simple_checksum(register unsigned far *pntr,register unsigned long words); 
extern void parameter_calc(void);
extern void load_flash_code(void);
extern void modbus_background(void);

/********************************************************************/
/********************************************************************/
/*		Background function											*/
/********************************************************************/
#pragma CODE_SECTION(background,".rom_code")

void background (void)
{
	register int temp;

	#define MAJOR_VERSION_NUMBER	(17)
	#define MINOR_VERSION_NUMBER	(9)

	software_version=100*MAJOR_VERSION_NUMBER + MINOR_VERSION_NUMBER;

	while(1)
	{   
	if (parameters.testpoint_select==TP48_BKGND)
		PF2.GPATOGGLE=TP48_OUTPUT;

/********************************************************************/
/*	Check FPGA bus interface										*/
/********************************************************************/
	fpga_version=VERSION_INPUT;
	temp=1;
	while (temp!=0)
	{
		FPGA_TEST_OUTPUT=temp;
		if (FPGA_TEST_INPUT!=temp)
			SET_FAULT(FPGA_FLT)
		temp<<=1;
	}	

/********************************************************************/
/*	Save fault queue in NVRAM										*/
/********************************************************************/
		// save request from manage_queue
		if ((global_flags&SAVE_QUEUE) && (nvram_busy==0))
		{
			if (parameters.testpoint_select==TP48_QUE)
				PF2.GPASET=TP48_OUTPUT;
			write_nvram((unsigned far*)&fault_queue,(unsigned far*)&saved_data.fault_queue,sizeof(fault_queue));
			global_flags&=~SAVE_QUEUE;
			if (parameters.testpoint_select==TP48_QUE)
				PF2.GPACLEAR=TP48_OUTPUT;
		}

/********************************************************************/
/*	Save primary parameters in NVRAM								*/
/********************************************************************/
		if (((global_flags&SAVE_PARAMETERS)||(save_command==1)||(comm_a.save_command!=0)) && (nvram_busy==0))
		{
			// do not include last word (chksum) in calculation
			parameters.chksum=simple_checksum((unsigned far*)&parameters,sizeof(parameters)-1);
			write_nvram ((unsigned far*)&parameters,(unsigned far*)&saved_data.parameters,sizeof(parameters));
			write_nvram ((unsigned far*)&parameters,(unsigned far*)&saved_data_2.parameters,sizeof(parameters));
			global_flags&=~SAVE_PARAMETERS;
			comm_a.save_command=0;
			if (save_command==1)
				save_command=0;
			if (FAULT_BIT(PARAM_CHANGE_FAULT) && (operating_state <= STATE_STOP))
			{
				parameter_calc();
				CLEAR_FAULT(PARAM_CHANGE_FAULT)
			}
		}

/********************************************************************/
/*	Save secondary parameters in NVRAM								*/
/********************************************************************/
		else if ((save_command==3) && (nvram_busy==0))
		{
			// do not include last word (chksum) in calculation
			parameters.chksum=simple_checksum((unsigned far*)&parameters,sizeof(parameters)-1);
			write_nvram ((unsigned far*)&parameters,(unsigned far*)&saved_data.parameters_2,sizeof(parameters));
			write_nvram ((unsigned far*)&parameters,(unsigned far*)&saved_data_2.parameters_2,sizeof(parameters));
			save_command=0;
		}

/********************************************************************/
/*	Recall primary parameters from NVRAM							*/
/********************************************************************/
		else if (save_command==2)
		{
			if (!(status_flags & STATUS_RUN))
			{
				read_nvram ((unsigned far*)&saved_data.parameters,(unsigned far*)&parameters,sizeof(parameters));
				checksums.params=simple_checksum((unsigned far*)&parameters,sizeof(parameters)-1);
				if (checksums.params!=parameters.chksum)
				{	// if checksum error recall second copy
					SET_FAULT(PARAM_CHKSUM_FLT_A1)
					read_nvram ((unsigned far*)&saved_data_2.parameters,(unsigned far*)&parameters,sizeof(parameters));
					checksums.params=simple_checksum((unsigned far*)&parameters,sizeof(parameters)-1);
					if (checksums.params!=parameters.chksum)
						SET_FAULT(PARAM_CHKSUM_FLT_A2)
				}
			}
			save_command=0;
		}

/********************************************************************/
/*	Recall secondary parameters from NVRAM							*/
/********************************************************************/
		else if (save_command==4)
		{
			if (!(status_flags & STATUS_RUN))
			{
				read_nvram ((unsigned far*)&saved_data.parameters_2,(unsigned far*)&parameters,sizeof(parameters));
				checksums.params=simple_checksum((unsigned far*)&parameters,sizeof(parameters)-1);
				if (checksums.params!=parameters.chksum)
				{	// if checksum error recall second copy
					SET_FAULT(PARAM_CHKSUM_FLT_B1)
					read_nvram ((unsigned far*)&saved_data_2.parameters_2,(unsigned far*)&parameters,sizeof(parameters));
					checksums.params=simple_checksum((unsigned far*)&parameters,sizeof(parameters)-1);
					if (checksums.params!=parameters.chksum)
					SET_FAULT(PARAM_CHKSUM_FLT_B2)
				}
			}
			save_command=0;
		}

/********************************************************************/
/*	Save variables in NVRAM											*/
/********************************************************************/
		if (global_flags&SAVE_VARIABLES)
		{		// do not include first word (checksum) in calculation
			saved_variables.chksum=simple_checksum((unsigned far*)&saved_variables+1,SAVED_VARS_SIZE-1);
			if (parameters.testpoint_select==TP48_VAR)
				PF2.GPASET=TP48_OUTPUT;
			write_nvram((unsigned far*)&saved_variables,(unsigned far*)&saved_data.saved_variables,SAVED_VARS_SIZE);
			global_flags&=~SAVE_VARIABLES;

			if (parameters.testpoint_select==TP48_VAR)
				PF2.GPACLEAR=TP48_OUTPUT;
		}

		modbus_background();

/********************************************************************/
/*	Reprogram flash memory											*/
/********************************************************************/
	/*	if (parameters.initialize_cmd==7)
		{
			if ((operating_state <= STATE_STOP) && (modbus_idle_timer==MODBUS_IDLE_DELAY))
			{
				parameters.initialize_cmd=0;
				load_flash_code();
			}
		}*/
	}	// bottom of background loop
}		// end of background function

