/********************************************************************/
/********************************************************************/
/*																	*/
/*    				Operating Sequence Module						*/
/*																	*/
/********************************************************************/
/********************************************************************/

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
Solstice version
2009-10-05	correct error in power factor control
2009-10-22	add ac voltage matching
*/
#include "literals.h"
#include "2812reg.h"
#include "io_def.h"
#include "struct.h"
#include "faults.h"

#define SECOND			1000

/********************************************************************/
/*		Global variables											*/
/********************************************************************/
struct DQ i_cmd_in;
int status_flags,operating_state;
int testmode_param,remote_cmd_param,pc_cmd;
int real_power_cmd_pu,reactive_power_cmd_pu,k_pf_q12;
int *real_power_cmd_pntr,*reactive_power_cmd_pntr;
int ac_contactor_output,dc_contactor_output,precharge_output,contactor_outputs;
int dc_contactor_flag;
/********************************************************************/
/*		Local variables												*/
/********************************************************************/
int sequence_timer_ms,remote_cmd_low,estop_timer_ms,test_mode;

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern int fdbk_pu[FDBK_SIZE],fdbk_avg[FDBK_SIZE];
extern int faults[FAULT_WORDS];
extern int ext_inputs,ext_outputs,fpga_outputs,status_flags,phase_rotation;
extern int inverse_vl_q12,vdc_cmd_pu,dc_volts_to_pu_q12,ac_contactor_input;
extern int inverter_volts_bc,dc_link_volts,thermal_limit,global_flags,v_fdbk_config;
extern struct PARAMS parameters;
extern int debugIndex,debug[],debug_control,dv[],junk;
extern int v0,v1,v2;
extern int d0,d1,d2;
/********************************************************************/
/*		External functions											*/
/********************************************************************/
extern void analog_io_test(void);
extern void external_io_test(void);
extern void gating_io_test(void);
extern void serial_comm_test(void);

/********************************************************************/
/*		Macro definitions											*/
/********************************************************************/
#define NEW_STATE(ST) {sequence_timer_ms=0;operating_state=ST;}

#pragma CODE_SECTION(operating_sequence,".h0_code")

/********************************************************************/
/********************************************************************/
/*		Operating Sequence function									*/
/********************************************************************/

void operating_sequence(void)
{
	register int temp,temp2;

	sequence_timer_ms++;

/********************************************************************/
/*	Process remote command											*/
/********************************************************************/
	temp=remote_cmd_param & remote_cmd_low;

    if((temp&REMOTE_ENABLE)!=0)
    {
		pc_cmd=CMD_START;
   		remote_cmd_low&=~REMOTE_ENABLE;
   		remote_cmd_param&=~REMOTE_ENABLE;
    }

    if((temp&REMOTE_RESET)!=0)
    {
   		pc_cmd=CMD_RESET;
   		remote_cmd_low&=~REMOTE_RESET;
   		remote_cmd_param&=~REMOTE_RESET;
    }

    if((temp&REMOTE_DISABLE)!=0)
    {
		pc_cmd=CMD_STOP;
   		remote_cmd_low&=~REMOTE_DISABLE;
   		remote_cmd_param&=~REMOTE_DISABLE;
    }

    if((temp&REMOTE_ESTOP)!=0)
    {
   		pc_cmd=CMD_SHUTDOWN;
   		remote_cmd_low&=~REMOTE_ESTOP;
   		remote_cmd_param&=~REMOTE_ESTOP;
    }

    if((temp&REMOTE_LOCAL)!=0)
    {
   		pc_cmd=CMD_LOCAL;
   		remote_cmd_low&=~REMOTE_LOCAL;
   		remote_cmd_param&=~REMOTE_LOCAL;
    }

    if((temp&REMOTE_REMOTE)!=0)
    {
   		pc_cmd=CMD_REMOTE;
   		remote_cmd_low&=~REMOTE_REMOTE;
   		remote_cmd_param&=~REMOTE_REMOTE;
    }

    if((temp&REMOTE_ANALOG)!=0)
    {
   		remote_cmd_low&=~REMOTE_ANALOG;
   		remote_cmd_param&=~REMOTE_ANALOG;
    }
   	remote_cmd_low|=~remote_cmd_param;
      	        
/********************************************************************/
/*		Process pc command											*/
/********************************************************************/
    if (pc_cmd==CMD_SHUTDOWN) 
    {
   		parameters.run_enable=0;
		SET_FAULT(SHUTDOWN_FLT)
		pc_cmd=0;
		if (operating_state!=0)
			NEW_STATE(STATE_SHUTDOWN)
    }

	else if (pc_cmd==CMD_STOP)
	{
   		parameters.run_enable=0;
//		SET_FAULT(STOP_FLT)
		dc_contactor_flag=0;
		pc_cmd=0;
	}

	else if (pc_cmd==CMD_START)
	{
   		parameters.run_enable=1;
		CLEAR_FAULT(STOP_FLT)
		pc_cmd=0;
	}

	else if (pc_cmd==CMD_LOCAL)
	{
   		real_power_cmd_pntr=&real_power_cmd_pu;
   		reactive_power_cmd_pntr=&reactive_power_cmd_pu;
		pc_cmd=0;
	}

	else if (pc_cmd==CMD_REMOTE)
	{
   		real_power_cmd_pntr=&real_power_cmd_pu;
   		reactive_power_cmd_pntr=&reactive_power_cmd_pu;
		pc_cmd=0;
	}

	else if (pc_cmd==CMD_ANALOG)
	{
		if (parameters.inv_bridges!=2)
		{
   			real_power_cmd_pntr=&fdbk_pu[EX3];
   			reactive_power_cmd_pntr=&fdbk_pu[EX4];
		}
		pc_cmd=0;
	}
	  
/********************************************************************/
/*		estop reset													*/
/********************************************************************/
	// PFP Test shutdown trigger
    if (((ext_inputs & ESTOP_INPUT)==0 || (FAULT_BIT(ESTOP_FLT))) &&
    	(operating_state!=0) && (operating_state!=STATE_BOARD_TEST)) {
		NEW_STATE(STATE_SHUTDOWN)
		SET_FAULT(ESTOP_FLT)
	}

    if(parameters.estop_reset)
    {
		if (ext_inputs & ESTOP_INPUT)	// estop input closed
    		estop_timer_ms=1;
        else 
        {								// estop input open
        	if(estop_timer_ms>0)
        	{
        		if(++estop_timer_ms>10) 
        		{
        			pc_cmd=CMD_RESET;
        			estop_timer_ms=0;
        		}
        	} 
        }
    }

/********************************************************************/
/*	Test mode														*/
/********************************************************************/
	if (test_mode!=testmode_param)
	{										// test mode has changed
		ext_outputs&=~contactor_outputs;
		test_mode=testmode_param;
		if (test_mode==GATE_TEST)
		{	
			temp=STATUS_GATE_TEST;
			temp2=STATE_GATE_TEST;
		}
		else if (test_mode==OPEN_CCT_TEST)
		{	
			parameters.run_enable=0;
			temp=STATUS_OPEN_CCT_TEST;
			temp2=STATE_OPEN_CCT_TEST;
		}
		else if (test_mode==SHORT_CCT_TEST)
		{	
			parameters.run_enable=0;
			temp=STATUS_SHORT_CCT_TEST;
			temp2=STATE_SHORT_CCT_TEST;
		}
		else if (test_mode==UL_TEST)
		{
			temp2=STATE_UL_TEST;
		}
		else if (test_mode >= ANALOG_IO_TEST)
		{
			temp2=STATE_BOARD_TEST;
		}
		else if (test_mode==SIMULATE)
		{
			temp=STATUS_SIMULATE;
			temp2=STATE_STOP;
		}
		else
		{
			temp=0;
			temp2=STATE_STOP;
		}

		status_flags=(status_flags&~(STATUS_RUN|STATUS_LINE_LINKED|STATUS_TEST_MASK))|temp;
		fpga_outputs|=FLT_RESET_OUTPUT;
		NEW_STATE(temp2)
	}

/********************************************************************/
/*	Open all contactors if not ready								*/
/********************************************************************/
	else if (((status_flags & STATUS_READY)==0)&&(operating_state!=0)
			&&(operating_state < STATE_GATE_TEST))
	{
		ext_outputs&=~contactor_outputs;
		status_flags&=~STATUS_RUN;
		NEW_STATE(STATE_STOP)
	}

	if ((status_flags & STATUS_RUN)==0)
		i_cmd_in.q=0;

/********************************************************************/
/*	Process operating sequence										*/
/********************************************************************/
//    operating_state=STATE_RUN; //rs debug

    switch(operating_state)
    {
   
/********************************************************************/
/*	Power up														*/
/********************************************************************/
    	case(STATE_POWERUP):
		ext_outputs|=GD_RESET_OUTPUT;
		if(sequence_timer_ms > 2*SECOND)
		{
			ext_outputs=ext_outputs&~GD_RESET_OUTPUT;
			NEW_STATE(STATE_SHUTDOWN)
		}
    	break;

/********************************************************************/
/*	Shutdown (contactors open)										*/
/********************************************************************/
    	case(STATE_SHUTDOWN):
			status_flags=(status_flags&~(STATUS_READY|STATUS_RUN))|STATUS_SHUTDOWN;
			ext_outputs&=~contactor_outputs;
			sequence_timer_ms=0;
			dc_contactor_flag=0;
        	if ((status_flags & STATUS_FAULT)==0)
        	{
		    	pc_cmd=0;
		    	status_flags&=~(STATUS_RUN|STATUS_SHUTDOWN);
				NEW_STATE(STATE_STOP)
        	}
    	 	break;

/********************************************************************/
/*	Stopped with precharge open										*/
/********************************************************************/
    	case(STATE_STOP):
			ext_outputs&=~contactor_outputs;
			dc_contactor_flag=0;
			if (status_flags & STATUS_READY) 
			{
	   			ext_outputs|=precharge_output;
				if(ADV_CON_QCLOSE_DC_CON & parameters.adv_control_configuration)
						ext_outputs|=DC_CONTACTOR_OUTPUT;

/* special for Solstice 500kW, UL, with transformer */
if(v_fdbk_config!=1 && parameters.inverter_type==NA) {
ext_outputs|=AC_CONTACTOR_OUTPUT_2;
}
				NEW_STATE(STATE_PRECHG_CLOSED)
			}
	    		break;

/********************************************************************/
/*	Precharge contactor closed										*/
/********************************************************************/
    	case(STATE_PRECHG_CLOSED):
#if 1
   			temp=MULQ(12,parameters.precharge_volts_min,dc_volts_to_pu_q12);
    		if ((fdbk_pu[VDC] > temp) && (sequence_timer_ms > 2*SECOND)) 
    		{
				ext_outputs|=ac_contactor_output;
				NEW_STATE(STATE_1ST_CONTACTOR_CLOSED)
    		}  
    	    else if (sequence_timer_ms > 30*SECOND)
				SET_FAULT(PRECHARGE_FLT)
#endif
    		break;

/********************************************************************/
/*	First contactor closed	(AC contactor)							*/
/********************************************************************/
    	case(STATE_1ST_CONTACTOR_CLOSED):

#if 1
			if (sequence_timer_ms > SECOND)
			{
			
				/* wait for confirmation that the AC contactor has closed */
	    		if(!(ext_inputs & ac_contactor_input) && (sequence_timer_ms >= 2*SECOND)) {

					/* open precharge contactor */
	    			ext_outputs&=~precharge_output;

					if(parameters.power_control_mode==PCM_DC_POWER) {
					}
					else {
					
						/* enable switching for voltage matching */
						status_flags|=STATUS_RUN;
					}
					/* move to voltage matching state */
					NEW_STATE(STATE_MATCH_VOLTAGE)
				}
			} 
#endif
    		break;

/********************************************************************/
/*	Match voltages													*/
/********************************************************************/
    	case(STATE_MATCH_VOLTAGE):

			
			if( (ADV_CON_SKIP_VMATCH & parameters.adv_control_configuration) || (parameters.power_control_mode==PCM_DC_POWER)) //for rectifier mode, no need to match the voltages. so force the condition to true all the time
				temp2=(int)(0.01*PER_UNIT_F); 
			else
			{
				temp2=fdbk_pu[VDC]-fdbk_pu[VDCIN];
/*???*/				vdc_cmd_pu=fdbk_pu[VDCIN]+(int)(0.03*PER_UNIT_F);
			}
		

#if 1
	   		if ((sequence_timer_ms > 5*SECOND) && (abs(temp2) < (int)(0.03*PER_UNIT_F))) {
			   	ext_outputs|=DC_CONTACTOR_OUTPUT;
			   	NEW_STATE(STATE_2ND_CONTACTOR_CLOSED)
			}
			else if(sequence_timer_ms > 10*SECOND) {
				SET_FAULT(VOLT_MATCHING_FLT)
			}
#endif

	   	 	break;

/********************************************************************/
/*	Second contactor closed	(DC contactor)							*/
/********************************************************************/
    	case(STATE_2ND_CONTACTOR_CLOSED):
			dc_contactor_flag=1;
			if (sequence_timer_ms > (int)(2*SECOND)) 
			{
				temp=DC_CONTACTOR_INPUT;

				/* if DC POWER mode - start switching now */
				if(parameters.power_control_mode==PCM_DC_POWER) {
	    			status_flags|=STATUS_RUN;
				}
				
				if ((ext_inputs & temp)==0) {
		    		NEW_STATE(STATE_RUN)
				}
				else {
//		    		NEW_STATE(STATE_RUN)
					SET_FAULT(DCIN_FLT)
				}
			}
	   	 	break;

/********************************************************************/
/*	Running															*/
/********************************************************************/
    	case(STATE_RUN):
    	{
    	 	dc_contactor_flag=1;
    	 	break;
    	}

/********************************************************************/
/*	Gating test mode												*/
/********************************************************************/
		case(STATE_GATE_TEST):
			ext_outputs&=~contactor_outputs;
			if( (fdbk_pu[VDC]>(PER_UNIT/5))
			||(fdbk_pu[VDCIN]>(PER_UNIT/5))
			||(DISCONNECT_CLOSED)
			||((ext_inputs & DC_CONTACTOR_INPUT)==0)
			||((ext_inputs & ac_contactor_input)!=ac_contactor_input) )
			{
				SET_FAULT(TEST_MODE_FLT)
				NEW_STATE(STATE_SHUTDOWN)
			}
			break;

/********************************************************************/
/*	Open and short circuit test mode								*/
/********************************************************************/
    	case(STATE_OPEN_CCT_TEST): case(STATE_SHORT_CCT_TEST):
		{
			if (((ext_outputs & DC_CONTACTOR_OUTPUT)==0) &&
				(abs(fdbk_pu[VDCIN]-fdbk_pu[VDC]) < (PER_UNIT/5)))
			   	ext_outputs|=DC_CONTACTOR_OUTPUT;
			if (((ext_outputs & ac_contactor_output)==0) &&
				((status_flags & STATUS_PLL_ENABLED)==0))
			   	ext_outputs|=ac_contactor_output;

			if (((status_flags & STATUS_FAULT)==0) &&
				(parameters.run_enable==1) && (ext_inputs & ESTOP_INPUT)
				&& ((ext_inputs & ON_OFF_INPUT)!=0))
				status_flags|=(STATUS_READY|STATUS_RUN);
			else
				status_flags&=~(STATUS_READY|STATUS_RUN);
		}
			break;

/********************************************************************/
/*	UL voltage & frequency test mode								*/
/********************************************************************/
    	case(STATE_UL_TEST):
			if ((status_flags & STATUS_LINE_OK) && ((status_flags & STATUS_FAULT)==0))
			    ext_outputs|=ac_contactor_output;
			else
			    ext_outputs&=~ac_contactor_output;
			break;

/********************************************************************/
/*	Board test mode													*/
/********************************************************************/
    	case(STATE_BOARD_TEST):
			if (test_mode==ANALOG_IO_TEST)
				analog_io_test();
			else if (test_mode==DIGITAL_IO_TEST)
				external_io_test();
			else if (test_mode==GATING_IO_TEST)
			{
				status_flags|=STATUS_GATE_TEST;
				gating_io_test();
			}
			else if (test_mode==SERIAL_IO_TEST)
				serial_comm_test();
			break;

	}
}





