/********************************************************************/
/********************************************************************/
/*																	*/
/*						Data Recorder module						*/
/*																	*/
/********************************************************************/
/********************************************************************/

#include "literals.h"
#include "struct.h"
#include "io_def.h"

/********************************************************************
	Definition of 16-bit attribute word

	bit 15-8	8-bit parameter type
	bit 7-6		4-bit modbus access level
	bit	5		not used
	bit 4		parameter fault
	bit 3-0		4-bit shift count		

*********************************************************************/

/********************************************************************/
/*		Literals													*/
/********************************************************************/
#define PRE_TRIGGER_MIN	10

// attributes
#define SHIFT_MASK		0x0007
#define TYPE_SHIFT		8
// trig_state
#define	NOT_READY		0
#define	PRE_TRIGGER		1
#define	POST_TRIGGER	2
#define RESTART			3
// trigger condition
#define EQU				0
#define NEQ				1
#define GT				2
#define LT				3
#define AND				4
#define NAND			5

/********************************************************************/
/*		Global variables											*/
/********************************************************************/
#pragma DATA_SECTION(nvram_busy,".rec_data")
#pragma DATA_SECTION(playback_index,".rec_data")
int nvram_busy,playback_index;

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
#pragma DATA_SECTION(new_setup,".rec_data")
#pragma DATA_SECTION(diag_data,".rec_data")
#pragma DATA_SECTION(data_addr,".rec_data")
#pragma DATA_SECTION(data_attrib,".rec_data")
#pragma DATA_SECTION(trigger_marker,".rec_data")
#pragma DATA_SECTION(old_channels,".rec_data")
#pragma DATA_SECTION(playback,".rec_data")
#pragma DATA_SECTION(trig_state,".rec_data")
#pragma DATA_SECTION(trig_mode,".rec_data")
#pragma DATA_SECTION(trig_index,".rec_data")
#pragma DATA_SECTION(dummy_variable,".rec_data")
#pragma DATA_SECTION(diag_timer,".rec_data")
#pragma DATA_SECTION(trig_delay,".rec_data")
#pragma DATA_SECTION(post_trigger,".rec_data")
#pragma DATA_SECTION(old_trigger_level,".rec_data")
#pragma DATA_SECTION(sample_divider,".rec_data")
#pragma DATA_SECTION(old_trigger_data,".rec_data")
#pragma DATA_SECTION(trig_attrib,".rec_data")
#pragma DATA_SECTION(index,".rec_data")
#pragma DATA_SECTION(trig_addr,".rec_data")
far struct DATA_SETUP new_setup;
int diag_data[DATA_CHANNELS];
int *data_addr[DATA_CHANNELS];
int data_attrib[DATA_CHANNELS];
int trigger_marker[DATA_CHANNELS];
int old_channels[DATA_CHANNELS];
int playback[DATA_CHANNELS];
int trig_state,trig_mode,trig_index,dummy_variable;
int diag_timer,trig_delay,post_trigger,old_trigger_level;
int sample_divider,old_trigger_data,trig_attrib,index;
int *trig_addr;

#pragma DATA_SECTION(power_curve,".rec_data")
far struct PWR_CURVE power_curve;

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern int *dac_addr[3],dac_shift[3];
extern struct PARAMS parameters;
extern far struct NV_RAM saved_data;
extern far struct PARAM_ENTRY param_table[PARAM_MAX+1];

#pragma CODE_SECTION(capture_data,".l1_code")
#pragma CODE_SECTION(setup_data_channel,".l1_code")
#pragma CODE_SECTION(setup_trigger_channel,".l1_code")
#pragma CODE_SECTION(setup_trigger_condition,".l1_code")
#pragma CODE_SECTION(setup_trigger_level,".l1_code")
#pragma CODE_SECTION(setup_trigger_delay,".l1_code")
#pragma CODE_SECTION(setup_trigger_interval,".l1_code")
#pragma CODE_SECTION(setup_analog_out,".l1_code")


/********************************************************************/
/********************************************************************/
/*		Data Capture function	(called by pwm_isr at pwm_Hz)		*/
/********************************************************************/
	
	void capture_data(void)
	{
		register int temp,*src_pntr,*dest_pntr,new_trigger_data;

/********************************************************************/
/*		Initialize data on first pass								*/
/********************************************************************/
		if (sample_divider==0)
		{
			new_setup.trig_channel=sample_divider=1;
			new_setup.pre_trigger=PRE_TRIGGER_MIN;
			post_trigger=DIAG_SIZE-PRE_TRIGGER_MIN;
			data_addr[0]=data_addr[1]=data_addr[2]=data_addr[3]=trig_addr=&dummy_variable;
			data_addr[4]=data_addr[5]=data_addr[6]=data_addr[7]=&dummy_variable;
		}

		switch(trig_state)
		{
			
/********************************************************************/
/*		Record pre trigger data										*/
/********************************************************************/
		case(NOT_READY):
		if (trig_delay > 0)
			trig_delay--;
		else if (trig_mode!=0)
		{
			new_setup.mode=trig_mode;
			if (trig_mode==1)			// single
				trig_mode=0;
			trig_state=PRE_TRIGGER;
		}
		break;
			
/********************************************************************/
/*		Wait for trigger											*/
/********************************************************************/
		case(PRE_TRIGGER):
		new_trigger_data=*trig_addr;
		temp=new_setup.condition;
		if (((temp==EQU) && (new_trigger_data==new_setup.level)) ||
			((temp==NEQ) && (new_trigger_data!=new_setup.level)) ||
			((temp==GT)  && (old_trigger_data<=new_setup.level) && (new_trigger_data>new_setup.level)) ||
			((temp==LT)  && (old_trigger_data>=new_setup.level) && (new_trigger_data<new_setup.level)) ||
			((temp==AND) && (new_trigger_data&new_setup.level)) ||
			((temp==NAND)&& !(new_trigger_data&new_setup.level)))
			{
				trig_index=index;
				trig_delay=post_trigger*sample_divider;
				trig_state=POST_TRIGGER;
			}
		break;
				
/********************************************************************/
/*		Complete post trigger data collection						*/
/********************************************************************/
		case(POST_TRIGGER):
		if (--trig_delay<=0)
		{
			write_nvram((unsigned far*)&new_setup,(unsigned far*)&saved_data.data_setup,sizeof(new_setup));
			playback[0]=playback[1]=playback[2]=playback[3]=0;
			playback[4]=playback[5]=playback[6]=playback[7]=0;
			trig_state=RESTART;
		}
		break;
				
		case(RESTART):
			trig_delay=new_setup.pre_trigger*sample_divider;
			trig_state=NOT_READY;
		break;
		}
			
/********************************************************************/
/*		Play back old data for analog display						*/
/********************************************************************/
		old_trigger_data=*trig_addr;
		if (nvram_busy==0)
		{
			if (++diag_timer >= sample_divider)
			{
				diag_timer=0;
				if (++index >= DIAG_SIZE)
					index=0;
				if ((temp=playback_index)==0)
					temp=index;
				else
				{
					if (temp > DIAG_SIZE)
						temp=DIAG_SIZE;
					temp--;
				}
				read_nvram((unsigned far*)&saved_data.data_record.data[temp<<1][0],(unsigned far*)&playback,DATA_CHANNELS);

/********************************************************************/
/*		Record new data if armed									*/
/********************************************************************/
				if (trig_state!=NOT_READY)
				{
					dest_pntr=(int*)&diag_data[0];
					src_pntr=(int*)&data_addr;
					*dest_pntr++=*(int*)(*src_pntr++);
					*dest_pntr++=*(int*)(*src_pntr++);
					*dest_pntr++=*(int*)(*src_pntr++);
					*dest_pntr++=*(int*)(*src_pntr++);
					*dest_pntr++=*(int*)(*src_pntr++);
					*dest_pntr++=*(int*)(*src_pntr++);
					*dest_pntr++=*(int*)(*src_pntr++);
					*dest_pntr++=*(int*)(*src_pntr++);

/********************************************************************/
/*		Save new data in NVRAM										*/
/********************************************************************/
					write_nvram((unsigned far*)&diag_data[0],(unsigned far*)&saved_data.data_record.data[index<<1][0],DATA_CHANNELS);
				
/********************************************************************/
/*		Generate analog display markers								*/
/********************************************************************/
					src_pntr=(int*)&trigger_marker;
					dest_pntr=(int*)&playback;
					if (index < PRE_TRIGGER_MIN)
					{							// start marker (full scale negative)
						*(dest_pntr++)=-(*(src_pntr++));
						*(dest_pntr++)=-(*(src_pntr++));
						*(dest_pntr++)=-(*(src_pntr++));
						*(dest_pntr++)=-(*(src_pntr++));
						*(dest_pntr++)=-(*(src_pntr++));
						*(dest_pntr++)=-(*(src_pntr++));
						*(dest_pntr++)=-(*(src_pntr++));
						*(dest_pntr++)=-(*(src_pntr++));
					}
					else if (index==new_setup.pre_trigger)
					{							// trigger marker (full scale positive)
						*(dest_pntr++)=*(src_pntr++);
						*(dest_pntr++)=*(src_pntr++);
						*(dest_pntr++)=*(src_pntr++);
						*(dest_pntr++)=*(src_pntr++);
						*(dest_pntr++)=*(src_pntr++);
						*(dest_pntr++)=*(src_pntr++);
						*(dest_pntr++)=*(src_pntr++);
						*(dest_pntr++)=*(src_pntr++);
					}
				}
			}
		}
		
	}

/********************************************************************/
/********************************************************************/
/*	Setup functions	(called by slow_param_update)					*/
/********************************************************************/

	void setup_data_channel(int n)
	{
		register unsigned temp;
	
/********************************************************************/
/*	Setup pointers, attributes, and scale factors for one channel	*/
/********************************************************************/
		if ((trig_state < POST_TRIGGER)&&(n < DATA_CHANNELS))
		{
			if((temp=parameters.channel[n])>=PARAM_MAX)
				temp=0;
			data_attrib[n]=param_table[temp].attrib;
			new_setup.channel[n]=temp;
			temp=(unsigned)param_table[temp].addr;
			if (VALID_DATA_ADDRESS(temp))
				data_addr[n]=(int*)temp;
			else
				data_addr[n]=&dummy_variable;
		}
	}

/********************************************************************/
/*	Trigger Setup functions											*/
/********************************************************************/
	void setup_trigger_channel(void)
	{
		if (new_setup.trig_channel!=parameters.trigger_channel)
		{
			if (parameters.trigger_channel < 1 || parameters.trigger_channel > DATA_CHANNELS)
				parameters.trigger_channel=1;
			new_setup.trig_channel=parameters.trigger_channel;
			trig_addr=(int*)data_addr[new_setup.trig_channel-1];
			trig_attrib=data_attrib[new_setup.trig_channel-1];
			trig_state=RESTART;
		}
	}

	void setup_trigger_condition(void)
	{
		if (new_setup.condition!=parameters.trigger_condition)
		{
			if (((trig_attrib>>TYPE_SHIFT)!=0x0F) && (parameters.trigger_condition>3))
				parameters.trigger_condition=0;
			new_setup.condition=parameters.trigger_condition;
			trig_state=RESTART;
		}
	}

	void setup_trigger_level(void)
	{
		if (parameters.trigger_level!=old_trigger_level)
		{
			old_trigger_level=parameters.trigger_level;
			if ((trig_attrib>>TYPE_SHIFT)==0x1D)
				new_setup.level=parameters.trigger_level*PERCENT_TO_PU;
			else
				new_setup.level=parameters.trigger_level;
			trig_state=RESTART;
		}
	}

	void setup_trigger_delay(void)
	{
		register int temp;
		temp=parameters.trigger_delay*(DIAG_SIZE/100);
		LIMIT_MAX_MIN(temp,(DIAG_SIZE-1),PRE_TRIGGER_MIN)
		if (temp!=new_setup.pre_trigger)
		{
			new_setup.pre_trigger=temp;
			post_trigger=DIAG_SIZE-new_setup.pre_trigger;
			trig_state=RESTART;
		}
	}

	void setup_trigger_interval(void)
	{
		register int temp;
		if ((temp=parameters.sample_interval>>1)!=sample_divider)
		{
			LIMIT_MAX_MIN(temp,32767,1)
			sample_divider=temp;
			parameters.sample_interval=new_setup.interval=temp<<1;
			trig_state=RESTART;
		}
	}

/********************************************************************/
/*		Analog Output Setup function								*/
/********************************************************************/

	void setup_analog_out(int channel)
	{
		register unsigned temp,addr,attrib;
		
/********************************************************************/
/*	Set up pointers, shifts, and trigger markers for analog outputs	*/
/********************************************************************/
		if (0 && channel < 3)
		{
			if((temp=parameters.analog_out[channel]) >= PARAM_MAX)
				temp=0; 
			addr=(unsigned)param_table[temp].addr;
			if (VALID_DATA_ADDRESS(addr))
			{
				dac_addr[channel]=(int*)addr;
				attrib=param_table[temp].attrib;
				temp=attrib>>TYPE_SHIFT;
				if ( (temp!=0) && (temp <= DATA_CHANNELS) )
				{				// playback parameters are of type 1 to DATA_CHANNELS
					temp--;		//convert to playback channel number
					// get parameter numbers assigned to playback channels
					// this works because .channel is the first element of .data_setup
					read_nvram((unsigned far*)&saved_data.data_setup.channel,(unsigned far*)&old_channels,DATA_CHANNELS);
					attrib=(param_table[old_channels[temp]].attrib)&SHIFT_MASK;
					trigger_marker[temp]=0x7FFF>>attrib;
				}
				dac_shift[channel]=attrib&SHIFT_MASK;
			}
			else
				dac_addr[channel]=&dummy_variable;
		}
	}
