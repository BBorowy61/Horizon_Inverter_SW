/********************************************************************/
/********************************************************************/
/*																	*/
/*						Fault Queue Module							*/
/*																	*/
/********************************************************************/
/********************************************************************/

#include "literals.h"
#include "struct.h"
#include "faults.h"

/********************************************************************/
/*		Global variables											*/
/********************************************************************/
#pragma DATA_SECTION(fault_queue,".que_data")
#pragma DATA_SECTION(display_fault,".que_data")
#pragma DATA_SECTION(display_fault_text,".que_data")
struct FAULT_QUEUE fault_queue;
struct QUEUE_ENTRY display_fault;
unsigned display_fault_text[8];

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern int fault_queue_index,number_faults;
extern int faults[FAULT_WORDS];
extern int global_flags;
extern struct RTC_PACKED bcd_time;
extern struct SAVED_VARS saved_variables;
extern struct PARAMS parameters;

#pragma CODE_SECTION(manage_queue,".h0_code")

/********************************************************************/
/********************************************************************/
/*	Fault queue function (called by pwm_isr every 1 ms)			*/
/********************************************************************/
void manage_queue (void)
{
	register unsigned int temp,new_faults,word_count,bit_count,*temp_pntr;

/********************************************************************/
/*	Clear fault queue												*/
/********************************************************************/
	if ((parameters.initialize_cmd==1) || (fault_queue.index >= FAULT_QUEUE_SIZE))
	{
		temp_pntr=&fault_queue.entry[0].number;
		for (temp=FAULT_QUEUE_SIZE;temp>0;temp--)
		{
			(*temp_pntr++)=0;
			(*temp_pntr++)=0;
			(*temp_pntr++)=0;
			(*temp_pntr++)=0;
		}
	fault_queue.index=parameters.initialize_cmd=0;
	global_flags|=SAVE_QUEUE;
	}

/********************************************************************/
/*	Count number of faults											*/
/********************************************************************/
	temp=0;
	for (word_count=0; word_count<FAULT_WORDS; word_count++)
	{
		if (new_faults=faults[word_count])
		{
			for (bit_count=1; bit_count<17; bit_count++)
			{
				temp+=(new_faults & 0x01);
				new_faults=new_faults>>1;
			}
		}

/********************************************************************/
/*	Enter new faults												*/
/********************************************************************/
		new_faults=faults[word_count]&~saved_variables.old_faults[word_count];
		saved_variables.old_faults[word_count]=faults[word_count];
		if (new_faults!=0)
		{
			global_flags|=SAVE_QUEUE;
			for (bit_count=1; bit_count<17; bit_count++)
			{
				if (new_faults & 0x01)
				{
					if (++fault_queue.index >= FAULT_QUEUE_SIZE)
						fault_queue.index=0;
					temp_pntr=&fault_queue.entry[fault_queue.index].number;
					*temp_pntr++=(word_count<<4)+bit_count;
					*temp_pntr++=bcd_time.second_hundredth;
					*temp_pntr++=bcd_time.hour_minute;
					*temp_pntr++=bcd_time.month_day;
				}
				new_faults=new_faults>>1;
			}
		}
	}
	number_faults=temp;

/********************************************************************/
/*	Get fault queue entry for display								*/
/********************************************************************/
	if ((temp=fault_queue_index) >= FAULT_QUEUE_SIZE)
		fault_queue_index=temp=0;
	if ((temp=fault_queue.index-temp) & 0x8000)
		temp+=FAULT_QUEUE_SIZE;
	temp_pntr=&fault_queue.entry[temp].number;
	display_fault.number=fault_queue_index*100+(temp=*temp_pntr++);
	CONVERT_TIME(display_fault.time[0],(*temp_pntr++))
	CONVERT_TIME(display_fault.time[1],(*temp_pntr++))
	CONVERT_TIME(display_fault.time[2],(*temp_pntr++))
}
