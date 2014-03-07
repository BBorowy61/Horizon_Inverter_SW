/********************************************************************/
/********************************************************************/
/*																	*/
/*						Real Time Clock Module						*/
/*																	*/
/*			crystal accuracy is 50 ppm or 4.3 sec/day				*/
/********************************************************************/
/********************************************************************/

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
1 MW version
2009-02-24	delete slow_task_status
			call at 10 ms instead of 1 ms intervals
2009-03-04	change from hardware calibration to software calibration
			change divide by 100 to multiply
*/
#include "literals.h"
#include "2812reg.h"
#include "io_def.h"
#include "struct.h"

#define	CALIBRATION_DONE	0x02

/********************************************************************/
/*		Global variables											*/
/********************************************************************/
struct RTC_PACKED bcd_time,display_time;
int	set_clock;

/********************************************************************/
/*		External  variables											*/
/********************************************************************/
extern struct PARAMS parameters;
extern int status_flags,fpga_outputs,timer_1ms;

const far unsigned int bcd_table[100]={
0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,
0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,
0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,
0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,
0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,
0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,
0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99};

#pragma CODE_SECTION(real_time_clock,".l1_code")
#pragma DATA_SECTION(bcd_table,".rom_const")

/********************************************************************/
/********************************************************************/
/*			Real Time Clock Function (called every 1 ms)			*/
/********************************************************************/

void real_time_clock(void)
{
	register unsigned temp,high_part;

	if (set_clock&1)
	{							// set hardware RTC to display time

/********************************************************************/
/*	Convert display time from binary to BCD							*/
/********************************************************************/
		if ((high_part=MULQ(16,display_time.second_hundredth,(65536U/100))) > 59)
			high_part=59;								// second
		if ((temp=display_time.second_hundredth-(high_part*100)) > 99)
			temp=99;									// hundredth
		bcd_time.second_hundredth=(bcd_table[high_part]<<8)+bcd_table[temp];

		if ((high_part=MULQ(16,display_time.hour_minute,(65536U/100))) > 23)
			high_part=23;								// hour
		if ((temp=display_time.hour_minute-(high_part*100)) > 59)
			temp=59;									// minute
		bcd_time.hour_minute=(bcd_table[high_part]<<8)+bcd_table[temp];

		if ((high_part=MULQ(16,display_time.month_day,(65536U/100))) > 12)
			high_part=12;								// month
		if ((temp=display_time.month_day-(high_part*100)) > 31)
			temp=31;									// day
		bcd_time.month_day=(bcd_table[high_part]<<8)+bcd_table[temp];

		bcd_time.year=bcd_table[(display_time.year-2000)&0x3F]+0x2000;

/********************************************************************/
/*	Load hardware RTC												*/
/********************************************************************/
		RTC_FLAGS		=0x02;				// stop hardware RTC
		RTC_SECOND		=bcd_time.second_hundredth>>8;;
		RTC_MINUTE		=bcd_time.hour_minute;
		RTC_HOUR		=bcd_time.hour_minute>>8;
		RTC_DATE		=bcd_time.month_day;
		RTC_MONTH		=bcd_time.month_day>>8;
		RTC_YEAR		=bcd_time.year;
		RTC_CENTURY		=bcd_time.year>>8;
		RTC_FLAGS		=0;					//start hardware RTC
	}										// end set_clock

/********************************************************************/
/*	Synchronize software clock with hardware RTC					*/
/********************************************************************/
	else
	{								// clock running
		RTC_FLAGS=0x01;				// freeze hardware RTC
		temp=RTC_SECOND<<8;			// read seconds
		if (temp!=(bcd_time.second_hundredth&0xFF00))
		{							// seconds has rolled over
			bcd_time.second_hundredth	=temp;
			bcd_time.hour_minute		=(RTC_HOUR<<8)|(RTC_MINUTE&0xFF);
			bcd_time.month_day			=(RTC_MONTH<<8)|(RTC_DATE&0xFF);
			bcd_time.year				=(RTC_CENTURY<<8)|RTC_YEAR&0xFF;

/********************************************************************/
/*	Apply calibration correction once per day at 12:00 am			*/
/********************************************************************/
			if(bcd_time.hour_minute==0)
			{											// midnight
				if ((set_clock & CALIBRATION_DONE)==0)
				{
					if (parameters.calibrate_clock < 0)
					{										// negative correction
						if ((bcd_time.second_hundredth>>8)==-(parameters.calibrate_clock))
						{
							bcd_time.second_hundredth=0;
							RTC_FLAGS		=0x02;			// stop hardware RTC
							RTC_SECOND		=0;				// set seconds to zero
							RTC_FLAGS		=0;				// start hardware RTC
							set_clock|=CALIBRATION_DONE;
						}
					}
					else if (bcd_time.second_hundredth==0)
					{										// positive correction
						bcd_time.second_hundredth=parameters.calibrate_clock<<8;
						RTC_FLAGS		=0x02;				// stop hardware RTC
						RTC_SECOND		=bcd_time.second_hundredth>>8;;
						RTC_FLAGS		=0;					// start hardware RTC
						set_clock|=CALIBRATION_DONE;
					}
				}
			}
			else
				set_clock&=~CALIBRATION_DONE;

/********************************************************************/
/*	Convert time from BCD to packed decimal for display				*/
/********************************************************************/
			CONVERT_TIME(display_time.second_hundredth,bcd_time.second_hundredth)
			CONVERT_TIME(display_time.hour_minute,bcd_time.hour_minute)
			CONVERT_TIME(display_time.month_day,bcd_time.month_day)
			CONVERT_TIME(display_time.year,bcd_time.year)
		}							// end seconds has rolled over

/********************************************************************/
/*	Increment sub-second counters									*/
/********************************************************************/
		else
		{
			temp=(bcd_time.second_hundredth & 0xFF)+1;
			if ((temp & 0x0F) > 9)
			{
				temp=(temp & 0xF0)+0x10;
				if (temp > 0x90)
					temp=0;
			}
			bcd_time.second_hundredth=(bcd_time.second_hundredth&0xFF00)+temp;
		}
		RTC_FLAGS=0;			// unfreeze hardware RTC
	}
}

