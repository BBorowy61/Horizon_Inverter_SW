/********************************************************************/
/********************************************************************/
/*																	*/
/*					Serial Communications Module					*/
/*																	*/
/********************************************************************/
/********************************************************************/

/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
2008-04-22	modify to use SCI FIFO
2008-05-28	correct error in read exception response
2008-06-05	correct another error in exception responses
1 MW version
2008-11-18	add slave id parameter
2009-01-08	add parameter change fault
			add exception response macros
			put local variables in a structure
2009-02-11	delete test point output
2009-10-05	correct error in checking access level
2009-10-06	delete checking of access level
*/

#include "2812reg.h"
#include "literals.h"
#include "io_def.h"
#include "struct.h"
#include "faults.h"

#define COMM_DEFAULT_ADDR	0x8000	// default address of comm_data
#define WAIT_TIME			1000
#define SCIRST				0x8000

/********************************************************************/
/*		Global variables											*/
/********************************************************************/
#pragma DATA_SECTION(comm_a,".comm_data")		
struct PCS_OP comm_a;

/********************************************************************/
/*		Local variables												*/
/********************************************************************/
#pragma DATA_SECTION(sci_data_a,".scia_data")
struct SCI_DATA sci_data_a;

/********************************************************************/
/*		External variables											*/
/********************************************************************/
extern int faults[FAULT_WORDS],status_flags,pwm_int_us;
extern struct PARAMS parameters;
extern far struct PARAM_ENTRY param_table[PARAM_MAX+1];
extern struct SAVED_VARS saved_variables;


void sci_respond(void);

#pragma CODE_SECTION(calc_crc,".h0_code")
#pragma CODE_SECTION(sci_a,".h0_code")
#pragma CODE_SECTION(sci_respond,".h0_code")


/********************************************************************/
/********************************************************************/
/*		Calculate CRC function 										*/
/********************************************************************/
               
unsigned int calc_crc(register unsigned far *addr,register unsigned index_max)
{
	register unsigned i,crcsum; 
    crcsum=0xFFFF;
    for(i=0;i<=index_max;i++)
    {
    	crcsum^=*addr++;

    	if((crcsum&1)==1)
    		crcsum=(crcsum>>1) ^ 0xA001;
    	else
    		crcsum=(crcsum>>1);
    	if((crcsum&1)==1)
    		crcsum=(crcsum>>1) ^ 0xA001;
    	else
    		crcsum=(crcsum>>1);
    	if((crcsum&1)==1)
    		crcsum=(crcsum>>1) ^ 0xA001;
    	else
    		crcsum=(crcsum>>1);
    	if((crcsum&1)==1)
    		crcsum=(crcsum>>1) ^ 0xA001;
    	else
    		crcsum=(crcsum>>1);
    	if((crcsum&1)==1)
    		crcsum=(crcsum>>1) ^ 0xA001;
    	else
    		crcsum=(crcsum>>1);
    	if((crcsum&1)==1)
    		crcsum=(crcsum>>1) ^ 0xA001;
    	else
    		crcsum=(crcsum>>1);
    	if((crcsum&1)==1)
    		crcsum=(crcsum>>1) ^ 0xA001;
    	else
    		crcsum=(crcsum>>1);
    	if((crcsum&1)==1)
    		crcsum=(crcsum>>1) ^ 0xA001;
    	else
    		crcsum=(crcsum>>1);
    }
	return crcsum;
}

/********************************************************************/
/********************************************************************/
/*		SCI function	(called by pwm_isr)							*/
/********************************************************************/
void sci_a(void)
{
	if (sci_data_a.tx_state==0)
	{
        // calculate the time we have been waiting, the counter increments
        // every 2 pwm-ints, which are each pwm_int_us long, so we can easily
        // calculate the number of us since the last character, the baudrate
        // stored in the parameter table is scaled by 100, so the time for one 
        // character can be calculated as 10 (8 data + start + stop) bits 
        // divided by the baud rate (19200), multiplied by 1000 to convert to ms
        // thus the time for 3.5 characters is 35000/19200 ms = 1.823ms or 1823 us.
        // 
        // This means that the timeout has expited when 
        // wait-timer * 2 * pwm_int_us  > 1823
        // or
        // wait-timer * pwm_int_us  > 911

        if (sci_data_a.function_code!=0) {

            if((sci_data_a.wait_timer * pwm_int_us) > 911 )
                sci_data_a.tx_state=REQUEST_RESP;
        }
	}
	else if (sci_data_a.tx_state==REQUEST_RESP)
		return;
		            		
/********************************************************************/
/*		Transmit next character										*/
/********************************************************************/
	else if (sci_data_a.tx_state==SEND_RESP)
	{
        if(sci_data_a.index < (sci_data_a.last_data_byte+2))
		{
			if ((PF2.SCIFFTX & 0x1000)==0)
        		PF2.SCITXBUF=sci_data_a.buffer[++sci_data_a.index];
		}
        else
        	sci_data_a.rx_state=sci_data_a.tx_state=0;
	}

	if(++sci_data_a.wait_timer>WAIT_TIME)
	{
		PF2.SCIFFTX&=~SCIRST;
		sci_data_a.wait_timer=0;
		sci_data_a.rx_state=sci_data_a.tx_state=0;
	}
	
/********************************************************************/
/*		Reset SCI if receive error									*/
/********************************************************************/
	if (PF2.SCIRXST & 0x80)
		PF2.SCIFFTX&=~SCIRST;
	else
		PF2.SCIFFTX|=SCIRST;
	 
/********************************************************************/
/*		Get new data from SCI Receive Buffer						*/
/********************************************************************/
    if ((PF2.SCIFFRX & 0x1F00)!=0)
	{
		sci_data_a.indata=PF2.SCIRXBUF; 
		sci_data_a.wait_timer=0; 

/********************************************************************/
/*		Process received data and copy to buffer					*/
/********************************************************************/
	    switch(sci_data_a.rx_state)
	    {

/********************************************************************/
/*		Wait for valid slave address								*/
/********************************************************************/
	    	case(NO_COMM):
	    		if (sci_data_a.indata==parameters.sci_slave_id || sci_data_a.indata==255)
	    		{
	    			sci_data_a.index=0;
				    sci_data_a.buffer[sci_data_a.index]=sci_data_a.indata;
		    		sci_data_a.rx_state=GET_FUN;
		    	}
		    	else
		    		sci_data_a.rx_state=WAIT_COMM;
	    		break;

/********************************************************************/
/*		Get function code											*/
/********************************************************************/
	    	case(GET_FUN):
	    		if ((sci_data_a.indata==3)||(sci_data_a.indata==4)||(sci_data_a.indata==16))
	    		{
	    			sci_data_a.buffer[++sci_data_a.index]=sci_data_a.indata;
	    			sci_data_a.rx_state=GET_ADD_NUM;
	    		}
	            else
	            	sci_data_a.rx_state=WAIT_COMM; 
	    		break;

/********************************************************************/
/*		Get starting parameter and number of parameters				*/
/********************************************************************/
	    	case(GET_ADD_NUM):
	    		sci_data_a.buffer[++sci_data_a.index]=sci_data_a.indata;
	    		if(sci_data_a.index>=NUMBER_LOW) 
	    			if(sci_data_a.buffer[FUNCTION_CODE]==16)
	    				sci_data_a.rx_state=GET_BYTE_CNT;
	    			else
	    			{
	    				sci_data_a.last_data_byte=NUMBER_LOW;
	    				sci_data_a.rx_state=GET_CHKLO;
	    			}
	    		break;

/********************************************************************/
/*		Get number of bytes for write function only					*/
/********************************************************************/
	    	case(GET_BYTE_CNT):
	    		if(sci_data_a.indata > MAXLEN-6)
	    			sci_data_a.rx_state=WAIT_COMM;
	    		else
	    		{
	    			sci_data_a.buffer[++sci_data_a.index]=sci_data_a.indata;
	    			sci_data_a.rx_state=GET_DATA;
	    		}
	    		break;

/********************************************************************/
/*		Get data for write function only							*/
/********************************************************************/
	    	case(GET_DATA):
	    		sci_data_a.buffer[++sci_data_a.index]=sci_data_a.indata;
	    		if(sci_data_a.index>=2*sci_data_a.buffer[NUMBER_LOW]+6)
	    		{
	    			sci_data_a.last_data_byte=sci_data_a.index;	
	    			sci_data_a.rx_state=GET_CHKLO;
	    		} 
	    		break;

/********************************************************************/
/*		Get low byte of checksum									*/
/********************************************************************/
	    	case(GET_CHKLO):           
	    		sci_data_a.buffer[++sci_data_a.index]=sci_data_a.indata;
	    		sci_data_a.rx_state=GET_CHKHI;
	    		break;

/********************************************************************/
/*		Get high byte of checksum									*/
/********************************************************************/
	    	case(GET_CHKHI):
	    		sci_data_a.buffer[++sci_data_a.index]=sci_data_a.indata;
	    		sci_data_a.buffer[sci_data_a.index+1]=23432;
	    		sci_data_a.function_code=sci_data_a.buffer[FUNCTION_CODE];
	    		sci_data_a.rx_state=WAIT_COMM;
	    		break;

/********************************************************************/
/*		Wait for timeout											*/
/********************************************************************/
			case(WAIT_COMM):
				break;
	    }
	}
}	// end sci_a function

/********************************************************************/
/********************************************************************/
/*		SCI Respond function	(called by pwm_isr every 1 ms)	*/
/********************************************************************/

void sci_respond(void)
{
	 register unsigned i,number_words,param_number;
	 register int new_value,*int_pntr;

	if (sci_data_a.tx_state!=REQUEST_RESP)
		return;

	if( ( !((sci_data_a.function_code==3) || (sci_data_a.function_code==4) || (sci_data_a.function_code==16)) ) || 
	( ((sci_data_a.buffer[sci_data_a.last_data_byte+2]<<8)+sci_data_a.buffer[sci_data_a.last_data_byte+1]) != calc_crc((unsigned far*)&sci_data_a.buffer,sci_data_a.last_data_byte)) )
	{
		sci_data_a.function_code=sci_data_a.rx_state=sci_data_a.tx_state=0;
		saved_variables.scia_error_count++;
		return;
	}

/********************************************************************/
/*		Assemble starting address from 2 bytes in .buffer			*/
/*		Three possible interpretations depending on value:			*/
/*		1. parameter number if < PARAM_MAX							*/
/*		2. comm_a structure address if > COMM_DEFAULT_ADDR			*/
/*		3. otherwise memory address 								*/
/********************************************************************/
	sci_data_a.tx_state=PREPARE_RESP;
	param_number=(sci_data_a.buffer[START_ADDR_HIGH]<<8) | sci_data_a.buffer[START_ADDR_LOW]; 

/********************************************************************/
/*	Determine number of parameters to be read						*/
/********************************************************************/
	number_words=(sci_data_a.buffer[NUMBER_HIGH]<<8) | sci_data_a.buffer[NUMBER_LOW];

/********************************************************************/
/*	If starting address not in PCSOp structure use Modbus protocol	*/
/********************************************************************/
	if (param_number < COMM_DEFAULT_ADDR)
	{

/********************************************************************/
/*	Modbus write function - write data to consecutive parameters	*/
/*	writes to parameters that could cause a parameter change fault	*/
/*	are allowed only when not running								*/
/********************************************************************/
		if (sci_data_a.function_code==16)
		{
			for(i=WRITE_DATA;i<=sci_data_a.last_data_byte;i+=2)
			{
				if ((param_number >= PARAM_MIN)&&(param_number < PARAM_MAX)) {

					new_value=(sci_data_a.buffer[i]<<8) | sci_data_a.buffer[i+1];
					int_pntr=(int*)param_table[param_number].addr;

					if (*int_pntr!=new_value) {

						if(	((new_value >= param_table[param_number].min) &&
							(new_value <= param_table[param_number].max)) &&
							((RD_ONLY & param_table[param_number].attrib)==0)
						) {

							/* ok to change at anytime */
							if(!(NOWR_RUN & param_table[param_number].attrib)) {
								*int_pntr=new_value;
							}

							/* can only change when not running */
							else if(!(status_flags & STATUS_RUN)) {
								*int_pntr=new_value;
								SET_FAULT(PARAM_CHANGE_FAULT)
							}			

							/* otherwise, reject WR */
							else {
								WRITE_EXCEPTION(sci_data_a,INVALID_DATA)
							}
						}
						else {
							WRITE_EXCEPTION(sci_data_a,INVALID_DATA)
						}
					}
					param_number++;
				}
			}
			sci_data_a.last_data_byte=NUMBER_LOW;
		}

/********************************************************************/
/*	Modbus read function - read data from consecutive parameters	*/
/********************************************************************/
		else if ( (sci_data_a.function_code==3) || (sci_data_a.function_code==4) ) 
		{
			sci_data_a.last_data_byte=(number_words+1)<<1;
			sci_data_a.buffer[READ_BYTE_COUNT]=number_words<<1;

			for(i=READ_DATA;i<=sci_data_a.last_data_byte;i+=2)
			{
				if (param_number < PARAM_MAX)
				{
					int_pntr=(int*)param_table[param_number++].addr;
					new_value=*int_pntr++;
					sci_data_a.buffer[i]  =(new_value>>8) & 0xFF;
					sci_data_a.buffer[i+1]=new_value & 0xFF;
				}
				else
					READ_EXCEPTION(sci_data_a,INVALID_ADDRESS)
			}
		}
		else
			READ_EXCEPTION(sci_data_a,INVALID_FUNCTION)
	}

/********************************************************************/
/*	Starting address is in PCSOp data structure						*/
/********************************************************************/
	else
	{

/********************************************************************/
/*	PCSOp write function - write data to consecutive locations		*/
/*	starting address could be parameter number or address			*/
/********************************************************************/
		if (sci_data_a.function_code==16)
		{
			for(i=WRITE_DATA;i<=sci_data_a.last_data_byte;i+=2)
			{
				if ((param_number >= PARAM_MIN)&&(param_number < PARAM_MAX))
					int_pntr=(int*)param_table[param_number].addr;
				else if (param_number >= COMM_DEFAULT_ADDR)
					int_pntr=(int*)(param_number-COMM_DEFAULT_ADDR+(unsigned)&comm_a);
				new_value=(sci_data_a.buffer[i]<<8) | sci_data_a.buffer[i+1];
				if (*int_pntr!=new_value) {

							/* ok to change at anytime */
							if(!(NOWR_RUN & param_table[param_number].attrib)) {
								*int_pntr=new_value;
							}

							/* can only change when not running */
							else if(!(status_flags & STATUS_RUN)) {
								*int_pntr=new_value;
								SET_FAULT(PARAM_CHANGE_FAULT)
							}			

							/* otherwise, reject WR */
							else {
								WRITE_EXCEPTION(sci_data_a,INVALID_DATA)
							}
				}
				param_number++;
			}
			sci_data_a.last_data_byte=NUMBER_LOW;
		}
	
/********************************************************************/
/*	PCSOp read function	- read data from random locations			*/
/*	Get data specified by array of pointers in comm_a.addr and		*/
/*	put in corresponding location in comm_a.data buffer				*/
/********************************************************************/
		else if ((sci_data_a.function_code==3)||(sci_data_a.function_code==4))
		{
			sci_data_a.last_data_byte=(number_words+1)<<1;
			sci_data_a.buffer[READ_BYTE_COUNT]=number_words<<1;

/********************************************************************/
/*	comm_a.addr could contain both addresses & parameter numbers	*/
/*	values < PARAM_MAX are assumed to be parameter numbers and 		*/
/*	must be converted to addresses									*/
/*	values > COMM_DEFAULT_ADDR are assumed to be addresses in comm_a*/
/*	all other values are assumed to be addresses and not changed	*/
/********************************************************************/
			for (i=0;i<number_words;i++)
			{
				param_number=(unsigned)comm_a.addr[i];
				if (param_number!=0)
				{
					if (param_number < PARAM_MAX)
						comm_a.addr[i]=(unsigned*)param_table[param_number].addr;
					else if (param_number >= COMM_DEFAULT_ADDR)
						comm_a.addr[i]=(unsigned*)(param_number-COMM_DEFAULT_ADDR+(unsigned)&comm_a);

/********************************************************************/
/*	Get data pointed to by comm_a.addr and place in comm_a.data buffer	*/
/********************************************************************/
					if (VALID_DATA_ADDRESS((unsigned)comm_a.addr[i]))
						comm_a.data[i]=*comm_a.addr[i];
					else
						comm_a.data[i]=2222;	// illegal address code
				}
			}

/********************************************************************/
/*	comm_a.data now contains latest data values 					*/
/*	Split words into high & low bytes and place in .buffer			*/
/********************************************************************/
			int_pntr=(int*)&comm_a.data[0];
			for(i=READ_DATA;i<=sci_data_a.last_data_byte;i+=2)
			{
				new_value=*int_pntr++;
				sci_data_a.buffer[i]=(new_value>>8) & 0xFF;
				sci_data_a.buffer[i+1]=new_value & 0xFF; 
			}
		}
	}

/********************************************************************/
/*		Calculate and write checksum of response					*/
/********************************************************************/
	i=sci_data_a.last_data_byte;
	param_number=calc_crc((unsigned far*)&sci_data_a.buffer,i);
	sci_data_a.buffer[++i]=param_number & 0xFF;
	sci_data_a.buffer[++i]=(param_number>>8) & 0xFF;

/********************************************************************/
/*		Initiate transmission of response							*/
/********************************************************************/
	sci_data_a.tx_state=SEND_RESP;
	sci_data_a.index=0;	
	PF2.SCITXBUF=sci_data_a.buffer[0];
	sci_data_a.function_code=0;
}	// end of sci_respond function

