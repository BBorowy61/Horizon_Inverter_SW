#include "literals.h"
#include "2812reg.h"
#include "io_def.h"
#include "modbus.h"
#include "struct.h"
//#include "hardware.h"

extern far struct PARAM_ENTRY param_table[PARAM_MAX];
extern volatile unsigned char theTxBuf[THE_TX_BUF_SIZE];
extern int txMsg(unsigned char* pMsg, unsigned len);
extern int modbus_access_level;

/*
Catchall for unrecognized commands
*/
#pragma CODE_SECTION(handleUnrecognizedCmd,".h0_code")
int handleUnrecognizedCmd(unsigned char* pMsg, unsigned msgLen)
{
	// Echo the address and command but flip the error bit
	theTxBuf[0] = pMsg[0];
	theTxBuf[1] = pMsg[1] | 0x80;
	txMsg(0, 2);
	return -HMSG_INVALID_COMMAND;
}


/*
Echoes the payload back to the caller. Useful as a ping.
*/
#pragma CODE_SECTION(handleEchoCmd,".h0_code")
int handleEchoCmd(unsigned char* pMsg, unsigned msgLen)
{
  // Echo the payload, whatever it is
  if(txMsg(pMsg, msgLen) == TX_OK)
    return HMSG_OK;
  else
    return -HMSG_TX_FAILED;
}


/********************************************************************/
/*		Read data from consecutive parameters ( function 3 or 4)	*/
/********************************************************************/
#pragma CODE_SECTION(handleReadConsecutive,".h0_code")
int handleReadConsecutive(unsigned char* pMsg, unsigned msgLen)
{
	unsigned i,param_number, last_data_byte;
	register int temp;

	// Echo address and command bytes
	theTxBuf[0] = pMsg[0];
	theTxBuf[1] = pMsg[1];

	// Get starting parameter number
	param_number=(pMsg[START_ADDR_HIGH]<<8) | pMsg[START_ADDR_LOW];
	i=(pMsg[NUMBER_HIGH]<<8) | pMsg[NUMBER_LOW];
	theTxBuf[READ_BYTE_COUNT]=i<<1;		// number of bytes
	last_data_byte=theTxBuf[READ_BYTE_COUNT]+2;
	for(i=READ_DATA;i<=last_data_byte;i+=2)
	{
		if (param_number < PARAM_MAX)
		{					// get address of parameter from table
			if (((param_table[param_number].attrib>>RD_ACCESS_SHIFT)&ACCESS_MASK) <= modbus_access_level) {
				temp= *((int *)param_table[param_number].addr);
			}
			else {
				temp=0;						// set to zero if no access
			}
			theTxBuf[i]  = 0xff & (temp>>8);		// split into high byte
			theTxBuf[i+1]= 0xff & (temp>>0);		// and low byte
		}
		else
		{
			theTxBuf[FUNCTION_CODE] |= 0x80;	// exception response
			theTxBuf[READ_DATA]=0;
			theTxBuf[READ_DATA+1]=2;// illegal address code
			last_data_byte=READ_DATA+1;	// for response
		}
		param_number++;					// next consecutive parameter
	}									// end for loop

	if(txMsg(0, last_data_byte + 1) == TX_OK)
		return HMSG_OK;
	else
		return -HMSG_TX_FAILED;
}

/********************************************************************/
/*		Write data to consecutive parameters (function 16)			*/
/********************************************************************/
#pragma CODE_SECTION(handleWriteConsecutive,".h0_code")
int handleWriteConsecutive(unsigned char* pMsg, unsigned msgLen)
{
	unsigned i,param_number, last_data_byte;
	register int temp;

	// Echo the address, command, starting reg #, and reg count
	memcpy((void*)(&theTxBuf[0]), (void*)pMsg, 6);

	// Get starting parameter number
	param_number=(pMsg[START_ADDR_HIGH]<<8) | pMsg[START_ADDR_LOW];
//last_data_byte=pMsg[msgLen-1];
	last_data_byte = msgLen-1;

	for(i=WRITE_DATA;i<=last_data_byte;i+=2) {

		if ((param_number >= PARAM_MIN)&&(param_number < PARAM_MAX)&&((RD_ONLY & param_table[param_number].attrib)==0))
		{	// if writable parameter
			// assemble word from next 2 data bytes
			temp=(pMsg[i]<<8) | pMsg[i+1];
			if ((temp >= param_table[param_number].min)&&
				(temp <= param_table[param_number].max)&&
				(((param_table[param_number].attrib>>WR_ACCESS_SHIFT)&ACCESS_MASK)<=modbus_access_level))
			{
				*((int *)param_table[param_number].addr)=temp;		// write new value
			}
			else
			{							// new value out of range
				theTxBuf[FUNCTION_CODE]|=0x80;	// exception response
				theTxBuf[NUMBER_HIGH]=0;
				theTxBuf[NUMBER_LOW]=3;	// illegal data code
			}
			param_number++;				// next consecutive parameter
		}
		else
		{								// not writable parameter
			theTxBuf[FUNCTION_CODE]|=0x80;	// exception response
			theTxBuf[NUMBER_HIGH]=0;
			theTxBuf[NUMBER_LOW]=2;	// illegal address code
		}
	} 											// end for loop

	last_data_byte=NUMBER_LOW;					// for response
    if(txMsg(0, 6) == TX_OK)
      return HMSG_OK;
    else
      return -HMSG_TX_FAILED;
}


