#include "2812reg.h"
#include "literals.h"
#include "modbus.h"
#include "struct.h"
//#include "hardware.h"

#define MIN_RX_MSG_LEN    4

extern struct PARAMS parameters;
extern int modbus_idle_timer;
extern void xceiverShutoffTask(void);
extern void resetEndOfPacketTimer(void);          
extern int isEndOfPacketTimerIntEnabled(void);      
extern void uartErrorMonitorTask(void);
extern int isTxInProgress(void);
extern void enableRxCharInt(void);                

#pragma DATA_SECTION(theRxBuf,".modbus_data")	// added by RT .bss is already full  
#pragma DATA_SECTION(pTheNextRxByte,".modbus_data")  
volatile unsigned char theRxBuf[THE_RX_BUF_SIZE];
// This pointer will be twiddled from interrupt-land
volatile unsigned char* pTheNextRxByte;

/********************************************************************/
/*		Service flags												*/
/********************************************************************/
#pragma DATA_SECTION(theMsgServiceFlag,".modbus_data")	// added by RT .bss is already full  
#pragma DATA_SECTION(theXceiverShutoffServiceFlag,".modbus_data")  
int theMsgServiceFlag;
int theXceiverShutoffServiceFlag;

/*
This is the ISR that services the timer that measures the dead time
which delimits Modbus message packets.
*/
#pragma CODE_SECTION(modbusPacketTimerIsr,".h0_code")
interrupt void modbusPacketTimerIsr(void)
{
	/*
	Disable and acknowledge the end-of-packet timer interrupt whether the
	packet is valid or not. If this is a good packet, we won't need to
	detect the end of a packet again until the existing packet is serviced. 
	If the packet is bad, we won't need to detect the end of a packet again 
	until we receive the first character of the next packet.
	*/
	// Don't write a 1 to TIF because that inadvertently clears the interrupt!
	PF0.TIMER2.TIMERTCR &= (~TIMERTCR_TIE & ~(unsigned)TIMERTCR_TIF);

	/*
	If there is a packet already sitting in the RX buffer waiting for  service,
	or the buffer is not empty, do nothing. Note: neither of these things should
	be possible since this interrupt is enabled when the first character of a
	packet is received, and disabled when an end-of-packet is detected, but an
	extra idiot-check can't hurt.
	*/
	if(!theMsgServiceFlag && (pTheNextRxByte > theRxBuf))
	{
		// If the packet is the minimum length for a valid packet (i.e. ignore
		// a single byte of noise)
		if((unsigned int)(pTheNextRxByte - theRxBuf) > MIN_RX_MSG_LEN)
		{
			/* 
			This might be a valid packet. Disable the RX char interrupt because
			we can't start receiving the next packet until this one has been serviced.
			*/
			// Disable the interrupt in the peripheral. Don't
			// bother disabling it in the PIE controller
			PF2.SCICTL2B &= ~SCICTL2_RX_BK_INT_ENA;

			// Notify the main task that a packet needs to be serviced.
			theMsgServiceFlag = 1;
		}
		else
		{
			// This is too short to be valid. Ignore it.
			pTheNextRxByte = theRxBuf;
		}
	}

	// Do this last because we don't want this ISR being re-entered.
	// Timer 2 has its own interrupt flag
	PF0.TIMER2.TIMERTCR |= (unsigned)TIMERTCR_TIF;

	// Timer 2 doesn't go through the PIE, it goes straight into the
	// core on interrupt 14. Bit 0 in the IFR is for INT1, so INT14
	// uses bit 13.
	asm("	AND IFR,#0dfffh");
}


/*
This is the ISR that services incoming characters.
*/
#pragma CODE_SECTION(uartRxCharIsr,".h0_code")
interrupt void uartRxCharIsr(void)
{
  	resetEndOfPacketTimer();

	/*
	Ignore the character if a packet is in the RX buffer, waiting 
	to be serviced. This should not be possible since the RX char 
	interrupt is turned off while a packet is waiting for service, 
	but it can't hurt.
	*/
	if(!theMsgServiceFlag)
	{
		/*
		If the end-of-packet timer interrupt has been disabled (i.e. 
		this is the first byte of a new packet), enable the interrupt 
		so we can detect the end of the packet.
		*/
		if(!isEndOfPacketTimerIntEnabled())
			PF0.TIMER2.TIMERTCR |= TIMERTCR_TIE;		// enable timer interrupt

		if(pTheNextRxByte >= (theRxBuf + THE_RX_BUF_SIZE))
		{
		  // RX overflow!!!
		  pTheNextRxByte = theRxBuf;
		}
		else
		{
		  // Buffer the next byte
		  *pTheNextRxByte++ = PF2.SCIRXBUFB;
		}
	}

	// Do this last because we don't want this ISR being re-entered.
	// UART B interrupts are in group 9, which is bit 8
	PF0.PIEACK = (1 << 8); 
}


/*
Call this once at boot-time.
*/
#pragma CODE_SECTION(receiveInit,".h0_code")
void receiveInit(void)
{
  pTheNextRxByte = &theRxBuf[0];
}

/*
Searches the handler function table for a handler for the given command.
Calls the handler if found, returns an error if not found.
*/
#pragma CODE_SECTION(serviceGoodMessage,".h0_code")
static int serviceGoodMessage(unsigned char* pMsg, unsigned msgLen)
{
  if(isTxInProgress())
    return -HMSG_TX_BUSY; // Should never happen

	switch(pMsg[1])
	{
	case 0:	 return handleEchoCmd((unsigned char*)pMsg, msgLen);
	case 3:  return handleReadConsecutive((unsigned char*)pMsg, msgLen);
	case 4:  return handleReadConsecutive((unsigned char*)pMsg, msgLen);
	case 16: return handleWriteConsecutive((unsigned char*)pMsg, msgLen);

	default: return handleUnrecognizedCmd((unsigned char*)pMsg, msgLen);
	}
}



/*
Call this when there's a message sitting in the RX buf that
needs to be handled.
*/
#pragma CODE_SECTION(handleMessage,".h0_code")
int handleMessage(void)
{
  int ret = HMSG_OK;
  unsigned msgLen;
  unsigned msgCrc;
  unsigned calculatedCrc;

  // TODO: why is 255 being used as the broadcast address?!
  if((theRxBuf[0] != parameters.modbus_slave_id) && (theRxBuf[0] != 0xff) && (theRxBuf[0] != 0)) 
  {
    ret = -HMSG_NOT_FOR_ME;
  }
  else
  {
    // Address is good. Run the CRC
    msgLen = (unsigned)(pTheNextRxByte - theRxBuf);
    msgCrc = (unsigned)(theRxBuf[msgLen - 1]);
    msgCrc |= ((unsigned)(theRxBuf[msgLen - 2]) << 8);
    calculatedCrc = calc_modbus_crc((unsigned*)theRxBuf, msgLen - 2);
    if(msgCrc != calculatedCrc)
    {
      ret = -HMSG_BAD_CRC;
    }
    else
    {
		modbus_idle_timer=0;
      // Message is for me and it has a good CRC. Strip the
      // address and CRC, and pass it along for service.
      ret = serviceGoodMessage((unsigned char*)theRxBuf, msgLen - 2); // Strip the CRC
    }
  }

  // Reset the RX buf ptr amd msg service flag
  pTheNextRxByte = theRxBuf;
  theMsgServiceFlag = 0;

  /*
  Re-enable the RX char interrupt. When the first character of
  the next packet is received, the end-of-packet timer will be
  turned back on.
  */
  enableRxCharInt();

  return ret;
}

#pragma CODE_SECTION(modbus_background,".rom_code")
void modbus_background(void)

{
    if(theMsgServiceFlag)
    {
      handleMessage();
    }
	if(theXceiverShutoffServiceFlag)
	{
		xceiverShutoffTask();
	}
	uartErrorMonitorTask();
}
