#define __TRANSMIT_C__

#include "2812reg.h"
#include "literals.h"
#include "modbus.h"

#pragma DATA_SECTION(theTxBuf,".modbus_data")	// added by RT .bss is already full  
#pragma DATA_SECTION(pTheEndOfTxMsg,".modbus_data")
#pragma DATA_SECTION(pTheNextTxByte,".modbus_data")
volatile unsigned char theTxBuf[THE_TX_BUF_SIZE];
// These two pointers will be twiddled from interrupt-land
volatile unsigned char* pTheEndOfTxMsg;
volatile unsigned char* pTheNextTxByte;

extern int isTxInProgress(void);
extern int theXceiverShutoffServiceFlag;
extern void xceiverEnableTx(void);
extern void xceiverEnableRx(void);               
extern struct P_FRAME0 PF0;
extern struct P_FRAME2 PF2;

// TODO: Why isn't it linking the standard memcpy?!
#pragma CODE_SECTION(mymemcpy,".h0_code")
void mymemcpy(void* pDst, void* pSrc, unsigned len)
{
	unsigned* ps = (unsigned*)pSrc;
	unsigned* pd = (unsigned*)pDst;
	unsigned* pe = &ps[len];

	while(ps < pe)
		*pd++ = *ps++;
}


#pragma CODE_SECTION(uartTxCharIsr,".h0_code")
interrupt void uartTxCharIsr(void)
{
	if(pTheNextTxByte <= pTheEndOfTxMsg)
	{
		PF2.SCITXBUFB = *pTheNextTxByte++;
	}
	else
	{
		// Done!
		pTheNextTxByte = 0;
		pTheEndOfTxMsg = theTxBuf;
		/*
		We can't shut of the xceiver transmitter yet because this
		UART double-buffers bytes, and there is still one byte
		waiting to be transmitted. The "empty" flag can't generate
		an interrupt, it must be polled.
		*/
		theXceiverShutoffServiceFlag = 1;
	}

	// Do this last because we don't want this ISR being re-entered.
	// Acknowledge interrupt
	// UART B interrupts are in group 9, which is bit 8
	PF0.PIEACK = (1 << 8); 
}


#pragma CODE_SECTION(xceiverShutoffTask,".h0_code")
void xceiverShutoffTask(void)
{
	if(PF2.SCICTL2B & SCICTL2_TX_EMPTY)
	{
		theXceiverShutoffServiceFlag = 0;
		xceiverEnableRx();
	}
}



/*
Call this once at boot-time.
*/
#pragma CODE_SECTION(transmitInit,".h0_code")
void transmitInit(void)
{
  // No message is pending
  pTheEndOfTxMsg = theTxBuf;
  pTheNextTxByte = 0;
}

/*
If the end-of-msg pointer is not pointing to the beginning
of the buffer, there's a transmiut in progress. The ISR will
reset the pointer after the last character is sent.
*/
#pragma CODE_SECTION(isTxInProgress,".h0_code")
int isTxInProgress(void)
{
  return !(pTheEndOfTxMsg == theTxBuf);
}

/*
Transmit a message. Non-blocking: function returns immediately
while transmission is handled from interrupt-land. Pass in a null
pointer if you have already written the message directly into the
TX buffer. Function automatically appends a Modbus CRC to the
message.
*/
#pragma CODE_SECTION(txMsg,".h0_code")
int txMsg(unsigned char* pMsg, unsigned len)
{
  unsigned crc;

  if(isTxInProgress())
    return -TX_IN_PROGRESS;
  if(len > (THE_TX_BUF_SIZE - 2))
    return -TX_OVERFLOW;

  // If the message pointer is null, assume the caller has already
  // written the reply directly into the TX buffer
  if(pMsg)
    mymemcpy((void*)theTxBuf, (void*)pMsg, len);
  
  // Append a CRC to the end
    if(pMsg)
      crc = calc_modbus_crc((unsigned*)pMsg, len);
    else
      crc = calc_modbus_crc((unsigned*)theTxBuf, len);      

  theTxBuf[len] = (unsigned char)(crc >> 8);
  theTxBuf[len + 1] = (unsigned char)(crc & 0xff);

  pTheEndOfTxMsg = &theTxBuf[len] + 1;

  // If we're in half duplex mode, turn the transmitter on.
  // The TX ISR will turn it off after the last byte is sent.
  xceiverEnableTx();

  // Send the first byte manually to prime the pump
  pTheNextTxByte = &theTxBuf[1];
  PF2.SCITXBUFB = theTxBuf[0];
  return TX_OK;
}
