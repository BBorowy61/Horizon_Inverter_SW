#define __HARDWARE_C__
#include "2812reg.h"
#include "literals.h"
#include "struct.h"
#include "modbus.h"
#include "io_def.h"

/********************************************************************/
/*		2812 I/O structures (defined in 2812reg.h)					*/
/********************************************************************/
extern struct P_FRAME0 PF0;
extern struct P_FRAME2 PF2;

#pragma CODE_SECTION(resetEndOfPacketTimer,".h0_code")
void resetEndOfPacketTimer(void)          
{
	PF0.TIMER2.TIMERTCR |= TIMERTCR_TSS;			// Stop the timer
	PF0.TIMER2.TIMERTCR |= TIMERTCR_TRB;			// reload the counter

	// Ack so you don't get a spurious interrupt
	// TODO: is that ack necessary on this chip?
	PF0.TIMER2.TIMERTCR |= (unsigned)TIMERTCR_TIF;	
	
	// Start the timer
	// Don't write a 1 to TIF because that inadvertently clears the interrupt!
	PF0.TIMER2.TIMERTCR &= (~TIMERTCR_TSS & ~(unsigned)TIMERTCR_TIF);
}

#pragma CODE_SECTION(enableRxCharInt,".h0_code")
void enableRxCharInt(void)                
{
	// Enable the interrupt in the peripheral
	PF2.SCICTL2B |= SCICTL2_RX_BK_INT_ENA;
}

#pragma CODE_SECTION(enableTxCharInt,".h0_code")
void enableTxCharInt(void)                
{
	// Enable the interrupt in the peripheral
   PF2.SCICTL2B |= SCICTL2_TX_INT_ENA;	
}

#pragma CODE_SECTION(enableRxUartB,".h0_code")
void enableRxUartB(void)
{
	 PF2.SCICTL1B |= SCICTL1_RXENA;
}

#pragma CODE_SECTION(enableTxUartB,".h0_code")
void enableTxUartB(void)
{
	 PF2.SCICTL1B |= SCICTL1_TXENA;
}
// Returns 1 IFF End Of Packet Timer interrupt is enabled
#pragma CODE_SECTION(isEndOfPacketTimerIntEnabled,".h0_code")
int isEndOfPacketTimerIntEnabled(void)      
{
	if(PF0.TIMER2.TIMERTCR & TIMERTCR_TIE)
		return 1;
	else
		return 0;
}

#pragma CODE_SECTION(xceiverEnableRx,".h0_code")
void xceiverEnableTx(void)               
{
	// Half duplex RS485
	// Enable transmitter
	PF2.GPBSET = RS485_TXEN_OUTPUT;
}

#pragma CODE_SECTION(xceiverEnableTx,".h0_code")
void xceiverEnableRx(void)               
{
	// Half duplex RS485
	// Enable Receiver
	PF2.GPBCLEAR = RS485_TXEN_OUTPUT;
}

/*
The uarts on this chip are designed to lock up until manually
reset if a comm error is detected. Have to monitor this condition.
Call this periodically.

TODO: I would like to install an ISR that resets the UART if any 
physical errors are detected (I would prefer not to poll the 
error bit). There is a bit called RX_ERR_INT_ENA in SCICTL1, 
but I don't see a vector for it in the PIE vector table 
( 6.3.4 The PIE Vector Table in SPRU078E). Does the error 
interrupt vector to the same ISR as the RX interrupt or 
something?
*/
#pragma CODE_SECTION(uartErrorMonitorTask,".h0_code")
void uartErrorMonitorTask(void)
{
	volatile int i;

	if(PF2.SCIRXSTB & SCIRXST_RX_ERROR)
	{
		PF2.SCICTL1B &= ~SCICTL1_SW_RESET;
		// Wait a couple clock cycles
		for(i=0; i<16; i++) ;		
		PF2.SCICTL1B |= SCICTL1_SW_RESET;
	}
}

