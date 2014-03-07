extern int theMsgServiceFlag;
extern int theXceiverShutoffServiceFlag;
extern void transmitInit(void);
extern void enableRxUartB(void);
extern void enableTxUartB(void);
extern void enableTxCharInt(void);
extern void enableRxCharInt(void);                
extern void xceiverEnableRx(void);               

#pragma CODE_SECTION(calc_modbus_crc,".h0_code")
unsigned int calc_modbus_crc(register unsigned far *addr,register unsigned length)
{
	register unsigned i,crcsum; 
    crcsum=0xFFFF;
    for(i=0;i<length;i++)
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
	return (crcsum << 8) | (crcsum >> 8);
}

#pragma CODE_SECTION(initialize_modbus,".h0_code")
void initialize_modbus(void)
{
	// Reset all the service flags and diag counters
	theMsgServiceFlag = 0;
	theXceiverShutoffServiceFlag = 0;
//	unhandledInterruptCount = 0;

	// Initialize the software sttructs in the RX and TX
	// subsuystems.
	receiveInit();
	transmitInit();

	// Enable RX and TX interrupts
	enableRxCharInt();
	enableTxCharInt();

	// Enable RX/TX in the UART, then turn on the receiver
	// side of the transciever.
	enableRxUartB();
	enableTxUartB();
    xceiverEnableRx();
}

