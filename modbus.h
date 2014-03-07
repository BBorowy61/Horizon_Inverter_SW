#ifndef __HANDLER_FUNCS_H__
#define __HANDLER_FUNCS_H__

// All the message handler functions look like this. The broadcast flag is
// because a message handler may behave differently if the command was
// broadcast.
typedef int (*MSG_HANDLER_FUNC)(unsigned char* pMsg, unsigned msgLen);

// Handler function prototypes
extern int handleUnrecognizedCmd(unsigned char* pMsg, unsigned msgLen);
extern int handleEchoCmd(unsigned char* pMsg, unsigned msgLen);
extern int handleReadConsecutive(unsigned char* pMsg, unsigned msgLen);
extern int handleWriteConsecutive(unsigned char* pMsg, unsigned msgLen);


extern MSG_HANDLER_FUNC getHandler(char cmd);

enum HandleMessageErrCodes
{
  HMSG_OK,
  HMSG_NOT_FOR_ME,
  HMSG_BAD_CRC,
  HMSG_INVALID_COMMAND,
  HMSG_TX_FAILED,
  HMSG_TX_BUSY,
  HMSG_REQUEST_TOO_BIG,
  HMSG_BAD_MODBUS_ADDRESS
};
enum SCICTL1_BITS
{
	SCICTL1_RX_ERR_INT_ENA	= 1 << 6,
	SCICTL1_SW_RESET		= 1 << 5,
	SCICTL1_TXWAKE			= 1 << 3,
	SCICTL1_SLEEP			= 1 << 2,
	SCICTL1_TXENA			= 1 << 1,
	SCICTL1_RXENA			= 1
};

enum SCICTL2_BITS
{
	SCICTL2_TXRDY			= 1 << 7,
	SCICTL2_TX_EMPTY		= 1 << 6,
	SCICTL2_RX_BK_INT_ENA	= 1 << 1,
	SCICTL2_TX_INT_ENA		= 1
};

enum SCICCR_BITS
{
	SCICCR_2_STOP_BITS		= 1 << 7,
	SCICCR_EVEN_PARITY		= 1 << 6,
	SCICCR_PARITY_ENABLE	= 1 << 5,
	SCICCR_LOOPBACK_ENA		= 1 << 4,
	SCICCR_ADDR_IDLE_MODE	= 1 << 3,
	SCICCR_SCICHAR2			= 1 << 2,
	SCICCR_SCICHAR1 		= 1 << 1,
	SCICCR_SCICHAR0			= 1
};

enum SCIRXST_BITS
{
	SCIRXST_RX_ERROR			= 1 << 7,
	SCIRXST_RXRDY				= 1 << 6,
	SCIRXST_BRKDT				= 1 << 5,
	SCIRXST_FE					= 1 << 4,
	SCIRXST_OE					= 1 << 3,
	SCIRXST_PE					= 1 << 2,
	SCIRXST_RXWAKE				= 1 << 1
};

enum TIMERTCR_BITS
{
	TIMERTCR_TIF			= 1 << 15,
	TIMERTCR_TIE			= 1 << 14,
	TIMERTCR_FREE			= 1 << 11,
	TIMERTCR_SOFT			= 1 << 10,
	TIMERTCR_TRB			= 1 << 5,
	TIMERTCR_TSS			= 1 << 4
};
enum TxErrCodes
{
  TX_OK,
  TX_IN_PROGRESS,
  TX_OVERFLOW
};

#endif // __HANDLER_FUNCS_H__
