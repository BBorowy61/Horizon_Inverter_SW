//	PERIPHERAL FRAME 0 (16/32-bit)
//	------------------------------

struct GPTIMER {
volatile unsigned int TIMERTIM;		// GP Timer 0 Counter Register 
volatile unsigned int TIMERTIMH;	// GP Timer 0 Counter Register 
volatile unsigned int TIMERPRD;		// GP Timer 0 Period Register 
volatile unsigned int TIMERPRDH;	// GP Timer 0 Period Register 
volatile unsigned int TIMERTCR;		// GP Timer 0 Control Register 
volatile unsigned int TIMER_REG5;
volatile unsigned int TIMERTPR;		// GP Timer 0 Prescale Register 
volatile unsigned int TIMERTPRH;	// GP Timer 0 Prescale Register
}; 

struct P_FRAME0 {

//	Reserved (800-A7F)

volatile unsigned int REG_000[640];

//	Flash Configuration Registers (A80-ADF)

volatile unsigned int FOPT;			// Flash Option Register 
volatile unsigned int REG_A81;
volatile unsigned int FPWR;			// Flash Power Mode Register 
volatile unsigned int FSTATUS;		// Flash Status Register 
volatile unsigned int FSTDBYWAIT;	// Flash Standby Wait Register 
volatile unsigned int FACTIVEWAIT;	// Flash Active Wait Register 
volatile unsigned int FBANKWAIT;	// Flash Access Wait Register
volatile unsigned int FOTPWAIT;		// OTP Access Wait Register
volatile unsigned int REG_A87[88];

//	Reserved (AF0-B1F)

volatile unsigned int REG_AF0[48];

//	Code Security Module Registers (AE0-AEF)

volatile unsigned int KEY[8];			// CSM KEY Register
volatile unsigned int REG_AE8[7];
volatile unsigned int CSMSCR;		// CSM Status & Control Register
 
//	XINTF Registers (B20-B3F)

volatile unsigned long XTIMING0;	// XINTF Timing Register Zone 0 
volatile unsigned long XTIMING1;	// XINTF Timing Register Zone 1 
volatile unsigned long XTIMING2;	// XINTF Timing Register Zone 2 
volatile unsigned long XTIMING3;	// XINTF Timing Register Zone 3 
volatile unsigned long XTIMING4;	// XINTF Timing Register Zone 4 
volatile unsigned long XTIMING5;	// XINTF Timing Register Zone 5 
volatile unsigned long XTIMING6;	// XINTF Timing Register Zone 6 
volatile unsigned long XTIMING7;	// XINTF Timing Register Zone 7 
volatile unsigned long REG_B30;
volatile unsigned long REG_B32;
volatile unsigned long XINTCNF2;	// XINTF Configuration Register 
volatile unsigned long XINTCNF1;	// XINTF Configuration Register (not used )
volatile unsigned int  XBANK;		// XINTF Bank Control Register 
volatile unsigned int  REG_B39;
volatile unsigned int  XREVISION;	// XINTF Revision Register 
volatile unsigned int  REG_B3B;
volatile unsigned long REG_B3C;
volatile unsigned long REG_B3E;

//	Reserved (B40-BFF)

volatile unsigned int REG_B40[192];

//	Timer Registers	(C00-C3F)

struct GPTIMER	TIMER0,TIMER1,TIMER2;

volatile unsigned int REG_C18[40];

//	Reserved (C40-CDF)

volatile unsigned int REG_C40[160];

//	PIE Registers (CE0-CFF)

volatile unsigned int PIECTRL;	// PIE Control Register 
volatile unsigned int PIEACK;	// PIE Acknowledge Register 
volatile unsigned int PIEIER1;	// PIE Interrupt Enable Registers 
volatile unsigned int PIEIFR1;
volatile unsigned int PIEIER2;
volatile unsigned int PIEIFR2;
volatile unsigned int PIEIER3;
volatile unsigned int PIEIFR3;
volatile unsigned int PIEIER4;
volatile unsigned int PIEIFR4;
volatile unsigned int PIEIER5;
volatile unsigned int PIEIFR5;
volatile unsigned int PIEIER6;
volatile unsigned int PIEIFR6;
volatile unsigned int PIEIER7;
volatile unsigned int PIEIFR7;
volatile unsigned int PIEIER8;
volatile unsigned int PIEIFR8;
volatile unsigned int PIEIER9;
volatile unsigned int PIEIFR9;
volatile unsigned int PIEIER10;
volatile unsigned int PIEIFR10;
volatile unsigned int PIEIER11;
volatile unsigned int PIEIFR11;
volatile unsigned int PIEIER12;
volatile unsigned int PIEIFR12;

volatile unsigned int REG_CFA[6];
};

//	PERIPHERAL FRAME 2 (16-bit peripheral bus)
//	------------------------------------------

struct P_FRAME2 {

//	Reserved (7000-700F)

volatile unsigned int REG_000[16];

//	System Control Registers (7010-702F)

volatile unsigned int REG_010[10];
volatile unsigned int HISPCP;	// High Speed Clock Prescaler
volatile unsigned int LOSPCP;	// Low Speed Clock Prescaler
volatile unsigned int PCLKCR;	// Peripheral Clock Control Register
volatile unsigned int REG_01D;
volatile unsigned int LPMCR0;	// Low Power Mode Control Register 0
volatile unsigned int LPMCR1;	// Low Power Mode Control Register 1
volatile unsigned int REG_020;
volatile unsigned int PLLCR;	// PLL Control Register
volatile unsigned int SCSR;		// System Control and Status Register
volatile unsigned int WDCNTR;
volatile unsigned int REG_024;
volatile unsigned int WDKEY;
volatile unsigned int REG_026[3];
volatile unsigned int WDCR;
volatile unsigned int REG_02A[6];

//	Reserved (7030-703F)

volatile unsigned int REG_030[16];

//	SPI-A Registers (7040-704F)

volatile unsigned int SPICCR;
volatile unsigned int SPICTL;
volatile unsigned int SPISTS;
volatile unsigned int REG_043;
volatile unsigned int SPIBRR;
volatile unsigned int REG_045;
volatile unsigned int SPIRXEMU;
volatile unsigned int SPIRXBUF;
volatile unsigned int SPITXBUF;
volatile unsigned int SPIDAT;
volatile unsigned int SPIFFTX;
volatile unsigned int SPIFFRX;
volatile unsigned int SPIFFCT;
volatile unsigned int REG_04D;
volatile unsigned int REG_04E;
volatile unsigned int SPIPRI;

//	SCI-A Registers (7050-705F)

volatile unsigned int SCICCR;		// SCI Communication Control Register 
volatile unsigned int SCICTL1;	// SCI Control Register 1 
volatile unsigned int SCIHBAUD;	// SCI Baud Select register, high byte 
volatile unsigned int SCILBAUD;	// SCI Baud Select register, low byte 
volatile unsigned int SCICTL2;	// SCI Control Register 2 
volatile unsigned int SCIRXST;	// SCI Receive Status Register 
volatile unsigned int SCIRXEMU;	// SCI Emulation data buffer Register 
volatile unsigned int SCIRXBUF;	// SCI Receiver data buffer Register 
volatile unsigned int REG_058;
volatile unsigned int SCITXBUF;	// SCI Transmit data buffer Register 
volatile unsigned int SCIFFTX;	// SCI FIFO Transmit Register 
volatile unsigned int SCIFFRX;	// SCI FIFO Receive Register 
volatile unsigned int SCIFFCT;	// SCI FIFO Control Register 
volatile unsigned int REG_05D[2];
volatile unsigned int SCIPRI;		// SCI Priority Control Register 

//	Reserved (7060-706F)

volatile unsigned int REG_060[16];

//	External Interrupt Registers (7070-707F)

volatile unsigned int XINT1CR;		// Interrupt 1 Control Register 
volatile unsigned int XINT2CR;		// Interrupt 2 Control Register 
volatile unsigned int REG_072[5];
volatile unsigned int XNMICR;		// NMI Control Register 
volatile unsigned int XINT1CTR;		// Interrupt 1 Counter Register 
volatile unsigned int XINT2CTR;		// Interrupt 2 Counter Register 
volatile unsigned int REG_07A[5];
volatile unsigned int XNMICTR;		// NMI Counter Register 

//	Reserved (7080-70BF)

volatile unsigned int REG_080[64];

//	GPIO Mux Registers (70C0-70DF)

volatile unsigned int GPAMUX;		// mux control register 
volatile unsigned int GPADIR;		// direction control register 
volatile unsigned int GPAQUAL;		// input qualification control register 
volatile unsigned int REG_0C3;

volatile unsigned int GPBMUX;		// mux control register 
volatile unsigned int GPBDIR;		// direction control register 
volatile unsigned int GPBQUAL;		// input qualification control register 
volatile unsigned int REG_0C7[5];

volatile unsigned int GPDMUX;		// mux control register 
volatile unsigned int GPDDIR;		// direction control register 
volatile unsigned int GPDQUAL;		// input qualification control register 
volatile unsigned int REG_0CF;

volatile unsigned int GPEMUX;		// mux control register 
volatile unsigned int GPEDIR;		// direction control register 
volatile unsigned int GPEQUAL;		// input qualification control register 
volatile unsigned int REG_0D3;

volatile unsigned int GPFMUX;		// mux control register 
volatile unsigned int GPFDIR;		// direction control register 
volatile unsigned int REG_0D6[2];

volatile unsigned int GPGMUX;		// mux control register 
volatile unsigned int GPGDIR;		// direction control register 
volatile unsigned int REG_0DA[6];

//	GPIO Data Registers (70E0-70FF)

volatile unsigned int GPADAT;		// data register 
volatile unsigned int GPASET;		// set register 
volatile unsigned int GPACLEAR;		// clear register 
volatile unsigned int GPATOGGLE;	// toggle register 

volatile unsigned int GPBDAT;		// data register 
volatile unsigned int GPBSET;		// set register 
volatile unsigned int GPBCLEAR;		// clear register 
volatile unsigned int GPBTOGGLE;	// toggle register 
volatile unsigned int REG_0E8[4];

volatile unsigned int GPDDAT;		// data register 
volatile unsigned int GPDSET;		// set register 
volatile unsigned int GPDCLEAR;		// clear register 
volatile unsigned int GPDTOGGLE;	// toggle register 

volatile unsigned int GPEDAT;		// data register 
volatile unsigned int GPESET;		// set register 
volatile unsigned int GPECLEAR;		// clear register 
volatile unsigned int GPETOGGLE;	// toggle register 

volatile unsigned int GPFDAT;		// data register 
volatile unsigned int GPFSET;		// set register 
volatile unsigned int GPFCLEAR;		// clear register 
volatile unsigned int GPFTOGGLE;	// toggle register
 
volatile unsigned int GPGDAT;		// data register 
volatile unsigned int GPGSET;		// set register 
volatile unsigned int GPGCLEAR;		// clear register 
volatile unsigned int GPGTOGGLE;	// toggle register 
volatile unsigned int REG_0FC[4];

//	ADC Registers (7100-711F)

volatile unsigned int ADCTRL1;		// ADC Control Register 1 
volatile unsigned int ADCTRL2;		// ADC Control Register 2
volatile unsigned int ADCMAXCONV;	// ADC maximum conversion channels 
volatile unsigned int ADCCHSELSEQ1;	// ADC channel select sequencing 
volatile unsigned int ADCCHSELSEQ2;
volatile unsigned int ADCCHSELSEQ3;
volatile unsigned int ADCCHSELSEQ4;
volatile unsigned int ADCASEQSR;	// ADC auto sequence status 
volatile unsigned int ADCRESULT0;	// ADC Result Register 0 
volatile unsigned int ADCRESULT1;
volatile unsigned int ADCRESULT2;
volatile unsigned int ADCRESULT3;
volatile unsigned int ADCRESULT4;
volatile unsigned int ADCRESULT5;
volatile unsigned int ADCRESULT6;
volatile unsigned int ADCRESULT7;
volatile unsigned int ADCRESULT8;
volatile unsigned int ADCRESULT9;
volatile unsigned int ADCRESULT10;
volatile unsigned int ADCRESULT11;
volatile unsigned int ADCRESULT12;
volatile unsigned int ADCRESULT13;
volatile unsigned int ADCRESULT14;
volatile unsigned int ADCRESULT15;
volatile unsigned int ADCTRL3;			// ADC Control Register 3 
volatile unsigned int ADCST;			// ADC Status Register 
volatile unsigned int REG_11A[6];

//	Reserved (7120-73FF)

volatile unsigned int REG_120[736];

//	EV-A Registers (7400-743F)

volatile unsigned int GPTCONA;		// GP Timer Control Register 
volatile unsigned int T1CNT;		// Timer 1 Count Register 
volatile unsigned int T1CMPR;		// Timer 1 Compare Register 
volatile unsigned int T1PR;			// Timer 1 Period Register 
volatile unsigned int T1CON;		// Timer 1 Control Register 
volatile unsigned int T2CNT;		// Timer 2 Count Register 
volatile unsigned int T2CMPR;		// Timer 2 Compare Register 
volatile unsigned int T2PR;			// Timer 2 Period Register 
volatile unsigned int T2CON;		// Timer 2 Control Register 
volatile unsigned int EXTCONA;		// Extension Control Register 
volatile unsigned int REG_40A[7];
volatile unsigned int COMCONA;		// Compare Control Register 
volatile unsigned int REG_412;
volatile unsigned int ACTRA;		// Compare Action Control Register 
volatile unsigned int REG_414;
volatile unsigned int DBTCONA;		// Dead Band Timer Control Register 
volatile unsigned int REG_416;
volatile unsigned int CMPR1;		// Compare Register 1
volatile unsigned int CMPR2;		// Compare Register 2
volatile unsigned int CMPR3;		// Compare Register 3
volatile unsigned int REG_41A[18];
volatile unsigned int EVAIMRA;		// Interrupt Mask Register A
volatile unsigned int EVAIMRB;		// Interrupt Mask Register B
volatile unsigned int EVAIMRC;		// Interrupt Mask Register C
volatile unsigned int EVAIFRA;		// Interrupt Flag Register A
volatile unsigned int EVAIFRB;		// Interrupt Flag Register B
volatile unsigned int EVAIFRC;		// Interrupt Flag Register C
volatile unsigned int REG_432[14];

//	Reserved (7440-74FF)

volatile unsigned int REG_440[192];

//	EV-B Registers (7500-753F)

volatile unsigned int GPTCONB;		// GP Timer Control Register 
volatile unsigned int T3CNT;		// Timer 3 Count Register 
volatile unsigned int T3CMPR;		// Timer 3 Compare Register 
volatile unsigned int T3PR;			// Timer 3 Period Register 
volatile unsigned int T3CON;		// Timer 3 Control Register 
volatile unsigned int T4CNT;		// Timer 4 Count Register 
volatile unsigned int T4CMPR;		// Timer 4 Compare Register 
volatile unsigned int T4PR;			// Timer 4 Period Register 
volatile unsigned int T4CON;		// Timer 4 Control Register 
volatile unsigned int EXTCONB;		// Extension Control Register 
volatile unsigned int REG_50A[7];
volatile unsigned int COMCONB;		// Compare Control Register 
volatile unsigned int REG_512;
volatile unsigned int ACTRB;		// Compare Action Control Register 
volatile unsigned int REG_514;
volatile unsigned int DBTCONB;		// Dead Band Timer Control Register 
volatile unsigned int REG_516;
volatile unsigned int CMPR4;		// Compare Register 4
volatile unsigned int CMPR5;		// Compare Register 5
volatile unsigned int CMPR6;		// Compare Register 6
volatile unsigned int REG_51A[18];
volatile unsigned int EVBIMRA;		// Interrupt Mask Register A
volatile unsigned int EVBIMRB;		// Interrupt Mask Register B
volatile unsigned int EVBIMRC;		// Interrupt Mask Register C
volatile unsigned int EVBIFRA;		// Interrupt Flag Register A
volatile unsigned int EVBIFRB;		// Interrupt Flag Register B
volatile unsigned int EVBIFRC;		// Interrupt Flag Register C
volatile unsigned int REG_532[14];

//	Reserved (7540-774F)

volatile unsigned int REG_540[528];

//	SCI-B Registers (7750-775F)

volatile unsigned int SCICCRB;		// SCI Communication Control Register 
volatile unsigned int SCICTL1B;		// SCI Control Register 1 
volatile unsigned int SCIHBAUDB;	// SCI Baud Select register, high bits 
volatile unsigned int SCILBAUDB;	// SCI Baud Select register, low bits 
volatile unsigned int SCICTL2B;		// SCI Control Register 2 
volatile unsigned int SCIRXSTB;		// SCI Receive Status Register 
volatile unsigned int SCIRXEMUB;	// SCI Emulation data buffer Register 
volatile unsigned int SCIRXBUFB;	// SCI Receiver data buffer Register 
volatile unsigned int REG_758;
volatile unsigned int SCITXBUFB;	// SCI Transmit data buffer Register 
volatile unsigned int SCIFFTXB;		// SCI FIFO Transmit Register 
volatile unsigned int SCIFFRXB;		// SCI FIFO Receive Register 
volatile unsigned int SCIFFCTB;		// SCI FIFO Control Register 
volatile unsigned int REG_75D[2];
volatile unsigned int SCIPRIB;		// SCI Priority Control Register 

//	Reserved (7760-77FF)

volatile unsigned int REG_760[160];


//	McBSP Registers (7800-783F)

volatile unsigned int DRR2;		// Data Receive Register 2
volatile unsigned int DRR1;		// Data Receive Register 1
volatile unsigned int DXR2;		// Data Transmit Register 2
volatile unsigned int DXR1;		// Data Transmit Register 1
volatile unsigned int SPCR2;	// Serial Port Control Register 2
volatile unsigned int SPCR1;	// Serial Port Control Register 1
volatile unsigned int RCR2;		// Receive Control Register 2
volatile unsigned int RCR1;		// Receive Control Register 1
volatile unsigned int XCR2;		// Transmit Control Register 2
volatile unsigned int XCR1;		// Transmit Control Register 1
volatile unsigned int SRGR2;	// Sample Rate Generator Register 2
volatile unsigned int SRGR1;	// Sample Rate Generator Register 1
volatile unsigned int MCR2;		// Sample Rate Generator Register 2
volatile unsigned int MCR1;		// Sample Rate Generator Register 1
volatile unsigned int RCERA;
volatile unsigned int RCERB;
volatile unsigned int XCERA;
volatile unsigned int XCERB;
volatile unsigned int PCR	;	// Pin Control Register
volatile unsigned int REG_813[13];
volatile unsigned int MFFTX;	// FIFO Transmit Register
volatile unsigned int MFFRX;	// FIFO Receive Register
volatile unsigned int MFFCT;	// FIFO Control Register
volatile unsigned int MFFINT;	// FIFO Interrupt Register
volatile unsigned int MFFST;	// FIFO Status Register
volatile unsigned int REG_825[24];
};
