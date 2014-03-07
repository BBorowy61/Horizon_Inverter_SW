
MEMORY
{
    PAGE 0 :												/* flash memory */
		FPGA_FLASH:	origin = 3DC000h	, length = 10000h	/* Sector E,F,G,H */
    	INIT_CODE:	origin = 3EC000h	, length = 400h
    	ROM		:	origin = 3EC400h	, length = 6C00h	/* Sector C,D */
	   	L1_FLASH:	origin = 3F3000h	, length = 1000h	/* Sector B */
	   	H0_FLASH:	origin = 3F4000h	, length = 1FF0h	/* Sector B */
	   	CHKSUM	:	origin = 3F5FF0h	, length = 10h
    	TEXT_FLASH:	origin = 3F6000h	, length = 1000h	/* Sector A */
	   	MAIN_CODE:	origin = 3F7000h	, length = 0400h
	  	RESET_VECT:	origin = 3F7FF6h	, length = 2h

    PAGE 1 :   											/* RAM data memory */
		M0_STACK: origin =  0000h 	, length =  0180h
		M0_RAM	: origin =  0180h 	, length =  0180h
		M1_RAM	: origin =  0300h 	, length =  0500h
		P_FRAME0: origin =	0800h	, length =	0500h
		PIE		: origin =	0D00h	, length =	0100h
		P_FRAME1: origin =	6000h	, length =	1000h
		P_FRAME2: origin =	7000h	, length =	1000h
		L0_COMM	: origin =  8000h 	, length =  0100h
		L0_DATA	: origin =  8100h 	, length =  0F00h
		L1_RAM	: origin =  9000h 	, length =  1000h	/* code */
		H0_RAM	: origin =  3F8000h , length =  2000h	/* code */
		NV_RAM	: origin =  80000h 	, length =  8000h
		NV_RAM2	: origin =  88000h 	, length =  8000h
}

SECTIONS
{
	.rom_code:		> ROM			PAGE 0
    .data 	:		> ROM			PAGE 0
	.cinit	:		> ROM 			PAGE 0
	.const	:		> ROM 			PAGE 0
	.rom_const:		> ROM 			PAGE 0
	.chksum	:		> CHKSUM		PAGE 0
	.main_code:		> MAIN_CODE		PAGE 0
	.begin	:		> RESET_VECT	PAGE 0
	.fpga_code:		> FPGA_FLASH	PAGE 0
	.init_code:		> INIT_CODE		PAGE 0
	.vectors:		> PIE			PAGE 1
	.stack	:		> M0_STACK 		PAGE 1
    .fbk_data:		> M0_RAM		PAGE 1
    .bss	:		> M1_RAM		PAGE 1
	.p_frame0:		> P_FRAME0		PAGE 1
	.p_frame1:		> P_FRAME1		PAGE 1
	.p_frame2:		> P_FRAME2		PAGE 1
	.switch:		> L1_FLASH PAGE=0,run=L1_RAM PAGE=1
	.h0_code:		> H0_FLASH PAGE=0,run=H0_RAM PAGE=1
	.comm_data:		> L0_COMM		PAGE 1
	UNION:			> L0_DATA		PAGE 1
	{
	GROUP:
		{
	    .que_data:
		.scia_data:
		.rec_data:
		.test_data:
		.flt_data:
		.modbus_data:
		.sysmem	:
		}    
	GROUP:
		{
	    .ebss:
	    .xm_data:
		}
	}
	UNION:			run=L1_RAM PAGE=1
	{
	GROUP:			> TEXT_FLASH PAGE=0
		{
		.econst:
		.text:
		}
	.l1_code:		> L1_FLASH PAGE=0
	}
	.nv_ram:		> NV_RAM
	.nv_ram2:		> NV_RAM2

}

