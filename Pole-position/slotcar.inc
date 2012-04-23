;*************************************************************************
;***** Created: 2011-04-14 19:00 *****************************************
;*************************************************************************
;* Library for the SlotCar project
;* 
;* Author	     : RobTek gruppe 2
;* File Name         : "slotcar.inc"
;* Title             : SlotCar library
;* Date              : 2011-02-09
;* Version           : 1.1
;* Target MCU        : ATmega32
;* 
;*************************************************************************

#ifndef _slotcar_INC_
#define _slotcar_INC_

//----------------------------------------------------------------------------------------------------------------------------
//
//	Start custom registers configuration
//
//	NOTICE:
//	The CBI and SBI instructions work with registers $00 to $1F only
//
	
	/*
		Register definitions
	*/
	.equ	PRGSR	= 0x99 	; Program Status Register
	.equ	COMSR	= 0x98	; Communication Status Register
	.equ	DCYCLE	= 0x97	; Dutycycle Register	
	.equ	NUM		= 0x96	; Numerator - Tæller
	.equ	DNOM	= 0x95	; Denominator - Nævner
	.equ	RESULT	= 0x94	; Result of 8bit division
	.equ	INDRL	= 0x93	; Input delay low byte
	.equ	INDRH	= 0x92	; Input delay high byte
	.equ	TMPL	= 0x91	; Temporary low byte holder for allowed next input delay
	.equ	TMPH	= 0x90	; Temproary high byte holder for allowed next input delay


	/*
		PRGSR - Program Status Register
	*/
	.equ	AUTO	= 0	; Auto Mode
	.equ	TLM		= 1	; Track Learning Mode
	.equ	IND		= 2	; Waiting for a new byte to arrive within a fixed time periode
	;.equ	-		= 3	; 
	;.equ	-		= 4	; 
	;.equ	-		= 5	; 
	;.equ	-		= 6	; 
	;.equ	-		= 7	; 


	/* 
		COMSR - Communication Status Register
	*/
	.equ	TYP0	= 0	; Type0
	.equ	TYP1	= 1	; Type1
	.equ	CMD0	= 2	; Command0
	.equ	CMD1	= 3	; Command1
	.equ	PAR0	= 4	; Parameter0
	.equ	PAR1	= 5	; Parameter1
	.equ	PAR2	= 6	; Parameter2
	;.equ	-		= 7	;

//
//	End custom registers configuration
//
//----------------------------------------------------------------------------------------------------------------------------

#endif  /* _M32DEF_INC_ */
