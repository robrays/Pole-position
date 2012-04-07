/*
 * Pole_position.asm
 *
 *  Created: 06-04-2012 19:30:57
 *  Author:
 *			Anders �stergaard Hansen, 
 *			Andreas Baldur Hansen,
 *			Morten Markmann,  
 *			Rasmus Strong Jensen,
 *			Yonas Moa Alizadeh
 */ 

 .INCLUDE "M32ADEF.INC"

//-------------------------------------------------------------------------------------------------------------
//
// Start custom definitions
//

	.DEF DUTY_CYCLE	= R32
	.DEF OVERFLOWS	= R31

//
// End custom definitions
//
//-------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------
//
// Start interupt vector configuration
//

	/* Interupt vector address for Reset */
	.CSEG
		RJMP MAIN

	/* Interupt vector address for UART Receiver */
	.ORG URXCaddr		
		RJMP URXC_INT_HANDELER

	/* Interupt vector address for INT0 */
	.ORG INT0addr	; 
		RJMP INT0_INT_HANDELER

	/* Interupt vector address for Timer0 */
	.ORG OVF0addr
		RJMP TIMER0_INT_HANDLER

//
// End interupt vector configuration
//
//-------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------
//
// Start MAIN routine
//

	.ORG 0x100	; The MAIN routines originates at address: 0x100, please be aware that this might chance
				; due to the number of interupt vectors in use

	MAIN:
		
	//-------------------------------------------------------------------------------------------------------------
	//
	// Initialising the stack
	//
	
		LDI R16, HIGH(RAMEND)
		OUT SPH, R16

		LDI R16, LOW(RAMEND)
		OUT SPL, R16

	//
	// Ending initialising the stack
	//
	//-------------------------------------------------------------------------------------------------------------

	//-------------------------------------------------------------------------------------------------------------
	//
	// Initialising connected bluetooth using USART
	//
	
		LDI R16, (( 1 << RXEN ) | ( 1 << RXCIE )  | ( 1 << TXEN ))		; Enables receiever, transmitter and
		OUT UCSRB, R16													; receiver interupts

		LDI R16, (( 1 << UCSZ1 ) | ( 1 << UCSZ0 ) | ( 1 << URSEL ))						; Setting character mode
		OUT UCSRC, R16													; to 8-bit

		LDI R16, 0x67													; Setting baudrate to 9600 
		OUT UBRRL, R16													; using x = 103 with a frequency of 16 mHz
																		

	//
	// End initialising connected bluetooth using USART
	//
	//-------------------------------------------------------------------------------------------------------------

	//-------------------------------------------------------------------------------------------------------------
	//
	// Start initialising Hall sensor (TIMER0) */
	//

		/*
		CBI DDRB, 0									; Making PBO and input ready for receving pulses from the hall sensor

		LDI R16, (1 << CS00) | 
				 (1 << CS01) | 
				 (1 << CS02) 
		OUT TCCR0, R16

		LDI R16, (1 << TOIE0)						; Enabling Timer0 overflow interrupt 
		OUT TIMSK, R16 
		*/

	//
	// End initialising Hall sensor */
	//
	//-------------------------------------------------------------------------------------------------------------

	//-------------------------------------------------------------------------------------------------------------
	//
	// Start initialising Motor PWM (PD7 (OC2))
	//

		SBI DDRD, 7											; Making pin 7 on port D an output pin
		LDI R16, (1 << COM21) | (1 << WGM20) | (1 << CS20)	; Configuring TCCR2 for non inverted phase correct PWM
		OUT TCCR2, R16										; and no prescaling 
	
		LDI DUTY_CYCLE, 0									; Configuring 0% duty cycle: 255*0/100 = 0 			
		OUT OCR2, DUTY_CYCLE

	//
	// Ending initialising Motor PWM (PD7 (OC2))
	//
	//-------------------------------------------------------------------------------------------------------------

//
// End Main routine
//
//-------------------------------------------------------------------------------------------------------------