/*
 * Pole_position.asm
 *
 *  Created: 06-04-2012 19:30:57
 *  Author:
 *			Anders Oestergaard Hansen, 
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
		RJMP INIT

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
// Start INIT routine
//

	.ORG 0x100	; The INIT routines originates at address: 0x100, please be aware that this might chance
				; due to the number of interupt vectors in use

	INIT:
		
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
	//	/* UCSRB */
	//	
	//	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|	<- Bit nr.
	//	-----------------------------------------------------------------
	//	| RXCIE	| TXCIE	| UDRIE	| RXEN	| TXEN	| UCSZ2	| RXB8	| TXB8	|	<- UCSRB
	//
	//	Bit 7 – RXCIE: RX Complete Interrupt Enable
	//	Writing this bit to one enables interrupt on the RXC Flag. A USART Receive Complete Interrupt
	//	will be generated only if the RXCIE bit is written to one, the Global Interrupt Flag in SREG is written
	//	to one and the RXC bit in UCSRA is set.
	//	
	//	Bit 6 – TXCIE: TX Complete Interrupt Enable
	//	Writing this bit to one enables interrupt on the TXC Flag. A USART Transmit Complete Interrupt
	//	will be generated only if the TXCIE bit is written to one, the Global Interrupt Flag in SREG is written
	//	to one and the TXC bit in UCSRA is set.
	//	
	//	Bit 5 – UDRIE: USART Data Register Empty Interrupt Enable
	//	Writing this bit to one enables interrupt on the UDRE Flag. A Data Register Empty Interrupt will
	//	be generated only if the UDRIE bit is written to one, the Global Interrupt Flag in SREG is written
	//	to one and the UDRE bit in UCSRA is set.
	//	
	//	Bit 4 – RXEN: Receiver Enable
	//	Writing this bit to one enables the USART Receiver. The Receiver will override normal port operation
	//	for the RxD pin when enabled. Disabling the Receiver will flush the receive buffer
	//	invalidating the FE, DOR, and PE Flags.
	//	
	//	Bit 3 – TXEN: Transmitter Enable
	//	Writing this bit to one enables the USART Transmitter. The Transmitter will override normal port
	//	operation for the TxD pin when enabled. The disabling of the Transmitter (writing TXEN to zero)
	//	will not become effective until ongoing and pending transmissions are completed, that is, when
	//	the transmit Shift Register and transmit Buffer Register do not contain data to be transmitted.
	//	When disabled, the transmitter will no longer override the TxD port.
	//
	//	Bit 2 – UCSZ2: Character Size
	//	The UCSZ2 bits combined with the UCSZ1:0 bit in UCSRC sets the number of data bits (Character
	//	Size) in a frame the receiver and transmitter use.
	//
	//	Bit 1 – RXB8: Receive Data Bit 8
	//	RXB8 is the ninth data bit of the received character when operating with serial frames with nine
	//	data bits. Must be read before reading the low bits from UDR.
	//
	//	/* UCSRC */
	//	
	//	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|	<- Bit nr.
	//	-----------------------------------------------------------------
	//	| URSEL	| UMSEL	| UPM1	| UPM0	| USBS	| UCSZ1	| UCSZ0	| UCPOL	|	<- UCSRC
	//
	//
	//	Bit 7 – URSEL: Register Select
	//	This bit selects between accessing the UCSRC or the UBRRH Register. It is read as one when
	//	reading UCSRC. The URSEL must be one when writing the UCSRC.
	//
	//	Bit 6 – UMSEL: USART Mode Select
	//	This bit selects between Asynchronous and Synchronous mode of operation.
	//
	//	|	UMSEL	|			Mode			|
	//	-----------------------------------------		
	//	|	  0		|  Asynchronous Operation	|	
	//	|	  1		|  Synchronous Operation	|
	//
	//
	//	Bit 5:4 – UPM1:0: Parity Mode
	//	These bits enable and set type of parity generation and check. If enabled, the transmitter will
	//	automatically generate and send the parity of the transmitted data bits within each frame. The
	//	Receiver will generate a parity value for the incoming data and compare it to the UPM0 setting.
	//	If a mismatch is detected, the PE Flag in UCSRA will be set.
	//
	//	|	UPM1	|	 UPM2	|		 Parity Mode		|
	//	-----------------------------------------------------	
	//	|	  0		|	  0		|	Disabled				|	
	//	|	  0		|	  1 	|	Reserved				|
	//	|	  1		|	  0 	|	Enabled, Even Parity	|
	//	|	  1		|	  1 	|	Enabled, Odd Parity		|
	//
	//
	//	Bit 3 – USBS: Stop Bit Select
	//	This bit selects the number of Stop Bits to be inserted by the Transmitter. The Receiver ignores
	//	this setting.
	//
	//	|	USBS	|		 Stop Bit(s)		|
	//	-----------------------------------------		
	//	|	  0		|		    1-bit			|	
	//	|	  1		|			2-bit			|
	//
	//
	//	Bit 2:1 – UCSZ1:0: Character Size
	//	The UCSZ1:0 bits combined with the UCSZ2 bit in UCSRB sets the number of data bits (Character
	//	Size) in a frame the Receiver and Transmitter use.
	//
	//	|	UCSZ2	|	UCSZ1	|	UCSZ0	|	Character Size	|
	//	---------------------------------------------------------	
	//	|	  0		|	  0		|	  0		|		5-bit		|	
	//	|	  0		|	  0 	|	  1		|		6-bit		|
	//	|	  0		|	  1 	|	  0		|		7-bit		|
	//	|	  0		|	  1 	|	  1		|		8-bit		|
	//	|	  1		|	  0		|	  0		|	   Reserved		|	
	//	|	  1		|	  0 	|	  1		|	   Reserved		|
	//	|	  1		|	  1 	|	  0		|	   Reserved		|
	//	|	  1		|	  1 	|	  1		|		9-bit		|
	//
	//
	//	Bit 0 – UCPOL: Clock Polarity
	//	This bit is used for Synchronous mode only. Write this bit to zero when Asynchronous mode is
	//	used. The UCPOL bit sets the relationship between data output change and data input sample,
	//	and the synchronous clock (XCK).
	//
	//	/* UBRRH and UBRRL */
	//
	//	|	15	|	14	|	13	|	12	|	11	|   10	|	9	|	8	|	<-	Bit nr.
	//	-----------------------------------------------------------------
	//	| URSEL	|	-	|	-	|	-	|		    UBRR[11:8]			|	<-	UBRRH
	//	-----------------------------------------------------------------
	//	|							UBRR[7:0]							|	<-	UBRRL
	//	-----------------------------------------------------------------
	//	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|	<-	Bit nr.
	//
	//	Actual code starts below
	//
	
		LDI R16, (( 1 << RXEN ) | ( 1 << RXCIE )  | ( 1 << TXEN ))		; Enables receiever, transmitter and
		OUT UCSRB, R16													; receiver interupts

		LDI R16, (( 1 << UCSZ1 ) | ( 1 << UCSZ0 ) | ( 1 << URSEL ))		; Setting character mode
		OUT UCSRC, R16													; to 8-bit

		LDI R16, 0x67													; Setting baudrate to 9600 
		OUT UBRRL, R16													; using x = 103 with a frequency of 16 mHz
																		
	//
	// End initialising connected bluetooth using USART
	//
	//-------------------------------------------------------------------------------------------------------------

	//-------------------------------------------------------------------------------------------------------------
	//
	// Start initialising Hall sensor (TIMER0)
	//

		/*
		CBI DDRB, 0				; Making PBO and input ready for receving pulses from the hall sensor

		LDI R16, (1 << CS00) | 
				 (1 << CS01) | 
				 (1 << CS02) 
		OUT TCCR0, R16

		LDI R16, (1 << TOIE0)	; Enabling Timer0 overflow interrupt 
		OUT TIMSK, R16 
		*/

	//
	// End initialising Hall sensor
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

	SEI	; Enable interupts

//
// End INIT routine
//
//-------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------
//
// Start MAIN routine
//
	MAIN:
		RJMP MAIN
//
// End MAIN routine
//
//-------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------
//
// Start USART RX interupt routine
//

	URXC_INT_HANDELER:

		/*
		LDI R16, (1 << 5)

		OUT PORTB, R16
		*/

		
		//LDI R21, 0x20
		IN R21, UDR
		//LDI R17, '0' 
		//SUB R21, R17
		
		//LDI DUTY_CYCLE, 0x1B // 0001 1100
		//MUL DUTY_CYCLE, R21
		
		
		//LDI DUTY_CYCLE, 0x7F // Duty cycle 50%

		OUT OCR2, R21
	
		RETI
	
//
// End USART RX interupt routine
//
//-------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------
//
// Start Timer0 interupt handeler
//

		TIMER0_INT_HANDLER:
			INC OVERFLOWS			; Adds one to the overflow count
			RETI					; Return from where left, and set TOV0 to 0

//
// End Timer0 interupt handeler
//
//-------------------------------------------------------------------------------------------------------------