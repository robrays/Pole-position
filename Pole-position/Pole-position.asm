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

 .include "M32ADEF.INC"
 .include "slotcar.inc"		


//-------------------------------------------------------------------------------------------------------------
//
// Start custom definitions
//

	.def 	temp = R17					; temporary register
	.def 	temp2 = R18					; temporary register 2
	.def	MDCYCLE = R19				; Motor PWM dutycycle register
	.def	EDCYCLE = R20				; Electromagnet PWM dutycycle register
	.def 	HallCounter1 = R21			; Hall Sensor Counter1 register
	.def	HallCounter2 = R22			; Hall Sensor Counter2 register
	.def	TXreg = R23					; Transmit USART register


//
// End custom definitions
//
//-------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------
//
// Start interupt vector configuration
//

	/* 
		Interupt vector address for Reset 
	*/
	.org 0x00
		RJMP INIT

	/* 
		Interupt vector address for UART Receiver 
	*/
	.org URXCaddr		
		RJMP SERIAL_DATA_RECEIVED_HANDELER	

	/* 
		Interupt vector address for INT0 - Accelerometer 
	*/
	.org INT0addr	 
		RJMP INTERRUPT0_HANDELER			

	/* 
		Interupt vector address for INT0 - Opto sensor 1 and 2 
	*/
	.org INT1addr	 
		RJMP INTERRUPT1_HANDELER			

	/* 
		Interupt vector address for INT0 - Hall sensor 
	*/
	.org INT2addr	 
		RJMP INTERRUPT2_HANDELER			
		
	/*
		Timer 1 Output Compare Match A
	*/
	.org OC1Aaddr
		JMP OC1A_INTERRUPT_HANDLER						

//
// End interupt vector configuration
//
//-------------------------------------------------------------------------------------------------------------


/*********************************************** INITIALSING MACROS ******************************************/

//-------------------------------------------------------------------------------------------------------------
//
// Start INIT routine
//

	.ORG 0x100	; The INIT routines originates at address: 0x100, please be aware that this might chance
				; due to the number of interupt vectors in use

	INIT:
		
		//-------------------------------------------------------------------------------------------------------------
		//
		// Start initialising stack
		//

			LDI 	R16, HIGH(RAMEND)			; Initialize stack		
			OUT 	SPH, R16
			LDI 	R16, LOW(RAMEND)
			OUT 	SPL, R16

		//
		// End initialising stack
		//
		//-------------------------------------------------------------------------------------------------------------

		//-------------------------------------------------------------------------------------------------------------
		//
		// Start initialising core system elements:
		//		- Serial communication
		//		- Motor PWM
		//		- Electromagnet PWM
		//		- Timer1
		//		- External interrupts
		//		- Accelerometer 
		//		- Hall sensor
		//
		
			CALL	INITIALIZE_SERIAL_COMMUNICATION	; Initialize Serial Communication
			CALL	INITIALIZE_MOTOR_PWM			; Initialize MOTOR PWM
			CALL 	INITIALIZE_TIMER1				; Initialize Timer1
			CALL	INITIALIZE_ELECTRO_MAGNET_PWM	; Initialize ELECTROMAGNET PWM
			CALL	INITIALIZE_EXTERNAL_INTERRUPTS	; Initialize INT0 + INT1 + INT2

			CLR		HallCounter1					; Clear Hall Counter1 Register
			CLR		HallCounter2					; Clear Hall Counter2 Register
			LDI		temp, 0x00
			STS		PRGSR, temp						; Clear Program Status register
			STS		COMSR, temp						; Clear Communication Status register
			
			SBI		DDRD, 6							; PD6 Input - Opto 1/2
			SBI		DDRC, 1							; PC1 Output - Red LED
			SBI		DDRA, 5							; PA5 Output - Blue LED
			SBI		DDRA, 6							; PA6 Output - Blue LED - Ground
			CBI 	PORTA, 6						; PA6 - Ground

		//
		// End initialising core system elements:
		//	
		//-------------------------------------------------------------------------------------------------------------

		SEI										; Enable global interrupt


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



/*********************************************** INITIALSING ROUTINES ****************************************/


//-------------------------------------------------------------------------------------------------------------
//
// Start initialising serial communication
//

	INITIALIZE_SERIAL_COMMUNICATION:
        LDI 	temp, 0x00								; Clear all error flags
        OUT		UCSRA, temp

		LDI 	temp, (1<<RXEN)|(1<<RXCIE)|(1<<TXEN)	; Enable receive, receive interrupt and transmit
		OUT		UCSRB, temp
		
		LDI		temp, (1<<UCSZ0)|(1<<UCSZ1)|(1<<URSEL)	; 8bit data, asynchronous, no parity, one stop bit.		
		OUT 	UCSRC, temp								; URSEL: 1 to reach UCSRC register

		LDI		temp, 0x67								; Configuring baudrate to 9600 bps.					
		OUT		UBRRL, temp	
		LDI 	temp, 0x00		
		OUT		UBRRH, temp		

		RET												; Return to caller 

//
// End initialising serial communication
//
//-------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------
//
// Start initialising electro magnet PWM 
//
		
	INITIALIZE_ELECTRO_MAGNET_PWM:
		SBI 	DDRB, 3									; OC0 (PB3) Output - Electromagnet
					
		LDI		EDCYCLE, 0x00							; Load startvalue of dcycle 
		OUT 	OCR0, EDCYCLE
 		
		LDI 	temp, (1<<CS00)|(1<<COM01)|(1<<WGM00)	; No prescaler. Phase correct PWM mode. Non-inverted PWM.
		OUT 	TCCR0, temp
		
		RET												; Return to caller 
//
// End initialising electro magnet PWM 
//
//-------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------
//
// Start initialising Timer1 
//

	INITIALIZE_TIMER1:
		LDI		temp, 0xFF								; Compare match occours when TCNT1 reaches 0xFFFF
		OUT		OCR1AH, temp
		OUT		OCR1AL, temp

		LDI		temp, 0x00								; Compare match mode, 1024 prescaller. If no prescaller: CS12 = 0
		OUT		TCCR1A, temp
		LDI		temp, (1<<WGM12)|(1<<CS12)|(1<<CS10)
		OUT		TCCR1B, temp
		
		LDI		temp, (1<<OCIE1A)						; Timer 1 Output Compare A Match Interrupt Enable
		OUT		TIMSK, temp

		RET												; Return to caller

//
// End initialising Timer1 
//
//-------------------------------------------------------------------------------------------------------------	


//-------------------------------------------------------------------------------------------------------------
//
// Start initialising Motor PWM 
//
		
	INITIALIZE_MOTOR_PWM:
		SBI 	DDRD, 7									; OC2 (PD7) Output - Motor
		
		LDI		MDCYCLE, 0x00							; Load startvalue of dcycle
		OUT 	OCR2, MDCYCLE			
 		
		LDI 	temp, (1<<CS20)|(1<<COM21)|(1<<WGM20)	; No prescaler. Phase correct PWM mode. Non-inverted PWM.
		OUT 	TCCR2, temp
		
		RET												; Return to caller

//
// End initialising Motor PWM 
//
//-------------------------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------------------------
//
// Start initialising External Interrupt 
//

	INITIALIZE_EXTERNAL_INTERRUPTS:
		LDI		temp, (1<<ISC00)|(1<<ISC01)|(1<<ISC10)|(1<<ISC11)	; Enable rising edge of INT0 and INT1
		OUT		MCUCR, temp									
	
		LDI 	temp, (1<<ISC2)										; Enable rising edge of INT2
		OUT 	MCUCSR, temp				

		LDI 	temp, (1<<INT0)|(1<<INT1)|(1<<INT2)					; Enable INT0, INT1 & INT2
		OUT 	GICR, temp					

		RET															; Return to caller

//
// End initialising External Interrupt 
//
//-------------------------------------------------------------------------------------------------------------



/*********************************************** INTERRUPT HANDELERS *****************************************/


//-------------------------------------------------------------------------------------------------------------
//
// Start USART data received interrupt handeler 
//

	SERIAL_DATA_RECEIVED_HANDELER:	
		SBI		PortC, 1								; PC1 LED blink
		SBI		PortA, 6
		CALL	DELAY
		CBI		PortC, 1
		CBI		PortA, 6		
		
		SDRH_STEP1:
			LDS		temp, COMSR							; Check if IND is flagged in
			MOV		R16, temp 							; COMSR
			ANDI	R16,(1 << IND)						; 			
		
			BRNE SDRH_STEP2								; If IND is flagged in COMSR
			SDRH_STEP1_1:								; unflagged it, if not
				LDI R16, (0 << IND)						; jump to SDRH_STEP2
				STS COMSR, R16							;

		SDRH_STEP2:
			IN		temp, UDR							; Read byte from UDR
			SUBI	temp, 0x30							; Convert from Ascii to HEX
		
		SDRH_STEP3:
			LDS		R16, COMSR							; Check if TYP0 or TYP1 is
			ANDI	R16, ((1 << TYP0) | (1 << TYP1))	; flagged in COMSR
			
			BRPL	SDRH_STEP4							; If they are set jump to SDRH_STEP4
			SDRH_STEP3_1:								; if they ae not set flag them
				MOV		R16, temp 						; based on the byte just read in UDR
				CPI		R16, 0x55						; 
				BRNE	SDRH_STEP3_2					; If read udr is 0x55, type is of SET
				LDI		R16, ((1 << TYP0) | (0 << TYP1)); Set the TYP flags to the correct combination
				STS		COMSR, R16						; and update COMSR
				CALL	WAIT_FOR_INPUT					; Start the input delay for next control byte
				RETI									; Return from where we were interruptet and enable interrupts

			SDRH_STEP3_2:								; Check if type is 0xAA (GET)
				MOV		R16, temp 						; 
				CPI		R16, 0xAA						; 
				BRNE	SDRH_STEP3_3					; If read udr is 0xAA, type is of GET
				LDI		R16, ((0 << TYP0) | (1 << TYP1)); Set the TYP flags to the correct combination
				STS		COMSR, R16						; and update COMSR
				CALL	WAIT_FOR_INPUT					; Start the input delay for next control byte
				RETI									; Return from where we were interruptet and enable interrupts

			SDRH_STEP3_3:								; Command is not recognized												
				CALL	CLRCOMSR						; Clear COMSR
				RETI									; Return from where we were interruptet and enable interrupts
					
		SDRH_STEP4:
			LDS		R16, COMSR							; Check if CMD0 or CMD1 is
			ANDI	R16, ((1 << CMD0) | (1 << CMD1))	; flagged in COMSR

			BRPL	SDRH_STEP5							; If they are set jump to SDRH_STEP5
			SDRH_STEP4_1:								; if they ae not set flag them
				MOV		R16, temp 						; based on the byte just read in UDR
				CPI		R16, 0x10						; 
				BRNE	SDRH_STEP4_2					; If read udr is 0x10, command is START
				LDI		R16, ((1 << CMD0) | (0 << CMD1)); Set the CMD flags to the correct combination
				STS		COMSR, R16						; and update COMSR
				CALL	WAIT_FOR_INPUT					; Start the input delay for next control byte
				RETI									; Return from where we were interruptet and enable interrupts
			SDRH_STEP4_2:	
				MOV		R16, temp 						; based on the byte just read in UDR
				CPI		R16, 0x11						; 
				BRNE	SDRH_STEP4_3					; If read udr is 0x11, type is of STOP
				LDI		MDCYCLE, 0x0					; Set MDCYCLE to 0%
				CALL	UPDATE_DUTYCYCLE				; and call UPDATE_DUTYCYCLE
				CALL	CLRCOMSR						; Clear COMSR
				RETI									; Return and enable interrupts
			SDRH_STEP4_3:
				MOV		R16, temp 						; based on the byte just read in UDR
				CPI		R16, 0x12						; 
				BRNE	SDRH_STEP4_4					; If read udr is 0x12, command is AUTOMODE
				CALL	AUTOMODE						; and afterwards call CLRSOMR which clears
				CALL	CLRCOMSR						; COMSR.
				RETI									; And finally return to from where we interruptet and
														; enable interrupts

			SDRH_STEP4_4:								; Command is not recognized												
				CALL	CLRCOMSR						; Clear COMSR
				RETI									; Return from where we were interruptet
														; and enable interrupts
		SDRH_STEP5:
			LDS		R16, COMSR							; Check if command is START COMSR
			ANDI	R16, ((1 << CMD0) | (1 << CMD1))	;
			CPI		R16, (1 << CMD0)					;

			BRNE	SDRH_STEP6							; If command is not START in COMSR
			SDRH_STEP5_1:								; Jump to SDRH_STEP6, if command is START
				LDS		R16, COMSR						; in COMSR, start checking parameters
				ANDI	R16, (1 << PAR0)
				
				BRPL	SDRH_STEP5_2					; If PAR0 is flagged jump to SDRH_STEP5_2
				SDRH_STEP5_1_1:							; Else get parameter byte
					MOV		R16, temp					; Reading parameter byte from UDR
					CPI		R16, 0x0					; Checking if parameter byte is 0
					
					BRNE	SDRH_STEP5_1_2				; If parameter byte is not 0 jump to SDRH_STEP5_1_2
					LDI		MDCYCLE, 0x0				; Else parameter byte is 0 and MDCYCLE is set to 0
					CALL 	UPDATE_DUTYCYCLE			; and motor PWM is updated with value in MDCYCLE
					CALL	CLRCOMSR					; Clear COMSR
					RETI								; Return to where we were interrupted and enable interrupts
				
				SDRH_STEP5_1_2:							; Parameter byte is greater than 0
					MOV		MDCYCLE, temp				; Copy parameter byte into MDCYCLE
					LDS		R17, COMSR					; Updating COMSR with the added PAR0 flagged
					ORI		R17, (1 << PAR0)			;
					STS		COMSR, R17					;

					CALL	WAIT_FOR_INPUT				; Start next input byte delay 
					RETI								; Return to where we were interrupted and enable interrupts

			SDRH_STEP5_2:
				LDS		R16, COMSR						; Check if PAR1 is flagged in COMSR
				ANDI	R16, (1 << PAR1)				;

				BRPL	SDRH_STEP5_3
				SDRH_STEP5_2_1:							; If PAR1 is flagged jump to SDRH_STEP5_3
					MOV		R16, temp					; Reading parameter byte from UDR
					CPI		R16, 0x0					; Checking if parameter byte is 0
					
					BRNE	SDRH_STEP5_2_2				; If parameter byte is not 0 jump to SDRH_STEP5_2_2
					LDI		R16, 0xA
					MUL		MDCYCLE, R16				; Else parameter byte is 0 and MDCYCLE multiplied with 10
					MOV		MDCYCLE, R1					; Moving the multiplied result from R1 to MDCYCLE
					LDS		R17, COMSR					; Updating COMSR with the added PAR1 flagged
					ORI		R17, (1 << PAR1)			;
					STS		COMSR, R17					;
					CALL	WAIT_FOR_INPUT				; Start next input byte delay
					RETI								; Return to where we were interrupted and enable interrupts
				
				SDRH_STEP5_2_2:							; Parameter byte is greater than 0
					LDI		R16, 0xA
					MUL		MDCYCLE, R16				; Multiply MDCYCLE with 10
					MOV		MDCYCLE, R1					; Copy result to MDCYCLE
					ADD		MDCYCLE, temp				; Add parameter byte just read from UDR
					LDS		R17, COMSR					; Updating COMSR with the added PAR1 flagged
					ORI		R17, (1 << PAR1)			;
					STS		COMSR, R17					;
					CALL	WAIT_FOR_INPUT				; Start next input byte delay 
					RETI								; Return to where we were interrupted and enable interrupts

			SDRH_STEP5_3:
				MOV		R16, temp
				CPI		R16, 0x0

				BRNE	SDRH_STEP6						; If the third parameter bute is not zero jump to SDRH_STEP6
				LDI		MDCYCLE, 0x64					; Set MDCYCLE to 100% dutycycle
				CALL 	UPDATE_DUTYCYCLE				; Update dutycycle with value in MDCYCLE 
				CALL	CLRCOMSR						; Return to where we were interrupted and enable interrupts
				RETI									
		
		SDRH_STEP6:
			RETI								; Return and enable global interrupt

//
// End USART data received interrupt handeler 
//
//-------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------
//
// Start External interrupt0 handeler 
//

	INTERRUPT0_HANDELER:	
		NOP
		RETI								; Return and enable global interrupt

//
// End External interrupt0 handeler 
//
//-------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------
//
// Start External interrupt1 handeler 
//

	INTERRUPT1_HANDELER:	
		SBIC	PinD, 6			; Opto2 = PD3, Opto1 = PD6
		RJMP	OptoON2
		RETI					; Return and enable global interrupt

		OptoON2:
			SBI		PortC, 1	; PC1 LED blink
			CALL	DELAY
			CBI		PortC, 1
				
			LDI		TXreg, '2'	; Transmit '2'
			CALL 	TRNSMT

		RETI					; Return and enable global interrupt

//
// End External interrupt1 handeler 
//
//-------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------
//
// Start External interrupt2 handeler 
//

	INTERRUPT2_HANDELER:	
		PUSH	temp						; Push temp to Stack
		IN		temp, SREG					; Push SREG to Stack
		PUSH	temp	
			
		INC		HallCounter1				; Increase HallCounter
		SBRS	temp, SREG_V				; Test for overflow in HallCounter1
		RJMP	DONE						; If no overflow, jump to DONE, pop SREG and return
		INC		HallCounter2				; If yes, increase HallCounter2 
		MOV		TXreg, HallCounter2			; Transmit HallCounter2
		CALL 	TRNSMT

		DONE:	POP		temp				; Pop SREG from Stack
				OUT		SREG, temp	
				POP		temp				; Pop temp from Stack	
				RETI						; Return and enable global interrupt

//
// End External interrupt2 handeler 
//
//-------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------
//
// Start Timer1 Compare match A interrupt handeler 
//

	OC1A_INTERRUPT_HANDLER:
		PUSH	temp						; Push temp to Stack		

		LDI		temp, (1<<OCF1A)
		OUT		TIFR, temp				
		
		LDS		R16, COMSR					; Check if next byte input delay flasg IND is set
		ANDI	R16, (1 << IND)
		CPI		R16, (1 << IND)
			
		BRNE	IND_OFF						; If IND is set, clear COMSR 
		IND_ON:								; and disable Compare Match interrupt on timer 1
			CALL	CLRCOMSR
			LDI		R16, (1 << OCIE1A)
			OUT		TIMSK, R16

		IND_OFF:
		POP		temp						; Pop temp from Stack
		RETI								; Return and enable global interrupt

//
// End Timer1 Compare match A interrupt handeler 
//
//-------------------------------------------------------------------------------------------------------------



/*********************************************** COURE ROUTINES **********************************************/



//-------------------------------------------------------------------------------------------------------------
//
// Start AUTOMODE routine
//

	AUTOMODE:	
		NOP

		RET									; Return

//
// End AUTOMODE routine
//
//-------------------------------------------------------------------------------------------------------------

	
/*********************************************** SERVICE ROUTINES ********************************************/


//-------------------------------------------------------------------------------------------------------------
//
// Start Motor PWM Dutycycle activator
//

	M_PWM:	
		OUT 	OCR2, MDCYCLE				; Loading value from DCYCLE to OCR2
		MOV		TXreg, MDCYCLE				; Transmit DCYCLE
		CALL	TRNSMT 
		RET									; Return

//
// End Motor PWM Dutycycle activator
//
//-------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------
//
// Start Electromagnet PWM Dutycycle activator routine
//

	E_PWM:	
		OUT 	OCR0, EDCYCLE				; Loading value from EDCYCLE to OCR0
		MOV		TXreg, EDCYCLE				; Transmit ELECTRODUTYCYCLE
		CALL	TRNSMT 
		RET									; Return

//
// End Electromagnet PWM Dutycycle activator routine
//
//-------------------------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------------------------
//
// Start Update dutycycle routine
//
//	Description:
//	Sets OCR2 register for Motor PWM with value from MDCYCLE (MDCYCLE = in percent)
//

	UPDATE_DUTYCYCLE:
		PUSH	temp						; Push temp to Stack
		PUSH	temp2						; Push temp2 to Stack

		LDI		temp, 2						; Multiply DCYCLE with 2
		MUL 	MDCYCLE, temp			
		
		MOV		temp2, R0					; Store result in temp2
		
		STS		NUM, MDCYCLE				; Divide DCYCLE with 2
		LDI		temp, 2
		STS		DNOM, temp
		CALL	DIVISION
	
		LDS		temp, RESULT				; Add result in temp2 and move to DCYCLE
		ADD		temp2, temp
		MOV		MDCYCLE, temp2

		CALL	M_PWM						; DCYCLE is now updated and is sent to M_PWM (Motor)

		POP		temp2						; Push temp2 from Stack
		POP		temp						; Push temp from Stack
		RET									; Return 

//
// End Update dutycycle routine
//
//-------------------------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------------------------
//
// Start Division routine
//

	DIVISION:					
		PUSH	R21							; 8bit division
		PUSH	R20
		PUSH	temp
		PUSH	temp2	

		LDS		R21, NUM
		LDS		R20, DNOM

		LDI		temp, 0x01					
		LDI		temp2, 0x00	
						
		DIV:	
			LSL		R21							
			ROL		temp2						
			CP		temp2, R20					
			BRSH	subtr						
			LSL		temp						
			BRCC	DIV							
			RJMP	RETURN						
											
		subtr:	
			SUB		temp2, R20					
			LSL		temp						
			INC		temp						
			BRCC	DIV							
			RJMP	RETURN						

		RETURN:
			STS		RESULT, temp
			POP		temp2
			POP		temp
			POP		R20
			POP		R21
			RET									; Return
//
// End Division routine
//
//-------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------
//
// Start Delay routine
//

	DELAY:	
		PUSH	temp						; Push temp from Stack
		PUSH	temp2						; Push temp2 from Stack
		LDI		temp, 0xFF
		LDI		temp2, 0xFF

	D1:		
		DEC		temp
		NOP	
		BRNE	D1
				
		DEC		temp2
		NOP
		BRNE	D1

		POP		temp2						; Pop temp2 from Stack
		POP		temp						; Pop temp from Stack
		RET									; Return
//
// End Delay routine
//
//-------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------
//
// Start Clear communication status register routine
//

	CLRCOMSR:	
		PUSH	temp						; Push temp to Stack
		
		LDI 	temp, 0x00					; Clear Communication Status Register
		STS		COMSR, temp
		
		POP		temp						; Pop temp from Stack
		RET									; Return

//
// End Clear communication status register routine
//
//-------------------------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------------------------
//
// Start Transmit routine
//

	TRNSMT:	
		SBIS	UCSRA, UDRE					; Wait for UDRE to be ready
		RJMP	TRNSMT						
		OUT		UDR, TXreg					; Transmit TXreg
		
		RET									; Return

//
// End Transmit routine
//
//-------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------
//
//	Start WAIT_FOR_INPUT subroutine 
//
	
	WAIT_FOR_INPUT:

		RJMP CALC_COMPARE_MATCH_LOW_BYTE
		RJMP CALC_COMPARE_MATCH_HIGH_BYTE
		 	

		LDI R16, (1 << OCIE1A)	; Enabling compare match interrupts on Timer1
		OUT TIMSK, R16

//
//	End WAIT_FOR_INPUT subroutine  
//
//----------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------
//
//	Start CALC_COMPARE_MATCH_LOW_BYTE subroutine 
//
	
	CALC_COMPARE_MATCH_LOW_BYTE:

		LDS R16, TCNT1L	; Getting Timer1 low byte count value

		/* 
			Calculate the clock offset until overflow is reached on TCNT1L
		 */
		MOV R18, R16	; Load an ekstra copy of TCNT1L value into R18 
		LDI R19, 255	; TCNT1L can go to a maximum value of 255, so this is the value we calculate the offset against 
		SUB R19, R18	; Subtracting TCNT1L maximum value from the current count value of TCNT1L 
		
		LDS R18, INDRL			; Load the value of the input delay low byte register into R18
		CP R19, R18				; If R19 < R18 or said in another way, if INDRL can be added to TCNT1L 
		BRLO INDRL_NO_OVERFLOW	; without calling overflow, then jump to the subroutine INDRL_NO_OVERFLOW
		JMP INDRL_OVERFLOW

		INDRL_NO_OVERFLOW:
			ADD R18, R16		; Add the value of INDRL to the value of TCNT1L and load it into R18
			STS TMPL, R18		; Load the new calculated compare match low value into out temporary storage TMPL		
			RET

		INDRL_OVERFLOW:
			SUB R18, R19		; Subtract the offset value up to TCNT1L maximum from INDRL 
			STS TMPL, R18		; Load the new found value in R18 into TMPL 
			RET

//
//	End CALC_COMPARE_MATCH_LOW_BYTE subroutine 
//
//----------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------
//
//	Start CALC_COMPARE_MATCH_LOW_BYTE subroutine 
//
	CALC_COMPARE_MATCH_HIGH_BYTE:

		LDS R16, TCNT1H	; Getting Timer1 high byte count value

		/* 
			Calculate the clock offset until overflow is reached on TCNT1H
		 */
		MOV R18, R16	; Load an ekstra copy of TCNT1H value into R18 
		LDI R19, 255	; TCNT1H can go to a maximum value of 255, so this is the value we calculate the offset against 
		SUB R19, R18	; Subtracting TCNT1H maximum value from the current count value of TCNT1H 
		
		LDS R18, INDRH			; Load the value of the input delay high byte register into R18
		CP R19, R18				; If R19 < R18 or said in another way, if INDRH can be added to TCNT1H 
		BRLO INDRH_NO_OVERFLOW	; without calling overflow, then jump to the subroutine INDRH_NO_OVERFLOW	
		JMP INDRL_OVERFLOW

		INDRH_NO_OVERFLOW:
			ADD R18, R16
			STS TMPH, R18
			RET

		INDRH_OVERFLOW:
			SUB R18, R19
			STS TMPH, R18 
			RET	

//
//	End CALC_COMPARE_MATCH_LOW_BYTE subroutine 
//
//----------------------------------------------------------------------------------------------------------------------------