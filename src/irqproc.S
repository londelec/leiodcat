/*
 ============================================================================
 Name        : irqproc.S
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : Atmel interrupt service routine loader

  Change log  :

  *********V1.00 25/02/2015**************
  Initial revision

 ============================================================================
 */

#include <avr/interrupt.h>


// Sample code from avr-libc source file 'gcrt1.S'
//	.text
//	.global	__bad_interrupt
//	.func	__bad_interrupt
//__bad_interrupt:
//	.weak	__vector_default
//	.set	__vector_default, __vectors
//	XJMP	__vector_default
//	.endfunc


	.text	// Text (code) section
/***************************************************************************
* UARTC1 interrupts
* Use generic ISR for all UART interputs.
* 8bit global variable 'irqasmenum' is used to select UART and processing function
* [25/02/2015]
***************************************************************************/
	.global	USARTC1_RXC_vect			// Making the label global initiates jump from actual IRQ vector to this function
USARTC1_RXC_vect:
	push	r30							// push the old value of R30
	ldi		r30, USARTC1_RXC_vect_num	// Enum used to distinguish interrupts, the same as IRQ vector address for simplicity
	sts		irqasmenum, r30				// Save enum to public variable
	pop		r30							// pop original value of R30
	jmp		UART_GENERIC_IRQ_VECTOR


	.global	USARTC1_DRE_vect
USARTC1_DRE_vect:
	push	r30
	ldi		r30, USARTC1_DRE_vect_num
	sts		irqasmenum, r30
	pop		r30
	jmp		UART_GENERIC_IRQ_VECTOR


	.global	USARTC1_TXC_vect
USARTC1_TXC_vect:
	push	r30
	ldi		r30, USARTC1_TXC_vect_num
	sts		irqasmenum, r30
	pop		r30
	jmp		UART_GENERIC_IRQ_VECTOR


/***************************************************************************
* UARTE1 interrupts
* Use generic ISR for all UART interputs.
* 8bit global variable 'irqasmenum' is used to select UART and processing function
* [25/02/2015]
***************************************************************************/
	.global	USARTE1_RXC_vect
USARTE1_RXC_vect:
	push	r30
	ldi		r30, USARTE1_RXC_vect_num
	sts		irqasmenum, r30
	pop		r30
	jmp		UART_GENERIC_IRQ_VECTOR


	.global	USARTE1_DRE_vect
USARTE1_DRE_vect:
	push	r30
	ldi		r30, USARTE1_DRE_vect_num
	sts		irqasmenum, r30
	pop		r30
	jmp		UART_GENERIC_IRQ_VECTOR


	.global	USARTE1_TXC_vect
USARTE1_TXC_vect:
	push	r30
	ldi		r30, USARTE1_TXC_vect_num
	sts		irqasmenum, r30
	pop		r30
	jmp		UART_GENERIC_IRQ_VECTOR




/***************************************************************************
* Create variable in the data section to store interrupt enum
* Value assignation to a data variable with 'db' doesn't work
* probably due to avr assembly limitation.
* Workaraound is to specify an assembly command which will automatically
* allocate two bytes in the data section
* [25/02/2015]
***************************************************************************/
	.data					// Data section
	.global	irqasmenum		// Make variable name publicly accessible (appears in .map file)
irqasmenum:
	//nop
	push	r30


