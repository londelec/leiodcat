/*
 ============================================================================
 Name        : timer.c
 Author      : AK
 Version     : V1.01
 Copyright   : Property of Londelec UK Ltd
 Description : Atmel timer management module

  Change log :

  *********V1.01 09/04/2016**************
  Interrupt levels defined in irq.h now

  *********V1.00 19/02/2015**************
  Initial revision

 ============================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer.h"
#include "irq.h"


finetm_t FineTimer;


#define FINETIMER_ENABLE_IRQ	FINETIMER_MCUTMR.INTCTRLA = FINETIMER_INTLVL;	// Enable interrupt
#define FINETIMER_DISABLE_IRQ	FINETIMER_MCUTMR.INTCTRLA = 0;					// Disable interrupt


/***************************************************************************
* System clock initialization
* [26/02/2015]
***************************************************************************/
void clock_init() {
#ifdef INTERNAL_32M_CLOCK

		CLKSYS_Enable(OSC_RC32KEN_bm);
		CLKSYS_Enable(OSC_RC32MEN_bm);
		CLKSYS_Enable(OSC_RC2MEN_bm);
		do {} while ( CLKSYS_IsReady( OSC_RC2MRDY_bm ) == 0 );
		do {} while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );

		CLKSYS_AutoCalibration_Enable( OSC_RC2MCREF_bm, 0 );
		CLKSYS_AutoCalibration_Enable( OSC_RC32MCREF_bm, 0 );
		CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_RC32M_gc);

#else
		OSC.XOSCCTRL = OSC_XOSCSEL_EXTCLK_gc;		// External clock or external oscillator is selected
		CLKSYS_Enable( OSC_XOSCEN_bm );				// Enables the selected external clock source
		do {} while ( CLKSYS_IsReady( OSC_XOSCRDY_bm ) == 0 );	// Flag is set when the external clock source is stable and ready
		CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_XOSC_gc);	// Select the main system clock source#endif

	FineTimer.sec = 0;
	FineTimer.usec100 = 0;
}


/***************************************************************************
* Initialize timers
* [19/02/2015]
* Timer interrupt levels defined in irq.h now
* [09/04/2016]
***************************************************************************/
void timer_init() {

//1ms tick
	TIMER1MS_MCUTMR.PER = TIMERPER_1MSEC;
	TIMER1MS_MCUTMR.INTCTRLA = (TIMER1MS_MCUTMR.INTCTRLA & ~TC1_OVFINTLVL_gm) | TIMER1MS_INTLVL;
	TIMER1MS_MCUTMR.CTRLA = (TIMER1MS_MCUTMR.CTRLA & ~TC1_CLKSEL_gm) | TC_CLKSEL_DIV1_gc;	//enable

//timer for debug
//	TCE0.CTRLA = ( TCE0.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;  // Start Timer with no prescaling

	FINETIMER_MCUTMR.PER = TIMERPER_100USEC;
	FINETIMER_MCUTMR.CTRLA = (FINETIMER_MCUTMR.CTRLA & ~TC0_CLKSEL_gm) | TC_CLKSEL_DIV1_gc;	// Source - undivided system clock, enables timer
	FINETIMER_ENABLE_IRQ		// Enable timer interrupt
}


/*! \brief CCP write helper function written in assembly.
 *
 *  This function is written in assembly because of the time critial
 *  operation of writing to the registers.
 *
 *  \param address A pointer to the address to write to.
 *  \param value   The value to put in to the register.
 */
void CCPWrite( volatile uint8_t * address, uint8_t value )
{
	volatile uint8_t * tmpAddr = address;
	//AVR_ENTER_CRITICAL_REGION( );
#ifdef RAMPZ
	RAMPZ = 0;
#endif
	asm volatile(
		"movw r30,  %0"	      "\n\t"
		"ldi  r16,  %2"	      "\n\t"
		"out   %3, r16"	      "\n\t"
		"st     Z,  %1"       "\n\t"
		:
		: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
		: "r16", "r30", "r31"
		);
	//AVR_LEAVE_CRITICAL_REGION( );
}



/***************************************************************************
* Set timer constant
* [19/02/2015]
***************************************************************************/
void timerset_fine(timerconst_t tconst, finetm_t *tptr) {
	uint16_t	tsec;


	FINETIMER_DISABLE_IRQ				// Disable timer interrupt
	finetm_t 	frozent = FineTimer;	// Need to freeze the timer as it can be updated by ISR any time;
	FINETIMER_ENABLE_IRQ				// Enable timer interrupt

	tsec = tconst / SEC_100USEC;
	tptr->sec = tsec + frozent.sec;
	tptr->usec100 = (tconst - (tsec * SEC_100USEC)) + frozent.usec100;
	if (tptr->usec100 >= SEC_100USEC) {
		tptr->usec100 -= SEC_100USEC;
		tptr->sec++;
	}
}


/***************************************************************************
* Check timer overflow
* [19/02/2015]
***************************************************************************/
uint8_t timercheck_fine(finetm_t *tptr) {

	FINETIMER_DISABLE_IRQ				// Disable timer interrupt
	finetm_t 	frozent = FineTimer;	// Need to freeze the timer as it can be updated by ISR any time;
	FINETIMER_ENABLE_IRQ				// Enable timer interrupt


	if (frozent.sec < tptr->sec) {
		return LE_FAIL;
	}
	else if (frozent.sec == tptr->sec) {
		if (frozent.usec100 < tptr->usec100) {
			return LE_FAIL;
		}
	}
	return LE_OK;
}


/***************************************************************************
* 100us timer ISR
* [19/02/2015]
***************************************************************************/
ISR(FINETIMER_ISR_VECTOR) {

	FineTimer.usec100++;
	if (FineTimer.usec100 >= SEC_100USEC) {
		FineTimer.usec100 = 0;
		FineTimer.sec++;
	}
}



/*! \brief Enable Watchdog and set prescaler. * *  This function enables the Watchdog and applies prescaler settings. *  The Watchdog will be reset automatically. * *  The function writes the correct signature to the Configuration *  Change Protection register before writing the CTRL register. Interrupts are *  automatically ignored during the change enable period. TThe function will *  wait for the watchdog to be synchronized to the other clock domains. before *  proceeding * *  \param  period  Watchdog Timer timeout period */void wdt_enable( WDT_PER_t period ){	uint8_t temp = WDT_ENABLE_bm | WDT_CEN_bm | period;	CCP = CCP_IOREG_gc;	WDT.CTRL = temp;
	/* Wait for WD to synchronize with new settings. */	while(WDT_IsSyncBusy()){}}/*! \brief This function selects the main system clock source.
	*
	*  Hardware will disregard any attempts to select a clock source that is not
	*  enabled or not stable. If the change fails, make sure the source is ready
	*  and running and try again.
	*
	*  \param  clockSource  Clock source to use as input for the system clock
	*                       prescaler block.
	*
	*  \return  Non-zero if change was successful.
	*/
uint8_t CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_t clockSource )
{
	uint8_t clkCtrl = ( CLK.CTRL & ~CLK_SCLKSEL_gm ) | clockSource;
	CCPWrite( &CLK.CTRL, clkCtrl );
	clkCtrl = ( CLK.CTRL & clockSource );
	return clkCtrl;
}#ifdef INTERNAL_32M_CLOCK
	/*! \brief This function enables automatic calibration of the selected internal
	 *         oscillator.
	 *
	 *  Either the internal 32kHz RC oscillator or an external 32kHz
	 *  crystal can be used as a calibration reference. The user must make sure
	 *  that the selected reference is ready and running.
	 *
	 *  \param  clkSource    Clock source to calibrate, either OSC_RC2MCREF_bm or
	 *                       OSC_RC32MCREF_bm.
	 *  \param  extReference True if external crystal should be used as reference.
	 */
	void CLKSYS_AutoCalibration_Enable( uint8_t clkSource, uint8_t extReference )
	{
		OSC.DFLLCTRL = ( OSC.DFLLCTRL & ~clkSource ) | ( extReference ? clkSource : 0 );
		if (clkSource == OSC_RC2MCREF_bm)
		{
			DFLLRC2M.CTRL |= DFLL_ENABLE_bm;
		}

		else if (clkSource == OSC_RC32MCREF_bm)
		{
			DFLLRC32M.CTRL |= DFLL_ENABLE_bm;
		}
	}

#endif

