/*
 ============================================================================
 Name        : spi.c
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : Atmel SPI module

  Change log :

  *********V1.00 01/06/2018**************
  Initial revision

 ============================================================================
 */

//#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>


#include "leiodcat.h"
#include "board.h"
#include "irq.h"


#ifdef GLOBAL_DEBUG
//#define DEBUG_NOUARTTX
#endif	// GLOBAL_DEBUG


// SPI runtime flags
#define SPIRTF_LOWBYTE				0x80			// Send/Receive lowbyte
#define SPIRTM_GROUP				0x03			// AI input group mask

// SPI constants
#define SPI_IDLETIME				1000			// Idle time in between ADC requests (default 0.1sec)

// ADC configuration constants
//#define ADC_MUX						0x4000			// Input multiplexer configuration (AINn is GND)
#define ADC_MUX						0x0000			// Input multiplexer configuration (AINn is AIN1 or AIN3)
#define ADCMUX_CHAN					0x3000			// ADC inputs (AIN0/AIN1 and AIN2/AIN3)
//#define ADCMUX_CURRENT				0x1000			// ADC Current inputs (AIN1 and AIN3) enabled
//#define ADC_FSR						0x0400			// Amplifier gain = ±2.048V
#define ADC_FSR						0x0600			// Amplifier gain = ±1.024V
//#define ADC_DR						0x0080			// Data rate = 128 SPS
#define ADC_DR						0x0000			// Data rate = 8 SPS
#define ADC_PULL_UP_EN				0x0009			// DOUT/DRDY pin pull-up enabled and Reserved bit set
#define ADC_NOP						0x0002			// Configuration register data valid


// SPI settings
#define SPI_CTRL					SPIF_CTRL		// Control register
#define SPI_DATA					SPIF_DATA		// Data register
#define SPI_INTCTRL					SPIF_INTCTRL	// Interrupt control register
#define SPI_ISR_VECTOR				SPIF_INT_vect	// SPI vector


// SPI device communication states
/*typedef enum {
	spist_high					= 0,				// Ready to receive first byte
	spist_data,										// Ready to receive first byte
} LEOPACK spistate_e;*/


struct spiadc_s {
	finetm_t				timer;					// State timer
	uint16_t				config;					// ADC config register
	uint16_t				rddata;					// Data read from ADC
	uint8_t					rflags;					// Operation state
} spidriver;


// External functions
//static void spi_isr(void);

// Timer macros
#define SPI_SET_MTIMER(mtconst) timerset_fine(mtconst, &spidriver.timer);
#define SPI_CHECK_MTIMER timercheck_fine(&spidriver.timer)


// PIN manipulation macros
#define SPI_CS_ACTIVATE		MCUP_ADCSPI.OUTCLR = 0x10 >> (spidriver.rflags & SPIRTM_GROUP);
#define SPI_CS_RELEASE		MCUP_ADCSPI.OUTSET = 0x10 >> (spidriver.rflags & SPIRTM_GROUP);


/*
 * Send SPI data
 * [02/06/2018]
 */
static void spi_send(void) {

	if (spidriver.rflags & SPIRTF_LOWBYTE) {
		spidriver.rflags &= ~SPIRTF_LOWBYTE;
		spidriver.rddata = (SPI_DATA << 8);
		SPI_DATA = (spidriver.config & 0xff);
	}
	else {
		spidriver.rflags |= SPIRTF_LOWBYTE;
		SPI_DATA = (spidriver.config >> 8) & 0xff;
	}
}


/*
 * SPI interrupt vector for communication to external ADC
 * [06/07/2019]
 */
static inline void spi_isr(void) {

	if (spidriver.rflags & SPIRTF_LOWBYTE) {
		spi_send();
	}
	else {
		spidriver.rddata |= SPI_DATA;
		SPI_CS_RELEASE

		uint8_t offs = (spidriver.rflags & SPIRTM_GROUP) << 1;
		if (!(spidriver.config & ADCMUX_CHAN))
			offs |= 1;

		boardio.aiptr->uval[offs] = spidriver.rddata;

		if (spidriver.rflags == SPIRTM_GROUP) {
			spidriver.rflags &= ~SPIRTM_GROUP;
			spidriver.config ^= ADCMUX_CHAN;
		}
		else
			spidriver.rflags++;
	}
}


/*
 * SPI interrupt
 * [01/06/2018]
 */
ISR(SPI_ISR_VECTOR) {
	spi_isr();
}


/*
 * Main process
 * [06/07/2019]
 */
void spidrv_mainproc(void) {

	if (SPI_CHECK_MTIMER == LE_OK) {	// Check delay
		SPI_CS_ACTIVATE
		SPI_SET_MTIMER(SPI_IDLETIME)
		spi_send();
	}
}


/*
 * Initialize SPI interface and pins
 * [02/06/2018]
 */
void spi_init(uint8_t aicount) {

	boardio.aiptr = calloc(1, sizeof(boardAI_t));
	boardio.aiptr->count = aicount;
	boardio.aiptr->uval = calloc(aicount, sizeof(uint16_t));
	boardio.aiptr->mode = calloc(aicount, sizeof(modbusaimode_e));

	SPI_SET_MTIMER(SPI_IDLETIME)

	MCUP_ADCSPI.OUT = 0x7E;			// MOSI=1; MISO=1; SS0=1; SS1=1; SS2=1; SS3=1
	MCUP_ADCSPI.DIR = 0xBE;			// Pins SCK[7]; MOSI[5]; SS0[4]; SS1[3]; SS2[2] and SS3[1] are outputs
	SPI_CTRL = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE0_bm | SPI_PRESCALER1_bm;	// DORD=0; MODE=1
	SPI_INTCTRL = SPI_INTLVL;		// Enable low level interrupt

	spidriver.config = ADC_MUX | ADC_FSR | ADC_DR | ADC_PULL_UP_EN | ADC_NOP;
}
