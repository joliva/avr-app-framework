/******************************************************************************
  Filename:	timers.c
  Created:	03/16/10 (JLO)
  Purpose:	Timers driver

  Notes:
*******************************************************************************
  Copyright (c) 2010, John Oliva
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.

  * Neither the name of the copyright holders nor the names of
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <stdio.h>
#include "debug.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "common.h"
#include "timers.h"

// constants & definitions
typedef void (*func_ptr)(void);

#define TIMER_CLKSEL_MASK	0x07		// timer clock select bit mask

// type of interrupt handler to use (value may be SIGNAL or INTERRUPT)
#ifndef TIMER_INTERRUPT_HANDLER
	#define TIMER_INTERRUPT_HANDLER		SIGNAL
#endif

typedef enum {
	TIMER_OCRA=0, TIMER_OCRB, TIMER_OCRC
} timers_ocr_t;

// storage
volatile static func_ptr	timerint_func[TIMER_NUM_INTERRUPTS];

/* ------------------------------------------------------------------------- */
void timer_init(uint8_t timer_num, timers_clksel_t clk_select) {
	timers_intid_t int_id, int_lower=0, int_upper=0;

#if DEV_CLASS_2
	assert(timer_num < 2);
#elif DEV_CLASS_1
	assert(timer_num < 2 || ((timer_num > 2) && (timer_num < 6)));
#endif

	assert(clk_select <= TIMER_CLK_EXT_RISE);

	timer_set_clkselect(timer_num, clk_select);

	if (timer_num == 0) {
		int_lower=TIMER0_OVERFLOW_INT;  int_upper=TIMER0_OUTCOMPAREB_INT;
		TCCR0A = 0;					// compare output & waveform normal mode
		TCCR0B &= ~(1<<WGM02);		// waveform normal mode
		TCNT0 = 0;					// clear count
		OCR0A = 0;					// clear output compare reg
		OCR0B = 0;					// clear output compare reg
		TIMSK0 = 0;					// disable all interrupts
		TIFR0 = 0;					// clear all interrupt flags
	} else if (timer_num == 1) {
		int_lower=TIMER1_OVERFLOW_INT;  int_upper=TIMER1_INPUTCAPTURE_INT;
		TCCR1A = 0;					// compare output & waveform normal mode
		TCCR1B &= ~(1<<WGM12 | 1<<WGM13);	// waveform normal mode
		TCCR1C = 0;					// don't force input capture
		TCNT1 = 0;					// clear count
		OCR1A = 0;					// clear output compare reg
		OCR1B = 0;					// clear output compare reg
#if DEV_CLASS_1
		OCR1C = 0;					// clear output compare reg
#endif
		ICR1 = 0;					// clear input capture reg
		TIMSK1 = 0;					// disable all interrupts
		TIFR1 = 0;					// clear all interrupt flags
	}
#if DEV_CLASS_1
	else if (timer_num == 3) {
		int_lower=TIMER3_OVERFLOW_INT;  int_upper=TIMER3_INPUTCAPTURE_INT;
		TCCR3A = 0;					// compare output & waveform normal mode
		TCCR3B &= ~(1<<WGM32 | 1<<WGM33);	// waveform normal mode
		TCCR3C = 0;					// don't force input capture
		TCNT3 = 0;					// clear count
		OCR3A = 0;					// clear output compare reg
		OCR3B = 0;					// clear output compare reg
		OCR3C = 0;					// clear output compare reg
		ICR3 = 0;					// clear input capture reg
		TIMSK3 = 0;					// disable all interrupts
		TIFR3 = 0;					// clear all interrupt flags
	} else if (timer_num == 4) {
		int_lower=TIMER4_OVERFLOW_INT;  int_upper=TIMER4_INPUTCAPTURE_INT;
		TCCR4A = 0;					// compare output & waveform normal mode
		TCCR4B &= ~(1<<WGM42 | 1<<WGM43);	// waveform normal mode
		TCCR4C = 0;					// don't force input capture
		TCNT4 = 0;					// clear count
		OCR4A = 0;					// clear output compare reg
		OCR4B = 0;					// clear output compare reg
		OCR4C = 0;					// clear output compare reg
		ICR4 = 0;					// clear input capture reg
		TIMSK4 = 0;					// disable all interrupts
		TIFR4 = 0;					// clear all interrupt flags
	} else if (timer_num == 5) {
		int_lower=TIMER5_OVERFLOW_INT;  int_upper=TIMER5_INPUTCAPTURE_INT;
		TCCR5A = 0;					// compare output & waveform normal mode
		TCCR5B &= ~(1<<WGM52 | 1<<WGM53);	// waveform normal mode
		TCCR5C = 0;					// don't force input capture
		TCNT5 = 0;					// clear count
		OCR5A = 0;					// clear output compare reg
		OCR5B = 0;					// clear output compare reg
		OCR5C = 0;					// clear output compare reg
		ICR5 = 0;					// clear input capture reg
		TIMSK5 = 0;					// disable all interrupts
		TIFR5 = 0;					// clear all interrupt flags
	}
#endif

	// detach handlers from interrupts
	for(int_id=int_lower; int_id<=int_upper; int_id++) {
		timerint_detach(int_id);
	}
}

/* ------------------------------------------------------------------------- */
void timer2_init(timer2_clksel_t clk_select) {
	timers_intid_t int_id;
	timers_intid_t int_lower=TIMER2_OVERFLOW_INT;
	timers_intid_t int_upper=TIMER2_OUTCOMPAREB_INT;

	assert(clk_select <= TIMER2_CLK_DIV1024);

	timer2_set_clkselect(clk_select);

	TCCR2A = 0;					// compare output & waveform normal mode
	TCCR2B &= ~(1<<WGM22);		// waveform normal mode
	TCNT2 = 0;					// clear count
	OCR2A = 0;					// clear output compare reg
	OCR2B = 0;					// clear output compare reg
	TIMSK2 = 0;					// disable all interrupts
	TIFR2 = 0;					// clear all interrupt flags

	// detach handlers from interrupts
	for(int_id=int_lower; int_id<=int_upper; int_id++) {
		timerint_detach(int_id);
	}
}

/* ------------------------------------------------------------------------- */
void timerint_enable(uint8_t timer_num) {

#if DEV_CLASS_2
	assert(timer_num < 3);
#elif DEV_CLASS_1
	assert(timer_num < 6);
#endif

	if (timer_num == 0) {
		TIFR0 &= ~(1<<TOV0);		// clear overflow interrupt flag
		TCNT0 = 0;					// clear count
		TIMSK0 |= (1<<TOIE0);		// enable overflow interrupt
	} else if (timer_num == 1) {
		TIFR1 &= ~(1<<TOV1);		// clear overflow interrupt flag
		TCNT1 = 0;					// clear count
		TIMSK1 |= (1<<TOIE1);		// enable overflow interrupt
	} else if (timer_num == 2) {
		TIFR2 &= ~(1<<TOV2);		// clear overflow interrupt flag
		TCNT2 = 0;					// clear count
		TIMSK2 |= (1<<TOIE2);		// enable overflow interrupt
	}
#if DEV_CLASS_1
	else if (timer_num == 3) {
		TIFR3 &= ~(1<<TOV3);		// clear overflow interrupt flag
		TCNT3 = 0;					// clear count
		TIMSK3 |= (1<<TOIE3);		// enable overflow interrupt
	} else if (timer_num == 4) {
		TIFR4 &= ~(1<<TOV4);		// clear overflow interrupt flag
		TCNT4 = 0;					// clear count
		TIMSK4 |= (1<<TOIE4);		// enable overflow interrupt
	} else if (timer_num == 5) {
		TIFR5 &= ~(1<<TOV5);		// clear overflow interrupt flag
		TCNT5 = 0;					// clear count
		TIMSK5 |= (1<<TOIE5);		// enable overflow interrupt
	}
#endif
}

/* ------------------------------------------------------------------------- */
void timerint_disable(uint8_t timer_num) {

#if DEV_CLASS_2
	assert(timer_num < 3);
#elif DEV_CLASS_1
	assert(timer_num < 6);
#endif

	if (timer_num == 0) {
		TIMSK0 &= ~(1<<TOIE0);		// disable overflow interrupt
		TIFR0 &= ~(1<<TOV0);		// clear overflow interrupt flag
	} else if (timer_num == 1) {
		TIMSK1 &= ~(1<<TOIE1);		// disable overflow interrupt
		TIFR1 &= ~(1<<TOV1);		// clear overflow interrupt flag
	} else if (timer_num == 2) {
		TIMSK2 &= ~(1<<TOIE2);		// disable overflow interrupt
		TIFR2 &= ~(1<<TOV2);		// clear overflow interrupt flag
	}
#if DEV_CLASS_1
	else if (timer_num == 3) {
		TIMSK3 &= ~(1<<TOIE3);		// disable overflow interrupt
		TIFR3 &= ~(1<<TOV3);		// clear overflow interrupt flag
	} else if (timer_num == 4) {
		TIMSK4 &= ~(1<<TOIE4);		// disable overflow interrupt
		TIFR4 &= ~(1<<TOV4);		// clear overflow interrupt flag
	} else if (timer_num == 5) {
		TIMSK5 &= ~(1<<TOIE5);		// disable overflow interrupt
		TIFR5 &= ~(1<<TOV5);		// clear overflow interrupt flag
	}
#endif
}

/* ------------------------------------------------------------------------- */
void timer_set_clkselect(uint8_t timer_num, timers_clksel_t clk_select) {

#if DEV_CLASS_2
	assert(timer_num < 2);
#elif DEV_CLASS_1
	assert(timer_num < 2 || ((timer_num > 2) && (timer_num < 6)));
#endif

	assert(clk_select <= TIMER_CLK_EXT_RISE);

	if (timer_num == 0) {
		TCCR0B &= ~(TIMER_CLKSEL_MASK);		// clear bits
		TCCR0B |= clk_select;				// set clock select
	} else if (timer_num == 1) {
		TCCR1B &= ~(TIMER_CLKSEL_MASK);		// clear bits
		TCCR1B |= clk_select;				// set clock select
	}
#if DEV_CLASS_1
	else if (timer_num == 3) {
		TCCR3B &= ~(TIMER_CLKSEL_MASK);		// clear bits
		TCCR3B |= clk_select;				// set clock select
	} else if (timer_num == 4) {
		TCCR4B &= ~(TIMER_CLKSEL_MASK);		// clear bits
		TCCR4B |= clk_select;				// set clock select
	} else if (timer_num == 5) {
		TCCR5B &= ~(TIMER_CLKSEL_MASK);		// clear bits
		TCCR5B |= clk_select;				// set clock select
	}
#endif
}

/* ------------------------------------------------------------------------- */
void timer2_set_clkselect(timer2_clksel_t clk_select) {
	assert(clk_select <= TIMER2_CLK_DIV1024);

	TCCR2B &= ~(TIMER_CLKSEL_MASK);		// clear bits
	TCCR2B |= clk_select;				// set clock select
}

/* ------------------------------------------------------------------------- */
timers_clksel_t timer_get_clkselect(uint8_t timer_num) {
	timers_clksel_t clk_select=0;

#if DEV_CLASS_2
	assert(timer_num < 2);
#elif DEV_CLASS_1
	assert(timer_num < 2 || ((timer_num > 2) && (timer_num < 6)));
#endif

	if (timer_num == 0) {
		clk_select = TCCR0B & TIMER_CLKSEL_MASK;	// extract clock select
	} else if (timer_num == 1) {
		clk_select = TCCR1B & TIMER_CLKSEL_MASK;	// extract clock select
	}
#if DEV_CLASS_1
	else if (timer_num == 3) {
		clk_select = TCCR3B & TIMER_CLKSEL_MASK;	// extract clock select
	} else if (timer_num == 4) {
		clk_select = TCCR4B & TIMER_CLKSEL_MASK;	// extract clock select
	} else if (timer_num == 5) {
		clk_select = TCCR5B & TIMER_CLKSEL_MASK;	// extract clock select
	}
#endif

	return clk_select;
}

/* ------------------------------------------------------------------------- */
timer2_clksel_t timer2_get_clkselect() {
	timers_clksel_t clk_select = TCCR2B & TIMER_CLKSEL_MASK;	// extract clock select

	return clk_select;
}

/* ------------------------------------------------------------------------- */
void timerint_attach(timers_intid_t int_id, void (*handler)(void)) {
	assert (int_id < TIMER_NUM_INTERRUPTS);
	assert (handler != NULL);

	timerint_func[int_id] = handler;
}

/* ------------------------------------------------------------------------- */
void timerint_detach(timers_intid_t int_id) {
	assert (int_id < TIMER_NUM_INTERRUPTS);

	timerint_func[int_id] = NULL;
}

/* ------------------------------------------------------------------------- */
void timer_pwm_init(uint8_t timer_num, uint8_t num_bits) {
	assert(timer_num < 6);
	assert(false);		// stub implementation
}

/* ------------------------------------------------------------------------- */
void timer_pwm_init_icr(uint8_t timer_num, uint16_t top_count) {
	assert(timer_num < 6);
	assert(false);		// stub implementation
}

/* ------------------------------------------------------------------------- */
void timer_pwm_off(uint8_t timer_num, timers_ocr_t ocr) {
	assert(timer_num < 6);
	assert(ocr <= TIMER_OCRC);
	assert(false);		// stub implementation
}

/* ------------------------------------------------------------------------- */
void timer_pwm_on(uint8_t timer_num, timers_ocr_t ocr) {
	assert(timer_num < 6);
	assert(ocr <= TIMER_OCRC);
	assert(false);		// stub implementation
}

/* ------------------------------------------------------------------------- */
void timer_pwm_set_dutycycle(uint8_t timer_num, timers_ocr_t ocr, uint16_t duty_cycle) {
	assert(timer_num < 6);
	assert(ocr <= TIMER_OCRC);
	assert(false);		// stub implementation
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer0 overflow interrupt
TIMER_INTERRUPT_HANDLER(TIMER0_OVF_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER0_OVERFLOW_INT] != NULL) {
		timerint_func[TIMER0_OVERFLOW_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer1 overflow interrupt
TIMER_INTERRUPT_HANDLER(TIMER1_OVF_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER1_OVERFLOW_INT] != NULL) {
		timerint_func[TIMER1_OVERFLOW_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer2 overflow interrupt
TIMER_INTERRUPT_HANDLER(TIMER2_OVF_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER2_OVERFLOW_INT] != NULL) {
		timerint_func[TIMER2_OVERFLOW_INT]();
	}
}

#if DEV_CLASS_1

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer3 overflow interrupt
TIMER_INTERRUPT_HANDLER(TIMER3_OVF_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER3_OVERFLOW_INT] != NULL) {
		timerint_func[TIMER3_OVERFLOW_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer4 overflow interrupt
TIMER_INTERRUPT_HANDLER(TIMER4_OVF_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER4_OVERFLOW_INT] != NULL) {
		timerint_func[TIMER4_OVERFLOW_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer5 overflow interrupt
TIMER_INTERRUPT_HANDLER(TIMER5_OVF_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER5_OVERFLOW_INT] != NULL) {
		timerint_func[TIMER5_OVERFLOW_INT]();
	}
}

#endif

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer0 output compare A match interrupt
TIMER_INTERRUPT_HANDLER(TIMER0_COMPA_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER0_OUTCOMPAREA_INT] != NULL) {
		timerint_func[TIMER0_OUTCOMPAREA_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer1 output compare A match interrupt
TIMER_INTERRUPT_HANDLER(TIMER1_COMPA_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER1_OUTCOMPAREA_INT] != NULL) {
		timerint_func[TIMER1_OUTCOMPAREA_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer2 output compare A match interrupt
TIMER_INTERRUPT_HANDLER(TIMER2_COMPA_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER2_OUTCOMPAREA_INT] != NULL) {
		timerint_func[TIMER2_OUTCOMPAREA_INT]();
	}
}

#if DEV_CLASS_1

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer3 output compare A match interrupt
TIMER_INTERRUPT_HANDLER(TIMER3_COMPA_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER3_OUTCOMPAREA_INT] != NULL) {
		timerint_func[TIMER3_OUTCOMPAREA_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer4 output compare A match interrupt
TIMER_INTERRUPT_HANDLER(TIMER4_COMPA_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER4_OUTCOMPAREA_INT] != NULL) {
		timerint_func[TIMER4_OUTCOMPAREA_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer5 output compare A match interrupt
TIMER_INTERRUPT_HANDLER(TIMER5_COMPA_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER5_OUTCOMPAREA_INT] != NULL) {
		timerint_func[TIMER5_OUTCOMPAREA_INT]();
	}
}

#endif

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer0 output compare B match interrupt
TIMER_INTERRUPT_HANDLER(TIMER0_COMPB_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER0_OUTCOMPAREB_INT] != NULL) {
		timerint_func[TIMER0_OUTCOMPAREB_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer1 output compare B match interrupt
TIMER_INTERRUPT_HANDLER(TIMER1_COMPB_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER1_OUTCOMPAREB_INT] != NULL) {
		timerint_func[TIMER1_OUTCOMPAREB_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer2 output compare B match interrupt
TIMER_INTERRUPT_HANDLER(TIMER2_COMPB_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER2_OUTCOMPAREB_INT] != NULL) {
		timerint_func[TIMER2_OUTCOMPAREB_INT]();
	}
}

#if DEV_CLASS_1

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer3 output compare B match interrupt
TIMER_INTERRUPT_HANDLER(TIMER3_COMPB_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER3_OUTCOMPAREB_INT] != NULL) {
		timerint_func[TIMER3_OUTCOMPAREB_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer4 output compare B match interrupt
TIMER_INTERRUPT_HANDLER(TIMER4_COMPB_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER4_OUTCOMPAREB_INT] != NULL) {
		timerint_func[TIMER4_OUTCOMPAREB_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer5 output compare B match interrupt
TIMER_INTERRUPT_HANDLER(TIMER5_COMPB_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER5_OUTCOMPAREB_INT] != NULL) {
		timerint_func[TIMER5_OUTCOMPAREB_INT]();
	}
}

#endif

#if DEV_CLASS_1

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer1 output compare C match interrupt
TIMER_INTERRUPT_HANDLER(TIMER1_COMPC_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER1_OUTCOMPAREC_INT] != NULL) {
		timerint_func[TIMER1_OUTCOMPAREC_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer3 output compare C match interrupt
TIMER_INTERRUPT_HANDLER(TIMER3_COMPC_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER3_OUTCOMPAREC_INT] != NULL) {
		timerint_func[TIMER3_OUTCOMPAREC_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer4 output compare C match interrupt
TIMER_INTERRUPT_HANDLER(TIMER4_COMPC_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER4_OUTCOMPAREC_INT] != NULL) {
		timerint_func[TIMER4_OUTCOMPAREC_INT]();
	}
}

/* ------------------------------------------------------------------------- */
// interrupt handler for Timer5 output compare C match interrupt
TIMER_INTERRUPT_HANDLER(TIMER5_COMPC_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(timerint_func[TIMER5_OUTCOMPAREC_INT] != NULL) {
		timerint_func[TIMER5_OUTCOMPAREC_INT]();
	}
}

#endif
