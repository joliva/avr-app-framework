/******************************************************************************
  Filename:	timers.h
  Created:	03/16/10 (JLO)
  Purpose:	

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
******************************************************************************
	Timer/clock select values and timer overflow rates

	8-bit overflow  = rate at which the timer overflows 8 bits (or reaches 256)
	16-bit overflow = rate at which the timer overflows 16 bits (or reaches 65536)

	overflows can be used to generate periodic interrupts

	Timers 0,2 (8-bit)
	Timers 1,3,4,5 (16-bit)
******************************************************************************
	Clock Modes:

	STOP:								Timers 0,1,2,3,4,5
	CLOCK:								Timers 0,1,2,3,4,5
	CLOCK/8:							Timers 0,1,2,3,4,5
	CLOCK/32:							Timers 2
	CLOCK/64:							Timers 0,1,2,3,4,5
	CLOCK/128:							Timers 2
	CLOCK/256:							Timers 0,1,2,3,4,5
	CLOCK/1024:							Timers 0,1,2,3,4,5
	External Clock (falling edge):		Timers 0,1,3,4,5
	External Clock (rising edge):		Timers 0,1,3,4,5
******************************************************************************
	CLOCK:				8-bit overflow= CPU_CLK_FREQ/256 Hz			16-bit overflow= CPU_CLK_FREQ/65536 Hz
	CLOCK/8				8-bit overflow= CPU_CLK_FREQ/(8*256) Hz		16-bit overflow= CPU_CLK_FREQ/(8*65536) Hz
	CLOCK/32			8-bit overflow= CPU_CLK_FREQ/(32*256) Hz	16-bit overflow= CPU_CLK_FREQ/(32*65536) Hz
	CLOCK/64			8-bit overflow= CPU_CLK_FREQ/(64*256) Hz	16-bit overflow= CPU_CLK_FREQ/(64*65536) Hz
	CLOCK/128			8-bit overflow= CPU_CLK_FREQ/(128*256) Hz	16-bit overflow= CPU_CLK_FREQ/(128*65536) Hz
	CLOCK/256			8-bit overflow= CPU_CLK_FREQ/(256*256) Hz	16-bit overflow= CPU_CLK_FREQ/(256*65536) Hz
	CLOCK/1024			8-bit overflow= CPU_CLK_FREQ/(1024*256) Hz	16-bit overflow= CPU_CLK_FREQ/(1024*65536) Hz
	External Clock:		8-bit overflow= EXT_CLK_FREQ/256 Hz			16-bit overflow= EXT_CLK_FREQ/65536 Hz
******************************************************************************/

#ifndef _TIMERS_H
#define _TIMERS_H

#include "common.h"

// constants & definitions

// Timers 0,1,3,4,5
typedef enum {
	TIMER_CLK_STOP=0, TIMER_CLK_DIV1=1, TIMER_CLK_DIV8=2, TIMER_CLK_DIV64=3,
	TIMER_CLK_DIV256=4, TIMER_CLK_DIV1024=5, TIMER_CLK_EXT_FALL=6, TIMER_CLK_EXT_RISE=7
} timers_clksel_t;

// Timer 2
typedef enum {
	TIMER2_CLK_STOP=0, TIMER2_CLK_DIV1=1, TIMER2_CLK_DIV8=2, TIMER2_CLK_DIV32=3,
	TIMER2_CLK_DIV64=4, TIMER2_CLK_DIV128=5, TIMER2_CLK_DIV256=6, TIMER2_CLK_DIV1024=7
} timer2_clksel_t;

// interrupt definitions for attaching user functions to timer interrupts
typedef enum {
	TIMER0_OVERFLOW_INT=0, TIMER0_OUTCOMPAREA_INT, TIMER0_OUTCOMPAREB_INT,
	TIMER1_OVERFLOW_INT, TIMER1_OUTCOMPAREA_INT, TIMER1_OUTCOMPAREB_INT, 
	TIMER1_OUTCOMPAREC_INT, TIMER1_INPUTCAPTURE_INT,
	TIMER2_OVERFLOW_INT, TIMER2_OUTCOMPAREA_INT, TIMER2_OUTCOMPAREB_INT, 
	TIMER2_OUTCOMPAREC_INT, TIMER2_INPUTCAPTURE_INT,
#if DEV_CLASS_1
	TIMER3_OVERFLOW_INT, TIMER3_OUTCOMPAREA_INT, TIMER3_OUTCOMPAREB_INT, 
	TIMER3_OUTCOMPAREC_INT, TIMER3_INPUTCAPTURE_INT,
	TIMER4_OVERFLOW_INT, TIMER4_OUTCOMPAREA_INT, TIMER4_OUTCOMPAREB_INT, 
	TIMER4_OUTCOMPAREC_INT, TIMER4_INPUTCAPTURE_INT,
	TIMER5_OVERFLOW_INT, TIMER5_OUTCOMPAREA_INT, TIMER5_OUTCOMPAREB_INT, 
	TIMER5_OUTCOMPAREC_INT, TIMER5_INPUTCAPTURE_INT,
#endif
	TIMER_NUM_INTERRUPTS
} timers_intid_t;

// prototypes

void			timer_init(uint8_t timer_num, timers_clksel_t clk_select);
void			timer_set_clkselect(uint8_t timer_num, timers_clksel_t clk_select);
timers_clksel_t	timer_get_clkselect(uint8_t timer_num);

void			timer2_init(timer2_clksel_t clk_select);
void			timer2_set_clkselect(timer2_clksel_t clk_select);
timer2_clksel_t	timer2_get_clkselect(void);

void			timerint_enable(uint8_t timer_num);
void			timerint_disable(uint8_t timer_num);
void			timerint_attach(timers_intid_t int_id, void (*handler)(void));
void			timerint_detach(timers_intid_t int_id);

#endif
