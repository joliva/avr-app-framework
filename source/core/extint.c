/******************************************************************************
  Filename:	extint.c
  Created:	03/10/10 (JLO)
  Purpose:	External interrupts driver

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
#include "extint.h"


// constants & definitions
typedef void (*func_ptr)(void);

// forward declarations
static void extint_clear(extint_id_t int_id);

// storage
volatile static func_ptr	extint_func[EXTINT_NUM_INTERRUPTS];
static extint_type_t		extint_type[EXTINT_NUM_INTERRUPTS];
bool 						initialized = false;

/* ------------------------------------------------------------------------- */
// initialize external interrupts
void extint_init(void) {
	if (initialized == false) {
		uint8_t int_id;

		for(int_id=0; int_id<EXTINT_NUM_INTERRUPTS; int_id++) {
			extint_disable((extint_id_t)int_id);	// disable interrupt
			extint_clear((extint_id_t)int_id);		// clear interrupt
			extint_detach((extint_id_t)int_id);		// detach user function
		}

		initialized = true;
	}
}

/* ------------------------------------------------------------------------- */
void extint_enable(extint_id_t int_id) {
	assert(int_id < EXTINT_NUM_INTERRUPTS);

	extint_clear(int_id);

#if DEV_CLASS_1
	if ((EXTINT0 <= int_id) && (int_id <= EXTINT7)) {
#elif DEV_CLASS_2
	if ((EXTINT0 <= int_id) && (int_id <= EXTINT1)) {
#endif
		EIMSK |= (1<<int_id);
	} else if ((XPCINT0 <= int_id) && (int_id <= XPCINT7)) {
		PCMSK0 |= (1<<(int_id-XPCINT0));
		if (PCMSK0 != 0) {
			PCICR |= (1<<PCIE0);
		}
	} else if ((XPCINT8 <= int_id) && (int_id <= XPCINT15)) {
		PCMSK1 |= (1<<(int_id-XPCINT8));
		if (PCMSK1 != 0) {
			PCICR |= (1<<PCIE1);
		}
	} else if ((XPCINT16 <= int_id) && (int_id <= XPCINT23)) {
		PCMSK2 |= (1<<(int_id-XPCINT16));
		if (PCMSK2 != 0) {
			PCICR |= (1<<PCIE2);
		}
	}
}

/* ------------------------------------------------------------------------- */
void extint_disable(extint_id_t int_id) {
	assert(int_id < EXTINT_NUM_INTERRUPTS);

	extint_clear(int_id);

#if DEV_CLASS_1
	if ((EXTINT0 <= int_id) && (int_id <= EXTINT7)) {
#elif DEV_CLASS_2
	if ((EXTINT0 <= int_id) && (int_id <= EXTINT1)) {
#endif
		EIMSK &= ~(1<<int_id);
	} else if ((XPCINT0 <= int_id) && (int_id <= XPCINT7)) {
		PCMSK0 &= ~(1<<(int_id-XPCINT0));
		if (PCMSK0 == 0) {
			PCICR &= ~(1<<PCIE0);
		}
	} else if ((XPCINT8 <= int_id) && (int_id <= XPCINT15)) {
		PCMSK1 &= ~(1<<(int_id-XPCINT8));
		if (PCMSK1 == 0) {
			PCICR &= ~(1<<PCIE1);
		}
	} else if ((XPCINT16 <= int_id) && (int_id <= XPCINT23)) {
		PCMSK2 &= ~(1<<(int_id-XPCINT16));
		if (PCMSK2 == 0) {
			PCICR &= ~(1<<PCIE2);
		}
	}
}

/* ------------------------------------------------------------------------- */
static void extint_clear(extint_id_t int_id) {
	assert(int_id < EXTINT_NUM_INTERRUPTS);

#if DEV_CLASS_1
	if ((EXTINT0 <= int_id) && (int_id <= EXTINT7)) {
#elif DEV_CLASS_2
	if ((EXTINT0 <= int_id) && (int_id <= EXTINT1)) {
#endif
		EIFR |= (1<<int_id);		// clear by writing 1 to bit
	} else if ((XPCINT0 <= int_id) && (int_id <= XPCINT7)) {
		PCIFR |= (1<<PCIF0);		// clear by writing 1 to bit
	} else if ((XPCINT8 <= int_id) && (int_id <= XPCINT15)) {
		PCIFR |= (1<<PCIF1);		// clear by writing 1 to bit
	} else if ((XPCINT16 <= int_id) && (int_id <= XPCINT23)) {
		PCIFR |= (1<<PCIF2);		// clear by writing 1 to bit
	}
}

/* ------------------------------------------------------------------------- */
// attach handler to an external interrupt
void extint_attach(extint_id_t int_id, extint_type_t int_type, void (*handler)(void)) {
	uint8_t	value;

	assert(int_id < EXTINT_NUM_INTERRUPTS);
	assert(int_type < EXTINT_NUM_TYPES);
	assert(handler != NULL);

	extint_func[int_id] = handler;

	// configure interrupt condition

	extint_disable(int_id);	// disable interrupt

	extint_type[int_id] = int_type;

#if DEV_CLASS_1
	if ((EXTINT0 <= int_id) && (int_id <= EXTINT3)) {
		value = EICRA & ~(0x3 << (int_id<<1));			// mask off condition
		EICRA = value | (int_type << (int_id<<1));	// set interrupt condition
	} else if ((EXTINT4 <= int_id) && (int_id <= EXTINT7)) {
		value = EICRB & ~(0x3 << ((int_id-4)<<1));		// mask off condition
		EICRB = value | (int_type << ((int_id-4)<<1));	// set interrupt condition
	}
#elif DEV_CLASS_2
	if ((EXTINT0 <= int_id) && (int_id <= EXTINT1)) {
		value = EICRA & ~(0x3 << (int_id<<1));			// mask off condition
		EICRA = value | (int_type << (int_id<<1));	// set interrupt condition
	}
#endif

	extint_clear(int_id);		// clear interrupt
	extint_enable(int_id);		// enable interrupt
}

/* ------------------------------------------------------------------------- */
// detach handler from an external interrupt
void extint_detach(extint_id_t int_id) {
	assert(int_id < EXTINT_NUM_INTERRUPTS);

	extint_func[int_id] = NULL;
}

#if DEV_CLASS_1 || DEV_CLASS_2
/* ------------------------------------------------------------------------- */
EXTINT_INTERRUPT_HANDLER(INT0_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(extint_func[EXTINT0] != NULL) {
		extint_func[EXTINT0]();
	}
}

/* ------------------------------------------------------------------------- */
EXTINT_INTERRUPT_HANDLER(INT1_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if(extint_func[EXTINT1] != NULL) {
		extint_func[EXTINT1]();
	}
}
#endif

#if DEV_CLASS_1
/* ------------------------------------------------------------------------- */
EXTINT_INTERRUPT_HANDLER(INT2_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if (extint_func[EXTINT2] != NULL) {
		extint_func[EXTINT2]();
	}
}

/* ------------------------------------------------------------------------- */
EXTINT_INTERRUPT_HANDLER(INT3_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if (extint_func[EXTINT3] != NULL) {
		extint_func[EXTINT3]();
	}
}

/* ------------------------------------------------------------------------- */
EXTINT_INTERRUPT_HANDLER(INT4_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if (extint_func[EXTINT4] != NULL) {
		extint_func[EXTINT4]();
	}
}

/* ------------------------------------------------------------------------- */
EXTINT_INTERRUPT_HANDLER(INT5_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if (extint_func[EXTINT5] != NULL) {
		extint_func[EXTINT5]();
	}
}

/* ------------------------------------------------------------------------- */
EXTINT_INTERRUPT_HANDLER(INT6_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if (extint_func[EXTINT6] != NULL) {
		extint_func[EXTINT6]();
	}
}

/* ------------------------------------------------------------------------- */
EXTINT_INTERRUPT_HANDLER(INT7_vect) {
	// interrupt flag is automatically cleared by handler
	 
	// if a handler is attached, execute it
	if (extint_func[EXTINT7] != NULL) {
		extint_func[EXTINT7]();
	}
}

/* ------------------------------------------------------------------------- */
EXTINT_INTERRUPT_HANDLER(PCINT0_vect) {
	uint8_t	value, index;

	// interrupt flag is automatically cleared by handler
	
	value = PINB;		// get current state of pins

	// check if pin change is enabled and interrupt type & pin state indicate
	// a valid interrupt condition. if a handler is attached, execute it

	for (index=0; index<8; index++) {
		if ((PCMSK0 & (1<<index)) != 0) {
			if (	(extint_type[index+XPCINT0] == EDGE_ANY) ||
					((extint_type[index+XPCINT0] == EDGE_FALLING) && ((value & (1<<index)) == 0)) ||	
					((extint_type[index+XPCINT0] == EDGE_RISING) && ((value & (1<<index)) != 0)) ) {

				if (extint_func[index+XPCINT0] != NULL) {
					extint_func[index+XPCINT0]();
				}
			}	
		}
	}
}

/* ------------------------------------------------------------------------- */
EXTINT_INTERRUPT_HANDLER(PCINT1_vect) {
	uint8_t	value, index;

	// interrupt flag is automatically cleared by handler
	
	value = PINJ;		// get current state of pins
	value <<= 1;		// NB: there is no PCINT8

	// check if pin change is enabled and interrupt type & pin state indicate
	// a valid interrupt condition. if a handler is attached, execute it

	for (index=1; index<8; index++) {
		if ((PCMSK1 & (1<<index)) != 0) {
			if (	(extint_type[index+XPCINT8] == EDGE_ANY) ||
					((extint_type[index+XPCINT8] == EDGE_FALLING) && ((value & (1<<index)) == 0)) ||	
					((extint_type[index+XPCINT8] == EDGE_RISING) && ((value & (1<<index)) != 0)) ) {

				if (extint_func[index+XPCINT8] != NULL) {
					extint_func[index+XPCINT8]();
				}
			}	
		}
	}
}

/* ------------------------------------------------------------------------- */
EXTINT_INTERRUPT_HANDLER(PCINT2_vect) {
	uint8_t	value, index;

	// interrupt flag is automatically cleared by handler
	
	value = PINK;		// get current state of pins

	// check if pin change is enabled and interrupt type & pin state indicate
	// a valid interrupt condition. if a handler is attached, execute it

	for (index=0; index<8; index++) {
		if ((PCMSK2 & (1<<index)) != 0) {
			if (	(extint_type[index+XPCINT16] == EDGE_ANY) ||
					((extint_type[index+XPCINT16] == EDGE_FALLING) && ((value & (1<<index)) == 0)) ||	
					((extint_type[index+XPCINT16] == EDGE_RISING) && ((value & (1<<index)) != 0)) ) {

				if (extint_func[index+XPCINT16] != NULL) {
					extint_func[index+XPCINT16]();
				}
			}	
		}
	}
}

#endif



