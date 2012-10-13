/******************************************************************************
  Filename:	swtimer.c
  Created:	03/13/10 (JLO)
  Purpose:	SW timers model

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

#include <avr/interrupt.h>
#include "common.h"
#include "debug.h"
#include "timers.h"
#include "swtimer.h"

// constants & definitions

struct m_swtimer {
	uint32_t			start_count;
	uint32_t			count;
	swtimer_mode_t		mode;
	struct m_swtimer	*prev;
	struct m_swtimer	*next;
	void 				(*handler)(void);
};

typedef struct m_swtimer m_swtimer_t;

#define SWTIMER_TICK_PERIOD_X1024		(1024*256000UL/(F_CPU/256))		// milliseconds x 1024  -- need to maintain precision

// storage
m_swtimer_t		*swtimer_list;		// linked-list of timers
bool			initialized = false;

// forward declarations
static void swtimer_handler(void);
static inline void swtimer_disable_timer0(void);
static inline void swtimer_enable_timer0(void);

/* ------------------------------------------------------------------------- */
void swtimer_init(void) {
	if (initialized == false) {
		// Timer0 will be used to update software downcounters
		timer_init(0, TIMER_CLK_DIV256);		// set timer0 for F_CPU/(256*256) Hz interrupt rate

		timerint_attach(TIMER0_OVERFLOW_INT, swtimer_handler);
		timerint_enable(0);

		swtimer_list=0;
		initialized = true;
	}
}

/* ------------------------------------------------------------------------- */
swtimer_t* swtimer_create(void (*handler)(void)) {
	// create the timer
	m_swtimer_t *new_timer = (m_swtimer_t*)malloc(sizeof(m_swtimer_t));

	new_timer->start_count = new_timer->count = 0;
	new_timer->mode = SWTIMER_MODE_OFF;
	new_timer->handler = handler;
	new_timer->next = 0;			// new end of timer list

	// find end of timer list and append new timer
	if (swtimer_list != 0) {
		m_swtimer_t *cur = swtimer_list;
		while (cur->next != 0) {
			cur = cur->next;
		}

		swtimer_disable_timer0();		// prevent interrupt access to list during update

		cur->next = new_timer;
		new_timer->prev = cur;

		swtimer_enable_timer0();
	} else {
		new_timer->prev = 0;
		swtimer_list = new_timer;
	}

	return (swtimer_t*)new_timer;
}

/* ------------------------------------------------------------------------- */
void swtimer_destroy(swtimer_t *swtimer) {
	assert(swtimer != NULL);

	// cast from opaque pointer to m_swtimer_t
	m_swtimer_t *m_swtimer = (m_swtimer_t*)swtimer;

	// search the timer list for specified timer, deallocate storage and remove from list
	if (swtimer_list != 0) {
		m_swtimer_t *cur = swtimer_list;

		while (cur != m_swtimer) {
			cur = cur->next;
		}

		if (cur == m_swtimer) {
			swtimer_disable_timer0();		// prevent interrupt access to list during update

			if (m_swtimer->prev == 0) {
				// m_swtimer is start of list
				if (m_swtimer->next != 0) {
					m_swtimer->next->prev = 0;
					swtimer_list = m_swtimer->next;
				} else {
					// m_swtimer is only one on list
					swtimer_list = 0;
				}
			} else if (m_swtimer->next == 0) {
				// m_swtimer is end of list
				if (m_swtimer->prev != 0) {
					m_swtimer->prev->next = 0;
				}
			} else {
				// m_swtimer is middle of list
				m_swtimer->next->prev = m_swtimer->prev;
				m_swtimer->prev->next = m_swtimer->next;
			}

			free (swtimer);
			swtimer_enable_timer0();
		}
	}
}

/* ------------------------------------------------------------------------- */
uint32_t swtimer_start(swtimer_t *swtimer, swtimer_mode_t mode, uint32_t time_msec) {
	assert(swtimer != NULL);
	assert(mode <= SWTIMER_MODE_REPEATING);

	uint32_t actual_time_msec= 0;

	// cast from opaque pointer to m_swtimer_t
	m_swtimer_t *m_swtimer = (m_swtimer_t*)swtimer;

	// search the timer list for specified timer and set timeout
	if (swtimer_list != 0) {
		m_swtimer_t *cur = swtimer_list;

		while (cur != m_swtimer) {
			cur = cur->next;
		}

		if (cur == m_swtimer) {
			swtimer_disable_timer0();		// prevent interrupt access to list during update

			// Note: little funky logic to round the count

			uint32_t count_10 = ((10 << 10) * time_msec)/SWTIMER_TICK_PERIOD_X1024;
			uint32_t count = (time_msec << 10)/SWTIMER_TICK_PERIOD_X1024;

			m_swtimer->mode = mode;
			m_swtimer->start_count = m_swtimer->count = ((count_10 % 10) < 5) ? count : count+1;
			actual_time_msec = (m_swtimer->count * SWTIMER_TICK_PERIOD_X1024)/1024;

			swtimer_enable_timer0();
		}
	}

	return actual_time_msec;
}

/* ------------------------------------------------------------------------- */
void swtimer_stop(swtimer_t *swtimer) {
	assert(swtimer != NULL);

	// cast from opaque pointer to m_swtimer_t
	m_swtimer_t *m_swtimer = (m_swtimer_t*)swtimer;

	// search the timer list for specified timer and clear timeout
	if (swtimer_list != 0) {
		m_swtimer_t *cur = swtimer_list;

		while (cur != m_swtimer) {
			cur = cur->next;
		}

		if (cur == m_swtimer) {
			swtimer_disable_timer0();		// prevent interrupt access to list during update
			m_swtimer->mode = SWTIMER_MODE_OFF;
			m_swtimer->start_count = m_swtimer->count = 0;
			swtimer_enable_timer0();
		}
	}
}

/* ------------------------------------------------------------------------- */
void swtimer_attach(swtimer_t *swtimer, void (*handler)(void)) {
	assert(swtimer != NULL);

	// cast from opaque pointer to m_swtimer_t
	m_swtimer_t *m_swtimer = (m_swtimer_t*)swtimer;

	m_swtimer->handler = handler;
}

/* ------------------------------------------------------------------------- */
void swtimer_detach(swtimer_t *swtimer) {
	assert(swtimer != NULL);

	// cast from opaque pointer to m_swtimer_t
	m_swtimer_t *m_swtimer = (m_swtimer_t*)swtimer;

	m_swtimer->handler = 0;
}

/* ------------------------------------------------------------------------- */
// return value: milliseconds left before timeout
uint32_t swtimer_remaining(swtimer_t *swtimer) {
	assert(swtimer != NULL);

	uint32_t remaining = 0;

	// cast from opaque pointer to m_swtimer_t
	m_swtimer_t *m_swtimer = (m_swtimer_t*)swtimer;

	// search the timer list for specified timer and return time left in milliseconds
	if (swtimer_list != 0) {
		m_swtimer_t *cur = swtimer_list;

		while (cur != m_swtimer) {
			cur = cur->next;
		}

		if (cur == m_swtimer) {
			remaining = (m_swtimer->count * SWTIMER_TICK_PERIOD_X1024)/1024;
		}
	}

	return remaining;
}

/* ------------------------------------------------------------------------- */
static inline void swtimer_disable_timer0(void) {
	timerint_disable(0);
}

/* ------------------------------------------------------------------------- */
static inline void swtimer_enable_timer0(void) {
	timerint_enable(0);
}

/* ------------------------------------------------------------------------- */
// interrupt handler running at F_CPU/(256*256) Hz tick rate
static void swtimer_handler(void) {
	// scan through timer list
	if (swtimer_list != 0) {
		m_swtimer_t *cur = swtimer_list;

		do {
			if (cur->mode == SWTIMER_MODE_ONCE) {
				if (cur->count > 1) {
					cur->count -= 1;
				} else {
					// timer expiration
					cur->mode = SWTIMER_MODE_OFF;
					cur->count = 0;
					if (cur->handler != 0)  cur->handler();
				}

			} else if (cur->mode == SWTIMER_MODE_REPEATING) {
				if (cur->count > 1) {
					cur->count -= 1;
				} else {
					// timer expiration
					cur->count = cur->start_count;		// reload timer
					if (cur->handler != 0)  cur->handler();
				}
			}
			cur = cur->next;
		} while (cur != 0);
	}
}
