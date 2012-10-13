/******************************************************************************
  Filename:	swclock.c
  Created:	03/23/10 (JLO)
  Purpose:	Software clock (hh:mm:ss)

  Notes:	Depends on the SW Timer module. Singleton
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

#include "common.h"
#include "debug.h"
#include "systypes.h"
#include "swtimer.h"
#include "swclock.h"

// constants & definitions

// storage
time_t		swclock;
swtimer_t	*swtimer;
bool		running = false;

// forward declarations
static void swclock_handler(void);

/* ------------------------------------------------------------------------- */
void swclock_start(void) {
	if (running == false) {
		swtimer_init();		// initialzes swtimer (if necessary)
		swtimer = swtimer_create(swclock_handler);
		swtimer_start(swtimer, SWTIMER_MODE_REPEATING, SWTIMER_1S);	// expires once a second, repeating
		running = true;
	}
}

/* ------------------------------------------------------------------------- */
void swclock_stop(void) {
	if (running == true) {
		swtimer_stop(swtimer);
		swtimer_destroy(swtimer);
		running = false;
	}
}

/* ------------------------------------------------------------------------- */
void swclock_settime(time_t *time) {
	assert (time != 0);

	swclock.tm_sec = time->tm_sec;
	swclock.tm_min = time->tm_min;
	swclock.tm_hour = time->tm_hour;
}

/* ------------------------------------------------------------------------- */
void swclock_gettime(time_t *time) {
	assert (time != 0);

	time->tm_sec = swclock.tm_sec;
	time->tm_min = swclock.tm_min;
	time->tm_hour = swclock.tm_hour;
}

/* ------------------------------------------------------------------------- */
static void swclock_handler(void) {
	// update clock time
	swclock.tm_sec++;
	if (swclock.tm_sec > 59) {
		swclock.tm_sec -= 60;
		swclock.tm_min++;
	}

	if (swclock.tm_min > 59) {
		swclock.tm_min -= 60;
		swclock.tm_hour++;
	}

	if (swclock.tm_hour > 23) {
		swclock.tm_hour -= 24;
	}
}

