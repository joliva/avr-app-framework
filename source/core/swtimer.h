/******************************************************************************
  Filename:	swtimer.h
  Created:	03/13/10 (JLO)
  Purpose:	

  Notes:	The SW Timer module makes use of Timer0 clock divisor of 128
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

#ifndef _SWTIMER_H
#define _SWTIMER_H

#include "common.h"

// constants & definitions

typedef void* swtimer_t;

typedef enum {
	SWTIMER_MODE_OFF, SWTIMER_MODE_ONCE, SWTIMER_MODE_REPEATING
} swtimer_mode_t;

// downcounter initial values (msec)
#define SWTIMER_10MS				10
#define SWTIMER_50MS				50
#define SWTIMER_100MS				100
#define SWTIMER_250MS				250
#define SWTIMER_500MS				500
#define SWTIMER_1S					1000
#define SWTIMER_2S					2000
#define SWTIMER_5S					5000
#define SWTIMER_10S					10000
#define SWTIMER_30S					30000
#define SWTIMER_1MIN				60000
#define SWTIMER_2MIN				120000
#define SWTIMER_5MIN				300000
#define SWTIMER_60MIN				3600000

// prototypes
void		swtimer_init(void);							// initialize module
swtimer_t*	swtimer_create(void (*handler)(void));		// handler = 0: no callback on timer expiration
void		swtimer_destroy(swtimer_t *swtimer);
uint32_t	swtimer_start(swtimer_t *swtimer, swtimer_mode_t mode, uint32_t time_msec);
void		swtimer_stop(swtimer_t *swtimer);
void		swtimer_attach(swtimer_t *swtimer, void (*handler)(void));
void		swtimer_detach(swtimer_t *swtimer);
uint32_t	swtimer_remaining(swtimer_t *swtimer);		// returns milliseconds left before timeout

#endif // _SWTIMER_H
