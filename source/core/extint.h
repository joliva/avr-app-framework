/******************************************************************************
  Filename:	extint.h
  Created:	03/10/10 (JLO)
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
******************************************************************************/

#ifndef _EXTINT_H
#define _EXTINT_H

#include "common.h"

// constants & definitions

// interrupt IDs for attaching user functions to external interrupts
// use these with extintAttach( intNum, function )
#if DEV_CLASS_1
typedef enum {
	EXTINT0=0, EXTINT1, EXTINT2, EXTINT3, EXTINT4, EXTINT5, EXTINT6, EXTINT7,
	XPCINT0, XPCINT1, XPCINT2, XPCINT3, XPCINT4, XPCINT5, XPCINT6, XPCINT7, XPCINT8, 
	XPCINT9, XPCINT10, XPCINT11, XPCINT12, XPCINT13, XPCINT14, XPCINT15, XPCINT16,
	XPCINT17, XPCINT18, XPCINT19, XPCINT20, XPCINT21, XPCINT22, XPCINT23,
	EXTINT_ID_NUM
} extint_id_t;
#elif DEV_CLASS_2
typedef enum {
	EXTINT0=0, EXTINT1,
	XPCINT0, XPCINT1, XPCINT2, XPCINT3, XPCINT4, XPCINT5, XPCINT6, XPCINT7, XPCINT8, 
	XPCINT9, XPCINT10, XPCINT11, XPCINT12, XPCINT13, XPCINT14, XPCINT15, XPCINT16,
	XPCINT17, XPCINT18, XPCINT19, XPCINT20, XPCINT21, XPCINT22, XPCINT23,
	EXTINT_ID_NUM
} extint_id_t;
#endif

typedef enum {
	LEVEL_LOW=0, EDGE_ANY=1, EDGE_FALLING=2, EDGE_RISING=3, EXTINT_TYPE_NUM
} extint_type_t;

#define 	EXTINT_NUM_INTERRUPTS	EXTINT_ID_NUM
#define 	EXTINT_NUM_TYPES		EXTINT_TYPE_NUM

// type of interrupt handler to use (value may be SIGNAL or INTERRUPT)
#ifndef EXTINT_INTERRUPT_HANDLER
#define EXTINT_INTERRUPT_HANDLER	SIGNAL
#endif

// prototypes

void extint_init(void);
void extint_enable(extint_id_t int_id);
void extint_disable(extint_id_t int_id);
void extint_attach(extint_id_t int_id, extint_type_t int_type, void (*handler)(void));
void extint_detach(extint_id_t int_id);

#endif
