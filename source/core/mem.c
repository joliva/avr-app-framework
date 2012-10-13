/******************************************************************************
  Filename:	mem.c
  Created:	03/21/10 (JLO)
  Purpose:	Internal RAM usage measurement
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
#include <stdlib.h>
#include <avr/io.h>
#include "common.h"
#include "debug.h"
#include "mem.h"

// constants & definitions

/* ------------------------------------------------------------------------- */
void ram_usage(ram_usage_t* usage) {
	char stack = 1;
	extern char *__bss_end, *__data_end, *__data_start;
	extern char *__brkval;
	extern char *__malloc_heap_end;
	extern size_t __malloc_margin;

	assert(usage != 0);

	usage->data_size = (int)&__data_end - (int)&__data_start;
	usage->bss_size = (int)&__bss_end - (int)&__data_end;
	usage->heap_size = (__brkval == 0 ? (int)__malloc_heap_start : (int)__brkval) - (int)&__bss_end;
	usage->stack_size = RAMEND - (int)&stack + 1;
	usage->available =  (RAMEND - (int)&__data_start + 1);
	usage->available -= usage->data_size + usage->bss_size + usage->heap_size + usage->stack_size;
}

