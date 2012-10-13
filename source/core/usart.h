/******************************************************************************
  Filename:	usart.h
  Created:	03/10/10 (JLO)
  Purpose:	

  Notes:	USART reads block by default
			USART reads are not echoed by default
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

#ifndef _USART_H
#define _USART_H

#include <stdio.h>
#include <avr/io.h>
#include "common.h"

// constants & definitions
#if DEV_CLASS_1
	#define USARTS_NUM	4
#elif DEV_CLASS_2
	#define USARTS_NUM	1
#endif

typedef enum {
	PARITY_NONE = ((0<<UPM01) | (0<<UPM00)),
	PARITY_EVEN = ((1<<UPM01) | (0<<UPM00)),
	PARITY_ODD  = ((1<<UPM01) | (1<<UPM00))
} usart_parity_t;

typedef enum {
	BAUD_9600=9600, BAUD_19200=19200, BAUD_38400=38400, BAUD_57600=57600, BAUD_115200=115200
} baud_rate_t;

// prototypes
bool		usart_init(uint8_t device_num, baud_rate_t baudrate, usart_parity_t parity, uint8_t xbuf_size, uint8_t rbuf_size);
bool		usart_config(uint8_t device_num, baud_rate_t baudrate, usart_parity_t parity, uint8_t xbuf_size, uint8_t rbuf_size);
uint8_t		usart_inbuf_fill(FILE* stream);					// return # bytes in input buffer
void		usart_inbuf_flush(FILE* stream);				// empty input buffer
void		usart_read_blocking(FILE* stream, bool block);	// controls whether reads blocks on empty buffer
void		usart_read_echo(FILE* stream, bool echo);		// controls whether reads are echoed
FILE*		id_to_stream(int device_num);					// maps USART number into i/o stream

#endif // _USART_H
