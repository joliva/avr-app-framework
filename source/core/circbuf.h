/******************************************************************************
  Filename:	circbuf.h
  Created:	03/14/10 (JLO)
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

#ifndef _CIRCBUF_H
#define _CIRCBUF_H

#include "common.h"

typedef struct {
	int		size;			// size of buffer in bytes
	int 	data_size;		// number of bytes stored in buffer
	int 	read_pointer;	// indice number of last read char
	int 	write_pointer;	// indice number of last written char
	char 	*buffer;
} circbuf_t;

// prototypes
bool		circbuf_full(circbuf_t* buffer);
bool		circbuf_empty(circbuf_t* buffer);
void		circbuf_write(circbuf_t* buffer, uint8_t c);
uint8_t		circbuf_read(circbuf_t* buffer);
void		circbuf_create(circbuf_t* buffer, uint16_t size);
void		circbuf_delete(circbuf_t* buffer);
uint8_t		circbuf_fill(circbuf_t* buffer);		// return # of bytes stored in buffer
void		circbuf_flush(circbuf_t* buffer);		// empty out buffer
 
#endif // _CIRCBUF_H
