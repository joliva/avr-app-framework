/******************************************************************************
  Filename:	circbuf.c
  Created:	03/14/10 (JLO)
  Purpose:	Circular queue buffer..

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

#include <stdlib.h>
#include "common.h"
#include "circbuf.h"

/* ------------------------------------------------------------------------- */
// create buffer
void circbuf_create(circbuf_t* cbuf, uint16_t size) {
	cbuf->size = size;
	cbuf->data_size = 0;
	cbuf->read_pointer = 0;
	cbuf->write_pointer = 0;
	cbuf->buffer = malloc(size * sizeof(uint8_t));
}

/* ------------------------------------------------------------------------- */
// delete buffer
inline void circbuf_delete(circbuf_t* cbuf) {
	if (cbuf->buffer != NULL)  free(cbuf->buffer);
}
 
/* ------------------------------------------------------------------------- */
// adds a char
inline void circbuf_write(circbuf_t* cbuf, uint8_t c) {
    // increase write_pointer, check if at end of array
    if (++cbuf->write_pointer >= cbuf->size) cbuf->write_pointer = 0;
 
    cbuf->buffer[cbuf->write_pointer] = c;    
    cbuf->data_size++;
}
 
/* ------------------------------------------------------------------------- */
// pull char from queue
inline uint8_t circbuf_read(circbuf_t* cbuf) {  
	if (++cbuf->read_pointer >= cbuf->size) cbuf->read_pointer = 0;

	cbuf->data_size--;
	return cbuf->buffer[cbuf->read_pointer];  
}

/* ------------------------------------------------------------------------- */
// returns 'true' if buffer is full
inline bool circbuf_full(circbuf_t* cbuf)  {
	return (cbuf->data_size == cbuf->size);
}

/* ------------------------------------------------------------------------- */
// returns 'true' if buffer is empty
inline bool circbuf_empty(circbuf_t* cbuf) {
	return (cbuf->data_size == 0);
}

/* ------------------------------------------------------------------------- */
// returns # of bytes stored in buffer
inline uint8_t circbuf_fill(circbuf_t* cbuf) {  
	return cbuf->data_size;  
}

/* ------------------------------------------------------------------------- */
// empty out buffer
inline void circbuf_flush(circbuf_t* cbuf) {  
	cbuf->data_size = 0;
	cbuf->read_pointer = 0;
	cbuf->write_pointer = 0;
}
