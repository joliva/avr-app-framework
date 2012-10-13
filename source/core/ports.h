/******************************************************************************
  Filename:	ports.h
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

#ifndef _PORTS_H
#define _PORTS_H

#include "common.h"

// constants & definitions

#if DEV_CLASS_1
typedef enum {
	PORT_A=0, PORT_B=3, PORT_C=6, PORT_D=9, PORT_E=12, PORT_F=15,
	PORT_G=18, PORT_H=256, PORT_J=259, PORT_K=262, PORT_L=265
} port_select_t;
#elif DEV_CLASS_2
typedef enum {
	PORT_B=3, PORT_C=6, PORT_D=9
} port_select_t; 
#endif

typedef enum {
	PORT_INPUT=0,
	PORT_OUTPUT=1
} port_direction_t;

typedef struct {
	port_select_t	port_id;
	uint8_t			port_state;
} port_t;

// prototypes
port_t*		port_create(port_select_t port_id);
void		port_destroy(port_t *device);
void	 	port_set_output(port_t *device, uint8_t bit_num);
void	 	port_set_outputs(port_t *device);
void	 	port_set_input(port_t *device, uint8_t bit_num, bool enable_pullup);
void	 	port_set_inputs(port_t *device, bool enable_pullups);
void		port_bit_on(port_t *device, uint8_t bit_num);
void		port_bit_off(port_t *device, uint8_t bit_num);
void		port_bit_toggle(port_t *device, uint8_t bit_num);
bool		port_bit_state(port_t *device, uint8_t bit_num);
void		port_write(port_t *device, uint8_t value);
uint8_t		port_read(port_t *device);

#endif // _PORTS_H
