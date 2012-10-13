/******************************************************************************
  Filename:	ports.c
  Created:	03/10/10 (JLO)
  Purpose:	PORTs model

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

#include "debug.h"
#include <stdlib.h>
#include <avr/io.h>
#include "common.h"
#include "ports.h"

// constants & definitions

// forward declarations
static uint8_t	port_read_port(port_select_t port_id);
static void		port_write_ddr(port_select_t port_id, uint8_t value);
static uint8_t	port_read_ddr(port_select_t port_id);
static void		port_write_port(port_select_t port_id, uint8_t value);

/* ------------------------------------------------------------------------- */
port_t*	port_create(port_select_t port_id) {
	port_t* device = (port_t *)malloc(sizeof(port_t));

	if (device != NULL) {
		device->port_id = port_id;
		device->port_state = 0;
		port_set_outputs(device);		// set all port bits to output
		port_write_port(port_id, 0);	// set port bits to 0
	}

	return device;
}

/* ------------------------------------------------------------------------- */
void port_destroy(port_t *device) {
	if (device != NULL) {
		free(device);
	}
}

/* ------------------------------------------------------------------------- */
void port_set_output(port_t *device, uint8_t bit_num) {
	uint8_t value;

	assert(bit_num < 8);

	value = port_read_ddr(device->port_id);
	port_write_ddr(device->port_id, value | (1<<bit_num));
}

/* ------------------------------------------------------------------------- */
void port_set_outputs(port_t *device) {
	port_write_ddr(device->port_id, 0xFF);	// set all port bits to output
}

/* ------------------------------------------------------------------------- */
void port_set_input(port_t *device, uint8_t bit_num, bool enable_pullup) {
	uint8_t value;

	assert(bit_num < 8);

	value = port_read_ddr(device->port_id);
	port_write_ddr(device->port_id, value & ~(1<<bit_num));

	value = port_read_port(device->port_id);
	if (enable_pullup == true) {
		port_write_port(device->port_id, value | (1<<bit_num));
	} else {
		port_write_port(device->port_id, value & ~(1<<bit_num));
	}
}

/* ------------------------------------------------------------------------- */
void port_set_inputs(port_t *device, bool enable_pullups) {
	port_write_ddr(device->port_id, 0x00);	// set all port bits to input

	if (enable_pullups == true) {
		port_write_port(device->port_id, 0xFF);
	} else {
		port_write_port(device->port_id, 0);
	}

}

/* ------------------------------------------------------------------------- */
void	port_bit_on(port_t *device, uint8_t bit_num) {
	assert(bit_num < 8);
	assert( (port_read_ddr(device->port_id) & (1<<bit_num)) != 0);	// output bit?

	device->port_state |= (1<<bit_num);
	port_write_port(device->port_id, device->port_state);
}

/* ------------------------------------------------------------------------- */
void	port_bit_off(port_t *device, uint8_t bit_num) {
	assert(bit_num < 8);
	assert( (port_read_ddr(device->port_id) & (1<<bit_num)) != 0);	// output bit?

	device->port_state &= ~(1<<bit_num);
	port_write_port(device->port_id, device->port_state);
}

/* ------------------------------------------------------------------------- */
void	port_bit_toggle(port_t *device, uint8_t bit_num) {
	assert(bit_num < 8);
	assert( (port_read_ddr(device->port_id) & (1<<bit_num)) != 0);	// output bit?

	device->port_state ^= (1<<bit_num);
	port_write_port(device->port_id, device->port_state);
}

/* ------------------------------------------------------------------------- */
bool	port_bit_state(port_t *device, uint8_t bit_num) {
	uint8_t value;

	if ((port_read_ddr(device->port_id) & (1<<bit_num)) == 0) {
		// input bit, update state
		value = port_read_port(device->port_id) & (1<<bit_num);
		device->port_state &= ~(1<<bit_num);	// clear bit state
		device->port_state |= value;				// update bit state
	} else {
		// output bit
		value = device->port_state & (1<<bit_num);
	}

	return (value != 0);
}

/* ------------------------------------------------------------------------- */
void	port_write(port_t *device, uint8_t value) {
	uint8_t ddr, bits;

	ddr = port_read_ddr(device->port_id);

	bits = device->port_state & ~ddr;
	bits |= value & ddr;
	device->port_state = bits;

	port_write_port(device->port_id, bits);
}

/* ------------------------------------------------------------------------- */
uint8_t port_read(port_t *device) {
	uint8_t ddr, value;

	ddr = port_read_ddr(device->port_id);

	// update state for input bits
	value = port_read_port(device->port_id) & ~ddr;
	value |= device->port_state & ddr;
	device->port_state = value;

	return (value);
}

/* ------------------------------------------------------------------------- */
static uint8_t port_read_port(port_select_t port_id) {
	return (port_id < 0x40) ? (_SFR_IO8(port_id)) : (_SFR_MEM8(port_id));
}

/* ------------------------------------------------------------------------- */
static void port_write_ddr(port_select_t port_id, uint8_t value) {
	if (port_id < 0x40) {
		_SFR_IO8(port_id+1) = value;
	} else {
		_SFR_MEM8(port_id+1) = value;
	}
}

/* ------------------------------------------------------------------------- */
static uint8_t port_read_ddr(port_select_t port_id) {
	return (port_id < 0x40) ? (_SFR_IO8(port_id+1)) : (_SFR_MEM8(port_id+1));
}

/* ------------------------------------------------------------------------- */
static void port_write_port(port_select_t port_id, uint8_t value) {
	if (port_id < 0x40) {
		_SFR_IO8(port_id+2) = value;
	} else {
		_SFR_MEM8(port_id+2) = value;
	}
}

