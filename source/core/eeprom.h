/******************************************************************************
  Filename:	eeprom.h
  Created:	03/20/10 (JLO)
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

#ifndef _EEPROM_H
#define _EEPROM_H

#include "common.h"

// prototypes

// writes a byte of data to EEPROM
void ee_write(uint8_t* addr, uint8_t data);

// reads a byte of data from EEPROM
uint8_t ee_read(uint8_t* addr);

// writes a word of data to EEPROM
void ee_write_word(uint16_t* addr, uint16_t data);

// reads a word of data from EEPROM
uint16_t ee_read_word(uint16_t* addr);

// writes a block of data to EEPROM from RAM
void ee_write_block(uint8_t* addr, uint8_t* pdata, uint16_t num_bytes);

// reads a block of data into RAM from EEPROM
void ee_read_block(uint8_t* addr, uint8_t* pdata, uint16_t num_bytes);

// returns the size of the EEPROM in bytes
uint16_t ee_size(void);

#endif
