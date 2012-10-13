/******************************************************************************
  Filename:	system_types.h
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

#ifndef _SYSTEM_TYPES_H
#define _SYSTEM_TYPES_H

// constants & definitions

#ifndef bool
	typedef unsigned char bool;
#endif

#ifndef true
	#define true 1
#endif

#ifndef false
	#define false 0
#endif

// based on C standard library structure in time.h
typedef struct {
  uint8_t tm_sec;		// Seconds. [0-59]
  uint8_t tm_min;		// Minutes. [0-59]
  uint8_t tm_hour;		// Hours.   [0-23]
  uint8_t tm_mday;		// Day.     [1-31]
  uint8_t tm_mon;		// Month.   [0-11]
  uint8_t tm_year;		// Year - 1900	(years since 1900)
  uint8_t tm_wday;		// Day of week.   [0-6]
} datetime_t;

typedef struct {
  uint8_t tm_sec;		// Seconds. [0-59]
  uint8_t tm_min;		// Minutes. [0-59]
  uint8_t tm_hour;		// Hours.   [0-23]
} time_t;

typedef struct {
  uint8_t tm_mday;		// Day.     [1-31]
  uint8_t tm_mon;		// Month.   [0-11]
  uint8_t tm_year;		// Year - 1900	(years since 1900)
} date_t;

#endif // _SYSTEM_TYPES_H
