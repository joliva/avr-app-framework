/******************************************************************************
  Filename:	usart.c
  Created:	03/10/10 (JLO)
  Purpose:	USART device control.

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

#include <stdio.h>
#include <stdlib.h>
#include "debug.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "common.h"
#include "circbuf.h"
#include "usart.h"


// constants and definitions

typedef struct {
	uint8_t		device_num;
	uint32_t	baudrate;
	circbuf_t	xmt_buf;
	circbuf_t	rcv_buf;
	bool		read_blocking;
	bool		read_echo;
	circbuf_t	*pxbuf;
	circbuf_t	*prbuf;
	FILE		file;
	FILE		*stream;
} usart_t;


// ID _must_ be a number, not a variable
#define SET_UCSRA(ID,VAL)		UCSR##ID##A = VAL
#define SET_UCSRB(ID,VAL)		UCSR##ID##B = VAL
#define SET_UCSRC(ID,VAL)		UCSR##ID##C = VAL
#define SET_UBRRH(ID,VAL)		UBRR##ID##H = VAL
#define SET_UBRRL(ID,VAL)		UBRR##ID##L = VAL

// prototypes
static int 			uart_putchar(char c, FILE *stream);
static int 			uart_getchar(FILE *stream);
static void 		uart_xmt_interrupt_enable(int device_num);
static void 		uart_xmt_interrupt_disable(int device_num);
static void 		uart_rcv_interrupt_enable(int device_num);
static void 		uart_rcv_interrupt_disable(int device_num);
static usart_t*		stream_to_usart(FILE* stream);

// Unit storage
static usart_t		usarts[USARTS_NUM];

/* ------------------------------------------------------------------------- */
inline FILE* id_to_stream(int device_num) {
	if (device_num > USARTS_NUM - 1) {
		// invalid device number for this processor
		return NULL;
	}

	return usarts[device_num].stream;
}

/* ------------------------------------------------------------------------- */
bool usart_init(uint8_t device_num, baud_rate_t baudrate, usart_parity_t parity, uint8_t xbuf_size, uint8_t rbuf_size) {
	bool		rtcode = false;
	usart_t		*pusart;
	uint16_t	ubbr;
	uint8_t		u2xn;

	assert ((parity==PARITY_NONE) || (parity==PARITY_EVEN) || (parity==PARITY_ODD));
	assert ((baudrate==BAUD_115200) || (baudrate==BAUD_57600) || (baudrate==BAUD_38400) || (baudrate==BAUD_19200) || (baudrate==BAUD_9600));
	assert (device_num <= USARTS_NUM - 1);
	
	switch (baudrate) {
		case BAUD_9600:	// intentional fall through
		case BAUD_19200:	// intentional fall through
		case BAUD_38400:
			u2xn = 0;
			ubbr = (uint16_t) ((F_CPU * 10)/(baudrate * 16));
			if ((ubbr % 10) >= 5) ubbr += 10;	// rounding
			ubbr = (ubbr/10) - 1;
			rtcode = true;
			break;
		case BAUD_57600:
#if F_CPU==8000000
			u2xn = 0x02;
			ubbr = 16;
			rtcode = true;
#elif F_CPU==16000000
			u2xn = 0x02;
			ubbr = 34;
			rtcode = true;
#endif
			break;
		case BAUD_115200:
#if F_CPU==16000000
			u2xn = 0x02;
			ubbr = 16;
			rtcode = true;
#endif
			break;
	}

	if (rtcode == true) {
		uart_xmt_interrupt_disable(device_num);
		uart_rcv_interrupt_disable(device_num);	

		pusart = &usarts[device_num];

		pusart->device_num = device_num;
		pusart->baudrate = baudrate;
		pusart->pxbuf = &(pusart->xmt_buf);
		pusart->prbuf = &(pusart->rcv_buf);
		circbuf_create(pusart->pxbuf,xbuf_size);	// add space in circular buffer
		circbuf_create(pusart->prbuf,rbuf_size);
		pusart->stream = &(pusart->file);
		pusart->read_blocking = true;
		pusart->read_echo = false;
		fdev_setup_stream(pusart->stream, uart_putchar, uart_getchar, _FDEV_SETUP_RW);

		// need an entry for each device for macro expansion
		switch (device_num) {
#if DEV_CLASS_1 || DEV_CLASS_2
			case 0:
				SET_UCSRA(0, u2xn);
				SET_UCSRB(0, (1<<RXEN0) | (1<<TXEN0));
				SET_UCSRC(0, (1<<UCSZ00) | (1<<UCSZ01) | parity);	// 8 bit, 1 stop bit
				SET_UBRRH(0, (ubbr >> 8)); 							// baud rate upper byte
				SET_UBRRL(0, (ubbr)); 								// baud rate lower byte
				break;
#endif
#if DEV_CLASS_1
			case 1:
				SET_UCSRA(1, u2xn);
				SET_UCSRB(1, (1<<RXEN1) | (1<<TXEN1));
				SET_UCSRC(1, (1<<UCSZ10) | (1<<UCSZ11) | parity);	// 8 bit, 1 stop bit
				SET_UBRRH(1, (ubbr >> 8)); 							// baud rate upper byte
				SET_UBRRL(1, (ubbr)); 								// baud rate lower byte
				break;
			case 2:
				SET_UCSRA(2, u2xn);
				SET_UCSRB(2, (1<<RXEN2) | (1<<TXEN2));
				SET_UCSRC(2, (1<<UCSZ20) | (1<<UCSZ21) | parity);	// 8 bit, 1 stop bit
				SET_UBRRH(2, (ubbr >> 8)); 							// baud rate upper byte
				SET_UBRRL(2, (ubbr)); 								// baud rate lower byte
				break;
			case 3:
				SET_UCSRA(3, u2xn);
				SET_UCSRB(3, (1<<RXEN3) | (1<<TXEN3));
				SET_UCSRC(3, (1<<UCSZ30) | (1<<UCSZ31) | parity);	// 8 bit, 1 stop bit
				SET_UBRRH(3, (ubbr >> 8)); 							// baud rate upper byte
				SET_UBRRL(3, (ubbr)); 								// baud rate lower byte
				break;
#endif
		}

		uart_xmt_interrupt_enable(device_num);
		uart_rcv_interrupt_enable(device_num);
	}

	return rtcode ;
}

/* ------------------------------------------------------------------------- */
// reconfigures a serial port previously initialized
bool usart_config(uint8_t device_num, baud_rate_t baudrate, usart_parity_t parity, uint8_t xbuf_size, uint8_t rbuf_size) {
	bool		rtcode = false;
	usart_t		*pusart;
	uint16_t	ubbr;
	uint8_t		u2xn;

	assert ((parity==PARITY_NONE) || (parity==PARITY_EVEN) || (parity==PARITY_ODD));
	assert ((baudrate==BAUD_115200) || (baudrate==BAUD_57600) || (baudrate==BAUD_38400) || (baudrate==BAUD_19200) || (baudrate==BAUD_9600));
	assert (device_num <= USARTS_NUM - 1);
	
	switch (baudrate) {
		case BAUD_9600:	// intentional fall through
		case BAUD_19200:	// intentional fall through
		case BAUD_38400:
			u2xn = 0;
			ubbr = (uint16_t) ((F_CPU * 10)/(baudrate * 16));
			if ((ubbr % 10) >= 5) ubbr += 10;	// rounding
			ubbr = (ubbr/10) - 1;
			rtcode = true;
			break;
		case BAUD_57600:
#if F_CPU==8000000
			u2xn = 0x02;
			ubbr = 16;
			rtcode = true;
#elif F_CPU==16000000
			u2xn = 0x02;
			ubbr = 34;
			rtcode = true;
#endif
			break;
		case BAUD_115200:
#if F_CPU==16000000
			u2xn = 0x02;
			ubbr = 16;
			rtcode = true;
#endif
			break;
	}
	
	if (rtcode == true) {
		uart_xmt_interrupt_disable(device_num);
		uart_rcv_interrupt_disable(device_num);	

		pusart = &usarts[device_num];

		pusart->baudrate = baudrate;
		circbuf_delete(pusart->pxbuf);				// remove existing buffer
		circbuf_create(pusart->pxbuf,xbuf_size);	// create new buffer
		circbuf_delete(pusart->prbuf);				// remove existing buffer
		circbuf_create(pusart->prbuf,rbuf_size);	// create new buffer

		// need an entry for each device for macro expansion
		switch (device_num) {
#if DEV_CLASS_1 || DEV_CLASS_2
			case 0:
				SET_UCSRA(0, u2xn);
				SET_UCSRC(0, (1<<UCSZ00) | (1<<UCSZ01) | parity);	// 8 bit, 1 stop bit
				SET_UBRRH(0, (ubbr >> 8)); 							// baud rate upper byte
				SET_UBRRL(0, (ubbr)); 								// baud rate lower byte
				break;
#endif
#if DEV_CLASS_1
			case 1:
				SET_UCSRA(1, u2xn);
				SET_UCSRC(1, (1<<UCSZ10) | (1<<UCSZ11) | parity);	// 8 bit, 1 stop bit
				SET_UBRRH(1, (ubbr >> 8)); 							// baud rate upper byte
				SET_UBRRL(1, (ubbr)); 								// baud rate lower byte
				break;
			case 2:
				SET_UCSRA(2, u2xn);
				SET_UCSRC(2, (1<<UCSZ20) | (1<<UCSZ21) | parity);	// 8 bit, 1 stop bit
				SET_UBRRH(2, (ubbr >> 8)); 							// baud rate upper byte
				SET_UBRRL(2, (ubbr)); 								// baud rate lower byte
				break;
			case 3:
				SET_UCSRA(3, u2xn);
				SET_UCSRC(3, (1<<UCSZ30) | (1<<UCSZ31) | parity);	// 8 bit, 1 stop bit
				SET_UBRRH(3, (ubbr >> 8)); 							// baud rate upper byte
				SET_UBRRL(3, (ubbr)); 								// baud rate lower byte
				break;
#endif
		}

		uart_xmt_interrupt_enable(device_num);
		uart_rcv_interrupt_enable(device_num);
	}

	return rtcode;
}

/* ------------------------------------------------------------------------- */
inline uint8_t usart_inbuf_fill(FILE *stream) {
	usart_t	*pusart;

	pusart = stream_to_usart(stream);
	assert(pusart != NULL);

	return circbuf_fill(pusart->prbuf);
}

/* ------------------------------------------------------------------------- */
inline void usart_inbuf_flush(FILE *stream) {
	usart_t	*pusart;

	pusart = stream_to_usart(stream);
	assert(pusart != NULL);

	circbuf_flush(pusart->prbuf);
}

/* ------------------------------------------------------------------------- */
inline void usart_read_blocking(FILE *stream, bool block) {
	usart_t	*pusart;

	pusart = stream_to_usart(stream);
	assert(pusart != NULL);

	pusart->read_blocking = block;
}

/* ------------------------------------------------------------------------- */
inline void usart_read_echo(FILE *stream, bool echo) {
	usart_t	*pusart;

	pusart = stream_to_usart(stream);
	assert(pusart != NULL);

	pusart->read_echo = echo;
}

/* ------------------------------------------------------------------------- */
// write len bytes from src to input stream
// note: normally filled from received character interrupt routine
void usart_inbuf_write(FILE* stream, uint8_t* src, uint8_t len) {
	usart_t*	pusart = stream_to_usart(stream);
	circbuf_t*	pcircbuf = (circbuf_t*)&(pusart->rcv_buf);
	uint8_t		idx;

	for (idx=0; idx<len; idx++) {
		assert (circbuf_full(pcircbuf) == false);
		circbuf_write(pcircbuf, *(src+idx));
	}
}

/* ------------------------------------------------------------------------- */
static void uart_xmt_interrupt_enable(int device_num) {
	assert(device_num < USARTS_NUM);

	switch (device_num) {
#if DEV_CLASS_1 || DEV_CLASS_2
		case 0:
			UCSR0B |= (1 << UDRIE0);
			break;
#endif
#if DEV_CLASS_1
		case 1:
			UCSR1B |= (1 << UDRIE1);
			break;
		case 2:
			UCSR2B |= (1 << UDRIE2);
			break;
		case 3:
			UCSR3B |= (1 << UDRIE3);
			break;
#endif
	}
}

/* ------------------------------------------------------------------------- */
static void uart_xmt_interrupt_disable(int device_num) {
	assert(device_num < USARTS_NUM);

	switch (device_num) {
#if DEV_CLASS_1 || DEV_CLASS_2
		case 0:
			UCSR0B &= ~(1 << UDRIE0);
			break;
#endif
#if DEV_CLASS_1
		case 1:
			UCSR1B &= ~(1 << UDRIE1);
			break;
		case 2:
			UCSR2B &= ~(1 << UDRIE2);
			break;
		case 3:
			UCSR3B &= ~(1 << UDRIE3);
			break;
#endif
	}
}

/* ------------------------------------------------------------------------- */
static void uart_rcv_interrupt_enable(int device_num) {
	assert(device_num < USARTS_NUM);

	switch (device_num) {
		case 0:
#if DEV_CLASS_1 || DEV_CLASS_2
			UCSR0B |= (1 << RXCIE0);
			break;
#endif
#if DEV_CLASS_1
		case 1:
			UCSR1B |= (1 << RXCIE1);
			break;
		case 2:
			UCSR2B |= (1 << RXCIE2);
			break;
		case 3:
			UCSR3B |= (1 << RXCIE3);
			break;
#endif
	}
}

/* ------------------------------------------------------------------------- */
static void uart_rcv_interrupt_disable(int device_num) {
	assert(device_num < USARTS_NUM);

	switch (device_num) {
#if DEV_CLASS_1 || DEV_CLASS_2
		case 0:
			UCSR0B &= ~(1 << RXCIE0);
			break;
#endif
#if DEV_CLASS_1
		case 1:
			UCSR1B &= ~(1 << RXCIE1);
			break;
		case 2:
			UCSR2B &= ~(1 << RXCIE2);
			break;
		case 3:
			UCSR3B &= ~(1 << RXCIE3);
			break;
#endif
	}
}
/* ------------------------------------------------------------------------- */
static int uart_putchar(char c, FILE *stream) {
	int 	id;
	usart_t	*pusart;

	pusart = stream_to_usart(stream);
	assert(pusart != NULL);

	id = pusart->device_num;

	if (c == '\n')
		uart_putchar('\r', stream);

	while (circbuf_full(pusart->pxbuf) == true) {};

	uart_xmt_interrupt_disable(id);
	if (circbuf_empty(pusart->pxbuf) == true) {
		circbuf_write(pusart->pxbuf, c);
		uart_xmt_interrupt_enable(id);

		// generate transmit complete interrupt to initiate transmission
		switch (id) {
#if DEV_CLASS_1 || DEV_CLASS_2
			case 0:
				UCSR0A |= (1<<UDRE0);
				break;
#endif
#if DEV_CLASS_1
			case 1:
				UCSR1A |= (1<<UDRE1);
				break;
			case 2:
#if KLUDGE==1
				// interrupt occurs after transmit completes
				if (circbuf_empty(usarts[2].pxbuf) == false) {
					UDR2 = circbuf_read(usarts[2].pxbuf);
				}
				while ((UCSR2A & (1<<UDRE2)) == 0) {};	// wait until sent
#else
				UCSR2A |= (1<<UDRE2);
#endif
				break;
			case 3:
#if KLUDGE==1
				// interrupt occurs after transmit completes
				if (circbuf_empty(usarts[3].pxbuf) == false) {
					UDR3 = circbuf_read(usarts[3].pxbuf);
				}
				while ((UCSR3A & (1<<UDRE3)) == 0) {};	// wait until sent
#else
				UCSR3A |= (1<<UDRE3);
#endif
				break;
#endif
		}
	} else {
		circbuf_write(pusart->pxbuf, c);
		uart_xmt_interrupt_enable(id);
	}

	return 0;
}

/* ------------------------------------------------------------------------- */
static int uart_getchar(FILE *stream) {
	int		ch;
	int 	id;
	usart_t	*pusart;

	pusart = stream_to_usart(stream);
	assert(pusart != NULL);

	id = pusart->device_num;

	if (pusart->read_blocking == true) {
		while (circbuf_empty(pusart->prbuf) == true) {};

		uart_rcv_interrupt_disable(id);
		ch = circbuf_read(pusart->prbuf);
		uart_rcv_interrupt_enable(id);
	} else {
		if (circbuf_empty(pusart->prbuf) == true) {
			ch = EOF;
		} else {
			uart_rcv_interrupt_disable(id);
			ch = circbuf_read(pusart->prbuf);
			uart_rcv_interrupt_enable(id);
		}
	}

	// convert carriage return to linefeed to make gets(), etc. happy
	if (ch == '\r') {
		ch = '\n';
	}

	if ((pusart->read_echo == true) && (ch != EOF)) {
		fputc(ch, stdout);
	}

	return ch;
}

/* ------------------------------------------------------------------------- */
static usart_t* stream_to_usart(FILE* stream) {
	int id;

	for (id=0; id<USARTS_NUM; id++) {
		if (usarts[id].stream == stream) {
			return &(usarts[id]);
		}
	}

	return NULL;
}

#if DEV_CLASS_2
/* ------------------------------------------------------------------------- */
// USART0 receive interrupt handler
ISR(USART_RX_vect) {
 	circbuf_write(usarts[0].prbuf, UDR0);
}

// USART0 transmit interrupt handler
ISR(USART_UDRE_vect) {
	// interrupt occurs when UDR buffer is empty
	if (circbuf_empty(usarts[0].pxbuf) == false) {
		UDR0 = circbuf_read(usarts[0].pxbuf);
	} else {
		uart_xmt_interrupt_disable(0);
	}
}
#endif

#if DEV_CLASS_1
/* ------------------------------------------------------------------------- */
// USART0 receive interrupt handler
ISR(USART0_RX_vect) {
 	circbuf_write(usarts[0].prbuf, UDR0);
} 

// USART0 transmit interrupt handler
ISR(USART0_UDRE_vect) {
	// interrupt occurs when UDR buffer is empty
	if (circbuf_empty(usarts[0].pxbuf) == false) {
		UDR0 = circbuf_read(usarts[0].pxbuf);
	} else {
		uart_xmt_interrupt_disable(0);
	}
}

/* ------------------------------------------------------------------------- */
// USART1 receive interrupt handler
ISR(USART1_RX_vect) {
 	circbuf_write(usarts[1].prbuf, UDR1);
} 

// USART1 transmit interrupt handler
ISR(USART1_UDRE_vect) {
	// interrupt occurs when UDR buffer is empty
	if (circbuf_empty(usarts[1].pxbuf) == false) {
		UDR1 = circbuf_read(usarts[1].pxbuf);
	} else {
		uart_xmt_interrupt_disable(1);
	}
}

/* ------------------------------------------------------------------------- */
// USART2 receive interrupt handler
ISR(USART2_RX_vect) {
 	circbuf_write(usarts[2].prbuf, UDR2);
} 

// USART2 transmit interrupt handler
ISR(USART2_UDRE_vect) {
	// interrupt occurs when UDR buffer is empty
	if (circbuf_empty(usarts[2].pxbuf) == false) {
		UDR2 = circbuf_read(usarts[2].pxbuf);
	} else {
		uart_xmt_interrupt_disable(2);
	}
}

/* ------------------------------------------------------------------------- */
// USART3 receive interrupt handler
ISR(USART3_RX_vect) {
 	circbuf_write(usarts[3].prbuf, UDR3);
} 

// USART3 transmit interrupt handler
ISR(USART3_UDRE_vect) {
	// interrupt occurs when UDR buffer is empty
	if (circbuf_empty(usarts[3].pxbuf) == false) {
		UDR3 = circbuf_read(usarts[3].pxbuf);
	} else {
		uart_xmt_interrupt_disable(3);
	}
}
#endif
 
