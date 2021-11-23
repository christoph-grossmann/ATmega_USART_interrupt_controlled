/*
 * Christoph Großmann, 2021
 * 
 * Non-blocking asynchronos USART communcation implementation using interrupts and ring buffers.
 * 
 */

#ifndef USART_H
#define USART_H

#include "usart_register.h"

/*
 * Frequency and baud rate settings.
 */

// Define system oscillator clock frequency if not defined
#ifndef FOCS
#define FOCS 3686400
#endif

// Set desired baud rate
#define BAUD 19200

// Calculate UBRR (baud rate register) value // ATmega328P[DATASHEET] p. 145
#define UBRRVAL ((FOCS / (BAUD * 16UL)) - 1)

/* 
 * Message settings.
 */

#define USART_FRAME_LENGTH 10

#define USART_PREAMBLE 0x40
// The useable payloud length is USART_MAX_PAYLOAD - 5.
#define USART_MAX_PAYLOAD 64

#define USART_TIMER_FRAME_HERTZ (BAUD / USART_FRAME_LENGTH + 1)
// (ticks/s) / (2 * frames/s)
#define USART_TIMER_PRESCALE \
    (FOCS /    2UL / USART_TIMER_FRAME_HERTZ < 0xff ?    1 : \
    (FOCS /   16UL / USART_TIMER_FRAME_HERTZ < 0xff ?    8 : \
    (FOCS /  128UL / USART_TIMER_FRAME_HERTZ < 0xff ?   64 : \
    (FOCS /  512UL / USART_TIMER_FRAME_HERTZ < 0xff ?  256 : \
    (FOCS / 2048UL / USART_TIMER_FRAME_HERTZ < 0xff ? 1024 : 0 \
    )))))
#define USART_TIMER_TIME \
    (FOCS / 2UL / USART_TIMER_PRESCALE / USART_TIMER_FRAME_HERTZ + 1)

/*
 * Provided functionality.
 */

void usart_init(void);
int usart_recv_buffer_read();
int usart_send_buffer_write(const unsigned char src, const unsigned char dst, unsigned char payload_length, unsigned char *payload);

static volatile unsigned char usart_recv_src;
static volatile unsigned char usart_recv_dst;
static volatile unsigned char usart_recv_payload_length;
static volatile unsigned char usart_recv_payload[USART_MAX_PAYLOAD - 5];

/*
 * Ring buffer settings.
 */

#define USART_RECV_BUFFER_SIZE 8
#define USART_SEND_BUFFER_SIZE 8

/*
 * Possible return values.
 */

#define USART_SUCCESS 0
#define USART_ERROR 1
#define USART_BUFFER_FULL 2
#define USART_TIMEOUT 'f'

#endif //USART_H
