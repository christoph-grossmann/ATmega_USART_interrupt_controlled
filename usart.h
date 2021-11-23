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
#ifndef BAUD
#define BAUD 19200
#endif

// Calculate UBRR (baud rate register) value // ATmega328P[DATASHEET] p. 145
#ifndef USART_UBRRVAL
#define USART_UBRRVAL ((FOCS / (BAUD * 16UL)) - 1)
#endif

/* 
 * Message settings.
 */

#define USART_FRAME_LENGTH 10

#ifndef USART_DEFAULT_DEVICE_ID
#define USART_DEFAULT_DEVICE_ID 0xaa
#endif
#ifndef USART_BROADCAST_ID
#define USART_BROADCAST_ID 0xff
#endif

#ifndef USART_PREAMBLE
#define USART_PREAMBLE 0x40
#endif
#ifndef USART_MAX_PAYLOAD
#define USART_MAX_PAYLOAD 16
#endif

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
unsigned char usart_device_id_read();
void usart_device_id_write(unsigned char device_id);

static volatile unsigned char usart_recv_src;
static volatile unsigned char usart_recv_dst;
static volatile unsigned char usart_recv_payload_length;
static volatile unsigned char usart_recv_payload[USART_MAX_PAYLOAD];

/*
 * Ring buffer settings.
 */

#ifndef USART_RECV_BUFFER_SIZE
#define USART_RECV_BUFFER_SIZE 4
#endif
#ifndef USART_SEND_BUFFER_SIZE
#define USART_SEND_BUFFER_SIZE 4
#endif

/*
 * Possible return values.
 */

#define USART_SUCCESS 0
#define USART_ERROR 1
#define USART_BUFFER_FULL 2
#define USART_TIMEOUT 0xAA

#endif //USART_H
