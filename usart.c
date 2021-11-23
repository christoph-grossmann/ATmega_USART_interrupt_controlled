/*
 * Christoph Großmann, 2021
 */

#include "usart.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <string.h>

#define USART_STATE_IDLE 0
#define USART_STATE_RXACT 1
#define USART_STATE_SRC 2
#define USART_STATE_DST 3
#define USART_STATE_PAYLOAD_LEN 4
#define USART_STATE_PAYLOAD 5
#define USART_STATE_CRC 6

#define USART_WTCHDG_DSBLD 0 // watchdog disabled
#define USART_WTCHDG_ALIVE 1 // process alive
#define USART_WTCHDG_TRMIN 2 // process marked for termination

// Set USART_SEND_PIN (DE) to Recv Mode
#define USART_DE_RX (USART_PORT &= ~(1 << USART_SEND_PIN))
// Set USART_SEND_PIN (DE) to Send Mode
#define USART_DE_TX (USART_PORT |= (1 << USART_SEND_PIN))

#define USART_TIMER_OFF {\
    USART_TIMER_TIMSK = 0;\
    USART_TIMER_TCNT = 0;\
    USART_TIMER_TCCRB = 0;\
    }

#define USART_TIMER_ON {\
    USART_TIMER_TIMSK |= (1 << USART_TIMER_OCIEA);\
    USART_TIMER_TIFR |= (1 << USART_TIMER_OCFA);\
    USART_TIMER_TCNT = 0;\
    USART_TIMER_TCCRB = (1 << USART_TIMER_CS0);\
    }

#define USART_UPDATE_CRC(crc, val) crc = pgm_read_byte(crc8_table + (crc ^ val))

#define USART_BUFF_INCR_PTR(ptr, max) ptr = (ptr + 1) % max
#define USART_BUFF_CONTAINS_UNREAD(write_ptr, read_ptr) (write_ptr != read_ptr)
#define USART_BUFF_HAS_SPACE(write_ptr, read_ptr, max) (\
    (write_ptr + 1 != read_ptr) &&\
    (write_ptr != max || read_ptr != 0)\
    )

typedef struct {
unsigned char src; // message source
unsigned char dst; // message destination
unsigned char len; // payloud length
unsigned char data[USART_MAX_PAYLOAD]; // payloud
} usart_message;

static volatile unsigned char usart_device_id;
unsigned char ee_usart_device_id EEMEM = USART_DEFAULT_DEVICE_ID; // default device id

const unsigned char crc8_table[256] PROGMEM = {
        0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
        0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
        0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
        0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
        0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
        0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
        0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
        0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
        0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
        0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
        0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
        0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
        0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
        0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
        0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
        0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

/*
 * Send
 */

// Enable receive interrupt
#define USART_SEND_INTRRPT_NABLE USART_UCSRB |= (1 << USART_UDRIE)
// Disable receive interrupt
#define USART_SEND_INTRRPT_DABLE USART_UCSRB &= ~(1 << USART_UDRIE)

static volatile unsigned char usart_send_state = USART_STATE_IDLE;
static volatile unsigned char usart_send_watchdog = USART_WTCHDG_DSBLD;
static volatile unsigned char usart_send_error_state = 0;

// USART send ring buffer
static volatile usart_message usart_send_buff[USART_SEND_BUFFER_SIZE];
static volatile unsigned char usart_send_buff_write_ptr = 0;
static volatile unsigned char usart_send_buff_read_ptr = 0;

/*
 * Recv
 */

// Enable receive interrupt
#define USART_RECV_INTRRPT_NABLE USART_UCSRB |= (1 << USART_RXCIE)
// Disable receive interrupt
#define USART_RECV_INTRRPT_DABLE USART_UCSRB &= ~(1 << USART_RXCIE)

static volatile unsigned char usart_recv_state = USART_STATE_IDLE;
static volatile unsigned char usart_recv_watchdog = USART_WTCHDG_DSBLD;

//USART recv ring buffer
static volatile usart_message usart_recv_buff[USART_RECV_BUFFER_SIZE];
static volatile unsigned char usart_recv_buff_write_ptr = 0;
static volatile unsigned char usart_recv_buff_read_ptr = 0;

/*
 * Implementation
 */

// ATmega328P[DATASHEET] p. 151
ISR(USART_UDRE_VECT)
{
    static unsigned char payload_length = 0;
    static unsigned char usart_crc = 0;
    static unsigned char *ptr = 0;
    static unsigned char send_byte = 0;

    USART_RECV_INTRRPT_DABLE;
    USART_DE_TX;

    usart_send_watchdog = USART_WTCHDG_ALIVE;

    if (usart_send_error_state != 0)
    {
        USART_UDR = usart_send_error_state;
        usart_send_error_state = 0;
        
        goto reset_send;
    }

    if (!USART_BUFF_CONTAINS_UNREAD(usart_send_buff_write_ptr, usart_send_buff_read_ptr))
    {
        goto reset_send;
    }

    if (usart_send_state != USART_STATE_IDLE)
    {
        if (USART_UCSRA & (1 << USART_RXC))
        {
            unsigned char in = USART_UDR;
            
            if (in != send_byte)
            {
                // Oh, collision. Damn...
                _delay_us(100);
                
                goto reset_send;
            }
        }
        else
        {
            // Didn't receive anything. This actually should never happen.
            _delay_us(100);
                
            goto reset_send;
        }
    }

    if (usart_send_state == USART_STATE_IDLE)
    {
        send_byte = USART_PREAMBLE;
        usart_crc = 0;
        usart_send_state = USART_STATE_RXACT;

        USART_UPDATE_CRC(usart_crc, send_byte);
        USART_UDR = send_byte;
    }
    else if (usart_send_state == USART_STATE_RXACT)
    {
        send_byte = usart_send_buff[usart_send_buff_read_ptr].src;
        usart_send_state = USART_STATE_SRC;

        USART_UPDATE_CRC(usart_crc, send_byte);
        USART_UDR = send_byte;
    }
    else if (usart_send_state == USART_STATE_SRC)
    {
        send_byte = usart_send_buff[usart_send_buff_read_ptr].dst;
        usart_send_state = USART_STATE_DST;

        USART_UPDATE_CRC(usart_crc, send_byte);
        USART_UDR = send_byte;
    }
    else if (usart_send_state == USART_STATE_DST)
    {
        ptr = usart_send_buff[usart_send_buff_read_ptr].data;
        payload_length = usart_send_buff[usart_send_buff_read_ptr].len;
        send_byte = payload_length;
        usart_send_state = USART_STATE_PAYLOAD_LEN;

        USART_UPDATE_CRC(usart_crc, send_byte);
        USART_UDR = send_byte;
    }
    else if (usart_send_state == USART_STATE_PAYLOAD_LEN)
    {
        send_byte = *ptr++;
        payload_length--;

        USART_UPDATE_CRC(usart_crc, send_byte);
        USART_UDR = send_byte;

        if (payload_length == 0)
        {
            usart_send_state = USART_STATE_PAYLOAD;
        }
    }
    else if (usart_send_state == USART_STATE_PAYLOAD)
    {
        send_byte = usart_crc;
        usart_send_state = USART_STATE_CRC;
        
        USART_UDR = send_byte;
    }
    else if (usart_send_state == USART_STATE_CRC)
    {
        USART_BUFF_INCR_PTR(usart_send_buff_read_ptr, USART_SEND_BUFFER_SIZE);

        goto reset_send;
    }

    return;

reset_send:
    ptr = 0;
    payload_length = 0;
    usart_crc = 0;
    usart_send_state = USART_STATE_IDLE;
    usart_send_watchdog = USART_WTCHDG_DSBLD;

    USART_SEND_INTRRPT_DABLE;
    USART_DE_RX;
    USART_RECV_INTRRPT_NABLE;
}

unsigned char usart_send(const unsigned char *data, unsigned int length)
{
    unsigned char retval = USART_ERROR;

    USART_RECV_INTRRPT_DABLE;
    USART_DE_TX;

    while(length--)
    {
        USART_UCSRA |= (1 << USART_TXC);
        
        while (!(USART_UCSRA & (1 << USART_UDRE))); //wait...
        
        USART_UDR = *data;
        
        while (!(USART_UCSRA & (1 << USART_TXC))); // wait until bits are sent

        if (USART_UCSRA & (1 << USART_RXC))
        {
            unsigned char in = USART_UDR;
            
            if (in != *data)
            {
                // Oh, collision. Damn...
                _delay_us(100);
                goto send;
            }
        }
        else
        {
            // Didn't receive anything. This actually should never happen.
            _delay_us(100);
            goto send;
        }

        data++;
    }

    retval = USART_SUCCESS;

send:
    USART_DE_RX;
    USART_RECV_INTRRPT_NABLE;

    return retval;
}

ISR(USART_RXC_VECT)
{
    static unsigned char my_payload_length = 0;
    static unsigned char usart_recv_crc = 0;
    static unsigned char *ptr = 0;

    unsigned char in = USART_UDR;

    if (usart_recv_state == USART_STATE_IDLE && in == USART_PREAMBLE) {
        usart_recv_crc = 0;
        USART_UPDATE_CRC(usart_recv_crc, in);
        usart_recv_state = USART_STATE_RXACT;

        if (!USART_BUFF_HAS_SPACE(usart_recv_buff_write_ptr, usart_recv_buff_read_ptr, USART_RECV_BUFFER_SIZE))
        {
            // Delte oldest message if no space is available in the buffer
            USART_BUFF_INCR_PTR(usart_recv_buff_write_ptr, USART_RECV_BUFFER_SIZE);
        }

        ptr = usart_recv_buff[usart_recv_buff_write_ptr].data;
    }
    else if (usart_recv_state == USART_STATE_RXACT)
    {
        USART_UPDATE_CRC(usart_recv_crc, in);
        usart_recv_state = USART_STATE_SRC;
        usart_recv_buff[usart_recv_buff_write_ptr].src = in;
    }
    else if (usart_recv_state == USART_STATE_SRC)
    {
        USART_UPDATE_CRC(usart_recv_crc, in);
        usart_recv_state = USART_STATE_DST;
        usart_recv_buff[usart_recv_buff_write_ptr].dst = in;
    }
    else if (usart_recv_state == USART_STATE_DST)
    {
        USART_UPDATE_CRC(usart_recv_crc, in);
        usart_recv_state = USART_STATE_PAYLOAD_LEN;
        my_payload_length = in;
        usart_recv_buff[usart_recv_buff_write_ptr].len = in;
        
        if (my_payload_length > USART_MAX_PAYLOAD)
        {
            goto reset_recv;
        }
        else if (my_payload_length == 0)
        {
            usart_recv_state = USART_STATE_PAYLOAD;
        }
    }
    else if (usart_recv_state == USART_STATE_PAYLOAD_LEN)
    {
        USART_UPDATE_CRC(usart_recv_crc, in);
        *ptr++ = in;
        my_payload_length--;

        if (my_payload_length == 0) {
            usart_recv_state = USART_STATE_PAYLOAD;
        }
    }
    else if (usart_recv_state == USART_STATE_PAYLOAD)
    {
        if (usart_recv_crc == in && usart_recv_buff[usart_recv_buff_write_ptr].dst == usart_device_id) {
            USART_BUFF_INCR_PTR(usart_recv_buff_write_ptr, USART_RECV_BUFFER_SIZE);
        }
        goto reset_recv;
    }

    return;

reset_recv:
    ptr = 0;
    my_payload_length = 0;
    usart_recv_crc = 0;
    usart_recv_state = USART_STATE_IDLE;
}

ISR(USART_TIMER_COMPA_VECT)
{
    USART_TIMER_TCNT = 0;

    if (usart_send_watchdog == USART_WTCHDG_TRMIN)
    {
        usart_send_state = USART_STATE_IDLE;
        usart_send_watchdog = USART_WTCHDG_DSBLD;

        USART_SEND_INTRRPT_DABLE;
        USART_DE_RX;
        USART_RECV_INTRRPT_NABLE;
    }

    if (usart_recv_watchdog == USART_WTCHDG_TRMIN)
    {
        usart_recv_state = USART_STATE_IDLE;
        usart_recv_watchdog = USART_WTCHDG_DSBLD;
        usart_send_error_state = USART_TIMEOUT;
        USART_SEND_INTRRPT_NABLE;
    }

    if (usart_send_watchdog == USART_WTCHDG_DSBLD
        && usart_recv_watchdog == USART_WTCHDG_DSBLD
        && USART_BUFF_CONTAINS_UNREAD(usart_send_buff_write_ptr, usart_send_buff_read_ptr))
    {
        USART_SEND_INTRRPT_NABLE;
    }
    
    if (usart_send_watchdog == USART_WTCHDG_ALIVE)
    {
        usart_send_watchdog = USART_WTCHDG_TRMIN;
    }

    if (usart_recv_watchdog == USART_WTCHDG_ALIVE)
    {
        usart_recv_watchdog = USART_WTCHDG_TRMIN;
    }
}

// Initialise everything needed for successfull USART communication
void usart_init(void)
{
    // Disable all interrupts
    cli();
    
    // Ensure that USART is not powered down (by powering it up). // ATmega328P[DATASHEET] p. 39
    USART_PRR &= ~(1 << USART_PRUSART);

    // Set baud rate // ATmega328P[DATASHEET] p. 149
    USART_UBRRH = (unsigned char) (USART_UBRRVAL >> 8);
    USART_UBRRL = (unsigned char) USART_UBRRVAL;

    // Enable receiver, transmitter, and receive interrupt
    USART_UCSRB = ( 1 << USART_RXEN) | (1 << USART_TXEN) | (1 << USART_RXCIE);

    // Set frame format: 8 data bit, 0 parity bit, 1 stop bit // ATmega328P[DATASHEET] p. 147
    USART_UCSRC = (1 << USART_UCSZ1) | (1 << USART_UCSZ0);

    // USART_RECV_PIN as recv + pull up
    USART_DDR &= ~(1 << USART_RECV_PIN);
    USART_PORT |= (1 << USART_RECV_PIN);

    // USART_SEND_PIN (DE) as send and to Recv Mode
    USART_DDR |= (1 << USART_SEND_PIN);
    USART_DE_RX;

    // Configure Timer
    USART_TIMER_TCCRA = 0;
    USART_TIMER_TCCRB = 0; // Timer deactivated
    USART_TIMER_TCNT = 0;
    USART_TIMER_OCRA = USART_TIMER_TIME;

    // Enable timer (watchdog for send/recv, enables send interrupt)
    USART_TIMER_ON;

    // Read id from EEPROM
    usart_device_id = usart_device_id_read();

    _delay_ms(500);

    // Enable all interrupts
    sei();
}

// Reads the next unread message from the ring buffer into the usart_recv_... variables
int usart_recv_buffer_next_message()
{
    if (!USART_BUFF_CONTAINS_UNREAD(usart_recv_buff_write_ptr, usart_recv_buff_read_ptr)) {
        return USART_ERROR; //  o new messages to return
    }

    // Disable all interrupts
    cli();

    usart_recv_src = usart_recv_buff[usart_recv_buff_read_ptr].src;
    usart_recv_dst = usart_recv_buff[usart_recv_buff_read_ptr].dst;
    usart_recv_payload_length = usart_recv_buff[usart_recv_buff_read_ptr].len;
    
    unsigned char length = usart_recv_payload_length;
    unsigned char *pntr_in = usart_recv_buff[usart_recv_buff_read_ptr].data;
    unsigned char *pntr_buff = usart_recv_payload;

    while (length--)
    {
        *pntr_buff++ = *pntr_in++;
    }

    USART_BUFF_INCR_PTR(usart_recv_buff_read_ptr, USART_RECV_BUFFER_SIZE);

    // Enable all interrupts
    sei();

    return USART_SUCCESS;
}

// Copy message from recv buffer into ring buffer
int usart_send_buffer_write(const unsigned char src, const unsigned char dst, unsigned char payload_length, unsigned char *payload)
{
    if (!USART_BUFF_HAS_SPACE(usart_send_buff_write_ptr, usart_send_buff_read_ptr, USART_SEND_BUFFER_SIZE)) {
        return USART_BUFFER_FULL; // No more space in the buffer for new messages
    }

    if (payload_length > USART_MAX_PAYLOAD) {
        return USART_ERROR; // Given data is longer than maximum payload length
    }

    usart_send_buff[usart_send_buff_write_ptr].src = src;
    usart_send_buff[usart_send_buff_write_ptr].dst = dst;
    usart_send_buff[usart_send_buff_write_ptr].len = payload_length;
    unsigned char *ptr = usart_send_buff[usart_send_buff_write_ptr].data;

    while(payload_length--)
    {
        *ptr++ = *payload++;
    }

    USART_BUFF_INCR_PTR(usart_recv_buff_write_ptr, USART_SEND_BUFFER_SIZE);

    return USART_SUCCESS;
}

unsigned char usart_device_id_read()
{
    return eeprom_read_byte(&ee_usart_device_id);
}

void usart_device_id_write(unsigned char device_id)
{
    eeprom_write_byte(&ee_usart_device_id, device_id);
    usart_device_id = device_id;
}
