/*
 * Christoph Großmann, 2021
 */

#include "usart.h"

#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

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

#define USART_UPDATE_CHECKSUM(sum, val) sum ^= val

#define USART_BUFF_INCR_PTR(ptr, max) ptr = (ptr + 1) % max
#define USART_BUFF_CONTAINS_UNREAD(write_ptr, read_ptr) (write_ptr != read_ptr)
#define USART_BUFF_HAS_SPACE(write_ptr, read_ptr, max) ((write_ptr + 1) % max != read_ptr)

typedef struct {
unsigned char src; // message source
unsigned char dst; // message destination
unsigned char len; // payloud length
unsigned char data[USART_MAX_PAYLOAD]; // payloud
} usart_message;

static volatile unsigned char usart_device_id;
unsigned char ee_usart_device_id EEMEM = USART_DEFAULT_DEVICE_ID; // default device id

/*
 * Send
 */

// Set USART_SEND_PIN (DE) to Recv Mode
#define USART_DE_RX (USART_PORT &= ~(1 << USART_SEND_PIN))
// Set USART_SEND_PIN (DE) to Send Mode
#define USART_DE_TX (USART_PORT |= (1 << USART_SEND_PIN))

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
    static unsigned char checksum = 0;
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
        checksum = send_byte;
        usart_send_state = USART_STATE_RXACT;

        USART_UDR = send_byte;
    }
    else if (usart_send_state == USART_STATE_RXACT)
    {
        send_byte = usart_send_buff[usart_send_buff_read_ptr].src;
        usart_send_state = USART_STATE_SRC;

        USART_UPDATE_CHECKSUM(checksum, send_byte);
        USART_UDR = send_byte;
    }
    else if (usart_send_state == USART_STATE_SRC)
    {
        send_byte = usart_send_buff[usart_send_buff_read_ptr].dst;
        usart_send_state = USART_STATE_DST;

        USART_UPDATE_CHECKSUM(checksum, send_byte);
        USART_UDR = send_byte;
    }
    else if (usart_send_state == USART_STATE_DST)
    {
        ptr = usart_send_buff[usart_send_buff_read_ptr].data;
        payload_length = usart_send_buff[usart_send_buff_read_ptr].len;
        send_byte = payload_length;
        usart_send_state = USART_STATE_PAYLOAD_LEN;

        USART_UPDATE_CHECKSUM(checksum, send_byte);
        USART_UDR = send_byte;
    }
    else if (usart_send_state == USART_STATE_PAYLOAD_LEN)
    {
        send_byte = *ptr++;
        payload_length--;

        USART_UPDATE_CHECKSUM(checksum, send_byte);
        USART_UDR = send_byte;

        if (payload_length == 0)
        {
            usart_send_state = USART_STATE_PAYLOAD;
        }
    }
    else if (usart_send_state == USART_STATE_PAYLOAD)
    {
        send_byte = checksum;
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
    checksum = 0;
    usart_send_state = USART_STATE_IDLE;
    usart_send_watchdog = USART_WTCHDG_DSBLD;

    USART_SEND_INTRRPT_DABLE;
    USART_DE_RX;
    USART_RECV_INTRRPT_NABLE;
}

ISR(USART_RXC_VECT)
{
    static unsigned char my_payload_length = 0;
    static unsigned char checksum = 0;
    static unsigned char *ptr = 0;

    unsigned char in = USART_UDR;

    if (usart_recv_state == USART_STATE_IDLE && in == USART_PREAMBLE) {
        checksum = in;
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
        USART_UPDATE_CHECKSUM(checksum, in);
        usart_recv_state = USART_STATE_SRC;
        usart_recv_buff[usart_recv_buff_write_ptr].src = in;
    }
    else if (usart_recv_state == USART_STATE_SRC)
    {
        USART_UPDATE_CHECKSUM(checksum, in);
        usart_recv_state = USART_STATE_DST;
        usart_recv_buff[usart_recv_buff_write_ptr].dst = in;
    }
    else if (usart_recv_state == USART_STATE_DST)
    {
        USART_UPDATE_CHECKSUM(checksum, in);
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
        USART_UPDATE_CHECKSUM(checksum, in);
        *ptr++ = in;
        my_payload_length--;

        if (my_payload_length == 0) {
            usart_recv_state = USART_STATE_PAYLOAD;
        }
    }
    else if (usart_recv_state == USART_STATE_PAYLOAD)
    {
        if (checksum == in && usart_recv_buff[usart_recv_buff_write_ptr].dst == usart_device_id) {
            USART_BUFF_INCR_PTR(usart_recv_buff_write_ptr, USART_RECV_BUFFER_SIZE);
        }
        goto reset_recv;
    }

    return;

reset_recv:
    ptr = 0;
    my_payload_length = 0;
    checksum = 0;
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

    // Configure and enable timer (watchdog for send/recv, enables send interrupt)
    USART_TIMER_OCRA = USART_TIMER_TIME;
    USART_TIMER_TCCRA = 0;
    USART_TIMER_TCCRB = (1 << USART_TIMER_CS0);
    USART_TIMER_TIMSK |= (1 << USART_TIMER_OCIEA);
    USART_TIMER_TIFR |= (1 << USART_TIMER_OCFA);
    USART_TIMER_TCNT = 0;

    // Read id from EEPROM
    usart_device_id = usart_device_id_read();

    _delay_ms(500);

    // Enable all interrupts
    sei();
}

// Reads the next unread message from the ring buffer into the usart_recv_... variables
unsigned char usart_recv_buffer_read()
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
unsigned char usart_send_buffer_write(const unsigned char src, const unsigned char dst, unsigned char payload_length, unsigned char *payload)
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
