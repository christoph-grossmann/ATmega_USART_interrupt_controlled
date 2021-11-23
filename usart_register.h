/*
 * Christoph Großmann, 2021
 * 
 * Non-blocking asynchronos USART communcation implementation using interrupts and ring buffers.
 * 
 */

#ifndef USART_REGISTER_H
#define USART_REGISTER_H

/*
 * Registers used by this implementation.
 */

#define USART_PRR PRR
#define USART_PRUSART PRUSART0

#define USART_DDR DDRD
#define USART_PORT PORTD
#define USART_SEND_PIN PD2
#define USART_RECV_PIN PD6

#define USART_UDR UDR0
#define USART_UBRRH UBRR0H
#define USART_UBRRL UBRR0L
#define USART_UCSRA UCSR0A
#define USART_TXC TXC0
#define USART_UDRE UDRE0
#define USART_RXC RXC0
#define USART_UCSRB UCSR0B
#define USART_RXEN RXEN0
#define USART_TXEN TXEN0
#define USART_RXCIE RXCIE0
#define USART_RXC_VECT USART_RX_vect
#define USART_UDRIE UDRIE0
#define USART_UDRE_VECT USART_UDRE_vect
#define USART_UCSRC UCSR0C
#define USART_UCSZ1 UCSZ01
#define USART_UCSZ0 UCSZ00

#define USART_TIMER_TIMSK TIMSK0
#define USART_TIMER_OCIEA OCIE1A
#define USART_TIMER_TIFR TIFR0
#define USART_TIMER_OCFA OCF1A
#define USART_TIMER_TCNT TCNT1
#define USART_TIMER_TCCRA TCCR1A
#define USART_TIMER_TCCRB TCCR1B
#define USART_TIMER_CS0 CS10
#define USART_TIMER_OCRA OCR1A
#define USART_TIMER_COMPA_VECT TIMER1_COMPA_vect

#endif //USART_REGISTER_H

