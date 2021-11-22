# ATmega USART interrupt controlled communication

A USART communication implemantaion using interrupts an ring buffers to allow for asynchronous sending an receiving of messages.

This software was so far tested for the ATmega 328p. The registers used are the ones needed for enabling communication on an ATmega 328p. When using this code you may need to change the used registers depending on which microcontroller you are using.
