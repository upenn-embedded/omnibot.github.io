#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "uart.h" // or use your own USART init & putchar

void Initialize() {
    DDRB |= (1<<PB2) | (1<<PB3) | (1<<PB4) | (1<<PB5); // set all as outputs
}

int main() {
    uart_init();
    Initialize(); 

    while (1) { // just cycle through different directions
        PORTB |= (1<<PB5); 
        printf("fwd");
        _delay_ms(500);
        PORTB &= ~(1<<PB5);
        PORTB |= (1<<PB4); 
        printf("rev");
        _delay_ms(500);
        PORTB &= ~(1<<PB4);
        PORTB |= (1<<PB3); 
        printf("left");
        _delay_ms(500);
        PORTB &= ~(1<<PB3);
        PORTB |= (1<<PB2); 
        printf("right");
        _delay_ms(500);
        PORTB &= ~(1<<PB2);
    }
}