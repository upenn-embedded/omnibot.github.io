#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "uart.h" // or use your own USART init & putchar

void SPI_MasterInit() {
    DDRB |= (1<<PB3)|(1<<PB5)|(1<<PB2); // MOSI, SCK, CS as output
    DDRB &= ~(1<<PB4); // MISO as input
    SPCR0 = (1<<SPE)|(1<<MSTR)|(1<<SPR0); // Enable SPI, Master, clk/16
    PORTB |= (1<<PB2); // Set CS high (inactive)
}

uint8_t SPI_RxByte() {
    SPDR0 = 0x15; // Send dummy byte
    while (!(SPSR0 & (1<<SPIF)));
    return SPDR0;
}

int main() {
    uart_init();
    SPI_MasterInit();

    while (1) {
        PORTB &= ~(1<<PB2); // CS low
        _delay_us(10);      // Small delay before transaction

        uint8_t data = SPI_RxByte(); // Read 1 byte

        PORTB |= (1<<PB2); // CS high
        printf("Received: %d\n", data);

        _delay_ms(500);
    }
}