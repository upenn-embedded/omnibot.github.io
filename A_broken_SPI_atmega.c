#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "uart.h" 

void SPI_SlaveInit(void) {
    DDRC |= (1<<PC0);   // MISO output
    DDRC &= ~(1<<PC1);  // SCK input
    DDRE &= ~(1<<PE3);  // MOSI input
    DDRE &= ~(1<<PE2);  // SS input
    SPCR1 = (1<<SPE);   // Enable SPI1
}

uint8_t SPI_SlaveReceive(void) {
    while (!(SPSR1 & (1<<SPIF))); // Wait for data
    return SPDR1;
}

int main(void) {
    uart_init();
    SPI_SlaveInit(); 

    uint8_t xl, xh, yl, yh;
    int16_t xa, ya;
    
    printf("start\n");
    
    while (1) {
    printf("waiting xl\n");
    SPDR1 = 0x00;
    xl = SPI_SlaveReceive();
    
    printf("waiting xh\n");
    SPDR1 = 0x00;
    xh = SPI_SlaveReceive();
    
    printf("waiting yl\n");
    SPDR1 = 0x00;
    yl = SPI_SlaveReceive();
    
    printf("waiting yh\n");
    SPDR1 = 0x00;
    yh = SPI_SlaveReceive();
    
    xa = (int16_t)((xh << 8) | xl);
    ya = (int16_t)((yh << 8) | yl);

    printf("x: %d | y: %d\r\n", xa, ya);
}

}
