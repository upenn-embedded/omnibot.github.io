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
        SPDR1 = 0x00;
        xl = SPI_SlaveReceive();  // HIGH byte of X
    
        SPDR1 = 0x00;
        xh = SPI_SlaveReceive();  // LOW byte of X
    
        SPDR1 = 0x00;
        yl = SPI_SlaveReceive();  // HIGH byte of Y
    
        SPDR1 = 0x00;
        yh = SPI_SlaveReceive();  // LOW byte of Y
    
        xa = (int16_t)((xl << 8) | xh);
        ya = (int16_t)((yl << 8) | yh);

    
        // Debug output for raw bytes and reconstructed values
        printf("Raw Bytes → XL: %u  XH: %u | YL: %u  YH: %u\r\n", xl, xh, yl, yh);
        printf("Parsed Accel → X: %d | Y: %d\r\n", xa, ya);
    }    

}
