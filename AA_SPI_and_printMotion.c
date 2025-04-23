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
    int16_t x, y;
    
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
    
        x = (int16_t)((xh << 8) | xl);
        y = (int16_t)((yh << 8) | yl);

    
        // Debug output for raw bytes and reconstructed values
        printf("Raw Bytes → XL: %u  XH: %u | YL: %u  YH: %u\r\n", xl, xh, yl, yh);
        printf("Parsed Accel → X: %d | Y: %d\r\n", x, y);
        
        if (x < -7500 && y > -4000 && y < 4000) {
             printf("forward\n");
//             move_forward(255);
         } else if (x > 7500 && y > -4000 && y < 4000) {
             printf("backward\n");
//             move_backward(255);
         } else if (x > -4000 && x < 4000 && y > 7500) {
             printf("right\n");
//             move_right(255);
         } else if (x > -4000 && x < 4000 && y < -7500) {
             printf("left\n");
//             move_left(255);
         } else {
             printf("stop\n");
//             stop_motors_AC();
         }
        
        _delay_ms(1000);
    }    

}
