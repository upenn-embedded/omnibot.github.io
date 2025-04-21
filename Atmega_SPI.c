#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "uart.h" 


    // set up SPI1 as slave
    // using SPI1 because timers are using PB2
    // need to solder port e
    // set CS as input

void SPI_SlaveInit(void)
{
   /* Set MISO output, all others input */
   DDRC = (1<<PC0); // MISO / SDO
   DDRC &= ~(1<<PC1); //SCK
   DDRE &= ~(1<<PE3); //MOSI / SDI
   DDRE &= ~(1<<PE2); //CS
   /* Enable SPI */
   SPCR1 = (1<<SPE);
   // don't set mstr bit
}
char SPI_SlaveReceive(void)
{
   /* Wait for reception complete */
   while(!(SPSR1 & (1<<SPIF)));
   /* Return Data Register */
   return SPDR1;
}
int main() {
    uart_init();
    SPI_SlaveInit(); 
    
    uint8_t rxData; 

    while (1) { 
        SPDR1 = 0x22; // data to send to esp32
        rxData = SPI_SlaveReceive();
        printf("%d\n", rxData);
    }
}