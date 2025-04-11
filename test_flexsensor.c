#define F_CPU               16000000UL

#include <xc.h>
#include "uart.h"
#include <util/delay.h>
#include <stdio.h>

#define UART_BAUD_RATE      74880
#define UART_BAUD_PRESCALER (((F_CPU / (UART_BAUD_RATE * 16UL))) - 1)


void Initialize() {
    // Setup for ADC (10bit = 0-1023)
    // Clear power reduction bit for ADC
    PRR0 &= ~(1 << PRADC);

    // Select Vref = AVcc
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);

    // Set the ADC clock div by 128
    // 16M/128 = 125kHz
    ADCSRA |= (1 << ADPS0);
    ADCSRA |= (1 << ADPS1);
    ADCSRA |= (1 << ADPS2);

    // Select Channel ADC0 (PC0) for flex sensor input
    ADMUX &= ~(1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADMUX &= ~(1 << MUX2);
    ADMUX &= ~(1 << MUX3);

    ADCSRA |= (1 << ADATE); // Autotriggering of ADC

    // Free running mode ADTS[2:0] = 000
    ADCSRB &= ~(1 << ADTS0);
    ADCSRB &= ~(1 << ADTS1);
    ADCSRB &= ~(1 << ADTS2);

    // Disable digital input buffer on ADC0 (PC0)
    DIDR0 |= (1 << ADC0D);

    // Enable ADC
    ADCSRA |= (1 << ADEN);

    // Start conversion
    ADCSRA |= (1 << ADSC);
}

int main(void) {
    Initialize();
    UART_init(UART_BAUD_PRESCALER);
    while (1) {
        char intStringBuffer[20]; // Buffer to hold the converted number
        sprintf(intStringBuffer, "Flex ADC:\t %d", ADC); // Read flex sensor value from ADC0
        __PRINT_NEW_LINE__ // Make space between prints
        UART_putstring(intStringBuffer);
        _delay_ms(2000);
    }
}