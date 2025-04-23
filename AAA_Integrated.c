#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "uart.h"

void SPI_SlaveInit(void) {
    DDRC |= (1 << PC0);    // MISO output
    DDRC &= ~(1 << PC1);   // SCK input
    DDRE &= ~(1 << PE3);   // MOSI input
    DDRE &= ~(1 << PE2);   // SS input
    SPCR1 = (1 << SPE);    // Enable SPI1
}

uint8_t SPI_SlaveReceive(void) {
    while (!(SPSR1 & (1 << SPIF)));
    return SPDR1;
}

// --- Motor Setup ---
void setup_motorA() {
    DDRB |= (1 << DDB1) | (1 << DDB2);
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11) | (1 << CS10);
    ICR1 = 255;
    OCR1A = OCR1B = 0;
}

void setup_motorB() {
    DDRD |= (1 << DDD0) | (1 << DDD4);
    TCCR3A = (1 << COM3A1) | (1 << WGM31);
    TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS31) | (1 << CS30);
    ICR3 = 255;
    OCR3A = 0;
}

void setup_motorC() {
    DDRD |= (1 << DDD1) | (1 << DDD3);
    TCCR4A = (1 << COM4A1) | (1 << WGM41);
    TCCR4B = (1 << WGM42) | (1 << WGM43) | (1 << CS41) | (1 << CS40);
    ICR4 = 255;
    OCR4A = 0;
}

void reset_motors() {
    OCR1A = OCR1B = 0;
    OCR3A = 0;
    OCR4A = 0;
}

void move_forward(uint8_t duty) {
    reset_motors();
    OCR1B = duty;
    PORTD &= ~(1 << PD3);
    OCR4A = duty;
}

void move_backward(uint8_t duty) {
    reset_motors();
    OCR1A = duty;
    PORTD |= (1 << PD3);
    OCR4A = 255 - duty;
}

void move_left(uint8_t duty) {
    reset_motors();
    OCR1A = duty;
    PORTD &= ~(1 << PD4);
    OCR3A = 255 - 49;
    PORTD &= ~(1 << PD3);
    OCR4A = duty;
}

void move_right(uint8_t duty) {
    reset_motors();
    PORTD |= (1 << PD3);
    OCR4A = duty;
    PORTD |= (1 << PD4);
    OCR3A = 49;
    OCR1B = duty;
}

void stop_motors_AC() {
    OCR1A = OCR1B = 0;
    OCR3A = OCR4A = 0;
    PORTD |= (1 << PD3);
    PORTD |= (1 << PD4);
}

// --- Main Program ---
int main(void) {
    uart_init();
    SPI_SlaveInit();

    setup_motorA();
    setup_motorB();
    setup_motorC();

    uint8_t xl, xh, yl, yh;
    int16_t xa, ya;

    printf("Ready for wireless control...\n");

    while (1) {
        SPDR1 = 0x00;
        xl = SPI_SlaveReceive();
        SPDR1 = 0x00;
        xh = SPI_SlaveReceive();
        SPDR1 = 0x00;
        yl = SPI_SlaveReceive();
        SPDR1 = 0x00;
        yh = SPI_SlaveReceive();

        xa = (int16_t)((xl << 8) | xh);
        ya = (int16_t)((yl << 8) | yh);

        printf("XA: %d | YA: %d\r\n", xa, ya);

        if (xa < -7500 && ya > -4000 && ya < 4000) {
            printf("→ Forward\n");
            move_forward(255);
        } else if (xa > 7500 && ya > -4000 && ya < 4000) {
            printf("→ Backward\n");
            move_backward(255);
        } else if (xa > -4000 && xa < 4000 && ya > 7500) {
            printf("→ Right\n");
            move_right(255);
        } else if (xa > -4000 && xa < 4000 && ya < -7500) {
            printf("→ Left\n");
            move_left(255);
        } else {
            printf("→ Stop\n");
            stop_motors_AC();
        }

        _delay_ms(200);
    }
}
