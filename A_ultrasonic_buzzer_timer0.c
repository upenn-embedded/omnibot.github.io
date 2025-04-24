/*
Uses PC2 and PC3 as trig and echo.
Uses 8-bit TIMER0 instead of TIMER2.
*/

#define F_CPU 16000000UL
#include "uart.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define TRIG_PIN PC2
#define ECHO_PIN PC3
#define BUZZER_PIN PD6

volatile uint8_t overflow_counter = 0;

void init() {
    cli();

    DDRC |= (1 << TRIG_PIN);      // TRIG as output (PC2)
    DDRC &= ~(1 << ECHO_PIN);     // ECHO as input  (PC3)
    DDRD |= (1 << BUZZER_PIN);    // Buzzer pin as output (PD6)

    // Timer 0: normal mode, prescaler = 64
    TCCR0A = 0x00;
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler = 64
    TIMSK0 = (1 << TOIE0);             // Enable overflow interrupt

    sei();
}

void send_pulse() {
    PORTC |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTC &= ~(1 << TRIG_PIN);
}

ISR(TIMER0_OVF_vect) {
    overflow_counter++;
}

uint16_t measure_distance() {
    uint16_t ticks;
    overflow_counter = 0;
    TCNT0 = 0;

    // Wait for echo to go HIGH
    while (!(PINC & (1 << ECHO_PIN)));

    // Start timing while echo is HIGH
    while (PINC & (1 << ECHO_PIN)) {
        // Timer0 is running in background
    }

    ticks = (overflow_counter * 256) + TCNT0;
    uint32_t time_us = ticks * 4; // 4 µs per tick with prescaler = 64
    return (uint16_t)(time_us / 58); // convert microseconds to cm
}

void control_buzzer(uint16_t distance_cm) {
    if (distance_cm > 15) {
        PORTD |= (1 << BUZZER_PIN);  // Turn buzzer ON
    } else {
        PORTD &= ~(1 << BUZZER_PIN); // Turn buzzer OFF
    }
}

int main(void) {
    uart_init();
    init();

    while (1) {
        send_pulse();
        uint16_t dist = measure_distance();
        printf("Distance: %u cm\r\n", dist);
        control_buzzer(dist);
        _delay_ms(400);
    }
    return 0;
}
