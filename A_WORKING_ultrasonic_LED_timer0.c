#define F_CPU 16000000UL
#include "uart.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define TRIG_PIN PC2
#define ECHO_PIN PC3
#define BUZZER_PIN PD6

volatile uint8_t overflow_counter = 0;
volatile uint8_t timer_ready = 0;

void init() {
    cli();

    DDRC |= (1 << TRIG_PIN);      // TRIG as output
    DDRC &= ~(1 << ECHO_PIN);     // ECHO as input
    DDRD |= (1 << BUZZER_PIN);    // Buzzer pin as output

    // Timer 2: normal mode, prescaler = 64
    TCCR2A = 0x00;
    TCCR2B = (1 << CS22); // Prescaler = 64
    TIMSK2 = 0x00;
    //TIMSK2 = (1 << TOIE2); // Enable overflow interrupt

    sei();
}

void send_pulse() {
    PORTC |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTC &= ~(1 << TRIG_PIN);
}


uint16_t measure_distance() {
    uint16_t ticks;
    overflow_counter = 0;
    TCNT2 = 0;

    while (!(PINC & (1 << ECHO_PIN)));  // Wait for rising edge

    while (PINC & (1 << ECHO_PIN)) {    // Time high pulse
        if (TCNT2 == 255) {
            overflow_counter++;
            TCNT2 = 0;
        }
    }

    ticks = (overflow_counter * 256) + TCNT2;
    uint32_t time_us = ticks * 4; // 4 µs per tick at prescaler 64
    return (uint16_t)(time_us / 58);   // cm
}



void control_buzzer(uint16_t distance_cm) {
    if (distance_cm < 25) {
        PORTD |= (1 << BUZZER_PIN);  // Turn buzzer ON
    } else {
        PORTD &= ~(1 << BUZZER_PIN); // Turn buzzer OFF
    }
}

int main(void) {
    uart_init();
    init();

    while (1) {
        printf("A\n");
        send_pulse();
        uint16_t dist = measure_distance();
        printf("Distance: %u cm\r\n", dist);
        control_buzzer(dist);
        _delay_ms(400);
    }
    return 0;
}