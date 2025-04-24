#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "uart.h" 
#include <avr/interrupt.h>

#define TRIG_PIN PC2
#define ECHO_PIN PC3
#define BUZZER_PIN PD6

//setting up ultrasonic sensor
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


//states of the direction motor
typedef enum {
    DIR_STOP,
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT
} Direction;

Direction current_direction = DIR_STOP;

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

//motor setup
 void setup_motorA() {
     DDRB |= (1 << DDB1) | (1 << DDB2);
 
     TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
     TCCR1A |= (1 << WGM11);
     TCCR1B |= (1 << WGM12) | (1 << WGM13);
 
     TCCR1B |= (1 << CS11) | (1 << CS10);
 
     ICR1 = 255;
 
     OCR1A = 0;
     OCR1B = 0;
 }
 
 void setup_motorB() {
     // Set PD0 (OC3A) for PWM, PD4 for direction
     DDRD |= (1 << DDD0) | (1 << DDD4);
 
     // Timer3 setup: Fast PWM Mode 14 using ICR3 as TOP
     TCCR3A = (1 << COM3A1) | (1 << WGM31);                   // Non-inverting PWM on OC3A
     TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS31) | (1 << CS30);                      // Prescaler = 64
 
     ICR3 = 255;      // Set TOP value
     OCR3A = 0;       // Motor off initially
 }
 
 void setup_motorC() {
     // Set PD1 (OC4A) as PWM output, PD3 as DIR output
     DDRD |= (1 << DDD1) | (1 << DDD3);
 
     // Timer4 setup: Fast PWM mode 14, non-inverting on OC4A (PD1)
     TCCR4A = (1 << COM4A1) | (1 << WGM41);                   // Non-inverting PWM on OC4A
     TCCR4B = (1 << WGM42) | (1 << WGM43)                     // Fast PWM mode using ICR4 as TOP
            | (1 << CS41) | (1 << CS40);                      // Prescaler = 64
 
     ICR4 = 255;      // Set PWM TOP value (8-bit style)
     OCR4A = 0;       // Start with motor off
 }
 
 
  void stop_motors_AC() {
     if (current_direction != DIR_STOP) {
        OCR1A = 0;
        OCR1B = 0;
        OCR3A = 0;
        OCR4A = 0;
        PORTD &= ~(1 << PD3); 
        PORTD &= ~(1 << PD4); 
        current_direction = DIR_STOP;
    }
 }
  
 void move_forward(uint8_t duty) {
     if (current_direction != DIR_FORWARD) {
        stop_motors_AC();
        current_direction = DIR_FORWARD;
    }
     
     OCR1A = duty;
     OCR1B = 0;
 
     PORTD |= (1 << PD3); 
     OCR4A = 255 - duty;
 }
 
 void move_backward(uint8_t duty) {
     if (current_direction != DIR_BACKWARD) {
        stop_motors_AC();
        current_direction = DIR_BACKWARD;
    }
 
     // Motor 1 forward
     OCR1A = 0;
     OCR1B = duty;
     
     PORTD &= ~(1 << PD3);  
     OCR4A = duty;  
 }
 void move_right() {
     if (current_direction != DIR_RIGHT) {
        stop_motors_AC();
        current_direction = DIR_RIGHT;
    }
 
     PORTD |= (1 << PD3); 
     OCR4A = 90;
     
     PORTD |= (1 << PD4);   // DIR HIGH ? reverse
     OCR3A = 49;  
     
     OCR1A = 0;
     OCR1B = 0;
 }
 
 void move_left() {
     if (current_direction != DIR_LEFT) {
        stop_motors_AC();
        current_direction = DIR_LEFT;
    }
 
     OCR1A = 235;
     OCR1B = 0;
 
     PORTD &= ~(1 << PD4);  // DIR LOW
     OCR3A = 255-49;
     
     PORTD &= ~(1 << PD4); 
 }
 

 void led(){
     PORTD |= (1 << PD6);
     _delay_ms(500);
     PORTD &= ~(1 << PD6);
 }
 
int main(void) {
//    uart_init();
    
    //ultrasonic sensor
    init();
    
    //motor initialize
     setup_motorA();
     setup_motorB();
     setup_motorC();
     
    SPI_SlaveInit(); 

    uint8_t xl, xh, yl, yh;
    int16_t x, y;
    
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

        
        send_pulse();
        uint16_t dist = measure_distance();

        uint16_t dist = measure_distance();

        if (dist < 20) {
            PORTD |= (1 << BUZZER_PIN);  // Buzzer ON

            if (x > 7500 && y > -4000 && y < 4000) {
                move_backward(255);
            } else if (x > -4000 && x < 4000 && y > 7500) {
                move_right();
            } else if (x > -4000 && x < 4000 && y < -7500) {
                move_left();
            } else {
                stop_motors_AC();  // Forward or no clear gesture → STOP
            }

        } else {
            PORTD &= ~(1 << BUZZER_PIN);  // Buzzer OFF

            if (x < -7500 && y > -4000 && y < 4000) {
                move_forward(255);
            } else if (x > 7500 && y > -4000 && y < 4000) {
                move_backward(255);
            } else if (x > -4000 && x < 4000 && y > 7500) {
                move_right();
            } else if (x > -4000 && x < 4000 && y < -7500) {
                move_left();
            } else {
                stop_motors_AC();
            }
        }
        
        _delay_ms(1000);
    }    

}
