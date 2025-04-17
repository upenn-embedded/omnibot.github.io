/*
 * File:   main.c
 * Author: ihuan
 *
 * Created on April 11, 2025, 11:42 AM
 */

 #define F_CPU 16000000UL

 #include <avr/io.h>
 #include <xc.h>
 #include <util/delay.h>
 
 void setup_motorA() {
     // Set PB1 and PB2 as outputs for OC1A and OC1B
     DDRB |= (1 << DDB1) | (1 << DDB2);
 
     // Fast PWM Mode 14: WGM13:0 = 1110 (TOP = ICR1)
     TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // Non-inverting on OC1A/B
     TCCR1A |= (1 << WGM11);
     TCCR1B |= (1 << WGM12) | (1 << WGM13);
 
     // Prescaler = 64 (for ~1 kHz frequency with TOP = 255 at 16 MHz)
     TCCR1B |= (1 << CS11) | (1 << CS10);
 
     // Set TOP value (PWM frequency control)
     ICR1 = 255;
 
     // Start with motor off
     OCR1A = 0;
     OCR1B = 0;
 }
 
 void setup_motorB() {
     // Set PD0 (OC3A) and PD2 (OC3B) as outputs
     DDRD |= (1 << DDD0) | (1 << DDD2);
 
     // Clear previous settings
     TCCR3A = 0;
     TCCR3B = 0;
 
     // Fast PWM Mode 14 (TOP = ICR3)
     TCCR3A |= (1 << COM3A1) | (1 << COM3B1) | (1 << WGM31); // Non-inverting
     TCCR3B |= (1 << WGM32) | (1 << WGM33);
 
     // Prescaler = 64
     TCCR3B |= (1 << CS31) | (1 << CS30);
 
     // Set TOP value for PWM frequency (approx 1 kHz)
     ICR3 = 255;
 
     // Start motor off
     OCR3A = 0;
     OCR3B = 0;
 }
 
 void setup_motorC() {
     // Set PD1 (OC4A) as output for PWM
     // Set PD3 as output for direction
     DDRD |= (1 << DDD1) | (1 << DDD3);
 
     // Fast PWM Mode 14 for Timer4 (TOP = ICR4)
     TCCR4A = 0;
     TCCR4B = 0;
     TCCR4A |= (1 << COM4A1) | (1 << WGM41);
     TCCR4B |= (1 << WGM42) | (1 << WGM43);
     TCCR4B |= (1 << CS41) | (1 << CS40); // Prescaler = 64
 
     ICR4 = 255;    // 8-bit-style PWM
     OCR4A = 0;     // Start with motor off
 }
 
 void move_backward(uint8_t duty) {
     OCR1A = 0;
     OCR1B = duty;
 
     PORTD &= ~(1 << PD3); 
     OCR4A = duty;
 }
 
 void move_forward(uint8_t duty) {
     // motor 3 needs to be inverted, 0 means 255 and 255 means 0.
     PORTD |= (1 << PD3);  
     OCR4A = 255 - duty;
 
     OCR1A = duty;
     OCR1B = 0;    
 }
 
 void stop_motors_AC() {
     OCR1A = 0;
     OCR1B = 0;
     OCR3A = 0;
     OCR3B = 0;
     PORTD &= ~(1 << PD3); 
     OCR4A = 0;
 }
 
 int main(void) {
     setup_motorA();
     setup_motorB();
     setup_motorC();
 
     while (1) {
         //moving forward full speed
         move_forward(255);
         _delay_ms(2000);
         stop_motors_AC();
         _delay_ms(1500);
         
         //moving backward full speed
         move_backward(255);
         _delay_ms(2000);
         stop_motors_AC();
         _delay_ms(1500);
     }
     return 0;
 }
 