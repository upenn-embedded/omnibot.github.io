/*
 * File:   main.c
 * Author: ihuang
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
 
 
 void runMotorA() {
     // === Forward === (PWM on AIN1)
     OCR1A = ICR1 * 0.5;  // 50% duty
     OCR1B = 0;           // AIN2 LOW
     _delay_ms(1500);
 
     OCR1A = ICR1 * 0.1;
     _delay_ms(1000);
 
     OCR1A = ICR1 * 0.15;
     _delay_ms(1000);
 
     OCR1A = ICR1 * 0.05;
     _delay_ms(1000);
 
     OCR1A = 0;
     _delay_ms(1000);
 
     // === Reverse === (PWM on AIN2)
     OCR1A = 0;           // AIN1 LOW
     OCR1B = ICR1 * 0.5;
     _delay_ms(1500);
 
     OCR1B = ICR1 * 0.2;
     _delay_ms(1000);
 
     OCR1B = 0;
     _delay_ms(1000);
 }
 
 
 void runMotorA_fullspeed() {
     PORTB |= (1 << PB1);  // AIN1 HIGH
     PORTB &= ~(1 << PB2); // AIN2 LOW
 }
 void setup_motorA_manual() {
     DDRB |= (1 << DDB1) | (1 << DDB2); // PB1, PB2 as outputs
 }
 int main(void) {
     setup_motorA();
 
     while (1) {
         runMotorA(); // Loop motor movement
     }
 
     return 0;
 }