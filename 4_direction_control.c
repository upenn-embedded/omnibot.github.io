/*
 * File:   main.c
 * Author: ihuan
 *
 * Created on April 11, 2025, 11:42 AM
 */

 #define F_CPU 16000000UL

 #include <avr/io.h>
 //#include <avr/portmux.h>
 #include <xc.h>
 #include <util/delay.h>
 
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
     DDRD |= (1 << DDD0) | (1 << DDD4);
 
     TCCR3A = (1 << COM3A1) | (1 << WGM31);
     TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS31) | (1 << CS30);
 
     ICR3 = 255;
     OCR3A = 0;
 }
 
 void setup_motorC() {
     DDRD |= (1 << DDD1) | (1 << DDD3);
 
     TCCR4A = 0;
     TCCR4B = 0;
     TCCR4A |= (1 << COM4A1) | (1 << WGM41);
     TCCR4B |= (1 << WGM42) | (1 << WGM43);
     TCCR4B |= (1 << CS41) | (1 << CS40); 
 
     ICR4 = 255;
     OCR4A = 0; 
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
 
 void move_right() {
     OCR1A = 0;
     OCR1B = 49;
 
     PORTD |= (1 << PD4);  
     OCR3A = 255 - 221;
 
     PORTD |= (1 << PD3);    
     OCR4A = 255 - 195;       
 }
 
 void move_left() {
     OCR1A = 204;
     OCR1B = 0;
 
     PORTD &= ~(1 << PD4);
     OCR3A = 221;
     
     //motor 3
     PORTD &= ~(1 << PD3);    
     OCR4A = 49;
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
         move_forward(255);
         _delay_ms(5000);
         move_backward(255);
         _delay_ms(5000);
         move_left(255);
         _delay_ms(5000);
         move_right(255);
         _delay_ms(5000);
     }
 }
 