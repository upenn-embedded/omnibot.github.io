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
 #include <avr/interrupt.h>
 #include "uart.h"
 
 #define dev_addr 0x6b // device address
 #define OUTX_L_A 0x28  
 #define OUTX_H_A 0x29
 #define OUTY_L_A 0x2A
 #define OUTY_H_A 0x2B
 #define OUTZ_L_A 0x2C
 #define OUTZ_H_A 0x2D
 #define OUTX_L_G 0x22
 #define OUTX_H_G 0x23
 #define OUTY_L_G 0x24
 #define OUTY_H_G 0x25
 #define OUTZ_L_G 0x26
 #define OUTZ_H_G 0x27
 #define CTRL1_XL 0x10
 #define CTRL2_G 0x11
 
 //imu setup
 void I2C_init() {
     TWBR0 = 0x48; // set the bit rate // 100kHz at 16MHz 
     // 100 kHz is the max of lsm i2c standard mode clock frequency
     // datasheet pg13
     // 100 kHz calculated using formula on atmega datasheet 256
     TWCR0 = (1 << TWEN); // enable TWI
     // possibly switch to interrupt based I2C ?
 }
 
 void start_con() {
     // send the start condition
     TWCR0 = (1<<TWINT)| (1<<TWSTA)|(1<<TWEN);
     while(!(TWCR0 & (1<<TWINT))); // wait for the flag to be set
 }
 
 void send(uint8_t data) {
     // send any data onto the I2C bus (could be device address or data)
     TWDR0 = data; 
     TWCR0 = (1<<TWINT) | (1<<TWEN);
     while (!(TWCR0 & (1<<TWINT)));
 }
 
 void stop_con() {
     TWCR0 = (1<<TWINT)| (1<<TWEN)|(1<<TWSTO);
 }
 
 void write_register(uint8_t reg, uint8_t data) {
     start_con(); // start condition
     send(dev_addr<<1); // send device address + W
     send(reg); // send register address
     send(data); // send data to be written
     stop_con();
 }
 
 uint8_t read_register(uint8_t reg) {
     start_con(); // start condition
     send(dev_addr<<1); // send device address + W
     send(reg); // send register address
     start_con(); // repeated start
     send((dev_addr<<1) | 1); // send device address + R
     send(reg); // send register address
     uint8_t data; 
     data = TWDR0; // read data from TWDR0 and return
     return data; 
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
     //clear motor 2, not needed to run forward and backward
     OCR3A = 0;
     OCR3B = 0;
     
     OCR1A = 0;
     OCR1B = duty;
 
     PORTD &= ~(1 << PD3); 
     OCR4A = duty;
 }
 
 void move_forward(uint8_t duty) {
     //clear motor 2 timer 3, not needed to run forward and backward
     OCR3A = 0;
     OCR3B = 0;
     
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
     //motor initialize
     setup_motorA();
     setup_motorB();
     setup_motorC();
     
     //imu intiialize
     I2C_init(); 
     uart_init(); 
     write_register(CTRL1_XL, (1<<6)); // 104 Hz set up accel
     write_register(CTRL2_G, (1<<6)); // 104 Hz set up gyro
     
     uint8_t x_l, x_h, y_l, y_h;
     int16_t x, y;
     
     
     while (1) {
         x_l = read_register(OUTX_L_A);
         x_h = read_register(OUTX_H_A);
         x = (int16_t)((x_h << 8) | x_l);
 
         y_l = read_register(OUTY_L_A);
         y_h = read_register(OUTY_H_A);
         y = (int16_t)((y_h << 8) | y_l);
 
         if (x < -7500 && y > -4000 && y < 4000) {
             move_forward(255);
         } else if (x > 7500 && y > -4000 && y < 4000) {
             move_backward(255);
         } else if (x > -4000 && x < 4000 && y > 7500) {
             move_right(255);
         } else if (x > -4000 && x < 4000 && y < -7500) {
             move_left(255);
         } else {
             stop_motors_AC();
         }
 
         _delay_ms(1000);
     }
 }
 