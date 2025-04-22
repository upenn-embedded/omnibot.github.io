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
 
 //void motor3_forward() {
 //    PORTD |= (1 << PD1);   // AIN1 = HIGH
 //    PORTD &= ~(1 << PD3);  // AIN2 = LOW
 //}
 //void motor3_reverse() {
 //    PORTD &= ~(1 << PD1);  // AIN1 = LOW
 //    PORTD &= ~(1 << PD3);  // AIN2 = HIGH
 //}
 //void motor3_stop() {
 //    PORTD &= ~(1 << PD1);  // AIN1 = LOW
 //    PORTD &= ~(1 << PD3);  // AIN2 = LOW ? coast
 //}
 void reset_motors() {
     // Motor A (Timer1)
     OCR1A = 0;
     OCR1B = 0;
 
     // Motor B (Timer3)
     OCR3A = 0;
     OCR3B = 0;
 //    PORTD |= (1 << PD4); 
 
     // Motor C (Timer4)
     OCR4A = 0;
 //    PORTD |= (1 << PD3); 
 }
 
 void move_backward(uint8_t duty) {
     reset_motors();
     
     OCR1A = duty;
     OCR1B = 0;
 
     PORTD |= (1 << PD3); 
     OCR4A = 255 - duty;
 }
 
 void move_forward(uint8_t duty) {
     reset_motors();
 
     // Motor 1 forward
     OCR1A = 0;
     OCR1B = duty;
     
     PORTD &= ~(1 << PD3);  
     OCR4A = duty;  
 }
 void move_right() {
     reset_motors();
 
     PORTD |= (1 << PD3); 
     OCR4A = 90;
     
     PORTD |= (1 << PD4);   // DIR HIGH ? reverse
     OCR3A = 49;  
     
     while (1) {
         OCR1A = 0;
         OCR1B = 48;
         _delay_ms(2000);
         OCR1A = 48;
         OCR1B = 0;
         _delay_ms(500);
     }
 }
 
 void move_right_notGood() {
     reset_motors();
 
     PORTD |= (1 << PD3); 
     OCR4A = 55;
     
     PORTD |= (1 << PD4);   // DIR HIGH ? reverse
     OCR3A = 255-49;  
     
     while (1) {
         OCR1A = 0;
         OCR1B = 48;
         _delay_ms(1250);
         OCR1B = 0;
         _delay_ms(1500);
     }
 }
 void move_left() {
     reset_motors();
 
     OCR1A = 235;
     OCR1B = 0;
 
     PORTD &= ~(1 << PD4);  // DIR LOW
     OCR3A = 255-49;
 
     while (1) {
         PORTD &= ~(1 << PD3);  // DIR LOW
         OCR4A = 55;
         _delay_ms(1500);
         
         PORTD |= (1 << PD3); 
         OCR4A = 185;
         _delay_ms(500);
     }
 }
 
 void move_left_notGood() {
     reset_motors();
     
     OCR1A = 204;
     OCR1B = 0;
 
     PORTD &= ~(1 << PD3);  
     OCR4A = 50; 
     
     //motor 3
     PORTD &= ~(1 << PD4);  // DIR LOW ? forward
     OCR3A = 255 - 195; 
 }
 
 void stop_motors_AC() {
     OCR1A = 0;
     OCR1B = 0;
     OCR3A = 0;
     OCR3B = 0;
     OCR4A = 0;
     PORTD |= (1 << PD3); 
     PORTD |= (1 << PD4); 
 }
 
 int main(void) {
     //motor initialize
     setup_motorA();
     setup_motorB();
     setup_motorC();
     
     //imu intiialize
     I2C_init(); 
 //    uart_init(); 
     write_register(CTRL1_XL, (1<<6)); // 104 Hz set up accel
     write_register(CTRL2_G, (1<<6)); // 104 Hz set up gyro
     
     SPI_SlaveInit(); 
     
 //    uint8_t rxData; 
     
     uint8_t x_l, x_h, y_l, y_h;
     int16_t x, y;
    
     while (1) {
         SPDR1 = 0x22; // data to send to esp32
 //        rxData = SPI_SlaveReceive();
         
         //data from esp for IMU
         x_l = SPI_SlaveReceive();
         x_h = SPI_SlaveReceive();
         x = (int16_t) (x_h<<8) | (x_l); // combine high and low registers
         //printf("%d\n", x);
         //printf("x: %d \r\n", x);
         y_l = SPI_SlaveReceive();
         y_h = SPI_SlaveReceive();
         y = (int16_t) (y_h<<8) | (y_l);
               
 
         if (x < -7500 && y > -4000 && y < 4000) {
             printf("forward\n");
             move_forward(255);
         } else if (x > 7500 && y > -4000 && y < 4000) {
             printf("backward\n");
             move_backward(255);
         } else if (x > -4000 && x < 4000 && y > 7500) {
             printf("right\n");
             move_right(255);
         } else if (x > -4000 && x < 4000 && y < -7500) {
             printf("left\n");
             move_left(255);
         } else {
             printf("stop\n");
             stop_motors_AC();
         }
 
         _delay_ms(1000);
     }
 }
 
 
 
   
 