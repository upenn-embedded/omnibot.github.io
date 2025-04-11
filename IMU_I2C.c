#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
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


int main() {
    I2C_init(); 
    uart_init(); 
    write_register(CTRL1_XL, (1<<6)); // 104 Hz set up LSM
    uint8_t x_l; 
    uint8_t x_h;
    int16_t x; 
    uint8_t y_l; 
    uint8_t y_h;
    int16_t y;
    uint8_t z_l; 
    uint8_t z_h;
    int16_t z;
    
    int16_t xg;
    int16_t yg;
    int16_t zg;
    
    while(1) {
        x_l = read_register(OUTX_L_A);
        x_h = read_register(OUTX_H_A);
        x = (int16_t) (x_h<<8) | (x_l); // combine high and low registers
        //printf("%d\n", x);
        //printf("x: %d \r\n", x);
        y_l = read_register(OUTY_L_A);
        y_h = read_register(OUTY_H_A);
        y = (int16_t) (y_h<<8) | (y_l);
        //printf("y: %d \r\n", y);
        z_l = read_register(OUTZ_L_A);
        z_h = read_register(OUTZ_H_A);
        z = (int16_t) (z_h<<8) | (z_l);
        //printf("z: %d \r\n", z);
        
        xg = (int16_t) (read_register(OUTX_H_G)<<8) | read_register(OUTX_L_G);
        yg = (int16_t) (read_register(OUTY_H_G)<<8) | read_register(OUTY_L_G);
        zg = (int16_t) (read_register(OUTZ_H_G)<<8) | read_register(OUTZ_L_G);
        
        printf("%d, %d, %d, %d, %d, %d \r\n", x, y, z, xg, yg, zg);
        
        // gyro showing all zero, need to write to ctrl register to activate gyro

        _delay_ms(50);
    }
}
