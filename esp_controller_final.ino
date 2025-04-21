#include "Wire.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

typedef struct data_packet { // class for the data to be sent
  uint16_t xa;
  uint16_t ya;
} data_packet;

data_packet data;

esp_now_peer_info_t receiver; 

uint8_t rxMACaddr[] = {0x60,0x55,0xF9,0xEB,0xDC,0xBA}; // feather s2

#define dev_addr 0x6b // device address
#define OUTX_L_A 0x28  
#define OUTX_H_A 0x29
#define OUTY_L_A 0x2A
#define OUTY_H_A 0x2B

#define CTRL1_XL 0x10

void writeRegister(uint8_t reg_address, uint8_t data) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_address);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg_address) {
  uint8_t data;
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_address);
  Wire.requestFrom(dev_addr,1); // sends repeated start, want one byte from imu
  data = Wire.read();
  Wire.endTransmission();
  return data;
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); 
  Serial.begin(115200);
  writeRegister(CTRL1_XL, (1<<6)); // set up accel

  WiFi.mode(WIFI_STA); // set as wifi station

  memcpy(receiver.peer_addr, rxMACaddr, 6);
  receiver.channel = 0;  
  receiver.encrypt = false;

  esp_now_init(); 

  esp_now_add_peer(&receiver);
}

uint8_t xl;
uint8_t xh;
uint8_t yl;
uint8_t yh;

void loop() {
  xl = readRegister(OUTX_L_A);
  xh = readRegister(OUTX_H_A);
  yl = readRegister(OUTY_L_A);
  yh = readRegister(OUTY_H_A);

  data.xa = (int16_t) (xh<<8) | (xl);
  data.ya = (int16_t) (yh<<8) | (yl);

  esp_now_send(rxMACaddr, (uint8_t *) &data, sizeof(data));

  Serial.println(xl);
  delay(500);
}
