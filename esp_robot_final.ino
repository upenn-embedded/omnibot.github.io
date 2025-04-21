#include "SPI.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// this is the receiver on the robot

typedef struct data_packet { // class for the data to be sent
  uint16_t xa;
  uint16_t ya;
} data_packet;

data_packet data;


//uint8_t txMACaddr[] = {0x10,0x06,0x1c,0x18,0xd0,0x5c}; (huzzah)
// rx mac addr: 0x60,0x55,0xF9,0xEB,0xDC,0xBA (feather s2)

void data_received(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // what to do once data gets received
  // copy incoming data over into the data packet
  memcpy(&data, incomingData, sizeof(data));
  Serial.println((int16_t) data.xa);
}

uint8_t rxData; // data received from atmega don't really care about
// set up as master

void setup() {
  SPI.begin();
  Serial.begin(115200);
  // CS = pin 42 labeled as 38 on feather s2

  WiFi.mode(WIFI_STA); 
  esp_now_init(); 

  esp_now_register_recv_cb(esp_now_recv_cb_t(data_received)); // create and register the call back function so its called on each packet recieved

}

uint8_t firstByte (uint16_t var) {
    return (uint8_t) (var>>8 & 0xFF);
}

uint8_t lastByte (uint16_t var) { 
  return (uint8_t) var & 0xFF;
}

uint8_t xl;
uint8_t xh;
uint8_t yl;
uint8_t yh;

void loop() {
  
  xl = firstByte(data.xa);
  xh = lastByte(data.xa);
  yl = firstByte(data.ya);
  yh = lastByte(data.ya);

  digitalWrite(SS, LOW); // pull CS low to start transaction
  rxData = SPI.transfer(xl);
  digitalWrite(SS,HIGH); // end transmission

  digitalWrite(SS, LOW); // pull CS low to start transaction
  rxData = SPI.transfer(xh);
  digitalWrite(SS,HIGH); // end transmission

  
  digitalWrite(SS, LOW); // pull CS low to start transaction
  rxData = SPI.transfer(yl);
  digitalWrite(SS,HIGH); // end transmission

  digitalWrite(SS, LOW); // pull CS low to start transaction
  rxData = SPI.transfer(yh); 
  digitalWrite(SS,HIGH); // end transmission
  
  
  delay(500); 
}
