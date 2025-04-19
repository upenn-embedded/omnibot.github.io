/*
esp32 on the controller

send data from the atmega to the esp32

just use gpio bc spi not working lol
pin for each of the different directions

*/
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

typedef struct data_packet { // class for the data to be sent
  uint8_t dir;
} data_packet;

data_packet data; 

uint8_t dir = 0; 
  // 0 = no move
  // 1 = fwd
  // 2 = rev
  // 3 = left
  // 4 = rigth

esp_now_peer_info_t receiver; 

uint8_t rxMACaddr[] = {0x10,0x06,0x1c,0x18,0xd0,0x5c}; //(huzzah) //{0x60,0x55,0xF9,0xEB,0xDC,0xBA}; // feather s2
// tx mac addr: 10:06:1c:18:d0:5c (huzzah)

void setup() {
  Serial.begin(115200); 

  pinMode(33, INPUT); // fwd 
  pinMode(1, INPUT); // rev
  pinMode(7,INPUT); // left
  pinMode(11, INPUT); // right

  WiFi.mode(WIFI_STA); // set as wifi station

  memcpy(receiver.peer_addr, rxMACaddr, 6);
  receiver.channel = 0;  
  receiver.encrypt = false;

  esp_now_init(); 

  esp_now_add_peer(&receiver);
}

void loop() {
    // assume only one pin will be high at a time

    if(digitalRead(33) == HIGH) {
      dir = 1;
    } else if (digitalRead(1) == HIGH) {
      dir = 2;
    } else if (digitalRead(7) == HIGH) {
      dir = 3;
    } else if(digitalRead(11) == HIGH) {
      dir = 4;
    } else {
      dir = 0;
    }

    data.dir = dir;
    esp_now_send(rxMACaddr, (uint8_t *) &data, sizeof(data)); // send data

    delay(500); 
  
}