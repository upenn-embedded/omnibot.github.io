
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

typedef struct data_packet { // class for the data to be sent
  uint16_t xa;
  uint16_t ya;
  uint16_t za;
  uint16_t xg;
  uint16_t yg;
  uint16_t zg;
} data_packet;

data_packet data; 

esp_now_peer_info_t receiver; 

uint8_t rxMACaddr[] = {0x60,0x55,0xF9,0xEB,0xDC,0xBA}; // feather s2
// tx mac addr: 10:06:1c:18:d0:5c (huzzah)

void setup() {
  Serial.begin(115200); 
  WiFi.mode(WIFI_STA); // set as wifi station

  memcpy(receiver.peer_addr, rxMACaddr, 6);
  receiver.channel = 0;  
  receiver.encrypt = false;

  esp_now_init(); 

  esp_now_add_peer(&receiver);

}

void loop() {
  data.xa = (uint16_t) random(1,100);
  data.ya = 12;
  data.za = 12;
  data.xg = 12;
  data.yg = 12;
  data.zg = 12;

  esp_now_send(rxMACaddr, (uint8_t *) &data, sizeof(data));

  delay(500); 
}