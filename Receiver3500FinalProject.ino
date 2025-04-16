
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


//uint8_t txMACaddr[] = {0x10,0x06,0x1c,0x18,0xd0,0x5c}; (huzzah)
// rx mac addr: 0x60,0x55,0xF9,0xEB,0xDC,0xBA (feather s2)

void data_received(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // what to do once data gets received
  // copy incoming data over into the data packet
  memcpy(&data, incomingData, sizeof(data));
  Serial.println(data.xa);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); 
  esp_now_init(); 

  esp_now_register_recv_cb(esp_now_recv_cb_t(data_received)); // create and register the call back function so its called on each packet recieved
}

void loop() {

}