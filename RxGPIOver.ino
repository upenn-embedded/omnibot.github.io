/*
esp32 on the robot

send data to atmega

use gpio into a resistive divider and adc on the atmega
*/

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

typedef struct data_packet { // class for the data to be sent
  uint8_t dir;
} data_packet;

data_packet data; 


void data_received(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // what to do once data gets received
  // copy incoming data over into the data packet
  memcpy(&data, incomingData, sizeof(data));
  Serial.println(data.dir);
}



void setup() {
  pinMode(13, OUTPUT); // right
  pinMode(12, OUTPUT); // left
  pinMode(27, OUTPUT); // rev
  pinMode(33, OUTPUT); // fwd

  Serial.begin(115200);
  WiFi.mode(WIFI_STA); 
  esp_now_init(); 

  esp_now_register_recv_cb(esp_now_recv_cb_t(data_received)); // create and register the call back function so its called on each packet recieved
}

void loop() { // poll the received data
  digitalWrite(13,LOW);
  digitalWrite(12,LOW);
  digitalWrite(27,LOW);
  digitalWrite(33,LOW);

  if (data.dir == 1) {
    digitalWrite(33, HIGH); // fwd
  } else if(data.dir == 2) {
    digitalWrite(27,HIGH); // rev
  } else if(data.dir == 3) {
    digitalWrite(12, HIGH);
  } else if(data.dir == 4) {
    digitalWrite(12, HIGH); 
  } else {
    digitalWrite(13,LOW);
    digitalWrite(12,LOW);
    digitalWrite(27,LOW);
    digitalWrite(33,LOW);
  }
}