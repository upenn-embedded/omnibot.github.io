#include "driver/spi_slave.h"
#include "Arduino.h"

#define PIN_NUM_MISO  37
#define PIN_NUM_MOSI  35
#define PIN_NUM_CLK   36
#define PIN_NUM_CS    12

// Allocate DMA-safe memory (static, global)
DRAM_ATTR static uint8_t txdata[1] = {42};

void setup() {
  Serial.begin(115200);

  spi_bus_config_t bus_config = {
    .mosi_io_num = PIN_NUM_MOSI,
    .miso_io_num = PIN_NUM_MISO,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 32,
  };

  spi_slave_interface_config_t slave_config = {
    .spics_io_num = PIN_NUM_CS,
    .flags = 0,
    .queue_size = 1,
    .mode = 0,
    .post_setup_cb = NULL,
    .post_trans_cb = NULL,
  };

  esp_err_t ret = spi_slave_initialize(SPI2_HOST, &bus_config, &slave_config, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    Serial.println("SPI slave init failed");
  } else {
    Serial.println("SPI slave initialized");
  }
}

void loop() {
  spi_slave_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = 8; // 8 bits = 1 byte
  t.tx_buffer = txdata;

  esp_err_t err = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
  if (err == ESP_OK) {
    Serial.printf("Slave sent: %d\n", txdata[0]);
  }

  delay(100);
}