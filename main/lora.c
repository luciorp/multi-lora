#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "lora.h"
#include "config.h"

#define READ_REG     0x7F
#define WRITE_REG    0x80

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK  5
#define PIN_NUM_RST 14
#define PIN_NUM_NSS 18
#define PIN_NUM_DIO0 26

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

lora32_cfg_t lora;

static xQueueHandle dio0_evt_queue = NULL;

const long long frequencies[] = { 433e+6, 866e+6, 915e+6 };
const long bandwidths[] = { 7.8e+3, 10.4e+3, 15.6e+3, 20.8e+3, 31.25e+3, 41.7e+3, 62.5e+3, 125e+3, 250e+3 };

const char *TAG = "LoRa32";

lora32_cfg_t lora32_create() {
  static spi_device_handle_t spi;

  return (lora32_cfg_t){
    .bandwidth = bandwidths[9],
    .codingRate = DEFAULT_CR,
    .dio0 = PIN_NUM_DIO0,
    .implicitHeader = false,
    .nss = PIN_NUM_NSS,
    .reset = PIN_NUM_RST,
    .frequency = 866000000,
    .poll_rx = false,
    .preamble = DEFAULT_PREAMBLE,
    .spreadingFactor = DEFAULT_SF,
    .spi = spi,
    .receive = NULL,
    .useCRC = false,
    .fifoIdx = 0
  };
}


uint8_t lora32_read_reg(lora32_cfg_t *lora, uint8_t address) {
	spi_transaction_t t;
  memset(&t, 0, sizeof(spi_transaction_t));

  t.length = 16;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.tx_data[0] = address & READ_REG;
  t.tx_data[1] = 0x00;

	ESP_ERROR_CHECK(spi_device_transmit(lora->spi, &t));

	return t.rx_data[1];
}

void lora32_write_reg(lora32_cfg_t *lora, uint8_t address, uint8_t value) {
  spi_device_handle_t spi = lora->spi;

  spi_transaction_t t;
  memset(&t, 0, sizeof(spi_transaction_t));

  t.length = 16;
  t.flags = SPI_TRANS_USE_TXDATA;
  t.tx_data[0] = address | WRITE_REG;
  t.tx_data[1] = value;

  ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
};

void lora23_set_explicit_header(lora32_cfg_t *lora) {
  lora->implicitHeader = false;

  lora32_write_reg(lora, REG_MODEM_CONFIG_1, lora32_read_reg(lora, REG_MODEM_CONFIG_1) & 0xFE);
}

void lora23_set_implicit_header(lora32_cfg_t *lora) {
  lora->implicitHeader = true;

  lora32_write_reg(lora, REG_MODEM_CONFIG_1, lora32_read_reg(lora, REG_MODEM_CONFIG_1) | 0x01);
}

void lora32_idle(lora32_cfg_t *lora) {
  lora32_write_reg(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STANDBY);
}

void lora32_sleep(lora32_cfg_t *lora) {
  lora32_write_reg(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void lora32_enable_tx(lora32_cfg_t *lora) {
  
  lora32_idle(lora);

  if(lora->implicitHeader)
    lora23_set_implicit_header(lora);
  else
    lora23_set_explicit_header(lora);

  // zero out receive buffer
  lora32_write_reg(lora, REG_FIFO_ADDR_PTR, 0);
  lora32_write_reg(lora, REG_PAYLOAD_LENGTH, 0);
}

void lora32_send(lora32_cfg_t *lora, uint8_t *data, uint8_t len) {
  
  lora32_enable_tx(lora);

  uint8_t i = 0;
  for(; (i < len && i < MAX_PKT_LENGTH); i++)
    lora32_write_reg(lora, REG_FIFO, data[i]);

  lora32_write_reg(lora, REG_PAYLOAD_LENGTH, len);

  lora32_write_reg(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  ESP_LOGD(TAG, "lora32_send waiting for TX to finish");

  // can be made async by waiting for DIO0 and checking for IRQ_TX_DONE_MASK
  while((lora32_read_reg(lora, REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  ESP_LOGD(TAG, "lora32_send TX done");

  lora32_write_reg(lora, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

  // questionable....
  lora32_enable_continous_rx(lora);
}

void lora32_set_frequency(lora32_cfg_t *lora, long frequency) {
 
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  //ESP_LOGI(TAG, "REG_FRF_MSB: 0x%2X", (uint8_t)(frf >> 16));
  //ESP_LOGI(TAG, "REG_FRF_MID: 0x%2X", (uint8_t)(frf >>  8));
  //ESP_LOGI(TAG, "REG_FRF_LSB: 0x%2X", (uint8_t)(frf >>  0));

  lora32_write_reg(lora, REG_FRF_MSB, (uint8_t)(frf >> 16) & 0xFF);
  lora32_write_reg(lora, REG_FRF_MID, (uint8_t)(frf >>  8) & 0xFF);
  lora32_write_reg(lora, REG_FRF_LSB, (uint8_t)(frf >>  0) & 0xFF);

}

void lora32_set_tx_power(lora32_cfg_t *lora, uint8_t level, uint8_t output) {
  
  //ESP_LOGI(TAG, "set_tx_power(%d, %d)", level, output);

  if(output == PA_OUTPUT_RFO_PIN) {
    if(level > 14) level = 14;

    lora32_write_reg(lora, REG_PA_CONFIG, 0x70 | level);
  } else {
    if(level < 2) level = 2;
    else if(level > 17) level = 17;

    lora32_write_reg(lora, REG_PA_CONFIG, PA_BOOST | (level - 2));
  }

}

uint8_t lora32_parse_packet(lora32_cfg_t *lora, uint8_t size) {
  uint8_t length = 0;
  uint8_t irqs = lora32_read_reg(lora, REG_IRQ_FLAGS);

  if(size > 0) {
    lora23_set_implicit_header(lora);

    lora32_write_reg(lora, REG_PAYLOAD_LENGTH, size & 0xFF);
  } else {
    lora23_set_explicit_header(lora);
  }

  lora32_write_reg(lora, REG_IRQ_FLAGS, irqs);

  //ESP_LOGI(TAG, "irqs: 0x%2X", irqs);
  //ESP_LOGI(TAG, "irqs: 0x%2X", lora32_read_reg(spi, REG_IRQ_FLAGS));

  if ((irqs & IRQ_RX_DONE_MASK) && (irqs & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    lora->fifoIdx = 0;

    if(lora->implicitHeader) {
      length = lora32_read_reg(lora, REG_PAYLOAD_LENGTH);
    } else {
      length = lora32_read_reg(lora, REG_RX_NB_BYTES);
    }

    lora32_write_reg(lora, REG_FIFO_ADDR_PTR, lora32_read_reg(lora, REG_FIFO_RX_CURRENT_ADDR));

    lora32_idle(lora);
  } else if(lora32_read_reg(lora, REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    lora32_write_reg(lora, REG_FIFO_ADDR_PTR, 0);
    //lora32_write_reg(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  } else {
    //ESP_LOGI(TAG, "no irqs?");
  }

  return length;
}

uint8_t lora32_data_available(lora32_cfg_t *lora) {
  return lora32_read_reg(lora, REG_RX_NB_BYTES) - lora->fifoIdx;
}


void lora32_dump_regs(lora32_cfg_t *lora) {
  for(uint8_t i = 0; i < 127; i++) {
    printf("0x%2X: 0x%2X\n", i, lora32_read_reg(lora, i));
  }
}

void lora32_toggle_reset(lora32_cfg_t *config) {
  // toggle reset (L/H)
  //ESP_LOGI(TAG, "Toggling reset pin %d", config->reset);

  gpio_set_level(config->reset, 0);
  vTaskDelay(100 / portTICK_PERIOD_MS); // requires 100us

  gpio_set_level(config->reset, 1);
  vTaskDelay(100 / portTICK_PERIOD_MS); // 5ms before available
}

void lora32_set_spreadfactor(lora32_cfg_t *lora, uint8_t factor) {
  //ESP_LOGI(TAG, "lora32_set_spreadfactor: %d", factor);

  if(factor <= 6) {
    factor = 6;

    lora32_write_reg(lora, REG_DETECTION_OPTIMIZE, DETECT_OPT_SF6);
    lora32_write_reg(lora, REG_DETECTION_THRESHOLD, DETECT_THRES_SF6);
  } else {
    if(factor > 12) factor = 12;

    lora32_write_reg(lora, REG_DETECTION_OPTIMIZE, DETECT_OPT_OTHER);
    lora32_write_reg(lora, REG_DETECTION_THRESHOLD, DETECT_THRES_OTHER);
  }

  lora32_write_reg(lora, REG_MODEM_CONFIG_2, (lora32_read_reg(lora, REG_MODEM_CONFIG_2) & 0x0F) | ((factor << 4) & 0xF0));
}

void lora32_enable_continous_rx(lora32_cfg_t *lora) {
  ESP_LOGD(TAG, "enabling continuous receive");

  lora32_write_reg(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void lora32_set_coding_rate(lora32_cfg_t *lora, uint8_t d) {
  if(d < 5) d = 5;
  else if(d > 8) d = 8;

  uint8_t cr = d  - 4;

  lora32_write_reg(lora, REG_MODEM_CONFIG_1, (lora32_read_reg(lora, REG_MODEM_CONFIG_1) & 0xF1) | (cr << 1));
}

void lora32_handle_dio0(void *arg) {
  lora32_cfg_t *lora = (lora32_cfg_t*)arg;
  static uint8_t msg[MAX_PKT_LENGTH];

  while(1) {
		if(xQueueReceive(dio0_evt_queue, lora, portMAX_DELAY)) {
      //ESP_LOGI(TAG, "handling DIO0");

      memset(msg, 0, MAX_PKT_LENGTH);

      // read IRQ flags
      uint8_t irqs = lora32_read_reg(lora, REG_IRQ_FLAGS);
      // clear IRQ flags
      lora32_write_reg(lora, REG_IRQ_FLAGS, irqs);

      // TODO: read packet length
      uint8_t len = lora32_read_reg(lora, lora->implicitHeader ? REG_PAYLOAD_LENGTH : REG_RX_NB_BYTES);
      //ESP_LOGI(TAG, "lora32_handle_dio0 packet length: %d", len);

      // TODO: set FIFO address to RX address
      uint8_t fifo_addr = lora32_read_reg(lora, REG_FIFO_RX_CURRENT_ADDR);
      //ESP_LOGI(TAG, "lora32_handle_dio0 current FIFO address: %d", fifo_addr);

      lora32_write_reg(lora, REG_FIFO_ADDR_PTR, fifo_addr);

      uint8_t i = 0;
      for(; i < len; i++) {
        msg[lora->fifoIdx] = lora32_read_reg(lora, REG_FIFO);

        lora->fifoIdx++;
      }

      //ESP_LOGI(TAG, "lora32_handle_dio0: %s", msg);

      lora->fifoIdx = 0;
      lora32_write_reg(lora, REG_FIFO_ADDR_PTR, 0);

      lora->receive((uint8_t*)&msg, len);
    }
  }
}

static void IRAM_ATTR lora32_on_dio0(void *arg) {
  uint8_t i = 0;

  xQueueSendFromISR(dio0_evt_queue, &i, NULL);
}

uint8_t lora32_init(lora32_cfg_t *lora) {
  ESP_LOGD(TAG, "lora32_init");

  // set pin outputs
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL<<lora->reset)|(1ULL<<lora->nss);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

  lora32_toggle_reset(lora);

  // set NSS high
  //ESP_LOGI(TAG, "Bringing NSS high: %d", lora->nss);
  gpio_set_level(lora->nss, 1);

  vTaskDelay(10 / portTICK_PERIOD_MS);

  // init spi
  //ESP_LOGI(TAG, "Initializing SPI bus");
  //ESP_LOGI(TAG, "\r\nMISO: %d\r\nMOSI: %d\r\nCLK: %d\r\nNSS: %d\r\n", PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, lora->nss);

  spi_bus_config_t buscfg = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1
  };

  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 8E6,
    .flags = 0,
    .mode = 0,
    .spics_io_num = lora->nss,
    .queue_size = 7,
  };

  ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 0));
  ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &lora->spi));

  uint8_t version = lora32_read_reg(lora, REG_VERSION);
  ESP_LOGD(TAG, "lora32_get_id() == 0x%2X", version);
  assert(version == 0x12);

  // TODO: confirm this is happening. Before/after power measurements?
  lora32_sleep(lora);
  //ESP_LOGI(TAG, "lora32_sleep");

  // TODO: VERIFY
  lora32_set_frequency(lora, lora->frequency);
  //ESP_LOGI(TAG, "lora32_set_frequency: %lu", lora->frequency);

  lora32_write_reg(lora, REG_FIFO_TX_BASE_ADDR, 0x00);
  lora32_write_reg(lora, REG_FIFO_RX_BASE_ADDR, 0x00);
  //ESP_LOGI(TAG, "clear rx/tx fifos");
  
  uint8_t lna = lora32_read_reg(lora, REG_LNA);
  lora32_write_reg(lora, REG_LNA, lna | 0x03);
  //ESP_LOGI(TAG, "set lna: 0x%2X", lna | 0x03);

  lora32_write_reg(lora, REG_MODEM_CONFIG_3, 0x04);
  //ESP_LOGI(TAG, "REG_MODEM_CONFIG_3: 0x%2X", lora32_read_reg(spi, REG_MODEM_CONFIG_3));

  lora32_set_tx_power(lora, 17, PA_OUTPUT_PA_BOOST_PIN);
  //ESP_LOGI(TAG, "lora32_set_tx_power");

  lora32_idle(lora);
  //ESP_LOGI(TAG, "lora32_idle");

  //ESP_LOGI(TAG, "REG_OP_MODE: 0x%2X", lora32_read_reg(spi, REG_OP_MODE));

  if(lora->receive != NULL) {
    //ESP_LOGI(TAG, "Setting callback handler");

    dio0_evt_queue = xQueueCreate(20, sizeof(uint8_t));

    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_DIO0);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_intr_type(PIN_NUM_DIO0, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);

    gpio_isr_handler_add(PIN_NUM_DIO0, lora32_on_dio0, (void*)lora);

    // this should probably be high priority
    xTaskCreate(&lora32_handle_dio0, "lora32_handle_dio0", 4096, lora, 6, NULL);

    lora32_write_reg(lora, REG_DIO_MAPPING_1, 0x00);

    lora32_enable_continous_rx(lora);
  }

  //lora32_dump_regs(spi);

  return 1;
};

/**
 * Return last packet's RSSI.
 */
int lora32_packet_rssi(lora32_cfg_t *lora)
{
   return (lora32_read_reg(lora, REG_PKT_RSSI_VALUE) - (lora->frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float lora32_packet_snr(lora32_cfg_t *lora)
{
   return ((int8_t)lora32_read_reg(lora, REG_PKT_SNR_VALUE)) * 0.25;
}

/**
 * Generate a random byte, based on the Wideband RSSI measurement run through a von Neumann Extractor. **NB** 
 * - these are not cryptographically secure random numbers! Use with caution!
 */
uint8_t lora32_packet_ramdomness(lora32_cfg_t *lora)
{
  int n=0, bits=7;
  while(bits--) {
    n<<=1;
    while(1){
      // implement a basic von Neumann Extractor
      uint8_t a=(lora32_read_reg(lora,REG_RSSI_WIDEBAND) & 1);
      if(a != (lora32_read_reg(lora,REG_RSSI_WIDEBAND) & 1)){
        // put random, whitened bit in n
        n |= a;
        break;
      }
    }
  }
  // return the random byte
  return n;
}
