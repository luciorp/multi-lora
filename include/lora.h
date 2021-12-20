#ifndef _LORA_H__
#define _LORA_H__

#include "driver/spi_common.h"
#include "driver/spi_master.h"

#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_BR_MSB               0x02
#define REG_BR_LSB               0x03
#define REG_FD_MSB               0x04
#define REG_FD_LSB               0x05
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STANDBY             0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

#define DETECT_OPT_SF6    0xC5
#define DETECT_OPT_OTHER  0xC3
#define DETECT_THRES_SF6    0x0C
#define DETECT_THRES_OTHER  0x0A

#define DEFAULT_SF 7
#define DEFAULT_PREAMBLE 8
#define DEFAULT_CR 5

enum freq {
  F433, F866, F915
} lora32_freq;

const long long frequencies[3];
const long bandwidths[9];

enum bandwidth {
  B78, B104, B156, B208, B3125, B417, B625, B125, B250
};


typedef void (*receiveCallback)(uint8_t *data, uint8_t size);

typedef struct lora32_cfg_t {
  uint8_t nss;
  uint8_t dio0;
  uint8_t reset;
  uint8_t fifoIdx;

  long frequency;
  enum bandwidth bandwidth;

  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint16_t preamble;

  bool useCRC;
  bool implicitHeader;
  bool poll_rx;

  receiveCallback receive;
  spi_device_handle_t spi;
} lora32_cfg_t;

extern lora32_cfg_t lora;

lora32_cfg_t lora32_create();
uint8_t lora32_init(lora32_cfg_t *config);
uint8_t lora32_data_available(lora32_cfg_t *lora);
uint8_t lora32_parse_packet(lora32_cfg_t *lora, uint8_t size);
void lora32_send(lora32_cfg_t *config, uint8_t *data, uint8_t len);
void lora32_set_spreadfactor(lora32_cfg_t *lora, uint8_t factor);
void lora32_dump_regs(lora32_cfg_t *lora);
void lora32_enable_continous_rx(lora32_cfg_t *lora);
void lora32_set_coding_rate(lora32_cfg_t *lora, uint8_t d);
int lora32_packet_rssi(lora32_cfg_t *lora);
float lora32_packet_snr(lora32_cfg_t *lora);
uint8_t lora32_packet_ramdomness(lora32_cfg_t *lora);

#endif // _LORA_H__
