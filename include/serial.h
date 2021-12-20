#ifndef _SERIAL_H__
#define _SERIAL_H__

#define UART_NUM UART_NUM_2
#define UART_PIN_TX 17
#define UART_PIN_RX 13

#define RD_BUF_SIZE 1024

#define SERIAL_PKT_DATA_SIZE 300
#define MAX_SERIAL_PKT_DATA 10


struct Serial_Packet {
    uint8_t dataLength;
    uint8_t source;
    uint8_t destination;
    uint8_t protocol;
    uint8_t proto_address;
    uint8_t data[SERIAL_PKT_DATA_SIZE];
};


void  serial_init();
void  serial_deinit();
struct Serial_Packet serial_read(void);
int serial_write(const char* writeData);

#endif // _SERIAL_H__
