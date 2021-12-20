/* 

*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "serial.h"
#include "config.h"

static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */



struct Serial_Packet serialRxPacketBuffer[MAX_SERIAL_PKT_DATA];
int serialRxBufferEntry = 0;
int	readIndex	    =	0;	// Index of the read pointer
int	writeIndex	    =	0;	// Index of the write pointer


static QueueHandle_t uart0_queue;

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t uart_receive_buff[RD_BUF_SIZE];
    int uart_receive_buff_index = 0;
    int old_uart_receive_buff_index = RD_BUF_SIZE;
    struct Serial_Packet tempPacket;

    for(;;) {
        //Waiting for UART event
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(uart_receive_buff, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", UART_NUM);
            switch(event.type) {
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_NUM, uart_receive_buff, event.size, portMAX_DELAY);
                    memcpy(&tempPacket.data[uart_receive_buff_index], uart_receive_buff, event.size);
                    uart_receive_buff_index = uart_receive_buff_index + event.size;
                    tempPacket.data[uart_receive_buff_index] ='\0';
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                 case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                 case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
            printf(">>>>>>>> Uart size %d data %s\r\n",uart_receive_buff_index-1, tempPacket.data);
            fflush(stdout);
            if((uart_receive_buff_index > 0) & (uart_receive_buff_index != old_uart_receive_buff_index)){
                old_uart_receive_buff_index = uart_receive_buff_index;
            } else {
                if(uart_receive_buff_index > 0){
                    memset(&serialRxPacketBuffer[writeIndex].data, 0, SERIAL_PKT_DATA_SIZE);
                    memcpy(&serialRxPacketBuffer[writeIndex].data, uart_receive_buff, uart_receive_buff_index-1);
                    if(writeIndex < MAX_SERIAL_PKT_DATA -1){
                        writeIndex++;
                    } else{
                        writeIndex = 0;
                    }
                    printf("Entreuq\r\n");
                    fflush(stdout);
                    uart_receive_buff_index = 0;
                    old_uart_receive_buff_index = RD_BUF_SIZE;
                }
            }
            
            vTaskDelay(10/ portTICK_PERIOD_MS);
            
        } 
        
               /* uart_read_bytes(UART_NUM, uart_receive_buff, event.size, portMAX_DELAY);
                    if(event.size < SERIAL_PKT_DATA_SIZE){
                        if(serialRxBufferEntry < MAX_SERIAL_PKT_DATA){
                            memset(&serialRxPacketBuffer[writeIndex].data, 0, SERIAL_PKT_DATA_SIZE);
                            memcpy(&serialRxPacketBuffer[writeIndex].data, uart_receive_buff, event.size);
                            serialRxPacketBuffer[writeIndex].dataLength = event.size;
                            if(writeIndex < MAX_SERIAL_PKT_DATA -1){
                                writeIndex++;
                            } else{
                                writeIndex = 0;
                            }
                            serialRxBufferEntry++;
                        } else {
                            ESP_LOGI(TAG, "[DATA EVT]: PKT buffer cheio\r\n");
                        }
                    }else{
                        ESP_LOGI(TAG, "[DATA EVT]: erro tamanho superou buffer de dado\r\n");
                    } */
                 
    }
    vTaskDelete(NULL);
}

void serial_init()
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM, UART_PIN_TX, UART_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM, RD_BUF_SIZE * 2, RD_BUF_SIZE * 2, 20, &uart0_queue, 0);
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
}

void serial_deinit()
{
  
    uart_driver_delete(UART_NUM);

}


int serial_write(const char* writeData)
{
    const int len = strlen(writeData);
    const int txBytes = uart_write_bytes(UART_NUM, writeData, len);
    ESP_LOGI(TAG,"Uart Wrote %d bytes", txBytes);
    return txBytes;
}

struct Serial_Packet serial_read(void)
{
    struct Serial_Packet readPacket;
    memset(&readPacket, 0, sizeof(serial_read));
    if(serialRxBufferEntry > 0){
        memcpy(&readPacket, &serialRxPacketBuffer[readIndex], sizeof(serial_read));
        if(readIndex < MAX_SERIAL_PKT_DATA -1){
            readIndex++;
        } else{
            readIndex = 0;
        }
        serialRxBufferEntry--;
    } else {
        ESP_LOGI(TAG, "[DATA EVT]: PKT buffer vazio\r\n");
    }
    return readPacket; 
}

