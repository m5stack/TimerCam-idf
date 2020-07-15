#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "string.h"

#include "uart_frame.h"

#define UART_NUM UART_NUM_0
#define BAUD_RATE 1500000
#define UART_QUEUE_LENGTH 10
#define RX_BUF_SIZE 256
#define TX_BUF_SIZE 1024*200

#define PACK_FIRST_BYTE 0xAA
#define PACK_SECOND_BYTE 0x55

volatile frame_state_n frame_state;
static QueueHandle_t uart_queue = NULL;
static QueueHandle_t uart_buffer_queue = NULL;
static SemaphoreHandle_t uart_lock = NULL;

void uart_frame_task(void *arg);
void uart_frame_send_task(void *arg);

frame_fun frame_callback = NULL;

typedef struct _UartFrame_t {
    bool free;
    uint8_t* frame;
    uint32_t len;
    /* data */
} UartFrame_t;


void uart_init() {
    uart_lock = xSemaphoreCreateMutex();
    uart_buffer_queue = xQueueCreate(10, sizeof(UartFrame_t));
    const uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE, TX_BUF_SIZE, UART_QUEUE_LENGTH, &uart_queue, ESP_INTR_FLAG_LOWMED);
    uart_set_rx_timeout(UART_NUM_0, 2);
    xTaskCreatePinnedToCore(uart_frame_task, "uart_queue_task", 4 * 1024, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(uart_frame_send_task, "uart_frame_send_task", 4 * 1024, NULL, 2, NULL, 1);
}

void uart_set_cb(frame_fun cb_in) {
    frame_callback = cb_in;
}

void uart_frame_send_task(void *arg) {
    UartFrame_t frame;
    char end_bytes[] = {0xa5, 0xa5, 0xa5, 0xa5, 0xa5};
    
    for (;;) {
        xQueueReceive(uart_buffer_queue, &frame, portMAX_DELAY);
        uart_wait_tx_done(UART_NUM_0, portMAX_DELAY);
        uart_write_bytes(UART_NUM_0, (const char *)frame.frame, frame.len);
        // insert end char in the frame end, 552 will lost data in 150K baud
        uart_wait_tx_done(UART_NUM_0, portMAX_DELAY);
        uart_write_bytes(UART_NUM_0, end_bytes, 5);
        if (frame.free) {
            free(frame.frame);
        }
    }
    vTaskDelete(NULL);
}

void uart_frame_task(void *arg) {
    uart_event_t xEvent;
    uint8_t *buf = (uint8_t *)malloc(256 * sizeof(uint8_t));
    for(;;) {
        if (xQueueReceive(uart_queue, (void*)&xEvent, portMAX_DELAY) == pdTRUE) {
            switch(xEvent.type) {
                case UART_DATA:
                    if (xEvent.size > 256 && xEvent.size < 8) {
                        uart_flush_input(UART_NUM_0);
                        break;
                    }

                    uart_read_bytes(UART_NUM_0, buf, xEvent.size, portMAX_DELAY);
                    
                    if(buf[0] != PACK_FIRST_BYTE || buf[1] != PACK_SECOND_BYTE) {
                        break ;
                    }

                    int len = (buf[2] << 24) | (buf[3] << 16) | (buf[4] << 8) | buf[5];
                    if (len != (xEvent.size - 7)) {
                        break ;
                    }

                    int xor_result = 0;
                    for (int i = 0; i < xEvent.size; i++) {
                        xor_result = xor_result ^ buf[i];
                    }

                    if (xor_result != 0) {
                        break ;
                    }
                    if (frame_callback != NULL) {
                        frame_callback(buf[7], &buf[8], len - 2);
                    }

                    break;
                case UART_FIFO_OVF:
                    xQueueReset(uart_queue);
                    break;
                case UART_BUFFER_FULL:
                    uart_flush_input(UART_NUM_0);
                    xQueueReset(uart_queue);
                    break;
                case UART_BREAK:
                    break;
                case UART_PARITY_ERR:
                    break;
                case UART_FRAME_ERR:
                    break;
                default:
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

void uart_frame_send(uint8_t cmd, const uint8_t* frame, uint32_t len, bool wait_finish) {
    uint32_t out_len = 9 + len;
    uint8_t* out_buf = (uint8_t *)malloc(sizeof(uint8_t) * out_len);

    out_buf[0] = PACK_FIRST_BYTE;
    out_buf[1] = PACK_SECOND_BYTE;
    out_buf[2] = (out_len - 7) >> 24;
    out_buf[3] = (out_len - 7) >> 16;
    out_buf[4] = (out_len - 7) >> 8;
    out_buf[5] = (out_len - 7);
    out_buf[6] = 0x00 ^ out_buf[2] ^ out_buf[3] ^ out_buf[4] ^ out_buf[5];
    out_buf[7] = cmd;
    memcpy(&out_buf[8], frame, len);

    int xor = 0x00;
    for (uint32_t i = 0; i < out_len - 1; i++) {
        xor = out_buf[i] ^ xor;
    }
    out_buf[out_len - 1] = xor;

    if (wait_finish) {
        while (uxQueueMessagesWaiting(uart_buffer_queue)) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    UartFrame_t uart_frame;
    uart_frame.frame = out_buf;
    uart_frame.len = out_len;
    uart_frame.free = true;

    xQueueSend(uart_buffer_queue, &uart_frame, portMAX_DELAY);
}