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
QueueHandle_t uart_buffer_quenue;
static QueueHandle_t uart_queue = NULL;

void uart_frame_task(void *arg);

frame_fun frame_callback = NULL;

void uart_init() {
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
    xTaskCreatePinnedToCore(uart_frame_task, "uart_queue_task", 4 * 1024, NULL, 1, NULL, 1);
}

void uart_set_cb(frame_fun cb_in) {
    frame_callback = cb_in;
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
                    if (len != (xEvent.size - 6)) {
                        break ;
                    }

                    int xor_result = 0;
                    for (int i = 0; i < xEvent.size - 1; i++) {
                        xor_result = xor_result ^ buf[i];
                    }

                    if (xor_result != buf[xEvent.size - 1]) {
                        break ;
                    }

                    uart_frame_send(buf[6], &buf[7], len - 2, false);
                    if (frame_callback != NULL) {
                        frame_callback(buf[6], &buf[7], len - 2);
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

void uart_frame_send(uint8_t cmd, const uint8_t* frame, int len, bool wait_finish) {
    uint8_t* out_buf = (uint8_t *)malloc(sizeof(uint8_t) * len + 8);
    out_buf[0] = PACK_FIRST_BYTE;
    out_buf[1] = PACK_SECOND_BYTE;
    int out_len = 2 + 4 + 1 + len + 1;
    out_buf[2] = (out_len - 6) >> 24;
    out_buf[3] = (out_len - 6) >> 16;
    out_buf[4] = (out_len - 6) >> 8;
    out_buf[5] = (out_len - 6);
    out_buf[6] = cmd;
    memcpy(&out_buf[7], frame, len);

    int xor = 0x00;
    for (int i = 0; i < out_len - 1; i++) {
        xor = out_buf[i] ^ xor;
    }
    out_buf[out_len - 1] = xor;

    uart_wait_tx_done(UART_NUM_0, portMAX_DELAY);
    uart_write_bytes(UART_NUM_0, (const char *)out_buf, out_len);

    if (wait_finish) {
        uart_wait_tx_done(UART_NUM_0, portMAX_DELAY);
    }
    free(out_buf);
}