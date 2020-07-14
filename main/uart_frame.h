#ifndef _UART_FRAME_H
#define _UART_FRAME_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    uint8_t *buf;
    uint32_t buf_size;
} rx_buffer_t;

typedef enum {
    IDLE = 0x00,
    WAIT_FINISH ,
    FINISH,
} frame_state_n;

typedef void(*frame_fun)(int, const uint8_t*, int);

extern QueueHandle_t uart_buffer_quenue;
extern volatile frame_state_n frame_state;

extern void uart_init();
extern void uart_frame_task(void *arg);
extern void uart_set_cb(frame_fun cb_in);
void uart_frame_send(uint8_t cmd, const uint8_t* frame, uint32_t len, bool wait_finish);

#endif
