#ifndef _UART_FRAME_H
#define _UART_FRAME_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    IDLE = 0x00,
    WAIT_FINISH ,
    FINISH,
} frame_state_n;

extern volatile frame_state_n frame_state;

void uart_init();
void uart_frame_send(uint8_t cmd, const uint8_t* frame, uint32_t len, bool wait_finish);

#endif
