/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * Uart example
 *
 * Copyright (c) 2019 LoBo (https://github.com/loboris)
*/

#ifndef INC_RBUART_H
#define INC_RBUART_H

#include "FreeRTOS.h"
#include "task.h"

#include "devices.h"
#include "platform.h"
#include "sysctl.h"
#include "uart.h"


#define UART_NUM_MAX            3
#define UART_PIN_NO_CHANGE      -1

#define UART_CB_TYPE_DATA		1
#define UART_CB_TYPE_PATTERN	2
#define UART_CB_TYPE_ERROR		3

#define UART_ERROR_BUFFER_FULL  7
#define UART_ERROR_NOMEM        8

typedef enum _uart_send_trigger
{
    UART_SEND_FIFO_0,
    UART_SEND_FIFO_2,
    UART_SEND_FIFO_4,
    UART_SEND_FIFO_8,
} uart_send_trigger_t;

typedef enum _uart_receive_trigger
{
    UART_RECEIVE_FIFO_1,
    UART_RECEIVE_FIFO_4,
    UART_RECEIVE_FIFO_8,
    UART_RECEIVE_FIFO_14,
} uart_receive_trigger_t;

typedef struct _rb_uart_obj_t {
    uint32_t uart_num;
    uint32_t baudrate;
    int8_t bits;
    int8_t parity;
    int8_t stop;
    int8_t tx;
    int8_t rx;
    int8_t rts;
    int8_t cts;
    int data_cb_size;
    uint8_t pattern[16];
    uint8_t pattern_len;
    uint16_t timeout;       // timeout waiting for first char (in ms)
    uint16_t buffer_size;
    void (*data_cb) (uint8_t uart_num, uint8_t type, int arg, uint8_t *buff);
    void (*pattern_cb) (uint8_t uart_num, uint8_t type, int arg, uint8_t *buff);
    void (*error_cb) (uint8_t uart_num, uint8_t type, int arg, uint8_t *buff);
    uint32_t inverted;
    uint8_t end_task;
    uint8_t lineend[3];
} rb_uart_obj_t;

typedef struct _uart_driver_t {
    volatile uart_t *uart;
    plic_irq_t irq;
    sysctl_clock_t clock;
} uart_driver_t;

typedef struct _uart_ringbuf_t {
    size_t size;        // size of the buffer
    size_t head;        // buffer head position
    size_t tail;        // buffer tail position
    size_t length;      // length of the received characters in the buffer
    size_t overflow;    // number of characters received but not placed in buffer
    uint8_t *buf;       // pointer to the receive buffer
    uint8_t uart_num;   // K210 uart num
    uint8_t notify;     // external task notification flag
} uart_ringbuf_t;

typedef struct _uart_uarts_t {
    bool active;                            // uart active flag
    handle_t handle;                        // handle to the uart's device driver
    bool irq_flag;                          // set to true while processing uart interrupt
    TaskHandle_t task_id;
    QueueSetMemberHandle_t task_semaphore;  // external task semaphore
    QueueHandle_t uart_mutex;
    uart_ringbuf_t uart_buffer;
    uart_ringbuf_t *uart_buf;
} uart_uarts_t;

extern uart_uarts_t k210_uarts[UART_NUM_MAX];


int rb_uart_config(uint32_t uart_num, uint32_t baud_rate, uint32_t databits, uart_stopbits_t stopbits, uart_parity_t parity);
void rb_uart_close(uint32_t uart_num);

int uart_hard_init(uint32_t uart_num, uint8_t tx, uint8_t rx, int8_t rts, int8_t cts, bool mutex, bool semaphore, int rb_size);
bool uart_deinit(uint32_t uart_num, uint8_t *end_task, uint8_t tx, uint8_t rx, int8_t rts, int8_t cts);
char *uart_read(handle_t uart_num, int timeout, char *lnend, char *lnstart);
int uart_write(uint32_t uart_num, const uint8_t *buff, size_t len);
int match_pattern(uint8_t *text, int text_length, uint8_t *pattern, int pattern_length);

void default_uart_event_task(void *pvParameters);

void uart_ringbuf_alloc(uint8_t uart_num, size_t sz);
int uart_buf_put(uart_ringbuf_t *r, uint8_t *src, size_t len);
int uart_buf_remove_from_end(uart_ringbuf_t *r, size_t len);
size_t uart_buf_length(uart_ringbuf_t *r, size_t *size);
int uart_buf_get(uart_ringbuf_t *r, uint8_t *dest, size_t len);
int uart_buf_blank(uart_ringbuf_t *r, size_t pos, size_t len);
int uart_buf_remove(uart_ringbuf_t *r, size_t len);
int uart_buf_copy(uart_ringbuf_t *r, uint8_t *dest, size_t len);
int uart_buf_copy_from(uart_ringbuf_t *r, size_t pos, uint8_t *dest, size_t len);
int uart_buf_find_from(uart_ringbuf_t *r, size_t start_pos, size_t size, const char *pattern, int pattern_length, size_t *buflen);
int uart_buf_find(uart_ringbuf_t *r, size_t size, const char *pattern, int pattern_length, size_t *buflen);
void uart_buf_flush(uart_ringbuf_t *r);

#endif
