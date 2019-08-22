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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rbuart.h"

#define PLL0_MAX_OUTPUT_FREQ    988000000UL
#define PLL1_MAX_OUTPUT_FREQ    400000000UL
#define PLL2_MAX_OUTPUT_FREQ    45100000UL

#define UART_NUM                1
#define UART_BUFFER_SIZE        4096
#define UART_TX_PIN             20
#define UART_RX_PIN             21
#define UART_RTS_PIN            UART_PIN_NO_CHANGE
#define UART_CTS_PIN            UART_PIN_NO_CHANGE


static int first_phase_num = 10;

// Used for FreeRTOS syslog
//-------------------------
uint64_t sys_ticks_us(void)
{
    return (read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000));
}

// uart callback function called from uart task
static void uart_callback(uint8_t uart_num, uint8_t type, int arg, uint8_t *buff)
{
    char *tmps;
    switch (type) {
        case UART_CB_TYPE_ERROR:
            printf("[UART_CB]: UART error %d%s\r\n", arg, (arg == UART_ERROR_BUFFER_FULL) ? " Buffer full" : "");
            break;
        case UART_CB_TYPE_DATA:
            tmps = calloc(arg+1, sizeof(char));
            memcpy(tmps, buff, arg);
            printf("[UART_CB]: Requested number of bytes (%d) received [%s]\r\n", arg, tmps);
            free(tmps);
            break;
        case UART_CB_TYPE_PATTERN:
            if (first_phase_num > 0) first_phase_num--;
            tmps = calloc(arg+1, sizeof(char));
            memcpy(tmps, buff, arg);
            printf("[UART_CB]: [%d] Pattern received at position %d (%s)\r\n", first_phase_num, arg, tmps);
            free(tmps);
            break;
    }
}


int main()
{
    vTaskDelay(10 / portTICK_PERIOD_MS);
    printf("\r\n=== RingBuffer UART example ==\r\n");

    // initialize the UART structure, set defaults
    rb_uart_obj_t rbuart;
    memset(&rbuart, 0, sizeof(rb_uart_obj_t));

    rbuart.uart_num = UART_NUM;
    rbuart.baudrate = 115200;
    rbuart.bits = 8;
    rbuart.parity = UART_PARITY_NONE;
    rbuart.stop = UART_STOP_1;
    rbuart.rts = UART_RTS_PIN;
    rbuart.cts = UART_CTS_PIN;
    rbuart.tx = UART_TX_PIN;
    rbuart.rx = UART_RX_PIN;
    rbuart.timeout = 1000;

    rbuart.data_cb = uart_callback;
    rbuart.pattern_cb = uart_callback;
    rbuart.error_cb = uart_callback;

    // data callback when 10 or more bytes received
    rbuart.data_cb_size = 10;
    // pattern callback when 'xyz' received
    sprintf((char *)rbuart.pattern, "xyz");
    rbuart.pattern_len = 3;

    rbuart.end_task = 0;
    sprintf((char *)rbuart.lineend, "\r\n");

    configASSERT(!k210_uarts[UART_NUM].active);
    k210_uarts[UART_NUM].active = true;

    // Set uart ring buffer size
    rbuart.buffer_size = UART_BUFFER_SIZE;

    // Initialize the uart hardware
    int res = uart_hard_init(rbuart.uart_num, rbuart.tx, rbuart.rx, rbuart.rts, rbuart.cts, true, true, rbuart.buffer_size);
    configASSERT(res == 0);
    // Configure uart
    uint32_t bdr = rb_uart_config(rbuart.uart_num, rbuart.baudrate, rbuart.bits, rbuart.stop, rbuart.parity);
    printf("Uart initialized, real baud rate=%u\r\n", bdr);

    // Create a task to handle UART receiving from uart ISR
    // Default task transfers received uart data to ring buffer
    // and executes callbacks on error, number of bytes received or pattern received
    // if callbacks function(s) is(are) defined
    BaseType_t tres = xTaskCreate(
        default_uart_event_task,                // function entry
        "uart_event_task",                      // task name
        configMINIMAL_STACK_SIZE,               // stack_deepth
        (void *)&rbuart,                        // function argument
        1,                                      // task priority
        &k210_uarts[rbuart.uart_num].task_id);  // task handle

    configASSERT(tres == pdPASS);
    vTaskDelay(20 / portTICK_PERIOD_MS);

    // === 1st phase, uart handled by callback function ===
    first_phase_num = 10;
    printf("Waiting for data (1st phase)\r\n");
    uart_write(UART_NUM, (uint8_t *)"Waiting for data (1st phase)\r\n", 0);
    while (1) {
        vTaskDelay(2 / portTICK_PERIOD_MS);
        if (first_phase_num <= 0) break;
    }

    // === 2nd phase, uart handled manually ===
    vTaskDelay(20 / portTICK_PERIOD_MS);
    printf("Waiting for 'ABC' sequence\r\n");
    // disable callbacks
    BaseType_t sres = xSemaphoreTake(k210_uarts[rbuart.uart_num].uart_mutex, 100);
    configASSERT(sres == pdPASS);
    rbuart.data_cb = 0;
    rbuart.data_cb_size = 0;
    rbuart.pattern_cb = 0;
    rbuart.pattern[0] = 0;
    rbuart.pattern_len = 0;
    rbuart.error_cb = 0;
    // Request termination of the uart task
    rbuart.end_task = 0;
    xSemaphoreGive(k210_uarts[rbuart.uart_num].uart_mutex);

    // enable uart notification from uart interrupt handler
    k210_uarts[rbuart.uart_num].uart_buf->notify = 1;

    char *received;
    uart_write(UART_NUM, (uint8_t *)"Waiting for 'ABC' sequence\r\n", 0);
    while (1) {
        if (k210_uarts[rbuart.uart_num].task_semaphore) {
            // === Using uart semaphore for fast response to received data ===
            if (xSemaphoreTake(k210_uarts[rbuart.uart_num].task_semaphore, 5000 / portTICK_PERIOD_MS) == pdTRUE) {
                // some new data received, get it
                received = uart_read(UART_NUM, 100, "ABC", NULL);
                if (received) {
                    printf("Received:\r\n%s\r\n", received);
                    // free the receive buffer !!
                    free(received);
                }
            }
        }
        else {
            // === just wait for data ===
            received = uart_read(UART_NUM, 5000, "ABC", NULL);
            if (received) {
                printf("Received:\r\n%s\r\n", received);
                // free the receive buffer !!
                free(received);
            }
        }
    }
}
