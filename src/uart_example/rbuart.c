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

#include <string.h>
#include <stdlib.h>

#include "rbuart.h"

#include "fpioa.h"
#include "queue.h"
#include "devices.h"
#include "hal.h"
#include "syslog.h"


#define UART_BRATE_CONST        16
#define UART_MUTEX_TIMEOUT      (100 / portTICK_PERIOD_MS)

extern pic_irq_handler_t extern_uart_irq_handler;
extern void *extern_uart_irq_userdata;

typedef struct _task_params_t {
    void *uart_obj;
    void *thread_handle;
} task_params_t;

volatile uart_t* const  uart[3] =
{
    (volatile uart_t*)UART1_BASE_ADDR,
    (volatile uart_t*)UART2_BASE_ADDR,
    (volatile uart_t*)UART3_BASE_ADDR
};

static const uint32_t uart_instances[3] = { 0, 1, 2 };

static const char *TAG = "[UART]";

uart_uarts_t k210_uarts[UART_NUM_MAX] = { 0 };


//---------------------
uint64_t ticks_us(void)
{
    return (read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000));
}

//---------------------
uint64_t ticks_ms(void)
{
    return ticks_us() / 1000;
}



// =================================================
// ==== UART Ring Buffer functions =================
// =================================================

//===========================================
// Interrupt handler for UART
// pushes received byte(s) to the uart buffer
//===========================================
static void uart_on_irq_recv(void *userdata)
{
    uint32_t *nuart = (uint32_t *)userdata;
    k210_uarts[*nuart].irq_flag = true;
    uart_ringbuf_t *r = k210_uarts[*nuart].uart_buf;
    uint8_t c;

    // put all received characters into the ringbuffer
    while (uart[*nuart]->LSR & 1) {
        c = (uint8_t)(uart[*nuart]->RBR & 0xff);
        if (r->buf) {
            if (r->length < r->size) {
                r->buf[r->tail] = c;
                r->tail = (r->tail + 1) % r->size;
                r->length++;
            }
            else r->overflow++;
        }
        else r->overflow++;
    }
    k210_uarts[*nuart].irq_flag = false;
    // If external task semaphore exists and ring buffer notification flag is set
    // give the semaphore
    if ((k210_uarts[*nuart].task_semaphore) && (r->notify)) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(k210_uarts[*nuart].task_semaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR();
        }
    }
}

// Put data into buffer
// Operation on uart's ringbuffer is not allowed
//-----------------------------------------------------------
int uart_buf_put(uart_ringbuf_t *r, uint8_t *src, size_t len)
{
    if (r->uart_num < UART_NUM_MAX) return 0;
    if (r->buf == NULL) {
        r->overflow += len;
        return 0;
    }
    size_t cnt = 0;

    for (int i=0; i<len; i++) {
        if (r->length < r->size) {
            r->buf[r->tail] = src[i];
            r->tail = (r->tail + 1) % r->size;
            r->length++;
            cnt++;
        }
        else r->overflow++;
    }
    return cnt;
}

// Remove data from buffer end
// Operation is not allowed on uart's ringbuffer
//---------------------------------------------------------
int uart_buf_remove_from_end(uart_ringbuf_t *r, size_t len)
{
    if (r->uart_num < UART_NUM_MAX) return 0;
    if (r->buf == NULL) return 0;
    size_t cnt = 0;
    size_t length = r->length;

    if (length == 0) return 0;

    for (int i=len; i>=0; i--) {
        if (r->length == 0) break;
        r->tail--;
        r->length--;
        cnt++;
    }
    return cnt;
}

// Get current buffer length
//-----------------------------------------------------
size_t uart_buf_length(uart_ringbuf_t *r, size_t *size)
{
    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    size_t length = r->length;
    if (size) *size = r->size;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    return length;
}

// Get and remove data from buffer
//------------------------------------------------------------
int uart_buf_get(uart_ringbuf_t *r, uint8_t *dest, size_t len)
{
    if (r->buf == NULL) return 0;

    size_t cnt = 0;
    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    size_t length = r->length;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    if (length == 0) return 0;

    while (length) {
        if (dest) dest[cnt] = r->buf[r->head];
        r->head = (r->head + 1) % r->size;
        length--;
        cnt++;
        if (cnt >= len) break;
    }

    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    r->length -= cnt;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    return cnt;
}

// Remove data from buffer
//------------------------------------------------
int uart_buf_remove(uart_ringbuf_t *r, size_t len)
{
    size_t cnt = 0;
    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    size_t length = r->length;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    if (length == 0) return 0;

    while (length) {
        r->head = (r->head + 1) % r->size;
        length--;
        cnt++;
        if (cnt >= len) break;
    }

    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    r->length -= cnt;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    return cnt;
}

// Set the data in uart buffer to blank
//-----------------------------------------------------------
int uart_buf_blank(uart_ringbuf_t *r, size_t pos, size_t len)
{
    size_t cnt = 0;
    if (r->buf == NULL) return 0;

    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    int length = r->length - pos;
    size_t head = r->head;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    if (length <= 0) return 0;

    head = (head + pos) % r->size;
    while (length) {
        r->buf[head] = '^';
        head = (head + 1) % r->size;
        length--;
        cnt++;
        if (cnt >= len) break;
    }

    return cnt;
}

// Get data from buffer, but leave it in buffer
// Copy from the requested position in the buffer
//------------------------------------------------------------------------------
int uart_buf_copy_from(uart_ringbuf_t *r, size_t pos, uint8_t *dest, size_t len)
{
    if (r->buf == NULL) return 0;
    if (dest == NULL) return 0;    // no destination buffer

    size_t cnt = 0;
    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    int length = r->length - pos;
    size_t head = r->head;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    if (length <= 0) return 0;
    if (len > length) len = length;

    head = (head + pos) % r->size;
    while (length) {
        dest[cnt] = r->buf[head];
        head = (head + 1) % r->size;
        length--;
        cnt++;
        if (cnt >= len) break;
    }

    return cnt;
}

// Get data from buffer, but leave it in buffer
// Copy from the buffer start
//-------------------------------------------------------------
int uart_buf_copy(uart_ringbuf_t *r, uint8_t *dest, size_t len)
{
    return uart_buf_copy_from(r, 0, dest, len);
}

// Find pattern in uart buffer
//-------------------------------------------------------------------------------------------------------------------------------
int uart_buf_find_from(uart_ringbuf_t *r, size_t start_pos, size_t size, const char *pattern, int pattern_length, size_t *buflen)
{
    if (r->buf == NULL) return -1;

    int c, d, e, pos, position = -1;
    //if ((pattern == NULL) || (pattern_length == 0)) return -1;

    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    int length = r->length - start_pos;
    size_t head = r->head;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    if (length <= 0) return -1;
    if (size > length) size = length;

    if (buflen) *buflen = (length > size) ? size : length;
    if (pattern_length <= length) {
        for (c = 0; c <= (length - pattern_length); c++) {
            if (c > size) break;
            position = e = c;
            // check pattern
            for (d = 0; d < pattern_length; d++) {
                pos = (head + start_pos + e) % r->size;
                if (pattern[d] == r->buf[pos]) e++;
                else {
                    position = -1;
                    break;
                }
            }
            if (d == pattern_length) break;
        }
    }

    if (position >= 0) return (start_pos + position);
    return position;
}

// Find pattern in uart buffer
//--------------------------------------------------------------------------------------------------------
int uart_buf_find(uart_ringbuf_t *r, size_t size, const char *pattern, int pattern_length, size_t *buflen)
{
    return uart_buf_find_from(r, 0, size, pattern, pattern_length, buflen);
}

// Empty uart buffer
//-------------------------------------
void uart_buf_flush(uart_ringbuf_t *r)
{
    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    if (r->length > 0) {
        r->tail = 0;
        r->head = 0;
        r->length = 0;
        r->overflow = 0;
    }
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();
}

//--------------------------------------
int uart_putc(uint32_t uart_num, char c)
{
    while (!(uart[uart_num]->LSR & (1u << 6)))
        ;
    uart[uart_num]->THR = c;
    return 0;
}

//----------------------------------------------------------------
int uart_write(uint32_t uart_num, const uint8_t *buff, size_t len)
{
    if (len == 0) len = strlen((char *)buff);
    int write = 0;
    while (write < len) {
        while (!(uart[uart_num]->LSR & (1u << 6)))
            ;
        uart[uart_num]->THR = *buff++;
        write++;
    }

    return write;
}

//--------------------------------------------------
void uart_ringbuf_alloc(uint8_t uart_num, size_t sz)
{
    if (uart_num >= UART_NUM_MAX) return;
    k210_uarts[uart_num].uart_buffer.buf = malloc(sz);
    k210_uarts[uart_num].uart_buffer.size = sz;
    k210_uarts[uart_num].uart_buffer.head = 0;
    k210_uarts[uart_num].uart_buffer.tail = 0;
    k210_uarts[uart_num].uart_buffer.length = 0;
    k210_uarts[uart_num].uart_buffer.uart_num = uart_num;
    k210_uarts[uart_num].uart_buffer.notify = false;
    k210_uarts[uart_num].uart_buf = &k210_uarts[uart_num].uart_buffer;
}

// =========================================================================


//--------------------------------------------------------------------------------------------------------------------------
int rb_uart_config(uint32_t uart_num, uint32_t baud_rate, uint32_t databits, uart_stopbits_t stopbits, uart_parity_t parity)
{

    k210_uarts[uart_num].uart_buf->head = 0;
    k210_uarts[uart_num].uart_buf->tail = 0;
    k210_uarts[uart_num].uart_buf->length = 0;

    configASSERT(databits >= 5 && databits <= 8);
    if (databits == 5) {
        configASSERT(stopbits != UART_STOP_2);
    }
    else {
        configASSERT(stopbits != UART_STOP_1_5);
    }

    uint32_t stopbit_val = stopbits == UART_STOP_1 ? 0 : 1;
    uint32_t parity_val = 0;
    switch (parity)
    {
    case UART_PARITY_NONE:
        parity_val = 0;
        break;
    case UART_PARITY_ODD:
        parity_val = 1;
        break;
    case UART_PARITY_EVEN:
        parity_val = 3;
        break;
    default:
        configASSERT(!"Invalid parity");
        break;
    }

    uint32_t freq = sysctl_clock_get_freq(SYSCTL_CLOCK_APB0); //SYSCTL_CLOCK_UART1+uart_num);
    uint32_t divisor = freq / baud_rate;
    uint8_t dlh = divisor >> 12;
    uint8_t dll = (divisor - (dlh << 12)) / UART_BRATE_CONST;
    uint8_t dlf = divisor - (dlh << 12) - dll * UART_BRATE_CONST;

    // Set UART registers
    uart[uart_num]->TCR &= ~(1u);
    uart[uart_num]->TCR &= ~(1u << 3);
    uart[uart_num]->TCR &= ~(1u << 4);
    uart[uart_num]->TCR |= (1u << 2);
    uart[uart_num]->TCR &= ~(1u << 1);
    uart[uart_num]->DE_EN &= ~(1u);

    uart[uart_num]->LCR |= 1u << 7;
    uart[uart_num]->DLH = dlh;
    uart[uart_num]->DLL = dll;
    uart[uart_num]->DLF = dlf;

    uart[uart_num]->LCR = 0;
    uart[uart_num]->LCR = (databits - 5) | (stopbit_val << 2) | (parity_val << 3);
    uart[uart_num]->LCR &= ~(1u << 7);
    uart[uart_num]->MCR &= ~3;
    uart[uart_num]->IER = 1; // interrupt on receive
    //uart[uart_num]->IER |= 0x80; // enable by THRESHOLD
    //uart[uart_num]->FCR = UART_RECEIVE_FIFO_1 << 6 | UART_SEND_FIFO_0 << 4 | 0x1 << 3 | 0x1;

    return (freq / divisor);
}

//-----------------------------------
void rb_uart_close(uint32_t uart_num)
{
    // Free the uart buffer
    if (k210_uarts[uart_num].uart_buf != NULL) {
        if (k210_uarts[uart_num].uart_buf->buf) free(k210_uarts[uart_num].uart_buf->buf);
    }
    // Delete semaphore & mutex
    if (k210_uarts[uart_num].task_semaphore != NULL) vSemaphoreDelete(k210_uarts[uart_num].task_semaphore);
    if (k210_uarts[uart_num].uart_mutex != NULL) vSemaphoreDelete(k210_uarts[uart_num].uart_mutex);

    memset(&k210_uarts[uart_num], 0, sizeof(uart_uarts_t));
}

//----------------------------------------------------------------------------------------------------------------------------
int uart_hard_init(uint32_t uart_num, uint8_t tx, uint8_t rx, int8_t rts, int8_t cts, bool mutex, bool semaphore, int rb_size)
{
    if (k210_uarts[uart_num].uart_buf == NULL) {
        // === First time use, create ring buffer ===
        uart_ringbuf_alloc(uart_num, rb_size);
        if (k210_uarts[uart_num].uart_buf == NULL) {
            return -3;
        }
    }

    if (mutex) {
        // Create uart mutex
        if (k210_uarts[uart_num].uart_mutex == NULL) {
            k210_uarts[uart_num].uart_mutex = xSemaphoreCreateMutex();
            if (!k210_uarts[uart_num].uart_mutex) {
                return -4;
            }
        }
    }

    if (semaphore) {
        // Create uart semaphore
        if (k210_uarts[uart_num].task_semaphore == NULL) {
            k210_uarts[uart_num].task_semaphore = xSemaphoreCreateBinary();
            if (!k210_uarts[uart_num].task_semaphore) {
                return -5;
            }
        }
    }

    // Configure tx & rx pins
    fpioa_set_function(rx, FUNC_UART1_RX + (uart_num * 2));
    fpioa_set_function(tx, FUNC_UART1_TX + (uart_num * 2));
    if (rts >= 0) fpioa_set_function(rts, FUNC_UART1_RTS + (uart_num * 14));
    if (cts >= 0) fpioa_set_function(cts, FUNC_UART1_CTS + (uart_num * 14));

    // Request external uart interrupt handling
    extern_uart_irq_handler = (pic_irq_handler_t)uart_on_irq_recv;
    extern_uart_irq_userdata = (void *)&uart_instances[uart_num];

    // Open the uart device driver
    switch (uart_num) {
        case 0:
            k210_uarts[uart_num].handle = io_open("/dev/uart1");
            break;
        case 1:
            k210_uarts[uart_num].handle = io_open("/dev/uart2");
            break;
        case 2:
            k210_uarts[uart_num].handle = io_open("/dev/uart3");
            break;
    }

    return 0;
}

//----------------------------------------------------------------------------------------------------
bool uart_deinit(uint32_t uart_num, uint8_t *end_task, uint8_t tx, uint8_t rx, int8_t rts, int8_t cts)
{
    if ((end_task) && (k210_uarts[uart_num].task_id != NULL)) {
        // stop the uart task
        if (k210_uarts[uart_num].uart_mutex) xSemaphoreTake(k210_uarts[uart_num].uart_mutex, UART_MUTEX_TIMEOUT);
        *end_task = 1;
        if (k210_uarts[uart_num].uart_mutex) xSemaphoreGive(k210_uarts[uart_num].uart_mutex);
        // wait until ended
        int tmo = 100;
        while ((tmo > 0) && (k210_uarts[uart_num].task_id != NULL)) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            tmo--;
        }
        if (k210_uarts[uart_num].task_id != NULL) {
            return false;
        }
    }

    // Close UART device
    if (k210_uarts[uart_num].handle) {
        if (k210_uarts[uart_num].handle) io_close(k210_uarts[uart_num].handle);
        rb_uart_close(uart_num);
    }

    // Deconfigure tx & rx pins
    fpioa_set_function(rx, FUNC_RESV0);
    fpioa_set_function(tx, FUNC_RESV0);
    if (rts >= 0) fpioa_set_function(rts, FUNC_RESV0);
    if (cts >= 0) fpioa_set_function(cts, FUNC_RESV0);

    return true;
}

//-------------------------------------------------------------------------------------
int match_pattern(uint8_t *text, int text_length, uint8_t *pattern, int pattern_length)
{
    int c, d, e, position = -1;

    if (pattern_length > text_length) return -1;

    for (c = 0; c <= (text_length - pattern_length); c++) {
        position = e = c;
        // check pattern
        for (d = 0; d < pattern_length; d++) {
            if (pattern[d] == text[e]) e++;
            else break;
        }
        if (d == pattern_length) return position;
    }

    return -1;
}

//----------------------------------------------
void default_uart_event_task(void *pvParameters)
{
    rb_uart_obj_t *self = (rb_uart_obj_t *)pvParameters;
    int res;

    LOGM(TAG, "UART task started");
    while (1) {
    	if (self->end_task) break;
        // Waiting for UART event.
        if ((k210_uarts[self->uart_num].uart_buf->length > 0) &&
            (xSemaphoreTake(k210_uarts[self->uart_num].uart_mutex, UART_MUTEX_TIMEOUT) == pdTRUE)) {

            // Received data are already placed in ring buffer
            // Check if any callbacks are defined
            if ((self->error_cb) && (k210_uarts[self->uart_num].uart_buf->overflow > 0)) {
                // ring buffer full (overflow)
                self->error_cb(self->uart_num, UART_CB_TYPE_ERROR, UART_ERROR_BUFFER_FULL, NULL);
            }
            else {
                // No error
                if ((self->data_cb) && (self->data_cb_size > 0) && (k210_uarts[self->uart_num].uart_buf->length >= self->data_cb_size)) {
                    // === callback on data length received ===
                    uint8_t *dtmp = malloc(self->data_cb_size);
                    if (dtmp) {
                        uart_buf_get(k210_uarts[self->uart_num].uart_buf, dtmp, self->data_cb_size);
                        self->data_cb(self->uart_num, UART_CB_TYPE_DATA, self->data_cb_size, dtmp);
                        free(dtmp);
                    }
                    else self->data_cb(self->uart_num, UART_CB_TYPE_ERROR, UART_ERROR_NOMEM, NULL);
                }
                else if (self->pattern_cb) {
                    // === callback on pattern received ===
                    size_t len = k210_uarts[self->uart_num].uart_buf->length;
                    uint8_t *dtmp = malloc(len+self->pattern_len);
                    if (dtmp) {
                        uart_buf_copy(k210_uarts[self->uart_num].uart_buf, dtmp, len);
                        res = match_pattern(dtmp, len, self->pattern, self->pattern_len);
                        if (res >= 0) {
                            // found, pull data, including pattern from buffer
                            uart_buf_get(k210_uarts[self->uart_num].uart_buf, dtmp, res+self->pattern_len);
                            self->pattern_cb(self->uart_num, UART_CB_TYPE_PATTERN, res, dtmp);
                            free(dtmp);
                        }
                        else free(dtmp);
                    }
                    else self->pattern_cb(self->uart_num, UART_CB_TYPE_ERROR, UART_ERROR_NOMEM, NULL);
                }
            }
            xSemaphoreGive(k210_uarts[self->uart_num].uart_mutex);
        }
        else (vTaskDelay(2));
    }

    // Terminate task
    xSemaphoreTake(k210_uarts[self->uart_num].uart_mutex, UART_MUTEX_TIMEOUT);
    k210_uarts[self->uart_num].task_id = NULL;
    xSemaphoreGive(k210_uarts[self->uart_num].uart_mutex);
    LOGQ(TAG, "UART task ended");
    vTaskDelete(NULL);
}

//-------------------------------------------------------------------------------------
static char *_uart_read_data(handle_t uart_num, size_t len, char *lnend, char *lnstart)
{
    char *rdstr = NULL;
    int rdlen = -1;
    int out_len = -1;

    uint8_t *dtmp = malloc(len+1);
    if (dtmp) {
        memset(dtmp, 0, len+1);
        uint8_t *start_ptr = dtmp;
        out_len = len;

        // Copy buffer content to the temporary buffer
        uart_buf_copy(k210_uarts[uart_num].uart_buf, dtmp, len);
        // And try to match the end string
        rdlen = match_pattern(dtmp, len, (uint8_t *)lnend, strlen(lnend));

        if (rdlen > 0) {
            // End string found
            if (lnstart) {
                // Match beginning string requested
                int start_idx = match_pattern(dtmp, rdlen, (uint8_t *)lnstart, strlen(lnstart));
                if (start_idx >= 0) {
                    start_ptr += start_idx;
                    rdlen += strlen(lnend);
                    out_len = rdlen - start_idx;
                }
                else {
                    free(dtmp);
                    return NULL;
                }
            }
            else {
                rdlen += strlen(lnend);
                out_len = rdlen;
            }

            // copy found string to the result string
            rdstr = malloc(out_len+1);
            if (rdstr) {
                memcpy(rdstr, start_ptr, out_len);
                rdstr[out_len] = '\0';
            }
            else {
                // Leave the string in uart buffer
                free(dtmp);
                return NULL;
            }

            // All OK, remove data from uart buffer
            uart_buf_get(k210_uarts[uart_num].uart_buf, (uint8_t *)dtmp, rdlen);
            free(dtmp);
        }
        else {
            // End string not found
            free(dtmp);
        }
    }
    else {
        LOGD(TAG, "uart_read: error allocating temporary buffer");
    }

    return rdstr;
}

//--------------------------------------------------------------------------
char *uart_read(handle_t uart_num, int timeout, char *lnend, char *lnstart)
{
    char *rdstr = NULL;
    int minlen = strlen(lnend);
    if (lnstart) minlen += strlen(lnstart);

	if (timeout == 0) {
        if (xSemaphoreTake(k210_uarts[uart_num].uart_mutex, UART_MUTEX_TIMEOUT) != pdTRUE) {
            LOGD(TAG, "uart_read: cannot get mutex");
            return NULL;
        }
    	// check for minimal length
        size_t len = k210_uarts[uart_num].uart_buf->length;
		if (len < minlen) {
	    	xSemaphoreGive(k210_uarts[uart_num].uart_mutex);
	    	return NULL;
		}
		rdstr = _uart_read_data(uart_num, len, lnend, lnstart);

		xSemaphoreGive(k210_uarts[uart_num].uart_mutex);
    }
    else {
    	// wait until 'lnend' received or timeout
    	int wait_end = ticks_ms() + timeout;
        int buflen = 0;
    	//wdt_reset();
        size_t len = 0;

        while (ticks_ms() < wait_end) {
            if (xSemaphoreTake(k210_uarts[uart_num].uart_mutex, UART_MUTEX_TIMEOUT) != pdTRUE) {
                vTaskDelay(5 / portTICK_PERIOD_MS);
                //wdt_reset();
                continue;
            }
            len = k210_uarts[uart_num].uart_buf->length;
			if (buflen < len) {
				// ** new data received, reset timeout
				buflen = len;
		        wait_end = ticks_ms() + timeout;
			}
			if (len < minlen) {
				// ** too few characters received
		    	xSemaphoreGive(k210_uarts[uart_num].uart_mutex);
	    		vTaskDelay(5 / portTICK_PERIOD_MS);
				//wdt_reset();
				continue;
			}
			else {
                // Requested minimal length of bytes received
                rdstr = _uart_read_data(uart_num, len, lnend, lnstart);
                xSemaphoreGive(k210_uarts[uart_num].uart_mutex);
                if (rdstr == NULL) {
                    //wdt_reset();
                    continue;
                }
                break;
			}
		}
    }
	return rdstr;
}
