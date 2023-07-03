/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/pm/device.h>

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define UART_DEVICE_NAME         DT_NODE_FULL_NAME(DT_NODELABEL(my_uart))

#define UART_BUF_SIZE 255
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(100)
#define UART_WAIT_FOR_RX 10000

static const struct device *uart;
typedef void (*rx_callback_t)(uint8_t *data, uint16_t len);
static rx_callback_t rx_callback;
const uint8_t help_string[] = "Send data from PC first, the connected device would echo back\r";

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static uint8_t uart_rx_buf[2][UART_BUF_SIZE];
static uint8_t *next_buf = uart_rx_buf[1];

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

// enum pm_device_action {
// 	/** Suspend. */
// 	PM_DEVICE_ACTION_SUSPEND,
// 	/** Resume. */
// 	PM_DEVICE_ACTION_RESUME,
// 	/** Turn off. */
// 	PM_DEVICE_ACTION_TURN_OFF,
// 	/** Force suspend. */
// 	PM_DEVICE_ACTION_FORCE_SUSPEND,
// };

int poweroff_uart(void)
{
	int err;
	uart_rx_disable(uart);
	k_sleep(K_MSEC(100));
	err = pm_device_action_run(uart, PM_DEVICE_ACTION_TURN_OFF);
	if (err) {
		LOG_ERR("Can't suspend uart: %d", err);
	}
	return err;
}

int poweron_uart(void)
{
	//#define SLM_SYNC_STR	"Ready\r\n"
	int err;
	// NRF_UARTE0 -> PSEL.TXD =  0x09;   //tx=p0.09
 	// NRF_UARTE0 -> PSEL.RXD =  0x0A;   //rx=p0.10	
	err = pm_device_action_run(uart, PM_DEVICE_ACTION_TURN_ON);
	// NRF_UARTE0 -> PSEL.TXD =  0x03;   //tx=p0.03
 	// NRF_UARTE0 -> PSEL.RXD =  0x04;   //rx=p0.04
	if (err == -EALREADY) {
		/* Already on, no action */
		return 0;
	}
	if (err) {
		return err;
	}
	k_sleep(K_MSEC(100));
	err = uart_receive();
	if (err) {
		return err;
	}	
	// //k_sem_give(&tx_done);
	// #if 0
	// rsp_send(SLM_SYNC_STR, sizeof(SLM_SYNC_STR)-1);
	// #endif

	return 0;
}



static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);
	int err;

	switch (evt->type) {
	case UART_TX_DONE:
	{
		struct uart_data_t *buf;
		struct uart_data_t *buf2;
		
		buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,  data);		
		LOG_INF("UART_TX_DONE %d", evt->data.tx.len);
		k_free(buf);

		buf2 = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf2) {
			return;
		}

		if (uart_tx(uart, buf2->data, buf2->len, SYS_FOREVER_MS)) {
			LOG_WRN("uart_tx fail @ cb");
		}
	}
		break;

	case UART_RX_RDY:
	{
		struct uart_data_t *buf = k_malloc(sizeof(struct uart_data_t));
		memcpy(buf->data, &evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);	
		buf->len = evt->data.rx.len;		
		k_fifo_put(&fifo_uart_rx_data, buf);
		LOG_INF("UART_RX_RDY %d", buf->len);
	}
		break;

	case UART_RX_DISABLED:
		LOG_INF("UART_RX_DISABLED");
		err = uart_rx_enable(uart, uart_rx_buf[0], sizeof(uart_rx_buf[0]), UART_WAIT_FOR_RX);
		if (err) {
			LOG_ERR("UART RX enable failed: %d", err);			
		}
		break;

	case UART_RX_BUF_REQUEST:
		err = uart_rx_buf_rsp(uart, next_buf,
			sizeof(uart_rx_buf[0]));
		if (err) {
			LOG_WRN("UART RX buf rsp: %d", err);
		}		
		break;

	case UART_RX_BUF_RELEASED:
		LOG_INF("UART_RX_BUF_RELEASED");
		next_buf = evt->data.rx_buf.buf;
		break;

	case UART_TX_ABORTED:
		LOG_INF("UART_TX_ABORTED");
		break;

	default:
		break;
	}
}

int uart_init(rx_callback_t cb)
{
	int err;
	
	uart = device_get_binding(UART_DEVICE_NAME);
	if (!uart) {
		return -ENXIO;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		return err;
	}

	rx_callback = cb;

	return uart_rx_enable(uart, uart_rx_buf[0], sizeof(uart_rx_buf[0]), UART_WAIT_FOR_RX);

}

int uart_send(const uint8_t *buf, uint16_t len)
{
	int err;
	struct uart_data_t *tx = k_malloc(sizeof(*tx));

	if (!tx) {
		LOG_WRN("Not able to allocate UART send data buffer");
		return -ENOMEM; 
	}

	memcpy(tx->data, buf, len);	
	tx->len = len;
	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		LOG_WRN("buffer uart tx data for later retry");
		k_fifo_put(&fifo_uart_tx_data, tx);
	}
	return err;		
}

void uart_receive()
{
	/* Wait indefinitely*/
	struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
							K_FOREVER);
	rx_callback(buf->data,buf->len);							
	k_free(buf);		
}

void uart_rx_cb(uint8_t *data, uint16_t len)
{
	int err;
	err = uart_send(data, len);	
	if (err)
	{
		LOG_INF("uart send err %d", err);
	}
}

static void button_handler(uint32_t button_state, uint32_t has_changed) 
{
	int err = 0;
	switch (has_changed) {
	case DK_BTN1_MSK:
		if (button_state & DK_BTN1_MSK){	
			err = poweroff_uart();
			if (err) {
				LOG_INF("uart poweroff err %d", err);
			}				
		}
		break;

	case DK_BTN2_MSK:
		if (button_state & DK_BTN2_MSK){	
			err = poweron_uart();
			if( err){
				LOG_INF("uart poweron err %d", err);			
			}
		}
		break;
	}
}

void main(void)
{
	int err = 0;

	LOG_INF("### high speed UART example %s %s\n", __TIME__, __DATE__);	
	err = dk_buttons_init(button_handler);
	if (err) {
		LOG_ERR("Buttons init failed (err: %d)", err);
		return err;
	}

	err = uart_init(uart_rx_cb);
	if (err) {
		LOG_ERR("uart init err %d", err);
		return;
	}
	
	uart_send(help_string, sizeof(help_string));
	for (;;) {
		/* Wait indefinitely for UART data*/
		uart_receive();
	}
}

