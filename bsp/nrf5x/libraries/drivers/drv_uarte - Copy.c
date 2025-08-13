 /*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-04-28     xckhmf       Modify for <nrfx>
 * 2020-10-31     xckhmf       Support for UART1
 *
 */
#include <rtdevice.h>
#include <nrfx_uarte.h>
#include "drv_uart.h"

#ifdef BSP_USING_UART
#ifndef BOARD_APP_UARTE_PIN_TX
#define BOARD_APP_UARTE_PIN_TX NRF_PIN_PORT_TO_PIN_NUMBER(0, 0)
#endif
#ifndef BOARD_APP_UARTE_PIN_RX
#define BOARD_APP_UARTE_PIN_RX NRF_PIN_PORT_TO_PIN_NUMBER(1, 0)
#endif
#ifndef BOARD_APP_UARTE_PIN_RTS
#define BOARD_APP_UARTE_PIN_RTS NRF_PIN_PORT_TO_PIN_NUMBER(2, 0)
#endif
#ifndef BOARD_APP_UARTE_PIN_CTS
#define BOARD_APP_UARTE_PIN_CTS NRF_PIN_PORT_TO_PIN_NUMBER(3, 0)
#endif
#if defined(BSP_USING_UART0) || defined(BSP_USING_UART1)|| defined(BSP_USING_UART2)|| defined(BSP_USING_UART3)
static  nrfx_uarte_t uarte_instance_te = NRFX_UARTE_INSTANCE(30);
typedef struct
{
    struct rt_serial_device *serial;
    nrfx_uarte_t uarte_instance;
    uint8_t rx_length;
    uint8_t tx_buffer[1];
    uint8_t rx_buffer[1];
    bool isInit;
    uint32_t rx_pin;
    uint32_t tx_pin;
} drv_uart_cb_t;

#ifdef BSP_USING_UART0
static struct rt_serial_device m_serial_0;
drv_uart_cb_t m_uarte0_cb = {
    .uarte_instance = NRFX_UARTE_INSTANCE(30),
    .rx_length = 0,
    .rx_pin = BOARD_APP_UARTE_PIN_RX,
    .tx_pin = BOARD_APP_UARTE_PIN_TX,
    .isInit = false
};
#endif  /* BSP_USING_UART0 */

#ifdef BSP_USING_UART1
static struct rt_serial_device m_serial_1;
drv_uart_cb_t m_uarte1_cb = {
    .uarte_instance = NRFX_UARTE_INSTANCE(1),
    .rx_length = 0,
    .rx_pin = BSP_UART1_RX_PIN,
    .tx_pin = BSP_UART1_TX_PIN,
    .isInit = false
};
#endif  /* BSP_USING_UART1 */
#ifdef BSP_USING_UART2
static struct rt_serial_device m_serial_2;
drv_uart_cb_t m_uarte2_cb = {
    .uarte_instance = NRFX_UARTE_INSTANCE(2),
    .rx_length = 0,
    .rx_pin = BSP_UART2_RX_PIN,
    .tx_pin = BSP_UART2_TX_PIN,
    .isInit = false
};
#endif  /* BSP_USING_UART2 */
#ifdef BSP_USING_UART3
static struct rt_serial_device m_serial_3;
drv_uart_cb_t m_uarte3_cb = {
    .uarte_instance = NRFX_UARTE_INSTANCE(3),
    .rx_length = 0,
    .rx_pin = BSP_UART3_RX_PIN,
    .tx_pin = BSP_UART3_TX_PIN,
    .isInit = false
};
#endif  /* BSP_USING_UART3 */


static void uarte_evt_handler(nrfx_uarte_event_t const * p_event,
                              void *                     p_context)
{
#if 1
    drv_uart_cb_t *p_cb = RT_NULL;
    p_cb = (drv_uart_cb_t*)p_context;
    switch (p_event->type)
    {
        case NRFX_UARTE_EVT_RX_DONE:
            p_cb->rx_length = p_event->data.rx.length;
            if(p_cb->serial->parent.open_flag&RT_DEVICE_FLAG_INT_RX)
            {
                rt_hw_serial_isr(p_cb->serial, RT_SERIAL_EVENT_RX_IND);
            }
            //(void)nrfx_uarte_rx(&(p_cb->uarte_instance), p_cb->rx_buffer, 1);
            /* Provide new UARTE RX buffer. */
		    nrfx_uarte_rx_enable(&(p_cb->uarte_instance), 0);
            break;
        case NRFX_UARTE_EVT_RX_BUF_REQUEST:
		nrfx_uarte_rx_buffer_set(&(p_cb->uarte_instance), p_cb->rx_buffer, 1);
		//buf_idx++;
		//buf_idx = (buf_idx < sizeof(uarte_rx_buf)) ? buf_idx : 0;
		break;

        case NRFX_UARTE_EVT_ERROR:
            //(void)nrfx_uarte_rx(&(p_cb->uarte_instance), p_cb->rx_buffer, 1);
            break;

        case NRFX_UARTE_EVT_TX_DONE:
            //if(p_cb->serial->parent.open_flag&RT_DEVICE_FLAG_INT_TX)
            {
                rt_hw_serial_isr(p_cb->serial, RT_SERIAL_EVENT_TX_DONE);
            }
            break;

        default:
            break;
    }
#endif
}

static rt_err_t _uart_cfg(struct rt_serial_device *serial, struct serial_configure *cfg)
{
#if 1
    int err;
    nrfx_uarte_config_t config = NRFX_UARTE_DEFAULT_CONFIG(BOARD_APP_UARTE_PIN_TX,
								     BOARD_APP_UARTE_PIN_RX);

    drv_uart_cb_t *p_cb = RT_NULL;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    #if defined(CONFIG_UARTE_HWFC)
	uarte_config.config.hwfc = NRF_UARTE_HWFC_ENABLED;
	uarte_config.cts_pin = BOARD_APP_UARTE_PIN_CTS;
	uarte_config.rts_pin = BOARD_APP_UARTE_PIN_RTS;
    #endif

    if (serial->parent.user_data == RT_NULL)
    {
        return -RT_ERROR;
    }
    p_cb = (drv_uart_cb_t*)serial->parent.user_data;
    if(p_cb->isInit)
    {
        nrfx_uarte_uninit(&(p_cb->uarte_instance));
        p_cb->isInit = false;
    }

    switch (cfg->baud_rate)
    {
    case BAUD_RATE_2400:
        config.baudrate = NRF_UARTE_BAUDRATE_2400;
        break;
    case BAUD_RATE_4800:
        config.baudrate = NRF_UARTE_BAUDRATE_4800;
        break;
    case BAUD_RATE_9600:
        config.baudrate = NRF_UARTE_BAUDRATE_9600;
        break;
    case BAUD_RATE_19200:
        config.baudrate = NRF_UARTE_BAUDRATE_19200;
        break;
    case BAUD_RATE_38400:
        config.baudrate = NRF_UARTE_BAUDRATE_38400;
        break;
    case BAUD_RATE_57600:
        config.baudrate = NRF_UARTE_BAUDRATE_57600;
        break;
    case BAUD_RATE_115200:
        config.baudrate = NRF_UARTE_BAUDRATE_115200;
        break;
    case BAUD_RATE_230400:
        config.baudrate = NRF_UARTE_BAUDRATE_230400;
        break;
    case BAUD_RATE_460800:
        config.baudrate = NRF_UARTE_BAUDRATE_460800;
        break;
    case BAUD_RATE_921600:
        config.baudrate = NRF_UARTE_BAUDRATE_921600;
        break;
#if defined(SOC_NRF5340)
    case 1000000:
        config.baudrate = NRF_UARTE_BAUDRATE_1000000;
        break;
#endif /* SOC_NRF5340*/
    case BAUD_RATE_2000000:
    case BAUD_RATE_3000000:
        return -RT_EINVAL;
    default:
        config.baudrate = NRF_UARTE_BAUDRATE_115200;
        break;
    }
   // config.parity = (cfg->parity == PARITY_NONE)?\
   //                         NRF_UARTE_PARITY_EXCLUDED:NRF_UARTE_PARITY_INCLUDED;
    //config.hal_cfg.hwfc = NRF_UARTE_HWFC_DISABLED;
    //config.pselrxd = p_cb->rx_pin;
    //config.pseltxd = p_cb->tx_pin;
    config.p_context = (void *)p_cb;

    err = nrfx_uarte_init(&(p_cb->uarte_instance),&config,uarte_evt_handler);
    if (err != NRFX_SUCCESS) {
		return RT_ERROR;
	}
    //nrfx_uarte_rx(&(p_cb->uarte_instance),p_cb->rx_buffer,1);
    /* Start reception */
	err = nrfx_uarte_rx_enable(&(p_cb->uarte_instance), 0);
	if (err != NRFX_SUCCESS) {
		//printk("UARTE RX failed, nrfx err %d\n", err);
        	return RT_ERROR;
	}
    p_cb->isInit = true;
#endif
    return RT_EOK;
}

static rt_err_t _uart_ctrl(struct rt_serial_device *serial, int cmd, void *arg)
{
    #if 1
    drv_uart_cb_t *p_cb = RT_NULL;
    RT_ASSERT(serial != RT_NULL);

    if (serial->parent.user_data == RT_NULL)
    {
        return -RT_ERROR;
    }
    p_cb = (drv_uart_cb_t*)serial->parent.user_data;

    switch (cmd)
    {
        /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        break;

        /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        break;

    case RT_DEVICE_CTRL_CUSTOM:
        if ((rt_uint32_t)(arg) == UART_CONFIG_BAUD_RATE_9600)
        {
            p_cb->serial->config.baud_rate = 9600;
        }
        else if ((rt_uint32_t)(arg) == UART_CONFIG_BAUD_RATE_115200)
        {
            p_cb->serial->config.baud_rate = 115200;
        }
        _uart_cfg(serial, &(serial->config));
        break;

    case RT_DEVICE_CTRL_PIN:
        _uart_cfg(serial, &(serial->config));
        break;

    case RT_DEVICE_POWERSAVE:
        if(p_cb->isInit)
        {
            nrfx_uarte_uninit(&(p_cb->uarte_instance));
            p_cb->isInit = false;
        }
        break;

    case RT_DEVICE_WAKEUP:
        _uart_cfg(serial, &(serial->config));
        break;

    default:
        return -RT_ERROR;
    }
#endif
    return RT_EOK;
}

static int _uart_putc(struct rt_serial_device *serial, char c)
{
    #if 1
    nrfx_err_t err;
    drv_uart_cb_t *p_cb = RT_NULL;
    int rtn = -1;
    RT_ASSERT(serial != RT_NULL);

    if (serial->parent.user_data != RT_NULL)
    {
        p_cb = (drv_uart_cb_t*)serial->parent.user_data;
    }
    p_cb->tx_buffer[0] = c;
    nrfx_uarte_tx(&(p_cb->uarte_instance),p_cb->tx_buffer,1,NRFX_UARTE_TX_BLOCKING);
    if (err != NRFX_SUCCESS) {
				return RT_ERROR;
			}
/*    if(!(serial->parent.open_flag&RT_DEVICE_FLAG_INT_TX))
    {
        while(nrfx_uarte_tx_in_progress(&(p_cb->uarte_instance)))
        {
        }
    }*/
    return RT_EOK;
#endif 
}

static int _uart_getc(struct rt_serial_device *serial)
{
#if 1
    int ch = -1;
    drv_uart_cb_t *p_cb = RT_NULL;
    RT_ASSERT(serial != RT_NULL);

    if (serial->parent.user_data != RT_NULL)
    {
        p_cb = (drv_uart_cb_t*)serial->parent.user_data;
    }
    if(p_cb->rx_length)
    {
        ch = p_cb->rx_buffer[0];
        p_cb->rx_length--;
    }
    return ch;
    #endif
}

static struct rt_uart_ops _uart_ops = {
    _uart_cfg,
    _uart_ctrl,
    _uart_putc,
    _uart_getc
};

int rt_hw_uart_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef BSP_USING_UART0
    m_serial_0.config = config;
#if defined(SOC_NRF5340)
    m_serial_0.config.baud_rate =  1000000;
#endif /* SOC_NRF5340*/
    m_serial_0.ops = &_uart_ops;
    m_uarte0_cb.serial = &m_serial_0;
    rt_hw_serial_register(&m_serial_0, "uart0", \
                            RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX ,  &m_uarte0_cb);
#endif  /* BSP_USING_UART0 */

#ifdef BSP_USING_UART1
    m_serial_1.config = config;
    m_serial_1.ops = &_uart_ops;
    m_uarte1_cb.serial = &m_serial_1;
    rt_hw_serial_register(&m_serial_1, "uart1", \
                            RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX |RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,  &m_uarte1_cb);
#endif  /* BSP_USING_UART1 */

#ifdef BSP_USING_UART2
    m_serial_2.config = config;
    m_serial_2.ops = &_uart_ops;
    m_uarte2_cb.serial = &m_serial_2;
    rt_hw_serial_register(&m_serial_2, "uart2", \
                            RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX |RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,  &m_uarte2_cb);
#endif  /* BSP_USING_UART2 */

#ifdef BSP_USING_UART3
    m_serial_3.config = config;
    m_serial_3.ops = &_uart_ops;
    m_uarte3_cb.serial = &m_serial_3;
    rt_hw_serial_register(&m_serial_3, "uart3", \
                            RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX |RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,  &m_uarte3_cb);
#endif  /* BSP_USING_UART3 */

        return RT_EOK;
}
#endif /* defined(BSP_USING_UART0) || defined(BSP_USING_UART1) */
#endif /* BSP_USING_UART */
