/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-04-29     supperthomas first version
 * 2021-06-26     supperthomas fix led
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <hal/nrf_gpio.h>
/** @brief Macro for extracting absolute pin number from the relative pin and port numbers. */
#define NRF_PIN_PORT_TO_PIN_NUMBER(pin, port) (((pin) & 0x1F) | ((port) << 5))
#define BOARD_PIN_LED_1 NRF_PIN_PORT_TO_PIN_NUMBER(10, 1)

#ifndef BOARD_PIN_LED_0
#define BOARD_PIN_LED_0 NRF_PIN_PORT_TO_PIN_NUMBER(9, 2)
#endif
#ifndef BOARD_PIN_LED_1
#define BOARD_PIN_LED_1 NRF_PIN_PORT_TO_PIN_NUMBER(10, 1)
#endif
#ifndef BOARD_PIN_LED_2
#define BOARD_PIN_LED_2 NRF_PIN_PORT_TO_PIN_NUMBER(7, 2)
#endif
#ifndef BOARD_PIN_LED_3
#define BOARD_PIN_LED_3 NRF_PIN_PORT_TO_PIN_NUMBER(14, 1)
#endif
extern int rtthread_startup(void);
int app(void)
{
    int count = 1;
    //rt_pin_mode(RT_BSP_LED_PIN, PIN_MODE_OUTPUT);
nrf_gpio_cfg_output(BOARD_PIN_LED_0);
nrf_gpio_cfg_output(BOARD_PIN_LED_1);
nrf_gpio_cfg_output(BOARD_PIN_LED_2);
nrf_gpio_cfg_output(BOARD_PIN_LED_3);
    while (count++)
    {
        //rt_pin_write(RT_BSP_LED_PIN, PIN_HIGH);
        nrf_gpio_pin_write(BOARD_PIN_LED_0, 1);
        nrf_gpio_pin_write(BOARD_PIN_LED_1, 1);
        nrf_gpio_pin_write(BOARD_PIN_LED_2, 1);
        nrf_gpio_pin_write(BOARD_PIN_LED_3, 1);
        rt_thread_mdelay(3000);
        nrf_gpio_pin_write(BOARD_PIN_LED_0, 0);
        nrf_gpio_pin_write(BOARD_PIN_LED_1, 0);
        nrf_gpio_pin_write(BOARD_PIN_LED_2, 0);
        nrf_gpio_pin_write(BOARD_PIN_LED_3, 0);
        //rt_pin_write(RT_BSP_LED_PIN, PIN_LOW);
        rt_thread_mdelay(3000);
    }
    return RT_EOK;
}
int main(void)
{
#ifndef __ARMCC_VERSION	
	rtthread_startup();
#endif	
    return RT_EOK;
}

