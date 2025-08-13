/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-09     Andrew       first version
 *
 */
#include <rtthread.h>
#include <rthw.h>
#include <nrfx_grtc.h>

#include "board.h"
#include "drv_uart.h"
#include <nrfx_clock.h>

#define GRTCIRQ 0xe4
#define SYS_CLOCK_HW_CYCLES_PER_SEC 1000000
#define SYS_CLOCK_TICKS_PER_SEC 1000
#define CYC_PER_TICK                                                                               \
	((uint64_t)SYS_CLOCK_HW_CYCLES_PER_SEC / (uint64_t)SYS_CLOCK_TICKS_PER_SEC)

volatile uint8_t init_done = 0;
static void sys_clock_timeout_handler(int32_t id, uint64_t cc_val, void *p_context);
static int sys_clock_driver_init(void);
static nrfx_grtc_channel_t system_clock_channel_data = {
	.handler = sys_clock_timeout_handler,
	.p_context = NULL,
	.channel = (uint8_t)-1,
};
static uint64_t last_count; /* Time (SYSCOUNTER value) @last sys_clock_announce() */
static inline uint64_t counter(void)
{
	uint64_t now;
	nrfx_grtc_syscounter_get(&now);
	return now;
}
static inline uint64_t counter_sub(uint64_t a, uint64_t b)
{
	return (a - b);
}
/*
 * Program a new callback in the absolute time given by <value>
 */
static void system_timeout_set_abs(uint64_t value)
{
	nrfx_grtc_syscounter_cc_absolute_set(&system_clock_channel_data, value,
					     true);
}
static void sys_clock_timeout_handler(int32_t id, uint64_t cc_val, void *p_context)
{
	//ARG_UNUSED(id);
	//ARG_UNUSED(p_context);
	uint64_t dticks;
	uint64_t now = counter();

	//if (unlikely(now < cc_val)) {
	//	return;
	//}

	dticks = counter_sub(cc_val, last_count) / CYC_PER_TICK;

	last_count += dticks * CYC_PER_TICK;


	system_timeout_set_abs(last_count + CYC_PER_TICK);

	if( init_done == 1 )
    {   /* enter interrupt */
    	rt_interrupt_enter();

    	rt_tick_increase();

    	/* leave interrupt */
    	rt_interrupt_leave();
	}
}
/**
 * This is the timer interrupt service routine.
 *
 
void SysTick_Handler(void)
{
   
    rt_interrupt_enter();

    rt_tick_increase();

    rt_interrupt_leave();
}*/

static void clk_event_handler(nrfx_clock_evt_type_t event){}

void SysTick_Configuration(void)
{
	nrfx_clock_init(clk_event_handler);


	
    nrfx_clock_enable();

    sys_clock_driver_init();
//	NVIC_SetPriority(GRTCIRQ, 1);
//    NVIC_EnableIRQ(GRTCIRQ);
	nrfx_clock_lfclk_start();
    /* Set interrupt priority */
    //NVIC_SetPriority(SysTick_IRQn, 0xf);

    /* Configure SysTick to interrupt at the requested rate.
    nrf_systick_load_set(SystemCoreClock / RT_TICK_PER_SECOND);
    nrf_systick_val_clear();
    nrf_systick_csr_set(NRF_SYSTICK_CSR_CLKSOURCE_CPU | NRF_SYSTICK_CSR_TICKINT_ENABLE
                        | NRF_SYSTICK_CSR_ENABLE);
 */
}



static void system_timeout_set_relative(uint64_t value)
{
	if (value <= NRF_GRTC_SYSCOUNTER_CCADD_MASK) {
		nrfx_grtc_syscounter_cc_relative_set(&system_clock_channel_data, value, true,
						     NRFX_GRTC_CC_RELATIVE_SYSCOUNTER);
	} else {
		nrfx_grtc_syscounter_cc_absolute_set(&system_clock_channel_data, value + counter(),
						     true);
	}
}

static int sys_clock_driver_init(void)
{
	nrfx_err_t err_code;

	//IRQ_CONNECT(DT_IRQN(GRTC_NODE), DT_IRQ(GRTC_NODE, priority), nrfx_isr,
	//	    nrfx_grtc_irq_handler, 0);

    nrfx_grtc_clock_source_set(NRF_GRTC_CLKSEL_LFXO);

	err_code = nrfx_grtc_init(0);
	if (err_code != NRFX_SUCCESS) {
		return -1;
	}


	err_code = nrfx_grtc_syscounter_start(true, &system_clock_channel_data.channel);
	if (err_code != NRFX_SUCCESS) {
		return -1;
	}


	//int_mask = NRFX_GRTC_CONFIG_ALLOWED_CC_CHANNELS_MASK;
	
	system_timeout_set_relative(CYC_PER_TICK);
	
	//z_nrf_clock_control_lf_on(CLOCK_CONTROL_NRF_LF_START_STABLE);

    //NVIC_SetPriority(GRTCIRQ, 1);
    //NVIC_EnableIRQ(GRTCIRQ);
	return 0;

}


void rt_hw_board_init(void)
{
    rt_hw_interrupt_enable(0);

    SysTick_Configuration();

#if defined(RT_USING_HEAP)
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
#endif

#ifdef RT_USING_SERIAL
    rt_hw_uart_init();
#endif

#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif


#ifdef BSP_USING_SOFTDEVICE
    extern uint32_t  Image$$RW_IRAM1$$Base;
    uint32_t const *const m_ram_start  = &Image$$RW_IRAM1$$Base;
    if ((uint32_t)m_ram_start == 0x20000000)
    {
        rt_kprintf("\r\n using softdevice the RAM couldn't be %p,please use the templete from package\r\n", m_ram_start);
        while (1);
    }
    else
    {
        rt_kprintf("\r\n using softdevice the RAM at %p\r\n", m_ram_start);
    }
#endif

}

