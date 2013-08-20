/*
 * AM33XX-CM3 firmware
 *
 * Cortex-M3 (CM3) firmware for power management on Texas Instruments' AM33XX series of SoCs
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#include <cm3.h>
#include <device_cm3.h>
#include <prcm_core.h>
#include <msg.h>
#include <hwmod.h>
#include <trace.h>
#include <sync.h>

void a8_notify(int cmd_stat_value)
{
	msg_cmd_stat_update(cmd_stat_value);
	__asm("sev");
}

/* If only notification is needed, use the a8_notify() API */
void a8_m3_low_power_sync(int cmd_stat_value)
{
	/* Disable this part for now */
	a8_notify(cmd_stat_value);

	/* Enable the PRCM interrupt for MPU gated state */
	nvic_clear_irq(CM3_IRQ_PRCM_M3_IRQ2);
	nvic_enable_irq(CM3_IRQ_PRCM_M3_IRQ2);
}

void a8_m3_low_power_fast(int cmd_stat_value)
{
	msg_cmd_stat_update(cmd_stat_value);

	/* Enable the PRCM interrupt for MPU gated state */
	nvic_clear_irq(CM3_IRQ_PRCM_M3_IRQ2);
	nvic_enable_irq(CM3_IRQ_PRCM_M3_IRQ2);
}

void init_m3_state_machine(void)
{
	int i;

	/* Flush out NVIC interrupts */
	for (i = 0; i < CM3_NUM_EXT_INTERRUPTS; i++)
	{
		nvic_disable_irq(i);
		nvic_clear_irq(i);
	}

	trace_init();

	pm_reset();

	/* Enable only the MBX IRQ */
	nvic_enable_irq(CM3_IRQ_MBINT0);
	nvic_enable_irq(53);

	/*
	 * In the remote case where we disabled the MPU CLOCK
	 * enable it again, no harm in writing to the reg
	 * even if this was not needed
	 */
	hwmod_enable(HWMOD_MPU);
	a8_notify(CMD_STAT_PASS);
}

/* Timer based sync scheme - Not required for now */
void timer_sync(void)
{

}
