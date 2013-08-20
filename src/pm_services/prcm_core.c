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

#include <stdint.h>

#include <stddef.h>
#include <cm3.h>
#include <device_am335x.h>
#include <prcm_core.h>
#include <system_am335.h>
#include <hwmod.h>
#include <powerdomain.h>
#include <clockdomain.h>
#include <dpll.h>
#include <ddr.h>
#include <ldo.h>
#include <msg.h>
#include <i2c.h>

#define BITBAND_SRAM_REF 	UMEM_ALIAS
#define BITBAND_SRAM_BASE 	0x22000000
#define BITBAND_SRAM(a,b) 	((BITBAND_SRAM_BASE + ((int)(a))*32 + (b*4)))

#define BITBAND_PERI_REF 	DMEM_ALIAS
#define BITBAND_PERI_BASE 	0x42000000
#define BITBAND_PERI(a,b) 	((BITBAND_PERI_BASE + (*(a) - BITBAND_PERI_REF)*32 + (b*4)))

#define BB_USB_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 0)))
#define BB_I2C0_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 1)))
#define BB_RTC_ALARM_WAKE	*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 2)))
#define BB_TIMER1_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 3)))
#define BB_UART0_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 4)))
#define BB_GPIO0_WAKE0		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 5)))
#define BB_GPIO0_WAKE1		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 6)))
#define BB_WDT1_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 7)))
#define BB_ADTSC_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 8)))
/* Not used currently */
#define BB_RTC_TIMER_WAKE	*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 9)))
#define BB_USBWOUT0		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 10)))
#define BB_MPU_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 11)))
#define BB_USBWOUT1		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 12)))

static unsigned int cmd_wake_sources;

unsigned int soc_id;
unsigned int soc_rev;
unsigned int soc_type;

/* Clear out the global variables here */
void pm_reset(void)
{
	cmd_global_data.cmd_id 	= CMD_ID_INVALID;
	cmd_global_data.data 	= NULL;

	powerdomain_reset();
	dpll_reset();
}

void setup_soc(void)
{
	unsigned int var = __raw_readl(DEVICE_ID);

	soc_id = (var & DEVICE_ID_PARTNUM_MASK ) >> DEVICE_ID_PARTNUM_SHIFT;
	soc_rev = (var & DEVICE_ID_DEVREV_MASK) >> DEVICE_ID_DEVREV_SHIFT;

	var = __raw_readl(CONTROL_STATUS);
	soc_type = (var & CONTROL_STATUS_DEVTYPE_MASK) >> CONTROL_STATUS_DEVTYPE_SHIFT;

	clockdomain_init();
	hwmod_init();
	powerdomain_init();
	dpll_init();
	ldo_init();
}

/* DeepSleep related */
int disable_master_oscillator(void)
{
	unsigned int v = __raw_readl(DEEPSLEEP_CTRL);

	v = var_mod(v, DS_ENABLE_MASK, (1 << DS_ENABLE_SHIFT));

	__raw_writel(v, DEEPSLEEP_CTRL);

	return 0;
}

int enable_master_oscillator(void)
{
	unsigned int v = __raw_readl(DEEPSLEEP_CTRL);

	v = var_mod(v, DS_ENABLE_MASK, (0 << DS_ENABLE_SHIFT));

	__raw_writel(v, DEEPSLEEP_CTRL);

	return 0;
}

void configure_deepsleep_count(int ds_count)
{
	unsigned int v = __raw_readl(DEEPSLEEP_CTRL);

	v = var_mod(v, DS_COUNT_MASK, (ds_count << DS_COUNT_SHIFT));

	__raw_writel(v, DEEPSLEEP_CTRL);
}

/*
 * A8 is expected to have left the module in a state where it will
 * cause a wakeup event. Ideally, this function should just enable
 * the NVIC interrupt
 */
void configure_wake_sources(int wake_sources)
{
	cmd_wake_sources = wake_sources;

	/* Enable wakeup interrupts from required wake sources */
	if (BB_USB_WAKE)
		nvic_enable_irq(AM335X_IRQ_USBWAKEUP);

	if(BB_I2C0_WAKE)
		nvic_enable_irq(AM335X_IRQ_I2C0_WAKE);

	if(BB_ADTSC_WAKE)
		nvic_enable_irq(AM335X_IRQ_ADC_TSC_WAKE);

	if(BB_UART0_WAKE)
		nvic_enable_irq(AM335X_IRQ_UART0_WAKE);

	if(BB_GPIO0_WAKE0)
		nvic_enable_irq(AM335X_IRQ_GPIO0_WAKE0);

	if(BB_GPIO0_WAKE1)
		nvic_enable_irq(AM335X_IRQ_GPIO0_WAKE1);

	if(BB_RTC_ALARM_WAKE)
		nvic_enable_irq(AM335X_IRQ_RTC_ALARM_WAKE);

	if(BB_TIMER1_WAKE)
		nvic_enable_irq(AM335X_IRQ_TIMER1_WAKE);

	if(BB_WDT1_WAKE)
		nvic_enable_irq(AM335X_IRQ_WDT1_WAKE);

#if 0
	/* Not recommended */
	if(BB_RTC_TIMER_WAKE)
		nvic_enable_irq(AM335X_IRQ_RTC_TIMER_WAKE);

	if(BB_TIMER0_WAKE)
		nvic_enable_irq(AM335X_IRQ_TIMER0_WAKE);

	if(BB_WDT0_WAKE)
		nvic_enable_irq(AM335X_IRQ_WDT0_WAKE);
#endif

	if(BB_USBWOUT0)
		nvic_enable_irq(AM335X_IRQ_USB0WOUT);

	if(BB_USBWOUT1)
		nvic_enable_irq(AM335X_IRQ_USB1WOUT);

	if(BB_MPU_WAKE)
		nvic_enable_irq(AM335X_IRQ_MPU_WAKE);
}

void clear_wake_sources(void)
{
	/*
	 * Clear the global variable
	 * and then disable all wake interrupts
	 */

	cmd_wake_sources = 0x0;	/* All disabled */

	/* Disable the wake interrupts */
	nvic_disable_irq(AM335X_IRQ_USBWAKEUP);
	nvic_disable_irq(AM335X_IRQ_I2C0_WAKE);
	nvic_disable_irq(AM335X_IRQ_RTC_TIMER_WAKE);
	nvic_disable_irq(AM335X_IRQ_RTC_ALARM_WAKE);
	nvic_disable_irq(AM335X_IRQ_TIMER0_WAKE);
	nvic_disable_irq(AM335X_IRQ_TIMER1_WAKE);
	nvic_disable_irq(AM335X_IRQ_UART0_WAKE);
	nvic_disable_irq(AM335X_IRQ_GPIO0_WAKE0);
	nvic_disable_irq(AM335X_IRQ_GPIO0_WAKE1);
	nvic_disable_irq(AM335X_IRQ_MPU_WAKE);
	nvic_disable_irq(AM335X_IRQ_WDT0_WAKE);
	nvic_disable_irq(AM335X_IRQ_WDT1_WAKE);
	nvic_disable_irq(AM335X_IRQ_ADC_TSC_WAKE);
	nvic_disable_irq(AM335X_IRQ_USB0WOUT);
	nvic_disable_irq(AM335X_IRQ_USB1WOUT);

	/* TODO: Clear all the pending interrupts */
}

void ds_save(void)
{
	if (soc_id == AM335X_SOC_ID) {
		set_ddr_reset();

		ddr_io_suspend();

		vtt_low();

		vtp_disable();

		ldo_power_down(LDO_MPU);

		pll_bypass(DPLL_CORE);
		pll_bypass(DPLL_DDR);
		pll_bypass(DPLL_DISP);
		pll_bypass(DPLL_PER);
		pll_bypass(DPLL_MPU);
	}
}

void ds_restore(void)
{
	if (soc_id == AM335X_SOC_ID) {
		pll_lock(DPLL_MPU);
		pll_lock(DPLL_PER);
		pll_lock(DPLL_DISP);
		pll_lock(DPLL_DDR);
		pll_lock(DPLL_CORE);

		ldo_power_up(LDO_MPU);

		vtp_enable();

		/* XXX: Why is this required here for DDR3? */
		hwmod_enable(HWMOD_EMIF);

		vtt_high();

		ddr_io_resume();

		clear_ddr_reset();
	}
}

int a8_i2c_sleep_handler(unsigned short i2c_sleep_offset)
{
	unsigned char *dmem = (unsigned char *) DMEM_BASE;
	int ret = 0;

	hwmod_enable(HWMOD_I2C0);

	if (i2c_sleep_offset != 0xffff)
		ret = i2c_write(dmem + i2c_sleep_offset);

	hwmod_disable(HWMOD_I2C0);

	return ret;
}

int a8_i2c_wake_handler(unsigned short i2c_wake_offset)
{
	unsigned char *dmem = (unsigned char *) DMEM_BASE;
	int ret = 0;

	hwmod_enable(HWMOD_I2C0);

	if (i2c_wake_offset != 0xffff)
		ret = i2c_write(dmem + i2c_wake_offset);

	hwmod_disable(HWMOD_I2C0);

	return ret;
}
