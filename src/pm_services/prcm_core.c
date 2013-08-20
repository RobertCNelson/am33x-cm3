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
#include <low_power.h>
#include <prcm.h>
#include <prmam335x.h>
#include <prm43xx.h>
#include <system_am335.h>
#include <clockdomain.h>
#include <hwmod.h>
#include <powerdomain.h>
#include <dpll.h>
#include <ddr.h>

union state_data rtc_mode_data = {
	.rtc = {
		.rtc_timeout_val		= RTC_TIMEOUT_DEFAULT
	},
};

union state_data ds0_data = {
	.deep_sleep = {
		.mosc_state 			= MOSC_OFF,
		.deepsleep_count 		= DS_COUNT_DEFAULT,

		.pd_mpu_state 			= PD_OFF,
		.pd_mpu_ram_ret_state 		= MEM_BANK_RET_ST_OFF,
		.pd_mpu_l1_ret_state 		= MEM_BANK_RET_ST_OFF,
		.pd_mpu_l2_ret_state 		= MEM_BANK_RET_ST_OFF,

		.pd_per_state 			= PD_RET,
		.pd_per_icss_mem_ret_state 	= MEM_BANK_RET_ST_OFF,
		.pd_per_mem_ret_state 		= MEM_BANK_RET_ST_OFF,
		.pd_per_ocmc_ret_state 		= MEM_BANK_RET_ST_RET,

		.wake_sources 			= WAKE_ALL,
	},
};

/* In case of HS devices MPU RAM must be held in retention */
union state_data ds0_data_hs = {
	.deep_sleep = {
		.mosc_state 			= MOSC_OFF,
		.deepsleep_count 		= DS_COUNT_DEFAULT,

		.pd_mpu_state 			= PD_RET,
		.pd_mpu_ram_ret_state 		= MEM_BANK_RET_ST_RET,
		.pd_mpu_l1_ret_state 		= MEM_BANK_RET_ST_OFF,
		.pd_mpu_l2_ret_state 		= MEM_BANK_RET_ST_OFF,

		.pd_per_state 			= PD_RET,
		.pd_per_icss_mem_ret_state 	= MEM_BANK_RET_ST_OFF,
		.pd_per_mem_ret_state 		= MEM_BANK_RET_ST_OFF,
		.pd_per_ocmc_ret_state 		= MEM_BANK_RET_ST_RET,

		.wake_sources 			= WAKE_ALL,
	},
};

union state_data ds1_data = {
	.deep_sleep = {
		.mosc_state 			= MOSC_OFF,
		.deepsleep_count 		= DS_COUNT_DEFAULT,

		.pd_mpu_state 			= PD_OFF,
		.pd_mpu_ram_ret_state 		= MEM_BANK_RET_ST_OFF,
		.pd_mpu_l1_ret_state 		= MEM_BANK_RET_ST_OFF,
		.pd_mpu_l2_ret_state 		= MEM_BANK_RET_ST_OFF,

		.pd_per_state 			= PD_ON,

		.wake_sources 			= WAKE_ALL,
	},
};

/* In case of HS devices MPU RAM must be held in retention */
union state_data ds1_data_hs = {
	.deep_sleep = {
		.mosc_state 			= MOSC_OFF,
		.deepsleep_count 		= DS_COUNT_DEFAULT,

		.pd_mpu_state 			= PD_RET,
		.pd_mpu_ram_ret_state 		= MEM_BANK_RET_ST_RET,
		.pd_mpu_l1_ret_state 		= MEM_BANK_RET_ST_OFF,
		.pd_mpu_l2_ret_state 		= MEM_BANK_RET_ST_OFF,

		.pd_per_state 			= PD_ON,

		.wake_sources 			= WAKE_ALL,
	},
};

union state_data ds2_data = {
	.deep_sleep = {
		.mosc_state 			= MOSC_OFF,
		.deepsleep_count 		= DS_COUNT_DEFAULT,

		.pd_mpu_state 			= PD_ON,

		.pd_per_state 			= PD_ON,

		.wake_sources 			= WAKE_ALL,
	},
};

union state_data standby_data = {
	.deep_sleep = {
		.mosc_state			= MOSC_ON,
		.deepsleep_count		= DS_COUNT_DEFAULT,

		.pd_mpu_state			= PD_OFF,
		.pd_mpu_ram_ret_state		= MEM_BANK_RET_ST_OFF,
		.pd_mpu_l1_ret_state		= MEM_BANK_RET_ST_OFF,
		.pd_mpu_l2_ret_state		= MEM_BANK_RET_ST_OFF,

		.wake_sources			= WAKE_ALL | MPU_WAKE,
	},
};

union state_data idle_data = {
	.deep_sleep = {
		.mosc_state			= MOSC_ON,
		.pd_mpu_state			= PD_ON,
		.pd_per_state			= PD_ON,
		.wake_sources			= MPU_WAKE,
	},
};

/* Clear out the global variables here */
void pm_init(void)
{
	cmd_global_data.cmd_id 	= CMD_ID_INVALID;
	cmd_global_data.data 	= NULL;

	powerdomain_reset();
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

void core_ldo_power_down(void)
{
	unsigned int core_ldo;

	/* Configure RETMODE_ENABLE for CORE LDO */
	if (soc_id == AM335X_SOC_ID && soc_rev > AM335X_REV_ES1_0) {
		core_ldo = __raw_readl(AM335X_PRM_LDO_SRAM_CORE_CTRL);
		core_ldo |= RETMODE_ENABLE;
		__raw_writel(core_ldo, AM335X_PRM_LDO_SRAM_CORE_CTRL);

		/* Poll for LDO Status to be in retention (SRAMLDO_STATUS) */
		while (!(__raw_readl(AM335X_PRM_LDO_SRAM_CORE_CTRL) & SRAMLDO_STATUS));
	} else if (soc_id == AM43XX_SOC_ID) {
		/* TODO */
		/* Note: Need to additionally check for Voltage Monitoring here */
	}
}

void core_ldo_power_up(void)
{
	unsigned int core_ldo;

	/* Disable RETMODE for CORE LDO */
	if (soc_id == AM335X_SOC_ID && soc_rev > AM335X_REV_ES1_0) {
		core_ldo = __raw_readl(AM335X_PRM_LDO_SRAM_CORE_CTRL);
		core_ldo &= ~RETMODE_ENABLE;
		__raw_writel(core_ldo, AM335X_PRM_LDO_SRAM_CORE_CTRL);

		/* Poll for LDO status to be out of retention (SRAMLDO_STATUS) */
		while (__raw_readl(AM335X_PRM_LDO_SRAM_CORE_CTRL) & SRAMLDO_STATUS);
	} else if (soc_id == AM43XX_SOC_ID) {
		/* TODO */
		/* Note: Need to additionally check for Voltage Monitoring here */
	}
}

void sram_ldo_ret_mode(int state)
{
	int var = __raw_readl(AM335X_PRM_LDO_SRAM_MPU_CTRL);

	if (state == RETMODE_ENABLE)
		var |= RETMODE_ENABLE;
	else
		var &= ~RETMODE_ENABLE;

	__raw_writel(var, AM335X_PRM_LDO_SRAM_MPU_CTRL);
}

void ds_save(void)
{
	if (soc_id == AM335X_SOC_ID) {
		set_ddr_reset();

		ddr_io_suspend();

		vtt_low();

		vtp_disable();

		sram_ldo_ret_mode(RETMODE_ENABLE);

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

		sram_ldo_ret_mode(RETMODE_DISABLE);

		vtp_enable();

		/* XXX: Why is this required here for DDR3? */
		hwmod_state_change(HWMOD_ENABLE, HWMOD_EMIF);

		vtt_high();

		ddr_io_resume();

		clear_ddr_reset();
	}
}

int a8_i2c_sleep_handler(unsigned short i2c_sleep_offset)
{
	unsigned char *dmem = (unsigned char *) DMEM_BASE;
	int ret = 0;

	hwmod_state_change(HWMOD_ENABLE, HWMOD_I2C0);

	if (i2c_sleep_offset != 0xffff)
		ret = i2c_write(dmem + i2c_sleep_offset);

	hwmod_state_change(HWMOD_DISABLE, HWMOD_I2C0);

	return ret;
}

int a8_i2c_wake_handler(unsigned short i2c_wake_offset)
{
	unsigned char *dmem = (unsigned char *) DMEM_BASE;
	int ret = 0;

	hwmod_state_change(HWMOD_ENABLE, HWMOD_I2C0);

	if (i2c_wake_offset != 0xffff)
		ret = i2c_write(dmem + i2c_wake_offset);

	hwmod_state_change(HWMOD_DISABLE, HWMOD_I2C0);

	return ret;
}
