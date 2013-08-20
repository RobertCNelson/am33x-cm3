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

#include <cm3.h>
#include <device_am335x.h>
#include <low_power.h>
#include <prmam335x.h>
#include <system_am335.h>
#include <prcm.h>

/* Enter RTC mode */
void a8_lp_rtc_handler(struct cmd_data *data)
{
	struct rtc_data *local_cmd = (struct rtc_data *)data->data;
	int timeout = 0;

	a8_i2c_sleep_handler(data->i2c_sleep_offset);

	/* If RTC module if not already configured... cannot continue */
	rtc_enable_check();

	if (local_cmd->rtc_timeout_val != 0 &&
			local_cmd->rtc_timeout_val <= RTC_TIMEOUT_MAX)
		timeout = local_cmd->rtc_timeout_val;
	else
		timeout = RTC_TIMEOUT_DEFAULT;

	/* Program the RTC_PMIC register for deasseting pmic_pwr_enable */
	rtc_reg_write(RTC_PMIC_REG, 0x00010000);

	timeout += rtc_reg_read(RTC_ALARM2_SECONDS_REG);

	rtc_reg_write(RTC_ALARM2_SECONDS_REG, timeout);

	/* Turn off interconnect */
	interconnect_modules_disable();

	/* Disable the clock domains except MPU */
	clkdm_sleep();

	/* Disable MPU clock domain */
	mpu_clkdm_sleep();

	wkup_clkdm_sleep();

	/* TODO: wait for power domain state change interrupt from PRCM */
}

/* Enter RTC_fast mode */
void a8_lp_rtc_fast_handler(struct cmd_data *data)
{
	struct rtc_data *rtc_data = (struct rtc_data *)data->data;
	int timeout = 0;

	a8_i2c_sleep_handler(data->i2c_sleep_offset);

	if (!rtc_data->rtc_timeout_val &&
		(rtc_data->rtc_timeout_val < RTC_TIMEOUT_MAX))
		timeout = rtc_data->rtc_timeout_val;
	else
		timeout = RTC_TIMEOUT_DEFAULT;

	/* If RTC module if not already configured... cannot continue */
	rtc_enable_check();

	/* Program the RTC_PMIC register for deasseting pmic_pwr_enable */
	__raw_writel(0x00010000, RTC_PMIC_REG);

	/* Read the RTC_ALARM2_SECS */
	timeout += __raw_readl(RTC_ALARM2_SECONDS_REG);

	/* Write val + timeout to RTC_ALARM */
	__raw_writel(timeout, RTC_ALARM2_SECONDS_REG);
}

/*
 * Enter DeepSleep0 mode
 * MOSC = OFF
 * PD_PER = RET
 * PD_MPU = RET
 */
void a8_lp_ds0_handler(struct cmd_data *data)
{
	struct deep_sleep_data *local_cmd = (struct deep_sleep_data *)data->data;

	int per_st = 0;
	int mpu_st = 0;
	int temp;

	if (cmd_handlers[cmd_global_data.cmd_id].do_ddr)
		ds_save();

	a8_i2c_sleep_handler(data->i2c_sleep_offset);

	configure_wake_sources(local_cmd->wake_sources);

	/* TODO: Check for valid range */
	if (local_cmd->deepsleep_count)
		configure_deepsleep_count(local_cmd->deepsleep_count);
	else
		configure_deepsleep_count(DS_COUNT_DEFAULT);

	per_st = get_pd_per_stctrl_val(local_cmd);
	mpu_st = get_pd_mpu_stctrl_val(local_cmd);

	/* VDD MPU lowering does not make sense here so ignore that field */

	/* MPU power domain state change */
	pd_state_change(mpu_st, PD_MPU);

	/* PER power domain state change */
	pd_state_change(per_st, PD_PER);

	/* XXX: New addition to resolve any issues that A8 might have */
	essential_modules_disable();

	interconnect_modules_disable();

	/* DPLL retention update for PG 2.0 */
	am33xx_power_down_plls();

	mpu_clkdm_sleep();

	clkdm_sleep();

	/* Disable MOSC if defaults are required or if user asked for it */
	if (local_cmd->mosc_state == MOSC_OFF) {
		disable_master_oscillator();
		/* Core LDO retention for PG 2.0 if PD_PER is in RET */
		if (soc_id == AM335X_SOC_ID && soc_rev > AM335X_REV_ES1_0) {
			if ((__raw_readl(AM335X_PM_PER_PWRSTST) &
				PWR_STATE_STS_MASK) == POWER_STATE_STS_RET) {
				/* set Auto_RAMP_EN in SMA2 Spare Register (SMA2). */
				temp = __raw_readl(SMA2_SPARE_REG);
				temp |= VSLDO_CORE_AUTO_RAMP_EN;
				__raw_writel(temp, SMA2_SPARE_REG);
				core_ldo_power_down();
			}
		}
	}

	/* TODO: wait for power domain state change interrupt from PRCM */
	wkup_clkdm_sleep();
}

/*
 * Enter DeepSleep1 mode
 * MOSC = OFF
 * PD_PER = ON
 * PD_MPU = RET
 */
void a8_lp_ds1_handler(struct cmd_data *data)
{
	struct deep_sleep_data *local_cmd = (struct deep_sleep_data *)data->data;

	int per_st = 0;
	int mpu_st = 0;

	if (cmd_handlers[cmd_global_data.cmd_id].do_ddr)
		ds_save();

	a8_i2c_sleep_handler(data->i2c_sleep_offset);

	/* Disable MOSC if possible */
	if (local_cmd->mosc_state == MOSC_OFF)
		disable_master_oscillator();

	configure_wake_sources(local_cmd->wake_sources);

	/* TODO: Check for valid range */
	if (local_cmd->deepsleep_count)
		configure_deepsleep_count(local_cmd->deepsleep_count);
	else
		configure_deepsleep_count(DS_COUNT_DEFAULT);

	per_st = get_pd_per_stctrl_val(local_cmd);
	mpu_st = get_pd_mpu_stctrl_val(local_cmd);

	/* MPU power domain state change */
	pd_state_change(mpu_st, PD_MPU);

	/* PER power domain state change */
	pd_state_change(per_st, PD_PER);

	/* DPLL retention update for PG 2.0 */
	am33xx_power_down_plls();

	mpu_clkdm_sleep();

	wkup_clkdm_sleep();

	/* TODO: wait for power domain state change interrupt from PRCM */
}

/*
 * Enter DeepSleep2 mode
 * MOSC = OFF
 * PD_PER = ON
 * PD_MPU = ON
 */
void a8_lp_ds2_handler(struct cmd_data *data)
{
	struct deep_sleep_data *local_cmd = (struct deep_sleep_data *)data->data;

	int per_st = 0;
	int mpu_st = 0;

	if (cmd_handlers[cmd_global_data.cmd_id].do_ddr)
		ds_save();

	a8_i2c_sleep_handler(data->i2c_sleep_offset);

	/* Disable MOSC if possible */
	if (local_cmd->mosc_state == MOSC_OFF)
		disable_master_oscillator();

	configure_wake_sources(local_cmd->wake_sources);

	/* TODO: Check for valid range */
	if (local_cmd->deepsleep_count)
		configure_deepsleep_count(local_cmd->deepsleep_count);
	else
		configure_deepsleep_count(DS_COUNT_DEFAULT);

	per_st = get_pd_per_stctrl_val(local_cmd);
	mpu_st = get_pd_mpu_stctrl_val(local_cmd);

	/* MPU power domain state change */
	pd_state_change(mpu_st, PD_MPU);

	/* PER power domain state change */
	pd_state_change(per_st, PD_PER);

	/* DPLL retention update for PG 2.0 */
	am33xx_power_down_plls();

	wkup_clkdm_sleep();

	/*TODO: wait for power domain state change interrupt from PRCM */
}

void a8_standby_handler(struct cmd_data *data)
{
	struct deep_sleep_data *local_cmd =
		(struct deep_sleep_data *)data->data;
	int mpu_st = 0;

	if (cmd_handlers[cmd_global_data.cmd_id].do_ddr)
		ds_save();

	a8_i2c_sleep_handler(data->i2c_sleep_offset);

	configure_wake_sources(local_cmd->wake_sources);

	/* TODO: Check for valid range */
	if (local_cmd->deepsleep_count)
		configure_deepsleep_count(local_cmd->deepsleep_count);
	else
		configure_deepsleep_count(DS_COUNT_DEFAULT);

	mpu_st = get_pd_mpu_stctrl_val(local_cmd);

	/* MPU power domain state change */
	pd_state_change(mpu_st, PD_MPU);

	mpu_clkdm_sleep();
}

void a8_cpuidle_handler(struct cmd_data *data)
{
	struct deep_sleep_data *local_cmd = (struct deep_sleep_data *)data->data;

	configure_wake_sources(local_cmd->wake_sources);

	mpu_clkdm_sleep();
}

/* Standalone application handler */
void a8_standalone_handler(struct cmd_data *data)
{
	/* TBD */
}

/* All wake interrupts invoke this function */
void generic_wake_handler(int wakeup_reason)
{
	int i = 0;

	if (halt_on_resume)
		while(1);

	/*
	 * Assuming that cmd_id is a valid reflection of what we did
	 */
	switch (cmd_global_data.cmd_id) {
	case CMD_ID_RTC:
		a8_wake_rtc_handler();
		break;
	case CMD_ID_RTC_FAST:
		a8_wake_rtc_fast_handler();
		break;
	case CMD_ID_DS0:
		a8_wake_ds0_handler();
		break;
	case CMD_ID_DS1:
		a8_wake_ds1_handler();
		break;
	case CMD_ID_DS2:
		a8_wake_ds2_handler();
		break;
	case CMD_ID_STANDBY:
		a8_wake_standby_handler();
		break;
	case CMD_ID_CPUIDLE:
		a8_wake_cpuidle_handler();
		break;
	default:
		while(1)
		;
	}

	msg_cmd_wakeup_reason_update(wakeup_reason);

	enable_master_oscillator();

	if (cmd_handlers[cmd_global_data.cmd_id].do_ddr)
		ds_restore();

	/*
	 * PSP kernels have a long standing bug in sleep33xx.S,
	 * they don't re-enable the EMIF hwmod in their resume
	 * path. Keep compatibily with these kernels.
	 */
	if (!cmd_handlers[cmd_global_data.cmd_id].do_ddr)
		module_state_change(MODULE_ENABLE, AM335X_CM_PER_EMIF_CLKCTRL);

	/* If everything is done, we init things again */
	/* Flush out NVIC interrupts */
	for (i=0; i<AM335X_NUM_EXT_INTERRUPTS; i++) {
		nvic_disable_irq(i);
		nvic_clear_irq(i);
	}

	msg_init();

	trace_init();

	pm_init();

	a8_i2c_wake_handler(cmd_global_data.i2c_wake_offset);

	/* Enable only the MBX IRQ */
	nvic_enable_irq(AM335X_IRQ_MBINT0);
	nvic_enable_irq(53);

	/* Enable MPU only after we are sure that we are done with the wakeup */
	mpu_enable();
}

/* Exit RTC mode */
void a8_wake_rtc_handler(void)
{
	/* RTC wake is a cold boot... so this doesn't make sense */
}

/* Exit RTC_fast mode */
void a8_wake_rtc_fast_handler(void)
{
	/* RTC fast wake is also similar to cold boot... */
}

/*
 * Exit DeepSleep0 mode
 * MOSC = OFF
 * PD_PER = RET
 * PD_MPU = RET
 */
void a8_wake_ds0_handler(void)
{
	int result = 0;

	if (soc_id == AM335X_SOC_ID && soc_rev > AM335X_REV_ES1_0)
		core_ldo_power_up();

	result = verify_pd_transitions();

	pd_state_restore(PD_PER);
	pd_state_restore(PD_MPU);

	wkup_clkdm_wake();

	clkdm_wake();

	am33xx_power_up_plls();

	interconnect_modules_enable();

	essential_modules_enable();

	msg_cmd_stat_update(result);

	clear_wake_sources();

	mpu_clkdm_wake();
}

/*
 * Exit DeepSleep1 mode
 * MOSC = OFF
 * PD_PER = ON
 * PD_MPU = RET
 */
void a8_wake_ds1_handler(void)
{
	int result = 0;

	result = verify_pd_transitions();

	pd_state_restore(PD_PER);
	pd_state_restore(PD_MPU);

	wkup_clkdm_wake();

	am33xx_power_up_plls();

	essential_modules_enable();

	msg_cmd_stat_update(result);

	clear_wake_sources();

	mpu_clkdm_wake();
}

/*
 * Exit DeepSleep2 mode
 * MOSC = OFF
 * PD_PER = ON
 * PD_MPU = ON
 */
void a8_wake_ds2_handler(void)
{
	int result = 0;

	result = verify_pd_transitions();

	pd_state_restore(PD_PER);
	pd_state_restore(PD_MPU);

	wkup_clkdm_wake();

	am33xx_power_up_plls();

	msg_cmd_stat_update(result);

	/* Interrupt MPU now */
	__asm("sev");

	clear_wake_sources();
}

/* Exit Standby mode
 * MOSC = ON
 * PD_PER = ON
 * PD_MPU = OFF
 */
void a8_wake_standby_handler(void)
{
	int result = 0;

	result = verify_pd_transitions();

	pd_state_restore(PD_MPU);

	essential_modules_enable();

	msg_cmd_stat_update(result);

	clear_wake_sources();

	mpu_clkdm_wake();
}

/* Exit cpuidle
 * MPU_MPU_CLKCTRL = OFF
 */
void a8_wake_cpuidle_handler(void)
{
	clear_wake_sources();

	mpu_clkdm_wake();

	return;
}
