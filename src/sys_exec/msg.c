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
#include <device_am335x.h>
#include <low_power.h>
#include <system_am335.h>
#include <pm_state_data.h>

static union state_data custom_state_data;

static void a8_version_handler(struct cmd_data *data)
{
	m3_firmware_version();
}

static void a8_reset_handler(struct cmd_data *data)
{
	init_m3_state_machine();
}

struct state_handler cmd_handlers[] = {
	[CMD_ID_RTC] = {
		.gp_data = &rtc_mode_data,
		.cmd_handler = a8_lp_rtc_handler,
		.wake_handler = a8_wake_rtc_handler,
		.needs_trigger = true,
	},
	[CMD_ID_RTC_FAST] = {
		.gp_data = &rtc_mode_data,
		.cmd_handler = a8_lp_rtc_handler,
		.wake_handler = a8_wake_rtc_handler,
		.needs_trigger = true,
	},
	[CMD_ID_DS0] = {
		.gp_data = &ds0_data,
		.hs_data = &ds0_data_hs,
		.cmd_handler = a8_lp_ds0_handler,
		.wake_handler = a8_wake_ds0_handler,
		.needs_trigger = true,
	},
	[CMD_ID_DS0_V2] = {
		.gp_data = &ds0_data,
		.hs_data = &ds0_data_hs,
		.cmd_handler = a8_lp_ds0_handler,
		.wake_handler = a8_wake_ds0_handler,
		.needs_trigger = true,
		.do_ddr = true,
	},
	[CMD_ID_DS1] = {
		.gp_data = &ds1_data,
		.hs_data = &ds1_data_hs,
		.cmd_handler = a8_lp_ds1_handler,
		.wake_handler = a8_wake_ds1_handler,
		.needs_trigger = true,
	},
	[CMD_ID_DS1_V2] = {
		.gp_data = &ds1_data,
		.hs_data = &ds1_data_hs,
		.cmd_handler = a8_lp_ds1_handler,
		.wake_handler = a8_wake_ds1_handler,
		.needs_trigger = true,
		.do_ddr = true,
	},
	[CMD_ID_DS2] = {
		.gp_data = &ds2_data,
		.cmd_handler = a8_lp_ds2_handler,
		.wake_handler = a8_wake_ds2_handler,
		.needs_trigger = true,
	},
	[CMD_ID_DS2_V2] = {
		.gp_data = &ds2_data,
		.cmd_handler = a8_lp_ds2_handler,
		.wake_handler = a8_wake_ds2_handler,
		.needs_trigger = true,
		.do_ddr = true,
	},
	[CMD_ID_STANDALONE] = {
		.cmd_handler = a8_standalone_handler,
		.needs_trigger = true,
	},
	[CMD_ID_STANDBY] = {
		.gp_data = &standby_data,
		.cmd_handler = a8_standby_handler,
		.wake_handler = a8_wake_standby_handler,
		.needs_trigger = true,
	},
	[CMD_ID_STANDBY_V2] = {
		.gp_data = &standby_data,
		.cmd_handler = a8_standby_handler,
		.wake_handler = a8_wake_standby_handler,
		.needs_trigger = true,
		.do_ddr = true,
	},
	[CMD_ID_RESET] = {
		.cmd_handler = a8_reset_handler,
	},
	[CMD_ID_VERSION] = {
		.cmd_handler = a8_version_handler,
	},
	[CMD_ID_CPUIDLE] = {
		.gp_data = &idle_data,
		.cmd_handler = a8_cpuidle_handler,
		.wake_handler = a8_wake_cpuidle_handler,
		.fast_trigger = true,
	},
};

/* Read one specific IPC register */
unsigned int msg_read(char reg)
{
	return __raw_readl(IPC_MSG_REG1 + (0x4*reg));
}

/*
 * Write to one specific IPC register
 * TODO: Should check for the reg no. as some are reserved?
 */
void msg_write(unsigned int value, char reg)
{
	__raw_writel(value, IPC_MSG_REG1 + (0x4*reg));
}

void msg_cmd_read_id(void)
{
	/* Extract the CMD_ID field of 16 bits */
	cmd_global_data.cmd_id = msg_read(STAT_ID_REG) & 0xffff;
}

/*
 * Check if the cmd_id is valid or not
 * return true on success, false on failure
 */
bool msg_cmd_is_valid(void)
{
	if (cmd_global_data.cmd_id >= CMD_ID_COUNT ||
	    cmd_global_data.cmd_id <= CMD_ID_INVALID)
		return false;

	return cmd_handlers[cmd_global_data.cmd_id].cmd_handler != NULL;
}

/* Read all the IPC regs and pass it along to the appropriate handler */
void msg_cmd_dispatcher(void)
{
	int id;
	unsigned int param1;
	unsigned int param2;
	unsigned int param3;
	unsigned int param4;

	param4 = msg_read(PARAM4_REG);
	cmd_global_data.i2c_sleep_offset = param4 & 0xffff;
	cmd_global_data.i2c_wake_offset = param4 >> 16;

	/* board specific data saved in global variables for now */
	param3 = msg_read(PARAM3_REG);
	mem_type = (param3 & MEM_TYPE_MASK) >> MEM_TYPE_SHIFT;
	vtt_toggle = (param3 & VTT_STAT_MASK) >> VTT_STAT_SHIFT;
	vtt_gpio_pin = (param3 & VTT_GPIO_PIN_MASK) >> VTT_GPIO_PIN_SHIFT;

	param1 = msg_read(PARAM1_REG);
	param2 = msg_read(PARAM2_REG);
	id = cmd_global_data.cmd_id;

	if (param1 == DS_IPC_DEFAULT && param2 == DS_IPC_DEFAULT) {
		if (soc_type != SOC_TYPE_GP && cmd_handlers[id].hs_data)
			cmd_global_data.data = cmd_handlers[id].hs_data;
		else if (cmd_handlers[id].gp_data)
			cmd_global_data.data = cmd_handlers[id].gp_data;
	} else {
		custom_state_data.raw.param1 = param1;
		custom_state_data.raw.param2 = param2;
		cmd_global_data.data = &custom_state_data;
	}

	cmd_handlers[id].cmd_handler(&cmd_global_data);
}

void m3_firmware_version(void)
{
	unsigned int value;

	value = msg_read(PARAM1_REG);
	value &= 0xffff0000;
	value |= CM3_VERSION;
	msg_write(value, PARAM1_REG);
}

void msg_cmd_stat_update(int cmd_stat_value)
{
	unsigned int value;

	value = msg_read(STAT_ID_REG);
	value &= 0x0000ffff;
	value |= cmd_stat_value << 16;
	msg_write(value, STAT_ID_REG);
}

void msg_cmd_wakeup_reason_update(int wakeup_source)
{
	unsigned int value;

	value = msg_read(TRACE_REG);
	value &= 0xffffff00;
	value |= wakeup_source;
	msg_write(value, TRACE_REG);
}

/*
 * Check whether command needs a trigger or not
 * returns true if trigger is needed
 * returns false if trigger is not needed (eg: checking the version)
 */
bool msg_cmd_needs_trigger(void)
{
	return cmd_handlers[cmd_global_data.cmd_id].needs_trigger;
}

bool msg_cmd_fast_trigger(void)
{
	return cmd_handlers[cmd_global_data.cmd_id].fast_trigger;
}
