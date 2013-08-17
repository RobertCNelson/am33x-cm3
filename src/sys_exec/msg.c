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

extern struct rtc_data rtc_mode_data;
extern struct deep_sleep_data standby_data;
extern struct deep_sleep_data ds0_data;
extern struct deep_sleep_data ds0_data_hs;
extern struct deep_sleep_data ds1_data;
extern struct deep_sleep_data ds1_data_hs;
extern struct deep_sleep_data ds2_data;
extern struct deep_sleep_data idle_data;

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
		.handler = a8_lp_rtc_handler,
		.needs_trigger = 1,
	},
	[CMD_ID_RTC_FAST] = {
		.gp_data = &rtc_mode_data,
		.handler = a8_lp_rtc_fast_handler,
		.needs_trigger = 1,
	},
	[CMD_ID_DS0] = {
		.gp_data = &ds0_data,
		.hs_data = &ds0_data_hs,
		.handler = a8_lp_ds0_handler,
		.needs_trigger = 1,
	},
	[CMD_ID_DS0_V2] = {
		.gp_data = &ds0_data,
		.hs_data = &ds0_data_hs,
		.handler = a8_lp_ds0_handler,
		.needs_trigger = 1,
		.do_ddr = 1,
	},
	[CMD_ID_DS1] = {
		.gp_data = &ds1_data,
		.hs_data = &ds1_data_hs,
		.handler = a8_lp_ds1_handler,
		.needs_trigger = 1,
	},
	[CMD_ID_DS1_V2] = {
		.gp_data = &ds1_data,
		.hs_data = &ds1_data_hs,
		.handler = a8_lp_ds1_handler,
		.needs_trigger = 1,
		.do_ddr = 1,
	},
	[CMD_ID_DS2] = {
		.gp_data = &ds2_data,
		.handler = a8_lp_ds2_handler,
		.needs_trigger = 1,
	},
	[CMD_ID_DS2_V2] = {
		.gp_data = &ds2_data,
		.handler = a8_lp_ds2_handler,
		.needs_trigger = 1,
		.do_ddr = 1,
	},
	[CMD_ID_STANDALONE] = {
		.handler = a8_standalone_handler,
		.needs_trigger = 1,
	},
	[CMD_ID_STANDBY] = {
		.gp_data = &standby_data,
		.handler = a8_standby_handler,
		.needs_trigger = 1,
	},
	[CMD_ID_STANDBY_V2] = {
		.gp_data = &standby_data,
		.handler = a8_standby_handler,
		.needs_trigger = 1,
		.do_ddr = 1,
	},
	[CMD_ID_RESET] = {
		.handler = a8_reset_handler,
	},
	[CMD_ID_VERSION] = {
		.handler = a8_version_handler,
	},
	[CMD_ID_CPUIDLE] = {
		.gp_data = &idle_data,
		.handler = a8_cpuidle_handler,
		.fast_trigger = 1,
	},
};


/* Clear out the IPC regs */
void msg_init(void)
{
	/* TODO: Global data related to msg also? */
	a8_m3_data_r.reg1 = 0;
	a8_m3_data_r.reg2 = 0;
	a8_m3_data_r.reg3 = 0;
	a8_m3_data_r.reg4 = 0;
	a8_m3_data_r.reg5 = 0;
	a8_m3_data_r.reg6 = 0;
	a8_m3_data_r.reg7 = 0;
	a8_m3_data_r.reg8 = 0;
}

/* Read all the IPC registers in one-shot */
void msg_read_all(void)
{
	a8_m3_data_r.reg1 = __raw_readl(IPC_MSG_REG1);
	a8_m3_data_r.reg2 = __raw_readl(IPC_MSG_REG2);
	a8_m3_data_r.reg3 = __raw_readl(IPC_MSG_REG3);
	a8_m3_data_r.reg4 = __raw_readl(IPC_MSG_REG4);
	a8_m3_data_r.reg5 = __raw_readl(IPC_MSG_REG5);
	a8_m3_data_r.reg6 = __raw_readl(IPC_MSG_REG6);
	a8_m3_data_r.reg7 = __raw_readl(IPC_MSG_REG7);
	a8_m3_data_r.reg8 = __raw_readl(IPC_MSG_REG8);
}

/* Read one specific IPC register */
unsigned int msg_read(char reg)
{
	unsigned int ret = __raw_readl(IPC_MSG_REG1 + (0x4*reg));
	__raw_writel(ret, (int)(&a8_m3_data_r) + (0x4*reg));
	return ret;
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
 * return 1 on success, 0 on failure
 */
int msg_cmd_is_valid(void)
{
	if (cmd_global_data.cmd_id >= CMD_ID_COUNT ||
	    cmd_global_data.cmd_id <= CMD_ID_INVALID)
		return 0;

	return cmd_handlers[cmd_global_data.cmd_id].handler != NULL;
}

/* Read all the IPC regs and pass it along to the appropriate handler */
void msg_cmd_dispatcher(void)
{
	char use_default_val = 0;
	int id;

	msg_read_all();

	if ((a8_m3_data_r.reg3 == 0xffffffff) &&
		(a8_m3_data_r.reg4 == 0xffffffff))
		use_default_val = 1;

	a8_m3_ds_data.reg1 = a8_m3_data_r.reg3;
	a8_m3_ds_data.reg2 = a8_m3_data_r.reg4;

	cmd_global_data.i2c_sleep_offset = a8_m3_data_r.reg6 & 0xffff;
	cmd_global_data.i2c_wake_offset = a8_m3_data_r.reg6 >> 16;

	/* board specific data saved in global variables for now */
	mem_type = (a8_m3_data_r.reg5 & MEM_TYPE_MASK) >> MEM_TYPE_SHIFT;
	vtt_toggle = (a8_m3_data_r.reg5 & VTT_STAT_MASK) >> VTT_STAT_SHIFT;
	vtt_gpio_pin = (a8_m3_data_r.reg5 & VTT_GPIO_PIN_MASK) >>
				VTT_GPIO_PIN_SHIFT;

	id = cmd_global_data.cmd_id;
	if (use_default_val) {
		if (soc_type != SOC_TYPE_GP && cmd_handlers[id].hs_data)
			cmd_global_data.data = cmd_handlers[id].hs_data;
		else if (cmd_handlers[id].gp_data)
			cmd_global_data.data = cmd_handlers[id].gp_data;
	} else
		cmd_global_data.data = &a8_m3_ds_data;

	cmd_handlers[id].handler(&cmd_global_data);
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
 * returns 1 if trigger is needed
 * returns 0 if trigger is not needed (eg: checking the version)
 */
int msg_cmd_needs_trigger(void)
{
	return cmd_handlers[cmd_global_data.cmd_id].needs_trigger;
}

int msg_cmd_fast_trigger(void)
{
	return cmd_handlers[cmd_global_data.cmd_id].fast_trigger;
}
