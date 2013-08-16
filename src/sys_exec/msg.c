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
extern struct deep_sleep_data ds1_data;
extern struct deep_sleep_data ds2_data;
extern struct deep_sleep_data idle_data;

static void a8_version_handler(struct cmd_data *data, char use_default_value)
{
	m3_firmware_version();
}

static void a8_reset_handler(struct cmd_data *data, char use_default_value)
{
	init_m3_state_machine();
}

struct state_handler cmd_handlers[] = {
	/* RTC */
	[0x1] = {
		.gp_data = &rtc_mode_data,
		.handler = a8_lp_cmd1_handler,
		.needs_trigger = 1,
	},
	/* RTC_FAST */
	[0x2] = {
		.gp_data = &rtc_mode_data,
		.handler = a8_lp_cmd2_handler,
		.needs_trigger = 1,
	},
	/* DS0 */
	[0x3] = {
		.gp_data = &ds0_data,
		.handler = a8_lp_cmd3_handler,
		.needs_trigger = 1,
	},
	/* DS0_V2 */
	[0x4] = {
		.gp_data = &ds0_data,
		.handler = a8_lp_cmd3_handler,
		.needs_trigger = 1,
		.do_ddr = 1,
	},
	/* DS1 */
	[0x5] = {
		.gp_data = &ds1_data,
		.handler = a8_lp_cmd5_handler,
		.needs_trigger = 1,
	},
	/* DS1_V2 */
	[0x6] = {
		.gp_data = &ds1_data,
		.handler = a8_lp_cmd5_handler,
		.needs_trigger = 1,
		.do_ddr = 1,
	},
	/* DS2 */
	[0x7] = {
		.gp_data = &ds2_data,
		.handler = a8_lp_cmd7_handler,
		.needs_trigger = 1,
	},
	/* DS2_V2 */
	[0x8] = {
		.gp_data = &ds2_data,
		.handler = a8_lp_cmd7_handler,
		.needs_trigger = 1,
		.do_ddr = 1,
	},
	/* Standalone app */
	[0x9] = {
		.handler = a8_standalone_handler,
		.needs_trigger = 1,
	},
	/* Standby */
	[0xb] = {
		.gp_data = &standby_data,
		.handler = a8_standby_handler,
		.needs_trigger = 1,
	},
	/* Standby */
	[0xc] = {
		.gp_data = &standby_data,
		.handler = a8_standby_handler,
		.needs_trigger = 1,
		.do_ddr = 1,
	},
	/* Reset State Machine */
	[0xe] = {
		.handler = a8_reset_handler,
	},
	/* Version */
	[0xf] = {
		.handler = a8_version_handler,
	},
	/* Idle */
	[0x10] = {
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

	a8_m3_data_w.reg1 = 0;
	a8_m3_data_w.reg2 = 0;
	a8_m3_data_w.reg3 = 0;
	a8_m3_data_w.reg4 = 0;
	a8_m3_data_w.reg5 = 0;
	a8_m3_data_w.reg6 = 0;
	a8_m3_data_w.reg7 = 0;
	a8_m3_data_w.reg8 = 0;

	ipc_reg_r = 0;
	ipc_reg_w = 0;
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
void msg_read(char reg)
{
	ipc_reg_r = __raw_readl(IPC_MSG_REG1 + (0x4*reg));
	__raw_writel(ipc_reg_r, (int)(&a8_m3_data_r) + (0x4*reg));
}

/* Write all the IPC registers in one-shot */
void msg_write_all(void)
{
	__raw_writel(a8_m3_data_w.reg1, IPC_MSG_REG1);
	__raw_writel(a8_m3_data_w.reg2, IPC_MSG_REG2);
	__raw_writel(a8_m3_data_w.reg3, IPC_MSG_REG3);
	__raw_writel(a8_m3_data_w.reg4, IPC_MSG_REG4);
	__raw_writel(a8_m3_data_w.reg5, IPC_MSG_REG5);
	__raw_writel(a8_m3_data_w.reg6, IPC_MSG_REG6);
	__raw_writel(a8_m3_data_w.reg7, IPC_MSG_REG7);
	/* TODO: Has to the removed since this is reserved */
	__raw_writel(a8_m3_data_w.reg8, IPC_MSG_REG8);
}

/*
 * Write to one specific IPC register
 * Before calling this fn, make sure ipc_reg_w has the correct val
 * TODO: Should check for the reg no. as some are reserved?
 */
void msg_write(char reg)
{
	__raw_writel(ipc_reg_w, IPC_MSG_REG1 + (0x4*reg));
	__raw_writel(ipc_reg_w, (int)(&a8_m3_data_w) + (0x4*reg));
}

/*
 * Check if the cmd_id is valid or not
 * return 1 on success, 0 on failure
 */
int msg_cmd_is_valid(void)
{
	msg_read(STAT_ID_REG);

	/* Extract the CMD_ID field of 16 bits */
	cmd_id = ipc_reg_r & 0xffff;

	if (cmd_id >= ARRAY_SIZE(cmd_handlers) || cmd_id < 0)
		return 0;

	return cmd_handlers[cmd_id].handler != NULL;
}

/* Read all the IPC regs and pass it along to the appropriate handler */
void msg_cmd_dispatcher(void)
{
	char use_default_val = 0;

	msg_read_all();

	if ((a8_m3_data_r.reg3 == 0xffffffff) &&
		(a8_m3_data_r.reg4 == 0xffffffff))
		use_default_val = 1;

	a8_m3_ds_data.reg1 = a8_m3_data_r.reg3;
	a8_m3_ds_data.reg2 = a8_m3_data_r.reg4;

	cmd_global_data.cmd_id = cmd_id;
	cmd_global_data.i2c_sleep_offset = a8_m3_data_r.reg6 & 0xffff;
	cmd_global_data.i2c_wake_offset = a8_m3_data_r.reg6 >> 16;

	/* board specific data saved in global variables for now */
	mem_type = (a8_m3_data_r.reg5 & MEM_TYPE_MASK) >> MEM_TYPE_SHIFT;
	vtt_toggle = (a8_m3_data_r.reg5 & VTT_STAT_MASK) >> VTT_STAT_SHIFT;
	vtt_gpio_pin = (a8_m3_data_r.reg5 & VTT_GPIO_PIN_MASK) >>
				VTT_GPIO_PIN_SHIFT;

	if (use_default_val) {
		if (cmd_handlers[cmd_id].gp_data)
			cmd_global_data.data = cmd_handlers[cmd_id].gp_data;
	} else
		cmd_global_data.data = &a8_m3_ds_data;

	cmd_handlers[cmd_id].handler(&cmd_global_data, use_default_val);
}

void m3_firmware_version(void)
{
	msg_read(PARAM1_REG);
	ipc_reg_r &= 0xffff0000;
	ipc_reg_w = ipc_reg_r | CM3_VERSION;
	msg_write(PARAM1_REG);
}

void msg_cmd_stat_update(int cmd_stat_value)
{
	msg_read(STAT_ID_REG);
	ipc_reg_r &= 0x0000ffff;
	ipc_reg_w = ipc_reg_r | (cmd_stat_value << 16);
	msg_write(STAT_ID_REG);
}

void msg_cmd_wakeup_reason_update(int wakeup_source)
{
	msg_read(TRACE_REG);
	ipc_reg_r &= 0xffffff00;
	ipc_reg_w = ipc_reg_r | wakeup_source;
	msg_write(TRACE_REG);
}

/*
 * Check whether command needs a trigger or not
 * returns 1 if trigger is needed
 * returns 0 if trigger is not needed (eg: checking the version)
 */
int msg_cmd_needs_trigger(void)
{
	msg_read(STAT_ID_REG);

	return cmd_handlers[cmd_id].needs_trigger;
}

int msg_cmd_fast_trigger(void)
{
	return cmd_handlers[cmd_global_data.cmd_id].fast_trigger;
}
