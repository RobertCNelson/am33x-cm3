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

#ifndef __SYSTEM_AM335_H__
#define __SYSTEM_AM335_H__

#include <stdint.h>
#include <stddef.h>
#include <low_power.h>

enum cmd_ids {
	CMD_ID_INVALID		= 0x0,
	CMD_ID_RTC		= 0x1,
	CMD_ID_RTC_FAST		= 0x2,
	CMD_ID_DS0		= 0x3,
	CMD_ID_DS0_V2		= 0x4,
	CMD_ID_DS1		= 0x5,
	CMD_ID_DS1_V2		= 0x6,
	CMD_ID_DS2		= 0x7,
	CMD_ID_DS2_V2		= 0x8,
	CMD_ID_STANDALONE	= 0x9,
	CMD_ID_STANDBY		= 0xb,
	CMD_ID_STANDBY_V2	= 0xc,
	CMD_ID_RESET		= 0xe,
	CMD_ID_VERSION		= 0xf,
	CMD_ID_CPUIDLE		= 0x10,
	CMD_ID_COUNT,
};

struct cmd_data {
	enum cmd_ids cmd_id;
	union state_data *data;
	unsigned short i2c_sleep_offset;
	unsigned short i2c_wake_offset;
};

struct cmd_data cmd_global_data;

struct state_handler {
	union state_data *gp_data;
	union state_data *hs_data;
	void (*cmd_handler)(struct cmd_data *data);
	void (*wake_handler)(void);
	bool needs_trigger;
	bool fast_trigger;
	bool do_ddr;
};

extern struct state_handler cmd_handlers[];

/* Board specifics populated in IPC_REG4 */
int mem_type;			/* Memory Type 2 = DDR2, 3 = DDR3 */
bool vtt_toggle; 		/* VTT Toggle  true = required */
int vtt_gpio_pin; 		/* VTT GPIO Pin */

/* Debug info */
bool halt_on_resume;

int cmd_wake_sources;

unsigned int soc_id;
unsigned int soc_rev;
unsigned int soc_type;

/* Placeholder for storing PLL mode */
unsigned int clk_mode_per_val;
unsigned int clk_mode_disp_val;
unsigned int clk_mode_ddr_val;
unsigned int clk_mode_mpu_val;
unsigned int clk_mode_core_val;

/* PD_PER registers */
unsigned int am33xx_per[85];

unsigned int watermark;

void pm_init(void);

void system_init(void);
void system_core_clock_update(void);

unsigned int msg_read(char);
void msg_write(unsigned int, char);

void msg_cmd_read_id(void);
bool msg_cmd_is_valid(void);
bool msg_cmd_needs_trigger(void);
bool msg_cmd_fast_trigger(void);
void msg_cmd_dispatcher(void);
void msg_cmd_stat_update(int);
void msg_cmd_wakeup_reason_update(int);

void a8_notify(int);
void a8_m3_low_power_sync(int);
void a8_m3_low_power_fast(int);

void a8_lp_rtc_handler(struct cmd_data *);
void a8_lp_ds0_handler(struct cmd_data *);
void a8_lp_ds1_handler(struct cmd_data *);
void a8_lp_ds2_handler(struct cmd_data *);
void a8_standalone_handler(struct cmd_data *);
void a8_standby_handler(struct cmd_data *);
void a8_cpuidle_handler(struct cmd_data *);

void generic_wake_handler(int);
void a8_wake_rtc_handler(void);
void a8_wake_ds0_handler(void);
void a8_wake_ds1_handler(void);
void a8_wake_ds2_handler(void);
void a8_wake_standby_handler(void);
void a8_wake_cpuidle_handler(void);

void m3_firmware_version(void);
void init_m3_state_machine(void);

void trace_init(void);
void trace_update(void);
void trace_get_current_pos(void);
void trace_set_current_pos(void);

int rtc_enable_check(void);
unsigned int rtc_reg_read(int);
void rtc_reg_write(unsigned int, int);

int i2c_write(const unsigned char *);

void setup_soc(void);

int a8_i2c_sleep_handler(unsigned short);
int a8_i2c_wake_handler(unsigned short);

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

#define RESUME_REG	0x0
#define STAT_ID_REG	0x1
#define PARAM1_REG	0x2
#define PARAM2_REG	0x3
#define PARAM3_REG	0x4
#define PARAM4_REG	0x5
#define TRACE_REG	0x6
#define CUST_REG	0x7

#define DS_IPC_DEFAULT	0xffffffff

#define CMD_STAT_PASS		0x0
#define CMD_STAT_FAIL		0x1
#define CMD_STAT_WAIT4OK	0x2

#define SET_BIT(x)		(1<<x)
#define CLR_BIT(x)		(0<<x)

#define AM335X_SOC_ID		0xB944
#define AM335X_REV_ES1_0	0
#define AM335X_REV_ES2_0	1
#define AM335X_REV_ES2_1	2

#define AM43XX_SOC_ID		0xB98C

#define SOC_TYPE_TEST	0
#define SOC_TYPE_EMU	1
#define SOC_TYPE_HS	2
#define SOC_TYPE_GP	3

#endif
