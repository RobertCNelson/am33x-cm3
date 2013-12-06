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

#ifndef __MSG_H__
#define __MSG_H__

#include <stddef.h>

#define CM3_VERSION		0x183

/*
 * 9-4 = VTT GPIO PIN (6 Bits)
 *   3 = VTT Status (1 Bit)
 * 2-0 = Memory Type (2 Bits)
*/
#define MEM_TYPE_SHIFT		(0x0)
#define MEM_TYPE_MASK		(0x7 << 0)
#define VTT_STAT_SHIFT		(0x3)
#define VTT_STAT_MASK		(0x1 << 3)
#define VTT_GPIO_PIN_SHIFT	(0x4)
#define VTT_GPIO_PIN_MASK	(0x3f << 4)

/* Memory type passed in IPC register */
#define MEM_TYPE_DDR2		2
#define MEM_TYPE_DDR3		3
#define MEM_TYPE_LPDDR2		4

#define RESUME_REG		0x0
#define STAT_ID_REG		0x1
#define PARAM1_REG		0x2
#define PARAM2_REG		0x3
#define PARAM3_REG		0x4
#define PARAM4_REG		0x5
#define TRACE_REG		0x6
#define CUST_REG		0x7

#define DS_IPC_DEFAULT		0xffffffff

#define CMD_STAT_PASS		0x0
#define CMD_STAT_FAIL		0x1
#define CMD_STAT_WAIT4OK	0x2


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
	CMD_ID_CPUIDLE_V2	= 0xd,
	CMD_ID_RESET		= 0xe,
	CMD_ID_VERSION		= 0xf,
	CMD_ID_CPUIDLE		= 0x10,
	CMD_ID_COUNT,
};

struct rtc_data {
	unsigned int rtc_timeout_val :4;	/* Delay for RTC alarm timeout. Default = 2secs */
};

struct deep_sleep_data {
	unsigned int mosc_state :1;		/* MOSC to be kept on (1) or off (0) */
	unsigned int deepsleep_count :16;	/* Count of how many OSC clocks needs to be seen \
						before exiting deep sleep mode */

	unsigned int vdd_mpu_val :15;		/* If vdd_mpu is to be lowered, vdd_mpu in mV */

	unsigned int pd_mpu_state :2;		/* Powerstate of PD_MPU */
	unsigned int pd_mpu_ram_ret_state :1;	/* Sabertooth RAM in retention state */
	unsigned int pd_mpu_l1_ret_state :1;	/* L1 memory in retention state */
	unsigned int pd_mpu_l2_ret_state :1;	/* L2 memory in retention state */
	unsigned int res1 :2;

	unsigned int pd_per_state :2;	 	/* Powerstate of PD_PER */
	unsigned int pd_per_icss_mem_ret_state :1; /* ICSS memory in retention state */
	unsigned int pd_per_mem_ret_state :1; 	/* Other memories in retention state */
	unsigned int pd_per_ocmc_ret_state :1; 	/* OCMC memory in retention state */
	unsigned int pd_per_ocmc2_ret_state :1;	/* OCMC bank 2 in retention state */
	unsigned int res2 :5;

	unsigned int wake_sources :13;		/* Wake sources */
						/* USB, I2C0, RTC_ALARM, TIMER1 \
						   UART0, GPIO0_WAKE0, GPIO0_WAKE1, \
						   WDT1, ADTSC, RTC_TIMER, USBWOUT0, \
						   MPU, USBWOUT1 */
	unsigned int reserved :1;		/* Internal use */
};

union state_data {
	struct deep_sleep_data deep_sleep;
	struct rtc_data rtc;
	struct {
		unsigned int param1;
		unsigned int param2;
	} raw;
};

struct cmd_data {
	enum cmd_ids cmd_id;
	union state_data *data;
	unsigned short i2c_sleep_offset;
	unsigned short i2c_wake_offset;
};

struct state_handler {
	union state_data *gp_data;
	union state_data *hs_data;
	void (*cmd_handler)(struct cmd_data *data);
	void (*wake_handler)(void);
	bool needs_trigger;
	bool fast_trigger;
	bool do_ddr;
};

extern struct cmd_data cmd_global_data;
extern struct state_handler cmd_handlers[];

/* Board specifics populated in IPC_REG4 */
extern int mem_type;		/* Memory Type 2 = DDR2, 3 = DDR3, 4 = LPDDR2 */
extern bool vtt_toggle;		/* VTT Toggle  true = required */
extern int vtt_gpio_pin;	/* VTT GPIO Pin */

void m3_firmware_version(void);
void m3_param_reset(void);

unsigned int msg_read(char);
void msg_write(unsigned int, char);

void msg_cmd_read_id(void);
bool msg_cmd_is_valid(void);
bool msg_cmd_needs_trigger(void);
bool msg_cmd_fast_trigger(void);
void msg_cmd_dispatcher(void);
void msg_cmd_stat_update(int);
void msg_cmd_wakeup_reason_update(int);

#endif
