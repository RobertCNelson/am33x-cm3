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

#ifndef __PM_STATE_DATA_H__
#define __PM_STATE_DATA_H__

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

extern union state_data rtc_mode_data;
extern union state_data standby_data;
extern union state_data ds0_data;
extern union state_data ds0_data_hs;
extern union state_data ds1_data;
extern union state_data ds1_data_hs;
extern union state_data ds2_data;
extern union state_data idle_data;

#endif
