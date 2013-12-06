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

#include <powerdomain.h>
#include <prcm_core.h>
#include <msg.h>
#include <pm_state_data.h>

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
		.pd_per_ocmc2_ret_state		= MEM_BANK_RET_ST_OFF,

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
		.pd_per_state 			= PD_ON,
		.wake_sources			= MPU_WAKE,
	},
};

union state_data idle_v2_data = {
	.deep_sleep = {
		.mosc_state			= MOSC_ON,

		.pd_mpu_state			= PD_OFF,
		.pd_mpu_ram_ret_state		= MEM_BANK_RET_ST_OFF,
		.pd_mpu_l1_ret_state		= MEM_BANK_RET_ST_OFF,
		.pd_mpu_l2_ret_state		= MEM_BANK_RET_ST_OFF,

		.pd_per_state 			= PD_ON,

		.wake_sources			= MPU_WAKE,
	},
};
