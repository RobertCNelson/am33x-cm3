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

#ifndef __POWERDOMAIN_H__
#define __POWERDOMAIN_H__

#define PD_MPU  0x1
#define PD_PER  0x2

#define PD_ON                   0x3
#define PD_RET                  0x1
#define PD_OFF                  0x0

#define MEM_BANK_RET_ST_RET     0x1
#define MEM_BANK_RET_ST_OFF     0x0

#define MEM_BANK_ON_ST_ON       0x3
#define MEM_BANK_ON_ST_RET      0x1
#define MEM_BANK_ON_ST_OFF      0x0

struct deep_sleep_data;

struct pd_mpu_bits {
	int	ram_retst_mask;
	int	ram_retst_shift;
	int	l2_retst_mask;
	int	l2_retst_shift;
	int	l1_retst_mask;
	int	l1_retst_shift;
	int	lpstchg_mask;
	int	lpstchg_shift;
	int	logicretst_mask;
	int	logicretst_shift;
	int	pwrst_mask;
	int	pwrst_shift;
};

struct pd_per_bits {
	int	per_retst_mask;
	int	per_retst_shift;
	int	ram1_retst_mask;
	int	ram1_retst_shift;
	int	ram2_retst_mask;
	int	ram2_retst_shift;
	int	icss_retst_mask;
	int	icss_retst_shift;
	int	lpstchg_mask;
	int	lpstchg_shift;
	int	logicretst_mask;
	int	logicretst_shift;
	int	pwrst_mask;
	int	pwrst_shift;
};

void powerdomain_reset(void);
void powerdomain_init(void);

int pd_state_change(int, int);
void pd_state_restore(int);

int mpu_ram_ret_state_change(int, int);
int mpu_l1_ret_state_change(int, int);
int mpu_l2_ret_state_change(int, int);
int icss_mem_ret_state_change(int, int);
int per_mem_ret_state_change(int, int);
int ocmc_mem_ret_state_change(int, int);

int per_powerst_change(int, int);
int mpu_powerst_change(int, int);

int get_pd_per_stctrl_val(struct deep_sleep_data *data);
int get_pd_mpu_stctrl_val(struct deep_sleep_data *data);

int verify_pd_transitions(void);

#endif

