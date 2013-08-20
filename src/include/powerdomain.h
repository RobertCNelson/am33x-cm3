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
	unsigned int	ram_retst_mask;
	unsigned int	ram_retst_shift;
	unsigned int	l2_retst_mask;
	unsigned int	l2_retst_shift;
	unsigned int	l1_retst_mask;
	unsigned int	l1_retst_shift;
	unsigned int	lpstchg_mask;
	unsigned int	lpstchg_shift;
	unsigned int	logicretst_mask;
	unsigned int	logicretst_shift;
	unsigned int	pwrst_mask;
	unsigned int	pwrst_shift;
};

struct pd_per_bits {
	unsigned int	per_retst_mask;
	unsigned int	per_retst_shift;
	unsigned int	ram1_retst_mask;
	unsigned int	ram1_retst_shift;
	unsigned int	ram2_retst_mask;
	unsigned int	ram2_retst_shift;
	unsigned int	icss_retst_mask;
	unsigned int	icss_retst_shift;
	unsigned int	lpstchg_mask;
	unsigned int	lpstchg_shift;
	unsigned int	logicretst_mask;
	unsigned int	logicretst_shift;
	unsigned int	pwrst_mask;
	unsigned int	pwrst_shift;
};

void powerdomain_reset(void);
void powerdomain_init(void);

unsigned int pd_state_change(unsigned int, int);
void pd_state_restore(int);

unsigned int get_pd_per_stctrl_val(struct deep_sleep_data *data);
unsigned int get_pd_mpu_stctrl_val(struct deep_sleep_data *data);

int verify_pd_transitions(void);

#endif

