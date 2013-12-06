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

#include <io.h>
#include <prcm_core.h>
#include <msg.h>
#include <pm_state_data.h>
#include <powerdomain.h>
#include <powerdomain_335x.h>
#include <powerdomain_43xx.h>

#define PD_STATE_MASK	0x3

struct powerdomain_state {
	unsigned int stctrl_next_val;
	unsigned int stctrl_prev_val;
	unsigned int pwrstst_prev_val;
};

static const struct pd_mpu_bits *mpu_bits;
static const struct pd_per_bits *per_bits;
static const struct powerdomain_regs *pd_regs;

static struct powerdomain_state pd_states[] = {
	[PD_MPU] = {},
	[PD_PER] = {},
};


/* Clear out the global variables here */
void powerdomain_reset(void)
{
	pd_states[PD_MPU].stctrl_next_val = 0;
	pd_states[PD_MPU].stctrl_prev_val = 0;
	pd_states[PD_MPU].pwrstst_prev_val = 0;
	pd_states[PD_PER].stctrl_next_val = 0;
	pd_states[PD_PER].stctrl_prev_val = 0;
	pd_states[PD_PER].pwrstst_prev_val = 0;
}

void powerdomain_init(void)
{
	if (soc_id == AM335X_SOC_ID) {
		mpu_bits = &am335x_mpu_bits;
		per_bits = &am335x_per_bits;
		pd_regs = am335x_pd_regs;
	} else if (soc_id == AM43XX_SOC_ID) {
		mpu_bits = &am43xx_mpu_bits;
		per_bits = &am43xx_per_bits;
		pd_regs = am43xx_pd_regs;
	}
}

unsigned int pd_state_change(unsigned int val, enum powerdomain_id pd)
{
	pd_states[pd].stctrl_next_val = val;
	pd_states[pd].stctrl_prev_val = __raw_readl(pd_regs[pd].stctrl);
	pd_states[pd].pwrstst_prev_val = __raw_readl(pd_regs[pd].pwrstst);
	__raw_writel(val, pd_regs[pd].stctrl);

	return 0;
}

static unsigned int mpu_ram_ret_state_change(unsigned int val, unsigned int v)
{
	return var_mod(v, mpu_bits->ram_retst_mask,
					val << mpu_bits->ram_retst_shift);
}

static unsigned int mpu_l1_ret_state_change(unsigned int val, unsigned int v)
{
	return var_mod(v, mpu_bits->l1_retst_mask,
					val << mpu_bits->l1_retst_shift);
}

static unsigned int mpu_l2_ret_state_change(unsigned int val, unsigned int v)
{
	return var_mod(v, mpu_bits->l2_retst_mask,
					val << mpu_bits->l2_retst_shift);
}

static unsigned int icss_mem_ret_state_change(unsigned int val, unsigned int v)
{
	return var_mod(v, per_bits->icss_retst_mask,
					val << per_bits->icss_retst_shift);
}

static unsigned int per_mem_ret_state_change(unsigned int val, unsigned int v)
{
	return var_mod(v, per_bits->per_retst_mask,
					val << per_bits->per_retst_shift);
}

static unsigned int ocmc_mem_ret_state_change(unsigned int val, unsigned int v)
{
	return var_mod(v, per_bits->ram1_retst_mask,
					val << per_bits->ram1_retst_shift);
}

static unsigned int ocmc2_mem_ret_state_change(unsigned int val, unsigned int v)
{
	if (per_bits->ram2_retst_mask)
		return var_mod(v, per_bits->ram2_retst_mask,
					val << per_bits->ram2_retst_shift);
	return v;
}
static unsigned int per_powerst_change(unsigned int val, unsigned int v)
{
	return var_mod(v, per_bits->pwrst_mask,
					val << per_bits->pwrst_shift);
}

static unsigned int mpu_powerst_change(unsigned int val, unsigned int v)
{
	return var_mod(v, mpu_bits->pwrst_mask,
					val << mpu_bits->pwrst_shift);
}

unsigned int get_pd_per_stctrl_val(struct deep_sleep_data *data)
{
	unsigned int v = 0;

	v = per_powerst_change(data->pd_per_state, v);
	v = icss_mem_ret_state_change(data->pd_per_icss_mem_ret_state, v);
	v = per_mem_ret_state_change(data->pd_per_mem_ret_state, v);
	v = ocmc_mem_ret_state_change(data->pd_per_ocmc_ret_state, v);
	v = ocmc2_mem_ret_state_change(data->pd_per_ocmc2_ret_state, v);

	return v;
}

unsigned int get_pd_mpu_stctrl_val(struct deep_sleep_data *data)
{
	unsigned int v = 0;

	v = mpu_powerst_change(data->pd_mpu_state, v);
	v = mpu_ram_ret_state_change(data->pd_mpu_ram_ret_state, v);
	v = mpu_l1_ret_state_change(data->pd_mpu_l1_ret_state, v);
	v = mpu_l2_ret_state_change(data->pd_mpu_l2_ret_state, v);

	return v;
}

void pd_state_restore(enum powerdomain_id pd)
{
	__raw_writel(pd_states[pd].stctrl_prev_val, pd_regs[pd].stctrl);
}

unsigned int pd_read_state(enum powerdomain_id pd)
{
	return __raw_readl(pd_regs[pd].pwrstst) & PD_STATE_MASK;
}

/* Checking only the stst bits for now */
static int verify_pd_transition(enum powerdomain_id pd)
{
	unsigned int ctrl;
	unsigned int stst;

	ctrl = __raw_readl(pd_regs[pd].stctrl);
	stst = __raw_readl(pd_regs[pd].pwrstst);

	if ((ctrl & PD_STATE_MASK) == (stst & PD_STATE_MASK))
		return CMD_STAT_PASS;
	else
		return CMD_STAT_FAIL;
}

int verify_pd_transitions(void)
{
	int result;

	result = verify_pd_transition(PD_MPU);
	if (result == CMD_STAT_FAIL)
		return result;

	return verify_pd_transition(PD_PER);
}

