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
#include <cm3.h>
#include <device_am335x.h>
#include <low_power.h>
#include <prcm.h>
#include <prmam335x.h>
#include <prm43xx.h>
#include <system_am335.h>
#include <clockdomain.h>
#include <hwmod.h>
#include <powerdomain.h>

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
		.pd_per_state			= PD_ON,
		.wake_sources			= MPU_WAKE,
	},
};

/* Clear out the global variables here */
void pm_init(void)
{
	cmd_global_data.cmd_id 	= CMD_ID_INVALID;
	cmd_global_data.data 	= NULL;

	powerdomain_reset();
}

void setup_soc(void)
{
	int var = __raw_readl(DEVICE_ID);

	soc_id = (var & DEVICE_ID_PARTNUM_MASK ) >> DEVICE_ID_PARTNUM_SHIFT;
	soc_rev = (var & DEVICE_ID_DEVREV_MASK) >> DEVICE_ID_DEVREV_SHIFT;

	var = __raw_readl(CONTROL_STATUS);
	soc_type = (var & CONTROL_STATUS_DEVTYPE_MASK) >> CONTROL_STATUS_DEVTYPE_SHIFT;

	clockdomain_init();
	hwmod_init();
	powerdomain_init();
}

/* DeepSleep related */
int disable_master_oscillator(void)
{
	int v = __raw_readl(DEEPSLEEP_CTRL);

	v = var_mod(v, DS_ENABLE_MASK, (1 << DS_ENABLE_SHIFT));

	__raw_writel(v, DEEPSLEEP_CTRL);

	return 0;
}

int enable_master_oscillator(void)
{
	int v = __raw_readl(DEEPSLEEP_CTRL);

	v = var_mod(v, DS_ENABLE_MASK, (0 << DS_ENABLE_SHIFT));

	__raw_writel(v, DEEPSLEEP_CTRL);

	return 0;
}

void configure_deepsleep_count(int ds_count)
{
	int v = __raw_readl(DEEPSLEEP_CTRL);

	v = var_mod(v, DS_COUNT_MASK, (ds_count << DS_COUNT_SHIFT));

	__raw_writel(v, DEEPSLEEP_CTRL);
}

/*
 * A8 is expected to have left the module in a state where it will
 * cause a wakeup event. Ideally, this function should just enable
 * the NVIC interrupt
 */
void configure_wake_sources(int wake_sources)
{
	cmd_wake_sources = wake_sources;

	/* Enable wakeup interrupts from required wake sources */
	if (BB_USB_WAKE)
		nvic_enable_irq(AM335X_IRQ_USBWAKEUP);

	if(BB_I2C0_WAKE)
		nvic_enable_irq(AM335X_IRQ_I2C0_WAKE);

	if(BB_ADTSC_WAKE)
		nvic_enable_irq(AM335X_IRQ_ADC_TSC_WAKE);

	if(BB_UART0_WAKE)
		nvic_enable_irq(AM335X_IRQ_UART0_WAKE);

	if(BB_GPIO0_WAKE0)
		nvic_enable_irq(AM335X_IRQ_GPIO0_WAKE0);

	if(BB_GPIO0_WAKE1)
		nvic_enable_irq(AM335X_IRQ_GPIO0_WAKE1);

	if(BB_RTC_ALARM_WAKE)
		nvic_enable_irq(AM335X_IRQ_RTC_ALARM_WAKE);

	if(BB_TIMER1_WAKE)
		nvic_enable_irq(AM335X_IRQ_TIMER1_WAKE);

	if(BB_WDT1_WAKE)
		nvic_enable_irq(AM335X_IRQ_WDT1_WAKE);

#if 0
	/* Not recommended */
	if(BB_RTC_TIMER_WAKE)
		nvic_enable_irq(AM335X_IRQ_RTC_TIMER_WAKE);

	if(BB_TIMER0_WAKE)
		nvic_enable_irq(AM335X_IRQ_TIMER0_WAKE);

	if(BB_WDT0_WAKE)
		nvic_enable_irq(AM335X_IRQ_WDT0_WAKE);
#endif

	if(BB_USBWOUT0)
		nvic_enable_irq(AM335X_IRQ_USB0WOUT);

	if(BB_USBWOUT1)
		nvic_enable_irq(AM335X_IRQ_USB1WOUT);

	if(BB_MPU_WAKE)
		nvic_enable_irq(AM335X_IRQ_MPU_WAKE);
}

void clear_wake_sources(void)
{
	/*
	 * Clear the global variable
	 * and then disable all wake interrupts
	 */

	cmd_wake_sources = 0x0;	/* All disabled */

	/* Disable the wake interrupts */
	nvic_disable_irq(AM335X_IRQ_USBWAKEUP);
	nvic_disable_irq(AM335X_IRQ_I2C0_WAKE);
	nvic_disable_irq(AM335X_IRQ_RTC_TIMER_WAKE);
	nvic_disable_irq(AM335X_IRQ_RTC_ALARM_WAKE);
	nvic_disable_irq(AM335X_IRQ_TIMER0_WAKE);
	nvic_disable_irq(AM335X_IRQ_TIMER1_WAKE);
	nvic_disable_irq(AM335X_IRQ_UART0_WAKE);
	nvic_disable_irq(AM335X_IRQ_GPIO0_WAKE0);
	nvic_disable_irq(AM335X_IRQ_GPIO0_WAKE1);
	nvic_disable_irq(AM335X_IRQ_MPU_WAKE);
	nvic_disable_irq(AM335X_IRQ_WDT0_WAKE);
	nvic_disable_irq(AM335X_IRQ_WDT1_WAKE);
	nvic_disable_irq(AM335X_IRQ_ADC_TSC_WAKE);
	nvic_disable_irq(AM335X_IRQ_USB0WOUT);
	nvic_disable_irq(AM335X_IRQ_USB1WOUT);

	/* TODO: Clear all the pending interrupts */
}

struct dpll_pd_data {
	int dpll_reg;
	int sw_ctrl_dpll_bit;
	int isoscan_bit;
	int ret_bit;
	int reset_bit;
	int iso_bit;
	int pgoodin_bit;
	int ponin_bit;
	int dpll_pwr_sw_status_reg;
	int pgoodout_status_bit;
	int ponout_status_bit;
};

static const struct dpll_pd_data dpll_data[]  = {
	{
		.dpll_reg		= DPLL_PWR_SW_CTRL,
		.sw_ctrl_dpll_bit	= SW_CTRL_PER_DPLL,
		.isoscan_bit		= ISOSCAN_PER,
		.ret_bit		= RET_PER,
		.reset_bit		= RESET_PER,
		.iso_bit		= ISO_PER,
		.pgoodin_bit		= PGOODIN_PER,
		.ponin_bit		= PONIN_PER,
		.dpll_pwr_sw_status_reg	= DPLL_PWR_SW_STATUS,
		.pgoodout_status_bit	= PGOODOUT_PER_STATUS,
		.ponout_status_bit	= PONOUT_PER_STATUS,
	},
	{
		.dpll_reg		= DPLL_PWR_SW_CTRL,
		.sw_ctrl_dpll_bit	= SW_CTRL_DISP_DPLL,
		.isoscan_bit		= ISOSCAN_DISP,
		.ret_bit		= RET_DISP,
		.reset_bit		= RESET_DISP,
		.iso_bit		= ISO_DISP,
		.pgoodin_bit		= PGOODIN_DISP,
		.ponin_bit		= PONIN_DISP,
		.dpll_pwr_sw_status_reg	= DPLL_PWR_SW_STATUS,
		.pgoodout_status_bit	= PGOODOUT_DISP_STATUS,
		.ponout_status_bit	= PONOUT_DISP_STATUS,
	},
	{
		.dpll_reg		= DPLL_PWR_SW_CTRL,
		.sw_ctrl_dpll_bit	= SW_CTRL_DDR_DPLL,
		.isoscan_bit		= ISOSCAN_DDR,
		.ret_bit		= RET_DDR,
		.reset_bit		= RESET_DDR,
		.iso_bit		= ISO_DDR,
		.pgoodin_bit		= PGOODIN_DDR,
		.ponin_bit		= PONIN_DDR,
		.dpll_pwr_sw_status_reg	= DPLL_PWR_SW_STATUS,
		.pgoodout_status_bit	= PGOODOUT_DDR_STATUS,
		.ponout_status_bit	= PONOUT_DDR_STATUS,
	},
};

/* DPLL power-down Sequence PG 2.x */
void dpll_power_down(unsigned int dpll)
{
	int var;

	/* Configure bit to select Control module selection for DPLL */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var |= dpll_data[dpll].sw_ctrl_dpll_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* Assert ISO bit high */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var |= dpll_data[dpll].iso_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* ISO_SCAN, RET should be asserted high */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var |= (dpll_data[dpll].isoscan_bit | dpll_data[dpll].ret_bit);
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* Assert DPLL reset to 1 */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var |= dpll_data[dpll].reset_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* PGOODIN signal is de-asserted low */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var &= ~dpll_data[dpll].pgoodin_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* PONIN signal is de-asserted low */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var &= ~dpll_data[dpll].ponin_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* Poll for PONOUT and PGOODOUT signal status as 0 */
	while (__raw_readl(dpll_data[dpll].dpll_pwr_sw_status_reg) &
			(dpll_data[dpll].pgoodout_status_bit |
			dpll_data[dpll].ponout_status_bit));
}

/* DPLL Power-up Sequence */
void dpll_power_up(unsigned int dpll)
{
	int var;

	/* PONIN is asserted high */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var |= dpll_data[dpll].ponin_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* Poll for PONOUT to become high  */
	while (!(__raw_readl(dpll_data[dpll].dpll_pwr_sw_status_reg) &
				dpll_data[dpll].ponout_status_bit));

	/* PGOODIN is asserted high */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var |= dpll_data[dpll].pgoodin_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* Poll for PGOODOUT to become high */
	while (!(__raw_readl(dpll_data[dpll].dpll_pwr_sw_status_reg) &
				dpll_data[dpll].pgoodout_status_bit));

	/* De-assert DPLL RESET to 0 */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var &= ~dpll_data[dpll].reset_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* ISO_SCAN, RET should be de-asserted low */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var &= ~(dpll_data[dpll].isoscan_bit | dpll_data[dpll].ret_bit);
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* De-assert ISO signal */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var &= ~dpll_data[dpll].iso_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* Re-Configure bit to select PRCM selection for DPLL */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var &= ~dpll_data[dpll].sw_ctrl_dpll_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);
}

/* DPLL retention update for PG 2.0 */
void am33xx_power_down_plls(void)
{
	if (soc_id == AM335X_SOC_ID && soc_rev > AM335X_REV_ES1_0) {
		dpll_power_down(DPLL_DDR);
		dpll_power_down(DPLL_DISP);
		dpll_power_down(DPLL_PER);
	} else if (soc_id == AM43XX_SOC_ID) {
		/* TODO */
	}
}

/* DPLL retention update for PG 2.x */
void am33xx_power_up_plls(void)
{
	if (soc_id == AM335X_SOC_ID && soc_rev > AM335X_REV_ES1_0) {
		dpll_power_up(DPLL_DDR);
		dpll_power_up(DPLL_DISP);
		dpll_power_up(DPLL_PER);
	} else if (soc_id == AM43XX_SOC_ID) {
		/* TODO */
	}
}

void core_ldo_power_down(void)
{
	unsigned int core_ldo;

	/* Configure RETMODE_ENABLE for CORE LDO */
	if (soc_id == AM335X_SOC_ID && soc_rev > AM335X_REV_ES1_0) {
		core_ldo = __raw_readl(AM335X_PRM_LDO_SRAM_CORE_CTRL);
		core_ldo |= RETMODE_ENABLE;
		__raw_writel(core_ldo, AM335X_PRM_LDO_SRAM_CORE_CTRL);

		/* Poll for LDO Status to be in retention (SRAMLDO_STATUS) */
		while (!(__raw_readl(AM335X_PRM_LDO_SRAM_CORE_CTRL) & SRAMLDO_STATUS));
	} else if (soc_id == AM43XX_SOC_ID) {
		/* TODO */
		/* Note: Need to additionally check for Voltage Monitoring here */
	}
}

void core_ldo_power_up(void)
{
	unsigned int core_ldo;

	/* Disable RETMODE for CORE LDO */
	if (soc_id == AM335X_SOC_ID && soc_rev > AM335X_REV_ES1_0) {
		core_ldo = __raw_readl(AM335X_PRM_LDO_SRAM_CORE_CTRL);
		core_ldo &= ~RETMODE_ENABLE;
		__raw_writel(core_ldo, AM335X_PRM_LDO_SRAM_CORE_CTRL);

		/* Poll for LDO status to be out of retention (SRAMLDO_STATUS) */
		while (__raw_readl(AM335X_PRM_LDO_SRAM_CORE_CTRL) & SRAMLDO_STATUS);
	} else if (soc_id == AM43XX_SOC_ID) {
		/* TODO */
		/* Note: Need to additionally check for Voltage Monitoring here */
	}
}

void ddr_io_suspend(void)
{
	int var;

	/* mddr mode selection required only for PG1.0 */
	if (soc_id == AM335X_SOC_ID && soc_rev == AM335X_REV_ES1_0) {
		var = __raw_readl(DDR_IO_CTRL_REG);
		var |= DDR_IO_MDDR_SEL;
		__raw_writel(var, DDR_IO_CTRL_REG);
	}

	/* Weak pull down for DQ, DM */
	__raw_writel(SUSP_IO_PULL_DATA, AM33XX_DDR_DATA0_IOCTRL);
	__raw_writel(SUSP_IO_PULL_DATA, AM33XX_DDR_DATA1_IOCTRL);

	/* Different sleep sequences for DDR2 and DDR3 */
	if (mem_type == MEM_TYPE_DDR3) {
		/* Weak pull down for macro CMD0/1 */
		__raw_writel(SUSP_IO_PULL_CMD1, AM33XX_DDR_CMD0_IOCTRL);
		__raw_writel(SUSP_IO_PULL_CMD1, AM33XX_DDR_CMD1_IOCTRL);

		/*
		 * Weak pull down for macro CMD2
		 * exception: keep DDR_RESET pullup
		 */
		__raw_writel(SUSP_IO_PULL_CMD2, AM33XX_DDR_CMD2_IOCTRL);

	}
}

void vtt_low(void)
{
	if (vtt_toggle == false)
		return;

	hwmod_state_change(HWMOD_ENABLE, HWMOD_GPIO0);

	__raw_writel((1 << vtt_gpio_pin), GPIO_BASE + GPIO_CLEARDATAOUT);

	hwmod_state_change(HWMOD_DISABLE, HWMOD_GPIO0);
}

/* same offsets for SA and Aegis */
void vtp_disable(void)
{
	if (mem_type == MEM_TYPE_DDR2)
		__raw_writel(VTP_CTRL_VAL_DDR2, VTP0_CTRL_REG);
	else
		__raw_writel(VTP_CTRL_VAL_DDR3, VTP0_CTRL_REG);
}

void sram_ldo_ret_mode(int state)
{
	int var = __raw_readl(AM335X_PRM_LDO_SRAM_MPU_CTRL);

	if (state == RETMODE_ENABLE)
		var |= RETMODE_ENABLE;
	else
		var &= ~RETMODE_ENABLE;

	__raw_writel(var, AM335X_PRM_LDO_SRAM_MPU_CTRL);
}

struct dpll_context {
	int clk_mode_addr;
	int idlest_addr;
	int pll_mode;
};

struct dpll_context am33xx_dplls[] = {
	{
		.clk_mode_addr	= AM335X_CM_CLKMODE_DPLL_PER,
		.idlest_addr	= AM335X_CM_IDLEST_DPLL_PER,
		.pll_mode	= 0,
	},
	{
		.clk_mode_addr	= AM335X_CM_CLKMODE_DPLL_DISP,
		.idlest_addr	= AM335X_CM_IDLEST_DPLL_DISP,
		.pll_mode	= 0,
	},
	{
		.clk_mode_addr	= AM335X_CM_CLKMODE_DPLL_DDR,
		.idlest_addr	= AM335X_CM_IDLEST_DPLL_DDR,
		.pll_mode	= 0,
	},
	{
		.clk_mode_addr	= AM335X_CM_CLKMODE_DPLL_MPU,
		.idlest_addr	= AM335X_CM_IDLEST_DPLL_MPU,
		.pll_mode	= 0,
	},
	{
		.clk_mode_addr	= AM335X_CM_CLKMODE_DPLL_CORE,
		.idlest_addr	= AM335X_CM_IDLEST_DPLL_CORE,
		.pll_mode	= 0,
	},
};

void pll_bypass(unsigned int dpll)
{
	am33xx_dplls[dpll].pll_mode = __raw_readl(am33xx_dplls[dpll].clk_mode_addr);
	__raw_writel(((am33xx_dplls[dpll].pll_mode & ~DPLL_EN_MASK) |
			DPLL_LP_BYP_MODE), am33xx_dplls[dpll].clk_mode_addr);

	/* Wait for DPLL to enter bypass mode */
	while (__raw_readl(am33xx_dplls[dpll].idlest_addr));
}

void pll_lock(unsigned int dpll)
{
	__raw_writel(am33xx_dplls[dpll].pll_mode, am33xx_dplls[dpll].clk_mode_addr);

	if ((am33xx_dplls[dpll].pll_mode & 0x7) == 0x7)
		/* Make sure DPLL Clock is out of Bypass */
		while (!(__raw_readl(am33xx_dplls[dpll].idlest_addr)));
}

void ddr_io_resume(void)
{
	int var;

	/* mddr mode selection required only for PG1.0 */
	if (soc_id == AM335X_SOC_ID && soc_rev == AM335X_REV_ES1_0) {
		var = __raw_readl(DDR_IO_CTRL_REG);
		var &= ~DDR_IO_MDDR_SEL;
		/* Take out IO of mDDR mode */
		__raw_writel(var, DDR_IO_CTRL_REG);
	}

	/* Different sleep sequences for DDR2 and DDR3 */
	if (mem_type == MEM_TYPE_DDR3) {
	/* Disable the pull for CMD2/1/0 */
		__raw_writel(RESUME_IO_PULL_CMD, AM33XX_DDR_CMD2_IOCTRL);
		__raw_writel(RESUME_IO_PULL_CMD, AM33XX_DDR_CMD1_IOCTRL);
		__raw_writel(RESUME_IO_PULL_CMD, AM33XX_DDR_CMD0_IOCTRL);
	}

	/* Disable the pull for DATA1/0 */
	__raw_writel(RESUME_IO_PULL_DATA, AM33XX_DDR_DATA1_IOCTRL);
	__raw_writel(RESUME_IO_PULL_DATA, AM33XX_DDR_DATA0_IOCTRL);
}

/* same offsets for SA and Aegis */
void vtp_enable(void)
{
	int var;

	/* clear the register */
	__raw_writel(0x0, VTP0_CTRL_REG);

	/* write the filter value */
	__raw_writel(0x6, VTP0_CTRL_REG);

	/* set the VTP enable bit */
	var = __raw_readl(VTP0_CTRL_REG);
	__raw_writel((var | VTP_CTRL_ENABLE), VTP0_CTRL_REG);

	/* toggle the CLRZ bit */
	var = __raw_readl(VTP0_CTRL_REG);

	__raw_writel((var & VTP_CTRL_START_EN), VTP0_CTRL_REG);
	__raw_writel((var | VTP_CTRL_START_EN), VTP0_CTRL_REG);

	/* poll for VTP ready */
	while (!(__raw_readl(VTP0_CTRL_REG) & VTP_CTRL_READY));
}

void vtt_high(void)
{
	if (vtt_toggle == false)
		return;

	hwmod_state_change(HWMOD_ENABLE, HWMOD_GPIO0);

	__raw_writel((1 << vtt_gpio_pin), GPIO_BASE + GPIO_SETDATAOUT);

	hwmod_state_change(HWMOD_DISABLE, HWMOD_GPIO0);
}

/* RESET line is applicable only to DDR3 */
void set_ddr_reset(void)
{
	int var;

	/*
	 * TODO: Make this a one-time change in MPU code itself
	 */
	if (mem_type == MEM_TYPE_DDR3) {
		/* hold DDR_RESET high via control module */
		var = __raw_readl(DDR_IO_CTRL_REG);
		var |= DDR3_RST_DEF_VAL;
		__raw_writel(var, DDR_IO_CTRL_REG);
	}
}

void clear_ddr_reset(void)
{
	int var;

	/*
	 * TODO: Make this a one-time change in MPU code itself
	 */
	if (mem_type == MEM_TYPE_DDR3) {
		/* make DDR_RESET low via control module */
		var = __raw_readl(DDR_IO_CTRL_REG);
		var &= ~DDR3_RST_DEF_VAL;
		__raw_writel(var, DDR_IO_CTRL_REG);
	}
}

void ds_save(void)
{
	if (soc_id == AM335X_SOC_ID) {
		set_ddr_reset();

		ddr_io_suspend();

		vtt_low();

		vtp_disable();

		sram_ldo_ret_mode(RETMODE_ENABLE);

		pll_bypass(DPLL_CORE);
		pll_bypass(DPLL_DDR);
		pll_bypass(DPLL_DISP);
		pll_bypass(DPLL_PER);
		pll_bypass(DPLL_MPU);
	}
}

void ds_restore(void)
{
	if (soc_id == AM335X_SOC_ID) {
		pll_lock(DPLL_MPU);
		pll_lock(DPLL_PER);
		pll_lock(DPLL_DISP);
		pll_lock(DPLL_DDR);
		pll_lock(DPLL_CORE);

		sram_ldo_ret_mode(RETMODE_DISABLE);

		vtp_enable();

		/* XXX: Why is this required here for DDR3? */
		hwmod_state_change(HWMOD_ENABLE, HWMOD_EMIF);

		vtt_high();

		ddr_io_resume();

		clear_ddr_reset();
	}
}

int a8_i2c_sleep_handler(unsigned short i2c_sleep_offset)
{
	unsigned char *dmem = (unsigned char *) DMEM_BASE;
	int ret = 0;

	hwmod_state_change(HWMOD_ENABLE, HWMOD_I2C0);

	if (i2c_sleep_offset != 0xffff)
		ret = i2c_write(dmem + i2c_sleep_offset);

	hwmod_state_change(HWMOD_DISABLE, HWMOD_I2C0);

	return ret;
}

int a8_i2c_wake_handler(unsigned short i2c_wake_offset)
{
	unsigned char *dmem = (unsigned char *) DMEM_BASE;
	int ret = 0;

	hwmod_state_change(HWMOD_ENABLE, HWMOD_I2C0);

	if (i2c_wake_offset != 0xffff)
		ret = i2c_write(dmem + i2c_wake_offset);

	hwmod_state_change(HWMOD_DISABLE, HWMOD_I2C0);

	return ret;
}
