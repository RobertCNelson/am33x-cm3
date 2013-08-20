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

#include <device_common.h>
#include <dpll.h>
#include <io.h>
#include <i2c.h>

#define OMAP_I2C_SYSC_AUTOIDLE	(1 << 0)

#define OMAP_I2C_STAT_BB	(1 << 12)
#define OMAP_I2C_STAT_ARDY	(1 << 2)
#define OMAP_I2C_STAT_NACK	(1 << 1)
#define OMAP_I2C_STAT_AL	(1 << 0)

#define OMAP_I2C_CON_EN		(1 << 15)
#define OMAP_I2C_CON_MST	(1 << 10)
#define OMAP_I2C_CON_TRX	(1 << 9)
#define OMAP_I2C_CON_STP	(1 << 1)
#define OMAP_I2C_CON_STT	(1 << 0)

#define OMAP_I2C_SYSC_REG	0x10
#define OMAP_I2C_STAT_RAW_REG	0x24
#define OMAP_I2C_STAT_REG	0x28
#define OMAP_I2C_IRQENABLE_SET	0x2c
#define OMAP_I2C_IRQENABLE_CLR	0x30
#define OMAP_I2C_CNT_REG	0x98
#define OMAP_I2C_DATA_REG	0x9c
#define OMAP_I2C_CON_REG	0xa4
#define OMAP_I2C_SA_REG		0xac
#define OMAP_I2C_PSC_REG	0xb0
#define OMAP_I2C_SCLL_REG	0xb4
#define OMAP_I2C_SCLH_REG	0xb8

static void i2c_reg_write(unsigned short val, int reg)
{
	__raw_writew(val, I2C0_BASE + reg);
}

static unsigned short i2c_reg_read(int reg)
{
	return __raw_readw(I2C0_BASE + reg);
}

static void i2c_ack_all(void)
{
	/* Ack all events */
	i2c_reg_write(0xffff, OMAP_I2C_STAT_REG);
}

/* Wait for bus to be ready (bus busy bit cleared) */
static int i2c_wait_for_bb(void)
{
	int ret = -1;
	int i;

	for (i = 0; i < 1000; i++) {
		if (!(i2c_reg_read(OMAP_I2C_STAT_RAW_REG) & OMAP_I2C_STAT_BB)) {
			ret = 0;
			break;
		}
	}

	i2c_ack_all();
	return ret;
}

/* Wait for transfer to complete */
static int i2c_wait_for_ardy(void)
{
	int ret = -1;
	int i;

	for (i = 0; i < 1000; i++) {
		unsigned short stat;
		stat = i2c_reg_read(OMAP_I2C_STAT_RAW_REG);
		if (stat & (OMAP_I2C_STAT_NACK | OMAP_I2C_STAT_AL))
			break;
		if (stat & OMAP_I2C_STAT_ARDY) {
			ret = 0;
			break;
		}
	}

	i2c_ack_all();
	return ret;
}

static void i2c_program_freq(int speed_khz)
{
	const int xtal_freqs[] = {19200, 24000, 25000, 26000};
	int index;
	int xtal_freq;
	int n2_div;
	int per_clkoutm2;
	int i2c_fclk;
	int scl;
	int scll;
	int sclh;

	/* Calculate I2C0 functional clock */
	index = __raw_readl(CONTROL_STATUS);
	index &= CONTROL_STATUS_SYSBOOT1_MASK;
	index >>= CONTROL_STATUS_SYSBOOT1_SHIFT;
	xtal_freq = xtal_freqs[index];

	/* FIXME: May need to change for am43xx */
	n2_div = dpll_get_div(DPLL_PER);
	per_clkoutm2 = xtal_freq / (n2_div + 1);
	i2c_fclk = per_clkoutm2 / 4;

	scl = i2c_fclk / (speed_khz ? : 1);
	scll = scl - (scl / 3) - 7;
	if (scll < 0)
		scll = 0;
	if (scll > 255)
		scll = 255;
	sclh = (scl / 3) - 5;
	if (sclh < 0)
		sclh = 0;
	if (sclh > 255)
		sclh = 255;

	/* Frequency of XTAL will never be high enough to require PSC */
	i2c_reg_write(0, OMAP_I2C_PSC_REG);
	i2c_reg_write(scll, OMAP_I2C_SCLL_REG);
	i2c_reg_write(sclh, OMAP_I2C_SCLH_REG);
}

int i2c_write(const unsigned char *sequence)
{
	unsigned char len;
	unsigned short speed_khz;
	unsigned long orig_sysc;
	unsigned short orig_con;
	unsigned short orig_irq_en;
	unsigned short orig_psc;
	unsigned short orig_scll;
	unsigned short orig_sclh;

	/* Save modified registers */
	orig_sysc = __raw_readl(I2C0_BASE + OMAP_I2C_SYSC_REG);
	orig_con = i2c_reg_read(OMAP_I2C_CON_REG);
	orig_irq_en = i2c_reg_read(OMAP_I2C_IRQENABLE_SET);
	orig_psc = i2c_reg_read(OMAP_I2C_PSC_REG);
	orig_scll = i2c_reg_read(OMAP_I2C_SCLL_REG);
	orig_sclh = i2c_reg_read(OMAP_I2C_SCLH_REG);

	/* Disable auto-idle */
	__raw_writel(orig_sysc & ~OMAP_I2C_SYSC_AUTOIDLE, I2C0_BASE +
							OMAP_I2C_SYSC_REG);
	/* Disable controller */
	i2c_reg_write(0, OMAP_I2C_CON_REG);

	speed_khz = sequence[0] | sequence[1] << 8;
	sequence += 2;

	i2c_program_freq(speed_khz);

	/* Enable controller */
	i2c_reg_write(OMAP_I2C_CON_EN, OMAP_I2C_CON_REG);

	/* Disable all event interrupts */
	i2c_reg_write(0xffff, OMAP_I2C_IRQENABLE_CLR);
	i2c_ack_all();

	while ((len = *sequence++)) {

		if (i2c_wait_for_bb() < 0)
			return -1;

		/* Program I2C target address */
		i2c_reg_write(*sequence++, OMAP_I2C_SA_REG);

		/* Store the length of the transfer */
		i2c_reg_write(len, OMAP_I2C_CNT_REG);

		/* Configure I2C controller for transfer */
		i2c_reg_write(OMAP_I2C_CON_EN |	OMAP_I2C_CON_MST |
			      OMAP_I2C_CON_TRX | OMAP_I2C_CON_STP |
			      OMAP_I2C_CON_STT, OMAP_I2C_CON_REG);
		i2c_ack_all();

		/* Write out the data */
		while (len--)
			i2c_reg_write(*sequence++, OMAP_I2C_DATA_REG);

		if (i2c_wait_for_ardy() < 0)
			return -1;
	}

	/* Disable controller */
	i2c_reg_write(0, OMAP_I2C_CON_REG);

	/* Restore registers */
	i2c_reg_write(orig_psc, OMAP_I2C_PSC_REG);
	i2c_reg_write(orig_scll, OMAP_I2C_SCLL_REG);
	i2c_reg_write(orig_sclh, OMAP_I2C_SCLL_REG);
	i2c_reg_write(orig_con, OMAP_I2C_CON_REG);
	i2c_reg_write(orig_irq_en, OMAP_I2C_IRQENABLE_SET);
	__raw_writel(orig_sysc, I2C0_BASE + OMAP_I2C_SYSC_REG);

	return 0;
}
