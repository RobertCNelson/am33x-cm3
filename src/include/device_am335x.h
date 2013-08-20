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

#ifndef __DEVICE_AM335X_H__
#define __DEVICE_AM335X_H__

#define AM335X_IRQ_PRCM_M3_IRQ1		16
#define AM335X_IRQ_UART0INT		17
#define AM335X_IRQ_TINT0		18
#define AM335X_IRQ_TINT1_1MS		19
#define AM335X_IRQ_WDT0INT		20
#define AM335X_IRQ_WDT1INT		21
#define AM335X_IRQ_I2C0INT		22
#define AM335X_IRQ_GPIOINTA		23
#define AM335X_IRQ_GPIOINTB		24
#define AM335X_IRQ_ADC_TSC_INT1		26
#define AM335X_IRQ_RTCINT1		27
#define AM335X_IRQ_RTCALARMINT		28
#define AM335X_IRQ_SMRFLX_MPU		29
#define AM335X_IRQ_SMRFLX_CORE		30
#define AM335X_IRQ_MBINT0		31
#define AM335X_IRQ_USBWAKEUP		33
#define AM335X_IRQ_PRCM_M3_IRQ2		34
#define AM335X_IRQ_USB0WOUT		35
#define AM335X_IRQ_USB1WOUT		36
#define AM335X_IRQ_DMA_INTR_PIN1	37
#define AM335X_IRQ_DMA_INTR_PIN2	38
#define AM335X_IRQ_I2C0_WAKE		40
#define AM335X_IRQ_RTC_TIMER_WAKE	41
#define AM335X_IRQ_RTC_ALARM_WAKE	42
#define AM335X_IRQ_TIMER0_WAKE		43
#define AM335X_IRQ_TIMER1_WAKE		44
#define AM335X_IRQ_UART0_WAKE		45
#define AM335X_IRQ_GPIO0_WAKE0		46
#define AM335X_IRQ_GPIO0_WAKE1		47
#define AM335X_IRQ_MPU_WAKE		48
#define AM335X_IRQ_WDT0_WAKE		49
#define AM335X_IRQ_WDT1_WAKE		50
#define AM335X_IRQ_ADC_TSC_WAKE		51

#define AM335X_NUM_EXT_INTERRUPTS	52

#define UMEM_BASE	0x00000000
#define DMEM_BASE	0x00080000
#define UMEM_BASE_ALIAS	0x20000000
#define DMEM_BASE_ALIAS	0x20080000
#define L4_WKUP_BASE	0x44C00000

#define UMEM_START	0x0
#define UMEM_ALIAS	0x20000000
#define UMEM_SIZE	0x3FFF		/* 16 KB */

#define DMEM_START	0x80000
#define DMEM_ALIAS	0x20080000
#define DMEM_SIZE	0x1FFF		/* 8 KB */

#define CM3_STACK_SIZE	(0xC00)		/* 3KB */
#define CM3_SP		(DMEM_START + CM3_STACK_SIZE)

#define PRCM_BASE	0x44E00000
#define DMTIMER_BASE	0x44E05000
#define GPIO0_BASE	0x44E07000
#define UART0_BASE	0x44E09000
#define I2C0_BASE	0x44E0B000
#define ADC_TSC_BASE	0x44E0D000
#define CONTROL_BASE	0x44E10000
#define DMTIMER1_BASE 	0x44E31000
#define WDT0_BASE	0x44E33000
#define WDT1_BASE	0x44E35000
#define SR0_BASE	0x44E37000
#define SR1_BASE	0x44E39000
#define RTCSS_BASE	0x44E3E000

/* RTC module */

#define RTC_SECONDS_REG		0x00
#define RTC_MINUTES_REG		0x04
#define RTC_HOURS_REG		0x08
#define RTC_DAYS_REG		0x0C
#define RTC_MONTHS_REG		0x10
#define RTC_YEARS_REG		0x14
#define RTC_WEEKS_REG		0x18

#define RTC_ALARM_SECONDS_REG	0x20
#define RTC_ALARM_MINUTES_REG	0x24
#define RTC_ALARM_HOURS_REG	0x28
#define RTC_ALARM_DAYS_REG	0x2c
#define RTC_ALARM_MONTHS_REG	0x30
#define RTC_ALARM_YEARS_REG	0x34

#define RTC_CTRL_REG		0x40
#define RTC_STATUS_REG		0x44
#define RTC_INTERRUPTS_REG	0x48
#define RTC_COMP_LSB_REG	0x4c
#define RTC_COMP_MSB_REG	0x50
#define RTC_OSC_REG		0x54

#define RTC_KICK0		0x6c
#define RTC_KICK1		0x70
#define RTC_SYSCONFIG		0x78
#define RTC_IRQWAKEEN_0		0x7c

#define RTC_ALARM2_SECONDS_REG	0x80
#define RTC_ALARM2_MINUTES_REG	0x84
#define RTC_ALARM2_HOURS_REG	0x88
#define RTC_ALARM2_DAYS_REG	0x8c
#define RTC_ALARM2_MONTHS_REG	0x90
#define RTC_ALARM2_YEARS_REG	0x94

#define RTC_PMIC_REG		0x98
#define RTC_DEBOUNCE_REG	0x9c

/* RTC_CTRL_REG bit fields: */
#define RTC_CTRL_DISABLE	(1<<6)
#define RTC_CTRL_STOP		(1<<0)

/* RTC_STATUS_REG bit fields: */
#define RTC_STATUS_POWER_UP        (1<<7)
#define RTC_STATUS_ALARM           (1<<6)
#define RTC_STATUS_1D_EVENT        (1<<5)
#define RTC_STATUS_1H_EVENT        (1<<4)
#define RTC_STATUS_1M_EVENT        (1<<3)
#define RTC_STATUS_1S_EVENT        (1<<2)
#define RTC_STATUS_RUN             (1<<1)
#define RTC_STATUS_BUSY            (1<<0)

/* RTC_INTERRUPTS_REG bit fields: */
#define RTC_INTERRUPTS_IT_ALARM    (1<<3)
#define RTC_INTERRUPTS_IT_TIMER    (1<<2)

/* I2C */
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

#define CONTROL_STATUS	(CONTROL_BASE + 0x0040)

/* CONTROL_STATUS bit-fields */
#define CONTROL_STATUS_SYSBOOT1_MASK	(0x3 << 22)
#define CONTROL_STATUS_SYSBOOT1_SHIFT	(22)

#define CONTROL_STATUS_DEVTYPE_MASK	(0x3 << 8)
#define CONTROL_STATUS_DEVTYPE_SHIFT	(8)

#define IPC_MSG_REG1	(CONTROL_BASE + 0x1328)
#define IPC_MSG_REG2	(CONTROL_BASE + 0x132c)
#define IPC_MSG_REG3	(CONTROL_BASE + 0x1330)
#define IPC_MSG_REG4	(CONTROL_BASE + 0x1334)
#define IPC_MSG_REG5	(CONTROL_BASE + 0x1338)
#define IPC_MSG_REG6	(CONTROL_BASE + 0x133c)
#define IPC_MSG_REG7	(CONTROL_BASE + 0x1340)
#define IPC_MSG_REG8	(CONTROL_BASE + 0x1344)

#define DEEPSLEEP_CTRL	(CONTROL_BASE + 0x0470)

#define DEVICE_ID	(CONTROL_BASE + 0x0600)

/* DEVICE_ID bit-fields */
#define DEVICE_ID_PARTNUM_MASK	(0xffff << 12)
#define DEVICE_ID_PARTNUM_SHIFT (12)
#define DEVICE_ID_DEVREV_MASK	(0xf << 28)
#define DEVICE_ID_DEVREV_SHIFT	(28)

#define DPLL_PWR_SW_STATUS	(CONTROL_BASE + 0x050C)
#define DPLL_PWR_SW_CTRL	(CONTROL_BASE + 0x1318)
#define SMA2_SPARE_REG		(CONTROL_BASE + 0x1320)

#define DDR_IO_CTRL_REG		(CONTROL_BASE + 0x0E04)
#define VTP0_CTRL_REG		(CONTROL_BASE + 0x0E0C)

#define AM33XX_DDR_CMD0_IOCTRL		(CONTROL_BASE + 0x1404)
#define AM33XX_DDR_CMD1_IOCTRL		(CONTROL_BASE + 0x1408)
#define AM33XX_DDR_CMD2_IOCTRL		(CONTROL_BASE + 0x140C)
#define AM33XX_DDR_DATA0_IOCTRL		(CONTROL_BASE + 0x1440)
#define AM33XX_DDR_DATA1_IOCTRL		(CONTROL_BASE + 0x1444)

/* DPLL_PWR_SW_STATUS bit fields: */
#define PGOODOUT_DDR_STATUS	(1 << 25)
#define PONOUT_DDR_STATUS	(1 << 24)
#define PGOODOUT_DISP_STATUS	(1 << 17)
#define PONOUT_DISP_STATUS	(1 << 16)
#define PGOODOUT_PER_STATUS	(1 << 9)
#define PONOUT_PER_STATUS	(1 << 8)

/* DPLL_PWR_SW_CTRL bit fields: */
#define SW_CTRL_DDR_DPLL	(1 << 31)
#define ISOSCAN_DDR		(1 << 29)
#define RET_DDR			(1 << 28)
#define RESET_DDR		(1 << 27)
#define ISO_DDR			(1 << 26)
#define PGOODIN_DDR		(1 << 25)
#define PONIN_DDR		(1 << 24)
#define SW_CTRL_DISP_DPLL	(1 << 23)
#define ISOSCAN_DISP		(1 << 21)
#define RET_DISP		(1 << 20)
#define RESET_DISP		(1 << 19)
#define ISO_DISP		(1 << 18)
#define PGOODIN_DISP		(1 << 17)
#define PONIN_DISP		(1 << 16)
#define SW_CTRL_PER_DPLL	(1 << 15)
#define ISOSCAN_PER		(1 << 13)
#define RET_PER			(1 << 12)
#define RESET_PER		(1 << 11)
#define ISO_PER			(1 << 10)
#define PGOODIN_PER		(1 << 9)
#define PONIN_PER		(1 << 8)

/* SMA2_SPARE_REG bit fields: */
#define VSLDO_CORE_AUTO_RAMP_EN	(1 << 1)

/* GPIO Register - needed to control VTT */
#define GPIO_CLEARDATAOUT	0x0190
#define GPIO_SETDATAOUT		0x0194

/* VTP0_CTRL_REG bits */
#define VTP_CTRL_START_EN	(1 << 0)
#define VTP_CTRL_LOCK_EN	(1 << 4)
#define VTP_CTRL_READY		(1 << 5)
#define VTP_CTRL_ENABLE		(1 << 6)

#define VTP_CTRL_VAL_DDR2	0x10117
#define VTP_CTRL_VAL_DDR3	0x00

/* DDR_IO_CTRL */
#define DDR3_RST_DEF_VAL	(0x1 << 31)
#define DDR_IO_MDDR_SEL		(0x1 << 28)

#endif
