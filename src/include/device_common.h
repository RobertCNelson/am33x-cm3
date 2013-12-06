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

#ifndef __DEVICE_COMMON_H__
#define __DEVICE_COMMON_H__

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

#define DDR_CMD0_IOCTRL		(CONTROL_BASE + 0x1404)
#define DDR_CMD1_IOCTRL		(CONTROL_BASE + 0x1408)
#define DDR_CMD2_IOCTRL		(CONTROL_BASE + 0x140C)
#define DDR_DATA0_IOCTRL	(CONTROL_BASE + 0x1440)
#define DDR_DATA1_IOCTRL	(CONTROL_BASE + 0x1444)

/* AM43XX Only DDR Control */
#define DDR_DATA2_IOCTRL	(CONTROL_BASE + 0x1448)
#define DDR_DATA3_IOCTRL	(CONTROL_BASE + 0x144C)
#define EMIF_SDRAM_CONFIG_EXT	(CONTROL_BASE + 0x1460)

#define DYNAMIC_PWR_DOWN	(1 << 8)
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
