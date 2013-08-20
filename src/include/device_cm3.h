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

#ifndef __DEVICE_CM3_H__
#define __DEVICE_CM3_H__

#define CM3_IRQ_PRCM_M3_IRQ1		16
#define CM3_IRQ_UART0INT		17
#define CM3_IRQ_TINT0			18
#define CM3_IRQ_TINT1_1MS		19
#define CM3_IRQ_WDT0INT			20
#define CM3_IRQ_WDT1INT			21
#define CM3_IRQ_I2C0INT			22
#define CM3_IRQ_GPIOINTA		23
#define CM3_IRQ_GPIOINTB		24
#define CM3_IRQ_ADC_TSC_INT1		26
#define CM3_IRQ_RTCINT1			27
#define CM3_IRQ_RTCALARMINT		28
#define CM3_IRQ_SMRFLX_MPU		29
#define CM3_IRQ_SMRFLX_CORE		30
#define CM3_IRQ_MBINT0			31
#define CM3_IRQ_USBWAKEUP		33
#define CM3_IRQ_PRCM_M3_IRQ2		34
#define CM3_IRQ_USB0WOUT		35
#define CM3_IRQ_USB1WOUT		36
#define CM3_IRQ_DMA_INTR_PIN1		37
#define CM3_IRQ_DMA_INTR_PIN2		38
#define CM3_IRQ_I2C0_WAKE		40
#define CM3_IRQ_RTC_TIMER_WAKE		41
#define CM3_IRQ_RTC_ALARM_WAKE		42
#define CM3_IRQ_TIMER0_WAKE		43
#define CM3_IRQ_TIMER1_WAKE		44
#define CM3_IRQ_UART0_WAKE		45
#define CM3_IRQ_GPIO0_WAKE0		46
#define CM3_IRQ_GPIO0_WAKE1		47
#define CM3_IRQ_MPU_WAKE		48
#define CM3_IRQ_WDT0_WAKE		49
#define CM3_IRQ_WDT1_WAKE		50
#define CM3_IRQ_ADC_TSC_WAKE		51

#define CM3_NUM_EXT_INTERRUPTS	52

#define UMEM_BASE	0x00000000
#define DMEM_BASE	0x00080000
#define UMEM_BASE_ALIAS	0x20000000
#define DMEM_BASE_ALIAS	0x20080000

#define UMEM_START	0x0
#define UMEM_ALIAS	0x20000000
#define UMEM_SIZE	0x3FFF		/* 16 KB */

#define DMEM_START	0x80000
#define DMEM_ALIAS	0x20080000
#define DMEM_SIZE	0x1FFF		/* 8 KB */

#define CM3_STACK_SIZE	(0xC00)		/* 3KB */
#define CM3_SP		(DMEM_START + CM3_STACK_SIZE)

#endif
