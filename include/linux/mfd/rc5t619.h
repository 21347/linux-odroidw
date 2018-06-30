/* 
 * include/linux/mfd/rc5t619.h
 *
 * Core driver interface to access RICOH RC5T619 power management chip.
 *
 * Copyright (C) 2012-2013 RICOH COMPANY,LTD
 *
 * Based on code
 *	Copyright (C) 2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef __LINUX_MFD_RC5T619_H
#define __LINUX_MFD_RC5T619_H

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/i2c.h>

/* Maximum number of main interrupts */
#define MAX_INTERRUPT_MASKS	13
#define MAX_MAIN_INTERRUPT	7
#define MAX_GPEDGE_REG		2

/* Power control register */
#define RC5T619_PWR_WD			0x0B
#define RC5T619_PWR_WD_COUNT		0x0C
#define RC5T619_PWR_FUNC		0x0D
#define RC5T619_PWR_SLP_CNT		0x0E
#define RC5T619_PWR_REP_CNT		0x0F
#define RC5T619_PWR_ON_TIMSET		0x10
#define RC5T619_PWR_NOE_TIMSET		0x11
#define RC5T619_PWR_IRSEL		0x15

/* Interrupt enable register */
#define RC5T619_INT_EN_SYS		0x12
#define RC5T619_INT_EN_DCDC		0x40
#define RC5T619_INT_EN_RTC		0xAE
#define RC5T619_INT_EN_ADC1		0x88
#define RC5T619_INT_EN_ADC2		0x89
#define RC5T619_INT_EN_ADC3		0x8A
#define RC5T619_INT_EN_GPIO		0x94
#define RC5T619_INT_EN_GPIO2		0x94 // dummy
#define RC5T619_INT_MSK_CHGCTR		0xBE
#define RC5T619_INT_MSK_CHGSTS1	0xBF
#define RC5T619_INT_MSK_CHGSTS2	0xC0
#define RC5T619_INT_MSK_CHGERR		0xC1
#define RC5T619_INT_MSK_CHGEXTIF	0xD1

/* Interrupt select register */
#define RC5T619_PWR_IRSEL			0x15
#define RC5T619_CHG_CTRL_DETMOD1	0xCA
#define RC5T619_CHG_CTRL_DETMOD2	0xCB
#define RC5T619_CHG_STAT_DETMOD1	0xCC
#define RC5T619_CHG_STAT_DETMOD2	0xCD
#define RC5T619_CHG_STAT_DETMOD3	0xCE


/* interrupt status registers (monitor regs)*/
#define RC5T619_INTC_INTPOL		0x9C
#define RC5T619_INTC_INTEN		0x9D
#define RC5T619_INTC_INTMON		0x9E

#define RC5T619_INT_MON_SYS		0x14
#define RC5T619_INT_MON_DCDC		0x42
#define RC5T619_INT_MON_RTC		0xAF

#define RC5T619_INT_MON_CHGCTR		0xC6
#define RC5T619_INT_MON_CHGSTS1	0xC7
#define RC5T619_INT_MON_CHGSTS2	0xC8
#define RC5T619_INT_MON_CHGERR		0xC9
#define RC5T619_INT_MON_CHGEXTIF	0xD3

/* interrupt clearing registers */
#define RC5T619_INT_IR_SYS		0x13
#define RC5T619_INT_IR_DCDC		0x41
#define RC5T619_INT_IR_RTC		0xAF
#define RC5T619_INT_IR_ADCL		0x8C
#define RC5T619_INT_IR_ADCH		0x8D
#define RC5T619_INT_IR_ADCEND		0x8E
#define RC5T619_INT_IR_GPIOR		0x95
#define RC5T619_INT_IR_GPIOF		0x96
#define RC5T619_INT_IR_CHGCTR		0xC2
#define RC5T619_INT_IR_CHGSTS1		0xC3
#define RC5T619_INT_IR_CHGSTS2		0xC4
#define RC5T619_INT_IR_CHGERR		0xC5
#define RC5T619_INT_IR_CHGEXTIF	0xD2

/* GPIO register base address */
#define RC5T619_GPIO_IOSEL		0x90
#define RC5T619_GPIO_IOOUT		0x91
#define RC5T619_GPIO_GPEDGE1		0x92
#define RC5T619_GPIO_GPEDGE2		0x93
//#define RC5T619_GPIO_EN_GPIR		0x94
//#define RC5T619_GPIO_IR_GPR		0x95
//#define RC5T619_GPIO_IR_GPF		0x96
#define RC5T619_GPIO_MON_IOIN		0x97
#define RC5T619_GPIO_LED_FUNC		0x98

#define RC5T619_REG_BANKSEL		0xFF

/* Charger Control register */
#define RC5T619_CHG_CTL1		0xB3
#define	TIMSET_REG			0xB9

/* ADC Control register */
#define RC5T619_ADC_CNT1		0x64
#define RC5T619_ADC_CNT2		0x65
#define RC5T619_ADC_CNT3		0x66
#define RC5T619_ADC_VADP_THL		0x7C
#define RC5T619_ADC_VSYS_THL		0x80

#define	RC5T619_FG_CTRL		0xE0
#define	RC5T619_PSWR			0x07

#define RC5T619_AIN1_DATAH		0x74
#define RC5T619_AIN1_DATAL		0x75
#define RC5T619_AIN0_DATAH		0x76
#define RC5T619_AIN0_DATAL		0x77

#define RICOH_DC1_SLOT 0x16
#define RICOH_DC2_SLOT 0x17
#define RICOH_DC3_SLOT 0x18
#define RICOH_DC4_SLOT 0x19
#define RICOH_DC5_SLOT 0x1a

#define RICOH_LDO1_SLOT 0x1b
#define RICOH_LDO2_SLOT 0x1c
#define RICOH_LDO3_SLOT 0x1d
#define RICOH_LDO4_SLOT 0x1e
#define RICOH_LDO5_SLOT 0x1f
#define RICOH_LDO6_SLOT 0x20
#define RICOH_LDO7_SLOT 0x21
#define RICOH_LDO8_SLOT 0x22
#define RICOH_LDO9_SLOT 0x23
#define RICOH_LDO10_SLOT 0x24



/* RC5T619 IRQ definitions */
enum {
	RC5T619_IRQ_POWER_ON,
	RC5T619_IRQ_EXTIN,
	RC5T619_IRQ_PRE_VINDT,
	RC5T619_IRQ_PREOT,
	RC5T619_IRQ_POWER_OFF,
	RC5T619_IRQ_NOE_OFF,
	RC5T619_IRQ_WD,
	RC5T619_IRQ_CLK_STP,

	RC5T619_IRQ_DC1LIM,
	RC5T619_IRQ_DC2LIM,
	RC5T619_IRQ_DC3LIM,
	RC5T619_IRQ_DC4LIM,
	RC5T619_IRQ_DC5LIM,

	RC5T619_IRQ_ILIMLIR,
	RC5T619_IRQ_VBATLIR,
	RC5T619_IRQ_VADPLIR,
	RC5T619_IRQ_VUSBLIR,
	RC5T619_IRQ_VSYSLIR,
	RC5T619_IRQ_VTHMLIR,
	RC5T619_IRQ_AIN1LIR,
	RC5T619_IRQ_AIN0LIR,
	
	RC5T619_IRQ_ILIMHIR,
	RC5T619_IRQ_VBATHIR,
	RC5T619_IRQ_VADPHIR,
	RC5T619_IRQ_VUSBHIR,
	RC5T619_IRQ_VSYSHIR,
	RC5T619_IRQ_VTHMHIR,
	RC5T619_IRQ_AIN1HIR,
	RC5T619_IRQ_AIN0HIR,

	RC5T619_IRQ_ADC_ENDIR,

	RC5T619_IRQ_GPIO0,
	RC5T619_IRQ_GPIO1,
	RC5T619_IRQ_GPIO2,
	RC5T619_IRQ_GPIO3,
	RC5T619_IRQ_GPIO4,

	RC5T619_IRQ_CTC,
	RC5T619_IRQ_DALE,

	RC5T619_IRQ_FVADPDETSINT,
	RC5T619_IRQ_FVUSBDETSINT,
	RC5T619_IRQ_FVADPLVSINT,
	RC5T619_IRQ_FVUSBLVSINT,
	RC5T619_IRQ_FWVADPSINT,
	RC5T619_IRQ_FWVUSBSINT,

	RC5T619_IRQ_FONCHGINT,
	RC5T619_IRQ_FCHGCMPINT,
	RC5T619_IRQ_FBATOPENINT,
	RC5T619_IRQ_FSLPMODEINT,
	RC5T619_IRQ_FBTEMPJTA1INT,
	RC5T619_IRQ_FBTEMPJTA2INT,
	RC5T619_IRQ_FBTEMPJTA3INT,
	RC5T619_IRQ_FBTEMPJTA4INT,

	RC5T619_IRQ_FCURTERMINT,
	RC5T619_IRQ_FVOLTERMINT,
	RC5T619_IRQ_FICRVSINT,
	RC5T619_IRQ_FPOOR_CHGCURINT,
	RC5T619_IRQ_FOSCFDETINT1,
	RC5T619_IRQ_FOSCFDETINT2,
	RC5T619_IRQ_FOSCFDETINT3,
	RC5T619_IRQ_FOSCMDETINT,

	RC5T619_IRQ_FDIEOFFINT,
	RC5T619_IRQ_FDIEERRINT,
	RC5T619_IRQ_FBTEMPERRINT,
	RC5T619_IRQ_FVBATOVINT,
	RC5T619_IRQ_FTTIMOVINT,
	RC5T619_IRQ_FRTIMOVINT,
	RC5T619_IRQ_FVADPOVSINT,
	RC5T619_IRQ_FVUSBOVSINT,

	RC5T619_IRQ_FGCDET,
	RC5T619_IRQ_FPCDET,
	RC5T619_IRQ_FWARN_ADP,

	/* Should be last entry */
	RC5T619_NR_IRQS,
};

/* Ricoh619 gpio definitions */
enum {
	RC5T619_GPIO0,
	RC5T619_GPIO1,
	RC5T619_GPIO2,
	RC5T619_GPIO3,
	RC5T619_GPIO4,

	RC5T619_NR_GPIO,
};

enum rc5t619_sleep_control_id {
	RC5T619_DS_DC1,
	RC5T619_DS_DC2,
	RC5T619_DS_DC3,
	RC5T619_DS_DC4,
	RC5T619_DS_DC5,
	RC5T619_DS_LDO1,
	RC5T619_DS_LDO2,
	RC5T619_DS_LDO3,
	RC5T619_DS_LDO4,
	RC5T619_DS_LDO5,
	RC5T619_DS_LDO6,
	RC5T619_DS_LDO7,
	RC5T619_DS_LDO8,
	RC5T619_DS_LDO9,
	RC5T619_DS_LDO10,
	RC5T619_DS_LDORTC1,
	RC5T619_DS_LDORTC2,
	RC5T619_DS_PSO0,
	RC5T619_DS_PSO1,
	RC5T619_DS_PSO2,
	RC5T619_DS_PSO3,
	RC5T619_DS_PSO4,
};

enum rc5t619_adc_id {
	RC5T619_ADC0,
	RC5T619_ADC1,
	RC5T619_ADC_MAX,
};

struct rc5t619_subdev_info {
	int		id;
	const char	*name;
	void		*platform_data;
};

/*
struct rc5t619_rtc_platform_data {
	int irq;
	struct rtc_time time;
};
*/

struct rc5t619_gpio_init_data {
	unsigned output_mode_en:1; 	/* Enable output mode during init */
	unsigned output_val:1;  	/* Output value if it is in output mode */
	unsigned init_apply:1;  	/* Apply init data on configuring gpios*/
	unsigned led_mode:1;  		/* Select LED mode during init */
	unsigned led_func:1;  		/* Set LED function if LED mode is 1 */
};

struct rc5t619 {
	struct device		*dev;
	struct i2c_client	*client;
	struct mutex		io_lock;
	int                 gpio_base;
	struct gpio_chip	gpio_chip;
	int                 irq_base;
//	struct irq_chip		irq_chip;
	int                 chip_irq;
	struct mutex		irq_lock;
	unsigned long		group_irq_en[MAX_MAIN_INTERRUPT];

	/* For main interrupt bits in INTC */
	u8			intc_inten_cache;
	u8			intc_inten_reg;

	/* For group interrupt bits and address */
	u8			irq_en_cache[MAX_INTERRUPT_MASKS];
	u8			irq_en_reg[MAX_INTERRUPT_MASKS];

	/* For gpio edge */
	u8			gpedge_cache[MAX_GPEDGE_REG];
	u8			gpedge_reg[MAX_GPEDGE_REG];

	int			bank_num;
};

struct rc5t619_platform_data {
	int		num_subdevs;
	struct	rc5t619_subdev_info *subdevs;
	int (*init_port)(int irq_num); // Init GPIO for IRQ pin
	int		gpio_base;
	int		irq_base;
	struct rc5t619_gpio_init_data *gpio_init_data;
	int num_gpioinit_data;
	bool enable_shutdown_pin;
	int (*pre_init)(struct rc5t619 *rc5t619);
	int (*post_init)(struct rc5t619 *rc5t619);
};

/* ==================================== */
/* RC5T619 Power_Key device data	*/
/* ==================================== */
struct rc5t619_pwrkey_platform_data {
	int irq;
	unsigned long delay_ms;
};
extern int pwrkey_wakeup;
extern struct rc5t619 *g_rc5t619;
/* ==================================== */
/* RC5T619 battery device data	*/
/* ==================================== */
extern int g_soc;
extern int g_fg_on_mode;

extern int rc5t619_read(struct device *dev, uint8_t reg, uint8_t *val);
extern int rc5t619_read_bank1(struct device *dev, uint8_t reg, uint8_t *val);
extern int rc5t619_bulk_reads(struct device *dev, u8 reg, u8 count,
								uint8_t *val);
extern int rc5t619_bulk_reads_bank1(struct device *dev, u8 reg, u8 count,
								uint8_t *val);
extern int rc5t619_write(struct device *dev, u8 reg, uint8_t val);
extern int rc5t619_write_bank1(struct device *dev, u8 reg, uint8_t val);
extern int rc5t619_bulk_writes(struct device *dev, u8 reg, u8 count,
								uint8_t *val);
extern int rc5t619_bulk_writes_bank1(struct device *dev, u8 reg, u8 count,
								uint8_t *val);
extern int rc5t619_set_bits(struct device *dev, u8 reg, uint8_t bit_mask);
extern int rc5t619_clr_bits(struct device *dev, u8 reg, uint8_t bit_mask);
extern int rc5t619_update(struct device *dev, u8 reg, uint8_t val,
								uint8_t mask);
extern int rc5t619_update_bank1(struct device *dev, u8 reg, uint8_t val,
								uint8_t mask);
extern int rc5t619_power_off(void);
extern int rc5t619_irq_init(struct rc5t619 *rc5t619, int irq, int irq_base);
extern int rc5t619_irq_exit(struct rc5t619 *rc5t619);
extern int rc5t619_power_off(void);

#endif
