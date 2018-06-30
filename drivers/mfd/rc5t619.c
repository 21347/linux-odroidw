/* 
 * driver/mfd/rc5t619.c
 *
 * Core driver implementation to access RICOH RC5T619 power management chip.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*#define DEBUG			1*/
/*#define VERBOSE_DEBUG		1*/
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/rc5t619.h>
#include <linux/power/rc5t619-battery.h>


struct rc5t619 *g_rc5t619;
struct sleep_control_data {
	u8 reg_add;
};

#define SLEEP_INIT(_id, _reg)		\
	[RC5T619_DS_##_id] = {.reg_add = _reg}

static struct sleep_control_data sleep_data[] = {
	SLEEP_INIT(DC1, 0x16),
	SLEEP_INIT(DC2, 0x17),
	SLEEP_INIT(DC3, 0x18),
	SLEEP_INIT(DC4, 0x19),
	SLEEP_INIT(DC5, 0x1A),
	SLEEP_INIT(LDO1, 0x1B),
	SLEEP_INIT(LDO2, 0x1C),
	SLEEP_INIT(LDO3, 0x1D),
	SLEEP_INIT(LDO4, 0x1E),
	SLEEP_INIT(LDO5, 0x1F),
	SLEEP_INIT(LDO6, 0x20),
	SLEEP_INIT(LDO7, 0x21),
	SLEEP_INIT(LDO8, 0x22),
	SLEEP_INIT(LDO9, 0x23),
	SLEEP_INIT(LDO10, 0x24),
	SLEEP_INIT(PSO0, 0x25),
	SLEEP_INIT(PSO1, 0x26),
	SLEEP_INIT(PSO2, 0x27),
	SLEEP_INIT(PSO3, 0x28),
	SLEEP_INIT(PSO4, 0x29),
	SLEEP_INIT(LDORTC1, 0x2A),
};

static inline int __rc5t619_read(struct i2c_client *client,
				  u8 reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
		return ret;
	}

	*val = (uint8_t)ret;
	dev_dbg(&client->dev, "rc5t619: reg read  reg=%x, val=%x\n",
				reg, *val);
	return 0;
}

static inline int __rc5t619_bulk_reads(struct i2c_client *client, u8 reg,
				int len, uint8_t *val)
{
	int ret;
	int i;

	ret = i2c_smbus_read_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading from 0x%02x\n", reg);
		return ret;
	}
	for (i = 0; i < len; ++i) {
		dev_dbg(&client->dev, "rc5t619: reg read  reg=%x, val=%x\n",
				reg + i, *(val + i));
	}
	return 0;
}

static inline int __rc5t619_write(struct i2c_client *client,
				 u8 reg, uint8_t val)
{
	int ret;

	dev_dbg(&client->dev, "rc5t619: reg write  reg=%x, val=%x\n",
				reg, val);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writing 0x%02x to 0x%02x\n",
				val, reg);
		return ret;
	}

	return 0;
}

static inline int __rc5t619_bulk_writes(struct i2c_client *client, u8 reg,
				  int len, uint8_t *val)
{
	int ret;
	int i;

	for (i = 0; i < len; ++i) {
		dev_dbg(&client->dev, "rc5t619: reg write  reg=%x, val=%x\n",
				reg + i, *(val + i));
	}

	ret = i2c_smbus_write_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writings to 0x%02x\n", reg);
		return ret;
	}

	return 0;
}

static inline int set_bank_rc5t619(struct device *dev, int bank)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	int ret;

	if (bank != (bank & 1))
		return -EINVAL;
	if (bank == rc5t619->bank_num)
		return 0;
	ret = __rc5t619_write(to_i2c_client(dev), RC5T619_REG_BANKSEL, bank);
	if (!ret)
		rc5t619->bank_num = bank;

	return ret;
}

int rc5t619_write(struct device *dev, u8 reg, uint8_t val)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&rc5t619->io_lock);
	ret = set_bank_rc5t619(dev, 0);
	if( !ret )
		ret = __rc5t619_write(to_i2c_client(dev), reg, val);
	mutex_unlock(&rc5t619->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rc5t619_write);

int rc5t619_write_bank1(struct device *dev, u8 reg, uint8_t val)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&rc5t619->io_lock);
	ret = set_bank_rc5t619(dev, 1);
	if( !ret ) 
		ret = __rc5t619_write(to_i2c_client(dev), reg, val);
	mutex_unlock(&rc5t619->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rc5t619_write_bank1);

int rc5t619_bulk_writes(struct device *dev, u8 reg, u8 len, uint8_t *val)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&rc5t619->io_lock);
	ret = set_bank_rc5t619(dev, 0);
	if( !ret )
		ret = __rc5t619_bulk_writes(to_i2c_client(dev), reg, len, val);
	mutex_unlock(&rc5t619->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rc5t619_bulk_writes);

int rc5t619_bulk_writes_bank1(struct device *dev, u8 reg, u8 len, uint8_t *val)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&rc5t619->io_lock);
	ret = set_bank_rc5t619(dev, 1);
	if( !ret ) 
		ret = __rc5t619_bulk_writes(to_i2c_client(dev), reg, len, val);
	mutex_unlock(&rc5t619->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rc5t619_bulk_writes_bank1);

int rc5t619_read(struct device *dev, u8 reg, uint8_t *val)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&rc5t619->io_lock);
	ret = set_bank_rc5t619(dev, 0);
	if( !ret )
		ret = __rc5t619_read(to_i2c_client(dev), reg, val);
	mutex_unlock(&rc5t619->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rc5t619_read);

int rc5t619_read_bank1(struct device *dev, u8 reg, uint8_t *val)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&rc5t619->io_lock);
	ret = set_bank_rc5t619(dev, 1);
	if( !ret )
		ret =  __rc5t619_read(to_i2c_client(dev), reg, val);
	mutex_unlock(&rc5t619->io_lock);

	return ret;
}

EXPORT_SYMBOL_GPL(rc5t619_read_bank1);

int rc5t619_bulk_reads(struct device *dev, u8 reg, u8 len, uint8_t *val)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&rc5t619->io_lock);
	ret = set_bank_rc5t619(dev, 0);
	if( !ret ) 
		ret = __rc5t619_bulk_reads(to_i2c_client(dev), reg, len, val);
	mutex_unlock(&rc5t619->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rc5t619_bulk_reads);

int rc5t619_bulk_reads_bank1(struct device *dev, u8 reg, u8 len, uint8_t *val)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&rc5t619->io_lock);
	ret = set_bank_rc5t619(dev, 1);
	if( !ret ) 
		ret = __rc5t619_bulk_reads(to_i2c_client(dev), reg, len, val);
	mutex_unlock(&rc5t619->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rc5t619_bulk_reads_bank1);

int rc5t619_set_bits(struct device *dev, u8 reg, uint8_t bit_mask)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&rc5t619->io_lock);
	ret = set_bank_rc5t619(dev, 0);
	if (!ret) {
		ret = __rc5t619_read(to_i2c_client(dev), reg, &reg_val);
		if (ret)
			goto out;

		if ((reg_val & bit_mask) != bit_mask) {
			reg_val |= bit_mask;
			ret = __rc5t619_write(to_i2c_client(dev), reg,
								 reg_val);
		}
	}
out:
	mutex_unlock(&rc5t619->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(rc5t619_set_bits);

int rc5t619_clr_bits(struct device *dev, u8 reg, uint8_t bit_mask)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&rc5t619->io_lock);
	ret = set_bank_rc5t619(dev, 0);
	if( !ret ){
		ret = __rc5t619_read(to_i2c_client(dev), reg, &reg_val);
		if (ret)
			goto out;

		if (reg_val & bit_mask) {
			reg_val &= ~bit_mask;
			ret = __rc5t619_write(to_i2c_client(dev), reg,
								 reg_val);
		}
	}
out:
	mutex_unlock(&rc5t619->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(rc5t619_clr_bits);

int rc5t619_update(struct device *dev, u8 reg, uint8_t val, uint8_t mask)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&rc5t619->io_lock);
	ret = set_bank_rc5t619(dev, 0);
	if( !ret ){
		ret = __rc5t619_read(rc5t619->client, reg, &reg_val);
		if (ret)
			goto out;

		if ((reg_val & mask) != val) {
			reg_val = (reg_val & ~mask) | (val & mask);
			ret = __rc5t619_write(rc5t619->client, reg, reg_val);
		}
	}
out:
	mutex_unlock(&rc5t619->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(rc5t619_update);

int rc5t619_update_bank1(struct device *dev, u8 reg, uint8_t val, uint8_t mask)
{
	struct rc5t619 *rc5t619 = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&rc5t619->io_lock);
	ret = set_bank_rc5t619(dev, 1);
	if( !ret ){
		ret = __rc5t619_read(rc5t619->client, reg, &reg_val);
		if (ret)
			goto out;

		if ((reg_val & mask) != val) {
			reg_val = (reg_val & ~mask) | (val & mask);
			ret = __rc5t619_write(rc5t619->client, reg, reg_val);
		}
	}
out:
	mutex_unlock(&rc5t619->io_lock);
	return ret;
}

static struct i2c_client *rc5t619_i2c_client;
int rc5t619_power_off(void)
{
	int ret;
	uint8_t val;
	int err = -1;
	int charge_state;
	uint8_t status;
	struct rc5t619 *rc5t619 = g_rc5t619;

	val = g_soc;
	val &= 0x7f;
	rc5t619_read(rc5t619->dev, 0xBD, &status);
	charge_state = (status & 0x1F);
//	supply_state = ((status & 0xC0) >> 6);

	ret = rc5t619_write(rc5t619->dev, RC5T619_PSWR, val);
	if (ret < 0)
		dev_err(rc5t619->dev, "Error in writing PSWR_REG\n");

	if (g_fg_on_mode == 0) {
		ret = rc5t619_clr_bits(rc5t619->dev,
					 RC5T619_FG_CTRL, 0x01);
		if (ret < 0)
			dev_err(rc5t619->dev, "Error in writing FG_CTRL\n");
	}
	
	/* set rapid timer 300 min */
	err = rc5t619_set_bits(rc5t619->dev, TIMSET_REG, 0x03);
	if (err < 0)
		dev_err(rc5t619->dev, "Error in writing the TIMSET_Reg\n");
  
    ret = rc5t619_write(rc5t619->dev, RC5T619_INTC_INTEN, 0); 

	if (!rc5t619_i2c_client)
		return -EINVAL;
	ret = rc5t619_clr_bits(rc5t619->dev,RC5T619_PWR_REP_CNT,(0x0<<0));//Not repeat power ON after power off(Power Off/N_OE)

	if(( charge_state == CHG_STATE_CHG_TRICKLE)||( charge_state == CHG_STATE_CHG_RAPID))
		 rc5t619_set_bits(rc5t619->dev, RC5T619_PWR_REP_CNT,(0x0<<0));//Power OFF
	ret = rc5t619_set_bits(rc5t619->dev, RC5T619_PWR_SLP_CNT,(0x1<<4));//Power OFF
	if (ret < 0) {
		dev_err(rc5t619->dev, "rc5t619 power off error!\n");
		return err;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(rc5t619_power_off);

static int rc5t619_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct rc5t619 *rc5t619 = container_of(gc, struct rc5t619, gpio_chip);
	uint8_t val;
	int ret;

	ret = rc5t619_read(rc5t619->dev, RC5T619_GPIO_MON_IOIN, &val);
	if (ret < 0)
		return ret;

	return ((val & (0x1 << offset)) != 0);
}

static void rc5t619_gpio_set(struct gpio_chip *gc, unsigned offset,
			int value)
{
	struct rc5t619 *rc5t619 = container_of(gc, struct rc5t619, gpio_chip);
	if (value)
		rc5t619_set_bits(rc5t619->dev, RC5T619_GPIO_IOOUT,
						1 << offset);
	else
		rc5t619_clr_bits(rc5t619->dev, RC5T619_GPIO_IOOUT,
						1 << offset);
}

static int rc5t619_gpio_input(struct gpio_chip *gc, unsigned offset)
{
	struct rc5t619 *rc5t619 = container_of(gc, struct rc5t619, gpio_chip);

	return rc5t619_clr_bits(rc5t619->dev, RC5T619_GPIO_IOSEL,
						1 << offset);
}

static int rc5t619_gpio_output(struct gpio_chip *gc, unsigned offset,
				int value)
{
	struct rc5t619 *rc5t619 = container_of(gc, struct rc5t619, gpio_chip);

	rc5t619_gpio_set(gc, offset, value);
	return rc5t619_set_bits(rc5t619->dev, RC5T619_GPIO_IOSEL,
						1 << offset);
}

static int rc5t619_gpio_to_irq(struct gpio_chip *gc, unsigned off)
{
	struct rc5t619 *rc5t619 = container_of(gc, struct rc5t619, gpio_chip);

	if ((off >= 0) && (off < 8))
		return rc5t619->irq_base + RC5T619_IRQ_GPIO0 + off;

	return -EIO;
}


static void rc5t619_gpio_init(struct rc5t619 *rc5t619,
	struct rc5t619_platform_data *pdata)
{
	int ret;
	int i;
	struct rc5t619_gpio_init_data *ginit;

	if (pdata->gpio_base  <= 0)
		return;

	for (i = 0; i < pdata->num_gpioinit_data; ++i) {
		ginit = &pdata->gpio_init_data[i];

		if (!ginit->init_apply)
			continue;

		if (ginit->output_mode_en) {
			/* GPIO output mode */
			if (ginit->output_val)
				/* output H */
				ret = rc5t619_set_bits(rc5t619->dev,
					RC5T619_GPIO_IOOUT, 1 << i);
			else
				/* output L */
				ret = rc5t619_clr_bits(rc5t619->dev,
					RC5T619_GPIO_IOOUT, 1 << i);
			if (!ret)
				ret = rc5t619_set_bits(rc5t619->dev,
					RC5T619_GPIO_IOSEL, 1 << i);
		} else
			/* GPIO input mode */
			ret = rc5t619_clr_bits(rc5t619->dev,
					RC5T619_GPIO_IOSEL, 1 << i);

		/* if LED function enabled in OTP */
		if (ginit->led_mode) {
			/* LED Mode 1 */
			if (i == 0)	/* GP0 */
				ret = rc5t619_set_bits(rc5t619->dev,
					 RC5T619_GPIO_LED_FUNC,
					 0x04 | (ginit->led_func & 0x03));
			if (i == 1)	/* GP1 */
				ret = rc5t619_set_bits(rc5t619->dev,
					 RC5T619_GPIO_LED_FUNC,
					 0x40 | (ginit->led_func & 0x03) << 4);

		}


		if (ret < 0)
			dev_err(rc5t619->dev, "Gpio %d init "
				"dir configuration failed: %d\n", i, ret);

	}

	rc5t619->gpio_chip.owner		= THIS_MODULE;
	rc5t619->gpio_chip.label		= rc5t619->client->name;
	rc5t619->gpio_chip.dev			= rc5t619->dev;
	rc5t619->gpio_chip.base		= pdata->gpio_base;
	rc5t619->gpio_chip.ngpio		= RC5T619_NR_GPIO;
	rc5t619->gpio_chip.can_sleep	= 1;

	rc5t619->gpio_chip.direction_input	= rc5t619_gpio_input;
	rc5t619->gpio_chip.direction_output	= rc5t619_gpio_output;
	rc5t619->gpio_chip.set			= rc5t619_gpio_set;
	rc5t619->gpio_chip.get			= rc5t619_gpio_get;
	rc5t619->gpio_chip.to_irq	  	= rc5t619_gpio_to_irq;

	ret = gpiochip_add(&rc5t619->gpio_chip);
	if (ret)
		dev_warn(rc5t619->dev, "GPIO registration failed: %d\n", ret);
}

static int rc5t619_remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int rc5t619_remove_subdevs(struct rc5t619 *rc5t619)
{
	return device_for_each_child(rc5t619->dev, NULL,
				     rc5t619_remove_subdev);
}

static int rc5t619_add_subdevs(struct rc5t619 *rc5t619,
				struct rc5t619_platform_data *pdata)
{
	struct rc5t619_subdev_info *subdev;
	struct platform_device *pdev;
	int i, ret = 0;

	for (i = 0; i < pdata->num_subdevs; i++) {
		subdev = &pdata->subdevs[i];

		pdev = platform_device_alloc(subdev->name, subdev->id);

		pdev->dev.parent = rc5t619->dev;
		pdev->dev.platform_data = subdev->platform_data;

		ret = platform_device_add(pdev);
		if (ret)
			goto failed;
	}
	return 0;

failed:
	rc5t619_remove_subdevs(rc5t619);
	return ret;
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
static void print_regs(const char *header, struct seq_file *s,
		struct i2c_client *client, int start_offset,
		int end_offset)
{
	uint8_t reg_val;
	int i;
	int ret;

	seq_printf(s, "%s\n", header);
	for (i = start_offset; i <= end_offset; ++i) {
		ret = __rc5t619_read(client, i, &reg_val);
		if (ret >= 0)
			seq_printf(s, "Reg 0x%02x Value 0x%02x\n", i, reg_val);
	}
	seq_printf(s, "------------------\n");
}

static int dbg_ricoh_show(struct seq_file *s, void *unused)
{
	struct rc5t619 *ricoh = s->private;
	struct i2c_client *client = ricoh->client;

	seq_printf(s, "RC5T619 Registers\n");
	seq_printf(s, "------------------\n");

	print_regs("System Regs",		s, client, 0x0, 0x05);
	print_regs("Power Control Regs",	s, client, 0x07, 0x2B);
	print_regs("DCDC  Regs",		s, client, 0x2C, 0x43);
	print_regs("LDO   Regs",		s, client, 0x44, 0x61);
	print_regs("ADC   Regs",		s, client, 0x64, 0x8F);
	print_regs("GPIO  Regs",		s, client, 0x90, 0x98);
	print_regs("INTC  Regs",		s, client, 0x9C, 0x9E);
	print_regs("RTC   Regs",		s, client, 0xA0, 0xAF);
	print_regs("OPT   Regs",		s, client, 0xB0, 0xB1);
	print_regs("CHG   Regs",		s, client, 0xB2, 0xDF);
	print_regs("FUEL  Regs",		s, client, 0xE0, 0xFC);
	return 0;
}

static int dbg_ricoh_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_ricoh_show, inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_ricoh_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static void __init rc5t619_debuginit(struct rc5t619 *ricoh)
{
	(void)debugfs_create_file("rc5t619", S_IRUGO, NULL,
			ricoh, &debug_fops);
}
#else
static void print_regs(const char *header, struct i2c_client *client,
		int start_offset, int end_offset)
{
	uint8_t reg_val;
	int i;
	int ret;

	printk(KERN_INFO "%s\n", header);
	for (i = start_offset; i <= end_offset; ++i) {
		ret = __rc5t619_read(client, i, &reg_val);
		if (ret >= 0)
			printk(KERN_INFO "Reg 0x%02x Value 0x%02x\n",
							 i, reg_val);
	}
	printk(KERN_INFO "------------------\n");
}
static void __init rc5t619_debuginit(struct rc5t619 *ricoh)
{
	struct i2c_client *client = ricoh->client;

	printk(KERN_INFO "RC5T619 Registers\n");
	printk(KERN_INFO "------------------\n");

	print_regs("System Regs",		client, 0x0, 0x05);
	print_regs("Power Control Regs",	client, 0x07, 0x2B);
	print_regs("DCDC  Regs",		client, 0x2C, 0x43);
	print_regs("LDO   Regs",		client, 0x44, 0x5C);
	print_regs("ADC   Regs",		client, 0x64, 0x8F);
	print_regs("GPIO  Regs",		client, 0x90, 0x9B);
	print_regs("INTC  Regs",		client, 0x9C, 0x9E);
	print_regs("OPT   Regs",		client, 0xB0, 0xB1);
	print_regs("CHG   Regs",		client, 0xB2, 0xDF);
	print_regs("FUEL  Regs",		client, 0xE0, 0xFC);

	return 0;
}
#endif

static void rc5t619_noe_init(struct rc5t619 *ricoh)
{
	struct i2c_client *client = ricoh->client;
	
	__rc5t619_write(client, RC5T619_PWR_NOE_TIMSET, 0x0); //N_OE timer setting to 128mS
	__rc5t619_write(client, RC5T619_PWR_REP_CNT, 0x0); //Repeat power ON after reset (Power Off/N_OE)
}

static int rc5t619_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct rc5t619 *rc5t619;
	struct rc5t619_platform_data *pdata = client->dev.platform_data;
	int ret;
	uint8_t control;
	printk(KERN_INFO "%s\n", __func__);

	rc5t619 = kzalloc(sizeof(struct rc5t619), GFP_KERNEL);
	if (rc5t619 == NULL)
		return -ENOMEM;

	rc5t619->client = client;
	rc5t619->dev = &client->dev;
	i2c_set_clientdata(client, rc5t619);

	mutex_init(&rc5t619->io_lock);

	ret = rc5t619_read(rc5t619->dev, 0x36, &control);
	if ((control < 0) || (control == 0xff)) {
		printk(KERN_INFO "The device is not rc5t619\n");
		return 0;
	}
	/***************set noe time 128ms**************/
	ret = rc5t619_set_bits(rc5t619->dev,0x11,(0x1 <<3));
	ret = rc5t619_clr_bits(rc5t619->dev,0x11,(0x7 <<0));
	ret = rc5t619_clr_bits(rc5t619->dev,0x11,(0x1 <<3));
 	/**********************************************/

	/***************set PKEY long press time 0sec*******/
	ret = rc5t619_set_bits(rc5t619->dev,0x10,(0x1 <<7));
	ret = rc5t619_clr_bits(rc5t619->dev,0x10,(0x1 <<3));
	ret = rc5t619_clr_bits(rc5t619->dev,0x10,(0x1 <<7));
 	/**********************************************/

	rc5t619->bank_num = 0;

//	ret = pdata->init_port(client->irq); // For init PMIC_IRQ port
	if (client->irq) {
		ret = rc5t619_irq_init(rc5t619, client->irq, pdata->irq_base);
		if (ret) {
			dev_err(&client->dev, "IRQ init failed: %d\n", ret);
			goto err_irq_init;
		}
	}
	
	ret = rc5t619_add_subdevs(rc5t619, pdata);
	if (ret) {
		dev_err(&client->dev, "add devices failed: %d\n", ret);
		goto err_add_devs;
	}
	rc5t619_noe_init(rc5t619);
	
	g_rc5t619 = rc5t619;
	if (pdata && pdata->pre_init) {
		ret = pdata->pre_init(rc5t619);
		if (ret != 0) {
			dev_err(rc5t619->dev, "pre_init() failed: %d\n", ret);
			goto err;
		}
	}

	rc5t619_gpio_init(rc5t619, pdata);

	rc5t619_debuginit(rc5t619);

	if (pdata && pdata->post_init) {
		ret = pdata->post_init(rc5t619);
		if (ret != 0) {
			dev_err(rc5t619->dev, "post_init() failed: %d\n", ret);
			goto err;
		}
	}

	rc5t619_i2c_client = client;
	return 0;
err:
	mfd_remove_devices(rc5t619->dev);
	kfree(rc5t619);
err_add_devs:
	if (client->irq)
		rc5t619_irq_exit(rc5t619);
err_irq_init:
	kfree(rc5t619);
	return ret;
}

static int rc5t619_i2c_remove(struct i2c_client *client)
{
	struct rc5t619 *rc5t619 = i2c_get_clientdata(client);

	if (client->irq)
		rc5t619_irq_exit(rc5t619);

	rc5t619_remove_subdevs(rc5t619);
	kfree(rc5t619);
	return 0;
}

#ifdef CONFIG_PM
static int rc5t619_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	if (client->irq)
		disable_irq(client->irq);
	return 0;
}

int pwrkey_wakeup;
static int rc5t619_i2c_resume(struct i2c_client *client)
{
	uint8_t reg_val=0;
	int ret;

	ret = __rc5t619_read(client, RC5T619_INT_IR_SYS, &reg_val);
	if(reg_val & 0x01) { //If PWR_KEY wakeup
		pwrkey_wakeup = 1;
		__rc5t619_write(client, RC5T619_INT_IR_SYS, 0x0); //Clear PWR_KEY IRQ
	}

	enable_irq(client->irq);
	return 0;
}

#endif

static const struct i2c_device_id rc5t619_i2c_id[] = {
	{"rc5t619", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, rc5t619_i2c_id);

static struct i2c_driver rc5t619_i2c_driver = {
	.driver = {
		   .name = "rc5t619",
		   .owner = THIS_MODULE,
		   },
	.probe = rc5t619_i2c_probe,
	.remove = rc5t619_i2c_remove,
#ifdef CONFIG_PM
	.suspend = rc5t619_i2c_suspend,
	.resume = rc5t619_i2c_resume,
#endif
	.id_table = rc5t619_i2c_id,
};


static int __init rc5t619_i2c_init(void)
{
	int ret = -ENODEV;
	ret = i2c_add_driver(&rc5t619_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);

	return ret;
}

subsys_initcall_sync(rc5t619_i2c_init);

static void __exit rc5t619_i2c_exit(void)
{
	i2c_del_driver(&rc5t619_i2c_driver);
}

module_exit(rc5t619_i2c_exit);

MODULE_DESCRIPTION("RICOH RC5T619 PMU multi-function core driver");
MODULE_AUTHOR("zhangqing <zhangqing@rock-chips.com>");
MODULE_LICENSE("GPL");
