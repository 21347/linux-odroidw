/* 
 * driver/iio/adc/rc5t619_adc.c
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

#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iio/machine.h>
#include <linux/mfd/rc5t619.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct rc5t619_adc {
	struct device      *dev;
	struct mutex lock;
};

static const int rc5t619_scale[RC5T619_ADC_MAX] = {
	[RC5T619_ADC0]      = 4095,
	[RC5T619_ADC1]      = 4095,
};

static int rc5t619_adc_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	struct rc5t619_adc *adc = iio_priv(indio_dev);
	enum rc5t619_adc_id id = chan->channel;
	unsigned int result;
	u8 adc_sel;
	u8 addr_H, addr_L;
	u8 rawdata_H,rawdata_L;
	int ret=0;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	if(id == RC5T619_ADC0){
		adc_sel = 0x17;
		addr_H = RC5T619_AIN0_DATAH;
		addr_L = RC5T619_AIN0_DATAL;
	}
	else if(id == RC5T619_ADC1){
		adc_sel = 0x16;
		addr_H = RC5T619_AIN1_DATAH;
		addr_L = RC5T619_AIN1_DATAL;
	}
	else {
		dev_err(adc->dev, "invalid ADC channel\n");
		goto err_io;
	}

	mutex_lock(&indio_dev->mlock);

	/* Start AIN0/AIN1 pin single-mode & 1-time conversion mode for ADC */
	rc5t619_write(adc->dev->parent, RC5T619_ADC_CNT3, adc_sel);
	usleep_range(200, 300);

	ret = rc5t619_read(adc->dev->parent, addr_H, &rawdata_H);
	ret = rc5t619_read(adc->dev->parent, addr_L, &rawdata_L);

	mutex_unlock(&indio_dev->mlock);

	if (ret)
		goto err_io;

	result = (unsigned int)(rawdata_H << 4) | (rawdata_L&0xf);
	*val = result;

	/* Stop AD conversion */
	rc5t619_write(adc->dev->parent, RC5T619_ADC_CNT3, 0x00);

	return IIO_VAL_INT;

err_io:
	return ret;
}

static const struct iio_info rc5t619_adc_info = {
	.read_raw = &rc5t619_adc_read_raw,
	.driver_module = THIS_MODULE,
};

#define RC5T619_CHAN(_id, _type) {				\
	.type = _type,					\
	.indexed = 1,					\
	.channel = RC5T619_##_id,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
		BIT(IIO_CHAN_INFO_SCALE),		\
	.datasheet_name = #_id,				\
}

static const struct iio_chan_spec rc5t619_adc_channels[] = {
	[RC5T619_ADC0] = RC5T619_CHAN(ADC0, IIO_VOLTAGE),
	[RC5T619_ADC1] = RC5T619_CHAN(ADC1, IIO_VOLTAGE),
};

static int rc5t619_adc_init(struct rc5t619_adc *adc)
{
	/* Set ADRQ=00 to stop ADC */
	rc5t619_write(adc->dev->parent, RC5T619_ADC_CNT3, 0x0);

	/* Enable AIN0L, AIN1L threshold Low interrupt */
	rc5t619_write(adc->dev->parent, RC5T619_INT_EN_ADC1, 0xC0);
	/* Enable AIN0H, AIN1H threshold High interrupt */
	rc5t619_write(adc->dev->parent, RC5T619_INT_EN_ADC2, 0xC0);
	/* Enable Single-mode interrupt */
	rc5t619_write(adc->dev->parent, RC5T619_INT_EN_ADC2, 0x01);

	/* Set ADC auto conversion interval 250ms */
	rc5t619_write(adc->dev->parent, RC5T619_ADC_CNT2, 0x0);

	/* Enable AIN0, AIN1 pin conversion in auto-ADC */
	rc5t619_write(adc->dev->parent, RC5T619_ADC_CNT1, 0xC0);

	/* Start auto-mode & average 4-time conversion mode for ADC */
	rc5t619_write(adc->dev->parent, RC5T619_ADC_CNT3, 0x17);
	/* Enable master ADC INT */
	rc5t619_set_bits(adc->dev->parent, RC5T619_INTC_INTEN, 0x08);

	return 0;	
}

static int rc5t619_adc_probe(struct platform_device *pdev)
{
	struct rc5t619_adc *adc;
	struct iio_dev *indio_dev;
	int ret;

	/* registering iio */
	indio_dev = iio_device_alloc(sizeof(*adc));
	if (!indio_dev) {
		dev_err(&pdev->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}

	adc = iio_priv(indio_dev);
	adc->dev = &pdev->dev;
	indio_dev->name = "rc5t619-adc";
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &rc5t619_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = rc5t619_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(rc5t619_adc_channels);

	mutex_init(&adc->lock);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "could not register iio (adc)");
		goto error;
	}

	platform_set_drvdata(pdev, indio_dev);

	rc5t619_adc_init(adc);

	return 0;

error:
	iio_device_free(indio_dev);
	return ret;
}

static int rc5t619_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);

	return 0;
}

static struct platform_driver rc5t619_adc_driver = {
	.probe = rc5t619_adc_probe,
	.remove = rc5t619_adc_remove,
	.driver = {
		.name = "rc5t619-adc",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(rc5t619_adc_driver);

MODULE_DESCRIPTION("Ricoh RC5T619 ADC Driver");
MODULE_AUTHOR("Kevin Kim");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rc5t619-adc");
