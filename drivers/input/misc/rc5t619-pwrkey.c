/*
* driver/input/misc/rc5t619-pwrkey.c
*
* Power Key driver for RICOH RC5T619 power management chip.
*
* Copyright (C) 2012-2013 RICOH COMPANY,LTD
*
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <linux/mfd/rc5t619.h>

#define RC5T619_ONKEY_TRIGGER_LEVEL	0
#define RC5T619_ONKEY_OFF_IRQ		0

struct rc5t619_pwrkey {
	struct device *dev;
	struct input_dev *pwr;
	#if RC5T619_ONKEY_TRIGGER_LEVEL
		struct timer_list timer;
	#endif
	struct workqueue_struct *workqueue;
	struct work_struct work;
	unsigned long delay;
	int key_irq;
	bool pressed_first;
	struct rc5t619_pwrkey_platform_data *pdata;
	spinlock_t lock;
};

struct rc5t619_pwrkey *g_pwrkey;

#if RC5T619_ONKEY_TRIGGER_LEVEL
void rc5t619_pwrkey_timer(unsigned long t)
{
	queue_work(g_pwrkey->workqueue, &g_pwrkey->work);
}
#endif

static void rc5t619_irq_work(struct work_struct *work)
{
	/* unsigned long flags; */
	uint8_t val;

	//spin_lock_irqsave(&g_pwrkey->lock, flags);

	if(pwrkey_wakeup){
		pwrkey_wakeup = 0;
		input_event(g_pwrkey->pwr, EV_KEY, KEY_POWER, 1);
		input_event(g_pwrkey->pwr, EV_SYN, 0, 0);
		input_event(g_pwrkey->pwr, EV_KEY, KEY_POWER, 0);
		input_event(g_pwrkey->pwr, EV_SYN, 0, 0);
		
		return;
	}
	rc5t619_read(g_pwrkey->dev->parent, RC5T619_INT_MON_SYS, &val);
	dev_dbg(g_pwrkey->dev, "pwrkey is pressed?(0x%x): 0x%x\n",
						RC5T619_INT_MON_SYS, val);
	val &= 0x1;
	if(val){
		#if (RC5T619_ONKEY_TRIGGER_LEVEL)
		g_pwrkey->timer.expires = jiffies + g_pwrkey->delay;
		dd_timer(&g_pwrkey->timer);
		#endif
		if (!g_pwrkey->pressed_first){
			g_pwrkey->pressed_first = true;
			//input_report_key(g_pwrkey->pwr, KEY_POWER, 1);
			//input_sync(g_pwrkey->pwr);
			input_event(g_pwrkey->pwr, EV_KEY, KEY_POWER, 1);
			input_event(g_pwrkey->pwr, EV_SYN, 0, 0);
		}
	} else {
		if (g_pwrkey->pressed_first) {
			/* input_report_key(g_pwrkey->pwr, KEY_POWER, 0); */
			/* input_sync(g_pwrkey->pwr); */
			input_event(g_pwrkey->pwr, EV_KEY, KEY_POWER, 0);
			input_event(g_pwrkey->pwr, EV_SYN, 0, 0);
		}
		g_pwrkey->pressed_first = false;
	}

	/* spin_unlock_irqrestore(&g_pwrkey->lock, flags); */
}

static irqreturn_t pwrkey_irq(int irq, void *_pwrkey)
{
	#if (RC5T619_ONKEY_TRIGGER_LEVEL)
	g_pwrkey->timer.expires = jiffies + g_pwrkey->delay;
	add_timer(&g_pwrkey->timer);
	#else
	queue_work(g_pwrkey->workqueue, &g_pwrkey->work);
	#endif
	return IRQ_HANDLED;
}

#if RC5T619_ONKEY_OFF_IRQ
static irqreturn_t pwrkey_irq_off(int irq, void *_pwrkey)
{
	dev_warn(g_pwrkey->dev, "ONKEY is pressed long time!\n");
	return IRQ_HANDLED;
}
#endif

static int rc5t619_pwrkey_probe(struct platform_device *pdev)
{
	struct input_dev *pwr;
	int key_irq;
	int err;
	struct rc5t619_pwrkey *pwrkey;
	struct rc5t619_pwrkey_platform_data *pdata = pdev->dev.platform_data;
	uint8_t val;

	if (!pdata) {
		dev_err(&pdev->dev, "power key platform data not supplied\n");
		return -EINVAL;
	}
	key_irq = pdata->irq;
	pwrkey = kzalloc(sizeof(*pwrkey), GFP_KERNEL);
	if (!pwrkey)
		return -ENOMEM;

	pwrkey->dev = &pdev->dev;
	pwrkey->pdata = pdata;
	pwrkey->pressed_first = false;
	pwrkey->delay = HZ / 1000 * pdata->delay_ms;
	g_pwrkey = pwrkey;
	pwr = input_allocate_device();
	if (!pwr) {
		dev_dbg(&pdev->dev, "Can't allocate power button\n");
		err = -ENOMEM;
		goto free_pwrkey;
	}
	input_set_capability(pwr, EV_KEY, KEY_POWER);
	pwr->name = "rc5t619_pwrkey";
	pwr->phys = "rc5t619_pwrkey/input0";
	pwr->dev.parent = &pdev->dev;

	#if RC5T619_ONKEY_TRIGGER_LEVEL
	init_timer(&pwrkey->timer);
	pwrkey->timer.function = rc5t619_pwrkey_timer;
	#endif

	spin_lock_init(&pwrkey->lock);
	err = input_register_device(pwr);
	if (err) {
		dev_dbg(&pdev->dev, "Can't register power key: %d\n", err);
		goto free_input_dev;
	}
	pwrkey->key_irq = key_irq;
	pwrkey->pwr = pwr;
	platform_set_drvdata(pdev, pwrkey);

	/* Check if power-key is pressed at boot up */
	err = rc5t619_read(pwrkey->dev->parent, RC5T619_INT_MON_SYS, &val);
	if (err < 0) {
		dev_err(&pdev->dev, "Key-press status at boot failed rc=%d\n",
									 err);
		goto unreg_input_dev;
	}
	val &= 0x1;
	if (val) {
		input_report_key(pwrkey->pwr, KEY_POWER, 1);
		input_sync(pwrkey->pwr);
		pwrkey->pressed_first = true;
	}

	#if !(RC5T619_ONKEY_TRIGGER_LEVEL)
		/* trigger both edge */
		rc5t619_set_bits(pwrkey->dev->parent, RC5T619_PWR_IRSEL, 0x1);
	#endif

	err = request_threaded_irq(key_irq, NULL, pwrkey_irq,
		IRQF_ONESHOT, "rc5t619_pwrkey", pwrkey);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't get %d IRQ for pwrkey: %d\n",
								key_irq, err);
		goto unreg_input_dev;
	}

	#if RC5T619_ONKEY_OFF_IRQ
	err = request_threaded_irq(key_irq + RC5T619_IRQ_ONKEY_OFF, NULL,
						pwrkey_irq_off, IRQF_ONESHOT,
						"rc5t619_pwrkey_off", pwrkey);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't get %d IRQ for pwrkey: %d\n",
			key_irq + RC5T619_IRQ_ONKEY_OFF, err);
		free_irq(key_irq, pwrkey);
		goto unreg_input_dev;
	}
	#endif

	pwrkey->workqueue = create_singlethread_workqueue("rc5t619_pwrkey");
	INIT_WORK(&pwrkey->work, rc5t619_irq_work);

	/* Enable power key IRQ */
	/* trigger both edge */
	rc5t619_set_bits(pwrkey->dev->parent, RC5T619_PWR_IRSEL, 0x1);
	/* Enable system interrupt */
	rc5t619_set_bits(pwrkey->dev->parent, RC5T619_INTC_INTEN, 0x1);
	/* Enable power-on interrupt */
	rc5t619_set_bits(pwrkey->dev->parent, RC5T619_INT_EN_SYS, 0x1);
	return 0;

unreg_input_dev:
	input_unregister_device(pwr);
	pwr = NULL;

free_input_dev:
	input_free_device(pwr);
	free_pwrkey:
	kfree(pwrkey);

	return err;
}

static int rc5t619_pwrkey_remove(struct platform_device *pdev)
{
	struct rc5t619_pwrkey *pwrkey = platform_get_drvdata(pdev);

	flush_workqueue(pwrkey->workqueue);
	destroy_workqueue(pwrkey->workqueue);
	free_irq(pwrkey->key_irq, pwrkey);
	input_unregister_device(pwrkey->pwr);
	kfree(pwrkey);

	return 0;
}

#ifdef CONFIG_PM
static int rc5t619_pwrkey_suspend(struct device *dev)
{
	struct rc5t619_pwrkey *info = dev_get_drvdata(dev);

	if (info->key_irq)
		disable_irq(info->key_irq);
	cancel_work_sync(&info->work);
	flush_workqueue(info->workqueue);

	return 0;
}

static int rc5t619_pwrkey_resume(struct device *dev)
{
	struct rc5t619_pwrkey *info = dev_get_drvdata(dev);

	queue_work(info->workqueue, &info->work);
	if (info->key_irq)
		enable_irq(info->key_irq);

	return 0;
}

static const struct dev_pm_ops rc5t619_pwrkey_pm_ops = {
	.suspend	= rc5t619_pwrkey_suspend,
	.resume		= rc5t619_pwrkey_resume,
};
#endif

static struct platform_driver rc5t619_pwrkey_driver = {
	.probe = rc5t619_pwrkey_probe,
	.remove = rc5t619_pwrkey_remove,
	.driver = {
		.name = "rc5t619-pwrkey",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &rc5t619_pwrkey_pm_ops,
#endif
	},
};

static int __init rc5t619_pwrkey_init(void)
{
	return platform_driver_register(&rc5t619_pwrkey_driver);
}
subsys_initcall_sync(rc5t619_pwrkey_init);

static void __exit rc5t619_pwrkey_exit(void)
{
	platform_driver_unregister(&rc5t619_pwrkey_driver);
}
module_exit(rc5t619_pwrkey_exit);


MODULE_ALIAS("platform:rc5t619-pwrkey");
MODULE_AUTHOR("zhangqing <zhangqing@rock-chips.com>");
MODULE_DESCRIPTION("rc5t619 Power Key");
MODULE_LICENSE("GPL v2");