/*
 * Copyright (c) 2011, NVIDIA Corporation.
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
 */

#include <linux/regulator/consumer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/pwm.h>
#include <linux/tegra_vibrator_enr.h>
#include "../staging/android/timed_output.h"

#include <linux/sched.h>

#define PLAYBACK_PERIOD_US 		50000
#define ZERO_DUTY_US 			25000

#define VIBE_DEBUG				0
#if VIBE_DEBUG
	#define D(x...) printk(KERN_DEBUG "[VIB]" ":" x)
	#define I(x...) printk(KERN_INFO "[VIB]" ":" x)
#else
	#define D(x...)
	#define I(x...)
#endif

struct vibrator {
	struct timed_output_dev timed_dev;
	struct vibrator_platform_data *pdata;
	int pwm_duty;
	struct work_struct work;
	struct hrtimer vib_timer;
};

struct vibrator *g_vib;
static struct regulator *regulator;
static struct workqueue_struct *vib_work_queue;

static enum hrtimer_restart vib_timer_func(struct hrtimer *timer)
{
	struct vibrator *vib = container_of(timer, struct vibrator, vib_timer);

	int rc;
	I(" %s +++\n", __func__);
	D("[VIB] vib_timer_func, vibration stop\n");
	rc = gpio_direction_output(vib->pdata->ena_gpio, 0);
	if(rc<0){
		pr_err("[VIB] set gpio output direction fail in timer function\n");
	}
	queue_work(vib_work_queue, &vib->work);
	I(" %s ---\n", __func__);
	return HRTIMER_NORESTART;
}

static void vibrator_start(struct vibrator *vib)
{
	int ret, rc = 0;
	I(" %s +++\n", __func__);

	ret = regulator_is_enabled(regulator);
	if (ret > 0)
		regulator_disable(regulator);
	rc = pwm_config(vib->pdata->pwm_data.pwm_dev, g_vib->pwm_duty, PLAYBACK_PERIOD_US);
	if (rc < 0) {
		printk("[VIB][START]: pwm config fails\n");
	}
	pwm_enable(vib->pdata->pwm_data.pwm_dev);
	gpio_direction_output(vib->pdata->ena_gpio, 1);
	regulator_enable(regulator);
	I(" %s ---\n", __func__);
}

static void vibrator_stop(struct vibrator *vib)
{
	int ret, rc = 0;;

	I(" %s +++\n", __func__);

	rc = pwm_config(vib->pdata->pwm_data.pwm_dev, ZERO_DUTY_US, PLAYBACK_PERIOD_US);
	if (rc < 0) {
		printk("[VIB][STOP]: pwm config fails\n");
	}
	pwm_enable(vib->pdata->pwm_data.pwm_dev);
	gpio_direction_output(vib->pdata->ena_gpio, 0);
	pwm_disable(vib->pdata->pwm_data.pwm_dev);
	ret = regulator_is_enabled(regulator);
	if (ret > 0)
		regulator_disable(regulator);
	I(" %s ---\n", __func__);
}

/*
 * Timeout value can be changed from sysfs entry
 * created by timed_output_dev.
 * echo 100 > /sys/class/timed_output/vibrator/enable
 */
static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct vibrator *vib = container_of(dev, struct vibrator, timed_dev);

	I(" %s +++\n", __func__);
	if (value < 0)
		return;
	if (value) {
		D(KERN_INFO "[VIB] vibration enable and duration time %d ms:%s(parent:%s): tgid=%d\n", value,current->comm, current->parent->comm, current->tgid);

		vibrator_start(vib);
		hrtimer_start(&vib->vib_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	} else {
		D("[VIB] vibration disable.\n");
		vibrator_stop(vib);
	}
	I(" %s ---\n", __func__);
}

static void vib_work_func(struct work_struct *work)
{
	struct vibrator *vib = container_of(work, struct vibrator, work);
	I(" %s +++\n", __func__);
	vibrator_stop(vib);
	I(" %s ---\n", __func__);
}

/*
 * Timeout value can be read from sysfs entry
 * created by timed_output_dev.
 * cat /sys/class/timed_output/vibrator/enable
 */
static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct vibrator *vib = container_of(dev, struct vibrator, timed_dev);
	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return (int)ktime_to_us(r);
	} else
		return 0;
}

static ssize_t show_dutycycle(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n",(int)g_vib->pwm_duty);
}

static ssize_t set_dutycycle(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	long val = simple_strtol(buf, NULL, 10);
	if(val>PLAYBACK_PERIOD_US)
		val = PLAYBACK_PERIOD_US;
	if(val<0)
		val=0;
	g_vib->pwm_duty = (int)val;
	I("set duty cycle:%d \n",g_vib->pwm_duty);
    return count;
}

static DEVICE_ATTR(dutycycle, 0644, show_dutycycle, set_dutycycle);


static int vibrator_probe(struct platform_device *pdev)
{
	struct vibrator_platform_data *pdata = pdev->dev.platform_data;
	struct vibrator *vib;
	int rc=0;

	printk("[VIB][PROBE] vibrator_probe +++\n");
	if (!pdata)
		return -EINVAL;
	vib=kzalloc(sizeof(struct vibrator), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;
	vib->pdata=kzalloc(sizeof(struct vibrator_platform_data), GFP_KERNEL);
	if(!(vib->pdata)){
		rc = -ENOMEM;
		pr_err("[VIB] %s: failed on allocate pm8xxx_pwm_data memory\n", __func__);
		goto err_platform_data_allocate;
	}
	
	vib->pdata->pwm_gpio = pdata->pwm_gpio;
	vib->pdata->ena_gpio = pdata->ena_gpio;
	vib->pdata->pwm_data.name = pdata->pwm_data.name;
	vib->pdata->pwm_data.bank = pdata->pwm_data.bank;
	vib->pwm_duty = pdata->duty;
	INIT_WORK(&vib->work, vib_work_func);
	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = vib_timer_func;

	vib->pdata->pwm_data.pwm_dev = pwm_request(vib->pdata->pwm_data.bank, "vibrator");
	rc = gpio_request(vib->pdata->pwm_gpio, "vibrator_pwm");
	if (rc) {
			rc = -ENOMEM;
			pr_err("[VIB] request pwm gpio fail\n");
		}
	rc = gpio_request(vib->pdata->ena_gpio, "vibrator_ena");
	if (rc) {
			rc = -ENOMEM;
			pr_err("[VIB] request ena gpio fail\n");
		}
	rc = gpio_direction_output(vib->pdata->ena_gpio,0);
	if(rc<0){
		pr_err("[VIB] set gpio output direction fail\n");
		goto err_ena_output;
	}
	tegra_gpio_enable(vib->pdata->ena_gpio);
	regulator = regulator_get(NULL, "v_vib_3v");
	if( (regulator==NULL) | (IS_ERR(regulator)) )
		pr_err("[VIB] Fail to get regulator: v_vib_3v");

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = vibrator_get_time;
	vib->timed_dev.enable = vibrator_enable;

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc){
			pr_err("[VIB] timed_output_dev_register failed.\n");
	}
	rc = device_create_file(vib->timed_dev.dev, &dev_attr_dutycycle);
	if (rc){
		pr_err("[VIB] fail to add dutycycle attribute.\n");
		goto err_create_file;
	}
	vib_work_queue = create_workqueue("vib");
	if (!vib_work_queue)
		goto err_create_file;

	g_vib = vib;
	g_vib->pwm_duty = pdata->duty;
	platform_set_drvdata(pdev, vib);

	printk("[VIB][PROBE] vibrator_probe ---\n");
	return 0;

err_create_file:
	timed_output_dev_unregister(&vib->timed_dev);
err_ena_output:
	gpio_free(vib->pdata->ena_gpio);
	gpio_free(vib->pdata->pwm_gpio);
err_platform_data_allocate:
	kfree(vib);
	pr_info(KERN_DEBUG "%s error\n",__func__);
	return rc;
}

static int vibrator_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct vibrator *vib = platform_get_drvdata(pdev);

	D("[VIB][SUSPEND] vibrator_suspend +++\n");
	vibrator_stop(vib);
	hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
	gpio_direction_output(vib->pdata->pwm_gpio, 0);
	tegra_gpio_enable(vib->pdata->pwm_gpio);
	D("[VIB][SUSPEND] vibrator_suspend ---\n");
	return 0;
}

static int vibrator_resume(struct platform_device *pdev)
{
	struct vibrator *vib = platform_get_drvdata(pdev);

	D("[VIB][RESUME] vibrator_resume +++\n");
	tegra_gpio_disable(vib->pdata->pwm_gpio);
	vib->pwm_duty = g_vib->pwm_duty;
	D("[VIB][RESUME] vibrator_resume ---\n");
	return 0;
}

static int __devexit vibrator_remove(struct platform_device *pdev)
{
	struct vibrator *vib = platform_get_drvdata(pdev);
	
	cancel_work_sync(&vib->work);
	hrtimer_cancel(&vib->vib_timer);
	gpio_free(vib->pdata->pwm_gpio);
	gpio_free(vib->pdata->ena_gpio);
	kfree(vib->pdata);
	kfree(vib);
	timed_output_dev_unregister(&vib->timed_dev);
	regulator_put(regulator);
	destroy_workqueue(vib_work_queue);
	return 0;
}

static struct platform_driver vibrator_driver = {
	.probe		= vibrator_probe,
	.remove		= __devexit_p(vibrator_remove),
	.suspend	= vibrator_suspend,
	.resume		= vibrator_resume,
	.driver		= {
		.name	= VIBRATOR_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init vibrator_init(void)
{
	return platform_driver_register(&vibrator_driver);
}
module_init(vibrator_init);

static void __exit vibrator_exit(void)
{
	platform_driver_unregister(&vibrator_driver);
}
module_exit(vibrator_exit);

MODULE_DESCRIPTION("tegra vibrator driver");
