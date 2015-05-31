#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/interrupt.h> 

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bogdan Bogush");
MODULE_DESCRIPTION("Driver for hall speed sensor driver");

#define HALL_SPEED_DRIVER "Hall speed driver: "

/* Change this line to use different GPIO */
#define HALL_DO	40 /* J11.9 -   PA8 */

#define PI 314
#define PI_COEFFICIENT 100

static int gpio_irq = -1;
static ktime_t t1, t2;
static struct timer_list stop_timer;

static uint wheel_diameter = 6;
module_param(wheel_diameter, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(wheel_diameter, "Wheel diameter in cm");

static uint magnet_number = 2;
module_param(magnet_number, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(mugnet_number, "Number of magnets on wheel");

static uint min_speed = 5;
module_param(min_speed, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(stop_time, "Minimun speed in cm/s");
 
/* Write in /sys/class/hall_speed/value */
static ssize_t hall_speed_value_write(struct class *class, const char *buf,
	size_t len) 
{
	return len;
}

/* Read from /sys/class/hall_speed/value */
static ssize_t hall_speed_value_read(struct class *class, char *buf)
{
	u32 speed = 0;
	u32 t_diff = ktime_to_ns(t1) && ktime_to_ns(t2) ?
		ktime_to_us(ktime_sub(t2, t1)) : 0;

	if (t_diff) {
		speed = PI * wheel_diameter * USEC_PER_SEC / PI_COEFFICIENT /
			magnet_number / t_diff;
	}

	return sprintf(buf, "%d\n", speed);
}

/* Sysfs definitions for hall speed class */
static struct class_attribute hall_speed_class_attrs[] = {
	__ATTR(value, S_IRUGO | S_IWUSR, hall_speed_value_read,
		hall_speed_value_write),
	__ATTR_NULL,
};

/* Name of directory created in /sys/class */
static struct class hall_speed_class = {
	.name =			"hall_speed",
	.owner =		THIS_MODULE,
	.class_attrs =	hall_speed_class_attrs,
};

/* Interrupt handler on HALL DO signal */
static irqreturn_t gpio_isr(int irq, void *data)
{
	if (!gpio_get_value(HALL_DO)) {
		t1 = t2;
		t2 = ktime_get();

		mod_timer(&stop_timer, jiffies + msecs_to_jiffies(PI *
			MSEC_PER_SEC * wheel_diameter / PI_COEFFICIENT /
			magnet_number / min_speed));
	}

	return IRQ_HANDLED;
}

void stop_timer_callback(unsigned long data)
{
	static const ktime_t ktime_zero;

	t1 = t2 = ktime_zero;
}

static int hall_speed_init(void)
{
	int ret;

	printk(KERN_INFO HALL_SPEED_DRIVER "initializing.\n");

	if (!wheel_diameter) {
		printk(KERN_INFO HALL_SPEED_DRIVER "wrong value of wheel "
			"diameter\n");
		return -1;
	}

	if (!magnet_number) {
		printk(KERN_INFO HALL_SPEED_DRIVER "wrong value of magnets "
			"number\n");
		return -1;
	}

	if (!min_speed) {
		printk(KERN_INFO HALL_SPEED_DRIVER "wrong value of minimum "
			"speed \n");
		return -1;
	}

	if (class_register(&hall_speed_class) < 0)
		return -1;

	ret = gpio_request(HALL_DO, "HALL_DO");
	if (ret) {
		printk(KERN_INFO HALL_SPEED_DRIVER "failed to request GPIO, ret"
			" %d\n", ret);
		goto fail_gpio_req;
	}

	ret = gpio_direction_input(HALL_DO);
	if (ret) {
		printk(KERN_INFO HALL_SPEED_DRIVER "failed to set GPIO "
			"direction, ret %d\n", ret);
		goto fail_gpio_setup;
	}

	ret = gpio_to_irq(HALL_DO);
	if (ret < 0) {
		printk(KERN_INFO HALL_SPEED_DRIVER "failed to get GPIO IRQ, "
			" %d\n", ret);
		goto fail_gpio_setup;
	} else {
		gpio_irq = ret;
	}

	ret = request_irq(gpio_irq, gpio_isr, IRQF_TRIGGER_FALLING |
		IRQF_TRIGGER_RISING | IRQF_DISABLED , "hall.do", NULL);
	if(ret) {
		printk(KERN_ERR HALL_SPEED_DRIVER "failed to request IRQ, ret "
			"%d\n", ret);
		goto fail_gpio_setup;
	}

	setup_timer(&stop_timer, stop_timer_callback, 0);
	ret = mod_timer(&stop_timer, jiffies + msecs_to_jiffies(PI *
		MSEC_PER_SEC * wheel_diameter / PI_COEFFICIENT / magnet_number /
		min_speed));
	if (ret) {
		printk(KERN_ERR HALL_SPEED_DRIVER "failed to setup stop timer "
			"%d\n", ret);
		goto fail_timer_setup;
	}

	return 0;

fail_timer_setup:
	free_irq(gpio_irq, NULL);
fail_gpio_setup:
	gpio_free(HALL_DO);
fail_gpio_req:
	class_unregister(&hall_speed_class);
	return -1;
}
 
static void hall_speed_exit(void)
{
	del_timer(&stop_timer);
	free_irq(gpio_irq, NULL);
	gpio_free(HALL_DO);
	class_unregister(&hall_speed_class);

	printk(KERN_INFO "Hall speed driver exit.\n");
}
 
module_init(hall_speed_init);
module_exit(hall_speed_exit);
