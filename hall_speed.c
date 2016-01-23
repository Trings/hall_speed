#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/interrupt.h> 

#define DRIVER_NAME "halls"
#define DRIVER_PREFIX DRIVER_NAME ": "

#define WHEEL_DIAMETER 6
#define MAGNET_NUMBER 2
#define MIN_SPEED 5

/* Change this line to use different GPIO */
#define HALL_DO_GPIO_NUM 40 /* J11.9 -   PA8 */

#define PI 314
#define PI_COEFFICIENT 100

static uint wheel_diameter = WHEEL_DIAMETER;
module_param(wheel_diameter, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(wheel_diameter, "Wheel diameter in cm");

static uint magnet_number = MAGNET_NUMBER;
module_param(magnet_number, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(magnet_number, "Number of magnets on wheel");

static uint min_speed = MIN_SPEED;
module_param(min_speed, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(min_speed, "Minimun speed in cm/s");

static uint hall_do_gpio_num = HALL_DO_GPIO_NUM;
module_param(hall_do_gpio_num, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(hall_do_gpio_num, "GPIO number to which Hall sensor digital "
	"output is connected");

struct halls {
	int gpio_irq;
	ktime_t t1, t2;
	unsigned int stop_time;
	struct timer_list stop_timer;
	spinlock_t lock;
};

static struct halls halls;
 
/* Write in /sys/class/hall_speed/value */
static ssize_t hall_speed_value_write(struct class *class, const char *buf,
	size_t len)
{
	return len;
}

/* Read from /sys/class/hall_speed/value */
static ssize_t hall_speed_value_read(struct class *class, char *buf)
{
	unsigned long flags;
	u32 t_diff;
	u32 speed = 0;

	spin_lock_irqsave(&halls.lock, flags);

	t_diff = ktime_to_ns(halls.t1) && ktime_to_ns(halls.t2) ?
		ktime_to_us(ktime_sub(halls.t2, halls.t1)) : 0;

	spin_unlock_irqrestore(&halls.lock, flags);

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
	/* When magnet goes past sensor the last one sets its DO to 0 */	
	if (!gpio_get_value(hall_do_gpio_num)) {
		halls.t1 = halls.t2;
		halls.t2 = ktime_get();

		mod_timer(&halls.stop_timer, jiffies +
			msecs_to_jiffies(halls.stop_time));
	}

	return IRQ_HANDLED;
}

void stop_timer_callback(unsigned long data)
{
	unsigned long flags;
	static const ktime_t ktime_zero;

	spin_lock_irqsave(&halls.lock, flags);
	halls.t1 = halls.t2 = ktime_zero;
	spin_unlock_irqrestore(&halls.lock, flags);
}

static int hall_speed_init(void)
{
	int ret;

	if (!wheel_diameter) {
		printk(KERN_ERR DRIVER_PREFIX "wrong value of wheel "
			"diameter\n");
		return -1;
	}

	if (!magnet_number) {
		printk(KERN_ERR DRIVER_PREFIX "wrong value of magnets "
			"number\n");
		return -1;
	}

	if (!min_speed) {
		printk(KERN_ERR DRIVER_PREFIX "wrong value of minimum "
			"speed \n");
		return -1;
	}

	spin_lock_init(&halls.lock);

	if (class_register(&hall_speed_class) < 0)
		return -1;

	ret = gpio_request(hall_do_gpio_num, "HALL_DO");
	if (ret) {
		printk(KERN_ERR DRIVER_PREFIX "failed to request GPIO, ret"
			" %d\n", ret);
		goto fail_gpio_req;
	}

	ret = gpio_direction_input(hall_do_gpio_num);
	if (ret) {
		printk(KERN_ERR DRIVER_PREFIX "failed to set GPIO "
			"direction, ret %d\n", ret);
		goto fail_gpio_setup;
	}

	ret = gpio_to_irq(hall_do_gpio_num);
	if (ret < 0) {
		printk(KERN_ERR DRIVER_PREFIX "failed to get GPIO IRQ, "
			" %d\n", ret);
		goto fail_gpio_setup;
	} else
		halls.gpio_irq = ret;

	ret = request_irq(halls.gpio_irq, gpio_isr, IRQF_TRIGGER_FALLING |
		IRQF_TRIGGER_RISING | IRQF_DISABLED, "hall.do", NULL);
	if(ret) {
		printk(KERN_ERR DRIVER_PREFIX "failed to request IRQ, ret "
			"%d\n", ret);
		goto fail_gpio_setup;
	}

	/* If there wasn't signal from hall sensor during stop_time then wheel
	 * has been stopped and we need set speed value to 0. stop_time
	 * is calculated from minimal speed parameter. The higher minimal speed,
	 * the lower stop detection time.
	 */
	halls.stop_time = PI * MSEC_PER_SEC * wheel_diameter / PI_COEFFICIENT /
		magnet_number /	min_speed;
	setup_timer(&halls.stop_timer, stop_timer_callback, 0);
	ret = mod_timer(&halls.stop_timer, jiffies +
		msecs_to_jiffies(halls.stop_time));
	if (ret) {
		printk(KERN_ERR DRIVER_PREFIX "failed to setup stop timer "
			"%d\n", ret);
		goto fail_timer_setup;
	}

	return 0;

fail_timer_setup:
	free_irq(halls.gpio_irq, NULL);
fail_gpio_setup:
	gpio_free(hall_do_gpio_num);
fail_gpio_req:
	class_unregister(&hall_speed_class);
	return -1;
}
 
static void hall_speed_exit(void)
{
	del_timer(&halls.stop_timer);
	free_irq(halls.gpio_irq, NULL);
	gpio_free(hall_do_gpio_num);
	class_unregister(&hall_speed_class);
}
 
module_init(hall_speed_init);
module_exit(hall_speed_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bogdan Bogush");
MODULE_DESCRIPTION("Hall effect speed sensor driver");
