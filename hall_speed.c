#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#define DRIVER_NAME "halls"
#define DRIVER_PREFIX DRIVER_NAME ": "

#define HALLS_MINOR 0
#define HALLS_MAX_DEVICES 1

#define WHEEL_DIAMETER 6
#define MAGNET_NUMBER 2
#define MIN_SPEED 20

/* Change this line to use different GPIO number
 * 32 (PA0) + 8 (PA8) (J11.9 AT91SAM9G45-EKES)
 * DO from sensor point of view
 */
#define HALL_DO_GPIO_NUM 40
#define HALL_DO_GPIO_NAME "halls_do"

#define PI 314
#define PI_COEFFICIENT 100

#define SPEED_STR_BUFF_SIZE 12

static uint wheel_diameter = WHEEL_DIAMETER;
module_param(wheel_diameter, uint, S_IRUGO);
MODULE_PARM_DESC(wheel_diameter, "Wheel diameter in cm");

static uint magnet_number = MAGNET_NUMBER;
module_param(magnet_number, uint, S_IRUGO);
MODULE_PARM_DESC(magnet_number, "Number of magnets on wheel");

static uint min_speed = MIN_SPEED;
module_param(min_speed, uint, S_IRUGO);
MODULE_PARM_DESC(min_speed, "Minimun speed in cm/s");

static uint hall_do_gpio_num = HALL_DO_GPIO_NUM;
module_param(hall_do_gpio_num, uint, S_IRUGO);
MODULE_PARM_DESC(hall_do_gpio_num, "GPIO number to which Hall sensor digital "
	"output is connected");

struct halls {
	uint do_gpio_num;
	int do_gpio_irq;
	ktime_t t1, t2;
	unsigned int stop_time;
	struct timer_list stop_timer;
	struct class class;
	struct device *sysfs_dev;
	int major;
	struct cdev cdev;
	spinlock_t lock;
};

static struct halls halls;

static inline u32 get_speed(struct halls *hs)
{
	u32 t_diff;
	unsigned long flags;

	spin_lock_irqsave(&hs->lock, flags);

	t_diff = ktime_to_ns(hs->t1) && ktime_to_ns(hs->t2) ?
		ktime_to_us(ktime_sub(hs->t2, hs->t1)) : 0;

	spin_unlock_irqrestore(&hs->lock, flags);

	return t_diff ? PI * wheel_diameter * USEC_PER_SEC / PI_COEFFICIENT /
		magnet_number / t_diff : 0;
}

static inline u32 get_stop_detection_time(u32 min_speed)
{
	return PI * MSEC_PER_SEC * wheel_diameter / PI_COEFFICIENT /
		magnet_number /	min_speed;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
static ssize_t speed_value_read(struct class *class, char *buf)
#else
static ssize_t speed_value_read(struct class *class,
	struct class_attribute *attr, char *buf)
#endif
{
	struct halls *hs = container_of(class, struct halls, class);

	return sprintf(buf, "%d\n", get_speed(hs));
}

static struct class_attribute halls_class_attrs[] = {
	__ATTR(value, S_IRUGO, speed_value_read, NULL),
	__ATTR_NULL,
};

/* Interrupt handler of hall sensor DO signal */
static irqreturn_t halls_do_isr(int irq, void *data)
{
	struct halls *hs = data;

	/* When magnet goes past sensor the last one sets its DO to 0 */
	if (!gpio_get_value(hs->do_gpio_num)) {
		hs->t1 = hs->t2;
		hs->t2 = ktime_get();

		mod_timer(&hs->stop_timer, jiffies +
			msecs_to_jiffies(hs->stop_time));
	}

	return IRQ_HANDLED;
}

void stop_timer_callback(unsigned long data)
{
	unsigned long flags;
	static const ktime_t ktime_zero;
	struct halls *hs = (struct halls *)data;

	spin_lock_irqsave(&hs->lock, flags);
	hs->t1 = hs->t2 = ktime_zero;
	spin_unlock_irqrestore(&hs->lock, flags);
}

static int halls_open(struct inode *inode, struct file *filp)
{
	struct halls *hs = container_of(inode->i_cdev, struct halls, cdev);

	filp->private_data = hs;

	return 0;
}

static int halls_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t halls_read(struct file *filp, char __user *buf, size_t count,
	loff_t *f_pos)
{
	size_t max_len;
	char speed_str[SPEED_STR_BUFF_SIZE];
	struct halls *hs = filp->private_data;

	max_len = snprintf(speed_str, SPEED_STR_BUFF_SIZE, "%u\n",
		get_speed(hs));
	if (max_len > SPEED_STR_BUFF_SIZE)
		max_len = SPEED_STR_BUFF_SIZE;

	return simple_read_from_buffer(buf, count, f_pos, speed_str, max_len);
}

static struct file_operations halls_fops = {
	.owner   = THIS_MODULE,
	.open    = halls_open,
	.release = halls_release,
	.read    = halls_read,
};

static int check_params(void)
{
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
			"speed\n");
		return -1;
	}

	return 0;
}

static int halls_char_dev_create(void)
{
	dev_t dev;
	int ret;

	ret = alloc_chrdev_region(&dev, HALLS_MINOR, HALLS_MAX_DEVICES,
		DRIVER_NAME);
	if (ret < 0) {
		printk(KERN_ERR DRIVER_PREFIX "failed to alloc major number, "
			"ret %d\n", ret);
		return ret;
	}
	halls.major = MAJOR(dev);
	cdev_init(&halls.cdev, &halls_fops);
	halls.cdev.owner = THIS_MODULE;
	ret = cdev_add(&halls.cdev, dev, HALLS_MAX_DEVICES);
	if (ret) {
		printk(KERN_ERR DRIVER_PREFIX "failed to add char device, "
			"ret %d\n", ret);
		unregister_chrdev_region(dev, HALLS_MAX_DEVICES);
		return ret;
	}

	return 0;
}

static void halls_char_dev_destroy(void)
{
	cdev_del(&halls.cdev);
	unregister_chrdev_region(MKDEV(halls.major, HALLS_MINOR),
		HALLS_MAX_DEVICES);
}

static int halls_sysfs_create(void)
{
	int ret;

	halls.class.name = DRIVER_NAME;
	halls.class.owner = THIS_MODULE;
	halls.class.class_attrs = halls_class_attrs;
	if ((ret = class_register(&halls.class))) {
		printk(KERN_ERR DRIVER_PREFIX "failed to register class, "
			"ret %d\n", ret);
		return ret;
	}

	halls.sysfs_dev = device_create(&halls.class, NULL, MKDEV(halls.major,
		HALLS_MINOR), NULL, DRIVER_NAME "%d", HALLS_MINOR);
	if (IS_ERR(halls.sysfs_dev)) {
		printk(KERN_ERR DRIVER_PREFIX "failed to create sysfs device, "
			"ret %ld\n", PTR_ERR(halls.sysfs_dev));
		class_unregister(&halls.class);
		return PTR_ERR(halls.sysfs_dev);
	}

	return 0;
}

static void halls_sysfs_destroy(void)
{
	device_destroy(&halls.class, MKDEV(halls.major, HALLS_MINOR));
	class_unregister(&halls.class);
}

static int __init halls_init(void)
{
	int ret;

	if (check_params())
		return -1;

	if ((ret = halls_char_dev_create()))
		return ret;

	spin_lock_init(&halls.lock);

	if ((ret = halls_sysfs_create()))
		goto fail_sysfs_create;

	halls.do_gpio_num = hall_do_gpio_num;
	ret = gpio_request(halls.do_gpio_num, HALL_DO_GPIO_NAME);
	if (ret) {
		printk(KERN_ERR DRIVER_PREFIX "failed to request GPIO, ret"
			" %d\n", ret);
		goto fail_gpio_req;
	}

	ret = gpio_direction_input(halls.do_gpio_num);
	if (ret) {
		printk(KERN_ERR DRIVER_PREFIX "failed to set GPIO "
			"direction, ret %d\n", ret);
		goto fail_gpio_setup;
	}

	ret = gpio_to_irq(halls.do_gpio_num);
	if (ret < 0) {
		printk(KERN_ERR DRIVER_PREFIX "failed to get GPIO IRQ, "
			" %d\n", ret);
		goto fail_gpio_setup;
	} else
		halls.do_gpio_irq = ret;

	ret = request_irq(halls.do_gpio_irq, halls_do_isr,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
		IRQF_DISABLED |
#endif
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		HALL_DO_GPIO_NAME, &halls);
	if (ret) {
		printk(KERN_ERR DRIVER_PREFIX "failed to request IRQ, ret "
			"%d\n", ret);
		goto fail_gpio_setup;
	}

	/* If there wasn't signal from hall sensor during stop_time then wheel
	 * has been stopped and we need set speed value to 0. stop_time
	 * is calculated from minimal speed parameter. The higher minimal speed,
	 * the lower stop detection time.
	 */
	halls.stop_time = get_stop_detection_time(min_speed);
	setup_timer(&halls.stop_timer, stop_timer_callback,
		(unsigned long)&halls);
	ret = mod_timer(&halls.stop_timer, jiffies +
		msecs_to_jiffies(halls.stop_time));
	if (ret) {
		printk(KERN_ERR DRIVER_PREFIX "failed to setup stop timer "
			"%d\n", ret);
		goto fail_timer_setup;
	}

	return 0;

fail_timer_setup:
	free_irq(halls.do_gpio_irq, &halls);
fail_gpio_setup:
	gpio_free(halls.do_gpio_num);
fail_gpio_req:
	halls_sysfs_destroy();
fail_sysfs_create:
	halls_char_dev_destroy();
	return ret;
}

static void __exit halls_exit(void)
{
	del_timer(&halls.stop_timer);
	free_irq(halls.do_gpio_irq, &halls);
	gpio_free(halls.do_gpio_num);
	halls_sysfs_destroy();
	halls_char_dev_destroy();
}

module_init(halls_init);
module_exit(halls_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bogdan Bogush");
MODULE_DESCRIPTION("Hall effect speed sensor driver");
