/*
 * ASUS Lid driver.
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio_event.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>

#define DRV_NAME		"asustek_lid"
#define LID_DEVICE_NAME		"lid_input"
#define LID_PHYS		"/dev/input/lid_indev"
#define CONVERSION_TIME_MS	50

static void lid_report_function(struct work_struct *work);
static ssize_t show_lid_status(struct device *dev,
		struct device_attribute *attr, char *buf);

struct asustek_lid_drvdata {
	struct input_dev *input;
	struct pinctrl *key_pinctrl;
	struct delayed_work dwork;
	int gpio;
	int active_low;
	int wakeup;
	unsigned int irq;
};

static DEVICE_ATTR(lid_status, S_IRUGO, show_lid_status, NULL);

static struct attribute *lid_attrs[] = {
	&dev_attr_lid_status.attr,
	NULL
};

static struct attribute_group lid_attr_group = {
	.attrs = lid_attrs,
};

static ssize_t show_lid_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct asustek_lid_drvdata *ddata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			!!gpio_get_value_cansleep(ddata->gpio));
}

static irqreturn_t lid_interrupt_handler(int irq, void *dev_id)
{
	struct asustek_lid_drvdata *ddata = dev_id;

	WARN_ONCE(irq != ddata->irq, "lid_interrupt failed\n");

	schedule_delayed_work(&ddata->dwork,
			msecs_to_jiffies(CONVERSION_TIME_MS));

	return IRQ_HANDLED;
}

static void lid_report_function(struct work_struct *work)
{
	struct asustek_lid_drvdata *ddata =
		container_of(work, struct asustek_lid_drvdata, dwork.work);
	int value;

	value = !!gpio_get_value_cansleep(ddata->gpio) ^ ddata->active_low;

	//input_report_switch(ddata->input, SW_LID, value);
	//input_sync(ddata->input);

	//pr_info("[lid] %s : SW_LID report value = %d\n",  __func__, value);
}

/* translate openfirmware node properties */
static int __init asustek_lid_get_devtree(struct device *dev)
{
	struct asustek_lid_drvdata *ddata = dev_get_drvdata(dev);
	struct device_node *node;
	enum of_gpio_flags flags;

	printk("[lid] %s \n", __func__);
	node = dev->of_node;
	if (!node)
		return -ENODEV;

	if (!of_find_property(node, "gpios", NULL)) {
		static const char dt_get_err[] __initconst =
					"lid sensor without gpios\n";
		dev_err(dev, dt_get_err);
		return -EINVAL;
	}

	ddata->gpio = of_get_gpio_flags(node, 0, &flags);
	ddata->active_low = flags & OF_GPIO_ACTIVE_LOW;
	ddata->wakeup = !!of_get_property(node, "asustek_lid,wakeup", NULL);

	return 0;
}

static int asustek_lid_pinctrl_configure(struct pinctrl *key_pinctrl,
								bool active)
{
	struct pinctrl_state *set_state;

	int retval;

	printk("[lid] %s \n", __func__);
	if (active) {
		set_state = pinctrl_lookup_state(key_pinctrl,
						"asustek_lid_active");
		if (IS_ERR(set_state)) {
			pr_err("Can't get asustek_lid pinctrl active state\n");
			return PTR_ERR(set_state);
		}
	} else {
		set_state = pinctrl_lookup_state(key_pinctrl,
						"asustek_lid_suspend");
		if (IS_ERR(set_state)) {
			pr_err("Can't get asustek_lid pinctrl sleep state\n");
			return PTR_ERR(set_state);
		}
	}
	retval = pinctrl_select_state(key_pinctrl, set_state);

	if (retval) {
		pr_err("Can't set asustek_lid pinctrl state\n");
		return retval;
	}
	return 0;
}

static int __init lid_driver_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct device *dev = &pdev->dev;
	struct asustek_lid_drvdata *ddata;
	struct input_dev *input;
	static const char lid_probe[] __initconst = "ASUSTek: %s\n";

	printk("[lid] %s : Start !!!\n", __func__);
	if (!pdev)
		return -EINVAL;

	dev_info(dev, lid_probe, __func__);

	ddata = devm_kzalloc(dev, sizeof(struct asustek_lid_drvdata),
							GFP_KERNEL);
	input = devm_input_allocate_device(dev);
	if (!ddata || !input) {
		static const char errmsg[] __initconst =
					"Failed to allocate state\n";
		dev_err(dev, errmsg);
		return -ENOMEM;
	}

	ddata->input = input;
	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = LID_DEVICE_NAME;
	input->phys = LID_PHYS;
	set_bit(EV_SW, input->evbit);
	set_bit(SW_LID, input->swbit);
	printk("[lid] %s : set ket capabilities\n", __func__);

	/* Get pinctrl if target uses pinctrl */
	ddata->key_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ddata->key_pinctrl)) {
		if (PTR_ERR(ddata->key_pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_dbg(dev, "Target does not use pinctrl\n");
		ddata->key_pinctrl = NULL;
	}

	if (ddata->key_pinctrl) {
		dev_dbg(dev, "Target uses pinctrl\n");
		ret = asustek_lid_pinctrl_configure(ddata->key_pinctrl, true);
		if (ret) {
			dev_err(dev, "cannot set pinctrl active state\n");
			return ret;
		}
	}

	ret = input_register_device(input);
	if (ret) {
		static const char errmsg[] __initconst =
				"Unable to register input device, error: %d\n";
		dev_err(dev, errmsg, ret);
		return ret;
	}
	printk("[lid] %s : input register ok\n", __func__);

	ret = sysfs_create_group(&pdev->dev.kobj, &lid_attr_group);
	if (ret) {
		static const char errmsg[] __initconst =
				"Unable to create sysfs, error: %d\n";
		dev_err(dev, errmsg, ret);
		return ret;
	}
	printk("[lid] %s : create sysfs ok\n", __func__);

	INIT_DELAYED_WORK(&ddata->dwork, lid_report_function);

	asustek_lid_get_devtree(dev);

	if (!gpio_is_valid(ddata->gpio)) {
		static const char errmsg[] __initconst =
					"Invalid GPIO %d\n";
		dev_err(dev, errmsg, ddata->gpio);
		goto fail_remove_group;
	}

	ret = devm_gpio_request(dev, ddata->gpio, DRV_NAME);
	if (ret < 0) {
		static const char errmsg[] __initconst =
				"Failed to request GPIO %d\n";
		dev_err(dev, errmsg, ddata->gpio);
		goto fail_remove_group;
	}
	printk("[lid] %s : request gpio-irq ok\n", __func__);

	ret = gpio_direction_input(ddata->gpio);
	if (ret < 0) {
		static const char errmsg[] __initconst =
				"Failed to configure direction for GPIO %d\n";
		dev_err(dev, errmsg, ddata->gpio);
		goto fail_remove_group;
	}

	irq = gpio_to_irq(ddata->gpio);
	ddata->irq = irq;
	if (irq < 0) {
		static const char errmsg[] __initconst =
				"Unable to get irq number for GPIO %d\n";
		dev_err(dev, errmsg, ddata->gpio);
		goto fail_remove_group;
	}

	ret = devm_request_any_context_irq(dev, irq, lid_interrupt_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"asustek_lid_isr", ddata);
	if (ret < 0) {
		static const char errmsg[] __initconst =
				"Unable to claim irq %d\n";
		dev_err(dev, errmsg, irq);
		goto fail_remove_group;
	}
	printk("[lid] %s : request irq handler ok\n", __func__);

	if (ddata->wakeup) {
		device_init_wakeup(&pdev->dev, 1);
		enable_irq_wake(irq);
	}

	printk("[lid] %s : End !!!\n", __func__);

	return ret;

fail_remove_group:
	sysfs_remove_group(&pdev->dev.kobj, &lid_attr_group);
	return ret;
}

static int lid_driver_remove(struct platform_device *pdev)
{
	struct asustek_lid_drvdata *ddata = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &lid_attr_group);

	if (ddata->wakeup)
		device_init_wakeup(&pdev->dev, 0);

	return 0;
}

static const struct of_device_id asustek_lid_of_match[] = {
	{ .compatible = DRV_NAME, },
	{},
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

static struct platform_driver asustek_lid_driver __refdata = {
	.probe = lid_driver_probe,
	.remove = lid_driver_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(asustek_lid_of_match),
	},
};

module_platform_driver(asustek_lid_driver);
MODULE_DESCRIPTION("ASUSTek Hall Sensor Driver");
MODULE_AUTHOR("Eric Yu <Eric_Yu@asus.com>");
MODULE_LICENSE("GPL");
