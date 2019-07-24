/*
 * ASUS googlekey driver.
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

#define DRV_NAME		"asustek_googlekey"
#define GOOGLEKEY_DEVICE_NAME		"googlekey_input"
#define GOOGLEKEY_PHYS		"/dev/input/googlekey_indev"
#define CONVERSION_TIME_MS	15
#define  DEBOUNCE_TIME_MS	10
static DEFINE_SPINLOCK(googlekey_slock);

#define GOOGLE_KEY_AVOID_GLITCH 1
#ifdef GOOGLE_KEY_AVOID_GLITCH
//static int cancel_key_send = 0;
#define MIN_AWAKE_TIME_MS	3000
#define GLITCH_RETRY_COUNT	0
#endif

static int g_google_key_code = KEY_ASUS_GOOGLE_ASSISTANT;

static void googlekey_report_function(struct work_struct *work);
static void googlekey_double_check(struct work_struct *work);
static ssize_t show_googlekey_status(struct device *dev,
		struct device_attribute *attr, char *buf);

struct asustek_googlekey_drvdata {
	struct input_dev *input;
	struct pinctrl *key_pinctrl;
	struct delayed_work dwork;
	struct delayed_work dcwork;
	int gpio;
	int active_low;
	int wakeup;
	unsigned int irq;
#ifdef GOOGLE_KEY_AVOID_GLITCH
	bool sendVirtualPress;
	int oldGpioValue;
	int keyState;
	bool key_press_queued;
	bool key_release_queued;
	bool key_cancel_able;
	struct wakeup_source staywake;
#endif
};

static int g_googlekey_enable=0;
bool googlekey_irq_wakup=0;

static ssize_t show_googlekey_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", g_googlekey_enable);
}

static ssize_t store_googlekey_enable(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	ssize_t ret;
	int state;
	struct asustek_googlekey_drvdata *ddata = dev_get_drvdata(dev);

	ret = sscanf(buf, "%d", &state);
	printk("[keypad] googlekey: buffer: %s\n",buf);

	if (ret != 1)
		return -EINVAL;
	else
	{
		g_googlekey_enable=state;
		if(g_googlekey_enable == 0) {
			printk("[keypad] googlekey: disable\n");
			if(googlekey_irq_wakup)
			{
				disable_irq_wake(ddata->irq);
				googlekey_irq_wakup=0;
				pr_info("[keypad] disable googlekey wakeup \n");
			}
			else
			{
				pr_info("[keypad] googlekey_irq_wakup=%d \n",googlekey_irq_wakup);
			}
			g_google_key_code = KEY_ASUS_GOOGLE_ASSISTANT;
			ddata->key_press_queued = false;
			ddata->key_release_queued = false;
		} else if(g_googlekey_enable == 1) {
			printk("[keypad] googlekey: enable\n");
			if(googlekey_irq_wakup)
				pr_info("[keypad] googlekey_irq_wakup=%d \n",googlekey_irq_wakup);
			else
			{
				enable_irq_wake(ddata->irq);
				googlekey_irq_wakup=1;
				pr_info("[keypad] enable googlekey wakeup \n");
			}
			g_google_key_code = KEY_ASUS_GOOGLE_ASSISTANT;
			ddata->key_press_queued = false;
			ddata->key_release_queued = false;
		} else if(g_googlekey_enable == 2) {
			printk("[keypad] googlekey: atd test mode\n");
			if(googlekey_irq_wakup)
				pr_info("[keypad] googlekey_irq_wakup=%d \n",googlekey_irq_wakup);
			else
			{
				enable_irq_wake(ddata->irq);
				googlekey_irq_wakup=1;
				pr_info("[keypad] enable googlekey wakeup \n");
			}
			g_google_key_code = BTN_0;
			ddata->key_press_queued = false;
			ddata->key_release_queued = false;
		} else {
			printk("[keypad] googlekey: not allow to change mode\n");
		}
	}
	return count;
}
static DEVICE_ATTR(googlekey_enable, (S_IRUGO|S_IWUSR), show_googlekey_enable, store_googlekey_enable);

static DEVICE_ATTR(googlekey_status, S_IRUGO, show_googlekey_status, NULL);

static struct attribute *googlekey_attrs[] = {
	&dev_attr_googlekey_status.attr,
	&dev_attr_googlekey_enable.attr,
	NULL
};

static struct attribute_group googlekey_attr_group = {
	.attrs = googlekey_attrs,
};

static ssize_t show_googlekey_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct asustek_googlekey_drvdata *ddata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			!!gpio_get_value_cansleep(ddata->gpio));
}

static irqreturn_t googlekey_interrupt_handler(int irq, void *dev_id)
{
	struct asustek_googlekey_drvdata *ddata = dev_id;
#ifdef GOOGLE_KEY_AVOID_GLITCH
	ktime_t getKeyTime = 0;
	static ktime_t oldGetKeyTime = 0;
	//static ktime_t press_key_time = 0;
	//ktime_t release_key_time = 0;
	long int key_duration=DEBOUNCE_TIME_MS;
	int gpiovalue = 0;

	//int retry=0;
#endif
	//printk("[keypad] googlekey_interrupt_handler\n");
	WARN_ONCE(irq != ddata->irq, "googlekey_interrupt failed\n");

#ifdef GOOGLE_KEY_AVOID_GLITCH
	//while( retry <= GLITCH_RETRY_COUNT ) { // try tree times for stable
		gpiovalue = gpio_get_value(ddata->gpio);

		getKeyTime = ktime_get();
		if(oldGetKeyTime)
			key_duration = ktime_to_ms(ktime_sub(getKeyTime,oldGetKeyTime));
		oldGetKeyTime=getKeyTime;
		if(key_duration < DEBOUNCE_TIME_MS)
			return IRQ_HANDLED;

		printk("[keypad] googlekey gpio: %d \n",gpiovalue); // press is 0, release is 1

		if (ddata->oldGpioValue == gpiovalue){
			if (gpiovalue == 0 ){
				gpiovalue=1; 
				printk("[keypad] googlekey should be Release, inverse for report correct key !!!\n");
			}else if(gpiovalue == 1){
				printk("[keypad] googlekey lost Press, send virtual press !!!\n");
				ddata->sendVirtualPress = true;
			}
		}
		ddata->oldGpioValue = gpiovalue;
		if (gpiovalue)
			ddata->key_release_queued = true;
		else
			ddata->key_press_queued = true;
		schedule_delayed_work(&ddata->dwork, msecs_to_jiffies(CONVERSION_TIME_MS));

		//printk("[GOOGLEKEY] gpio: %d \n",gpiovalue); // press is 0, release is 1
/*
		if( gpiovalue == 0 ) {
			// press key
			if(ddata->key_press_queued == false && ddata->key_release_queued == false) {
				// both key is not sent
				press_key_time = ktime_get();
				cancel_key_send = 0;
				printk("[GOOGLEKEY] press key \n");
				ddata->key_press_queued = true;
				schedule_delayed_work(&ddata->dwork,
						msecs_to_jiffies(CONVERSION_TIME_MS));
				retry = 0xf; // exit
			} else {
				printk("[GOOGLEKEY] Could be glitch for press key case. IGNOR this isr.\n");
				retry++;
			}

		} else {
			// release key
			if( ddata->key_press_queued == true && ddata->key_release_queued == false)  {
				//note: ddata->key_press_queued will be set to false after release key be sent
				release_key_time = ktime_get();
				key_duration = ktime_to_ms(ktime_sub(release_key_time,press_key_time));
				//printk("[GOOGLEKEY] duration: %ld ms\n",key_duration);
				press_key_time = 0;
				if(key_duration < CONVERSION_TIME_MS && ddata->key_cancel_able){
					printk("[GOOGLEKEY] Cancel sending key: duration: %ld \n",key_duration);
					cancel_key_send = 1;
					ddata->key_press_queued = false;
					ddata->key_release_queued = false;
				} else if(ddata->key_cancel_able == false) {
					// press key has been sent
					printk("[GOOGLEKEY] release key \n");
					cancel_key_send = 0;
					ddata->key_release_queued = true;
					schedule_delayed_work(&ddata->dwork,
						msecs_to_jiffies(CONVERSION_TIME_MS));
				} else {
					printk("[GOOGLEKEY] worng state for release key \n");
				}
				retry = 0xf; // exit
			} else {
				printk("[GOOGLEKEY] Could be glitch for release key case. IGNOR this isr.\n");
				retry++;
			}
		}
		*/
	//}
#else

	schedule_delayed_work(&ddata->dwork,
			msecs_to_jiffies(CONVERSION_TIME_MS));

#endif
	return IRQ_HANDLED;
}

static void googlekey_report_function(struct work_struct *work)
{
	struct asustek_googlekey_drvdata *ddata =
		container_of(work, struct asustek_googlekey_drvdata, dwork.work);
	//int value;
	unsigned long flags;

	if(g_googlekey_enable == 0)
	{
		printk("[keypad] googlekey is disabled\n");
		return;
	}

#ifdef GOOGLE_KEY_AVOID_GLITCH
	spin_lock_irqsave(&googlekey_slock, flags);

	if (ddata->sendVirtualPress){
		input_report_key(ddata->input, g_google_key_code, 1);
		input_sync(ddata->input);
		ddata->sendVirtualPress = false;
		ddata->keyState = 1;
		pr_info("[keypad] googlekey Virtual Press report\n");
		msleep(15);
	}

	if (ddata->key_press_queued){
		input_report_key(ddata->input, g_google_key_code, 1);
		input_sync(ddata->input);
		ddata->key_press_queued = false;
		ddata->keyState = 1;
		pr_info("[keypad] googlekey EV_KEY report = %d\n",  1);
		if(ddata->key_release_queued)
			msleep(15);
		else
			schedule_delayed_work(&ddata->dcwork, msecs_to_jiffies(300));
	}

	if (ddata->key_release_queued){
		input_report_key(ddata->input, g_google_key_code, 0);
		input_sync(ddata->input);
		ddata->key_release_queued = false;
		ddata->keyState = 0;
		pr_info("[keypad] googlekey EV_KEY report = %d\n",  0);
	}

	spin_unlock_irqrestore(&googlekey_slock, flags);
/*
	if(cancel_key_send) {
		pr_info("[keypad] googlekey glitch, avoid to send key\n");
	} else {

		if(ddata->key_press_queued == true && ddata->key_release_queued == false){
			// press key is not sent
			input_report_key(ddata->input, g_google_key_code, 1);
			input_sync(ddata->input);
			pr_info("[keypad] googlekey EV_KEY report value = %d\n", 1);
			//ddata->key_press_queued = false; // clear after release
			ddata->key_cancel_able = false;
			__pm_wakeup_event(&ddata->staywake, MIN_AWAKE_TIME_MS);
		} else if(ddata->key_press_queued == true && ddata->key_release_queued == true){
			// press key is sent, now sending release key
			input_report_key(ddata->input, g_google_key_code, 0);
			input_sync(ddata->input);
			pr_info("[keypad] googlekey EV_KEY report value = %d\n", 0);
			ddata->key_release_queued = false;
			ddata->key_press_queued = false;
			ddata->key_cancel_able = true;
		} else {
			pr_info("[keypad] googlekey WRONG: ddata->key_press_queued = %d, value: %d\n", ddata->key_press_queued, value);
			ddata->key_press_queued = false;
			ddata->key_release_queued = false;
		}
	}
	*/
#else
	value = !!gpio_get_value_cansleep(ddata->gpio) ^ ddata->active_low;

	input_report_key(ddata->input, g_google_key_code, value);
	input_sync(ddata->input);
	pr_info("[keypad] googlekey EV_KEY report value = %d\n", value);
#endif

}

static void googlekey_double_check(struct work_struct *work)
{
	struct asustek_googlekey_drvdata *ddata =
		container_of(work, struct asustek_googlekey_drvdata, dcwork.work);
	unsigned long flags;
	spin_lock_irqsave(&googlekey_slock, flags);
	if(ddata->keyState)
	{
		int gpiovalue = 0;
		gpiovalue = gpio_get_value(ddata->gpio);
		//pr_info("[keypad] googlekey double_check gpio = %d\n", gpiovalue);
		if(gpiovalue){
			input_report_key(ddata->input, g_google_key_code, 0);
			input_sync(ddata->input);
			ddata->key_release_queued = false;
			ddata->keyState = 0;
			ddata->oldGpioValue=1;//avoid gpio detect 0->0 to do inverse handle
			pr_info("[keypad] googlekey Virtual Release report\n");
		}
	}
	spin_unlock_irqrestore(&googlekey_slock, flags);
}
/* translate openfirmware node properties */
static int __init asustek_googlekey_get_devtree(struct device *dev)
{
	struct asustek_googlekey_drvdata *ddata = dev_get_drvdata(dev);
	struct device_node *node;
	enum of_gpio_flags flags;

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
	ddata->wakeup = !!of_get_property(node, "asustek_googlekey,wakeup", NULL);

	return 0;
}

static int asustek_googlekey_pinctrl_configure(struct pinctrl *key_pinctrl, bool active)
{
	struct pinctrl_state *set_state;

	int retval;

	if (active) {
		set_state = pinctrl_lookup_state(key_pinctrl,"asustek_googlekey_active");

		if (IS_ERR(set_state)) {
			pr_err("Can't get asustek_googlekey pinctrl active state\n");
			return PTR_ERR(set_state);
		}
	} else {
		set_state = pinctrl_lookup_state(key_pinctrl, "asustek_googlekey_suspend");

		if (IS_ERR(set_state)) {
			pr_err("Can't get asustek_googlekey pinctrl sleep state\n");
			return PTR_ERR(set_state);
		}
	}
	retval = pinctrl_select_state(key_pinctrl, set_state);

	if (retval) {
		pr_err("Can't set asustek_googlekey pinctrl state\n");
		return retval;
	}
	return 0;
}

static int __init googlekey_driver_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct device *dev = &pdev->dev;
	struct asustek_googlekey_drvdata *ddata;
	struct input_dev *input;
	static const char googlekey_probe[] __initconst = "ASUSTek: %s\n";

	if (!pdev)
		return -EINVAL;

	dev_info(dev, googlekey_probe, __func__);
	ddata = kzalloc(sizeof(struct asustek_googlekey_drvdata), GFP_KERNEL);
	input = input_allocate_device();

	if (!ddata || !input) {
		static const char errmsg[] __initconst =
					"Failed to allocate state\n";
		ret = -ENOMEM;
		dev_err(dev, errmsg);
		input_free_device(input);
		goto fail_data;
	}

	ddata->input = input;
	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = GOOGLEKEY_DEVICE_NAME;
	input->phys = GOOGLEKEY_PHYS;
	set_bit(EV_KEY, input->evbit);
	set_bit(KEY_ASUS_GOOGLE_ASSISTANT, input->keybit);
	set_bit(BTN_0, input->keybit);


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
		ret = asustek_googlekey_pinctrl_configure(ddata->key_pinctrl, true);
		if (ret) {
			dev_err(dev, "cannot set pinctrl active state\n");
			goto fail_data;
		}
	}

	ret = input_register_device(input);

	if (ret) {
		static const char errmsg[] __initconst =
				"Unable to register input device, error: %d\n";
		dev_err(dev, errmsg, ret);
		input_free_device(input);
		goto fail_data;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &googlekey_attr_group);

	if (ret) {
		static const char errmsg[] __initconst =
				"Unable to create sysfs, error: %d\n";
		dev_err(dev, errmsg, ret);
		goto fail_unregister;
	}

	INIT_DELAYED_WORK(&ddata->dwork, googlekey_report_function);
	INIT_DELAYED_WORK(&ddata->dcwork, googlekey_double_check);

#ifdef GOOGLE_KEY_AVOID_GLITCH
	ddata->sendVirtualPress = false;
	ddata->oldGpioValue = 2;
	ddata->keyState = 0;
	ddata->key_press_queued = false;
	ddata->key_release_queued = false;
	ddata->key_cancel_able = true;
	wakeup_source_init(&ddata->staywake, "googlekey_wake");
#endif

	asustek_googlekey_get_devtree(dev);

	if (!gpio_is_valid(ddata->gpio)) {
		static const char errmsg[] __initconst =
					"Invalid GPIO %d\n";
		dev_err(dev, errmsg, ddata->gpio);
		goto fail_create;
	}

	ret = gpio_request(ddata->gpio, DRV_NAME);

	if (ret < 0) {
		static const char errmsg[] __initconst =
				"Failed to request GPIO %d\n";
		dev_err(dev, errmsg, ddata->gpio);
		goto fail_create;
	}

	ret = gpio_direction_input(ddata->gpio);

	if (ret < 0) {
		static const char errmsg[] __initconst =
				"Failed to configure direction for GPIO %d\n";
		dev_err(dev, errmsg, ddata->gpio);
		goto fail_free;
	}

	irq = gpio_to_irq(ddata->gpio);
	ddata->irq = irq;

	if (irq < 0) {
		static const char errmsg[] __initconst =
				"Unable to get irq number for GPIO %d\n";
		dev_err(dev, errmsg, ddata->gpio);
		goto fail_free;
	}

	ret = request_any_context_irq(irq, googlekey_interrupt_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"asustek_googlekey_isr", ddata);

	if (ret < 0) {
		static const char errmsg[] __initconst =
				"Unable to claim irq %d\n";
		dev_err(dev, errmsg, irq);
		goto fail_free;
	}

	if (ddata->wakeup) {
		device_init_wakeup(&pdev->dev, 1);
		enable_irq_wake(irq);
		googlekey_irq_wakup=1;
	}

	return ret;

fail_free:
	gpio_free(ddata->gpio);
fail_create:
	sysfs_remove_group(&pdev->dev.kobj, &googlekey_attr_group);
fail_unregister:
	input_unregister_device(input);
fail_data:
	kfree(ddata);
	return ret;
}

static int googlekey_driver_remove(struct platform_device *pdev)
{
	struct asustek_googlekey_drvdata *ddata = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &googlekey_attr_group);
	free_irq(ddata->irq, ddata);
	cancel_delayed_work_sync(&ddata->dwork);

	if (gpio_is_valid(ddata->gpio))
		gpio_free(ddata->gpio);

	input_unregister_device(ddata->input);

	if (ddata->wakeup)
		device_init_wakeup(&pdev->dev, 0);

	kfree(ddata);

	return 0;
}

static const struct of_device_id asustek_googlekey_of_match[] = {
	{ .compatible = DRV_NAME, },
	{},
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

static int __maybe_unused googlekey_suspend(struct device *dev)
{
	struct asustek_googlekey_drvdata *ddata = dev_get_drvdata(dev);

#ifdef GOOGLE_KEY_AVOID_GLITCH
	if(ddata->key_press_queued == true || ddata->key_release_queued == true){
		pr_err("googlekey_suspend(): key is processing, so not to suspend.\n");
		__pm_wakeup_event(&ddata->staywake, MIN_AWAKE_TIME_MS);
		return -EBUSY;
	}
#endif
	return 0;
}

static int __maybe_unused googlekey_resume(struct device *dev)
{
//	struct asustek_googlekey_drvdata *ddata = dev_get_drvdata(dev);

	return 0;
}


static SIMPLE_DEV_PM_OPS(googlekey_pm_ops, googlekey_suspend, googlekey_resume);
static struct platform_driver asustek_googlekey_driver __refdata = {
	.probe = googlekey_driver_probe,
	.remove = googlekey_driver_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm	= &googlekey_pm_ops,
		.of_match_table = of_match_ptr(asustek_googlekey_of_match),
	},
};

module_platform_driver(asustek_googlekey_driver);
MODULE_DESCRIPTION("ASUSTek Google Assistant Key Driver");
MODULE_AUTHOR("Bross Kuo <Bross_Kuo@asus.com>");
MODULE_LICENSE("GPL");
