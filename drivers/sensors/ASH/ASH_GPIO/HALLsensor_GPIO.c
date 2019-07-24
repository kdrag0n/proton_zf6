/* 
 * Copyright (C) 2015 ASUSTek Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 /******************************/
/* HALL Sensor GPIO Module */
/*****************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/input/ASH.h>

#define HALL_INTEL_NAME 	"hall_det#"
#define HALL_QCOM_NAME 	"qcom,hall-gpio"
#define HALL_IRQ_NAME		"HALL_SENSOR_IRQ"
#define HALL_INT_NAME		"HALL_SENSOR_INT"

static int ASUS_HALL_SENSOR_GPIO;
static HALLsensor_GPIO * mHALLsensor_GPIO;

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_GPIO"
#define SENSOR_TYPE_NAME		"Hall"

#undef dbg
#ifdef ASH_GPIO_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

static irqreturn_t HALLsensor_irq_handler(int irq, void *dev_id);

#ifdef GPIO_INTEL
#include <asm/intel-mid.h>
#endif

#ifdef GPIO_QCOM
#include <linux/of_gpio.h>
#define GPIO_LOOKUP_STATE	"hall_gpio_high"

static void set_pinctrl(struct device *dev)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	key_pinctrl = devm_pinctrl_get(dev);
	set_state = pinctrl_lookup_state(key_pinctrl, GPIO_LOOKUP_STATE);
	ret = pinctrl_select_state(key_pinctrl, set_state);
	if(ret < 0)
		err("%s: pinctrl_select_state ERROR(%d).\n", __FUNCTION__, ret);
}
#endif
static int init_irq (void)
{
	int ret = 0;
	int irq = 0;

	/* GPIO to IRQ */
	irq = gpio_to_irq(ASUS_HALL_SENSOR_GPIO);
	if (irq < 0) {
		err("%s: gpio_to_irq ERROR(%d).\n", __FUNCTION__, irq);
		return irq;
	}else {
		log("gpio_to_irq IRQ %d successed on GPIO:%d\n", irq, ASUS_HALL_SENSOR_GPIO);
	}

	/*Request IRQ*/	
	#ifdef GPIO_INTEL
	ret = request_irq(irq,	hall_sensor_interrupt_handler, 
			IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, 
			HALL_INT_NAME, NULL);
	#endif
	
	#ifdef GPIO_QCOM
	ret = request_threaded_irq(irq, NULL, HALLsensor_irq_handler,
				IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				HALL_INT_NAME, NULL);
	#endif
	
	if (ret < 0){
		err("%s: request_irq/request_threaded_irq ERROR(%d).\n", __FUNCTION__, ret);
		return ret;
	}else {
		dbg("Enable irq\n");
		enable_irq_wake(irq);
	}

	return irq;
}

irqreturn_t HALLsensor_irq_handler(int irq, void *dev_id)
{
	mHALLsensor_GPIO->HALLsensor_isr();
	return IRQ_HANDLED;
}

int HALLsensor_gpio_register(struct platform_device *pdev, HALLsensor_GPIO *gpio_ist)
{
	int ret = 0;
	int irq = 0;

	mHALLsensor_GPIO = gpio_ist;
	
	/* GPIO */
	#ifdef GPIO_INTEL
	log("Intel GPIO \n");
	ASUS_HALL_SENSOR_GPIO = get_gpio_by_name(HALL_INTEL_NAME);
	#endif
	
	#ifdef GPIO_QCOM
	log("Qcom GPIO \n");
	set_pinctrl(&pdev->dev);
	ASUS_HALL_SENSOR_GPIO = of_get_named_gpio(pdev->dev.of_node, HALL_QCOM_NAME, 0);	
	#endif
		
	dbg("GPIO =%d(%d)\n", ASUS_HALL_SENSOR_GPIO, gpio_get_value(ASUS_HALL_SENSOR_GPIO));	
	/* GPIO Request */
	ret = gpio_request(ASUS_HALL_SENSOR_GPIO, HALL_IRQ_NAME);
	if (ret) {
		err("%s: gpio_request ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}
	/* GPIO Direction */
	ret = gpio_direction_input(ASUS_HALL_SENSOR_GPIO);
	if (ret < 0) {
		err("%s: gpio_direction_input ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}
	/*IRQ*/
	irq = init_irq();
	
	return irq;

}
EXPORT_SYMBOL(HALLsensor_gpio_register);


int HALLsensor_gpio_unregister(int irq)
{
	free_irq(irq, NULL);
	gpio_free(ASUS_HALL_SENSOR_GPIO);
	return 0;
}
EXPORT_SYMBOL(HALLsensor_gpio_unregister);

int HALLsensor_gpio_value(void)
{
	 if (gpio_get_value(ASUS_HALL_SENSOR_GPIO) > 0)
			return 1;
        else
                	return 0;		
}
EXPORT_SYMBOL(HALLsensor_gpio_value);