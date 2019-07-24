/* 
 * Copyright (C) 2019 ASUSTek Inc.
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
/* SAR Sensor GPIO Module */
/*****************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/input/ASH.h>

#define SAR_INTEL_NAME 	"SAR_INT#"
#define SAR_QCOM_NAME 	"qcom,sar-gpio"
#define SAR_IRQ_NAME		"SAR_SENSOR_IRQ"
#define SAR_INT_NAME		"SAR_SENSOR_INT"

static int ASUS_SAR_SENSOR_GPIO;
static SAR_sensor_GPIO * mSAR_sensor_GPIO;

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_GPIO"
#define SENSOR_TYPE_NAME		"SAR"

#undef dbg
#ifdef ASH_GPIO_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

static irqreturn_t SAR_sensor_irq_handler(int irq, void *dev_id);

#ifdef GPIO_INTEL
#include <asm/intel-mid.h>
#endif

#ifdef GPIO_QCOM
#include <linux/of_gpio.h>
#define SAR_SENSOR_GPIO_LOOKUP_STATE	"sar_gpio_high"

static void set_pinctrl(struct i2c_client *client)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	key_pinctrl = devm_pinctrl_get(&client->dev);
	set_state = pinctrl_lookup_state(key_pinctrl, SAR_SENSOR_GPIO_LOOKUP_STATE);
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
	irq = gpio_to_irq(ASUS_SAR_SENSOR_GPIO);
	if (irq < 0)	{
		err("%s: gpio_to_irq ERROR(%d).\n", __FUNCTION__, irq);
		return irq;
	} else	{
		log("gpio_to_irq IRQ %d successed on GPIO:%d\n", irq, ASUS_SAR_SENSOR_GPIO);
	}

	/*Request IRQ*/	
	#ifdef GPIO_INTEL
	ret = request_irq(irq, SAR_sensor_irq_handler,
			IRQF_SHARED |IRQF_TRIGGER_FALLING, SAR_INT_NAME, NULL);
	#endif
	
	#ifdef GPIO_QCOM
	ret = request_threaded_irq(irq, NULL, SAR_sensor_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT , SAR_INT_NAME, NULL);
	#endif
	
	if (ret < 0) {
		err("%s: request_irq/request_threaded_irq ERROR(%d).\n", __FUNCTION__, ret);
		return ret;
	} else	{
		dbg("Disable irq\n");
		disable_irq(irq);
	}

	return irq;
}

irqreturn_t SAR_sensor_irq_handler(int irq, void *dev_id)
{
	mSAR_sensor_GPIO->SAR_sensor_isr();
	return IRQ_HANDLED;
}

int SAR_sensor_gpio_register(struct i2c_client *client, SAR_sensor_GPIO *gpio_ist)
{
	int ret = 0;
	int irq = 0;

	mSAR_sensor_GPIO = gpio_ist;
	
	/* GPIO */
	#ifdef GPIO_INTEL
	log("Intel GPIO \n");
	ASUS_SAR_SENSOR_GPIO = get_gpio_by_name(SAR_INTEL_NAME);
	#endif
	
	#ifdef GPIO_QCOM
	log("Qcom GPIO \n");
	set_pinctrl(client);
	ASUS_SAR_SENSOR_GPIO = of_get_named_gpio(client->dev.of_node, SAR_QCOM_NAME, 0);	
	#endif
		
	dbg("GPIO =%d(%d)\n", ASUS_SAR_SENSOR_GPIO, gpio_get_value(ASUS_SAR_SENSOR_GPIO));	
	/* GPIO Request */
	ret = gpio_request(ASUS_SAR_SENSOR_GPIO, SAR_IRQ_NAME);
	if (ret) {
		err("%s: gpio_request ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}
	/* GPIO Direction */
	ret = gpio_direction_input(ASUS_SAR_SENSOR_GPIO);
	if (ret < 0) {
		err("%s: gpio_direction_input ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}
	/*IRQ*/
	irq = init_irq();
	
	return irq;

}
EXPORT_SYMBOL(SAR_sensor_gpio_register);


int SAR_sensor_gpio_unregister(int irq)
{
	free_irq(irq, NULL);
	gpio_free(ASUS_SAR_SENSOR_GPIO);
	return 0;
}
EXPORT_SYMBOL(SAR_sensor_gpio_unregister);

int SAR_sensor_gpio_value(void)
{
	if (gpio_get_value(ASUS_SAR_SENSOR_GPIO) > 0)
		return 1;
	else
		return 0;		
}
EXPORT_SYMBOL(SAR_sensor_gpio_value);
