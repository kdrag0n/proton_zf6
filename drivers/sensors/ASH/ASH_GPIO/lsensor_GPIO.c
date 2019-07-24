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

 /***************************/
/* IR Sensor GPIO Module */
/**************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/input/ASH.h>

#define ALS_INTEL_NAME 	"ALS_INT#"
#define ALS_QCOM_NAME 	"qcom,als-gpio"
#define ALS_IRQ_NAME	"ALS_SENSOR_IRQ"
#define ALS_INT_NAME		"ALS_SENSOR_INT"

static int ALS_SENSOR_GPIO;
static lsensor_GPIO * mlsensor_GPIO;

/*******************************/
/* Debug and Log System */
/*****************************/
#define MODULE_NAME			"ASH_GPIO"
#define SENSOR_TYPE_NAME		"light"

#undef dbg
#ifdef ASH_GPIO_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

static irqreturn_t lsensor_irq_handler(int irq, void *dev_id);

#ifdef GPIO_INTEL
#include <asm/intel-mid.h>
#endif

#ifdef GPIO_QCOM
#include <linux/of_gpio.h>
#define GPIO_LOOKUP_STATE	"als_gpio_high"

static void set_pinctrl(struct i2c_client *client)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	key_pinctrl = devm_pinctrl_get(&client->dev);
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
	irq = gpio_to_irq(ALS_SENSOR_GPIO);
	if (irq < 0) {
		err("%s: gpio_to_irq ERROR(%d). \n", __FUNCTION__, irq);
		return irq;
	}else {
		log("gpio_to_irq IRQ %d successed on GPIO:%d\n", irq, ALS_SENSOR_GPIO);
	}

	/*Request IRQ*/	
	#ifdef GPIO_INTEL
	ret = request_irq(irq,lsensor_irq_handler, IRQF_TRIGGER_LOW, ALS_INT_NAME, NULL);
	#endif
	
	#ifdef GPIO_QCOM
	ret = request_threaded_irq(irq, NULL, lsensor_irq_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, ALS_INT_NAME, NULL);
	#endif
	if (ret < 0) {
		err("%s: request_irq/request_threaded_irq ERROR(%d).\n", __FUNCTION__, ret);
		return ret;
	}else {		
		dbg("Disable irq !! \n");
		disable_irq(irq);
	}

	return irq;
}

irqreturn_t lsensor_irq_handler(int irq, void *dev_id)
{
	mlsensor_GPIO->lsensor_isr();
	return IRQ_HANDLED;
}

int lsensor_gpio_register(struct i2c_client *client, lsensor_GPIO *gpio_ist)
{
	int ret = 0;
	int irq = 0;

	mlsensor_GPIO = gpio_ist;
	
	/* GPIO */
	#ifdef GPIO_INTEL
	log("Intel GPIO \n");
	ALS_SENSOR_GPIO = get_gpio_by_name(ALS_INTEL_NAME);
	#endif
	
	#ifdef GPIO_QCOM
	log("Qcom GPIO \n");
	set_pinctrl(client);
//	ALS_SENSOR_GPIO = of_get_named_gpio_flags(client->dev.of_node, ALS_QCOM_NAME, 0, NULL);
	ALS_SENSOR_GPIO = of_get_named_gpio(client->dev.of_node, ALS_QCOM_NAME, 0);
	#endif
		
	dbg("[GPIO] GPIO =%d(%d)\n", ALS_SENSOR_GPIO, gpio_get_value(ALS_SENSOR_GPIO));	
	/* GPIO Request */
	ret = gpio_request(ALS_SENSOR_GPIO, ALS_IRQ_NAME);
	if (ret) {
		err("%s: gpio_request ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}

	/* GPIO Direction */
	ret = gpio_direction_input(ALS_SENSOR_GPIO);

	if (ret < 0) {
		err("%s: gpio_direction_input ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}
	/*IRQ*/
	irq = init_irq();

	return irq;

}
EXPORT_SYMBOL(lsensor_gpio_register);


int lsensor_gpio_unregister(int irq)
{
	free_irq(irq, NULL);
	gpio_free(ALS_SENSOR_GPIO);
	return 0;
}
EXPORT_SYMBOL(lsensor_gpio_unregister);
