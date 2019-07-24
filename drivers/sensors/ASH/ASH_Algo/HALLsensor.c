#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/input.h>
//#include <linux/wakelock.h>
#include "ASH_Wakelock.h"
#include <linux/input/ASH.h>

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_ALGO"
#define SENSOR_TYPE_NAME		"Hallsensor"

#undef dbg
#ifdef ASH_ALGO_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) do{	\
		printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args);	\
		sprintf(g_error_mesg, "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args);	\
	}while(0)

static void hall_sensor_report_function(struct work_struct *data);

/********************/
/* Global Variables */
/******************/
static struct workqueue_struct 			*hall_sensor_wq;
static struct ASUS_hall_sensor_data	*g_hall_data;
static spinlock_t 							g_hall_spin_lock;
static struct wake_lock 					g_hall_wake_lock;
static 	DECLARE_DELAYED_WORK(hall_sensor_report_work, hall_sensor_report_function);
static int 									ASUS_HALL_SENSOR_IRQ;
static char *g_error_mesg;
/*****************************/
/* Hall Sensor Configuration */
/****************************/
#define DRIVER_NAME 		"hall_sensor"
#define GPIO_NAME 		"hall_det#"
#define IRQ_Name			"ASUS_Hall_Sensor-irq"
#define INT_NAME			"HallSensor_INT"
#define KERNEL_OBJECT	"hall_sensor_kobject"
#define WAKE_LOCK_TIME	(800)

/**************************/
/* Driver Data Structure */
/*************************/
struct ASUS_hall_sensor_data { 	
	bool status;	
	bool enable; 
	int debounce;	
	int int_counter;
	int event_counter;
};

/*===========================
 *|| sysfs DEVICE_ATTR part ||
 *===========================
 */
bool mshow_action_status(void)
{
	return g_hall_data->status;
}

bool mshow_hall_sensor_enable(void)
{
	return g_hall_data->enable;
}

int mstore_hall_sensor_enable(bool bOn)
{
	unsigned long flags;
	
	if(bOn==g_hall_data->enable){
		return 0;
	}	
		
	if(false == bOn) {
		/* Turn off */
		log("Turn off.\n");
		
		spin_lock_irqsave(&g_hall_spin_lock, flags);
		g_hall_data->enable=bOn;
		spin_unlock_irqrestore(&g_hall_spin_lock, flags);
		
		disable_irq_nosync(ASUS_HALL_SENSOR_IRQ);
		
	}else if(true == bOn){
		/* Turn on */
		log("Turn on. \n");
		
		spin_lock_irqsave(&g_hall_spin_lock, flags);
		g_hall_data->enable=bOn;
		spin_unlock_irqrestore(&g_hall_spin_lock, flags);
		
		enable_irq(ASUS_HALL_SENSOR_IRQ);
		
	}else{
		err("Enable/Disable Error, can not recognize (%d)", bOn);
		return -EINVAL;
	}	
	
	return 0;
}

int mshow_hall_sensor_debounce(void)
{
	return g_hall_data->debounce;
}

int mstore_hall_sensor_debounce(int debounce)
{
	g_hall_data->debounce = debounce;
	return 0;
}

int mshow_hall_sensor_int_count(void)
{
	return g_hall_data->int_counter;
}

int mshow_hall_sensor_event_count(void)
{
	return g_hall_data->event_counter;
}

int mshow_hall_sensor_error_mesg(char *error_mesg)
{
	memcpy(error_mesg, g_error_mesg, strlen(g_error_mesg)+1);
	return 0;
}

static HALLsensor_ATTR mHALLsensor_ATTR = {
	.show_action_status = mshow_action_status,
	.show_hall_sensor_enable = mshow_hall_sensor_enable,
	.store_hall_sensor_enable = mstore_hall_sensor_enable,
	.show_hall_sensor_debounce = mshow_hall_sensor_debounce,
	.store_hall_sensor_debounce = mstore_hall_sensor_debounce,
	.show_hall_sensor_int_count = mshow_hall_sensor_int_count,
	.show_hall_sensor_event_count = mshow_hall_sensor_event_count,
	.show_hall_sensor_error_mesg = mshow_hall_sensor_error_mesg,
};

/*===============================
 *|| Interrupt Service Routine part ||
 *===============================
 */
static void hall_sensor_report_function(struct work_struct *data)
{
	  int GPIO_value; 

	//msleep(50);
        if(!g_hall_data->enable){
                log("[ISR] hall sensor is disable!\n");
		goto IST_RET;
        }
        
        if (HALLsensor_gpio_value() > 0)
			GPIO_value = 1;
        else
                	GPIO_value = 0;		
	  
	  if(GPIO_value != g_hall_data->status){
		g_hall_data->status =GPIO_value;
	  }else{
	  	dbg("filter the same status : %d\n", GPIO_value);
		goto IST_RET;
	  }
        		
	  if(g_hall_data->status)
	  	hallsensor_report_lid(HALLSENSOR_REPORT_LID_OPEN);
	  else
		hallsensor_report_lid(HALLSENSOR_REPORT_LID_CLOSE);
	g_hall_data->event_counter++;	/* --- For stress test debug --- */
        log("[ISR] report value = %s\n", g_hall_data->status?"open":"close");
IST_RET:
	//wake_unlock(&hall_sensor_dev->wake_lock);
	return;

}

void hall_sensor_interrupt_handler(void)
{
	int GPIO_value;	
	
	cancel_delayed_work(&hall_sensor_report_work);

	/*get GPIO status*/
	if (HALLsensor_gpio_value() > 0) GPIO_value = 1;
      else GPIO_value = 0;		
	log("[ISR] hall_sensor_interrupt = %s\n",GPIO_value?"open":"close");
	g_hall_data->int_counter++;	/* --- For stress test debug --- */
	
	/*start work queue when status change*/
	if(GPIO_value != g_hall_data->status){
		queue_delayed_work(hall_sensor_wq, &hall_sensor_report_work, 
			msecs_to_jiffies(g_hall_data->debounce));
		wake_lock_timeout(&g_hall_wake_lock, msecs_to_jiffies(WAKE_LOCK_TIME));
	}	
	
	return;
}

static HALLsensor_GPIO mHALLsensor_GPIO = {
	.HALLsensor_isr = hall_sensor_interrupt_handler,
};

void mhall_sensor_probe(struct platform_device *pdev)
{	
	log("Driver PROBE +++\n");

	/* GPIO */
	ASUS_HALL_SENSOR_IRQ = HALLsensor_gpio_register(pdev, &mHALLsensor_GPIO);
	if (ASUS_HALL_SENSOR_IRQ < 0)
		goto probe_err;

	/* Get the First Status */
	if (HALLsensor_gpio_value() > 0) {
		g_hall_data->status = true;
		hallsensor_report_lid(HALLSENSOR_REPORT_LID_OPEN);
	} else {
               	g_hall_data->status = false;
		hallsensor_report_lid(HALLSENSOR_REPORT_LID_CLOSE);
        }
	log("report value = %s\n", g_hall_data->status?"open":"close");
	log("Driver PROBE ---\n");
	return;
probe_err:
	err("Driver PROBE ERROR ---\n");
	return;
	
}

void mhall_sensor_suspend(void)
{
	log("Driver SUSPEND +++\n");

	log("Driver SUSPEND ---\n");

}

void mhall_sensor_resume(void)
{
	log("Driver RESUME +++\n");

	log("Driver RESUME ---\n");

}

static HALLsensor_Platform mHALLsensor_Platform = {
	.HALLsensor_probe = mhall_sensor_probe,
	.HALLsensor_suspend = mhall_sensor_suspend,
	.HALLsensor_resume = mhall_sensor_resume,
};

static int init_data(void)
{
	int ret = 0;
	
	/* Memory allocation for data structure */
	g_hall_data = kzalloc(sizeof (struct ASUS_hall_sensor_data), GFP_KERNEL);
	if (!g_hall_data) {
		err("Memory allocation fails for hall sensor\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	
	g_hall_data->enable = true;

	g_hall_data->debounce = 150;

	g_hall_data->int_counter = 0;
	g_hall_data->event_counter = 0;

	/*Record the error message*/
	g_error_mesg = kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);
	
	spin_lock_init(&g_hall_spin_lock);
	wake_lock_init(&g_hall_wake_lock, WAKE_LOCK_SUSPEND, "HallSensor_wake_lock");

	return 0;
init_data_err:
	err("Init Data ERROR\n");
	return ret;
}

static int __init hall_sensor_init(void)
{
	int ret = 0;
	log("Driver INIT +++\n");

	/* Work Queue */
	hall_sensor_wq = create_singlethread_workqueue(SENSOR_TYPE_NAME"_wq");

	/* driver data structure initialize */
	ret = init_data();
	if (ret < 0)
		goto init_err;

	/* Input Device */
	ret = HALLsensor_report_register();
	if (ret < 0)
		goto init_err;

	/* i2c Registration */				
	ret = HALLsensor_platform_register(&mHALLsensor_Platform);
	if (ret < 0)
		goto init_err;

	/* Attribute */
	ret = HALLsensor_ATTR_register(&mHALLsensor_ATTR);
	if (ret < 0)
		goto init_err;
	
	log("Driver INIT ---\n");
	return 0;
init_err:
	err("Driver INIT ERROR ---\n");
	return ret;
}

static void __exit hall_sensor_exit(void)
{
	log("Driver EXIT +++\n");
	
	HALLsensor_platform_unregister();

	HALLsensor_report_unregister();

	wake_lock_destroy(&g_hall_wake_lock);

	kfree(g_hall_data);
	destroy_workqueue(hall_sensor_wq);
		
	log("Driver EXIT ---\n");
}


module_init(hall_sensor_init);
module_exit(hall_sensor_exit);

MODULE_AUTHOR("sr_Huang <sr_Huang@asus.com>");
MODULE_DESCRIPTION("Hall Sensor");
MODULE_LICENSE("GPL v2");
