/* Copyright (c) 2014, 2016-2017 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/msm_ion.h>
#include <dsp/msm_audio_ion.h>
#include <dsp/audio_calibration.h>
#include <dsp/audio_cal_utils.h>

//Jessy +++ AudioWizard hifi & ringtone mode
#include <linux/input.h>

int g_audiowizard_force_preset_state = 0;
struct input_dev *audiowizard;

//Jessy ---

//Jessy +++ AudioWizard muti-channel mode
struct input_dev *audiowizard_channel;
//Jessy ---

/* ASUS_BSP +++ Add warning uevent for input occupied issue ( TT1290090 ) */
static struct kset *activeinputpid_uevent_kset;
static struct kobject *activeinputpid_kobj;
static void send_activeinput_pid_uevent(int activeinputpid, int failedinputpid);
/* ASUS_BSP --- */

/* ASUS_BSP +++ Audio mode and device */
static int audio_mode = -1;
static int audmode = -1;
static int rcv_device = -1;
static int rcvdev = -1;
/* bool fts_set_pmode(bool pmode); */
/* ASUS_BSP --- Audio mode and device */

/* ASUS_BSP +++ Add uevent to IMS for Line audio_mode = 0 when Line VoIP incall */
static int active_outputpid = 0;
static struct kset *rcv_notification_uevent_kset;
static struct kobject *rcv_notification_kobj;
static void send_rcv_notification_uevent(int rcvdev);
/* ASUS_BSP --- Add uevent to IMS for Line audio_mode = 0 when Line VoIP incall */

struct audio_cal_client_info {
	struct list_head		list;
	struct audio_cal_callbacks	*callbacks;
};

struct audio_cal_info {
	struct mutex			common_lock;
	struct mutex			cal_mutex[MAX_CAL_TYPES];
	struct list_head		client_info[MAX_CAL_TYPES];
	int				ref_count;
};

static struct audio_cal_info	audio_cal;


//Jessy +++ AudioWizard hifi & ringtone mode
//hifi:2 , ringtone:1
static void send_audiowizard_state(struct input_dev *dev ,int audiowizard_state)
{
    if(audiowizard_state == 2) {
        input_report_switch(audiowizard,SW_AUDIOWIZARD_RINGTONG,0);
        input_report_switch(audiowizard,SW_AUDIOWIZARD_HIFI,1);
    } else if (audiowizard_state == 1) {
        input_report_switch(audiowizard,SW_AUDIOWIZARD_RINGTONG,1);
        input_report_switch(audiowizard,SW_AUDIOWIZARD_HIFI,0);
    } else {
        input_report_switch(audiowizard,SW_AUDIOWIZARD_RINGTONG,0);
        input_report_switch(audiowizard,SW_AUDIOWIZARD_HIFI,0);
    }
}
//Jessy ---

//Jessy +++ AudioWizard muti-channel mode
static void send_audiowizard_channel(struct input_dev *dev ,int channel)
{
    if(channel == 1 ){
        input_report_switch(audiowizard_channel,SW_AUDIOWIZARD_CHANNELS_2,0);
        input_report_switch(audiowizard_channel,SW_AUDIOWIZARD_CHANNELS_M,0);
        input_report_switch(audiowizard_channel,SW_AUDIOWIZARD_CHANNELS_1,1);
    } else if (channel == 2) {
        input_report_switch(audiowizard_channel,SW_AUDIOWIZARD_CHANNELS_1,0);
        input_report_switch(audiowizard_channel,SW_AUDIOWIZARD_CHANNELS_M,0);
        input_report_switch(audiowizard_channel,SW_AUDIOWIZARD_CHANNELS_2,1);    
    } else if (channel > 2) {
        input_report_switch(audiowizard_channel,SW_AUDIOWIZARD_CHANNELS_1,0);
        input_report_switch(audiowizard_channel,SW_AUDIOWIZARD_CHANNELS_2,0);
        input_report_switch(audiowizard_channel,SW_AUDIOWIZARD_CHANNELS_M,1);
    } else {
        input_report_switch(audiowizard_channel,SW_AUDIOWIZARD_CHANNELS_1,0);
        input_report_switch(audiowizard_channel,SW_AUDIOWIZARD_CHANNELS_2,0);
        input_report_switch(audiowizard_channel,SW_AUDIOWIZARD_CHANNELS_M,0);
    }    
}
//Jessy ---

static bool callbacks_are_equal(struct audio_cal_callbacks *callback1,
				struct audio_cal_callbacks *callback2)
{
	bool ret = true;
	struct audio_cal_callbacks *call1 = callback1;
	struct audio_cal_callbacks *call2 = callback2;

	pr_debug("%s\n", __func__);

	if ((call1 == NULL) && (call2 == NULL))
		ret = true;
	else if ((call1 == NULL) || (call2 == NULL))
		ret = false;
	else if ((call1->alloc != call2->alloc) ||
		(call1->dealloc != call2->dealloc) ||
		(call1->pre_cal != call2->pre_cal) ||
		(call1->set_cal != call2->set_cal) ||
		(call1->get_cal != call2->get_cal) ||
		(call1->post_cal != call2->post_cal))
		ret = false;
	return ret;
}

int audio_cal_deregister(int num_cal_types,
			 struct audio_cal_reg *reg_data)
{
	int ret = 0;
	int i = 0;
	struct list_head *ptr, *next;
	struct audio_cal_client_info *client_info_node = NULL;

	pr_debug("%s\n", __func__);

	if (reg_data == NULL) {
		pr_err("%s: reg_data is NULL!\n", __func__);
		ret = -EINVAL;
		goto done;
	} else if ((num_cal_types <= 0) ||
		(num_cal_types > MAX_CAL_TYPES)) {
		pr_err("%s: num_cal_types of %d is Invalid!\n",
			__func__, num_cal_types);
		ret = -EINVAL;
		goto done;
	}

	for (; i < num_cal_types; i++) {
		if ((reg_data[i].cal_type < 0) ||
			(reg_data[i].cal_type >= MAX_CAL_TYPES)) {
			pr_err("%s: cal type %d at index %d is Invalid!\n",
				__func__, reg_data[i].cal_type, i);
			ret = -EINVAL;
			continue;
		}

		mutex_lock(&audio_cal.cal_mutex[reg_data[i].cal_type]);
		list_for_each_safe(ptr, next,
			&audio_cal.client_info[reg_data[i].cal_type]) {

			client_info_node = list_entry(ptr,
				struct audio_cal_client_info, list);
			if (callbacks_are_equal(client_info_node->callbacks,
				&reg_data[i].callbacks)) {
				list_del(&client_info_node->list);
				kfree(client_info_node->callbacks);
				client_info_node->callbacks = NULL;
				kfree(client_info_node);
				client_info_node = NULL;
				break;
			}
		}
		mutex_unlock(&audio_cal.cal_mutex[reg_data[i].cal_type]);
	}
done:
	return ret;
}


int audio_cal_register(int num_cal_types,
			 struct audio_cal_reg *reg_data)
{
	int ret = 0;
	int i = 0;
	struct audio_cal_client_info *client_info_node = NULL;
	struct audio_cal_callbacks *callback_node = NULL;

	pr_debug("%s\n", __func__);

	if (reg_data == NULL) {
		pr_err("%s: callbacks are NULL!\n", __func__);
		ret = -EINVAL;
		goto done;
	} else if ((num_cal_types <= 0) ||
		(num_cal_types > MAX_CAL_TYPES)) {
		pr_err("%s: num_cal_types of %d is Invalid!\n",
			__func__, num_cal_types);
		ret = -EINVAL;
		goto done;
	}

	for (; i < num_cal_types; i++) {
		if ((reg_data[i].cal_type < 0) ||
			(reg_data[i].cal_type >= MAX_CAL_TYPES)) {
			pr_err("%s: cal type %d at index %d is Invalid!\n",
				__func__, reg_data[i].cal_type, i);
			ret = -EINVAL;
			goto err;
		}

		client_info_node = kmalloc(sizeof(*client_info_node),
			GFP_KERNEL);
		if (client_info_node == NULL) {
			ret = -ENOMEM;
			goto err;
		}
		INIT_LIST_HEAD(&client_info_node->list);

		callback_node = kmalloc(sizeof(*callback_node),
			GFP_KERNEL);
		if (callback_node == NULL) {
			ret = -ENOMEM;
			goto err;
		}

		memcpy(callback_node, &reg_data[i].callbacks,
			sizeof(*callback_node));
		client_info_node->callbacks = callback_node;

		mutex_lock(&audio_cal.cal_mutex[reg_data[i].cal_type]);
		list_add_tail(&client_info_node->list,
			&audio_cal.client_info[reg_data[i].cal_type]);
		mutex_unlock(&audio_cal.cal_mutex[reg_data[i].cal_type]);
	}
done:
	return ret;
err:
	audio_cal_deregister(num_cal_types, reg_data);
	return ret;
}

static int call_allocs(int32_t cal_type,
				size_t cal_type_size, void *data)
{
	int ret = 0;
	int ret2 = 0;
	struct list_head *ptr, *next;
	struct audio_cal_client_info *client_info_node = NULL;

	pr_debug("%s\n", __func__);

	list_for_each_safe(ptr, next,
			&audio_cal.client_info[cal_type]) {

		client_info_node = list_entry(ptr,
			struct audio_cal_client_info, list);

		if (client_info_node->callbacks->alloc == NULL)
			continue;

		ret2 = client_info_node->callbacks->
			alloc(cal_type, cal_type_size, data);
		if (ret2 < 0) {
			pr_err("%s: alloc failed!\n", __func__);
			ret = ret2;
		}
	}
	return ret;
}

static int call_deallocs(int32_t cal_type,
				size_t cal_type_size, void *data)
{
	int ret = 0;
	int ret2 = 0;
	struct list_head *ptr, *next;
	struct audio_cal_client_info *client_info_node = NULL;

	pr_debug("%s cal type %d\n", __func__, cal_type);

	list_for_each_safe(ptr, next,
			&audio_cal.client_info[cal_type]) {

		client_info_node = list_entry(ptr,
			struct audio_cal_client_info, list);

		if (client_info_node->callbacks->dealloc == NULL)
			continue;

		ret2 = client_info_node->callbacks->
			dealloc(cal_type, cal_type_size, data);
		if (ret2 < 0) {
			pr_err("%s: dealloc failed!\n", __func__);
			ret = ret2;
		}
	}
	return ret;
}

static int call_pre_cals(int32_t cal_type,
				size_t cal_type_size, void *data)
{
	int ret = 0;
	int ret2 = 0;
	struct list_head *ptr, *next;
	struct audio_cal_client_info *client_info_node = NULL;

	pr_debug("%s cal type %d\n", __func__, cal_type);

	list_for_each_safe(ptr, next,
			&audio_cal.client_info[cal_type]) {

		client_info_node = list_entry(ptr,
			struct audio_cal_client_info, list);

		if (client_info_node->callbacks->pre_cal == NULL)
			continue;

		ret2 = client_info_node->callbacks->
			pre_cal(cal_type, cal_type_size, data);
		if (ret2 < 0) {
			pr_err("%s: pre_cal failed!\n", __func__);
			ret = ret2;
		}
	}
	return ret;
}

static int call_post_cals(int32_t cal_type,
				size_t cal_type_size, void *data)
{
	int ret = 0;
	int ret2 = 0;
	struct list_head *ptr, *next;
	struct audio_cal_client_info *client_info_node = NULL;

	pr_debug("%s cal type %d\n", __func__, cal_type);

	list_for_each_safe(ptr, next,
			&audio_cal.client_info[cal_type]) {

		client_info_node = list_entry(ptr,
			struct audio_cal_client_info, list);

		if (client_info_node->callbacks->post_cal == NULL)
			continue;

		ret2 = client_info_node->callbacks->
			post_cal(cal_type, cal_type_size, data);
		if (ret2 < 0) {
			pr_err("%s: post_cal failed!\n", __func__);
			ret = ret2;
		}
	}
	return ret;
}

static int call_set_cals(int32_t cal_type,
				size_t cal_type_size, void *data)
{
	int ret = 0;
	int ret2 = 0;
	struct list_head *ptr, *next;
	struct audio_cal_client_info *client_info_node = NULL;

	pr_debug("%s cal type %d\n", __func__, cal_type);

	list_for_each_safe(ptr, next,
			&audio_cal.client_info[cal_type]) {

		client_info_node = list_entry(ptr,
			struct audio_cal_client_info, list);

		if (client_info_node->callbacks->set_cal == NULL)
			continue;

		ret2 = client_info_node->callbacks->
			set_cal(cal_type, cal_type_size, data);
		if (ret2 < 0) {
			pr_err("%s: set_cal failed!\n", __func__);
			ret = ret2;
		}
	}
	return ret;
}

static int call_get_cals(int32_t cal_type,
				size_t cal_type_size, void *data)
{
	int ret = 0;
	int ret2 = 0;
	struct list_head *ptr, *next;
	struct audio_cal_client_info *client_info_node = NULL;

	pr_debug("%s cal type %d\n", __func__, cal_type);

	list_for_each_safe(ptr, next,
			&audio_cal.client_info[cal_type]) {

		client_info_node = list_entry(ptr,
			struct audio_cal_client_info, list);

		if (client_info_node->callbacks->get_cal == NULL)
			continue;

		ret2 = client_info_node->callbacks->
			get_cal(cal_type, cal_type_size, data);
		if (ret2 < 0) {
			pr_err("%s: get_cal failed!\n", __func__);
			ret = ret2;
		}
	}
	return ret;
}

static int audio_cal_open(struct inode *inode, struct file *f)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	mutex_lock(&audio_cal.common_lock);
	audio_cal.ref_count++;
	mutex_unlock(&audio_cal.common_lock);

	return ret;
}

static void dealloc_all_clients(void)
{
	int i = 0;
	struct audio_cal_type_dealloc dealloc_data;

	pr_debug("%s\n", __func__);

	dealloc_data.cal_hdr.version = VERSION_0_0;
	dealloc_data.cal_hdr.buffer_number = ALL_CAL_BLOCKS;
	dealloc_data.cal_data.mem_handle = -1;

	for (; i < MAX_CAL_TYPES; i++)
		call_deallocs(i, sizeof(dealloc_data), &dealloc_data);
}

static int audio_cal_release(struct inode *inode, struct file *f)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	mutex_lock(&audio_cal.common_lock);
	audio_cal.ref_count--;
	if (audio_cal.ref_count <= 0) {
		audio_cal.ref_count = 0;
		dealloc_all_clients();
	}
	mutex_unlock(&audio_cal.common_lock);

	return ret;
}

static void check_audio_and_set_pmode(void)
{
	if ((audio_mode == AUDIO_DRV_MODE_IN_CALL || audio_mode == AUDIO_DRV_MODE_IN_COMMUNICATION) &&
			rcv_device == AUDIO_DRV_DEVICE_RECEIVER) {
		/* fts_set_pmode(true); */
		printk("fts_set_pmode to true\n");
	} else {
		/* fts_set_pmode(false); */
		printk("fts_set_pmode to false\n");
	}
}

/* ASUS_BSP +++ Add uevent to IMS for Line audio_mode = 0 when Line VoIP incall */
static void send_rcv_notification_uevent(int rcvdev)
{
        if (rcv_notification_kobj) {
		char uevent_buf[512];
		char *envp[] = { uevent_buf, NULL };
		if (rcvdev == 1) {
			snprintf(uevent_buf, sizeof(uevent_buf), "PHONE_RECEIVER_NOTIFICATION=%d", active_outputpid);
			printk("PHONE_RECEIVER_NOTIFICATION=%d, RCV\n", active_outputpid);
		} else {
			snprintf(uevent_buf, sizeof(uevent_buf), "PHONE_RECEIVER_NOTIFICATION=0");
			printk("PHONE_RECEIVER_NOTIFICATION=0, not RCV\n");
		}
		kobject_uevent_env(rcv_notification_kobj, KOBJ_CHANGE, envp);
	}
}

static void rcv_notification_uevent_release(struct kobject *kobj)
{
	kfree(kobj);
}

static struct kobj_type rcv_notification_uevent_ktype = {
	.release = rcv_notification_uevent_release,
};

static int rcv_notification_uevent_init(void)
{
	int ret;

	rcv_notification_uevent_kset = kset_create_and_add("rcv_notification_uevent", NULL, kernel_kobj);
	if (!rcv_notification_uevent_kset) {
		pr_err("%s: failed to create rcv_notification_uevent_kset", __func__);
		return -ENOMEM;
	}
	rcv_notification_kobj = kzalloc(sizeof(*rcv_notification_kobj), GFP_KERNEL);
	if (!rcv_notification_kobj) {
		pr_err("%s: failed to create rcv_notification_kobj", __func__);
		return -ENOMEM;
	}

	rcv_notification_kobj->kset = rcv_notification_uevent_kset;

	ret = kobject_init_and_add(rcv_notification_kobj, &rcv_notification_uevent_ktype, NULL, "audio_rcv_notification");
	if (ret) {
		pr_err("%s: failed to init rcv_notification_kobj", __func__);
		kobject_put(rcv_notification_kobj);
		return -EINVAL;
	}

	kobject_uevent(rcv_notification_kobj, KOBJ_ADD);

	return 0;
}
/* ASUS_BSP --- Add uevent to IMS for Line audio_mode = 0 when Line VoIP incall */

static long audio_cal_shared_ioctl(struct file *file, unsigned int cmd,
							void __user *arg)
{
	int ret = 0;
	int32_t size;
	/* ASUS_BSP +++ Add warning uevent for input occupied issue ( TT1290090 ) */
	int activeinputpid = 0;
	int failedinputpid = 0;
	/* ASUS_BSP --- */
	struct audio_cal_basic *data = NULL;

        int audiowizard_force_channel = 2;//Jessy +++ AudioWizard muti-channel mode

	pr_debug("%s\n", __func__);

	switch (cmd) {
	case AUDIO_ALLOCATE_CALIBRATION:
	case AUDIO_DEALLOCATE_CALIBRATION:
	case AUDIO_PREPARE_CALIBRATION:
	case AUDIO_SET_CALIBRATION:
	case AUDIO_GET_CALIBRATION:
	case AUDIO_POST_CALIBRATION:
		break;
//Jessy +++ AudioWizard hifi & ringtone mode
        case AUDIO_SET_AUDIOWIZARD_FORCE_PRESET:
            mutex_lock(&audio_cal.cal_mutex[AUDIOWIZARD_FORCE_PRESET_TYPE]);
            printk("audio_cal_shared_ioctl AUDIO_SET_AUDIOWIZARD_FORCE_PRESET ");
                if (copy_from_user(&g_audiowizard_force_preset_state, (void *)arg,
                        sizeof(g_audiowizard_force_preset_state))) {
                    pr_err("%s: Could not copy g_audiowizard_force_preset_state from user\n", __func__);
                    ret = -EFAULT;
                }
            printk("audio_cal_shared_ioctl AUDIO_SET_AUDIOWIZARD_FORCE_PRESET g_audiowizard_force_preset_state:%d",g_audiowizard_force_preset_state);
            send_audiowizard_state(audiowizard,g_audiowizard_force_preset_state);
            input_sync(audiowizard);
            mutex_unlock(&audio_cal.cal_mutex[AUDIOWIZARD_FORCE_PRESET_TYPE]);
            goto done;
//Jessy ---

//Jessy +++ AudioWizard muti-channel mode
        case AUDIO_SET_AUDIOWIZARD_FORCE_CHANNEL:
            mutex_lock(&audio_cal.cal_mutex[AUDIOWIZARD_FORCE_CHANNEL_TYPE]);
            if (copy_from_user(&audiowizard_force_channel, (void *)arg,
                    sizeof(audiowizard_force_channel))) {
                pr_err("%s: Could not copy audiowizard_channel from user\n", __func__);
                ret = -EFAULT;
            }
            printk("audio_cal_shared_ioctl AUDIO_SET_AUDIOWIZARD_FORCE_CHANNEL audiowizard_force_channel:%d",audiowizard_force_channel);
            send_audiowizard_channel(audiowizard_channel,audiowizard_force_channel);
            input_sync(audiowizard_channel);
            mutex_unlock(&audio_cal.cal_mutex[AUDIOWIZARD_FORCE_CHANNEL_TYPE]);
            goto done;
//Jessy ---

	/* ASUS_BSP +++ Add warning uevent for input occupied issue ( TT1290090 ) */
	case AUDIO_SET_ACTIVEINPUT_PID:
		mutex_lock(&audio_cal.cal_mutex[AUDIO_SET_ACTIVEINPUT_PID_TYPE]);
		if (copy_from_user(&activeinputpid, (void *)arg, sizeof(activeinputpid))) {
			pr_err("%s: Could not copy state from user\n", __func__);
			ret = -EFAULT;
		}
		if (copy_from_user(&failedinputpid, (void *)(arg + sizeof(activeinputpid)), sizeof(failedinputpid))) {
			pr_err("%s: Could not copy state from user\n", __func__);
			ret = -EFAULT;
		}
		printk("activeinputpid=%d, failedinputpid=%d\n", activeinputpid, failedinputpid);
		send_activeinput_pid_uevent(activeinputpid, failedinputpid);
		mutex_unlock(&audio_cal.cal_mutex[AUDIO_SET_ACTIVEINPUT_PID_TYPE]);
		goto done;
	/* ASUS_BSP --- */

	/* ASUS_BSP +++ Add uevent for receiver checking */
	case AUDIO_SET_ACTIVEOUTPUT_PID:
		mutex_lock(&audio_cal.cal_mutex[AUDIO_SET_ACTIVEOUTPUT_PID_TYPE]);
		if (copy_from_user(&active_outputpid, (void *)arg, sizeof(active_outputpid))) {
			pr_err("%s: Could not copy state from user\n", __func__);
			ret = -EFAULT;
		}
		printk("active_outputpid=%d\n", active_outputpid);
		if (active_outputpid == 0) {
			/* previous outputpid's track was removed */
			send_rcv_notification_uevent(0);
		} else {
			/* outputpid's track is added */
			send_rcv_notification_uevent(rcv_device);
		}
		mutex_unlock(&audio_cal.cal_mutex[AUDIO_SET_ACTIVEOUTPUT_PID_TYPE]);
		goto done;
	/* ASUS_BSP --- */

	/* ASUS_BSP +++ Audio mode */
	case AUDIO_SET_MODE:
		mutex_lock(&audio_cal.cal_mutex[SET_MODE_TYPE]);
		if(copy_from_user(&audmode, (void *)arg,sizeof(audmode))) {
			pr_err("%s: Could not copy lmode to user\n", __func__);
			ret = -EFAULT;
		}

		audio_mode = audmode;
		check_audio_and_set_pmode();
		printk("%s: Audio mode status:audio_mode=%d\n", __func__, audio_mode);
		mutex_unlock(&audio_cal.cal_mutex[SET_MODE_TYPE]);
		goto done;
	/* ASUS_BSP --- */

	/* ASUS_BSP +++ Audio RCV device */
	case AUDIO_SET_RCV_DEVICE:
		mutex_lock(&audio_cal.cal_mutex[SET_RCV_DEVICE_TYPE]);
		if(copy_from_user(&rcvdev, (void *)arg,sizeof(rcvdev))) {
			pr_err("%s: Could not copy lmode to user\n", __func__);
			ret = -EFAULT;
		}

		rcv_device = rcvdev;
		/* check_audio_and_set_pmode(); */
		send_rcv_notification_uevent(rcv_device);
		printk("%s: Audio device status:rcv_device=%d\n", __func__, rcv_device);
		mutex_unlock(&audio_cal.cal_mutex[SET_RCV_DEVICE_TYPE]);
		goto done;
	/* ASUS_BSP --- */

	default:
		pr_err("%s: ioctl not found!\n", __func__);
		ret = -EFAULT;
		goto done;
	}

	if (copy_from_user(&size, (void *)arg, sizeof(size))) {
		pr_err("%s: Could not copy size value from user\n", __func__);
		ret = -EFAULT;
		goto done;
	} else if ((size < sizeof(struct audio_cal_basic))
		|| (size > MAX_IOCTL_CMD_SIZE)) {
		pr_err("%s: Invalid size sent to driver: %d, max size is %d, min size is %zd\n",
			__func__, size, MAX_IOCTL_CMD_SIZE,
			sizeof(struct audio_cal_basic));
		ret = -EINVAL;
		goto done;
	}

	data = kmalloc(size, GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto done;
	} else if (copy_from_user(data, (void *)arg, size)) {
		pr_err("%s: Could not copy data from user\n",
			__func__);
		ret = -EFAULT;
		goto done;
	} else if ((data->hdr.cal_type < 0) ||
		(data->hdr.cal_type >= MAX_CAL_TYPES)) {
		pr_err("%s: cal type %d is Invalid!\n",
			__func__, data->hdr.cal_type);
		ret = -EINVAL;
		goto done;
	} else if ((data->hdr.cal_type_size <
		sizeof(struct audio_cal_type_basic)) ||
		(data->hdr.cal_type_size >
		get_user_cal_type_size(data->hdr.cal_type))) {
		pr_err("%s: cal type size %d is Invalid! Max is %zd!\n",
			__func__, data->hdr.cal_type_size,
			get_user_cal_type_size(data->hdr.cal_type));
		ret = -EINVAL;
		goto done;
	} else if (data->cal_type.cal_hdr.buffer_number < 0) {
		pr_err("%s: cal type %d Invalid buffer number %d!\n",
			__func__, data->hdr.cal_type,
			data->cal_type.cal_hdr.buffer_number);
		ret = -EINVAL;
		goto done;
	} else if ((data->hdr.cal_type_size + sizeof(data->hdr)) > size) {
		pr_err("%s: cal type hdr size %zd + cal type size %d is greater than user buffer size %d\n",
			__func__, sizeof(data->hdr), data->hdr.cal_type_size,
			size);
		ret = -EFAULT;
		goto done;
	}


	mutex_lock(&audio_cal.cal_mutex[data->hdr.cal_type]);

	switch (cmd) {
	case AUDIO_ALLOCATE_CALIBRATION:
		ret = call_allocs(data->hdr.cal_type,
			data->hdr.cal_type_size, &data->cal_type);
		break;
	case AUDIO_DEALLOCATE_CALIBRATION:
		ret = call_deallocs(data->hdr.cal_type,
			data->hdr.cal_type_size, &data->cal_type);
		break;
	case AUDIO_PREPARE_CALIBRATION:
		ret = call_pre_cals(data->hdr.cal_type,
			data->hdr.cal_type_size, &data->cal_type);
		break;
	case AUDIO_SET_CALIBRATION:
		ret = call_set_cals(data->hdr.cal_type,
			data->hdr.cal_type_size, &data->cal_type);
		break;
	case AUDIO_GET_CALIBRATION:
		ret = call_get_cals(data->hdr.cal_type,
			data->hdr.cal_type_size, &data->cal_type);
		break;
	case AUDIO_POST_CALIBRATION:
		ret = call_post_cals(data->hdr.cal_type,
			data->hdr.cal_type_size, &data->cal_type);
		break;
	}

	if (cmd == AUDIO_GET_CALIBRATION) {
		if (data->hdr.cal_type_size == 0)
			goto unlock;
		if (data == NULL)
			goto unlock;
		if (copy_to_user(arg, data,
			sizeof(data->hdr) + data->hdr.cal_type_size)) {
			pr_err("%s: Could not copy cal type to user\n",
				__func__);
			ret = -EFAULT;
			goto unlock;
		}
	}

unlock:
	mutex_unlock(&audio_cal.cal_mutex[data->hdr.cal_type]);
done:
	kfree(data);
	return ret;
}

/* ASUS_BSP +++ */
int get_audiomode(void)
{
    printk("%s: Audio mode=%d\n",__func__, audio_mode);
    return audio_mode;
}
EXPORT_SYMBOL(get_audiomode);

int get_audio_rcv_device(void)
{
    printk("%s: Audio RCV device=%d\n",__func__, rcv_device);
    return rcv_device;
}
EXPORT_SYMBOL(get_audio_rcv_device);
/* ASUS_BSP --- */

static long audio_cal_ioctl(struct file *f,
		unsigned int cmd, unsigned long arg)
{
	return audio_cal_shared_ioctl(f, cmd, (void __user *)arg);
}

#ifdef CONFIG_COMPAT

#define AUDIO_ALLOCATE_CALIBRATION32	_IOWR(CAL_IOCTL_MAGIC, \
							200, compat_uptr_t)
#define AUDIO_DEALLOCATE_CALIBRATION32	_IOWR(CAL_IOCTL_MAGIC, \
							201, compat_uptr_t)
#define AUDIO_PREPARE_CALIBRATION32	_IOWR(CAL_IOCTL_MAGIC, \
							202, compat_uptr_t)
#define AUDIO_SET_CALIBRATION32		_IOWR(CAL_IOCTL_MAGIC, \
							203, compat_uptr_t)
#define AUDIO_GET_CALIBRATION32		_IOWR(CAL_IOCTL_MAGIC, \
							204, compat_uptr_t)
#define AUDIO_POST_CALIBRATION32	_IOWR(CAL_IOCTL_MAGIC, \
							205, compat_uptr_t)

//Jessy +++ AudioWizard hifi & ringtone mode
#define AUDIO_SET_AUDIOWIZARD_FORCE_PRESET32	_IOWR(CAL_IOCTL_MAGIC, \
							221, compat_uptr_t)
//Jessy ---

//Jessy +++ AudioWizard muti-channel mode
#define AUDIO_SET_AUDIOWIZARD_FORCE_CHANNEL32	_IOWR(CAL_IOCTL_MAGIC, \
							222, compat_uptr_t)
//Jessy ---

/* ASUS_BSP +++ Add warning uevent for input occupied issue ( TT1290090 ) */
#define AUDIO_SET_ACTIVEINPUT_PID32 _IOWR(CAL_IOCTL_MAGIC, \
							232,compat_uptr_t)
/* ASUS_BSP --- */

/* ASUS_BSP +++ Audio mode and device */
#define AUDIO_SET_MODE32		_IOWR(CAL_IOCTL_MAGIC, \
							225, compat_uptr_t)
#define AUDIO_SET_RCV_DEVICE32		_IOWR(CAL_IOCTL_MAGIC, \
							233, compat_uptr_t)
#define AUDIO_SET_ACTIVEOUTPUT_PID32 _IOWR(CAL_IOCTL_MAGIC, \
                                                        234,compat_uptr_t)
/* ASUS_BSP --- Audio mode and device */

static long audio_cal_compat_ioctl(struct file *f,
		unsigned int cmd, unsigned long arg)
{
	unsigned int cmd64;
	int ret = 0;

	switch (cmd) {
	case AUDIO_ALLOCATE_CALIBRATION32:
		cmd64 = AUDIO_ALLOCATE_CALIBRATION;
		break;
	case AUDIO_DEALLOCATE_CALIBRATION32:
		cmd64 = AUDIO_DEALLOCATE_CALIBRATION;
		break;
	case AUDIO_PREPARE_CALIBRATION32:
		cmd64 = AUDIO_PREPARE_CALIBRATION;
		break;
	case AUDIO_SET_CALIBRATION32:
		cmd64 = AUDIO_SET_CALIBRATION;
		break;
	case AUDIO_GET_CALIBRATION32:
		cmd64 = AUDIO_GET_CALIBRATION;
		break;
	case AUDIO_POST_CALIBRATION32:
		cmd64 = AUDIO_POST_CALIBRATION;
		break;
//Jessy +++ AudioWizard hifi & ringtone mode
	case AUDIO_SET_AUDIOWIZARD_FORCE_PRESET32:
		cmd64 = AUDIO_SET_AUDIOWIZARD_FORCE_PRESET;
		break;
//Jessy ---

//Jessy +++ AudioWizard muti-channel mode
	case AUDIO_SET_AUDIOWIZARD_FORCE_CHANNEL32:
		cmd64 = AUDIO_SET_AUDIOWIZARD_FORCE_CHANNEL;
		break;
//Jessy ---

/* ASUS_BSP +++ Add warning uevent for input occupied issue ( TT1290090 ) */
	case AUDIO_SET_ACTIVEINPUT_PID32:
		cmd64 = AUDIO_SET_ACTIVEINPUT_PID;
		break;
/* ASUS_BSP --- */

/* ASUS_BSP +++ Audio mode and device */
	case AUDIO_SET_MODE32:
		cmd64 = AUDIO_SET_MODE;
		break;
	case AUDIO_SET_RCV_DEVICE32:
		cmd64 = AUDIO_SET_RCV_DEVICE;
		break;
	case AUDIO_SET_ACTIVEOUTPUT_PID32:
		cmd64 = AUDIO_SET_ACTIVEOUTPUT_PID;
		break;
/* ASUS_BSP --- Audio mode and device */
	default:
		pr_err("%s: ioctl not found!\n", __func__);
		ret = -EFAULT;
		goto done;
	}

	ret = audio_cal_shared_ioctl(f, cmd64, compat_ptr(arg));
done:
	return ret;
}
#endif

static const struct file_operations audio_cal_fops = {
	.owner = THIS_MODULE,
	.open = audio_cal_open,
	.release = audio_cal_release,
	.unlocked_ioctl = audio_cal_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl =   audio_cal_compat_ioctl,
#endif
};

struct miscdevice audio_cal_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_audio_cal",
	.fops	= &audio_cal_fops,
};

/* ASUS_BSP +++ Add warning uevent for input occupied issue ( TT1290090 ) */
static void send_activeinput_pid_uevent(int activeinputpid, int failedinputpid)
{
	if (activeinputpid_kobj) {
		char uevent_buf1[512];
		char uevent_buf2[512];
		char *envp[] = { uevent_buf1, uevent_buf2, NULL };
		snprintf(uevent_buf1, sizeof(uevent_buf1), "ACTIVEINPUT_PID=%d", activeinputpid);
		snprintf(uevent_buf2, sizeof(uevent_buf2), "FAILEDINPUT_PID=%d", failedinputpid);
		kobject_uevent_env(activeinputpid_kobj, KOBJ_CHANGE, envp);
	}
}

static void activeinputpid_uevent_release(struct kobject *kobj)
{
	kfree(kobj);
}

static struct kobj_type activeinputpid_uevent_ktype = {
	.release = activeinputpid_uevent_release,
};

static int activeinputpid_uevent_init(void)
{
	int ret;

	activeinputpid_uevent_kset = kset_create_and_add("activeinputpid_uevent", NULL, kernel_kobj);
	if (!activeinputpid_uevent_kset) {
		pr_err("%s: failed to create activeinputpid_uevent_kset", __func__);
		return -ENOMEM;
	}
	activeinputpid_kobj = kzalloc(sizeof(*activeinputpid_kobj), GFP_KERNEL);
	if (!activeinputpid_kobj) {
		pr_err("%s: failed to create activeinputpid_kobj", __func__);
		return -ENOMEM;
	}

	activeinputpid_kobj->kset = activeinputpid_uevent_kset;

	ret = kobject_init_and_add(activeinputpid_kobj, &activeinputpid_uevent_ktype, NULL, "audio_activeinputpid");
	if (ret) {
		pr_err("%s: failed to init activeinputpid_kobj", __func__);
		kobject_put(activeinputpid_kobj);
		return -EINVAL;
	}

	kobject_uevent(activeinputpid_kobj, KOBJ_ADD);

	return 0;
}
/* ASUS_BSP --- */

int __init audio_cal_init(void)
{
	int i = 0;
//Jessy +++ AudioWizard hifi & ringtone mode
       int ret =0;
//Jessy ---

	pr_debug("%s\n", __func__);

//Jessy +++ AudioWizard hifi & ringtone mode
	audiowizard = input_allocate_device();
	if(!audiowizard)
		pr_err("%s: failed to allocate inputevent audiowizard\n", __func__);
	audiowizard->name = "audiowizard";
	input_set_capability(audiowizard,EV_SW,0x0b);
	input_set_capability(audiowizard,EV_SW,0x0d);
	ret = input_register_device(audiowizard);
	if (ret<0)
		pr_err("%s: failed to register inputevent audiowizard\n", __func__);
//Jessy ---

//Jessy +++ AudioWizard muti-channel mode
	audiowizard_channel= input_allocate_device();
	if(!audiowizard_channel)
		pr_err("%s: failed to allocate inputevent audiowizard_channel\n", __func__);
	audiowizard_channel->name = "audiowizard_channel";
	input_set_capability(audiowizard_channel,EV_SW,SW_AUDIOWIZARD_CHANNELS_1);
	input_set_capability(audiowizard_channel,EV_SW,SW_AUDIOWIZARD_CHANNELS_2);
    	input_set_capability(audiowizard_channel,EV_SW,SW_AUDIOWIZARD_CHANNELS_M);
	ret = input_register_device(audiowizard_channel);
	if (ret<0)
		pr_err("%s: failed to register inputevent audiowizard_channel\n", __func__);
//Jessy ---

/* ASUS_BSP +++ Add warning uevent for input occupied issue ( TT1290090 ) */
	activeinputpid_uevent_init();
/* ASUS_BSP --- */

/* ASUS_BSP +++ Add uevent to IMS for Line audio_mode = 0 when Line VoIP incall */
	rcv_notification_uevent_init();
/* ASUS_BSP --- Add uevent to IMS for Line audio_mode = 0 when Line VoIP incall */

	memset(&audio_cal, 0, sizeof(audio_cal));
	mutex_init(&audio_cal.common_lock);
	for (; i < MAX_CAL_TYPES; i++) {
		INIT_LIST_HEAD(&audio_cal.client_info[i]);
		mutex_init(&audio_cal.cal_mutex[i]);
	}

	return misc_register(&audio_cal_misc);
}

void audio_cal_exit(void)
{
	int i = 0;
	struct list_head *ptr, *next;
	struct audio_cal_client_info *client_info_node;

//Jessy +++ AudioWizard hifi & ringtone mode
    input_free_device(audiowizard);
//Jessy ---

//Jessy +++ AudioWizard muti-channel mode
    input_free_device(audiowizard_channel);
//Jessy ---

	for (; i < MAX_CAL_TYPES; i++) {
		list_for_each_safe(ptr, next,
			&audio_cal.client_info[i]) {
			client_info_node = list_entry(ptr,
				struct audio_cal_client_info, list);
			list_del(&client_info_node->list);
			kfree(client_info_node->callbacks);
			client_info_node->callbacks = NULL;
			kfree(client_info_node);
			client_info_node = NULL;
		}
	}
	misc_deregister(&audio_cal_misc);
}


MODULE_DESCRIPTION("SoC QDSP6v2 Audio Calibration driver");
MODULE_LICENSE("GPL v2");
