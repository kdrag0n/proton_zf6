#include "asus_flash.h"
#include "cam_flash_core.h"
#include <linux/proc_fs.h>

#undef  pr_fmt
#define pr_fmt(fmt) "FLASH-ATD %s() " fmt, __func__

#define PROC_ATD_FLASH1 "driver/asus_flash"
#define PROC_ATD_FLASH2 "driver/asus_flash2"

#define PROC_ATD_STATUS "driver/flash_status"

#define PROC_CTRL1_LEDS "driver/ctrl1_leds"

#define BATTERY_CAPACITY "/sys/class/power_supply/bms/capacity"
#define LOW_BATTERY_THRESHOLD 20

#define MAX_CTRL 2

#define ATD_TORCH_CURRENT 200
#define ATD_FLASH_CURRENT 1000

typedef struct
{
	int (*led_init)(uint32_t id);
	int (*led_release)(uint32_t id);
	int (*led_off)(uint32_t id);
	int (*led_high)(uint32_t id, uint32_t current1, uint32_t current2);
	int (*led_low)(uint32_t id, uint32_t current1, uint32_t current2);
}asus_flash_function_table_t;

static asus_flash_function_table_t g_func_tbl;
static struct cam_flash_ctrl *g_fctrl[MAX_CTRL];//ctrl for flash
static uint8_t g_flash_index[MAX_CTRL] = {1,2};
static uint8_t g_ATD_status = 0;

static int g_error_value = 0;
static uint8_t g_camera_in_use = 0;//used to avoid control led when camera open

static const char * get_state_string(enum cam_flash_state state)
{
	switch(state)
	{
		case CAM_FLASH_STATE_INIT:
			 return "INIT";
		case CAM_FLASH_STATE_ACQUIRE:
			 return "ACQUIRE";
		case CAM_FLASH_STATE_CONFIG:
			 return "CONFIG";
		case CAM_FLASH_STATE_START:
			 return "START";
		default:
			 return "UNKNOWN";
	}
}

static int fs_read_uint8(char *filename, uint8_t *value)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[5];
	ssize_t read_size = 0;
	mm_segment_t old_fs;

	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("file (%s) not exist!\n",filename);
		return -ENOENT;	/*No such file or directory*/
	}

	pos_lsts = 0;

	old_fs = get_fs();
	set_fs(get_ds());
	memset(buf,0,sizeof(buf));
	read_size = vfs_read(fp, (void __user *)buf, sizeof(buf), &pos_lsts);
	set_fs(old_fs);

	if(read_size < 1) {
		pr_err("read failed!\n");
		/* close file */
		filp_close(fp, NULL);
		return -1;
	}
	else
	{
		pr_info("read size is %ld\n",read_size);
	}

	sscanf(buf, "%hhu", value);

	/* close file */
	filp_close(fp, NULL);

	return 0;
}

int asus_flash_is_battery_low(void)
{
	uint8_t capacity=0;
	if(fs_read_uint8(BATTERY_CAPACITY,&capacity) == 0)
	{
		if(capacity <= LOW_BATTERY_THRESHOLD)
		{
			pr_info("battery %d%%, low\n",capacity);
			return 1;
		}
	}
	else
	{
		pr_err("get battery capacity failed!\n");
	}
	return 0;
}

static int asus_led_init(uint32_t id)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl;

	if(id >= MAX_CTRL)
	{
		pr_err("invalid flash id %d!\n",id);
		return -EINVAL;
	}

	fctrl = g_fctrl[id];
	if(fctrl == NULL)
	{
		pr_err("fctrl for id %d is NULL!\n",id);
		return -EINVAL;
	}

	mutex_lock(&(fctrl->flash_mutex));
	if(!fctrl->is_regulator_enabled)
	{
		rc = cam_flash_prepare(fctrl, true);
	}
	else
	{
		pr_info("regulator already enabled, do nothing\n");
	}
	mutex_unlock(&(fctrl->flash_mutex));
	return rc;
}

static int asus_led_release(uint32_t id)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl;

	if(id >= MAX_CTRL)
	{
		pr_err("invalid flash id %d!\n",id);
		return -EINVAL;
	}

	fctrl = g_fctrl[id];
	if(fctrl == NULL)
	{
		pr_err("fctrl for id %d is NULL!\n",id);
		return -EINVAL;
	}

	mutex_lock(&(fctrl->flash_mutex));
	if(fctrl->is_regulator_enabled)
	{
		rc = cam_flash_prepare(fctrl, false);
	}
	else
	{
		pr_info("regulator already disabled, do nothing\n");
	}
	mutex_unlock(&(fctrl->flash_mutex));
	return rc;
}

static int asus_led_off(uint32_t id)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl;

	if(id >= MAX_CTRL)
	{
		pr_err("invalid flash id %d!\n",id);
		return -EINVAL;
	}

	fctrl = g_fctrl[id];
	if(fctrl == NULL)
	{
		pr_err("fctrl for id %d is NULL!\n",id);
		return -EINVAL;
	}

	mutex_lock(&(fctrl->flash_mutex));
	if(fctrl->is_regulator_enabled)
	{
		rc = cam_flash_off(fctrl);
	}
	else
	{
		pr_info("flash state is %d -> %s, regulator disabled, not call off\n",
				fctrl->flash_state,get_state_string(fctrl->flash_state));
		rc = -1;
	}
	mutex_unlock(&(fctrl->flash_mutex));
	return rc;
}

static int asus_led_high(uint32_t id, uint32_t current1, uint32_t current2)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl;
	struct cam_flash_frame_setting setting;

	if(id >= MAX_CTRL)
	{
		pr_err("invalid flash id %d!\n",id);
		return -EINVAL;
	}

	fctrl = g_fctrl[id];
	if(fctrl == NULL)
	{
		pr_err("fctrl for id %d is NULL!\n",id);
		return -EINVAL;
	}

	setting.led_current_ma[0] = current1;
	setting.led_current_ma[1] = current2;
	setting.led_current_ma[2] = 0;

	mutex_lock(&(fctrl->flash_mutex));
	if(fctrl->is_regulator_enabled)
	{
		rc = cam_flash_high(fctrl,&setting);
	}
	else
	{
		pr_info("flash state is %d -> %s, regulator disabled, not call high\n",
				fctrl->flash_state,get_state_string(fctrl->flash_state));
		rc = -1;
	}
	mutex_unlock(&(fctrl->flash_mutex));
	return rc;
}

static int asus_led_low(uint32_t id, uint32_t current1, uint32_t current2)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl;
	struct cam_flash_frame_setting setting;

	if(id >= MAX_CTRL)
	{
		pr_err("invalid flash id %d!\n",id);
		return -EINVAL;
	}

	fctrl = g_fctrl[id];
	if(fctrl == NULL)
	{
		pr_err("fctrl for id %d is NULL!\n",id);
		return -EINVAL;
	}

	setting.led_current_ma[0] = current1;
	setting.led_current_ma[1] = current2;
	setting.led_current_ma[2] = 0;

	mutex_lock(&(fctrl->flash_mutex));
	if(fctrl->is_regulator_enabled)
	{
		rc = cam_flash_low(fctrl,&setting);
	}
	else
	{
		pr_info("flash state is %d -> %s, regulator disabled, not call low\n",
				fctrl->flash_state,get_state_string(fctrl->flash_state));
		rc = -1;
	}
	mutex_unlock(&(fctrl->flash_mutex));
	return rc;
}

void asus_flash_set_camera_state(uint8_t in_use)
{
	g_camera_in_use = in_use;
	pr_info("Camera use flash %d\n",g_camera_in_use);
}

void asus_flash_set_led_fault(int error_value)
{
	g_error_value = error_value;
	pr_err("ERROR occured! value is 0x%x -> %d\n",g_error_value,g_error_value);
}

static ssize_t status_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
	int n;
	char kbuf[8];

	if(*ppos == 0)
	{
		n = snprintf(kbuf, sizeof(kbuf),"%d\n%c", g_ATD_status,'\0');
		if(copy_to_user(buffer,kbuf,n))
		{
			pr_err("%s(): copy_to_user fail !\n",__func__);
			return -EFAULT;
		}
		*ppos += n;
		g_ATD_status = 0;
		return n;
	}
	return 0;
}

static ssize_t flash_led_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
	ssize_t ret=0;
	char buff[1024];
	ssize_t desc = 0;

	struct cam_flash_ctrl *fctrl;
	struct cam_flash_private_soc *soc_private;

	uint8_t* index_data = (uint8_t*)PDE_DATA(file_inode(dev));
	uint8_t  index = *index_data -1;

	if(index == 0 || index == 1)
	{
		fctrl = g_fctrl[0];
	}
	else
		fctrl = NULL;

	if(fctrl == NULL)
	{
		pr_err("fctrl for led index %d is NULL!\n",*index_data);
		return 0;
	}

	soc_private = (struct cam_flash_private_soc *)fctrl->soc_info.soc_private;

	desc+=sprintf(buff+desc, "flash_num_sources: %d\n",fctrl->flash_num_sources);
	desc+=sprintf(buff+desc, "torch_num_sources: %d\n",fctrl->torch_num_sources);
	desc+=sprintf(buff+desc, "flash_state: %d %s\n",fctrl->flash_state, get_state_string(fctrl->flash_state));

	desc+=sprintf(buff+desc, "switch_trigger_name: %s\n", soc_private->switch_trigger_name);
	desc+=sprintf(buff+desc, "flash_trigger_name: %s\n", soc_private->flash_trigger_name[index]);
	desc+=sprintf(buff+desc, "flash_op_current: %d\n",soc_private->flash_op_current[index]);
	desc+=sprintf(buff+desc, "flash_max_current: %d\n",soc_private->flash_max_current[index]);
	desc+=sprintf(buff+desc, "flash_max_duration: %d\n",soc_private->flash_max_duration[index]);
	desc+=sprintf(buff+desc, "torch_trigger_name: %s\n", soc_private->torch_trigger_name[index]);
	desc+=sprintf(buff+desc, "torch_op_current: %d\n",soc_private->torch_op_current[index]);
	desc+=sprintf(buff+desc, "torch_max_current: %d\n",soc_private->torch_max_current[index]);

	ret = simple_read_from_buffer(buffer,count,ppos,buff,desc);
	return ret;
}

static ssize_t flash_led_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	char kbuf[8];
	int  mode = -1;
	int  on = -1;
	int  rc = 0;
	int  val = 0;

	ssize_t ret_len = count;

	uint8_t* index = (uint8_t*)PDE_DATA(file_inode(dev));
	uint32_t id = 0;


	if(count > 8)
		count = 8;

	if(copy_from_user(kbuf, buf, count))
	{
		pr_err("%s(): copy_from_user fail !\n",__func__);
		return -EFAULT;
	}

	rc=sscanf(kbuf, "%d %d %d", &mode, &on, &val);

	pr_info("node %d, command %d %d %d rc=%d\n",*index, mode, on, val,rc);

	g_error_value = 0;
	g_ATD_status = 0;

	g_func_tbl.led_init(id);

	if(mode == 0)//torch
	{
		if(on == 1)
		{
			val = val==0 ? ATD_TORCH_CURRENT : val;
			val = val > ATD_TORCH_CURRENT ? ATD_TORCH_CURRENT : val;
			g_func_tbl.led_off(id);//prevent improper command seqence

			if(*index == 1)
				rc = g_func_tbl.led_low(id,val,0);
			else
				rc = g_func_tbl.led_low(id,0,val);
		}
		else if(on == 0)
		{
			rc = g_func_tbl.led_off(id);
		}
		else
		{
			pr_err("invalid command value %d, should be 0-OFF, 1-ON\n",on);
			rc = -1;
		}
	}
	else if(mode == 1)//flash
	{
		if(on == 1)
		{
			val = val==0 ? ATD_FLASH_CURRENT : val;
			val = val > ATD_FLASH_CURRENT ? ATD_FLASH_CURRENT : val;
			g_func_tbl.led_off(id);//prevent improper command seqence
			if(*index == 1)
				rc = g_func_tbl.led_high(id,val,0);
			else
				rc = g_func_tbl.led_high(id,0,val);
		}
		else if(on == 0)
		{
			rc = g_func_tbl.led_off(id);
		}
		else
		{
			pr_err("invalid command value %d, should be 0-OFF, 1-ON\n",on);
			rc = -1;
		}
	}
	else
	{
		pr_err("invalid command type %d, should be 0 - Torch, 1 - Flash\n",mode);
		rc = -1;
	}

	if(rc < 0)
	{
		g_func_tbl.led_off(id);
		g_func_tbl.led_release(id);
		g_ATD_status = 0;//failure
	}
	else
	{
		if(on == 0 )//torch off or flash off
		{
			g_func_tbl.led_release(id);
			g_ATD_status = 1;
		}
		else
		{
			usleep_range(5*1000,6*1000);
			g_ATD_status = (g_error_value == 0);
			if(g_ATD_status == 0)
			{
				pr_err("Fault detected! Turn off and Release\n");
				g_func_tbl.led_off(id);
				g_func_tbl.led_release(id);
			}
		}
	}
	pr_info("node %d, command %d %d, rc %d, ATD status %d\n",*index, mode, on, rc, g_ATD_status);

	return ret_len;
}

static ssize_t flash_leds_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	char kbuf[16];
	int  rc = 0;

	int  mode;
	int  mA[2] = {0,0};
	int  val[3];
	int  n;

	ssize_t ret_len = count;
	uint32_t id = 0;

	if(count > 16)
		count = 16;

	if(copy_from_user(kbuf, buf, count))
	{
		pr_err("%s(): copy_from_user fail !\n",__func__);
		return -EFAULT;
	}

	n = sscanf(kbuf, "%d %d %d", &val[0], &val[1], &val[2]);
	if(n < 1)
	{
		pr_err("need at least one argument!\n");
		return ret_len;
	}
	else if(n == 1)
	{
		mode = val[0];
		if(mode == 0)//torch
		{
			mA[0]=ATD_TORCH_CURRENT;//default current
		}
		else if(mode == 1)
		{
			mA[0]=ATD_FLASH_CURRENT;//default current
		}

		if(id == 0)
			mA[1] = mA[0];
	}
	else if(n == 2)
	{
		mode = val[0];
		mA[0] = val[1];
		mA[1] = val[1];
	}
	else
	{
		mode = val[0];
		mA[0] = val[1];
		mA[1] = val[2];
	}

	if(mode != 0 && mode != 1)
	{
		pr_err("mode %d not valid! should be 0-Torch, 1-Flash\n",mode);
		return ret_len;
	}

	if(mA[0] < 0)
		mA[0] = 0;
	if(mA[1] < 0)
		mA[1] = 0;


	pr_info("id %d, command mode %d, current[%d %d]\n",id, mode, mA[0], mA[1]);

	g_error_value = 0;
	g_ATD_status = 0;

	g_func_tbl.led_init(id);

	if(mA[0] == 0 && mA[1] == 0)
	{
		rc = g_func_tbl.led_off(id);
	}
	else
	{
		if(mode == 0)//torch
		{
			g_func_tbl.led_off(id);//turn off to show effect
			rc = g_func_tbl.led_low(id,mA[0],mA[1]);
		}
		else if(mode == 1)//flash
		{
			g_func_tbl.led_off(id);//turn off to show effect
			rc = g_func_tbl.led_high(id,mA[0],mA[1]);
		}
	}

	if(rc < 0)
	{
		g_func_tbl.led_off(id);
		g_func_tbl.led_release(id);
		g_ATD_status = 0;//failure
	}
	else
	{
		if((mA[0] == 0 && mA[1] == 0))
		{
			g_func_tbl.led_release(id);
			g_ATD_status = 1;
		}
		else
		{
			usleep_range(5*1000,6*1000);
			g_ATD_status = (g_error_value == 0);
			if(g_ATD_status == 0)
			{
				pr_err("Fault detected! Turn off and Release\n");
				g_func_tbl.led_off(id);
				g_func_tbl.led_release(id);
			}
		}
	}
	pr_info("id %d, command mode %d, current [%d %d], rc %d\n",id, mode, mA[0], mA[1], rc);

	return ret_len;
}

static const struct file_operations flash_leds_proc_fops = {
       .read = NULL,
       .write = flash_leds_store,
};

static const struct file_operations flash_led_proc_fops = {
       .read = flash_led_show,
       .write = flash_led_store,
};

static const struct file_operations status_proc_fops = {
       .read = status_show,
       .write = NULL,
};

static void create_proc_file(const char *PATH,const struct file_operations* f_ops, void *data)
{
	struct proc_dir_entry *pde;

	pde = proc_create_data(PATH, 0666, NULL, f_ops, data);
	if(pde)
	{
		pr_info("create(%s) done\n",PATH);
	}
	else
	{
		pr_err("create(%s) failed!\n",PATH);
	}
}

static void create_flash_proc_files(uint32_t id)
{
	if(id == 0)
	{
		create_proc_file(PROC_ATD_STATUS, &status_proc_fops, NULL);
		create_proc_file(PROC_ATD_FLASH1, &flash_led_proc_fops, &g_flash_index[0]);
		create_proc_file(PROC_ATD_FLASH2, &flash_led_proc_fops, &g_flash_index[1]);
		create_proc_file(PROC_CTRL1_LEDS, &flash_leds_proc_fops, NULL);
	}
	else
		pr_err("invalid flash id %d! not create any node\n",id);
}

void asus_flash_init(struct cam_flash_ctrl * fctrl)
{
	uint32_t id;

	if(fctrl)
	{
		id = fctrl->soc_info.index;
		if(id >= 1)
		{
			pr_err("invalid flash id %d\n",id);
			return;
		}

		if(g_fctrl[id] == NULL)
		{
			g_fctrl[id] = fctrl;

			g_func_tbl.led_init = asus_led_init;
			g_func_tbl.led_release = asus_led_release;
			g_func_tbl.led_off = asus_led_off;
			g_func_tbl.led_high = asus_led_high;
			g_func_tbl.led_low = asus_led_low;

			create_flash_proc_files(id);
		}
		else
		{
			pr_info("flash ctrl for id %d already inited!\n",id);
		}
	}
	else
	{
		pr_err("flash ctrl passed in is NULL!\n");
	}
}
