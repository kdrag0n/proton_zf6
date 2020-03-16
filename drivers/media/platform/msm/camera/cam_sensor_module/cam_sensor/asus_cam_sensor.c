#include "asus_cam_sensor.h"
#include <linux/proc_fs.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include "cam_sensor_core.h"
#include "cam_eeprom_dev.h"//EEPROM
#include <linux/thermal.h>

#pragma GCC diagnostic ignored "-Wgcc-compat"
#undef	pr_fmt
#define pr_fmt(fmt) "SENSOR-ATD %s(): " fmt, __func__

#define MAX_CAMERA_ID CAMERA_3
#include "asus_cam_sensor_def.h"

static struct cam_sensor_ctrl_t *g_sensor_ctrl[MAX_CAMERA_ID+1]={0};
static uint16_t g_sensor_id[MAX_CAMERA_ID+1]={0};
static uint8_t	g_sensor_init_state[MAX_CAMERA_ID+1]={0};
static uint32_t g_module_changed[MAX_CAMERA_ID + 1];
static uint8_t module_change[MAX_CAMERA_ID + 1];

#include "asus_cam_sensor_utils.c"
#include "asus_cam_sensor_spec.c"

static int modules_proc_read(struct seq_file *buf, void *v)
{
	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl= (struct cam_sensor_ctrl_t *)buf->private;

	if(!s_ctrl)
	{
		seq_printf(buf, "s_ctrl is NULL\n");
		pr_err("s_ctrl is NULL!\n");
	}
	else
	{
		seq_printf(buf, "%s\n",get_sensor_name(s_ctrl->sensordata->slave_info.sensor_id));
	}
	return rc;
}

static int module_proc_open(struct inode *inode, struct	 file *file)
{
	return single_open(file, modules_proc_read, PDE_DATA(inode));
}
static struct file_operations camera_module_fops = {
	.owner = THIS_MODULE,
	.open = module_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static char *g_module_proc_file_name[MAX_CAMERA_ID + 1]=
{	PROC_REAR_MODULE_1,
	PROC_FRONT_MODULE_1,
	PROC_REAR_MODULE_2,
	PROC_FRONT_MODULE_2,
};

static void module_create(uint16_t camera_id)
{
	char* file_name = g_module_proc_file_name[camera_id];
	create_proc_file(file_name,&camera_module_fops,g_sensor_ctrl[camera_id]);
}
//camera_res +++

static sensor_define_t project_camera[MAX_CAMERA_ID+1] = {
	{SENSOR_ID_IMX586, DIRECTION_REAR},
	{SENSOR_ID_IMX586, DIRECTION_FRONT},
	{SENSOR_ID_OV13855, DIRECTION_REAR},
	{SENSOR_ID_OV13855, DIRECTION_FRONT},
};
static char g_sensors_res_string[64];

static void fill_sensors_res_string(void)
{
	//asume each side has MAX_CAMERA_ID+1 cameras
	uint8_t front_cam_res[MAX_CAMERA_ID+1];
	uint8_t rear_cam_res[MAX_CAMERA_ID+1];

	int i;
	int front_count=0;
	int rear_count=0;

	char *p = g_sensors_res_string;
	int offset = 0;

	memset(front_cam_res,0,sizeof(front_cam_res));
	memset(rear_cam_res,0,sizeof(rear_cam_res));

	for(i=0;i<MAX_CAMERA_ID+1;i++)
	{
		if(project_camera[i].direction == DIRECTION_FRONT)
		{
			front_cam_res[front_count++] = get_sensor_resolution(project_camera[i].sensor_id);
		}
		else if(project_camera[i].direction == DIRECTION_REAR)
		{
			rear_cam_res[rear_count++] = get_sensor_resolution(project_camera[i].sensor_id);
		}
	}

	sort_cam_res(front_cam_res,front_count);
	sort_cam_res(rear_cam_res,rear_count);

	//format: front0+front1+...+frontN-rear0+rear2+...+rearN
	for(i=0;i<front_count;i++)
	{
		if(i==0)
			offset+=sprintf(p+offset, "%dM",front_cam_res[i]);
		else
			offset+=sprintf(p+offset, "+%dM",front_cam_res[i]);
	}

	for(i=0;i<rear_count;i++)
	{
		if(i==0)
			offset+=sprintf(p+offset, "-%dM",rear_cam_res[i]);
		else
			offset+=sprintf(p+offset, "+%dM",rear_cam_res[i]);
	}

	offset+=sprintf(p+offset,"\n");
}

static int sensors_res_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s",g_sensors_res_string);
	return 0;
}

static int sensors_res_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, sensors_res_proc_read, NULL);
}

static struct file_operations sensors_res_fops = {
	.owner = THIS_MODULE,
	.open = sensors_res_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void sensors_res_create(void)
{
	fill_sensors_res_string();
	create_proc_file(PROC_SENSORS_RES,&sensors_res_fops,NULL);
}

//camera_res ---


static int temperature_proc_read(struct seq_file *buf, void *v)
{
	int32_t rc = 0;
	int32_t temperature = 0;
	struct cam_sensor_ctrl_t *s_ctrl= (struct cam_sensor_ctrl_t *)buf->private;
	if(!s_ctrl)
	{
		seq_printf(buf, "s_ctrl is NULL\n");
		pr_err("s_ctrl is NULL!\n");
	}
	else
	{
		rc = read_sensor_temperature(s_ctrl,&temperature);
		if(rc == 0)
			seq_printf(buf, "%hhd\n",temperature);
		else
			seq_printf(buf, "error: %hhd\n",rc);
	}
	return 0;
}

static int temperature_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, temperature_proc_read, PDE_DATA(inode));
}

static struct file_operations temperature_fops = {
	.owner = THIS_MODULE,
	.open = temperature_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static char *g_temperature_proc_file_name[MAX_CAMERA_ID + 1]=
{	PROC_THERMAL_REAR,
	PROC_THERMAL_FRONT,
	PROC_THERMAL_REAR_2,
	PROC_THERMAL_FRONT_2,
};

static void temperature_create(uint16_t camera_id)
{
	char* file_name = g_temperature_proc_file_name[camera_id];
	create_proc_file(file_name,&temperature_fops,g_sensor_ctrl[camera_id]);
}

//Thermal OPs +++

static char *g_thermal_type_name[MAX_CAMERA_ID + 1]=
{	THERMAL_TYPE_REAR,
	THERMAL_TYPE_FRONT,
	THERMAL_TYPE_REAR_2,
	THERMAL_TYPE_FRONT_2,
};
static struct thermal_zone_device * g_thermal_zone_devices[MAX_CAMERA_ID + 1];
static int g_thermal_zone_id_for_camera_sensor[MAX_CAMERA_ID + 1];
static uint8_t g_thermal_zone_init_done[MAX_CAMERA_ID + 1];

static int thermal_read_temperature(struct thermal_zone_device *tzd, int *temperature)
{
	int rc;
	struct cam_sensor_ctrl_t *s_ctrl;

	if(tzd == NULL)
	{
		pr_err("themal_zone_device is NULL!\n");
		return -EINVAL;
	}

	s_ctrl = (struct cam_sensor_ctrl_t *)tzd->devdata;
	if(s_ctrl == NULL)
	{
		pr_err("s_ctrl is NULL!\n");
		return -EINVAL;
	}
	if(g_thermal_zone_init_done[s_ctrl->id])
		rc = read_sensor_temperature(s_ctrl,temperature);
	else
	{
		pr_info("register on going, do dummy temperature read\n");
		rc = 0;
		*temperature = 0;
	}

	return rc;
}

static struct thermal_zone_device_ops tzd_ops ={
	.get_temp = thermal_read_temperature,
};

static void thermal_create(uint16_t camera_id)
{
	char* type_name = g_thermal_type_name[camera_id];
	struct thermal_zone_device *tzd;
	tzd = thermal_zone_device_register(type_name,0,0,g_sensor_ctrl[camera_id],&tzd_ops,NULL,0,0);
	if(IS_ERR(tzd))
	{
		pr_err("create thermal zone for camera %d failed!\n",camera_id);
	}
	else
	{
		pr_info("create(%s) type(%s) for camera %d done!\n",tzd->device.kobj.name,tzd->type,camera_id);
		g_thermal_zone_devices[camera_id] = tzd;
		g_thermal_zone_id_for_camera_sensor[camera_id] = tzd->id;
		g_thermal_zone_init_done[camera_id] = 1;
	}
}
//Thermal OPs ---


//OTP +++
typedef struct
{
	uint32_t num_map;
	uint32_t num_data;
	uint8_t *mapdata;
	uint8_t	 id;
}eeprom_info_t;

static eeprom_info_t g_eeprom_info[MAX_CAMERA_ID + 1];
static eeprom_info_t g_dit_eeprom_info[MAX_CAMERA_ID + 1];
static uint8_t g_otp_data_banks[MAX_CAMERA_ID + 1][OTP_DATA_LEN_BYTE*3];

static uint8_t g_otp_data_override_done[MAX_CAMERA_ID + 1] = {0};


static int otp_proc_read(struct seq_file *buf, void *v)
{
	int i;
	int bank;

	uint8_t* p_otp_data;
	uint32_t camera_id = 0;

	struct cam_sensor_ctrl_t *s_ctrl = (struct cam_sensor_ctrl_t *)buf->private;


	if(s_ctrl == NULL)
	{
		seq_printf(buf, "s_ctrl is NULL!\n");
		return 0;
	}

	camera_id = s_ctrl->id;

/*  debug: It will read OTP when cat otp every time
	pr_info("read_sensor_otp :E \n");
	read_sensor_otp(g_sensor_ctrl[camera_id],&g_otp_data_banks[camera_id][0]);
	pr_info("read_sensor_otp :X \n");
*/

	p_otp_data = &g_otp_data_banks[camera_id][0];

	//Camera 0 & 2 share EEPROM 0
	if((camera_id == 0 || camera_id == 2) && !g_otp_data_override_done[camera_id])
	{

		if(g_eeprom_info[0].mapdata == NULL || g_eeprom_info[0].num_data <= 6438 )
		{
			pr_err("eeprom_data of camera %d is NULL! not override otp\n", camera_id);
		}
		else if (g_eeprom_info[0].mapdata[6437] != 0x65)
		{
		    pr_err("camera %d : g_eeprom_info[0].mapdata[6437] = 0x%02X  is not 0x65 , not override otp\n",
		        camera_id, g_eeprom_info[0].mapdata[6437]);
		}
		else
		{
			pr_info("override_otp_from_eeprom  \n");

			override_otp_from_eeprom(p_otp_data,g_eeprom_info[0].mapdata,camera_id);
			g_otp_data_override_done[camera_id] = 1;
		}
	}


	for(bank=0;bank<3;bank++)
	{
		if(!g_otp_data_override_done[camera_id])
			p_otp_data = &g_otp_data_banks[camera_id][OTP_DATA_LEN_BYTE*bank];

		for( i = 0; i < OTP_DATA_LEN_WORD; i++)//show 32 bytes, although one bank have 64 bytes
		{
			seq_printf(buf,"0x%02X ",p_otp_data[i]);
			if( (i&7) == 7)
				seq_printf(buf,"\n");
		}
		if(bank<2)
			seq_printf(buf ,"\n");
	}

	return 0;
}

static int otp_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, otp_proc_read, PDE_DATA(inode));
}

static struct file_operations otp_fops = {
	.owner = THIS_MODULE,
	.open = otp_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static char *g_otp_proc_file_name[MAX_CAMERA_ID + 1]=
{	PROC_OTP_REAR,
	PROC_OTP_FRONT,
	PROC_OTP_REAR_AUX,
	PROC_OTP_FRONT_AUX,
};

static void otp_dump_create(uint16_t camera_id)
{
	char* file_name = g_otp_proc_file_name[camera_id];

	if(g_sensor_init_state[camera_id])
		{
		remove_proc_entry(file_name, NULL);
		pr_info("remove otp proc entry done  \n");
	}

	if(read_sensor_otp(g_sensor_ctrl[camera_id],&g_otp_data_banks[camera_id][0]) < 0)
		pr_err("read otp for camera sensor %d failed!\n",camera_id);

	create_proc_file(file_name,&otp_fops,g_sensor_ctrl[camera_id]);

}
//OTP ---


static struct kobject* kobj_sensor_test = NULL;
static ssize_t resolutions_read(struct device *dev, struct device_attribute *attr, char *buf);
static struct kobject* kobj_resolution = NULL;
static struct device_attribute dev_attr_camera_resolutions[MAX_CAMERA_ID + 1] ={
	__ATTR(SYSFS_ATTR_REAR_1, S_IRUGO | S_IWUSR | S_IWGRP, resolutions_read, NULL),
	__ATTR(SYSFS_ATTR_FRONT_1, S_IRUGO | S_IWUSR | S_IWGRP, resolutions_read, NULL),
	__ATTR(SYSFS_ATTR_REAR_2, S_IRUGO | S_IWUSR | S_IWGRP, resolutions_read, NULL),
	__ATTR(SYSFS_ATTR_FRONT_2, S_IRUGO | S_IWUSR | S_IWGRP, resolutions_read, NULL),
};

static ssize_t resolutions_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i = 0;

	if(!attr)
	{
		ret = sprintf(buf, "attr is NULL!\n");
		pr_err("attr is NULL!\n");
	}
	else
	{
		for(i=0;i<=MAX_CAMERA_ID;i++)
		{
			if(!strcmp(attr->attr.name,dev_attr_camera_resolutions[i].attr.name))
				break;
		}

		if(i > MAX_CAMERA_ID)
		{
			ret = sprintf(buf, "can't find resolution\n");
			pr_err("find resolution for %s failed\n",attr->attr.name);
		}
		else
		{
			ret = sprintf(buf, "%dM\n",get_sensor_resolution(g_sensor_id[i]));
		}
	}
	return ret;
}

static void resolution_create(uint16_t camera_id)
{
	if(kobj_sensor_test == NULL)
		create_sysfs_root_dir(SYSFS_ROOT_DIR, &kobj_sensor_test);
	if(kobj_resolution == NULL)
		create_sysfs_dir(SYSFS_RESOLUTION_DIR,&kobj_resolution,kobj_sensor_test);
	if(kobj_resolution)
		create_sysfs_file(kobj_resolution,&dev_attr_camera_resolutions[camera_id]);
}

static ssize_t status_read(struct device *dev, struct device_attribute *attr, char *buf);
static struct kobject* kobj_status = NULL;
static struct device_attribute dev_attr_camera_status[MAX_CAMERA_ID + 1] =
{
	__ATTR(SYSFS_ATTR_REAR_1, S_IRUGO | S_IWUSR | S_IWGRP, status_read, NULL),
	__ATTR(SYSFS_ATTR_FRONT_1, S_IRUGO | S_IWUSR | S_IWGRP, status_read, NULL),
	__ATTR(SYSFS_ATTR_REAR_2, S_IRUGO | S_IWUSR | S_IWGRP, status_read, NULL),
	__ATTR(SYSFS_ATTR_FRONT_2, S_IRUGO | S_IWUSR | S_IWGRP, status_read, NULL),
};

static ssize_t status_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i = 0;

	if(!attr)
	{
		ret = sprintf(buf, "attr is NULL!\n");
		pr_err("attr is NULL!\n");
	}
	else
	{
		for(i=0;i<=MAX_CAMERA_ID;i++)
		{
			if(!strcmp(attr->attr.name,dev_attr_camera_resolutions[i].attr.name))
				break;
		}

		if(i > MAX_CAMERA_ID)
		{
			ret = sprintf(buf, "can't find status\n");
			pr_err("find status for %s failed\n",attr->attr.name);
		}
		else
		{
			if(cam_sensor_test_i2c(g_sensor_ctrl[i]) < 0)
				return sprintf(buf, "%s\n%s\n", "ERROR: i2c r/w test fail","Driver version:");
			else
				return sprintf(buf, "%s\n%s\n%s 0x%x\n", "ACK:i2c r/w test ok","Driver version:","sensor_id:",g_sensor_id[i]);
		}
	}
	return ret;
}

static void status_create(uint16_t camera_id)
{
	if(kobj_sensor_test == NULL)
		create_sysfs_root_dir(SYSFS_ROOT_DIR, &kobj_sensor_test);
	if(kobj_status == NULL)
		create_sysfs_dir(SYSFS_STATUS_DIR,&kobj_status,kobj_sensor_test);
	if(kobj_status)
		create_sysfs_file(kobj_status,&dev_attr_camera_status[camera_id]);
}

static uint8_t g_camera_id;
static uint32_t g_camera_reg_addr;
static uint32_t g_camera_reg_val;
static enum camera_sensor_i2c_type g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
static uint8_t g_camera_sensor_operation = 0;
static int sensor_i2c_debug_read(struct seq_file *buf, void *v)
{
	uint32_t reg_val = 0xEEEE;
	int rc;

	if(g_sensor_ctrl[g_camera_id] == NULL)
	{
		pr_err("Sensor ID %d Not Exist!\n",g_camera_id);
		seq_printf(buf,"Camera ID %d Not Exist!\n",g_camera_id);
		return 0;
	}
	mutex_lock(&(g_sensor_ctrl[g_camera_id]->cam_sensor_mutex));
	seq_printf(buf,"=========================\n\
Camera %d, PowerState %d\n\
Camera %d, PowerState %d\n\
Camera %d, PowerState %d\n\
Camera %d, PowerState %d\n\
=========================\n",
	CAMERA_0,g_sensor_ctrl[CAMERA_0]?g_sensor_ctrl[CAMERA_0]->power_state:-1,
	CAMERA_1,g_sensor_ctrl[CAMERA_1]?g_sensor_ctrl[CAMERA_1]->power_state:-1,
	CAMERA_2,g_sensor_ctrl[CAMERA_2]?g_sensor_ctrl[CAMERA_2]->power_state:-1,
	CAMERA_3,g_sensor_ctrl[CAMERA_2]?g_sensor_ctrl[CAMERA_2]->power_state:-1
	);
	if(g_sensor_ctrl[g_camera_id]->power_state != 1)
	{
		pr_err("Please power up Camera Sensor %d first!\n",g_camera_id);
		seq_printf(buf,"Camera ID %d POWER DOWN!\n",g_camera_id);
	}
	else
	{
		if(g_camera_data_type == CAMERA_SENSOR_I2C_TYPE_BYTE)
			rc = cam_sensor_read_byte(g_sensor_ctrl[g_camera_id],g_camera_reg_addr,&reg_val);
		else
			rc = cam_sensor_read_word(g_sensor_ctrl[g_camera_id],g_camera_reg_addr,&reg_val);

		if(g_camera_sensor_operation == 1)//write
		{
			if(reg_val == g_camera_reg_val)
			{
				pr_info("read back the same value as write!\n");
			}
			else
			{
				pr_err("write value 0x%x and read back value 0x%x not same!\n",
						g_camera_reg_val,reg_val
				);
			}
		}
		seq_printf(buf,"Camera %d reg 0x%04x val 0x%x\n",g_camera_id,g_camera_reg_addr,reg_val);
	}
	mutex_unlock(&(g_sensor_ctrl[g_camera_id]->cam_sensor_mutex));
	return 0;
}

static int sensor_i2c_debug_open(struct inode *inode, struct  file *file)
{
	return single_open(file, sensor_i2c_debug_read, NULL);
}

static ssize_t sensor_i2c_debug_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	int n;
	char messages[32]="";
	uint32_t val[4];
	int rc;

	ret_len = len;
	if (len > 32) {
		len = 32;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	n = sscanf(messages,"%x %x %x %x",&val[0],&val[1],&val[2],&val[3]);

	if(n < 2 || n > 4 )
	{
		rc = -1;
		pr_err("Invalid Argument count %d!\n",n);
		goto RETURN;
	}
	else if( n == 2)
	{
		//camera id, reg addr
		g_camera_id = val[0];
		g_camera_reg_addr = val[1];

		g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		g_camera_sensor_operation = 0;
	}
	else if( n == 3)
	{
		//camera id, reg addr, data type
		g_camera_id = val[0];
		g_camera_reg_addr = val[1];
		if(val[2] == 1)
			g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		else if(val[2] == 2)
			g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		else
			g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		g_camera_sensor_operation = 0;
	}
	else if( n == 4)
	{
		//camera id, reg addr, data type, reg val
		g_camera_id = val[0];
		g_camera_reg_addr = val[1];
		if(val[2] == 1)
			g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		else if(val[2] == 2)
			g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		else
			g_camera_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		g_camera_reg_val = val[3];
		g_camera_sensor_operation = 1;
	}

	if(g_camera_id > MAX_CAMERA_ID)
	{
		pr_err("Invalid Sensor ID %d!\n",g_camera_id);
		g_camera_id = CAMERA_0;//reset to default ID
		goto RETURN;
	}

	if(g_sensor_ctrl[g_camera_id] == NULL)
	{
		pr_err("Sensor ID %d Not Exist!\n",g_camera_id);
		g_camera_id = CAMERA_0;//reset to default ID
		goto RETURN;
	}

	mutex_lock(&(g_sensor_ctrl[g_camera_id]->cam_sensor_mutex));
	if(g_sensor_ctrl[g_camera_id]->power_state != 1)
	{
		pr_err("Please power up Camera Sensor %d first!\n",g_camera_id);
		goto RETURN;
	}

	pr_info("gona %s Camera ID %d reg 0x%04x, data type %s\n",
			g_camera_sensor_operation ? "WRITE":"READ",
			g_camera_id,
			g_camera_reg_addr,
			g_camera_data_type == CAMERA_SENSOR_I2C_TYPE_BYTE?"BYTE":"WORD"
			);


	if(g_camera_sensor_operation == 1)
	{
		if(g_camera_data_type == CAMERA_SENSOR_I2C_TYPE_BYTE)
			rc = cam_sensor_write_byte(g_sensor_ctrl[g_camera_id],g_camera_reg_addr,g_camera_reg_val);
		else
			rc = cam_sensor_write_word(g_sensor_ctrl[g_camera_id],g_camera_reg_addr,g_camera_reg_val);

		if(rc < 0)
		{
			pr_err("write 0x%x to camera id %d addr 0x%04x FAIL\n",g_camera_reg_val,g_camera_id,g_camera_reg_addr);
		}
		else
		{
			pr_info("write 0x%x to camera id %d addr 0x%04x OK\n",g_camera_reg_val,g_camera_id,g_camera_reg_addr);
		}
	}
RETURN:
	mutex_unlock(&(g_sensor_ctrl[g_camera_id]->cam_sensor_mutex));
	return ret_len;
}
static const struct file_operations sensor_i2c_debug_fops = {
	.owner = THIS_MODULE,
	.open = sensor_i2c_debug_open,
	.read = seq_read,
	.write = sensor_i2c_debug_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void sensor_i2c_create(void)
{
	create_proc_file(PROC_SENSOR_I2C_RW,&sensor_i2c_debug_fops,NULL);
}

static uint32_t g_eeprom_reg_addr = 0x0000;
static uint32_t g_eeprom_id = 0;//camera id
static struct cam_eeprom_ctrl_t* g_eeprom_ctrl[MAX_CAMERA_ID+1] = {0};

static int eeprom_read(struct cam_eeprom_ctrl_t *e_ctrl, uint32_t reg_addr, uint32_t * reg_data)
{
	int rc;

	rc = camera_io_dev_read(&(e_ctrl->io_master_info),
								reg_addr,
								reg_data,
								CAMERA_SENSOR_I2C_TYPE_WORD,//addr_type
								CAMERA_SENSOR_I2C_TYPE_BYTE);//data_type
	if(rc < 0)
	{
		pr_err("EEPROM read reg 0x%x failed! rc = %d\n",reg_addr,rc);
	}
	else
	{
		pr_info("EEPROM read reg 0x%x get val 0x%x\n",reg_addr,*reg_data);
	}
	return rc;
}

static int eeprom_i2c_debug_read(struct seq_file *buf, void *v)
{
	uint32_t reg_val = 0xEEEE;
	int rc;

	struct cam_eeprom_ctrl_t * e_ctrl = g_eeprom_ctrl[g_eeprom_id];

	if(e_ctrl == NULL)
	{
		seq_printf(buf,"e_ctrl for eeprom %d is NULL!\n",g_eeprom_id);
		return 0;
	}
	mutex_lock(&(g_sensor_ctrl[g_eeprom_id]->cam_sensor_mutex));
	if(g_sensor_ctrl[g_eeprom_id]->power_state != 1)
	{
		pr_err("Please Power UP Camera Sensor %d for eeprom %d!\n",g_eeprom_id,g_eeprom_id);
		seq_printf(buf,"EEPROM %d POWER DOWN!\n",g_eeprom_id);
	}
	else
	{
		rc = eeprom_read(e_ctrl,g_eeprom_reg_addr,&reg_val);
		if(rc < 0)
		{
			seq_printf(buf,"read reg 0x%04x failed, rc = %d\n",g_eeprom_reg_addr,rc);
		}
		else
			seq_printf(buf,"0x%x\n",reg_val);
	}
	mutex_unlock(&(g_sensor_ctrl[g_eeprom_id]->cam_sensor_mutex));
	return 0;
}

static int eeprom_i2c_debug_open(struct inode *inode, struct  file *file)
{
	return single_open(file, eeprom_i2c_debug_read, NULL);
}

static ssize_t eeprom_i2c_debug_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";

	ret_len = len;

	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%x",&g_eeprom_reg_addr);

	pr_info("Gona read EEPROM reg addr 0x%04x\n",g_eeprom_reg_addr);

	return ret_len;
}

static const struct file_operations eeprom_i2c_debug_fops = {
	.owner = THIS_MODULE,
	.open = eeprom_i2c_debug_open,
	.read = seq_read,
	.write = eeprom_i2c_debug_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void eeprom_i2c_create(void)
{
	create_proc_file(PROC_EEPROM_I2C_R,&eeprom_i2c_debug_fops,NULL);
}

#define DUAL_CALI_START 0x821
#define DUAL_CALI_END	0x1020
#define DUAL_CALI_SIZE (DUAL_CALI_END-DUAL_CALI_START+1)
//ASUS_BSP +++ Combine dit cali data and EEPROM data into EEPROM_CTRL->mapdata
#define PDAF_CALI_START 0x21
#define PDAF_CALI_END 0x58A
#define PDAF_CALI_SIZE (PDAF_CALI_END-PDAF_CALI_START+1)

/*sm8150 only*/
#define REMOSAIC_CALI_START 0x1023
#define REMOSAIC_CALI_END 0x1924
#define REMOSAIC_CALI_SIZE (REMOSAIC_CALI_END-REMOSAIC_CALI_START+1)
//ASUS_BSP --- Combine dit cali data and EEPROM data into EEPROM_CTRL->mapdata


int32_t get_file_size(const char *filename, uint64_t* size)
{
	struct kstat stat;
	mm_segment_t fs;
	int rc = 0;

	stat.size = 0;

	fs = get_fs();
	set_fs(KERNEL_DS);

	rc = vfs_stat(filename,&stat);
	if(rc < 0)
	{
		pr_err("vfs_stat(%s) failed, rc = %d\n",filename,rc);
		rc = -1;
		goto END;
	}

	*size = stat.size;
END:
	set_fs(fs);
	return rc;
}

static int put_dual_cali_bin_buffer(struct seq_file *buf)
{
	int ret = 0;
	uint64_t i, size;
	if (get_file_size(DUAL_CALI_BIN, &size) == 0 && size == DUAL_CALI_SIZE) {
		uint8_t* pBuffer;
		pBuffer = kzalloc(sizeof(uint8_t) * DUAL_CALI_SIZE, GFP_KERNEL);
		if (pBuffer) {
			ret = read_file_into_buffer(DUAL_CALI_BIN, pBuffer, size);
			if (ret == DUAL_CALI_SIZE) {
				for (i = 0; i < size; i++)
				{
					seq_putc(buf, pBuffer[i]);
				}
			}
			kfree(pBuffer);
		}
	}
	return ret;
}
//ASUS_BSP Byron Add for load defult dual cali bin +++
static int put_default_dual_cali_bin_buffer(struct seq_file *buf)
{
	int ret = 0;
	uint64_t i, size;
	if (get_file_size("/vendor/bin/dualcam_default_cali.bin", &size) == 0 && size == DUAL_CALI_SIZE) {
		uint8_t* pBuffer;
		pBuffer = kzalloc(sizeof(uint8_t) * DUAL_CALI_SIZE, GFP_KERNEL);
		if (pBuffer) {
			ret = read_file_into_buffer("/vendor/bin/dualcam_default_cali.bin", pBuffer, size);
			pr_err("Byron check ret = %d\n",ret);
			if (ret == DUAL_CALI_SIZE) {
				for (i = 0; i < size; i++)
				{
					seq_putc(buf, pBuffer[i]);
				}
			}
			kfree(pBuffer);
		}
	}
	return ret;
}
//ASUS_BSP Byron Add for load defult dual cali bin ---

static int dual_cali_read(struct seq_file *buf, void *v)
{
	int i;
	int report_eeprom_dual=0; //ASUS_BSP Byron Add for load defult dual cali bin
	eeprom_info_t * eeprom_info = (eeprom_info_t *)buf->private;

	if (put_dual_cali_bin_buffer(buf) == DUAL_CALI_SIZE){
		pr_err("Use dual cali bin file Success !!\n");
		return 0;
	}

	if(eeprom_info == NULL || eeprom_info->mapdata == NULL)
	{
		//seq_printf(buf,"eeprom info invalid!\n");
		put_default_dual_cali_bin_buffer(buf);	  //ASUS_BSP Byron Add for load defult dual cali bin
		return 0;
	}

	if(eeprom_info->num_data < DUAL_CALI_END+1)
	{
		//seq_printf(buf,"eeprom data not cover all dual cali data!\n");
		put_default_dual_cali_bin_buffer(buf);	  //ASUS_BSP Byron Add for load defult dual cali bin
		return 0;
	}

	for(i=DUAL_CALI_START;i<=DUAL_CALI_END;i++)
	{
		//ASUS_BSP Byron Add for load defult dual cali bin +++
		if(eeprom_info->mapdata[i] == 0 || eeprom_info->mapdata[i] == 0xFF){
			report_eeprom_dual ++;
		}
		//ASUS_BSP Byron Add for load defult dual cali bin ---
		seq_putc(buf, eeprom_info->mapdata[i]);
	}
	//ASUS_BSP Byron Add for load defult dual cali bin +++
	if(report_eeprom_dual >= 2000){
		buf->count = 0;
		pr_err("Use default dual calib bin instead of eeprom dual data\n");
		put_default_dual_cali_bin_buffer(buf);
	}
	//ASUS_BSP Byron Add for load defult dual cali bin ---
	return 0;
}

static int dual_cali_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dual_cali_read, PDE_DATA(inode));
}

static const struct file_operations dual_cali_fops = {
	.owner = THIS_MODULE,
	.open = dual_cali_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void dual_cali_dump_create(void)
{
	if(g_eeprom_info[CAMERA_0].mapdata && g_eeprom_info[CAMERA_0].num_data > DUAL_CALI_END+1)
	{
		//create_proc_file(PROC_ARCSOFT_CALI,&dual_cali_fops,&g_eeprom_info[CAMERA_0]);
		struct proc_dir_entry *pde;
		pde = proc_create_data(PROC_ARCSOFT_CALI, 0444, NULL, &dual_cali_fops, &g_eeprom_info[CAMERA_0]);
		if(pde)
		{
			pr_info("create(%s) done\n", PROC_ARCSOFT_CALI);
		}
		else
		{
			pr_err("create(%s) failed!\n", PROC_ARCSOFT_CALI);
		}
	}
	else
		pr_err("eeprom data not correct! can not create dual cali data dump node\n");
}
//dual cali data ---

static char *dit_eeprom_file_name[MAX_CAMERA_ID + 1]=
{	FACTORYDIR DIT_DUT_REAR,
	FACTORYDIR DIT_DUT_FRONT,
	FACTORYDIR DIT_DUT_REAR2,
	FACTORYDIR DIT_DUT_FRONT2,
};

static char *dit_eeprom_golden_file_name[MAX_CAMERA_ID + 1]=
{	GOLDENDIR DIT_DUT_REAR,
	GOLDENDIR DIT_DUT_FRONT,
	GOLDENDIR DIT_DUT_REAR2,
	GOLDENDIR DIT_DUT_FRONT2,
};

#define AFC_SIZE 6
uint8_t AFC_golden_data[AFC_SIZE] =
{
	0x04,
	0x07,
	0x04,
	0x87,
	0x05,
	0xD4
};

uint8_t is_AFC_data_valid(void)
{
	uint8_t is_AFC_valid = 0;
	int i;
	if(g_eeprom_info[0].mapdata)
	{
		for(i=0;i<AFC_SIZE;i++)
		{
			if(g_eeprom_info[0].mapdata[i] != 0xFF)
			{
				is_AFC_valid = 1;
				break;
			}
		}
	}
	return is_AFC_valid;
}

static void copy_to_dit_eeprom(struct cam_eeprom_ctrl_t *e_ctrl,uint32_t camera_id)
{
	if(g_dit_eeprom_info[camera_id].mapdata == NULL)
	{
		g_dit_eeprom_info[camera_id].mapdata = kzalloc(e_ctrl->cal_data.num_data, GFP_KERNEL);
		if(!g_dit_eeprom_info[camera_id].mapdata)
		{
			pr_err("alloc memory for DIT eeprom mapdata failed!\n");
			return;
		}
	}
	memcpy(g_dit_eeprom_info[camera_id].mapdata, e_ctrl->cal_data.mapdata, e_ctrl->cal_data.num_data);
}

static void compareCameraSN()
{
	static bool DoOnce = false;
	uint8_t otp_data_id[MAX_CAMERA_ID + 1][OTP_ID_LEN];
	uint16_t cameraidbuffer[MAX_CAMERA_ID + 1][OTP_ID_LEN];
	char *filename_cali_id = "/vendor/factory/cali_id.txt";
	uint8_t buf_cali_id[128];
	uint16_t buf_cali_id0[OTP_ID_LEN];
	uint16_t buf_cali_id2[OTP_ID_LEN];
	struct file *fp;
	mm_segment_t fs;
	loff_t pos = 0;
	int rc, i;

	if (DoOnce)
		return;

	memset(otp_data_id, 0, (MAX_CAMERA_ID + 1)*OTP_ID_LEN);
	if (g_eeprom_info[0].mapdata == NULL) {
		pr_err("rear_otp did not exist (no eeprom)\n");
		DoOnce = true;
		return;
	}
	for (i = 0; i <= MAX_CAMERA_ID; i++) {
		if (i == CAMERA_0 || i == CAMERA_1) //IMX586
			memcpy(otp_data_id[i], g_eeprom_info[0].mapdata+10, OTP_ID_LEN);
		else if (i == CAMERA_2 || i == CAMERA_3) //OV13855
			memcpy(otp_data_id[i], g_eeprom_info[0].mapdata+6437+2, OTP_ID_LEN);
		pr_debug("otp_data_id[%d] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", i,
			otp_data_id[i][0], otp_data_id[i][1], otp_data_id[i][2], otp_data_id[i][3], otp_data_id[i][4], otp_data_id[i][5],
			otp_data_id[i][6], otp_data_id[i][7], otp_data_id[i][8], otp_data_id[i][9], otp_data_id[i][10], otp_data_id[i][11]);
		sprintf((char *)cameraidbuffer[i], "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
			otp_data_id[i][0], otp_data_id[i][1], otp_data_id[i][2], otp_data_id[i][3],
			otp_data_id[i][4], otp_data_id[i][5], otp_data_id[i][6], otp_data_id[i][7],
			otp_data_id[i][8], otp_data_id[i][9], otp_data_id[i][10], otp_data_id[i][11]);
		pr_info("cameraidbuffer[%d]=%s", i, cameraidbuffer[i]);
	}

	memset(buf_cali_id, 0, 128);
	fp = filp_open(filename_cali_id, O_RDONLY,S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR(fp)) {
		pr_err("open(%s) failed\n", filename_cali_id);
		DoOnce = true;
		return;
	}
	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	rc = vfs_read(fp, buf_cali_id, 128, &pos);
	set_fs(fs);
	filp_close(fp, NULL);
	pr_info("buf_cali_id %s, pos %d", buf_cali_id, (int)pos);

	memset(buf_cali_id0, 0, sizeof(buf_cali_id0));
	sscanf(buf_cali_id+1, "%24s", buf_cali_id0);
	pr_info("buf_cali_id0 %s", buf_cali_id0);
	for (i = 0; i < OTP_ID_LEN; i++) {
		if (buf_cali_id0[i] != cameraidbuffer[CAMERA_0][i])
			module_change[CAMERA_0] = 1;
		if (buf_cali_id0[i] != cameraidbuffer[CAMERA_1][i])
			module_change[CAMERA_1] = 1;
	}
	for (i = 0; i < OTP_ID_LEN; i++)
		pr_debug("buf_cali_id0[%d]=%x, cameraidbuffer[CAMERA_0][%d]=%x\n", i, buf_cali_id0[i], i, cameraidbuffer[CAMERA_0][i]);

	memset(buf_cali_id2, 0, sizeof(buf_cali_id2));
	sscanf(buf_cali_id+29, "%24s", buf_cali_id2);
	pr_info("buf_cali_id2 %s", buf_cali_id2);
	for (i = 0; i < OTP_ID_LEN; i++) {
		if (buf_cali_id2[i] != cameraidbuffer[CAMERA_2][i])
			module_change[CAMERA_2] = 1;
		if (buf_cali_id2[i] != cameraidbuffer[CAMERA_3][i])
			module_change[CAMERA_3] = 1;
	}
	for (i = 0; i < OTP_ID_LEN; i++)
		pr_debug("buf_cali_id2[%d]=%x, cameraidbuffer[CAMERA_2][%d]=%x\n", i, buf_cali_id2[i], i, cameraidbuffer[CAMERA_2][i]);

	pr_info("module_change %d, %d, %d, %d", module_change[0], module_change[1], module_change[2], module_change[3]);

	DoOnce = true;
}

static int cover_dit_cal_data(struct cam_eeprom_ctrl_t *e_ctrl,uint32_t camera_id)
{
	int num;
	char* load_file_name;
	int wrong_data = 0;

#ifdef DIT_CHKCALI
	if(g_module_changed[camera_id])
	{
		load_file_name = dit_eeprom_golden_file_name[camera_id];
	}
	else
	{
		load_file_name = dit_eeprom_file_name[camera_id];
	}
#else
	// compare if the two id are matched: vendor/factory/cali_id.txt and /proc/driver/rear_otp (rear2_otp)
	// if not, use golden calibration file
	// if any file not exit, use original factory calibration file
	compareCameraSN();
	if(module_change[camera_id])
	{
		load_file_name = dit_eeprom_golden_file_name[camera_id];
	}
	else
	{
		load_file_name = dit_eeprom_file_name[camera_id];
	}
#endif

	num = read_file_into_buffer(load_file_name,e_ctrl->cal_data.mapdata,8192);

	//check data: AWB 0x02~0x07 can't be Null or 0
	if (num >= 8) {
		for(int i = 2; i < 8; i=i+2)
		{
			int32_t check_data = 0;
			if (e_ctrl->cal_data.mapdata+i != NULL && e_ctrl->cal_data.mapdata+(i+1) != NULL)
			{
				check_data = (e_ctrl->cal_data.mapdata[i+1]<<8) + e_ctrl->cal_data.mapdata[i];
				pr_info("[DIT_EEPROM]  (0x%04X 0x%04X)  data: 0x%04X ", i+1, i, check_data);

				if (check_data == 0)
				{
					wrong_data = 1;
					pr_info("[DIT_EEPROM]  %s wrong_data: address:(0x%04X 0x%04X) data:0x%02X\n",load_file_name , i+1, i, check_data);
					break;
				}
			}
			else
			{
				wrong_data = 1;
				pr_info("[DIT_EEPROM] %s wrong_data:  address:0x%04X = NULL  OR  address:0x%04X = NULL \n", load_file_name, i+1, i);
				break;
			}
		}
	}
	else
	{
		wrong_data = 1;
	}

	pr_info("[DIT_EEPROM] module_change %d load_file_name %s", module_change[camera_id], load_file_name);

	if(wrong_data == 0)
	{
		pr_info("[DIT_EEPROM] read %d bytes from %s",num,load_file_name);
#ifdef DIT_CHKCALI
	}else if(!g_module_changed[camera_id] && (num = read_file_into_buffer(dit_eeprom_golden_file_name[camera_id],e_ctrl->cal_data.mapdata,8192)) >0)
#else
	}else if (!module_change[camera_id] && (num = read_file_into_buffer(dit_eeprom_golden_file_name[camera_id],e_ctrl->cal_data.mapdata,8192)) > 0)
#endif
	{
		pr_info("[DIT_EEPROM] read %d bytes from %s",num,dit_eeprom_golden_file_name[camera_id]);
	}else{
		pr_err("[DIT_EEPROM] read %s failed, rc %d",dit_eeprom_file_name[camera_id],num);
	}
	if(camera_id == CAMERA_0 || camera_id == CAMERA_1)
	{
		pr_info("[DIT_EEPROM] camera %d Start", camera_id);
		if(num<=0)
		{/*If there is no dit bin ,set user space eeprom data to 0*/
			memset(e_ctrl->cal_data.mapdata,0, e_ctrl->cal_data.num_data);
		}
		//dit_eeprom:	AF:0x1150~0x1153	PDAF:0x1500~18E8
		if(e_ctrl->cal_data.mapdata[0x1158]==0&&e_ctrl->cal_data.mapdata[0x1159]==0)
		{/*If there is no dit bin or af check data=0,copy af data from eeprom*/
			if(!is_AFC_data_valid())
			{
				#ifdef CAM_FACTORY_CONFIG
					pr_err("[DIT_EEPROM] AFC in EEPROM invalid! FACTORY build, not override golden data\n");
				#else
					pr_err("[DIT_EEPROM] AFC in EEPROM invalid! Override with golden data for userspace...\n");
					memcpy(e_ctrl->cal_data.mapdata+0x1150,AFC_golden_data+1,1);
					memcpy(e_ctrl->cal_data.mapdata+0x1151,AFC_golden_data,1);
					memcpy(e_ctrl->cal_data.mapdata+0x1152,AFC_golden_data+5,1);
					memcpy(e_ctrl->cal_data.mapdata+0x1153,AFC_golden_data+4,1);
				#endif
			}
			else
			{
				pr_info("[DIT_EEPROM] camera %d, copy AF data from eeprom",camera_id);
				memcpy(e_ctrl->cal_data.mapdata+0x1150,g_eeprom_info[0].mapdata+1,1);
				memcpy(e_ctrl->cal_data.mapdata+0x1151,g_eeprom_info[0].mapdata,1);
				memcpy(e_ctrl->cal_data.mapdata+0x1152,g_eeprom_info[0].mapdata+5,1);
				memcpy(e_ctrl->cal_data.mapdata+0x1153,g_eeprom_info[0].mapdata+4,1);
			}
		}
		//ASUS_BSP +++ Combine dit cali data and EEPROM data into EEPROM_CTRL->mapdata
		/*Copy pdaf cali data from eeprom*/
		if (g_eeprom_info[0].mapdata != NULL)
		{
			memcpy(e_ctrl->cal_data.mapdata + 0x116A, g_eeprom_info[0].mapdata + PDAF_CALI_START, PDAF_CALI_SIZE);
			/*Read Remosaic for sm8150*/
			{
				uint32_t cur_ptr = 0x116A + PDAF_CALI_SIZE;
				memcpy(e_ctrl->cal_data.mapdata + cur_ptr, g_eeprom_info[0].mapdata + REMOSAIC_CALI_START, REMOSAIC_CALI_SIZE);
			}
		}
		//ASUS_BSP --- Combine dit cali data and EEPROM data into EEPROM_CTRL->mapdata

	}else if((camera_id == CAMERA_2 || camera_id == CAMERA_3) && g_dit_eeprom_info[0].mapdata != NULL){
		memcpy(e_ctrl->cal_data.mapdata+0x115A,g_dit_eeprom_info[0].mapdata+0x115A,16); //ASUS_BSP Porting from Draco for DIT work around
	}
	copy_to_dit_eeprom(e_ctrl,camera_id);
	return 0;
}

static int save_actual_eeprom_info(struct cam_eeprom_ctrl_t * e_ctrl, uint32_t camera_id)
{
	if(camera_id == CAMERA_0)
	{
		g_eeprom_info[camera_id].id = camera_id;
		g_eeprom_info[camera_id].num_map = e_ctrl->cal_data.num_map;
		g_eeprom_info[camera_id].num_data = e_ctrl->cal_data.num_data;
		if(g_eeprom_info[camera_id].mapdata == NULL)
		{
			g_eeprom_info[camera_id].mapdata = kzalloc(e_ctrl->cal_data.num_data, GFP_KERNEL);
			if(!g_eeprom_info[camera_id].mapdata)
			{
				pr_err("alloc memory for eeprom mapdata failed!\n");
				return -1;
			}
		}
		memcpy(g_eeprom_info[camera_id].mapdata, e_ctrl->cal_data.mapdata, e_ctrl->cal_data.num_data);
	}
	return 0;
}

static int eeprom_proc_read(struct seq_file *buf, void *v)
{
	int i;
	uint8_t *index = (uint8_t *)buf->private;

	pr_info("eeprom info, id %d, num_map %d, num_data %d\n",
			g_eeprom_info[*index].id, g_eeprom_info[*index].num_map, g_eeprom_info[*index].num_data);
	if(!g_eeprom_info[*index].mapdata)
	{
		seq_printf(buf,"eeprom data is NULL\n");
	}
	else
	{
		for( i = 0; i < g_eeprom_info[*index].num_data; i++)
		{
			seq_printf(buf, "0x%04X 0x%02X\n",i,g_eeprom_info[*index].mapdata[i]);
		}
	}
	return 0;
}
static int eeprom_proc_open(struct inode *inode, struct	 file *file)
{
	return single_open(file, eeprom_proc_read, PDE_DATA(inode));
}

static struct file_operations eeprom_proc_fops = {
	.owner = THIS_MODULE,
	.open = eeprom_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int eeprom_to_otp_read(struct seq_file *buf, void *v)
{
	int i,j,num;
	uint8_t *index = (uint8_t *)buf->private;

	pr_info("eeprom info, id %d, num_map %d, num_data %d\n",
			g_eeprom_info[*index].id, g_eeprom_info[*index].num_map, g_eeprom_info[*index].num_data);
	if(!g_eeprom_info[*index].mapdata)
	{
		seq_printf(buf,"eeprom data is NULL\n");
	}
	else
	{
		num=0;
		for( i = 0; i < 4; i++)
		{
			for( j = 0; j < 8; j++)
			{
				seq_printf(buf, "0x%02X ",g_eeprom_info[*index].mapdata[num++]);
			}
			seq_printf(buf, "\n");
		}
	}

	return 0;
}
static int eeprom_to_otp_open(struct inode *inode, struct  file *file)
{
	return single_open(file, eeprom_to_otp_read, PDE_DATA(inode));
}


static struct file_operations eeprom_to_otp_fops = {
	.owner = THIS_MODULE,
	.open = eeprom_to_otp_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dit_eeprom_proc_read(struct seq_file *buf, void *v)
{
	int i;
	uint8_t *index = (uint8_t *)buf->private;

	pr_info("[DIT_EEPROM], id %d, num_map %d, num_data %d\n",
			g_dit_eeprom_info[*index].id, g_dit_eeprom_info[*index].num_map, g_dit_eeprom_info[*index].num_data);
	if(!g_dit_eeprom_info[*index].mapdata)
	{
		seq_printf(buf,"eeprom data is NULL\n");
	}
	else
	{
		for( i = 0; i < g_dit_eeprom_info[*index].num_data; i++)
		{
			seq_printf(buf, "0x%04X 0x%02X\n",i,g_dit_eeprom_info[*index].mapdata[i]);
		}
	}
	return 0;
}
static int dit_eeprom_proc_open(struct inode *inode, struct	 file *file)
{
	return single_open(file, dit_eeprom_proc_read, PDE_DATA(inode));
}

static struct file_operations dit_eeprom_proc_fops = {
	.owner = THIS_MODULE,
	.open = dit_eeprom_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static char *g_dit_eeprom_proc_file_name[MAX_CAMERA_ID + 1]=
{	PROC_DIT_EEPROM_REAR,
	PROC_DIT_EEPROM_FRONT,
	PROC_DIT_EEPROM_REAR2,
	PROC_DIT_EEPROM_FRONT2,
};

static char *g_eeprom_proc_file_name[MAX_CAMERA_ID + 1]=
{	PROC_EEPROM_REAR,
	PROC_EEPROM_FRONT,
	PROC_EEPROM_REAR2,
	PROC_EEPROM_FRONT2,
};


void eeprom_dump_create(struct cam_eeprom_ctrl_t * e_ctrl)
{
	char* file_name;
	char* dit_file_name;
	uint32_t camera_id;
	uint32_t module_sn;

	if(e_ctrl == NULL)
	{
		pr_err("e_ctrl is NULL!\n");
		return;
	}

	if(get_camera_id_for_submodule(SUB_MODULE_EEPROM,e_ctrl->soc_info.index,&camera_id) != 0)
	{
		pr_err("can not find related camera id for eeprom index %d, not create debug node",e_ctrl->soc_info.index);
		return;
	}

	if(g_eeprom_ctrl[camera_id] != NULL)//Free
	{
		remove_proc_entry(g_dit_eeprom_proc_file_name[camera_id], NULL);
		kfree(g_dit_eeprom_info[camera_id].mapdata);

		if(camera_id == CAMERA_0)
		{
			kfree(g_eeprom_info[camera_id].mapdata);
			remove_proc_entry(PROC_ARCSOFT_CALI, NULL);
			remove_proc_entry(g_eeprom_proc_file_name[camera_id], NULL);
			remove_proc_entry(PROC_OTP_REAR, NULL);
		}

		g_dit_eeprom_info[camera_id].num_map = 0;
		g_dit_eeprom_info[camera_id].num_data = 0;
		g_dit_eeprom_info[camera_id].mapdata = NULL;
		g_dit_eeprom_info[camera_id].id = 0;

		g_eeprom_info[camera_id].num_map = 0;
		g_eeprom_info[camera_id].num_data = 0;
		g_eeprom_info[camera_id].mapdata = NULL;
		g_eeprom_info[camera_id].id = 0;

		g_eeprom_ctrl[camera_id] = NULL;
	}

	if(g_eeprom_ctrl[camera_id] == NULL)//first time
	{
		save_actual_eeprom_info(e_ctrl,camera_id);
		g_eeprom_ctrl[camera_id] = e_ctrl;
		file_name = g_eeprom_proc_file_name[camera_id];
		dit_file_name=g_dit_eeprom_proc_file_name[camera_id];
		if(camera_id == CAMERA_0)
		{
			create_proc_file(file_name,&eeprom_proc_fops,&g_eeprom_info[camera_id].id);
			create_proc_file(PROC_OTP_REAR,&eeprom_to_otp_fops,&g_eeprom_info[camera_id].id); // eeprom to otp

			dual_cali_dump_create();
			if(g_eeprom_info[camera_id].num_data >= 18)
			{
				module_sn = (g_eeprom_info[camera_id].mapdata[14]<<24 | g_eeprom_info[camera_id].mapdata[15]<<16 |
							 g_eeprom_info[camera_id].mapdata[16]<<8  | g_eeprom_info[camera_id].mapdata[17] );
				pr_info("CAMERA %d: Module ID 0x%x, Vendor ID 0x%x\n, Module SN 0x%08x",
						camera_id, g_eeprom_info[camera_id].mapdata[8],g_eeprom_info[camera_id].mapdata[9],module_sn);
				/* Kirin no ois
				set_ois_module_vendor_from_eeprom(g_eeprom_info[camera_id].mapdata[9]);
				set_ois_module_sn_from_eeprom(module_sn);
				set_actual_afc_data();
				*/
			}
			else
			{
				pr_err("eeprom data size %d is less than 18!\n",g_eeprom_info[camera_id].num_data);
			}
		}
		create_proc_file(dit_file_name,&dit_eeprom_proc_fops,&g_dit_eeprom_info[camera_id].id);
		//Do not create procfs twice. Please think the logic again.
		//create_proc_file("driver/rear_eeprom_to_otp",&eeprom_to_otp_fops,&g_eeprom_info[camera_id].id);

		g_dit_eeprom_info[camera_id].id=camera_id;
		g_dit_eeprom_info[camera_id].num_map = e_ctrl->cal_data.num_map;
		g_dit_eeprom_info[camera_id].num_data = e_ctrl->cal_data.num_data;
	}
	else
	{
		pr_info("EEPROM debug file for camera %d already created\n",camera_id);
	}

	cover_dit_cal_data(e_ctrl,camera_id);
}

//Module Changed ++++

static int module_changed_read(struct seq_file *buf, void *v)
{
	uint32_t* module_changed = (uint32_t *)buf->private;
	seq_printf(buf, "%d\n",*module_changed);
	return 0;
}

static int module_changed_open(struct inode *inode, struct file *file)
{
	return single_open(file, module_changed_read, PDE_DATA(inode));
}

static ssize_t module_changed_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	char messages[8]="";

	uint32_t val;

	uint32_t *module_changed = PDE_DATA(file_inode(dev));

	ret_len = len;
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages, "%d", &val);

	if(val == 0)
		*module_changed = 0;
	else
		*module_changed = 1;

	return ret_len;
}

static const struct file_operations module_changed_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= module_changed_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= module_changed_write,
};

static char *g_module_changed_file_name[MAX_CAMERA_ID + 1]=
{	PROC_MODULE_CHANGE_REAR,
	PROC_MODULE_CHANGE_FRONT,
	PROC_MODULE_CHANGE_REAR2,
	PROC_MODULE_CHANGE_FRONT2
};

void module_changed_create(uint16_t camera_id)
{
	char* file_name = g_module_changed_file_name[camera_id];
	create_proc_file(file_name,&module_changed_proc_fops,&g_module_changed[camera_id]);
}

//Module Changed ----



void asus_cam_sensor_init(struct cam_sensor_ctrl_t *s_ctrl)
{

	pr_info("CAMERA ID %d   init :E", s_ctrl->id);
	if(!s_ctrl)
	{
		pr_err("s_ctrl is NULL\n");
		return;
	}

	if(s_ctrl->id > MAX_CAMERA_ID)
	{
		pr_err("s_ctrl id %d invalid!\n",s_ctrl->id);
		return;
	}

	if(!g_sensor_init_state[s_ctrl->id])
	{
		g_sensor_ctrl[s_ctrl->id] = s_ctrl;
		g_sensor_id[s_ctrl->id] = s_ctrl->sensordata->slave_info.sensor_id;
		pr_info("CAMERA ID %d, Sensor id 0x%X E\n",s_ctrl->id,g_sensor_id[s_ctrl->id]);
		module_create(s_ctrl->id);
		resolution_create(s_ctrl->id);
		status_create(s_ctrl->id);

		temperature_create(s_ctrl->id);
		thermal_create(s_ctrl->id);

		module_changed_create(s_ctrl->id);

		if(s_ctrl->id == CAMERA_0)
		{
			sensor_i2c_create();
			eeprom_i2c_create();//for debug
			sensors_res_create();//for shipping image
		}

		if(s_ctrl->id == CAMERA_2 || s_ctrl->id == CAMERA_3)
		{
			otp_dump_create(s_ctrl->id);
		}

		pr_info("CAMERA ID %d, Sensor id 0x%X X\n",s_ctrl->id,g_sensor_id[s_ctrl->id]);
		g_sensor_init_state[s_ctrl->id] = 1;
	}
	else
	{
		pr_err("camera id %d already inited!\n",s_ctrl->id);

		if(s_ctrl->id == CAMERA_2 || s_ctrl->id == CAMERA_3)
		{
			pr_info("otp_dump_create again");
			otp_dump_create(s_ctrl->id);
		}
	}


	pr_info("CAMERA ID %d   init :X", s_ctrl->id);
}
