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

 /************************************/
/* Light Sensor Factory Module */
/***********************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/input/ASH.h>

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_Factory"
#define SENSOR_TYPE_NAME		"light"

#undef dbg
#ifdef ASH_FACTORY_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

/*******************************************/
/* Light Sensor read/write Calibration*/
/******************************************/
int lsensor_factory_read_200lux(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[16];
	int cal_val = 0, readlen = 0;
	mm_segment_t old_fs;	

	fp = filp_open(LSENSOR_200LUX_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor read 200lux Calibration open (%s) fail\n", LSENSOR_200LUX_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		readlen = vfs_read(fp, buf, 16, &pos_lsts);
		buf[readlen] = '\0';
	} else {
		err("Light Sensor read 200lux Calibration f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		err("Light Sensor read 200lux Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Light Sensor read 200lux Calibration: Cal: %d\n", cal_val);
	}	
	
	return cal_val;
}
EXPORT_SYMBOL(lsensor_factory_read_200lux);

bool lsensor_factory_write_200lux(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(LSENSOR_200LUX_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor write 200lux Calibration open (%s) fail\n", LSENSOR_200LUX_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Light Sensor write 200lux Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Light Sensor write 200lux Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(lsensor_factory_write_200lux);

int lsensor_factory_read_1000lux(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[16];
	int cal_val = 0, readlen = 0;
	mm_segment_t old_fs;	

	fp = filp_open(LSENSOR_1000LUX_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor read 1000lux Calibration open (%s) fail\n", LSENSOR_1000LUX_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		readlen = vfs_read(fp, buf, 16, &pos_lsts);
		buf[readlen] = '\0';
	} else {
		err("Light Sensor read 1000lux Calibration f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		err("Light Sensor read 1000lux Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Light Sensor read 1000lux Calibration: Cal: %d\n", cal_val);
	}	
	
	return cal_val;
}
EXPORT_SYMBOL(lsensor_factory_read_1000lux);

bool lsensor_factory_write_1000lux(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(LSENSOR_1000LUX_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor write 1000lux Calibration open (%s) fail\n", LSENSOR_1000LUX_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Light Sensor write 1000lux Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Light Sensor write 1000lux Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(lsensor_factory_write_1000lux);

int lsensor_factory_read(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[16];
	int cal_val = 0, readlen = 0;
	mm_segment_t old_fs;	

	fp = filp_open(LSENSOR_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor read Calibration open (%s) fail\n", LSENSOR_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		readlen = vfs_read(fp, buf, 16, &pos_lsts);
		buf[readlen] = '\0';
	} else {
		err("Light Sensor read Calibration f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		err("Light Sensor read Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Light Sensor read Calibration: Cal: %d\n", cal_val);
	}	
	
	return cal_val;
}
EXPORT_SYMBOL(lsensor_factory_read);

bool lsensor_factory_write(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(LSENSOR_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor write Calibration open (%s) fail\n", LSENSOR_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Light Sensor write Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Light Sensor write Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(lsensor_factory_write);

/*For transition period from 100ms to 50ms +++*/
int lsensor_factory_read_50ms(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[16];
	int cal_val = 0, readlen = 0;
	mm_segment_t old_fs;	

	fp = filp_open(LSENSOR_50MS_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor read 50MS Calibration open (%s) fail\n", LSENSOR_50MS_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		readlen = vfs_read(fp, buf, 16, &pos_lsts);
		buf[readlen] = '\0';
	} else {
		err("Light Sensor read 50MS Calibration f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		err("Light Sensor read 50MS Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Light Sensor read 50MS Calibration: Cal: %d\n", cal_val);
	}	
	
	return cal_val;
}
EXPORT_SYMBOL(lsensor_factory_read_50ms);

bool lsensor_factory_write_50ms(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(LSENSOR_50MS_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor write 50MS Calibration open (%s) fail\n", LSENSOR_50MS_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Light Sensor write 50MS Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Light Sensor write 50MS Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(lsensor_factory_write_50ms);

int lsensor_factory_read_100ms(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[16];
	int cal_val = 0, readlen = 0;
	mm_segment_t old_fs;	

	fp = filp_open(LSENSOR_100MS_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor read 100MS Calibration open (%s) fail\n", LSENSOR_100MS_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		readlen = vfs_read(fp, buf, 16, &pos_lsts);
		buf[readlen] = '\0';
	} else {
		err("Light Sensor read 100MS Calibration f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		err("Light Sensor read 100MS Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Light Sensor read 100MS Calibration: Cal: %d\n", cal_val);
	}	
	
	return cal_val;
}
EXPORT_SYMBOL(lsensor_factory_read_100ms);

bool lsensor_factory_write_100ms(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(LSENSOR_100MS_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor write 100MS Calibration open (%s) fail\n", LSENSOR_100MS_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Light Sensor write 100MS Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Light Sensor write 100MS Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(lsensor_factory_write_100ms);

/*For transition period from 100ms to 50ms ---*/