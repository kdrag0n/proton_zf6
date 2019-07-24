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

 /******************************************/
/* Front RGB Sensor Factory Module */
/******************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/input/ASH.h>

/******************************/
/* Debug and Log System */
/*****************************/
#define MODULE_NAME			"ASH_Factory"
#define SENSOR_TYPE_NAME		"FRGB"

#undef dbg
#ifdef ASH_FACTORY_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

/*****************************************/
/* FRGB read/write Calibration File*/
/****************************************/
int FRGBsensor_factory_read_light1(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(FRGB_LIGHT1_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("FRGB read Light1 Calibration open (%s) fail\n", FRGB_LIGHT1_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		readlen = vfs_read(fp, buf, 6, &pos_lsts);
		buf[readlen] = '\0';		
	} else {
		err("FRGB read Light1 Calibration strlen: f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);
	if(cal_val < 0) {
		err("FRGB read Light1 Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("FRGB read Light1 Calibration : %d\n", cal_val);
	}
	
	return cal_val;
}
EXPORT_SYMBOL(FRGBsensor_factory_read_light1);

int FRGBsensor_factory_read_light2(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(FRGB_LIGHT2_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("FRGB read Light2 Calibration open (%s) fail\n", FRGB_LIGHT2_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		readlen = vfs_read(fp, buf, 6, &pos_lsts);
		buf[readlen] = '\0';		
	} else {
		err("FRGB read Light2 Calibration strlen f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		err("FRGB read Light2 Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("FRGB read Light2 Calibration : %d\n", cal_val);
	}	
	
	return cal_val;
}
EXPORT_SYMBOL(FRGBsensor_factory_read_light2);

int FRGBsensor_factory_read_light3(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(FRGB_LIGHT3_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("FRGB read Light3 Calibration open (%s) fail\n", FRGB_LIGHT3_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		readlen = vfs_read(fp, buf, 6, &pos_lsts);
		buf[readlen] = '\0';		
	} else {
		err("FRGB read Light3 Calibration strlen f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		err("FRGB read Light3 Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("FRGB read Light3 Calibration : %d\n", cal_val);
	}	
	
	return cal_val;
}
EXPORT_SYMBOL(FRGBsensor_factory_read_light3);

bool FRGBsensor_factory_write_light1(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(FRGB_LIGHT1_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("FRGB write Light1 Calibration open (%s) fail\n", FRGB_LIGHT1_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("FRGB Light1 Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("FRGB write Light1 Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(FRGBsensor_factory_write_light1);

bool FRGBsensor_factory_write_light2(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(FRGB_LIGHT2_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("FRGB write Light2 Calibration open (%s) fail\n", FRGB_LIGHT2_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("FRGB Light2 Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("FRGB write Light2 Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(FRGBsensor_factory_write_light2);

bool FRGBsensor_factory_write_light3(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(FRGB_LIGHT3_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("FRGB write Light3 Calibration open (%s) fail\n", FRGB_LIGHT3_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("FRGB Light3 Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("FRGB write Light3 Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(FRGBsensor_factory_write_light3);
