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

 /*****************************************/
/* Proximity Sensor Factory Module */
/****************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/input/ASH.h>

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_Factory"
#define SENSOR_TYPE_NAME		"proximity"

#undef dbg
#ifdef ASH_FACTORY_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

/***************************************/
/* Proximity read/write Calibration File*/
/**************************************/
int psensor_factory_read_high(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(PSENSOR_HI_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity read High Calibration open (%s) fail\n", PSENSOR_HI_CALIBRATION_FILE);
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
		err("Proximity read High Calibration strlen: f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);
	if(cal_val < 0) {
		err("Proximity read High Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Proximity read High Calibration : %d\n", cal_val);
	}
	
	return cal_val;
}
EXPORT_SYMBOL(psensor_factory_read_high);

bool psensor_factory_write_high(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(PSENSOR_HI_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity write High Calibration open (%s) fail\n", PSENSOR_HI_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Proximity Hi-Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Proximity write High Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(psensor_factory_write_high);

int psensor_factory_read_low(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(PSENSOR_LOW_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity read Low Calibration open (%s) fail\n", PSENSOR_LOW_CALIBRATION_FILE);
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
		err("Proximity read Low Calibration strlen f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		err("Proximity read Low Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Proximity read Low Calibration : %d\n", cal_val);
	}	
	
	return cal_val;
}
EXPORT_SYMBOL(psensor_factory_read_low);

bool psensor_factory_write_low(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(PSENSOR_LOW_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity write Low Calibration open (%s) fail\n", PSENSOR_LOW_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Proximity Lo-Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Proximity write Low Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(psensor_factory_write_low);


/********************************/
/* Proximity Inf calibration*/
/*******************************/
int psensor_factory_read_inf(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(PSENSOR_INF_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity read INF Calibration open (%s) fail\n", PSENSOR_INF_CALIBRATION_FILE);
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
		err("Proximity read INF Calibration strlen: f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);
	if(cal_val < 0) {
		err("Proximity read INF Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Proximity read INF Calibration : %d\n", cal_val);
	}
	
	return cal_val;
}
EXPORT_SYMBOL(psensor_factory_read_inf);

bool psensor_factory_write_inf(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(PSENSOR_INF_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity write INF Calibration open (%s) fail\n", PSENSOR_INF_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Proximity INF-Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Proximity write INF Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(psensor_factory_write_inf);

/*For transition period from 3/5 to 2/4*/
int psensor_factory_read_2cm(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(PSENSOR_2CM_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity read 2CM Calibration open (%s) fail\n", PSENSOR_2CM_CALIBRATION_FILE);
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
		err("Proximity read 2CM Calibration strlen: f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);
	if(cal_val < 0) {
		err("Proximity read 2CM Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Proximity read 2CM Calibration : %d\n", cal_val);
	}
	
	return cal_val;
}
EXPORT_SYMBOL(psensor_factory_read_2cm);

bool psensor_factory_write_2cm(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(PSENSOR_2CM_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity write 2CM Calibration open (%s) fail\n", PSENSOR_2CM_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Proximity 2CM Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Proximity write 2CM Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(psensor_factory_write_2cm);

int psensor_factory_read_4cm(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(PSENSOR_4CM_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity read 4CM Calibration open (%s) fail\n", PSENSOR_4CM_CALIBRATION_FILE);
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
		err("Proximity read 4CM Calibration strlen: f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);
	if(cal_val < 0) {
		err("Proximity read 4CM Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Proximity read 4CM Calibration : %d\n", cal_val);
	}
	
	return cal_val;
}
EXPORT_SYMBOL(psensor_factory_read_4cm);

bool psensor_factory_write_4cm(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(PSENSOR_4CM_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity write 4CM Calibration open (%s) fail\n", PSENSOR_4CM_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Proximity 4CM Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Proximity write 4CM Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(psensor_factory_write_4cm);

int psensor_factory_read_3cm(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(PSENSOR_3CM_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity read 3CM Calibration open (%s) fail\n", PSENSOR_3CM_CALIBRATION_FILE);
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
		err("Proximity read 3CM Calibration strlen: f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);
	if(cal_val < 0) {
		err("Proximity read 3CM Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Proximity read 3CM Calibration : %d\n", cal_val);
	}
	
	return cal_val;
}
EXPORT_SYMBOL(psensor_factory_read_3cm);

bool psensor_factory_write_3cm(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(PSENSOR_3CM_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity write 3CM Calibration open (%s) fail\n", PSENSOR_3CM_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Proximity 3CM Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Proximity write 3CM Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(psensor_factory_write_3cm);

int psensor_factory_read_5cm(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(PSENSOR_5CM_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity read 5CM Calibration open (%s) fail\n", PSENSOR_5CM_CALIBRATION_FILE);
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
		err("Proximity read 5CM Calibration strlen: f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);
	if(cal_val < 0) {
		err("Proximity read 5CM Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Proximity read 5CM Calibration : %d\n", cal_val);
	}
	
	return cal_val;
}
EXPORT_SYMBOL(psensor_factory_read_5cm);

bool psensor_factory_write_5cm(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(PSENSOR_5CM_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity write 5CM Calibration open (%s) fail\n", PSENSOR_5CM_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Proximity 5CM Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Proximity write 5CM Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(psensor_factory_write_5cm);

int psensor_factory_read_1cm(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(PSENSOR_1CM_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity read 1CM Calibration open (%s) fail\n", PSENSOR_1CM_CALIBRATION_FILE);
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
		err("Proximity read 1CM Calibration strlen: f_op=NULL or op->read=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);
	if(cal_val < 0) {
		err("Proximity read 1CM Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Proximity read 1CM Calibration : %d\n", cal_val);
	}
	
	return cal_val;
}
EXPORT_SYMBOL(psensor_factory_read_1cm);

bool psensor_factory_write_1cm(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(PSENSOR_1CM_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity write 1CM Calibration open (%s) fail\n", PSENSOR_1CM_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL) {
		pos_lsts = 0;
		vfs_write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Proximity 1CM Calibration strlen: f_op=NULL or op->write=NULL\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Proximity write 1CM Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(psensor_factory_write_1cm);

