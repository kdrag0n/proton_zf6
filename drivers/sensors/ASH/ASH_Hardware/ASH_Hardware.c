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

 /***********************************/
/* Asus Sensor Hub Hardware */
/**********************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input/ASH.h>

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME	"ASH_i2c"

#undef dbg
#ifdef ASH_I2C_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s]"fmt,MODULE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s]"fmt,MODULE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s]"fmt,MODULE_NAME,##args)

/****************************/
/*i2c read/write function*/
/****************************/
uint8_t i2c_read_reg_u8(struct i2c_client* client, u8 reg)
{
	uint8_t data =0 ;
	
	if(client == NULL) {
		err("%s: i2c client is NULL.\n", __FUNCTION__);
		return -1;
	}
	
	data = i2c_smbus_read_byte_data(client, reg);
	if (data < 0) {
		err("%s: i2c_smbus_read_byte_data ERROR(0x%02X). \n", __FUNCTION__, reg);
	}
	
	return data;
}
EXPORT_SYMBOL(i2c_read_reg_u8);

int i2c_write_reg_u8(struct i2c_client* client, u8 reg, uint8_t data)
{
	int ret = 0;

	if(client == NULL) {
		err("%s: i2c client is NULL.\n", __FUNCTION__);
		return -1;
	}
	
	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0) {
		err("%s: i2c_smbus_write_byte_data ERROR(0x%02X). \n", __FUNCTION__, reg);		
		return ret;
	}
	
	return 0;
}
EXPORT_SYMBOL(i2c_write_reg_u8);

int i2c_read_reg_u16(struct i2c_client* client, u8 reg, uint8_t* data)
{
	int ret = 0;	
	struct i2c_msg msg[] = {
		{
		    .addr = client->addr,
		    .flags = 0, //write
		    .len = 1,
		    .buf = &reg,
		},
		{
		    .addr = client->addr,
		    .flags = I2C_M_RD, //read
		    .len = 2,
		    .buf = data,
		}
	};

	if(client == NULL) {
		err("%s: i2c client is NULL.\n", __FUNCTION__);
		return -1;
	}
	
	if (!client->adapter) {
	    return -ENODEV;
	}
	memset(&data, 0, sizeof(data));
	
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	
	/*return 2 is expected.*/
	if (ret != ARRAY_SIZE(msg)) {
		err("%s: i2c_transfer ERROR(0x%0X). \n", __FUNCTION__, reg);
		return -1;
	}

	return 0; 
}
EXPORT_SYMBOL(i2c_read_reg_u16);

int i2c_write_reg_u16(struct i2c_client* client, u8 reg, uint8_t* data)
{
	int ret = 0;	
	int len = 3;
	uint8_t buf[len];
	struct i2c_msg msg;

	if(client == NULL) {
		err("%s: i2c client is NULL.\n", __FUNCTION__);
		return -1;
	}

	if(data == NULL) {
		err("%s: data is NULL.\n", __FUNCTION__);
		return -1;
	}	
	
	msg.addr = client->addr;
	msg.flags = 0; /*write*/
	msg.len = len;
	msg.buf = buf;

	if (!client->adapter)
		return -ENODEV;

	buf[0] = reg;
	memcpy(buf + 1, &data[0], sizeof(data[0]));
	memcpy(buf + 2, &data[1], sizeof(data[1]));


	ret = i2c_transfer(client->adapter, &msg, 1);

	/*return postive is expected.*/
	if(ret < 0){
		err("%s: i2c_transfer ERROR. (reg=0x%x, data_l=%d, data_h=%d, err = 0x%x)\n", 
			__FUNCTION__, reg, data[0], data[1], ret);
		return ret;
	}

    return 0; 
}
EXPORT_SYMBOL(i2c_write_reg_u16);
