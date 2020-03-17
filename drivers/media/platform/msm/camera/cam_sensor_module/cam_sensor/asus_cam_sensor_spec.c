
typedef enum{
	THERMAL_ERROR_NOT_SUPPORT = 1,
	THERMAL_ERROR_INVALID_PARAMTER,
	THERMAL_ERROR_INVALID_STATE,
	THERMAL_ERROR_READ_TEMPERATURE,
}sensor_read_temperature_err_t;


void delay_ms(uint32_t time)
{
	usleep_range(time*1000,time*1000+time*10);
}

static int32_t read_sensor_temperature_sony(struct cam_sensor_ctrl_t *s_ctrl, int32_t * temperature)
{
	int rc = 0;
	uint32_t reg_val;
	static int8_t return_temp = -1;

	struct timeval tv_now;
	static struct timeval tv_last;
	int64_t check_interval;

	sensor_read_temperature_err_t return_err;

	if(s_ctrl == NULL)
	{
		pr_err("s_ctrl is NULL!\n");
		return -THERMAL_ERROR_INVALID_PARAMTER;
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	if(s_ctrl->power_state != 1)
	{
		pr_err("camera id %d not power up!\n",s_ctrl->id);
		mutex_unlock(&(s_ctrl->cam_sensor_mutex));
		return -THERMAL_ERROR_INVALID_STATE;
	}

	do_gettimeofday(&tv_now);
	check_interval = diff_time_us(&tv_now,&tv_last);

	if(check_interval <= 50*1000)
	{
		pr_info("camera_thermal : cached (%hhd) interval = %lld us\n", return_temp, check_interval);
	}
	else
	{
		return_err = -THERMAL_ERROR_READ_TEMPERATURE;
		rc = cam_sensor_read_byte(s_ctrl,0x0138,&reg_val);
		if(rc < 0)
		{
			pr_err("camera_thermal: read control state failed!\n");
			goto END;
		}
		rc = cam_sensor_write_byte(s_ctrl,0x0138,(reg_val|0x01));
		if(rc < 0)
		{
			pr_err("camera_thermal: enable control failed!\n");
			goto END;
		}

		rc = cam_sensor_read_byte(s_ctrl,0x013a,&reg_val);
		if(rc < 0)
		{
			pr_err("camera_thermal: read temperature failed!\n");
			goto END;
		}

		return_temp = reg_val & 0xff;
		if(return_temp < -20)
			return_temp = -20;
		if(return_temp > 80)
			return_temp = 80;
		pr_info("camera_thermal: read %hhd, return %hhd\n",reg_val & 0xff,return_temp);

		tv_last = tv_now;
	}
	*temperature = return_temp;
	return_err = 0;
END:
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return return_err;
}

static int32_t read_sensor_temperature(struct cam_sensor_ctrl_t *s_ctrl, int32_t * temperature)
{
	int32_t ret = -THERMAL_ERROR_NOT_SUPPORT;
	switch(s_ctrl->sensordata->slave_info.sensor_id)
	{
		pr_err("CHHO sensor_id	%x \n", s_ctrl->sensordata->slave_info.sensor_id);
		case SENSOR_ID_IMX214:
		case SENSOR_ID_IMX298:
		case SENSOR_ID_IMX351:
		case SENSOR_ID_IMX362:
		case SENSOR_ID_IMX363:
		case SENSOR_ID_IMX563:
			ret = read_sensor_temperature_sony(s_ctrl,temperature);
			break;
	}
	return ret;
}


static int read_otp_ov13855(struct cam_sensor_ctrl_t *s_ctrl ,uint8_t *otp_data)
{
	int rc;
	int i;

	uint32_t start_addr_production;
	uint32_t start_addr_otp;
	uint32_t end_addr;

	uint8_t  internal_data[16];

	start_addr_production = 0x7000;
	start_addr_otp = 0x7220;
	end_addr = 0x727F;



	//reset sensor as default
	rc = cam_sensor_write_byte(s_ctrl,0x0103,0x01);
	if(rc < 0)
	{
		pr_err("Reset sensor as default failed!\n");
		goto END;
	}

	rc = cam_sensor_write_byte(s_ctrl,0x5000,0x00);
	if(rc < 0)
	{
		pr_err("Write 0x5000 >00 failed!\n");
		goto END;
	}

	//0x3D84[6] 1, Manual mode(partial)  0x3D84[7] 1, Program disable
	rc = cam_sensor_write_byte(s_ctrl,0x3D84,0xC0);
	if(rc < 0)
	{
		pr_err("Write 0x3D84 failed!\n");
		goto END;
	}

	//[3] 1, OTP power up load data enable;  [1] 1, OTP power up load setting enable
	rc = cam_sensor_write_byte(s_ctrl,0x3D85,0x0A);
	if(rc < 0)
	{
		pr_err("Write 0x3D85 failed!\n");
		goto END;
	}

	//[0x3D8C 0x3D8D] set 0x73C0
	rc = cam_sensor_write_byte(s_ctrl,0x3D8C,0x73);
	if(rc < 0)
	{
		pr_err("Write 0x3D8C failed!\n");
		goto END;
	}
	rc = cam_sensor_write_byte(s_ctrl,0x3D8D,0xC0);
	if(rc < 0)
	{
		pr_err("Write 0x3D8C failed!\n");
		goto END;
	}

	//set address range, MSB in lower address
	rc = cam_sensor_write_byte(s_ctrl,0x3D88,0x70);
	if(rc == 0)
		rc = cam_sensor_write_byte(s_ctrl,0x3D89,0x00);
	if(rc == 0)
		rc = cam_sensor_write_byte(s_ctrl,0x3D8A,0x72);
	if(rc == 0)
		rc = cam_sensor_write_byte(s_ctrl,0x3D8B,0x7F);
	if(rc < 0)
	{
		pr_err("Set address range failed!\n");
		goto END;
	}


	//trigger auto_load, stream mode enable; write 0x01 to 0x3D81, OTP_load_enable
	rc = cam_sensor_write_byte(s_ctrl, 0x0100, 0x01);
	if(rc == 0)
		rc = cam_sensor_write_byte(s_ctrl, 0x3D81, 0x01);
	if(rc < 0)
	{
		pr_err("Trigger load failed!\n");
		goto END;
	}

	delay_ms(20);


	rc = cam_sensor_read_seq_bytes(s_ctrl,start_addr_production,internal_data,16);
	if(rc < 0)
	{
		pr_err("Read internal 16 bytes data failed!\n");
		goto END;
	}
	else
	{
		pr_info("Production ID is [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n",
				internal_data[0],internal_data[1],internal_data[2],internal_data[3],internal_data[4],internal_data[5],
				internal_data[6],internal_data[7],internal_data[8],internal_data[9],internal_data[10],internal_data[11],
				internal_data[12],internal_data[13],internal_data[14],internal_data[15]
				);
	}


	for(i=0;i<3;i++)
	{

		rc = cam_sensor_read_seq_bytes(s_ctrl,
                                       start_addr_otp+(i*OTP_DATA_LEN_WORD),//read 32 bytes
                                       otp_data+(i*OTP_DATA_LEN_BYTE),//store in 64 bytes
                                       OTP_DATA_LEN_WORD);

		if(rc < 0)
		{
			pr_err("Read OTP data bank %d failed!\n",i);
			goto END;
		}
	}


	rc = cam_sensor_write_byte(s_ctrl,0x5000,0x10);
	if(rc < 0)
	{
		pr_err("Write 0x5000 > 10 failed!\n");
		goto END;
	}
END:
	return rc;
}


static int read_sensor_otp(struct cam_sensor_ctrl_t *s_ctrl ,uint8_t *otp_data)
{
	int rc;
	uint8_t auto_power;//whether need power up/down
	int read_result = -1;

	if(!s_ctrl)
	{
		pr_err("s_ctrl is NULL!\n");
		return -EINVAL;
	}

	//this is called from sensor probe cmd, no need use mutex
	if(s_ctrl->power_state == 1)
		auto_power = 0;
	else
		auto_power = 1;

	if(auto_power)
	{
		rc = cam_sensor_power_up(s_ctrl);
		if (rc < 0) {
			pr_err("power up failed");
			goto END;
		}
	}

	switch(s_ctrl->sensordata->slave_info.sensor_id)
	{
		case SENSOR_ID_OV13855:
			 read_result = read_otp_ov13855(s_ctrl,otp_data);
			 break;
		default:
			 pr_err("not support for sensor id 0x%x\n",s_ctrl->sensordata->slave_info.sensor_id);
			 read_result = -1;
			 break;
	}

	if(auto_power)
	{
		rc = cam_sensor_power_down(s_ctrl);
		if (rc < 0) {
			pr_err("power down failed");
		}
	}

END:
	return read_result;
}

static void override_otp_from_eeprom(uint8_t * otp_data, uint8_t * eeprom_data, uint32_t camera_id)
{
	if(camera_id == CAMERA_0)//IMX586
	{
		pr_info("override OTP %d bytes for IMX586\n",OTP_DATA_LEN_WORD);
		memcpy(otp_data,eeprom_data,OTP_DATA_LEN_WORD);
	}
	else if(camera_id == CAMERA_2)//OV13855
	{
		pr_info("override OTP %d bytes for OV13855\n",OTP_DATA_LEN_WORD-8);

		memcpy(otp_data+8,eeprom_data+6437,24);//OV13855 NO AF data (8 bytes)
	}
}
