/* Copyright (C) 2018 Vishay MCU Microsystems Limited
 * Author: Randy Change <Randy_Change@asus.com>
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

#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
//#include <asm/mach-types.h>
#include <linux/regulator/consumer.h>
//#include <asm/setup.h>
#include <linux/jiffies.h>
#include <linux/extcon.h>
#include "MSP430FR2311.h"
#define MSP430FR2311_I2C_NAME "msp430fr2311"

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#define D(x...) do {} while (0)

#define I2C_RETRY_COUNT 3
#define UPDATE_FW_RETRY_COUNT 3

#define MCU_POLLING_DELAY 5000
#define MSP430_READY_I2C 0x35

struct MSP430FR2311_info *mcu_info;
static struct mutex MSP430FR2311_control_mutex;

enum eMCUState {
	MCU_TBD,
	MCU_EMPTY,
	MCU_PROGRAMMING,
	MCU_WAIT_POWER_READY,
	MCU_CHECKING_READY,
	MCU_READY,
	MCU_LOOP_TEST
} MCUState=MCU_TBD;

static signed char iCloseCounter=0;
static signed char iOpenCounter=0;
int (*fManualMode)(int , int , int );


static void mcu_do_work_later(struct work_struct *work);
static DECLARE_DELAYED_WORK(report_work, mcu_do_work_later);

//const int powerUpDuration=200;
#define DEFAULT_POWERDOWNDURATION 30000
#define MCU_5V_ALWAYS_ON
int powerDownDuration=DEFAULT_POWERDOWNDURATION;
bool bShowStopInfoOnce=1;


static void waitDelayAndShowText(char * s) {
	int i=10;
	D("Randy mcu (%s) command, wait for sec : %d...", s, i);
	return;
		for (i=10;i>0;i--) {
			pr_err(" %d...", i);
//			for (delayLoop=0;delayLoop<1000;delayLoop++)
//				udelay(500);
			msleep(500);
		}
		D(" issue\n");
}

int iProgrammingCounter=0;
int iProgrammingFail=0;
static inline void dumpI2CData(char *s, uint8_t slave_addr, uint8_t* writeBuffer,
    uint32_t numOfWriteBytes ) {
}




bool MSP430_I2CWriteReadA (uint8_t slave_addr, uint8_t* writeBuffer,  
    uint32_t numOfWriteBytes, uint8_t* readBuffer, uint32_t numOfReadBytes)
{
	uint8_t loop_i;

__u8* rxDMA;
		__u8* txDMA;

struct i2c_msg msgs[] = {
	{
	 .addr = slave_addr,
	 .flags = 0,
	 .len = numOfWriteBytes,
	 .buf = writeBuffer,
	},		 
	{
	 .addr = slave_addr,
	 .flags = I2C_M_RD,
	 .len = numOfReadBytes,
	 .buf = readBuffer,
	},		 
};

if (numOfWriteBytes>32||numOfReadBytes>=32) {
	rxDMA = kzalloc(sizeof(uint8_t)*numOfReadBytes, GFP_DMA | GFP_KERNEL);
	txDMA = kzalloc(sizeof(uint8_t)*numOfWriteBytes, GFP_DMA | GFP_KERNEL);
//	memcpy(rxDMA, readBuffer, numOfReadBytes);
	memcpy(txDMA, writeBuffer, numOfWriteBytes);
	msgs[0].buf=txDMA;
	msgs[1].buf=rxDMA;
}
	
	dumpI2CData("dump write in I2C_WriteRead +++",msgs[0].addr,  writeBuffer, numOfWriteBytes);


		for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		
			if (i2c_transfer(mcu_info->i2c_client->adapter, msgs, 2) > 0)
				break;
		
			/*check intr GPIO when i2c error*/
			if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
				pr_err("Randy [MCU][mcp error] %s, i2c err, slaveAddr 0x%x\n",
					__func__, slave_addr);
		msleep(10);
		}
		if (loop_i >= I2C_RETRY_COUNT) {
			pr_err(KERN_ERR "Randy [MCU][mcp error] %s retry over %d\n",
				__func__, I2C_RETRY_COUNT);
			
			if (numOfWriteBytes>32||numOfReadBytes>=32) {
				memcpy(readBuffer, rxDMA, numOfReadBytes);
				kfree(rxDMA);
				kfree(txDMA);
				
			}
			dumpI2CData("dump read result in I2C_WriteRead ---", msgs[1].addr, readBuffer, numOfReadBytes);
			return false;
		}
		if (numOfWriteBytes>32||numOfReadBytes>=32) {
			memcpy(readBuffer, rxDMA, numOfReadBytes);
			kfree(rxDMA);
			kfree(txDMA);
		}
		dumpI2CData("dump read result in I2C_WriteRead ---", msgs[1].addr, readBuffer, numOfReadBytes);

    return true;
}

bool MSP430_I2CWriteRead (uint8_t* writeBuffer,  
    uint32_t numOfWriteBytes, uint8_t* readBuffer, uint32_t numOfReadBytes)
{
  return MSP430_I2CWriteReadA(mcu_info->slave_addr, writeBuffer, numOfWriteBytes, readBuffer, numOfReadBytes);
}

bool MSP430_I2CAction(struct i2c_msg* msgs, uint8_t slave_addr, uint8_t* buffer, uint32_t numberOfBytes)
{
	uint8_t loop_i;
for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
	if (i2c_transfer(mcu_info->i2c_client->adapter, msgs, 1) > 0)
		break;

	/*check intr GPIO when i2c error*/
	if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
		pr_err("Randy [MCU][mcp error] %s, i2c err, slaveAddr 0x%x\n",
			__func__, slave_addr);

	msleep(10);
}
if (loop_i >= I2C_RETRY_COUNT) {
	pr_err(KERN_ERR "Randy [MCU][mcp error] %s retry over %d\n",
		__func__, I2C_RETRY_COUNT);
	return false;
}

return true;

}


bool MSP430_I2CRead(uint8_t slave_addr, uint8_t* buffer, uint32_t numberOfBytes)
{
struct i2c_msg msgs[] = {
	{
	 .addr = slave_addr,
	.flags = I2C_M_RD,
	 .len = numberOfBytes,
	 .buf = buffer,
	},		 
};

	if ( MSP430_I2CAction(msgs, slave_addr, buffer, numberOfBytes)) {
		dumpI2CData("dump read in I2C", slave_addr, buffer, numberOfBytes);
		return true;
	};
	return false;
}


bool MSP430_I2CWriteA(uint8_t slave_addr, uint8_t* buffer, uint32_t numberOfBytes)
{
struct i2c_msg msgs[] = {
	{
	 .addr = slave_addr,
	 .flags = 0,
	 .len = numberOfBytes,
	 .buf = buffer,
	},		 
};

dumpI2CData("dump write in I2C", slave_addr, buffer, numberOfBytes);
return MSP430_I2CAction(msgs, slave_addr, buffer, numberOfBytes);

}

bool MSP430_I2CWrite(uint8_t* buffer, uint32_t numberOfBytes)
{
	return MSP430_I2CWriteA(mcu_info->slave_addr, buffer, numberOfBytes);

}

char gFWVersion[4];
static uint16_t LEAD_DELTA = 57;
uint16_t ConstSpeedMode[]={0, 600, 600, 600, 600, 600, 600, 30, 60, 100, 280, 350, 420};
uint16_t TightenMode[]={1, 600, 600, 600, 600, 600, 600, 30, 60, 100, 280, 350, 420};
uint16_t ConvertFRQMode[][13]={
	{0, 600, 600, 600, 600, 600, 600, 30, 60, 100, 280, 350, 420},
	{1, 600, 600, 600, 600, 600, 600, 30, 60, 100, 280, 350, 420},
};
uint16_t ConvertFRQModeForSmallAngle[]={0, 25, 25, 25, 25, 25, 25, 50, 50, 50, 50, 50, 50};

//uint16_t ConvertFRQModeDown[]={0, 600, 600, 600, 600, 600, 600, 30, 60, 100, 280, 350, 420};
uint16_t AutoEmergencyMode[] = {0, 600, 600, 800, 800, 600, 600, 30, 60, 100, 280, 350, 420};
uint16_t AutoWarmUpMode[] = {0, 600, 600, 600, 600, 600, 600, 420, 420, 420, 420, 420, 420};
uint16_t AutoWarmUpMode2[] = {0, 600, 600, 600, 600, 600, 600, 420, 420, 420, 420, 420, 420};
uint16_t defaultManualSpeed[] = { 25, 25, 17, 7, 600, 800, 49, 39, 39, 39, 39 };
static uint16_t CONVERT_FRQ_SS_STEP[]={30, 40};
static uint16_t CONVERT_FRQ_FS_STEP[]={320, 310};

static int parse_param(char* buf, const char* title, uint16_t* target, int count) {
#define PARSE_DIG_PARAM_1 "%d"
  uint16_t cali_item[15];
	char parseLine[255];
	int i=0, matched_param=0;
	snprintf(parseLine, sizeof(parseLine), "%s=%s", title, PARSE_DIG_PARAM_1);
	  if (sscanf(buf, parseLine, &cali_item[i])>0) {
			target[i]=cali_item[i];
			matched_param++;
			snprintf(parseLine, sizeof(parseLine), "%s=" PARSE_DIG_PARAM_1, title, cali_item[i]);
			buf += strlen(parseLine);
//			pr_err("[MCU] detail %s", parseLine);
			for (i=1;i<count;i++) {
//				pr_err("[MCU] detail >>>%s", buf);
				if (sscanf(buf," %d", &cali_item[i])>0) {
					target[i]=cali_item[i];
					matched_param++;
					snprintf(parseLine, sizeof(parseLine),  " " PARSE_DIG_PARAM_1, cali_item[i]);
					buf += strlen(parseLine);
//					pr_err("[MCU] detail %s", parseLine);
				} else {
					break;
				}
			}


		{//debug
			char value[50];
			i=0;
			if (matched_param) {
				snprintf(parseLine, sizeof(parseLine), "%s(%d)=" PARSE_DIG_PARAM_1, title, matched_param, cali_item[i]);
				for (i=1;i<matched_param;i++) {
					snprintf(value, sizeof(value), " " PARSE_DIG_PARAM_1, cali_item[i]);
					strncat(parseLine, value, sizeof(parseLine)-1);
				}
			} 
		}

			
	}else {
//		pr_err("[MCU] No match %s", title);			
	}
	return matched_param;
	
}


static void process_cali_item(char* buf, unsigned int bufLen) {
/*
AUTO_DEFAULT        =100 100 100 100 100 100 0 0 0 4 4 6
AUTO_EMERGENCY=100 100 100 100 100 100 0 0 0 4 4 6
MANUAL_DEFAULT=100 100 100 100 100 100 0 0 0 4 4 6
MANUAL_LEAD_DELTA=53
MANUAL_GEAR_RATIO=166/100
*/
char* start_buf=buf;
do {
	while (buf[0]==0xd || buf[0]==0xa) buf++;

	parse_param(buf, "CONST_FRQ_SPEED", &ConstSpeedMode[1], 12);
	parse_param(buf, "CONVERT_FRQ_SMALL_ANGLE", &ConvertFRQModeForSmallAngle[1], 12);
	parse_param(buf, "CONVERT_FRQ_UP", &ConvertFRQMode[0][1], 12);
	parse_param(buf, "CONVERT_FRQ_DOWN", &ConvertFRQMode[1][1], 12);
	parse_param(buf, "AUTO_EMERGENCY", &AutoEmergencyMode[1], 12);
	parse_param(buf, "WARM_UP_STEP", &AutoWarmUpMode[1], 12);
	parse_param(buf, "WARM_UP_STEP_2", &AutoWarmUpMode2[1], 12);
	parse_param(buf, "MANUAL_LEAD_DELTA", &LEAD_DELTA, 1);
	parse_param(buf, "CONVERT_FRQ_UP_SS_STEP", &CONVERT_FRQ_SS_STEP[0], 1);
	parse_param(buf, "CONVERT_FRQ_DOWN_SS_STEP", &CONVERT_FRQ_SS_STEP[1], 1);
	parse_param(buf, "CONVERT_FRQ_UP_FS_STEP", &CONVERT_FRQ_FS_STEP[0], 1);
	parse_param(buf, "CONVERT_FRQ_DOWN_FS_STEP", &CONVERT_FRQ_FS_STEP[1], 1);
	parse_param(buf, "MANUAL_SPEED", &defaultManualSpeed[1], 10);
	parse_param(buf, "TIGHTEN_STEP", &TightenMode[1], 12);
	
	while (*buf != 0xa) {
		buf++;
		if (buf-start_buf>bufLen) {
			buf=NULL;
			break;
		}
	}
}while (buf !=NULL);


}

extern bool read_kernel_file(const char*, void (*)(char*, unsigned int) );

void read_cali_file() {	
	const char mcu_cali[]= {"/vendor/firmware/mcu_cali"};
	read_kernel_file(mcu_cali,  process_cali_item);
}


static uint8_t invokeString[8] = {0xCA, 0xFE, 0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xBE};
static uint8_t bslPassword[32];
static uint8_t resetVector[2];
static uint32_t resetVectorValue;

#include "i2cbsl.h"
#include "firmware_parser.h"



static int MSP43FR2311_Go_BSL_Mode(void) {
		waitDelayAndShowText("Init GPIO");
		
		
#define isAfterSR 1
		
//			gpio_set_value(mcu_info->mcu_test, 0^isAfterSR);
			gpio_set_value(mcu_info->mcu_reset, 0);
			udelay(5);
			gpio_set_value(mcu_info->mcu_test, 0^isAfterSR);
			udelay(5);
			gpio_set_value(mcu_info->mcu_test, 1^isAfterSR);
			udelay(5);
			gpio_set_value(mcu_info->mcu_test, 0^isAfterSR);
			udelay(5);
			gpio_set_value(mcu_info->mcu_test, 1^isAfterSR);
			udelay(5);
			gpio_set_value(mcu_info->mcu_reset, 1);
			udelay(5);
			gpio_set_value(mcu_info->mcu_test, 0^isAfterSR);
			udelay(10);
//			gpio_set_value(mcu_info->mcu_test, 0);
			
			D("[MCU] INFO: Invoking the BSL .\n");
			return MSP430BSL_invokeBSL(invokeString, 8);
			
}

static int MSP43FR2311_Update_Firmware_Load_File(bool bLoadFromFile) {
	int res=0, uu=0, ii=0;
	int delayLoop;
	tMSPMemorySegment* tTXTFile=NULL;
	MSP430FR2311_power_control(1);

	for (delayLoop=0;delayLoop<3;delayLoop++) {
		res=MSP43FR2311_Go_BSL_Mode();
		if( res== MSP430_STATUS_OPERATION_OK )
		{
			break;
		}
	}
	
	if( res!= MSP430_STATUS_OPERATION_OK )
	{
			pr_err("[MCU] ERROR: Could not invoke BSL!\n");
//					msleep(5000);
			goto BSLCleanUp;
	}

D("[MCU] INFO: Opening TI-TXT firmware file firmware.txt... ");
	
	MCUState=MCU_PROGRAMMING;
	iProgrammingCounter++;
	if (bLoadFromFile) {
		tTXTFile = read_firmware_file();
	} else {
		tTXTFile = MSP430BSL_parseTextFile();
	}
	 
	
			/* Sleeping after the invoke */
//			for (delayLoop=0;delayLoop<1000;delayLoop++)
//			udelay(2000);
	
#if 0 	
			/* Issuing a mass reset  */
			D("[MCU] INFO: Issuing a mass reset\n");
			if(MSP430BSL_massErase() != MSP430_STATUS_OPERATION_OK )
			{
					pr_err("[MCU] ERROR: Could not issue mass erase!\n");
					for (delayLoop=0;delayLoop<1000;delayLoop++)
					udelay(5000);
					goto BSLCleanUp;
			}
	
			/* Sleeping after the mass reset */
			for (delayLoop=0;delayLoop<1000;delayLoop++)
			udelay(2000);
	
			/* Unlocking the device */
#endif
#if 1	
	memset(bslPassword, 0xFF, 32);
	for (ii=0;ii<3;ii++)  {
			D("[MCU] INFO: Unlocking the device \n");
			waitDelayAndShowText("unlock device");
			if(MSP430BSL_unlockDevice(bslPassword) != MSP430_STATUS_OPERATION_OK )
			{
					pr_err("ERROR: Could not unlock device!\n");
//				goto BSLCleanUp;
			}
		}
#endif	
			waitDelayAndShowText("Programming all");
			/* Programming all memory segments */
			for(uu=0;uu<5;uu++)
			{
					D("[MCU] INFO: Programming attempt number %d\n", uu);
	
//					tTXTFile = firmwareImage;
					while(tTXTFile != NULL)
					{
						pr_err("MCU txtfile assign to %p", tTXTFile);
							D("[MCU] INFO: Programming @0x%x with %d bytes of data...0x%X, 0x%X, 0x%X ", 
									tTXTFile->ui32MemoryStartAddr, 
									tTXTFile->ui32MemoryLength,
									tTXTFile->ui8Buffer[0],
									tTXTFile->ui8Buffer[1],
									tTXTFile->ui8Buffer[2]
									);
	
							/* Programming the memory segment */
							ii = 0;
							while(ii < tTXTFile->ui32MemoryLength)
							{
									if((tTXTFile->ui32MemoryLength - ii) > 128)
									{
											res = MSP430BSL_sendData(tTXTFile->ui8Buffer + ii, 
																									 tTXTFile->ui32MemoryStartAddr + ii, 
																									 128);
											ii += 128;
									}
									else
									{
											res = MSP430BSL_sendData(tTXTFile->ui8Buffer + ii, 
																									 tTXTFile->ui32MemoryStartAddr + ii, 
																									 (tTXTFile->ui32MemoryLength - ii));
											ii = tTXTFile->ui32MemoryLength;
									}
	
									if(res != MSP430_STATUS_OPERATION_OK)
									{
											pr_err("[MCU] FAIL!ERROR: Programming address 0x%x (Code 0x%x).\n", 
																	tTXTFile->ui32MemoryStartAddr + ii, res);
											break;
									}
							}
	
							if(res != MSP430_STATUS_OPERATION_OK)
							{
									break;
							}
	
							D("[MCU] done!\n");
	
							tTXTFile = tTXTFile->pNextSegment;
					}
	
					if(res == MSP430_STATUS_OPERATION_OK)
					{
							D("[MCU] INFO: Programmed all memory locations successfully.\n");
							break;
					} else {
					}
			}


			if (uu>5) {
				goto BSLCleanUp;
			}
			/* Resetting the device */
			D("[MCU] INFO: Resetting the device.\n");
			res = MSP430BSL_readData(resetVector, MSP430_RESET_VECTOR_ADDR, 2);
	
			if(res != MSP430_STATUS_OPERATION_OK)
			{
					pr_err("[MCU] ERROR: Could not read reset vector address!\n");
					msleep(5000);
					goto BSLCleanUp;
			}
	
			resetVectorValue = (resetVector[1] << 8) | resetVector[0]; 
			D("[MCU] INFO: Reset vector read as 0x%x\n", resetVectorValue);
			res = MSP430BSL_setProgramCounter(resetVectorValue);
	
			if(res != MSP430_STATUS_OPERATION_OK)
			{
					pr_err("[MCU] ERROR: Could not set program counter!\n");
					msleep(5000);
					goto BSLCleanUp;
			}
	
	D("[MCU] INFO: Firmware updated without issue, %d/%d.\n", iProgrammingFail, iProgrammingCounter);
	
	D("[MCU] INFO: Power off the device.\n");
	MSP430FR2311_power_control(0);
	gpio_set_value(mcu_info->mcu_5v_boost_enable, 0);
	msleep(5);
	D("[MCU] INFO: Power on the device.\n");
	MSP430FR2311_power_control(1);
	gpio_set_value(mcu_info->mcu_5v_boost_enable, 1);
	msleep(200);
	
	return res;
	
BSLCleanUp:
//			MSP430BSL_cleanUpPointer(firmwareImage);
	iProgrammingFail++;
	D("[MCU] INFO: Programmed fail, %d/%d.\n", iProgrammingFail, iProgrammingCounter);
	return res;

}


static int MSP43FR2311_Update_Firmware(void) {
	int rc=0;

	rc=MSP43FR2311_Update_Firmware_Load_File(1);
	if (rc != MSP430_STATUS_INVOKE_FAIL) {
			MSP430BSL_cleanUpPointer();
	} 
	return rc;
}


int MSP430FR2311_Get_Steps() {
	char getsteps[] = { 0xAA, 0x55, 0x10};
	char steps[] = { 0, 0};
//	int i=0;
	
//	for(i=0;i<2;i++, msleep(10)) 
	if ( !MSP430_I2CWriteA(MSP430_READY_I2C, getsteps, sizeof(getsteps)) |!MSP430_I2CRead(MSP430_READY_I2C, steps, sizeof(steps)) ) {
		pr_err("[MCU] I2C error!");
		return -1;
	}

	return steps[1];
}


void MSP430FR2311_wakeup(uint8_t enable) {
	if (enable) {
		//power up
		iOpenCounter++;
		if (iOpenCounter==1) {
			gpio_set_value(mcu_info->mcu_wakeup, 0);
			msleep(1);
		} 
	}  else {
		iCloseCounter++;
		if (iCloseCounter==1) {
			queue_delayed_work(mcu_info->mcu_wq, &report_work, msecs_to_jiffies(powerDownDuration));
		} else  {
			mod_delayed_work(mcu_info->mcu_wq, &report_work, msecs_to_jiffies(powerDownDuration));
		}
	}
}

int MSP430FR2311_Get_Version(char * version) {
	char i2cfwversion[] = { 0xAA, 0x55, 0x0A};
	
	MSP430FR2311_wakeup(1);
	if (!MSP430_I2CWriteA(MSP430_READY_I2C, i2cfwversion, sizeof(i2cfwversion))  | !MSP430_I2CWriteReadA(MSP430_READY_I2C, i2cfwversion, sizeof(i2cfwversion), version, 4)) {
		pr_err("[MCU] I2C error!");
		MSP430FR2311_wakeup(0);
		return -1;
	}
	MSP430FR2311_wakeup(0);
	return 0;
}

#include "../power/supply/qcom/fg-core.h"
#include "../power/supply/qcom/fg-reg.h"
#include "../power/supply/qcom/fg-alg.h"
extern void asus_extcon_set_fnode_name(struct extcon_dev *edev, const char *fname);
extern void asus_extcon_set_name(struct extcon_dev *edev, const char *name);

struct extcon_dev *mcu_ver_extcon;
char mcuVersion[13];


int MSP430FR2311_Check_Version(void) {
	memset(gFWVersion, 0x0, sizeof(gFWVersion));
	if (MSP430FR2311_Get_Version(gFWVersion)==MSP430_STATUS_OPERATION_OK){
		tMSPMemorySegment* tTXTFile = NULL;
		tTXTFile = read_firmware_file();
		if (tTXTFile==NULL) {
			pr_err("[MCU] read firmware file error, can not to check firmware version");
			
//			pr_err("[MCU] seLinux security issue, workaround update firmware manually");
//			MCUState=MCU_READY;			
//			g_motor_status = 1; //probe success
			
			return MSP430_STATUS_TXTFILE_ERROR;
		}
		D("[MCU] Firmware version=%d%02d%02d%02X, new version=%d%02d%02d%02X\n", 
			gFWVersion[0], gFWVersion[1], gFWVersion[2], gFWVersion[3],
			tTXTFile->ui8Buffer[0],
			tTXTFile->ui8Buffer[2],
			tTXTFile->ui8Buffer[4],
			tTXTFile->ui8Buffer[6]	);

		if (tTXTFile->ui8Buffer[1]||
			tTXTFile->ui8Buffer[3]||
			tTXTFile->ui8Buffer[5]||
			tTXTFile->ui8Buffer[7]) {
			pr_err("[MCU] Firmware formate is incorrect\n"); 					
		}
		
		if (
			tTXTFile->ui8Buffer[0]*100000
			+tTXTFile->ui8Buffer[2]*1000
			+tTXTFile->ui8Buffer[4]*10
			+tTXTFile->ui8Buffer[6] ==
			gFWVersion[0]*100000
			+gFWVersion[1]*1000 
			+gFWVersion[2]*10 
			+gFWVersion[3]
			) {

			MCUState=MCU_READY;			
			g_motor_status = 1; //probe success

			read_cali_file();
			
		} else {
			pr_err("[MCU] Firmware need to be updated\n"); 
		}

		MSP430BSL_cleanUpPointer();

	} else {
		pr_err("[MCU] Firmware version get error\n");
		return MSP430_STATUS_I2C_NOT_FOUND;
	}
	return 0;
}


int loopCounter=0;
int totalLoopCounter=0;



void mcu_loop_test(void) {
//	pr_err("[MCU] do loop test, loop=%d, wake up=%d", loop_i, loop_i&1);
//	gpio_set_value(mcu_info->mcu_wakeup, loop_i&1);
	int delay=1500;
	pr_err("[MCU] do loop test, loop=%d/%d", loopCounter, totalLoopCounter);
	AutoEmergencyMode[0]=0;
	MSP430FR2311_Set_ParamMode(AutoEmergencyMode);
	if (AutoEmergencyMode[1]<600) {
		delay=delay*600/AutoEmergencyMode[1];
	}
	msleep(delay);
	AutoEmergencyMode[0]=1;
	MSP430FR2311_Set_ParamMode(AutoEmergencyMode);
	msleep(delay);
}

int MSP430FR2311_Pulldown_Drv_Power() {
	char MSP430PullDownDrvMode[]={0xAA, 0x55, 0x0E, 0x00};
	if (MCUState<MCU_READY) {
		pr_err("[MCU] Not ready!, state=%d", MCUState);
		return -MCUState;
	}
	if (!MSP430_I2CWriteA(MSP430_READY_I2C, MSP430PullDownDrvMode, sizeof(MSP430PullDownDrvMode))) {
		pr_err("[MCU] I2C error!");
		return -1;
	}
	return 0;
	
}


void mcu_do_later_power_down() {

	mutex_lock(&MSP430FR2311_control_mutex);
	if (iCloseCounter!=0) {
		MSP430FR2311_Pulldown_Drv_Power();
		gpio_set_value(mcu_info->mcu_wakeup, 1);
		iCloseCounter=0;
		iOpenCounter=0;
	}
	mutex_unlock(&MSP430FR2311_control_mutex);		

}


static void mcu_do_work_later(struct work_struct *work)
{
	loopCounter++;
	pr_err("Randy mcu, delay doing work, loopCounter=%d, state=%d", loopCounter, MCUState);	
	if (MCUState!=MCU_READY) {
		if (MCUState==MCU_EMPTY) { 
			if (MSP430FR2311_Check_Version()) {
				pr_err("[MCU][MSP430FR2311 ]%s: Fail\n", __func__); 	
			}
		}
		
		if (MCUState==MCU_CHECKING_READY) {
			if (MSP430FR2311_Check_Version()!=0 && loopCounter <UPDATE_FW_RETRY_COUNT) {
				MCUState=MCU_PROGRAMMING;
				queue_delayed_work(mcu_info->mcu_wq, &report_work, mcu_info->mcu_polling_delay);		 
			} else {
			}
			return;
		}

		if (MCUState==MCU_WAIT_POWER_READY) {
			D("[MCU] power ready");
			MCUState=MCU_READY;
			return;
		}

		if (MCUState==MCU_LOOP_TEST) {
			//loop i2c
			if (loopCounter < totalLoopCounter) {
				mcu_loop_test();
				queue_delayed_work(mcu_info->mcu_wq, &report_work, mcu_info->mcu_polling_delay/2); //60 secs 				
			} else {
				MCUState=MCU_READY;
			}
			return;
		}
		
		if (MCUState!=MCU_READY && MSP43FR2311_Update_Firmware()==0) {
			MCUState=MCU_CHECKING_READY;
		}

		//finally, we do not update firmware successfully, do this again
		if (MCUState<=MCU_CHECKING_READY  && loopCounter <UPDATE_FW_RETRY_COUNT) {
			queue_delayed_work(mcu_info->mcu_wq, &report_work, mcu_info->mcu_polling_delay);		 
		} else {
//			pr_err("[MCU] FATAL, mcu firmware update fail !!!! loopCounter=%d, state=%d", loopCounter, MCUState); 
		}
	}  else {	
		mcu_do_later_power_down();

	}
}


static int mcu_open(struct inode *inode, struct file *file)
{

	D("[MCU][MSP430FR2311] %s\n", __func__);


	return 0;
}

static int mcu_release(struct inode *inode, struct file *file)
{
	D("[MCU][MSP430FR2311] %s\n", __func__);

	return 0;
}

static long mcu_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int auto_mode = 0;
	int ret = 0;
	char nameMotor[ASUS_MOTOR_NAME_SIZE];
	motorDrvManualConfig_t data;

	D("[MCU][MSP430FR2311] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
		case ASUS_MOTOR_DRV_AUTO_MODE:
			ret = copy_from_user(&auto_mode, (int __user*)arg, sizeof(auto_mode));
			D("[MCU][MSP430FR2311] auto_mode %d\n", auto_mode);
			if(ret < 0 )
			{
				pr_err("%s: cmd = ASUS_MOTOR_DRV_AUTO_MODE, copy_from_user error(%d)\n", __func__, ret);
				goto end;
			}

			ret = MSP430FR2311_Set_AutoMode(auto_mode);
			if(ret < 0)
				pr_err("Set AutoMode failed\n");

			break;
		case ASUS_MOTOR_DRV_MANUAL_MODE:
			ret = copy_from_user(&data, (int __user*)arg, sizeof(data));
			if(ret < 0 )
			{
				pr_err("%s: cmd = ASUS_MOTOR_DRV_MANUAL_MODE, copy_from_user error(%d)\n", __func__, ret);
				goto end;
			}
			ret = MSP430FR2311_Set_ManualMode(data.dir, data.angle, data.speed);
			if(ret < 0)
				pr_err("Set ManualMode failed\n");

			break;
			case ASUS_MOTOR_DRV_AUTO_MODE_WITH_ANGLE:
				ret = copy_from_user(&data, (int __user*)arg, sizeof(data));
				if(ret < 0 )
				{
					pr_err("%s: cmd = ASUS_MOTOR_DRV_MANUAL_MODE, copy_from_user error(%d)\n", __func__, ret);
					goto end;
				}
				ret = MSP430FR2311_Set_AutoModeWithAngle(data.dir, data.angle);
				if(ret < 0)
					pr_err("Set AutoModeWithAngle failed\n");
			
				break;
		case ASUS_MOTOR_DRV_STOP:
			ret = MSP430FR2311_Stop();
			if(ret < 0)
				pr_err("Stop Motor failed\n");
			break;
			case ASUS_MOTOR_DRV_GET_STEPS:
				ret = MSP430FR2311_Get_Steps();
				if(ret < 0) {
					pr_err("Get Motor steps failed\n");	
					goto end;
				}
				ret = copy_to_user((int __user*)arg, &ret, sizeof(ret));
				break;
		case ASUS_MOTOR_DRV_GET_NAME:
		    snprintf(nameMotor, sizeof(nameMotor), "%s", ASUS_MOTOR_DRV_DEV_PATH);
			D("%s: cmd = MODULE_NAME, name = %s\n", __func__, nameMotor);
			ret = copy_to_user((int __user*)arg, &nameMotor, sizeof(nameMotor));
			break;
		case ASUS_MOTOR_DRV_CLOSE:
			D("[MCU]ASUS_MOTOR_DRV_CLOSE+++, ask mcu power down immediately\n");			
			flush_delayed_work(&report_work);
			D("[MCU]ASUS_MOTOR_DRV_CLOSE---, ask mcu power down immediately\n");			
			break;
		default:
			pr_err("[MCU][MSP430FR2311 error]%s: invalid cmd %d\n",
				__func__, _IOC_NR(cmd));
			return -EINVAL;
	}
end:
	return ret;
}

static const struct file_operations mcu_fops = {
	.owner = THIS_MODULE,
	.open = mcu_open,
	.release = mcu_release,
	.unlocked_ioctl = mcu_ioctl
};

struct miscdevice mcu_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "asusMotoDrv",
	.fops = &mcu_fops
};


static int mcu_setup(void)
{
	int ret;


	ret = misc_register(&mcu_misc);
	if (ret < 0) {
		pr_err(
			"[MCU][MSP430FR2311 error]%s: could not register mcu misc device\n",
			__func__);
	}

	return ret;

}


static int initial_MSP430FR2311_gpio(void)
{
	int ret;

		ret = gpio_request(mcu_info->mcu_reset, "gpio_msp430fr423111_reset");
		if (ret < 0) {
			pr_err("Randy [MCU][msp430fr23111 error]%s: gpio %d request failed (%d)\n",
				__func__, mcu_info->mcu_reset, ret);
			return ret;
		}
	
		ret = gpio_request(mcu_info->mcu_test, "gpio_msp430fr423111_test");
		if (ret < 0) {
			pr_err("Randy [MCU][msp430fr23111 error]%s: gpio %d request failed (%d)\n",
				__func__, mcu_info->mcu_test, ret);
			return ret;
		}
		
		ret = gpio_direction_output(mcu_info->mcu_reset, 1);
		if (ret < 0) {
			pr_err(
				"Randy [MCU][msp430fr3111 error]%s: fail to set gpio %d as input (%d)\n",
				__func__, mcu_info->mcu_reset, ret);
		gpio_free(mcu_info->mcu_reset);
		return ret;
		}
		
		ret = gpio_direction_output(mcu_info->mcu_test, 0);
		if (ret < 0) {
			pr_err(
				"Randy [MCU][msp430fr23111 error]%s: fail to set gpio %d as input (%d)\n",
				__func__, mcu_info->mcu_test, ret);
			 gpio_free(mcu_info->mcu_test);
			 return ret;
		}	


	ret = gpio_request(mcu_info->mcu_wakeup, "gpio_msp430fr423111_wakeup");
	if (ret < 0) {
		pr_err("Randy [MCU][msp430fr23111 error]%s: gpio %d request failed (%d)\n",
			__func__, mcu_info->mcu_wakeup, ret);
		return ret;
	}
	
	ret = gpio_direction_output(mcu_info->mcu_wakeup, 0);
	if (ret < 0) {
		pr_err(
			"Randy [MCU][msp430fr3111 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, mcu_info->mcu_wakeup, ret);
	gpio_free(mcu_info->mcu_wakeup);
	return ret;
	}

	return ret;
}
	
int MCU_I2C_power_control(bool enable)
{
        int ret = 0;
 //       pr_info("[MSP430FR2311] %s +\n", __func__);
				
        if (enable) {
	              ret = regulator_enable(mcu_info->vcc_l8c_1p8);
 	              if (ret) { 
					pr_err("[MCU] Regulator enable failed vcc_l8c_1p8 ret=%d\n", ret);
	              }
        } else {
				ret = regulator_disable(mcu_info->vcc_l8c_1p8);
				if (ret) pr_err("[MCU] Regulator disable failed vcc_l8c_1p8 ret=%d\n", ret);
        }

 //       pr_info("[MSP430FR2311] %s -\n", __func__);
        return ret;
}


int MSP430FR2311_power_control(uint8_t enable)
{
        int ret = 0;

//        pr_info("[MCU] %s, enable=%d +\n", __func__, enable);

		if (!enable) {
			//power down
//			gpio_set_value(mcu_info->mcu_wakeup, 1);
			gpio_set_value(mcu_info->mcu_reset, enable);
			gpio_set_value(mcu_info->mcu_test, 0);
//			msleep(powerDownDuration); //wait motor end
		}

		MCU_I2C_power_control(enable);

		g_motor_power_state = enable;
		if (enable) {
			//power up
//			for ( i=0;i<1000;i++)	udelay(powerUpDuration);
			gpio_set_value(mcu_info->mcu_reset, enable);
			gpio_set_value(mcu_info->mcu_test, 0);
//			msleep(powerUpDuration);
//			gpio_set_value(mcu_info->mcu_wakeup, 0);
//			for ( i=0;i<1000;i++)	udelay(1);
		}	

 //       pr_info("[MCU] %s -\n", __func__);
        return ret;
}

int FrqConvertMode(int dir, int angle, int speed) {
	uint16_t MotorDefault[]={0, 600, 600, 600, 600, 600, 600, 30, 60, 100, 280, 350, 420};
	memcpy(MotorDefault, ConvertFRQMode[dir], sizeof(MotorDefault));
	
	if (MCUState!=MCU_READY) {
		pr_err("[MCU] Not ready!, state=%d", MCUState);
		return -MCUState;
	}

	if (angle!=180) {
		int i=0;
		if (angle <=10) {
			memcpy(MotorDefault, ConvertFRQModeForSmallAngle, sizeof(MotorDefault));	
			MotorDefault[0]=dir;
		} else {
			int FS_steps=angle*CONVERT_FRQ_FS_STEP[dir]/180;
			for (i=7;i<12;i++) {
				MotorDefault[i]=FS_steps;
			}
			MotorDefault[i]=FS_steps+CONVERT_FRQ_SS_STEP[dir];
		}
	}  else {
	}
	
	pr_err("[MCU] PR2 auto control (D=%d, A=%d), param=%d %d %d %d %d %d %d %d %d %d %d %d %d", dir, angle,
		MotorDefault[0],
		MotorDefault[1],
		MotorDefault[2],
		MotorDefault[3],
		MotorDefault[4],
		MotorDefault[5],
		MotorDefault[6],
		MotorDefault[7],
		MotorDefault[8],
		MotorDefault[9],
		MotorDefault[10],
		MotorDefault[11],
		MotorDefault[12]
	);	
 	 
	return MSP430FR2311_Set_ParamMode(MotorDefault);
 }



int MSP430FR2311_Set_AutoModeWithAngle(int mode, int angle) {

	if (mode==0xbf) {
		pr_err("[MCU] Burn firmware");
		if (MSP43FR2311_Update_Firmware_Load_File(1)==MSP430_STATUS_OPERATION_OK){
			MCUState=MCU_READY;
			MSP430BSL_cleanUpPointer();
			}
		return 0;
	}

	if (mode==0xdd) {
		MSP430FR2311_Check_Version();		
		return 0;
	}

	if (mode==222) {			 
		read_cali_file();
		return 0;
	}


	 if (mode==242) { 
	 	int rc=0;
		 uint16_t MotorDefault[]={1, 600, 600, 600, 600, 600, 600, 30, 60, 100, 280, 350, 420};
		 memcpy(MotorDefault, TightenMode, sizeof(MotorDefault));
		 rc= MSP430FR2311_Set_ParamMode(MotorDefault);
		 msleep(5000);
		 return rc;
	 }

   if (mode==1 || mode==2)   {
		return FrqConvertMode(--mode, angle, 6);
   	}
	if (mode==3 || mode==4) { 
		AutoEmergencyMode[0]=mode-3;
		powerDownDuration=DEFAULT_POWERDOWNDURATION;
		return MSP430FR2311_Set_ParamMode(AutoEmergencyMode);
	}

	if (mode==5 || mode==6) { 
		AutoWarmUpMode[0]=mode-5;
		powerDownDuration=DEFAULT_POWERDOWNDURATION;
		return MSP430FR2311_Set_ParamMode(AutoWarmUpMode);
	}

	 if (mode==7 || mode==8) { 
		 AutoWarmUpMode2[0]=mode-7;
		 powerDownDuration=DEFAULT_POWERDOWNDURATION;
		 return MSP430FR2311_Set_ParamMode(AutoWarmUpMode2);
	 }

	 pr_err("[MCU] Not supported mode for %d", mode);

	 return -1;
#if 0	
	char MSP430AutoMode[]={0xaa, 0x55, 0x04, 0x01, 0, 0};
	pr_info("[MSP430FR2311] %s +\n", __func__);
	MSP430FR2311_wakeup(1);
	if (MCUState!=MCU_READY) {
		pr_err("[MCU] Not ready!, state=%d", MCUState);
		return -MCUState;
	}
	MSP430AutoMode[3]=mode;
	if (!MSP430_I2CWriteA(MSP430_READY_I2C, MSP430AutoMode, sizeof(MSP430AutoMode))) {
//		if (MCUState != MCU_LOOP_TEST)  MSP430FR2311_power_control(0);
		MSP430FR2311_wakeup(0);

		pr_err("[MCU] I2C error!");
		return -1;
	}
	
//	if (MCUState != MCU_LOOP_TEST) MSP430FR2311_power_control(0);
	MSP430FR2311_wakeup(0);

	pr_info("[MSP430FR2311] %s -\n", __func__);
	return 0;
	#endif
}

int MSP430FR2311_Set_AutoMode(int mode) {
	return MSP430FR2311_Set_AutoModeWithAngle(mode, 180);
}


int MSP430FR2311_Set_ParamMode(const uint16_t* vals) {
	char MSP430ParamMode[]={0xAA, 0x55, 0x0C, 0x00, 0x4B, 0x50, 0x57, 0x64, 0x57, 0x50, 0x0F, 0x1E, 0x32, 0x8C, 0xAF, 0xBE};

	int i=0;
	#ifdef ENABLE_LOOP_TEST
	if (vals[0]==-1 || vals[0]==65535) {
		memcpy(AutoEmergencyMode, vals, 13*sizeof (uint16_t));
		return 0;
	}

	if (vals[0]>=2 && vals[0]<=60000) {
		memcpy(AutoEmergencyMode, vals, 13*sizeof (uint16_t));
		MCUState=MCU_LOOP_TEST;		
		loopCounter=0;
		totalLoopCounter=vals[0]+1;
		queue_delayed_work(mcu_info->mcu_wq, &report_work, mcu_info->mcu_polling_delay);	
		return 0;
	}
	#endif
		mutex_lock(&MSP430FR2311_control_mutex);
		MSP430FR2311_wakeup(1);
		if (MCUState<MCU_READY) {
			pr_err("[MCU] Not ready!, state=%d", MCUState);
			return -MCUState;
		}

		powerDownDuration=0;
		for (i=1;i<7;i++) {
			uint16_t speed=vals[i]<<2;
//			const int defaultSpeedDuration=1200000;
			if (vals[i] < 50) {  //micro wave step
				switch (vals[i] ) {
					case 49:
						speed=88;
						break;
					case 39:
						speed=116;
						break;
					case 25:
						speed=165;
					break;
					case 20:
						speed=193;
					break;
					case 17:
						speed=232;
					break;
					case 8:
						speed=385;
					break;						
					case 7:
						speed=575;
					break;						
				}
				MSP430ParamMode[i+3]=vals[i]>>2;
			} else {
				MSP430ParamMode[i+3]=(vals[i]<<2)/50+50;			
			}
			powerDownDuration+=(vals[i+6]-((i==1)?0:vals[i+5]))*2400*500/speed/300;
//			pr_err("[MCU] Power duration += (%d-%d)*2400*500/%d/300 =   %d", vals[i+6], ((i==1)?0:vals[i+5]), speed, powerDownDuration);
		};
		powerDownDuration=DEFAULT_POWERDOWNDURATION;
		bShowStopInfoOnce=1;
		
		MSP430ParamMode[3]=vals[0];
		MSP430ParamMode[10]=vals[7]>>1;
		MSP430ParamMode[11]=vals[8]>>1;
		MSP430ParamMode[12]=vals[9]>>1;
		MSP430ParamMode[13]=vals[10]>>1;
		MSP430ParamMode[14]=vals[11]>>1;
		MSP430ParamMode[15]=vals[12]>>1;
		
//		pr_err("[MCU] dump param=%d %d %d %d %d %d %d %d %d %d %d %d %d 254", 
//		vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7], vals[8], vals[9], vals[10], vals[11], vals[12]);
		
		if (!MSP430_I2CWriteA(MSP430_READY_I2C, MSP430ParamMode, sizeof(MSP430ParamMode))) {
	//		if (MCUState != MCU_LOOP_TEST)	MSP430FR2311_power_control(0);
			MSP430FR2311_wakeup(0);
			mutex_unlock(&MSP430FR2311_control_mutex);
	
			pr_err("[MCU] I2C error!");
			return -1;
		}
		
	//	if (MCUState != MCU_LOOP_TEST) MSP430FR2311_power_control(0);
		MSP430FR2311_wakeup(0);
		mutex_unlock(&MSP430FR2311_control_mutex);

		return 0;
}

inline int MSP430FR2311_Set_ManualMode(int dir, int angle, int speed) {
	return (*fManualMode)(dir, angle, speed);
}

static char old_dir=-1;
static int BASE_ANGLE=120;
static int leadStep=-57; 

int ManualMode_BeforePR(int dir, int angle, int speed) {
	uint16_t MotorDefault[]={0, 600, 600, 600, 600, 600, 600, 30, 60, 100, 280, 350, 420};
	const int MAX_RANGE_STEP = MotorDefault[12]-LEAD_DELTA;
	const int MAX_RANGE_ACC_STEP = MotorDefault[12]-(LEAD_DELTA<<1);
	if (MCUState!=MCU_READY) {
		pr_err("[MCU] Not ready!, state=%d", MCUState);
		return -MCUState;
	}
	MotorDefault[0]=dir;

	if (angle!=180) {
		int accStep,gearStep;
		int leadDelta=LEAD_DELTA*(BASE_ANGLE-angle)/BASE_ANGLE;
		accStep=gearStep=angle*MotorDefault[12]/180;
		if (leadStep>MAX_RANGE_ACC_STEP || leadStep <0||old_dir!=dir) {
		} else {
			leadDelta=0;
		}

		if (old_dir!=dir) {
			gearStep+=leadDelta;
			old_dir=dir;
		} else {
			gearStep-=accStep/4;
		}
		
		if (dir==0)  {
			leadStep+=accStep;
			if (leadStep>MAX_RANGE_STEP) leadStep=MAX_RANGE_STEP;
		} else {
			leadStep-=accStep;
			if (leadStep<-LEAD_DELTA) leadStep=-LEAD_DELTA;
		}


		pr_info("[MCU] manual control gear step=%d, leadStep=%d, leadDelta=%d", gearStep, leadStep, leadDelta);
	
		MotorDefault[7]=MotorDefault[7]*gearStep/MotorDefault[12];
		MotorDefault[8]=MotorDefault[8]*gearStep/MotorDefault[12];
		MotorDefault[9]=MotorDefault[9]*gearStep/MotorDefault[12];
		MotorDefault[10]=MotorDefault[10]*gearStep/MotorDefault[12];
		MotorDefault[11]=MotorDefault[11]*gearStep/MotorDefault[12];
		MotorDefault[12]=MotorDefault[12]*gearStep/MotorDefault[12];
	}  else {
	   if (dir==0) {
			 leadStep=MAX_RANGE_STEP;
	   	}else {
			 leadStep=-LEAD_DELTA;
	   	}
		old_dir=-1;
	}


    if (speed !=6) {
	     int i=0;
		 for (i=0;i<6;i++) {
		 	const int defaultSpeed[] = { 0, 25,100,200,300,375 };// first 0 is not used
		 	MotorDefault[i+1]=defaultSpeed[speed];
			if (speed !=0) {
				powerDownDuration=DEFAULT_POWERDOWNDURATION+DEFAULT_POWERDOWNDURATION*600*angle/defaultSpeed[speed]/180;
			}
	 	}			
    	} else {
	    	powerDownDuration=DEFAULT_POWERDOWNDURATION;
    	}
		
	return MSP430FR2311_Set_ParamMode(MotorDefault);
}


int ManualMode_AfterAndPR2(int dir, int angle, int speed) {
	uint16_t MotorDefault[]={0, 600, 600, 600, 600, 600, 600, 30, 60, 100, 280, 350, 420};
	memcpy(MotorDefault, ConstSpeedMode, sizeof(MotorDefault));
	if (MCUState!=MCU_READY) {
		pr_err("[MCU] Not ready!, state=%d", MCUState);
		return -MCUState;
	}
	MotorDefault[0]=dir;

	if (angle!=180) {
		int gearStep;
		gearStep=0;

		if (old_dir!=dir) {
			gearStep+=LEAD_DELTA;
//			old_dir=dir;
		}
		MotorDefault[7]=MotorDefault[7]*angle/180+gearStep;
		MotorDefault[8]=MotorDefault[8]*angle/180+gearStep;
		MotorDefault[9]=MotorDefault[9]*angle/180+gearStep;
		MotorDefault[10]=MotorDefault[10]*angle/180+gearStep;
		MotorDefault[11]=MotorDefault[11]*angle/180+gearStep;
		MotorDefault[12]=MotorDefault[12]*angle/180+gearStep;
	}
	old_dir=dir;	

 
	if (speed <=10) {//micro-step process
	 	 int i=0;
//	 	 const uint16_t internalspeed[]={112, 112, 190, 472};
	 	 for (i=0;i<6;i++) {
	 		 MotorDefault[i+1]=defaultManualSpeed[speed];
			 MotorDefault[i+7]=MotorDefault[12];
	 	 }		 
//	 	 powerDownDuration=DEFAULT_POWERDOWNDURATION+DEFAULT_POWERDOWNDURATION*2400*angle/internalspeed[speed]/180;
	} else {
//		 powerDownDuration=DEFAULT_POWERDOWNDURATION;
	}
 	 
	return MSP430FR2311_Set_ParamMode(MotorDefault);
 }


int MSP430FR2311_Stop(void) {

	char MSP430Stop[]={0xAA, 0x55, 0x08, 00, 00};
	if (MCUState<MCU_CHECKING_READY) {
		pr_err("[MCU] Not ready!");
		return -MCUState;
	}
	mutex_lock(&MSP430FR2311_control_mutex);
	MSP430FR2311_wakeup(1);
	powerDownDuration=DEFAULT_POWERDOWNDURATION;
	if (!MSP430_I2CWriteA(MSP430_READY_I2C, MSP430Stop, sizeof(MSP430Stop))) {
		MSP430FR2311_wakeup(0);
		mutex_unlock(&MSP430FR2311_control_mutex);
		pr_err("[MCU] I2C error!");
		return -1;
	}
	if (bShowStopInfoOnce) {
		bShowStopInfoOnce=0;
		MSP430FR2311_Get_Steps();
	}
	MSP430FR2311_wakeup(0);
	mutex_unlock(&MSP430FR2311_control_mutex);
	return 0;
	
}

static int MSP430FR2311_power_init(void)
{
        int ret = 0;
pr_err("randy mcu power init, set requlator");
        mcu_info->vcc_l8c_1p8 = regulator_get(&mcu_info->i2c_client->dev, "vcc_l8c_1p8");
        if (IS_ERR(mcu_info->vcc_l8c_1p8))
        {
                ret = PTR_ERR(mcu_info->vcc_l8c_1p8);
                pr_err("[MCU] Regulator get failed vdd ret=%d", ret);
                return ret;
        }

        if (regulator_count_voltages(mcu_info->vcc_l8c_1p8) > 0)
        {
                ret = regulator_set_voltage(mcu_info->vcc_l8c_1p8, 1800000, 1800000);
                if (ret)
                {
                        pr_err("[MCU] Regulator set_vtg failed vcc_l8c_1p8 ret=%d", ret);
                        goto reg_vdd_put;
                }
        }

	mcu_info->vcc_s4a_1p8 = regulator_get(&mcu_info->i2c_client->dev, "vcc_s4a_1p8");
	if (IS_ERR(mcu_info->vcc_s4a_1p8))
	{
					ret = PTR_ERR(mcu_info->vcc_s4a_1p8);
					pr_err("[MCU] Regulator get failed vdd ret=%d", ret);
					return ret;
	}
	
	if (regulator_count_voltages(mcu_info->vcc_s4a_1p8) > 0)
	{
					ret = regulator_set_voltage(mcu_info->vcc_s4a_1p8, 1800000, 1800000);
					if (ret)
					{
									pr_err("[MCU] Regulator set_vtg failed vcc_l8c_1p8 ret=%d", ret);
									goto reg_vdd_put;
					}
	}


	ret = gpio_request(mcu_info->mcu_5v_boost_enable, "gpio_msp430fr423111_5v_boost_enable");
	if (ret < 0) {
		pr_err("Randy [MCU][msp430fr23111 error]%s: gpio %d request failed (%d)\n",
			__func__, mcu_info->mcu_5v_boost_enable, ret);
		return ret;
	}

	ret = gpio_direction_output(mcu_info->mcu_5v_boost_enable, 1);
	if (ret < 0) {
		pr_err(
			"Randy [MCU][msp430fr3111 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, mcu_info->mcu_5v_boost_enable, ret);
	gpio_free(mcu_info->mcu_5v_boost_enable);
	return ret;
	}

	return ret;

reg_vdd_put:
	regulator_put(mcu_info->vcc_l8c_1p8);
	return ret;
}


#ifdef CONFIG_OF
static int MSP430FR2311_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	
	D("[MCU][MSP430FR2311] %s\n", __func__);
	
	rc = of_get_named_gpio_flags(np, "MCU,mcu5V_boost_enable-gpios",
			0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read mcureset pin number\n");
		return rc;
	} 
	else
	{
		mcu_info->mcu_5v_boost_enable= rc;
 	  D("[MCU][MSP430FR2311]%s GET mcu 5v enable PIN =%d\n", __func__, rc);   
	}

	rc = of_get_named_gpio_flags(np, "MCU,mcureset-gpios",
			0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read mcureset pin number\n");
		return rc;
	} 
	else
	{
		mcu_info->mcu_reset= rc;
 	  D("[MCU][MSP430FR2311]%s GET mcu reset PIN =%d\n", __func__, rc);   
	}
	rc = of_get_named_gpio_flags(np, "MCU,mcutest-gpios",
			0, NULL);
	if (rc < 0)	{
		dev_err(dev, "Unable to read mcutest pin number\n");
		return rc;
	} 	else	{
		mcu_info->mcu_test= rc;
 	  D("[MCU][MSP430FR2311]%s GET mcu test PIN=%d \n", __func__, rc);   
	}

	rc = of_get_named_gpio_flags(np, "MCU,mcuwakeup-gpios",
			0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read mcu wakeup pin number\n");
		return rc;
	} 
	else
	{
		mcu_info->mcu_wakeup= rc;
 	  D("[MCU][MSP430FR2311]%s GET mcu wakeup PIN =%d\n", __func__, rc);   
	}
	
	rc = of_property_read_u32(np, "MCU,slave_address", &temp_val);
	if (rc)	{
		dev_err(dev, "Unable to read slave_address\n");
		return rc;
	} 	else	{
		mcu_info->slave_addr = (uint8_t)temp_val;
	}
  
	D("[MCU][MSP430FR2311]%s PARSE OK \n", __func__);

	return 0;
}
#endif

static int MSP430FR2311_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

pr_err("Randy MCU probe\n");

	mcu_info = kzalloc(sizeof(struct MSP430FR2311_info), GFP_KERNEL);
	if (!mcu_info)
		return -ENOMEM;

	/*D("[MSP430FR2311] %s: client->irq = %d\n", __func__, client->irq);*/

	mcu_info->i2c_client = client;

	mcu_info->mcu_reset=-1;
	mcu_info->mcu_test=-1;
	i2c_set_clientdata(client, mcu_info);
	mcu_info->mcu_polling_delay = msecs_to_jiffies(MCU_POLLING_DELAY);

	asus_motor_init(mcu_info);



	if( MSP430FR2311_parse_dt(&client->dev) < 0 )
	{
		ret = -EBUSY;
		goto err_platform_data_null;  
	}
	
		
        ret = MSP430FR2311_power_init();
        if (ret < 0) {
                pr_err("[MSP430FR2311 error]%s: set regulator fail\n", __func__);
        	}

				ret= initial_MSP430FR2311_gpio();
				if (ret < 0) {
					pr_err(
						"[MCU][MSP430FR2311 error]%s: fail to initial MSP430FR2311 (%d)\n",
						"Randy mcu", ret);
					goto err_platform_data_null;	
					
				}


        ret = MSP430FR2311_power_control(1);
        if (ret < 0) {
                pr_err("[MSP430FR2311 error]%s: enable regulator fail\n", __func__);
								goto err_platform_data_null;	
        	}


//	if (MSP430FR2311_Check_Version() && MSP43FR2311_Go_BSL_Mode()) {
		MCUState=MCU_EMPTY;
//		pr_err("[MCU][MSP430FR2311 ]%s: Fail\n", __func__);		
//		goto err_initial_MSP430FR2311_gpio;
//	}


	mutex_init(&MSP430FR2311_control_mutex);

	ret = mcu_setup();
	if (ret < 0) {
		pr_err("[PS][MSP430FR2311 error]%s: mcu_setup error!!\n",
			__func__);
		goto err_mcu_setup;
	}
  

//	if (MCUState!=MCU_READY) {
		mcu_info->mcu_wq = create_singlethread_workqueue("MSP430FR2311_wq");
		if (!mcu_info->mcu_wq) {
			pr_err("[MCU][MSP430FR2311 error]%s: can't create workqueue\n", __func__);
			ret = -ENOMEM;
			goto err_create_singlethread_workqueue;
		}
		queue_delayed_work(mcu_info->mcu_wq, &report_work, mcu_info->mcu_polling_delay);		 
//	}

//	ret = MSP430FR2311_setup();
//	if (ret < 0) {
//		pr_err("[MCU][MSP430FR2311 error]%s: MSP430FR2311_setup error!\n", __func__);
//		goto err_MSP430FR2311_setup;
//	}

	mcu_info->MSP430FR2311_class = class_create(THIS_MODULE, "TI_mcu");
	if (IS_ERR(mcu_info->MSP430FR2311_class)) {
		ret = PTR_ERR(mcu_info->MSP430FR2311_class);
		mcu_info->MSP430FR2311_class = NULL;
		goto err_create_class;
	}

	mcu_info->mcu_dev = device_create(mcu_info->MSP430FR2311_class,
				NULL, 0, "%s", "mcu");
	if (unlikely(IS_ERR(mcu_info->mcu_dev))) {
		ret = PTR_ERR(mcu_info->mcu_dev);
		mcu_info->mcu_dev = NULL;
		goto err_create_mcu_device;
	}	



//		ret = MSP430FR2311_power_control(0);

	fManualMode=ManualMode_AfterAndPR2;
	D("[MCU][MSP430FR2311] %s: Probe success, manual mode=PR2!\n", __func__);

	return ret;

err_create_mcu_device:
  device_destroy(mcu_info->MSP430FR2311_class, mcu_info->mcu_dev->devt);
	class_destroy(mcu_info->MSP430FR2311_class);
err_create_class:
	if (mcu_info->mcu_wq) destroy_workqueue(mcu_info->mcu_wq);
err_create_singlethread_workqueue:
err_mcu_setup:
	mutex_destroy(&MSP430FR2311_control_mutex);
	misc_deregister(&mcu_misc); //lightsensor_setup
//err_initial_MSP430FR2311_gpio:
		gpio_free(mcu_info->mcu_reset); 
		gpio_free(mcu_info->mcu_test); 
err_platform_data_null:
	kfree(mcu_info);
	g_motor_status = 0; //probe fail
	return ret;
}
   

static const struct i2c_device_id MSP430FR2311_i2c_id[] = {
	{MSP430FR2311_I2C_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static struct of_device_id MSP430FR2311_match_table[] = {
	{ .compatible = "MCU,MSP430FR2311",},
	{ },
};
#else
#define MSP430FR2311_match_table NULL
#endif

#ifdef CONFIG_PM_SLEEP
static int mcu_suspend(struct device *dev)
{
	struct mcu_info *mpi;
	mpi = dev_get_drvdata(dev);
	pr_err("[MCU] go to power off");
	MSP430FR2311_power_control(0);

	return 0;
}

static int mcu_resume(struct device *dev)
{
	struct mcu_info *mpi;
	mpi = dev_get_drvdata(dev);
	pr_err("[MCU] go to power on");
	MSP430FR2311_power_control(1);


	return 0;
}
#endif


static UNIVERSAL_DEV_PM_OPS(mcu_pm, mcu_suspend, mcu_resume, NULL);


static struct i2c_driver MSP430FR2311_driver = {
	.id_table = MSP430FR2311_i2c_id,
	.probe = MSP430FR2311_probe,
	.driver = {
		.name = MSP430FR2311_I2C_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
			.pm = &mcu_pm,	
#endif
		.of_match_table = of_match_ptr(MSP430FR2311_match_table),     
	},
};

static int __init MSP430FR2311_init(void)
{
	return i2c_add_driver(&MSP430FR2311_driver);
}

static void __exit MSP430FR2311_exit(void)
{
	i2c_del_driver(&MSP430FR2311_driver);
}

module_init(MSP430FR2311_init);
module_exit(MSP430FR2311_exit);

MODULE_AUTHOR("Randy Change <randy_change@asus.com>");
MODULE_DESCRIPTION("MCU MSP430FR2311 micro processor Driver");
MODULE_LICENSE("GPL v2");
