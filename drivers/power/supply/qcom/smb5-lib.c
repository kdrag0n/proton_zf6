/* Copyright (c) 2018-2019 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/irq.h>
#include <linux/iio/consumer.h>
#include <linux/pmic-voter.h>
#include <linux/of_batterydata.h>
#include "smb5-lib.h"
#include "smb5-reg.h"
#include "battery.h"
#include "schgm-flash.h"
#include "step-chg-jeita.h"
#include "storm-watch.h"
#include "schgm-flash.h"
//[+++]ASUS : Add include files
#include "fg-core.h"
#include "fg-reg.h"
#include "fg-alg.h"
#include "battery.h"
#include <linux/gpio.h>
#include <linux/alarmtimer.h>
#include <linux/atomic.h>
//[---]ASUS : Add include files

#include "../../../usb/pd/usbpd.h"

#ifdef CONFIG_ASUS_PD_CHARGER
#define ASUS_PD_INPUT_VOL_CFG_9V 9000000	// 9V
#define ASUS_PD_INPUT_VOL_CFG_7_5V 7500000	// 7.5V
#define ASUS_PD_INPUT_VOL_CFG_5V 5000000	// 5V
#define ASUS_PD_ICL_CUR_CFG_3A 3000000		// 3A
#define ASUS_PD_ICL_CUR_CFG_2A 2000000		// 2A
#define ASUS_PD_ICL_CUR_CFG_1_7A 1700000	// 1.7A
#define ASUS_PD_ICL_CUR_CFG_1_5A 1500000	// 1.5A

u32 default_src_caps[] = { 0x360190c8 };  /* VSafe5V @ 2A */
int default_src_caps_size = ARRAY_SIZE(default_src_caps);
int min_sink_current = 500;
#endif

#define smblib_err(chg, fmt, ...)		\
	pr_err("%s: %s: " fmt, chg->name,	\
		__func__, ##__VA_ARGS__)	\

#define smblib_dbg(chg, reason, fmt, ...)			\
	do {							\
		if (*chg->debug_mask & (reason))		\
			pr_info("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)

#define typec_rp_med_high(chg, typec_mode)			\
	((typec_mode == POWER_SUPPLY_TYPEC_SOURCE_MEDIUM	\
	|| typec_mode == POWER_SUPPLY_TYPEC_SOURCE_HIGH)	\
	&& !chg->typec_legacy)

//[+++]ASUS : Add delayed works
#define ICL_500mA	0x0A
extern struct smb_charger *smbchg_dev;  //global smb_charger
extern struct gpio_control *global_gpio;  //global gpio_control
extern struct timespec last_jeita_time;
static struct alarm bat_alarm;
//[---]ASUS : Add delayed works

//[+++]ASUS : Add variables
static int ASUS_ADAPTER_ID = 0;
static int HVDCP_FLAG = 0;
static int UFP_FLAG = 0;
static int LEGACY_CABLE_FLAG = 2; // 0: non-legacy, 1: noncomp-legacy, 2: legacy
static bool asus_flow_processing = 0;
volatile bool asus_adapter_detecting_flag = 0;
bool asus_flow_done_flag = 0;
int asus_CHG_TYPE = 0;
extern int charger_limit_value;
extern int charger_limit_enable_flag;
extern bool no_input_suspend_flag;
extern bool g_Charger_mode;
int vbus_rising_count = 0;
u8 asus_set_icl = ICL_500mA;
static int Total_FCC_Value = 1750000;  //Add the interface for charging debug apk
extern bool g_usb_water_enable;
extern bool g_usb_thermal_enable;
extern int g_usb_thermal_debug;
extern bool demo_app_status_flag;
static atomic_t feature_stop_need_release = ATOMIC_INIT(0);
extern bool ultra_bat_life_flag;
extern bool smartchg_stop_flag;
bool qc_stat_registed = false;
static bool usb_thermal_once_flag = false;  //In both MOS and COS when conn_temp >= 700, do not resume charging unless user unplugs and conn_temp <= 500 simultaneously
int g_usb_otg = 0;

//ASUS_BSP battery safety upgrade +++
int FV_JEITA_uV = 4360000;
//ASUS_BSP battery safety upgrade ---
//[---]ASUS : Add variables

//[+++]ASUS BSP gauge
extern struct fg_gen4_chip *g_fgChip;
extern struct fg_dev *g_fg;
extern int fg_get_msoc(struct fg_dev *fg, int *msoc);
//[---]ASUS BSP gauge

int asus_get_prop_batt_temp(struct smb_charger *chg);
int asus_get_prop_batt_volt(struct smb_charger *chg);
int asus_get_prop_batt_current(struct smb_charger *chg);
int asus_get_prop_batt_capacity(struct smb_charger *chg);
int asus_get_prop_batt_health(struct smb_charger *chg);
int asus_get_prop_usb_present(struct smb_charger *chg);
//[+++]ASUS : Add the interface for charging debug apk
int asus_get_prop_adapter_id(void);
int asus_get_prop_is_legacy_cable(void);
int asus_get_prop_total_fcc(void);
int asus_get_apsd_result_by_bit(void);
//[---]ASUS : Add the interface for charging debug apk
extern void asus_extcon_set_fnode_name(struct extcon_dev *edev, const char *fname);
extern int asus_extcon_set_state_sync(struct extcon_dev *edev, int cable_state);

enum ADAPTER_ID {
	NONE = 0,
	ASUS_750K,
	ASUS_200K,
	PB,
	OTHERS,
	ADC_NOT_READY,
};

static char *asus_id[] = {
	"NONE",
	"ASUS_750K",
	"ASUS_200K",
	"PB",
	"OTHERS",
	"ADC_NOT_READY"
};

char *ufp_type[] = {
	"NONE",
	"DEFAULT",
	"MEDIUM",
	"HIGH",
	"OTHERS"
};

char *health_type[] = {
	"GOOD",
	"COLD",
	"COOL",
	"WARM",
	"OVERHEAT",
	"OVERVOLT",
	"OTHERS"
};

char *bat_status_text[] = {
	"Unknown",
	"Charging",
	"Discharging",
	"Not charging",
	"Full",
	"Quick charging",
	"Quick charging plus",
	"Thermal alert"
};

char *chg_status_text[] = {
	"Inhibit",
	"Trickle",
	"Precharge",
	"Fullon",
	"Taper",
	"Terminate",
	"Pause",
	"Disable"
};

struct wakeup_source asus_chg_ws;
static bool create_asus_chg_ws = false;
extern bool asus_chg_ws_disable;

#ifdef CONFIG_ASUS_PD_CHARGER
static final_pd_mw = 5000;
int * chg_eval_src_caps_asus(int src_cap_cnt, u32 *received_pdos)
{
	int i = 0, index = 0;
	int cur_mw = 0, cur_uv = 0, cur_ua = 0;
	int pre_mw = 0, pre_uv = 0, pre_ua = 0;
	int pdo_candidate = 0, pdo_candidate_9v = 0;
	static int chg_rdo[3];
	for (i = 0; i < src_cap_cnt; i++) {
		if (PD_SRC_PDO_TYPE(received_pdos[i]) != PD_SRC_PDO_TYPE_FIXED) {
			pr_info("[CHG][PD] src_cap %d invalid! %08x\n", i, received_pdos[i]);
			continue;
		}

		cur_uv = PD_SRC_PDO_FIXED_VOLTAGE(received_pdos[i]) * 50 * 1000;
		cur_ua = PD_SRC_PDO_FIXED_MAX_CURR(received_pdos[i]) * 10 * 1000;

		if (ASUS_PD_INPUT_VOL_CFG_5V == cur_uv) {
			cur_ua = (cur_ua > ASUS_PD_ICL_CUR_CFG_3A) ? ASUS_PD_ICL_CUR_CFG_3A : cur_ua;
		} else if (cur_uv > ASUS_PD_INPUT_VOL_CFG_5V && cur_uv <= ASUS_PD_INPUT_VOL_CFG_7_5V) {
			cur_ua = (cur_ua > ASUS_PD_ICL_CUR_CFG_2A) ? ASUS_PD_ICL_CUR_CFG_2A : cur_ua;
		} else if (cur_uv > ASUS_PD_INPUT_VOL_CFG_7_5V&& cur_uv <= ASUS_PD_INPUT_VOL_CFG_9V) {
			cur_ua = (cur_ua > ASUS_PD_ICL_CUR_CFG_1_7A) ? ASUS_PD_ICL_CUR_CFG_1_7A : cur_ua;
		} else {
			pr_info("[CHG][PD] pdo-%d: uv (%d), ua (%d), out of spec! skip!\n", i, cur_uv, cur_ua);
			continue;
		}

		pdo_candidate = 0;
		cur_mw = (cur_uv / 1000) * (cur_ua / 1000);
		pr_info("[CHG][PD] pdo-%d: uv (%d), ua (%d), mw (%d) meet spec!\n", i, cur_uv, cur_ua, cur_mw);

		/*
		  *	Limit Input Current:
		  *	ICL_CFG Rule: (by PDO Voltage)
		  *	Vin = 5V			=> Set I Adp_max = min(3A, I PDO )
		  *	5V < Vin <= 7.5V	=> Set I Adp_max = min(2A, I PDO )
		  *	7.5V < Vin <= 9V	=> Set I Adp_max = min(1.7A, I PDO )
		  *	9V < Vin			=> Set I Adp_max = 0A
		  *
		  *	PDO Select Condition:
		  *	1. Max Padp profile, no more than 15W
		  *	2. If two Padp profiles are the same:
		  * 	- Selecting the one Vin = 9V
		  * 	- If there is no pdo with Vin = 9V, selecting the one Vin is smaller.
		  */

		if (cur_mw > pre_mw) {
			pdo_candidate = 1;
			if (ASUS_PD_INPUT_VOL_CFG_9V == cur_uv) {
				pdo_candidate_9v = 1;
			} else {
				pdo_candidate_9v = 0;
			}
		} else if (cur_mw == pre_mw && !pdo_candidate_9v) {
			if (ASUS_PD_INPUT_VOL_CFG_9V == cur_uv) {
				pdo_candidate = 1;
				pdo_candidate_9v = 1;
			} else if (cur_uv <  pre_uv) {
				pdo_candidate = 1;
			}
		}

		if (pdo_candidate) {
			pre_mw = cur_mw;
			pre_uv = cur_uv;
			pre_ua = cur_ua;
			index = i;
			pr_info("[CHG][PD] pdo-%d: uv (%d), ua (%d), candidate!\n", i, pre_uv, pre_ua);
		}
	}

	/*
	 * chg_rdo[0] => pdo number
	 * chg_rdo[1] => request voltage
	 * chg_rdo[2] => request current
	 * */

	chg_rdo[0] = index;
	chg_rdo[1] = pre_uv;
	chg_rdo[2] = pre_ua;

	final_pd_mw = (pre_uv / 1000) * (pre_ua / 1000);
	pr_info("[CHG][PD]: uv (%d), ua (%d), mw (%d) final!\n", pre_uv, pre_ua, final_pd_mw);
	
	return chg_rdo;
}
#endif

void asus_smblib_stay_awake(struct smb_charger *chg)
{
	if (asus_chg_ws_disable)
		return;

	if (create_asus_chg_ws) {
		CHG_DBG("ASUS set awake\n");
		__pm_stay_awake(&asus_chg_ws);
	} else {
		CHG_DBG_E("ASUS set awake fail, asus_chg_ws not initial\n");
		wakeup_source_init(&asus_chg_ws, "asus_chg_ws");
		create_asus_chg_ws = true;

		CHG_DBG("ASUS set awake after asus_chg_ws initial\n");
		__pm_stay_awake(&asus_chg_ws);
	}
}

void asus_smblib_relax(struct smb_charger *chg)
{
	if (asus_chg_ws_disable)
		return;

	if (create_asus_chg_ws) {
		CHG_DBG("ASUS set relax\n");
		__pm_relax(&asus_chg_ws);
	} else {
		CHG_DBG_E("ASUS set relax fail, asus_chg_ws not initial\n");
	}
}

static void update_sw_icl_max(struct smb_charger *chg, int pst);

int smblib_read(struct smb_charger *chg, u16 addr, u8 *val)
{
	unsigned int value;
	int rc = 0;

	rc = regmap_read(chg->regmap, addr, &value);
	if (rc >= 0)
		*val = (u8)value;

	return rc;
}

int smblib_batch_read(struct smb_charger *chg, u16 addr, u8 *val,
			int count)
{
	return regmap_bulk_read(chg->regmap, addr, val, count);
}

int smblib_write(struct smb_charger *chg, u16 addr, u8 val)
{
	return regmap_write(chg->regmap, addr, val);
}

int smblib_batch_write(struct smb_charger *chg, u16 addr, u8 *val,
			int count)
{
	return regmap_bulk_write(chg->regmap, addr, val, count);
}

int smblib_masked_write(struct smb_charger *chg, u16 addr, u8 mask, u8 val)
{
	return regmap_update_bits(chg->regmap, addr, mask, val);
}

int smblib_get_iio_channel(struct smb_charger *chg, const char *propname,
					struct iio_channel **chan)
{
	int rc = 0;

	rc = of_property_match_string(chg->dev->of_node,
					"io-channel-names", propname);
	if (rc < 0)
		return 0;

	*chan = iio_channel_get(chg->dev, propname);
	if (IS_ERR(*chan)) {
		rc = PTR_ERR(*chan);
		if (rc != -EPROBE_DEFER)
			smblib_err(chg, "%s channel unavailable, %d\n",
							propname, rc);
		*chan = NULL;
	}

	return rc;
}

#define DIV_FACTOR_MICRO_V_I	1
#define DIV_FACTOR_MILI_V_I	1000
#define DIV_FACTOR_DECIDEGC	100
int smblib_read_iio_channel(struct smb_charger *chg, struct iio_channel *chan,
							int div, int *data)
{
	int rc = 0;
	*data = -ENODATA;

	if (chan) {
		rc = iio_read_channel_processed(chan, data);
		if (rc < 0) {
			smblib_err(chg, "Error in reading IIO channel data, rc=%d\n",
					rc);
			return rc;
		}

		if (div != 0)
			*data /= div;
	}

	return rc;
}

int smblib_get_jeita_cc_delta(struct smb_charger *chg, int *cc_delta_ua)
{
	int rc, cc_minus_ua;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_7_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}

	if (stat & BAT_TEMP_STATUS_HOT_SOFT_BIT) {
		rc = smblib_get_charge_param(chg, &chg->param.jeita_cc_comp_hot,
					&cc_minus_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get jeita cc minus rc=%d\n",
					rc);
			return rc;
		}
	} else if (stat & BAT_TEMP_STATUS_COLD_SOFT_BIT) {
		rc = smblib_get_charge_param(chg,
					&chg->param.jeita_cc_comp_cold,
					&cc_minus_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get jeita cc minus rc=%d\n",
					rc);
			return rc;
		}
	} else {
		cc_minus_ua = 0;
	}

	*cc_delta_ua = -cc_minus_ua;

	return 0;
}

int smblib_icl_override(struct smb_charger *chg, enum icl_override_mode  mode)
{
	int rc;
	u8 usb51_mode, icl_override, apsd_override;

	switch (mode) {
	case SW_OVERRIDE_USB51_MODE:
		usb51_mode = 0;
		icl_override = ICL_OVERRIDE_BIT;
		apsd_override = 0;
		break;
	case SW_OVERRIDE_HC_MODE:
		usb51_mode = USBIN_MODE_CHG_BIT;
		icl_override = 0;
		apsd_override = ICL_OVERRIDE_AFTER_APSD_BIT;
		break;
	case HW_AUTO_MODE:
	default:
		usb51_mode = USBIN_MODE_CHG_BIT;
		icl_override = 0;
		apsd_override = 0;
		break;
	}

	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
				USBIN_MODE_CHG_BIT, usb51_mode);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set USBIN_ICL_OPTIONS rc=%d\n", rc);
		return rc;
	}

	rc = smblib_masked_write(chg, CMD_ICL_OVERRIDE_REG,
				ICL_OVERRIDE_BIT, icl_override);
	if (rc < 0) {
		smblib_err(chg, "Couldn't override ICL rc=%d\n", rc);
		return rc;
	}

	rc = smblib_masked_write(chg, USBIN_LOAD_CFG_REG,
				ICL_OVERRIDE_AFTER_APSD_BIT, apsd_override);
	if (rc < 0) {
		smblib_err(chg, "Couldn't override ICL_AFTER_APSD rc=%d\n", rc);
		return rc;
	}

	return rc;
}

/*
 * This function does smb_en pin access, which is lock protected.
 * It should be called with smb_lock held.
 */
static int smblib_select_sec_charger_locked(struct smb_charger *chg,
					int sec_chg)
{
	int rc = 0;

	switch (sec_chg) {
	case POWER_SUPPLY_CHARGER_SEC_CP:
		vote(chg->pl_disable_votable, PL_SMB_EN_VOTER, true, 0);

		/* select Charge Pump instead of slave charger */
		rc = smblib_masked_write(chg, MISC_SMB_CFG_REG,
					SMB_EN_SEL_BIT, SMB_EN_SEL_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't select SMB charger rc=%d\n",
				rc);
			return rc;
		}
		/* Enable Charge Pump, under HW control */
		rc = smblib_masked_write(chg, MISC_SMB_EN_CMD_REG,
					EN_CP_CMD_BIT, EN_CP_CMD_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't enable SMB charger rc=%d\n",
						rc);
			return rc;
		}
		vote(chg->smb_override_votable, PL_SMB_EN_VOTER, false, 0);
		break;
	case POWER_SUPPLY_CHARGER_SEC_PL:
		/* select slave charger instead of Charge Pump */
		rc = smblib_masked_write(chg, MISC_SMB_CFG_REG,
					SMB_EN_SEL_BIT, 0);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't select SMB charger rc=%d\n",
				rc);
			return rc;
		}
		/* Enable slave charger, under HW control */
		rc = smblib_masked_write(chg, MISC_SMB_EN_CMD_REG,
					EN_STAT_CMD_BIT, EN_STAT_CMD_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't enable SMB charger rc=%d\n",
						rc);
			return rc;
		}
		vote(chg->smb_override_votable, PL_SMB_EN_VOTER, false, 0);

		vote(chg->pl_disable_votable, PL_SMB_EN_VOTER, false, 0);

		break;
	case POWER_SUPPLY_CHARGER_SEC_NONE:
	default:
		vote(chg->pl_disable_votable, PL_SMB_EN_VOTER, true, 0);

		/* SW override, disabling secondary charger(s) */
		vote(chg->smb_override_votable, PL_SMB_EN_VOTER, true, 0);
		break;
	}

	return rc;
}

static int smblib_select_sec_charger(struct smb_charger *chg, int sec_chg,
					int reason, bool toggle)
{
	int rc;

	mutex_lock(&chg->smb_lock);

	if (toggle && sec_chg == POWER_SUPPLY_CHARGER_SEC_CP) {
		rc = smblib_select_sec_charger_locked(chg,
					POWER_SUPPLY_CHARGER_SEC_NONE);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't disable secondary charger rc=%d\n",
				rc);
			goto unlock_out;
		}

		/*
		 * A minimum of 20us delay is expected before switching on STAT
		 * pin.
		 */
		usleep_range(20, 30);
	}

	rc = smblib_select_sec_charger_locked(chg, sec_chg);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't switch secondary charger rc=%d\n",
			rc);
		goto unlock_out;
	}

	chg->sec_chg_selected = sec_chg;
	chg->cp_reason = reason;

unlock_out:
	mutex_unlock(&chg->smb_lock);

	return rc;
}

static void smblib_notify_extcon_props(struct smb_charger *chg, int id)
{
	union extcon_property_value val;
	union power_supply_propval prop_val;

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_TYPEC) {
		smblib_get_prop_typec_cc_orientation(chg, &prop_val);
		val.intval = ((prop_val.intval == 2) ? 1 : 0);
		extcon_set_property(chg->extcon, id,
				EXTCON_PROP_USB_TYPEC_POLARITY, val);
		val.intval = true;
		extcon_set_property(chg->extcon, id,
				EXTCON_PROP_USB_SS, val);
	} else if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB) {
		val.intval = false;
		extcon_set_property(chg->extcon, id,
				EXTCON_PROP_USB_SS, val);
	}
}

static void smblib_notify_device_mode(struct smb_charger *chg, bool enable)
{
	if (enable)
		smblib_notify_extcon_props(chg, EXTCON_USB);

	extcon_set_state_sync(chg->extcon, EXTCON_USB, enable);
}

static void smblib_notify_usb_host(struct smb_charger *chg, bool enable)
{
	int rc = 0;

	if (enable) {
		smblib_dbg(chg, PR_OTG, "enabling VBUS in OTG mode\n");
		rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG,
					OTG_EN_BIT, OTG_EN_BIT);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't enable VBUS in OTG mode rc=%d\n", rc);
			return;
		}

		smblib_notify_extcon_props(chg, EXTCON_USB_HOST);
	} else {
		smblib_dbg(chg, PR_OTG, "disabling VBUS in OTG mode\n");
		rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG,
					OTG_EN_BIT, 0);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't disable VBUS in OTG mode rc=%d\n",
				rc);
			return;
		}
	}

	extcon_set_state_sync(chg->extcon, EXTCON_USB_HOST, enable);
}

/********************
 * REGISTER GETTERS *
 ********************/

int smblib_get_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int *val_u)
{
	int rc = 0;
	u8 val_raw;

	rc = smblib_read(chg, param->reg, &val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't read from 0x%04x rc=%d\n",
			param->name, param->reg, rc);
		return rc;
	}

	if (param->get_proc)
		*val_u = param->get_proc(param, val_raw);
	else
		*val_u = val_raw * param->step_u + param->min_u;
	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, *val_u, val_raw);

	return rc;
}

int smblib_get_usb_suspend(struct smb_charger *chg, int *suspend)
{
	int rc = 0;
	u8 temp;

	rc = smblib_read(chg, USBIN_CMD_IL_REG, &temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_CMD_IL rc=%d\n", rc);
		return rc;
	}
	*suspend = temp & USBIN_SUSPEND_BIT;

	return rc;
}


static const s16 therm_lookup_table[] = {
	/* Index -30C~85C, ADC raw code */
	0x6C92, 0x6C43, 0x6BF0, 0x6B98, 0x6B3A, 0x6AD8, 0x6A70, 0x6A03,
	0x6990, 0x6916, 0x6897, 0x6811, 0x6785, 0x66F2, 0x6658, 0x65B7,
	0x650F, 0x6460, 0x63AA, 0x62EC, 0x6226, 0x6159, 0x6084, 0x5FA8,
	0x5EC3, 0x5DD8, 0x5CE4, 0x5BE9, 0x5AE7, 0x59DD, 0x58CD, 0x57B5,
	0x5696, 0x5571, 0x5446, 0x5314, 0x51DD, 0x50A0, 0x4F5E, 0x4E17,
	0x4CCC, 0x4B7D, 0x4A2A, 0x48D4, 0x477C, 0x4621, 0x44C4, 0x4365,
	0x4206, 0x40A6, 0x3F45, 0x3DE6, 0x3C86, 0x3B28, 0x39CC, 0x3872,
	0x3719, 0x35C4, 0x3471, 0x3322, 0x31D7, 0x308F, 0x2F4C, 0x2E0D,
	0x2CD3, 0x2B9E, 0x2A6E, 0x2943, 0x281D, 0x26FE, 0x25E3, 0x24CF,
	0x23C0, 0x22B8, 0x21B5, 0x20B8, 0x1FC2, 0x1ED1, 0x1DE6, 0x1D01,
	0x1C22, 0x1B49, 0x1A75, 0x19A8, 0x18E0, 0x181D, 0x1761, 0x16A9,
	0x15F7, 0x154A, 0x14A2, 0x13FF, 0x1361, 0x12C8, 0x1234, 0x11A4,
	0x1119, 0x1091, 0x100F, 0x0F90, 0x0F15, 0x0E9E, 0x0E2B, 0x0DBC,
	0x0D50, 0x0CE8, 0x0C83, 0x0C21, 0x0BC3, 0x0B67, 0x0B0F, 0x0AB9,
	0x0A66, 0x0A16, 0x09C9, 0x097E,
};

int smblib_get_thermal_threshold(struct smb_charger *chg, u16 addr, int *val)
{
	u8 buff[2];
	s16 temp;
	int rc = 0;
	int i, lower, upper;

	rc = smblib_batch_read(chg, addr, buff, 2);
	if (rc < 0) {
		pr_err("failed to write to 0x%04X, rc=%d\n", addr, rc);
		return rc;
	}

	temp = buff[1] | buff[0] << 8;

	lower = 0;
	upper = ARRAY_SIZE(therm_lookup_table) - 1;
	while (lower <= upper) {
		i = (upper + lower) / 2;
		if (therm_lookup_table[i] < temp)
			upper = i - 1;
		else if (therm_lookup_table[i] > temp)
			lower = i + 1;
		else
			break;
	}

	/* index 0 corresonds to -30C */
	*val = (i - 30) * 10;

	return rc;
}

struct apsd_result {
	const char * const name;
	const u8 bit;
	const enum power_supply_type pst;
};

enum {
	UNKNOWN,
	SDP,
	CDP,
	DCP,
	OCP,
	FLOAT,
	HVDCP2,
	HVDCP3,
	MAX_TYPES
};

static const struct apsd_result smblib_apsd_results[] = {
	[UNKNOWN] = {
		.name	= "UNKNOWN",
		.bit	= 0,
		.pst	= POWER_SUPPLY_TYPE_UNKNOWN
	},
	[SDP] = {
		.name	= "SDP",
		.bit	= SDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB
	},
	[CDP] = {
		.name	= "CDP",
		.bit	= CDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_CDP
	},
	[DCP] = {
		.name	= "DCP",
		.bit	= DCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[OCP] = {
		.name	= "OCP",
		.bit	= OCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[FLOAT] = {
		.name	= "FLOAT",
		.bit	= FLOAT_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_FLOAT
	},
	[HVDCP2] = {
		.name	= "HVDCP2",
		.bit	= DCP_CHARGER_BIT | QC_2P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP
	},
	[HVDCP3] = {
		.name	= "HVDCP3",
		.bit	= DCP_CHARGER_BIT | QC_3P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP_3,
	},
};

static const struct apsd_result *smblib_get_apsd_result(struct smb_charger *chg)
{
	int rc, i;
	u8 apsd_stat, stat;
	const struct apsd_result *result = &smblib_apsd_results[UNKNOWN];

	rc = smblib_read(chg, APSD_STATUS_REG, &apsd_stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return result;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", apsd_stat);

	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT))
		return result;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_RESULT_STATUS rc=%d\n",
			rc);
		return result;
	}
	stat &= APSD_RESULT_STATUS_MASK;

	for (i = 0; i < ARRAY_SIZE(smblib_apsd_results); i++) {
		if (smblib_apsd_results[i].bit == stat)
			result = &smblib_apsd_results[i];
	}

	if (apsd_stat & QC_CHARGER_BIT) {
		/* since its a qc_charger, either return HVDCP3 or HVDCP2 */
		if (result != &smblib_apsd_results[HVDCP3])
			result = &smblib_apsd_results[HVDCP2];
	}

	return result;
}

#define INPUT_NOT_PRESENT	0
#define INPUT_PRESENT_USB	BIT(1)
#define INPUT_PRESENT_DC	BIT(2)
static int smblib_is_input_present(struct smb_charger *chg,
				   int *present)
{
	int rc;
	union power_supply_propval pval = {0, };

	*present = INPUT_NOT_PRESENT;

	rc = smblib_get_prop_usb_present(chg, &pval);
	if (rc < 0) {
		pr_err("Couldn't get usb presence status rc=%d\n", rc);
		return rc;
	}
	*present |= pval.intval ? INPUT_PRESENT_USB : INPUT_NOT_PRESENT;

	rc = smblib_get_prop_dc_present(chg, &pval);
	if (rc < 0) {
		pr_err("Couldn't get dc presence status rc=%d\n", rc);
		return rc;
	}
	*present |= pval.intval ? INPUT_PRESENT_DC : INPUT_NOT_PRESENT;

	return 0;
}

#define AICL_RANGE2_MIN_MV		5600
#define AICL_RANGE2_STEP_DELTA_MV	200
#define AICL_RANGE2_OFFSET		16
int smblib_get_aicl_cont_threshold(struct smb_chg_param *param, u8 val_raw)
{
	int base = param->min_u;
	u8 reg = val_raw;
	int step = param->step_u;


	if (val_raw >= AICL_RANGE2_OFFSET) {
		reg = val_raw - AICL_RANGE2_OFFSET;
		base = AICL_RANGE2_MIN_MV;
		step = AICL_RANGE2_STEP_DELTA_MV;
	}

	return base + (reg * step);
}

/********************
 * REGISTER SETTERS *
 ********************/
static const struct buck_boost_freq chg_freq_list[] = {
	[0] = {
		.freq_khz	= 2400,
		.val		= 7,
	},
	[1] = {
		.freq_khz	= 2100,
		.val		= 8,
	},
	[2] = {
		.freq_khz	= 1600,
		.val		= 11,
	},
	[3] = {
		.freq_khz	= 1200,
		.val		= 15,
	},
};

int smblib_set_chg_freq(struct smb_chg_param *param,
				int val_u, u8 *val_raw)
{
	u8 i;

	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	/* Charger FSW is the configured freqency / 2 */
	val_u *= 2;
	for (i = 0; i < ARRAY_SIZE(chg_freq_list); i++) {
		if (chg_freq_list[i].freq_khz == val_u)
			break;
	}
	if (i == ARRAY_SIZE(chg_freq_list)) {
		pr_err("Invalid frequency %d Hz\n", val_u / 2);
		return -EINVAL;
	}

	*val_raw = chg_freq_list[i].val;

	return 0;
}

int smblib_set_opt_switcher_freq(struct smb_charger *chg, int fsw_khz)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher, fsw_khz);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_buck rc=%d\n", rc);

	if (chg->mode == PARALLEL_MASTER && chg->pl.psy) {
		pval.intval = fsw_khz;
		/*
		 * Some parallel charging implementations may not have
		 * PROP_BUCK_FREQ property - they could be running
		 * with a fixed frequency
		 */
		power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_BUCK_FREQ, &pval);
	}

	return rc;
}

int smblib_set_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int val_u)
{
	int rc = 0;
	u8 val_raw;

	if (param->set_proc) {
		rc = param->set_proc(param, val_u, &val_raw);
		if (rc < 0)
			return -EINVAL;
	} else {
		if (val_u > param->max_u || val_u < param->min_u)
			smblib_dbg(chg, PR_MISC,
				"%s: %d is out of range [%d, %d]\n",
				param->name, val_u, param->min_u, param->max_u);

		if (val_u > param->max_u)
			val_u = param->max_u;
		if (val_u < param->min_u)
			val_u = param->min_u;

		val_raw = (val_u - param->min_u) / param->step_u;
	}

	rc = smblib_write(chg, param->reg, val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't write 0x%02x to 0x%04x rc=%d\n",
			param->name, val_raw, param->reg, rc);
		return rc;
	}

	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, val_u, val_raw);
	CHG_DBG("start, %s = %d (0x%02x)\n", param->name, val_u, val_raw);

	return rc;
}

extern int asus_extcon_get_state(struct extcon_dev *edev);
int smblib_set_usb_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;
	if (suspend)
		vote(chg->icl_irq_disable_votable, USB_SUSPEND_VOTER,
				true, 0);

	if (usb_thermal_once_flag) {
		CHG_DBG("[usb_thermal] Force to suspend charger input because of conn_temp\n");
		suspend = 1;
		//ASUSErclog(ASUS_USB_THERMAL_ALERT, "USB Thermal Alert is triggered");
	}

	if (no_input_suspend_flag) {
		CHG_DBG("Start thermal test, unable to suspend input\n");
		suspend = 0;
	}

	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT,
				 suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to USBIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	if (!suspend)
		vote(chg->icl_irq_disable_votable, USB_SUSPEND_VOTER,
				false, 0);

	return rc;
}

int smblib_set_dc_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_SUSPEND_BIT,
				 suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to DCIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

static int smblib_set_adapter_allowance(struct smb_charger *chg,
					u8 allowed_voltage)
{
	int rc = 0;

	/* PMI632 only support max. 9V */
	if (chg->smb_version == PMI632_SUBTYPE) {
		switch (allowed_voltage) {
		case USBIN_ADAPTER_ALLOW_12V:
		case USBIN_ADAPTER_ALLOW_9V_TO_12V:
			allowed_voltage = USBIN_ADAPTER_ALLOW_9V;
			break;
		case USBIN_ADAPTER_ALLOW_5V_OR_12V:
		case USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V:
			allowed_voltage = USBIN_ADAPTER_ALLOW_5V_OR_9V;
			break;
		case USBIN_ADAPTER_ALLOW_5V_TO_12V:
			allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_9V;
			break;
		}
	}

	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG, allowed_voltage);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to USBIN_ADAPTER_ALLOW_CFG rc=%d\n",
			allowed_voltage, rc);
		return rc;
	}

	return rc;
}

#define MICRO_5V	5000000
#define MICRO_9V	9000000
#define MICRO_12V	12000000
static int smblib_set_usb_pd_fsw(struct smb_charger *chg, int voltage)
{
	int rc = 0;

	if (voltage == MICRO_5V)
		rc = smblib_set_opt_switcher_freq(chg, chg->chg_freq.freq_5V);
	else if (voltage > MICRO_5V && voltage < MICRO_9V)
		rc = smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_6V_8V);
	else if (voltage >= MICRO_9V && voltage < MICRO_12V)
		rc = smblib_set_opt_switcher_freq(chg, chg->chg_freq.freq_9V);
	else if (voltage == MICRO_12V)
		rc = smblib_set_opt_switcher_freq(chg, chg->chg_freq.freq_12V);
	else {
		smblib_err(chg, "Couldn't set Fsw: invalid voltage %d\n",
				voltage);
		return -EINVAL;
	}

	return rc;
}

static int smblib_set_usb_pd_allowed_voltage(struct smb_charger *chg,
					int min_allowed_uv, int max_allowed_uv)
{
	int rc;
	u8 allowed_voltage;

	if (min_allowed_uv == MICRO_5V && max_allowed_uv == MICRO_5V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V;
	} else if (min_allowed_uv == MICRO_9V && max_allowed_uv == MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V;
	} else if (min_allowed_uv == MICRO_12V && max_allowed_uv == MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_12V;
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_9V;
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_12V;
	} else if (min_allowed_uv < MICRO_12V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V_TO_12V;
	} else {
		smblib_err(chg, "invalid allowed voltage [%d, %d]\n",
			min_allowed_uv, max_allowed_uv);
		return -EINVAL;
	}

	rc = smblib_set_adapter_allowance(chg, allowed_voltage);
	if (rc < 0) {
		smblib_err(chg, "Couldn't configure adapter allowance rc=%d\n",
				rc);
		return rc;
	}

	return rc;
}

int smblib_set_aicl_cont_threshold(struct smb_chg_param *param,
				int val_u, u8 *val_raw)
{
	int base = param->min_u;
	int offset = 0;
	int step = param->step_u;

	if (val_u > param->max_u)
		val_u = param->max_u;
	if (val_u < param->min_u)
		val_u = param->min_u;

	if (val_u >= AICL_RANGE2_MIN_MV) {
		base = AICL_RANGE2_MIN_MV;
		step = AICL_RANGE2_STEP_DELTA_MV;
		offset = AICL_RANGE2_OFFSET;
	};

	*val_raw = ((val_u - base) / step) + offset;

	return 0;
}

/********************
 * HELPER FUNCTIONS *
 ********************/

int smblib_get_prop_from_bms(struct smb_charger *chg,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy, psp, val);

	return rc;
}

void smblib_apsd_enable(struct smb_charger *chg, bool enable)
{
	int rc;

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				BC1P2_SRC_DETECT_BIT,
				enable ? BC1P2_SRC_DETECT_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "failed to write USBIN_OPTIONS_1_CFG rc=%d\n",
				rc);
}

void smblib_hvdcp_detect_enable(struct smb_charger *chg, bool enable)
{
	int rc;
	u8 mask;

	if (chg->hvdcp_disable || chg->pd_not_supported)
		return;

	mask = HVDCP_AUTH_ALG_EN_CFG_BIT | HVDCP_EN_BIT;
	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG, mask,
						enable ? mask : 0);
	if (rc < 0)
		smblib_err(chg, "failed to write USBIN_OPTIONS_1_CFG rc=%d\n",
				rc);

	return;
}

void smblib_hvdcp_exit_config(struct smb_charger *chg)
{
	u8 stat;
	int rc;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0)
		return;

	if (stat & (QC_3P0_BIT | QC_2P0_BIT)) {
		/* force HVDCP to 5V */
		smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT, 0);
		smblib_write(chg, CMD_HVDCP_2_REG, FORCE_5V_BIT);

		/* rerun APSD */
		smblib_masked_write(chg, CMD_APSD_REG, APSD_RERUN_BIT,
				APSD_RERUN_BIT);
	}
}

static int smblib_request_dpdm(struct smb_charger *chg, bool enable)
{
	int rc = 0;

	if (chg->pr_swap_in_progress)
		return 0;

	/* fetch the DPDM regulator */
	if (!chg->dpdm_reg && of_get_property(chg->dev->of_node,
				"dpdm-supply", NULL)) {
		chg->dpdm_reg = devm_regulator_get(chg->dev, "dpdm");
		if (IS_ERR(chg->dpdm_reg)) {
			rc = PTR_ERR(chg->dpdm_reg);
			smblib_err(chg, "Couldn't get dpdm regulator rc=%d\n",
					rc);
			chg->dpdm_reg = NULL;
			return rc;
		}
	}

	if (enable) {
		if (chg->dpdm_reg && !regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "enabling DPDM regulator\n");
			rc = regulator_enable(chg->dpdm_reg);
			if (rc < 0)
				smblib_err(chg,
					"Couldn't enable dpdm regulator rc=%d\n",
					rc);
		}
	} else {
		if (chg->dpdm_reg && regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "disabling DPDM regulator\n");
			rc = regulator_disable(chg->dpdm_reg);
			if (rc < 0)
				smblib_err(chg,
					"Couldn't disable dpdm regulator rc=%d\n",
					rc);
		}
	}

	return rc;
}

static void smblib_rerun_apsd(struct smb_charger *chg)
{
	int rc;

	smblib_dbg(chg, PR_MISC, "re-running APSD\n");

	CHG_DBG("Qcom Rerun APSD\n");
	rc = smblib_masked_write(chg, CMD_APSD_REG,
				APSD_RERUN_BIT, APSD_RERUN_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't re-run APSD rc=%d\n", rc);
}

static bool asus_quick_trigger_hvdcp = false;
static const struct apsd_result *smblib_update_usb_type(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);

	/* if PD is active, APSD is disabled so won't have a valid result */
	if (chg->pd_active) {
		chg->real_charger_type = POWER_SUPPLY_TYPE_USB_PD;
	} else if (asus_adapter_detecting_flag && apsd_result->pst == POWER_SUPPLY_TYPE_UNKNOWN) {
		chg->real_charger_type = POWER_SUPPLY_TYPE_USB_DCP;  //ASUS BSP charger +++
	} else {
		/*
		 * Update real charger type only if its not FLOAT
		 * detected as as SDP
		 */
		if (!(apsd_result->pst == POWER_SUPPLY_TYPE_USB_FLOAT &&
			chg->real_charger_type == POWER_SUPPLY_TYPE_USB))
			chg->real_charger_type = apsd_result->pst;
	}

	smblib_dbg(chg, PR_MISC, "APSD=%s PD=%d\n",
					apsd_result->name, chg->pd_active);
	if (chg->asus_print_usb_src_change) {
		CHG_DBG("FAKE = %d, APSD = %s, PD = %d\n",
				asus_adapter_detecting_flag, apsd_result->name, chg->pd_active);
	}

	if (!asus_quick_trigger_hvdcp && asus_adapter_detecting_flag && (apsd_result->pst == POWER_SUPPLY_TYPE_USB_HVDCP_3)) {
		CHG_DBG("Detect HVDCP3, quick trigger asus_chg_flow\n");
		asus_quick_trigger_hvdcp = true;
		if (cancel_delayed_work(&smbchg_dev->asus_chg_flow_work))
			schedule_delayed_work(&smbchg_dev->asus_chg_flow_work, msecs_to_jiffies(0));
	}

	return apsd_result;
}

static int smblib_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct smb_charger *chg = container_of(nb, struct smb_charger, nb);

	if (!strcmp(psy->desc->name, "bms")) {
		if (!chg->bms_psy)
			chg->bms_psy = psy;
		if (ev == PSY_EVENT_PROP_CHANGED)
			schedule_work(&chg->bms_update_work);
	}

	if (chg->jeita_configured == JEITA_CFG_NONE)
		schedule_work(&chg->jeita_update_work);

	if (chg->sec_pl_present && !chg->pl.psy
		&& !strcmp(psy->desc->name, "parallel")) {
		chg->pl.psy = psy;
		schedule_work(&chg->pl_update_work);
	}

	return NOTIFY_OK;
}

static int smblib_register_notifier(struct smb_charger *chg)
{
	int rc;

	chg->nb.notifier_call = smblib_notifier_call;
	rc = power_supply_reg_notifier(&chg->nb);
	if (rc < 0) {
		smblib_err(chg, "Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_mapping_soc_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	*val_raw = val_u << 1;

	return 0;
}

int smblib_mapping_cc_delta_to_field_value(struct smb_chg_param *param,
					   u8 val_raw)
{
	int val_u  = val_raw * param->step_u + param->min_u;

	if (val_u > param->max_u)
		val_u -= param->max_u * 2;

	return val_u;
}

int smblib_mapping_cc_delta_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u - param->max_u)
		return -EINVAL;

	val_u += param->max_u * 2 - param->min_u;
	val_u %= param->max_u * 2;
	*val_raw = val_u / param->step_u;

	return 0;
}

static void smblib_uusb_removal(struct smb_charger *chg)
{
	int rc;
	struct smb_irq_data *data;
	struct storm_watch *wdata;
	int sec_charger;

	sec_charger = chg->sec_pl_present ? POWER_SUPPLY_CHARGER_SEC_PL :
				POWER_SUPPLY_CHARGER_SEC_NONE;
	smblib_select_sec_charger(chg, sec_charger, POWER_SUPPLY_CP_NONE,
					false);

	cancel_delayed_work_sync(&chg->pl_enable_work);

	if (chg->wa_flags & BOOST_BACK_WA) {
		data = chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data;
		if (data) {
			wdata = &data->storm_data;
			update_storm_count(wdata, WEAK_CHG_STORM_COUNT);
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
			vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
					false, 0);
		}
	}
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);

	/* reset both usbin current and voltage votes */
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
			is_flash_active(chg) ? SDP_CURRENT_UA : SDP_100_MA);
	vote(chg->usb_icl_votable, SW_QC3_VOTER, false, 0);
	vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER, false, 0);
	vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
	vote(chg->usb_icl_votable, THERMAL_THROTTLE_VOTER, false, 0);
	vote(chg->limited_irq_disable_votable, CHARGER_TYPE_VOTER,
			true, 0);
	vote(chg->hdc_irq_disable_votable, CHARGER_TYPE_VOTER, true, 0);
	vote(chg->hdc_irq_disable_votable, HDC_IRQ_VOTER, false, 0);

	/* Remove SW thermal regulation WA votes */
	vote(chg->usb_icl_votable, SW_THERM_REGULATION_VOTER, false, 0);
	vote(chg->pl_disable_votable, SW_THERM_REGULATION_VOTER, false, 0);
	vote(chg->dc_suspend_votable, SW_THERM_REGULATION_VOTER, false, 0);
	if (chg->cp_disable_votable)
		vote(chg->cp_disable_votable, SW_THERM_REGULATION_VOTER,
								false, 0);

	/* reconfigure allowed voltage for HVDCP */
	rc = smblib_set_adapter_allowance(chg,
			USBIN_ADAPTER_ALLOW_5V_TO_9V);
	if (rc < 0)
		smblib_err(chg, "Couldn't set USBIN_ADAPTER_ALLOW_5V_TO_9V rc=%d\n",
			rc);

	/* reset USBOV votes and cancel work */
	cancel_delayed_work_sync(&chg->usbov_dbc_work);
	vote(chg->awake_votable, USBOV_DBC_VOTER, false, 0);
	chg->dbc_usbov = false;

	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;
	chg->usbin_forced_max_uv = 0;
	chg->usb_icl_delta_ua = 0;
	chg->pulse_cnt = 0;
	chg->uusb_apsd_rerun_done = false;

	/* write back the default FLOAT charger configuration */
	rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
				(u8)FLOAT_OPTIONS_MASK, chg->float_cfg);
	if (rc < 0)
		smblib_err(chg, "Couldn't write float charger options rc=%d\n",
			rc);

	/* clear USB ICL vote for USB_PSY_VOTER */
	rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't un-vote for USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for DCP_VOTER */
	rc = vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote DCP from USB ICL rc=%d\n", rc);

	/*
	 * if non-compliant charger caused UV, restore original max pulses
	 * and turn SUSPEND_ON_COLLAPSE_USBIN_BIT back on.
	 */
	if (chg->qc2_unsupported_voltage) {
		rc = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
				HVDCP_PULSE_COUNT_MAX_QC2_MASK,
				chg->qc2_max_pulses);
		if (rc < 0)
			smblib_err(chg, "Couldn't restore max pulses rc=%d\n",
					rc);

		rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
				SUSPEND_ON_COLLAPSE_USBIN_BIT,
				SUSPEND_ON_COLLAPSE_USBIN_BIT);
		if (rc < 0)
			smblib_err(chg, "Couldn't turn on SUSPEND_ON_COLLAPSE_USBIN_BIT rc=%d\n",
					rc);

		chg->qc2_unsupported_voltage = QC2_COMPLIANT;
	}
}

void smblib_suspend_on_debug_battery(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval val;

	rc = smblib_get_prop_from_bms(chg,
			POWER_SUPPLY_PROP_DEBUG_BATTERY, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get debug battery prop rc=%d\n", rc);
		return;
	}
	if (chg->suspend_input_on_debug_batt) {
		vote(chg->usb_icl_votable, DEBUG_BOARD_VOTER, val.intval, 0);
		vote(chg->dc_suspend_votable, DEBUG_BOARD_VOTER, val.intval, 0);
		if (val.intval)
			pr_info("Input suspended: Fake battery\n");
	} else {
		vote(chg->chg_disable_votable, DEBUG_BOARD_VOTER,
					val.intval, 0);
	}
}

int smblib_rerun_apsd_if_required(struct smb_charger *chg)
{
	union power_supply_propval val;
	int rc;

	rc = smblib_get_prop_usb_present(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb present rc = %d\n", rc);
		return rc;
	}

	if (!val.intval)
		return 0;

	rc = smblib_request_dpdm(chg, true);
	if (rc < 0)
		smblib_err(chg, "Couldn't to enable DPDM rc=%d\n", rc);

	chg->uusb_apsd_rerun_done = true;
	smblib_rerun_apsd(chg);

	return 0;
}

static int smblib_get_pulse_cnt(struct smb_charger *chg, int *count)
{
	*count = chg->pulse_cnt;
	return 0;
}

#define USBIN_25MA	25000
#define USBIN_100MA	100000
#define USBIN_150MA	150000
#define USBIN_500MA	500000
#define USBIN_900MA	900000
static int set_sdp_current(struct smb_charger *chg, int icl_ua)
{
	int rc;
	u8 icl_options;
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);

	/* power source is SDP */
	switch (icl_ua) {
	case USBIN_100MA:
		/* USB 2.0 100mA */
		icl_options = 0;
		break;
	case USBIN_150MA:
		/* USB 3.0 150mA */
		icl_options = CFG_USB3P0_SEL_BIT;
		break;
	case USBIN_500MA:
		/* USB 2.0 500mA */
		icl_options = USB51_MODE_BIT;
		break;
	case USBIN_900MA:
		/* USB 3.0 900mA */
		icl_options = CFG_USB3P0_SEL_BIT | USB51_MODE_BIT;
		break;
	default:
		return -EINVAL;
	}

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB &&
		apsd_result->pst == POWER_SUPPLY_TYPE_USB_FLOAT) {
		/*
		 * change the float charger configuration to SDP, if this
		 * is the case of SDP being detected as FLOAT
		 */
		rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
			FORCE_FLOAT_SDP_CFG_BIT, FORCE_FLOAT_SDP_CFG_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set float ICL options rc=%d\n",
						rc);
			return rc;
		}
	}

	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
			CFG_USB3P0_SEL_BIT | USB51_MODE_BIT, icl_options);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL options rc=%d\n", rc);
		return rc;
	}

	rc = smblib_icl_override(chg, SW_OVERRIDE_USB51_MODE);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL override rc=%d\n", rc);
		return rc;
	}

	return rc;
}

int smblib_set_icl_current(struct smb_charger *chg, int icl_ua)
{
	int rc = 0;
	enum icl_override_mode icl_override = HW_AUTO_MODE;
	/* suspend if 25mA or less is requested */
	bool suspend = (icl_ua <= USBIN_25MA);

	CHG_DBG("+++\n");

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_TYPEC) {
		rc = smblib_masked_write(chg, USB_CMD_PULLDOWN_REG,
				EN_PULLDOWN_USB_IN_BIT,
				suspend ? 0 : EN_PULLDOWN_USB_IN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't write %s to EN_PULLDOWN_USB_IN_BIT rc=%d\n",
				suspend ? "disable" : "enable", rc);
			goto out;
		}
	}

	if (suspend)
		return smblib_set_usb_suspend(chg, true);

	if (icl_ua == INT_MAX)
		goto set_mode;

	/* configure current */
	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB
		&& (chg->typec_legacy
		|| chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT
		|| chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)) {
		rc = set_sdp_current(chg, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set SDP ICL rc=%d\n", rc);
			goto out;
		}
	} else {
		/*
		 * Try USB 2.0/3,0 option first on USB path when maximum input
		 * current limit is 500mA or below for better accuracy; in case
		 * of error, proceed to use USB high-current mode.
		 */
		if (icl_ua <= USBIN_500MA) {
			rc = set_sdp_current(chg, icl_ua);
			if (rc >= 0)
				goto unsuspend;
		}

		rc = smblib_set_charge_param(chg, &chg->param.usb_icl, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set HC ICL rc=%d\n", rc);
			goto out;
		}
		icl_override = SW_OVERRIDE_HC_MODE;
	}

set_mode:
	CHG_DBG("icl(%d), asus_set_icl(%d), icl_override_mode(%d)\n", icl_ua, (int)asus_set_icl*50000, icl_override);

	rc = smblib_icl_override(chg, icl_override);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL override rc=%d\n", rc);
		goto out;
	}

unsuspend:
	/* unsuspend after configuring current and override */
	rc = smblib_set_usb_suspend(chg, false);
	if (rc < 0) {
		smblib_err(chg, "Couldn't resume input rc=%d\n", rc);
		goto out;
	}

	/* Re-run AICL */
	if (icl_override != SW_OVERRIDE_HC_MODE)
		rc = smblib_run_aicl(chg, RERUN_AICL);
out:
	CHG_DBG("---\n");
	return rc;
}

int smblib_get_icl_current(struct smb_charger *chg, int *icl_ua)
{
	int rc;

	rc = smblib_get_charge_param(chg, &chg->param.icl_max_stat, icl_ua);
	if (rc < 0)
		smblib_err(chg, "Couldn't get HC ICL rc=%d\n", rc);

	return rc;
}

int smblib_toggle_smb_en(struct smb_charger *chg, int toggle)
{
	int rc = 0;

	if (!toggle)
		return rc;

	rc = smblib_select_sec_charger(chg, chg->sec_chg_selected,
				chg->cp_reason, true);

	return rc;
}

int smblib_get_irq_status(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 reg;

	mutex_lock(&chg->irq_status_lock);
	/* Report and clear cached status */
	val->intval = chg->irq_status;
	chg->irq_status = 0;

	/* get real time status of pulse skip irq */
	rc = smblib_read(chg, MISC_PBS_RT_STS_REG, &reg);
	if (rc < 0)
		smblib_err(chg, "Couldn't read MISC_PBS_RT_STS_REG rc=%d\n",
				rc);
	else
		val->intval |= (reg & PULSE_SKIP_IRQ_BIT);
	mutex_unlock(&chg->irq_status_lock);

	return rc;
}

/****************************
 * uUSB Moisture Protection *
 ****************************/
#define MICRO_USB_DETECTION_ON_TIME_20_MS 0x08
#define MICRO_USB_DETECTION_PERIOD_X_100 0x03
#define U_USB_STATUS_WATER_PRESENT 0x00
static int smblib_set_moisture_protection(struct smb_charger *chg,
				bool enable)
{
	int rc = 0;

	if (chg->moisture_present == enable) {
		smblib_dbg(chg, PR_MISC, "No change in moisture protection status\n");
		return rc;
	}

	if (enable) {
		chg->moisture_present = true;

		/* Disable uUSB factory mode detection */
		rc = smblib_masked_write(chg, TYPEC_U_USB_CFG_REG,
					EN_MICRO_USB_FACTORY_MODE_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable uUSB factory mode detection rc=%d\n",
				rc);
			return rc;
		}

		/* Disable moisture detection and uUSB state change interrupt */
		rc = smblib_masked_write(chg, TYPE_C_INTERRUPT_EN_CFG_2_REG,
					TYPEC_WATER_DETECTION_INT_EN_BIT |
					MICRO_USB_STATE_CHANGE_INT_EN_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable moisture detection interrupt rc=%d\n",
			rc);
			return rc;
		}

		/* Set 1% duty cycle on ID detection */
		rc = smblib_masked_write(chg,
				((chg->smb_version == PMI632_SUBTYPE) ?
				PMI632_TYPEC_U_USB_WATER_PROTECTION_CFG_REG :
				TYPEC_U_USB_WATER_PROTECTION_CFG_REG),
				EN_MICRO_USB_WATER_PROTECTION_BIT |
				MICRO_USB_DETECTION_ON_TIME_CFG_MASK |
				MICRO_USB_DETECTION_PERIOD_CFG_MASK,
				EN_MICRO_USB_WATER_PROTECTION_BIT |
				MICRO_USB_DETECTION_ON_TIME_20_MS |
				MICRO_USB_DETECTION_PERIOD_X_100);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set 1 percent CC_ID duty cycle rc=%d\n",
				rc);
			return rc;
		}

		vote(chg->usb_icl_votable, MOISTURE_VOTER, true, 0);
	} else {
		chg->moisture_present = false;
		vote(chg->usb_icl_votable, MOISTURE_VOTER, false, 0);

		/* Enable moisture detection and uUSB state change interrupt */
		rc = smblib_masked_write(chg, TYPE_C_INTERRUPT_EN_CFG_2_REG,
					TYPEC_WATER_DETECTION_INT_EN_BIT |
					MICRO_USB_STATE_CHANGE_INT_EN_BIT,
					TYPEC_WATER_DETECTION_INT_EN_BIT |
					MICRO_USB_STATE_CHANGE_INT_EN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable moisture detection and uUSB state change interrupt rc=%d\n",
				rc);
			return rc;
		}

		/* Disable periodic monitoring of CC_ID pin */
		rc = smblib_write(chg, ((chg->smb_version == PMI632_SUBTYPE) ?
				PMI632_TYPEC_U_USB_WATER_PROTECTION_CFG_REG :
				TYPEC_U_USB_WATER_PROTECTION_CFG_REG), 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable 1 percent CC_ID duty cycle rc=%d\n",
				rc);
			return rc;
		}

		/* Enable uUSB factory mode detection */
		rc = smblib_masked_write(chg, TYPEC_U_USB_CFG_REG,
					EN_MICRO_USB_FACTORY_MODE_BIT,
					EN_MICRO_USB_FACTORY_MODE_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable uUSB factory mode detection rc=%d\n",
				rc);
			return rc;
		}
	}

	smblib_dbg(chg, PR_MISC, "Moisture protection %s\n",
			chg->moisture_present ? "enabled" : "disabled");
	return rc;
}

/*********************
 * VOTABLE CALLBACKS *
 *********************/
static int smblib_smb_disable_override_vote_callback(struct votable *votable,
			void *data, int disable_smb, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;

	/* Enable/disable SMB_EN pin */
	rc = smblib_masked_write(chg, MISC_SMB_EN_CMD_REG,
			SMB_EN_OVERRIDE_BIT | SMB_EN_OVERRIDE_VALUE_BIT,
			disable_smb ? SMB_EN_OVERRIDE_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't configure SMB_EN, rc=%d\n", rc);

	return rc;
}

static int smblib_dc_suspend_vote_callback(struct votable *votable, void *data,
			int suspend, const char *client)
{
	struct smb_charger *chg = data;

	if (chg->smb_version == PMI632_SUBTYPE)
		return 0;

	/* resume input if suspend is invalid */
	if (suspend < 0)
		suspend = 0;

	return smblib_set_dc_suspend(chg, (bool)suspend);
}

static int smblib_awake_vote_callback(struct votable *votable, void *data,
			int awake, const char *client)
{
	struct smb_charger *chg = data;

	if (awake)
		pm_stay_awake(chg->dev);
	else
		pm_relax(chg->dev);

	return 0;
}

static int smblib_chg_disable_vote_callback(struct votable *votable, void *data,
			int chg_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				 CHARGING_ENABLE_CMD_BIT,
				 chg_disable ? 0 : CHARGING_ENABLE_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s charging rc=%d\n",
			chg_disable ? "disable" : "enable", rc);
		return rc;
	}

	return 0;
}

static int smblib_hdc_irq_disable_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq)
		return 0;

	if (chg->irq_info[HIGH_DUTY_CYCLE_IRQ].enabled) {
		if (disable)
			disable_irq_nosync(
				chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq);
	} else {
		if (!disable)
			enable_irq(chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq);
	}

	chg->irq_info[HIGH_DUTY_CYCLE_IRQ].enabled = !disable;

	return 0;
}

static int smblib_limited_irq_disable_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].irq)
		return 0;

	if (chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].enabled) {
		if (disable)
			disable_irq_nosync(
				chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].irq);
	} else {
		if (!disable)
			enable_irq(
				chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].irq);
	}

	chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].enabled = !disable;

	return 0;
}

static int smblib_icl_irq_disable_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq)
		return 0;

	if (chg->irq_info[USBIN_ICL_CHANGE_IRQ].enabled) {
		if (disable)
			disable_irq_nosync(
				chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);
	} else {
		if (!disable)
			enable_irq(chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);
	}

	chg->irq_info[USBIN_ICL_CHANGE_IRQ].enabled = !disable;

	return 0;
}

/*******************
 * VCONN REGULATOR *
 * *****************/

int smblib_vconn_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 stat, orientation;

	smblib_dbg(chg, PR_OTG, "enabling VCONN\n");

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}

	/* VCONN orientation is opposite to that of CC */
	orientation =
		stat & TYPEC_CCOUT_VALUE_BIT ? 0 : VCONN_EN_ORIENTATION_BIT;
	rc = smblib_masked_write(chg, TYPE_C_VCONN_CONTROL_REG,
				VCONN_EN_VALUE_BIT | VCONN_EN_ORIENTATION_BIT,
				VCONN_EN_VALUE_BIT | orientation);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_CCOUT_CONTROL_REG rc=%d\n",
			rc);
		return rc;
	}

	return 0;
}

int smblib_vconn_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	smblib_dbg(chg, PR_OTG, "disabling VCONN\n");
	rc = smblib_masked_write(chg, TYPE_C_VCONN_CONTROL_REG,
				 VCONN_EN_VALUE_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't disable vconn regulator rc=%d\n", rc);

	return 0;
}

int smblib_vconn_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;
	u8 cmd;

	rc = smblib_read(chg, TYPE_C_VCONN_CONTROL_REG, &cmd);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			rc);
		return rc;
	}

	return (cmd & VCONN_EN_VALUE_BIT) ? 1 : 0;
}

/*****************
 * OTG REGULATOR *
 *****************/
extern int chg_set_src_cap(void);
extern int cam_sensor_is_power_up(void);
extern int cam_flash_battery_low(int enable);
int smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;
	int batt_current;
	int batt_capacity;
	
	smblib_dbg(chg, PR_OTG, "enabling OTG\n");
	CHG_DBG("enabling OTG\n");
	
	batt_current = asus_get_prop_batt_current(smbchg_dev);
	batt_capacity = asus_get_prop_batt_capacity(smbchg_dev);
	
	if (batt_current <= 2500000 && batt_capacity > 30 && !cam_sensor_is_power_up()) {
		//0x1152 = 0x03, OTG_ILIMIT_2000MA
		rc = smblib_masked_write(smbchg_dev, DCDC_OTG_CURRENT_LIMIT_CFG_REG,
			DCDC_OTG_CURRENT_LIMIT_MASK, 0x03);
		if (rc < 0) {
			CHG_DBG_E("Couldn't set default DCDC_OTG_CURRENT_LIMIT_CFG_REG rc=%d\n", rc);
		}
	
		rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG, OTG_EN_BIT, OTG_EN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable OTG rc=%d\n", rc);
			return rc;
		}
	
		CHG_DBG("set otg current limit to 2A\n");
		schedule_delayed_work(&smbchg_dev->asus_reverse_charge_work, 0);
	}
	else {
		//0x1152 = 0x01, OTG_ILIMIT_1000MA
		rc = smblib_masked_write(smbchg_dev, DCDC_OTG_CURRENT_LIMIT_CFG_REG,
			DCDC_OTG_CURRENT_LIMIT_MASK, 0x01);
		if (rc < 0) {
			CHG_DBG_E("Couldn't set default DCDC_OTG_CURRENT_LIMIT_CFG_REG rc=%d\n", rc);
		}
		
		rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG, OTG_EN_BIT, OTG_EN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable OTG rc=%d\n", rc);
			return rc;
		}
	
		default_src_caps[0] = 0x36019032;
		chg_set_src_cap();
		
		CHG_DBG("batt_current = %d, batt_capacity = %d, set otg current limit to 1A\n", batt_current, batt_capacity);
	}
	
	if(batt_capacity <= 15){
		cam_flash_battery_low(1);
		asus_extcon_set_state_sync(smbchg_dev->reversechg_extcon, 1);
	}
	else{
		cam_flash_battery_low(0);
		asus_extcon_set_state_sync(smbchg_dev->reversechg_extcon, 0);
		schedule_delayed_work(&smbchg_dev->asus_reverse_charge_check_camera, 0);
	}
	
	g_usb_otg = 1;
	
	return 0;
}

int smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;

	smblib_dbg(chg, PR_OTG, "disabling OTG\n");
	CHG_DBG("disabling OTG\n");

	g_usb_otg = 0;
	cancel_delayed_work(&smbchg_dev->asus_reverse_charge_work);
	cancel_delayed_work(&smbchg_dev->asus_reverse_charge_check_camera);

	rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG, OTG_EN_BIT, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable OTG regulator rc=%d\n", rc);
		return rc;
	}
	
	//0x1152 = 0x01, OTG_ILIMIT_1000MA
	rc = smblib_masked_write(smbchg_dev, DCDC_OTG_CURRENT_LIMIT_CFG_REG,
		DCDC_OTG_CURRENT_LIMIT_MASK, 0x01);
	if (rc < 0) {
		CHG_DBG_E("Couldn't set default DCDC_OTG_CURRENT_LIMIT_CFG_REG rc=%d\n", rc);
	}

	default_src_caps[0] = 0x360190c8;

	cam_flash_battery_low(0);
	asus_extcon_set_state_sync(smbchg_dev->reversechg_extcon, 0);
	
	return 0;
}

int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 cmd;

	rc = smblib_read(chg, DCDC_CMD_OTG_REG, &cmd);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CMD_OTG rc=%d", rc);
		return rc;
	}

	return (cmd & OTG_EN_BIT) ? 1 : 0;
}

/********************
 * BATT PSY GETTERS *
 ********************/

int asus_get_batt_status (void)
{
	if(smbchg_dev->pd_active){
		if(final_pd_mw > 10000000)
			return QC_PLUS;
		else if(final_pd_mw == 10000000)
			return QC;
		else
			return NORMAL;
	} 
	else if (HVDCP_FLAG == 3)
		return QC_PLUS;
	else if (ASUS_ADAPTER_ID == OTHERS && UFP_FLAG == 3 && LEGACY_CABLE_FLAG == 0)
		return QC;
	else if (HVDCP_FLAG == 0 && ASUS_ADAPTER_ID == PB && UFP_FLAG == 3 && LEGACY_CABLE_FLAG == 0)
		return QC;
	else if (HVDCP_FLAG == 0 && (ASUS_ADAPTER_ID == ASUS_750K || ASUS_ADAPTER_ID == PB) && (LEGACY_CABLE_FLAG || UFP_FLAG == 1))
		return QC;
	else
		return NORMAL;
}

int smblib_get_prop_input_suspend(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	val->intval
		= (get_client_vote(chg->usb_icl_votable, USER_VOTER) == 0)
		 && get_client_vote(chg->dc_suspend_votable, USER_VOTER);
	return 0;
}

int smblib_get_prop_batt_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATIF_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATIF_INT_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = !(stat & (BAT_THERM_OR_ID_MISSING_RT_STS_BIT
					| BAT_TERMINAL_MISSING_RT_STS_BIT));

	return rc;
}

int smblib_get_prop_batt_capacity(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;

	if (chg->fake_capacity >= 0) {
		val->intval = chg->fake_capacity;
		return 0;
	}

	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CAPACITY, val);

	return rc;
}

static bool is_charging_paused(struct smb_charger *chg)
{
	int rc;
	u8 val;

	rc = smblib_read(chg, CHARGING_PAUSE_CMD_REG, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CHARGING_PAUSE_CMD rc=%d\n", rc);
		return false;
	}

	return val & CHARGING_PAUSE_CMD_BIT;
}

int smblib_get_prop_batt_status(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	bool usb_online, dc_online;
	u8 stat;
	int rc, suspend = 0;
	static int now_bat_status = POWER_SUPPLY_STATUS_DISCHARGING, pre_bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
	static int now_chg_status = 7, pre_chg_status = 7;
	pre_bat_status = now_bat_status;
	pre_chg_status = now_chg_status;

	if (usb_thermal_once_flag && g_Charger_mode) {
		val->intval = POWER_SUPPLY_STATUS_THERMAL_ALERT;
		return 0;
	}

	if (chg->dbc_usbov) {
		rc = smblib_get_prop_usb_present(chg, &pval);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't get usb present prop rc=%d\n", rc);
			return rc;
		}

		rc = smblib_get_usb_suspend(chg, &suspend);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't get usb suspend rc=%d\n", rc);
			return rc;
		}

		/*
		 * Report charging as long as USBOV is not debounced and
		 * charging path is un-suspended.
		 */
		if (pval.intval && !suspend) {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			return 0;
		}
	}

	rc = smblib_get_prop_usb_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb online property rc=%d\n",
			rc);
		return rc;
	}
	usb_online = (bool)pval.intval;

	rc = smblib_get_prop_dc_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get dc online property rc=%d\n",
			rc);
		return rc;
	}
	dc_online = (bool)pval.intval;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (!usb_online && !dc_online) {
		switch (stat) {
		case TERMINATE_CHARGE:
		case INHIBIT_CHARGE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}
		return rc;
	}

	switch (stat) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
	case FULLON_CHARGE:
	case TAPER_CHARGE:
		if (g_fgChip != NULL && smbchg_dev != NULL) {
			if (asus_get_prop_batt_capacity(smbchg_dev) == 100) {
				val->intval = POWER_SUPPLY_STATUS_FULL;
			} else {
				if (g_Charger_mode) {
					if (asus_get_batt_status() == QC_PLUS) {
						val->intval = POWER_SUPPLY_STATUS_QUICK_CHARGING_PLUS;
					} else if (asus_get_batt_status() == QC) {
						val->intval = POWER_SUPPLY_STATUS_QUICK_CHARGING;
					} else {
						val->intval = POWER_SUPPLY_STATUS_CHARGING;
					}
				} else {
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
				}
			}
		} else {
			if (g_Charger_mode) {
				if (asus_get_batt_status() == QC_PLUS) {
					val->intval = POWER_SUPPLY_STATUS_QUICK_CHARGING_PLUS;
				} else if (asus_get_batt_status() == QC) {
					val->intval = POWER_SUPPLY_STATUS_QUICK_CHARGING;
				} else {
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
				}
			} else {
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			}
		}
		break;
	case TERMINATE_CHARGE:
	case INHIBIT_CHARGE:
		if (asus_get_prop_batt_capacity(smbchg_dev) != 100) {
			if (g_Charger_mode) {
				if (asus_get_batt_status() == QC_PLUS) {
					val->intval = POWER_SUPPLY_STATUS_QUICK_CHARGING_PLUS;
				} else if (asus_get_batt_status() == QC) {
					val->intval = POWER_SUPPLY_STATUS_QUICK_CHARGING;
				} else {
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
				}
			} else {
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			}
		} else {
			val->intval = POWER_SUPPLY_STATUS_FULL;
		}
		break;
	case DISABLE_CHARGE:
	case PAUSE_CHARGE:
		if (g_fgChip != NULL && smbchg_dev != NULL) {
			if (asus_get_prop_batt_capacity(smbchg_dev) == 100) {
				val->intval = POWER_SUPPLY_STATUS_FULL;
			} else {
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
		} else {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		break;
	default:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	now_bat_status = val->intval;
	now_chg_status = stat;

	if (is_charging_paused(chg)) {
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	/*
	 * If charge termination WA is active and has suspended charging, then
	 * continue reporting charging status as FULL.
	 */
	if (is_client_vote_enabled(chg->usb_icl_votable,
						CHG_TERMINATION_VOTER)) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	if ((val->intval != POWER_SUPPLY_STATUS_CHARGING) || (val->intval != POWER_SUPPLY_STATUS_QUICK_CHARGING) 
		|| (val->intval != POWER_SUPPLY_STATUS_QUICK_CHARGING_PLUS))
		return 0;

	if (!usb_online && dc_online
		&& chg->fake_batt_status == POWER_SUPPLY_STATUS_FULL) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_5_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
				rc);
			return rc;
	}

	stat &= ENABLE_TRICKLE_BIT | ENABLE_PRE_CHARGING_BIT |
						ENABLE_FULLON_MODE_BIT;

	if (!stat)
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

int smblib_get_prop_batt_charge_type(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	switch (stat & BATTERY_CHARGER_STATUS_MASK) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case FULLON_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TAPER;
		break;
	default:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	return rc;
}

int smblib_get_prop_batt_health(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval;
	int rc;
	int effective_fv_uv;
	u8 stat;
	//u8 chg_stat,vbus_stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "BATTERY_CHARGER_STATUS_2 = 0x%02x\n",
		   stat);

	if (stat & CHARGER_ERROR_STATUS_BAT_OV_BIT) {
		rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
		if (!rc) {
			/*
			 * If Vbatt is within 40mV above Vfloat, then don't
			 * treat it as overvoltage.
			 */
			effective_fv_uv = get_effective_result(chg->fv_votable);
			if (pval.intval >= effective_fv_uv + 40000) {
				val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
				smblib_err(chg, "battery over-voltage vbat_fg = %duV, fv = %duV\n",
						pval.intval, effective_fv_uv);
				goto done;
			}
		}
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_7_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}
	if (stat & BAT_TEMP_STATUS_TOO_COLD_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & BAT_TEMP_STATUS_TOO_HOT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & BAT_TEMP_STATUS_COLD_SOFT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & BAT_TEMP_STATUS_HOT_SOFT_BIT) {
		val->intval = POWER_SUPPLY_HEALTH_WARM;
/*
		//WeiYu ++ modify for jeita icon issue
		rc = smblib_read(smbchg_dev, USBIN_BASE + INT_RT_STS_OFFSET, &vbus_stat);	
		rc = smblib_read(smbchg_dev, CHARGING_ENABLE_CMD_REG, &chg_stat);	
		if((bool)(vbus_stat & USBIN_PLUGIN_RT_STS_BIT)){
			if((chg_stat&CHARGING_ENABLE_CMD_BIT) == 1){
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
				//CHG_DBG_EVT("Warm-temp but reporting health good due to charging is enabled\n");
			}
			else{
				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			}
		}
*/
	}
	else
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

done:
	return rc;
}

int smblib_get_prop_system_temp_level(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->system_temp_level;
	return 0;
}

int smblib_get_prop_system_temp_level_max(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_levels;
	return 0;
}

int smblib_get_prop_input_current_limited(struct smb_charger *chg,
				union power_supply_propval *val)
{
	u8 stat;
	int rc;

	if (chg->fake_input_current_limited >= 0) {
		val->intval = chg->fake_input_current_limited;
		return 0;
	}

	rc = smblib_read(chg, AICL_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n", rc);
		return rc;
	}
	val->intval = (stat & SOFT_ILIMIT_BIT) || chg->is_hdc;
	return 0;
}

int smblib_get_prop_batt_iterm(struct smb_charger *chg,
		union power_supply_propval *val)
{
	int rc, temp;
	u8 stat, buf[2];

	/*
	 * Currently, only ADC comparator-based termination is supported,
	 * hence read only the threshold corresponding to ADC source.
	 * Proceed only if CHGR_ITERM_USE_ANALOG_BIT is 0.
	 */
	rc = smblib_read(chg, CHGR_ENG_CHARGING_CFG_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CHGR_ENG_CHARGING_CFG_REG rc=%d\n",
				rc);
		return rc;
	}

	if (stat & CHGR_ITERM_USE_ANALOG_BIT) {
		val->intval = -EINVAL;
		return 0;
	}

	rc = smblib_batch_read(chg, CHGR_ADC_ITERM_UP_THD_MSB_REG, buf, 2);

	if (rc < 0) {
		smblib_err(chg, "Couldn't read CHGR_ADC_ITERM_UP_THD_MSB_REG rc=%d\n",
				rc);
		return rc;
	}

	temp = buf[1] | (buf[0] << 8);
	temp = sign_extend32(temp, 15);

	if (chg->smb_version == PMI632_SUBTYPE)
		temp = DIV_ROUND_CLOSEST(temp * ITERM_LIMITS_PMI632_MA,
					ADC_CHG_ITERM_MASK);
	else
		temp = DIV_ROUND_CLOSEST(temp * ITERM_LIMITS_PM8150B_MA,
					ADC_CHG_ITERM_MASK);

	val->intval = temp;

	return rc;
}

//[+++]ASUS : Add to get the batt_id
int smblib_get_prop_batt_id(struct smb_charger *chg,
			      union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_RESISTANCE_ID, val);
	return rc;
}
//[---]ASUS : Add to get the batt_id

int smblib_get_prop_batt_charge_done(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	val->intval = (stat == TERMINATE_CHARGE);
	return 0;
}

/***********************
 * BATTERY PSY SETTERS *
 ***********************/

int smblib_set_prop_input_suspend(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc;

	/* vote 0mA when suspended */
	rc = vote(chg->usb_icl_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s USB rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s DC rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	power_supply_changed(chg->batt_psy);
	return rc;
}

int smblib_set_prop_batt_capacity(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	chg->fake_capacity = val->intval;

	power_supply_changed(chg->batt_psy);

	return 0;
}

int smblib_set_prop_batt_status(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	/* Faking battery full */
	if (val->intval == POWER_SUPPLY_STATUS_FULL)
		chg->fake_batt_status = val->intval;
	else
		chg->fake_batt_status = -EINVAL;

	power_supply_changed(chg->batt_psy);

	return 0;
}

int smblib_set_prop_system_temp_level(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	CHG_DBG("start, level = %d, use asus settings return this function", val->intval);
	return 0;

	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels)
		return -EINVAL;

	chg->system_temp_level = val->intval;

	if (chg->system_temp_level == chg->thermal_levels)
		return vote(chg->chg_disable_votable,
			THERMAL_DAEMON_VOTER, true, 0);

	vote(chg->chg_disable_votable, THERMAL_DAEMON_VOTER, false, 0);
	if (chg->system_temp_level == 0)
		return vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, false, 0);

	vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, true,
			chg->thermal_mitigation[chg->system_temp_level]);
	return 0;
}

int smblib_set_prop_input_current_limited(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	chg->fake_input_current_limited = val->intval;
	return 0;
}

int smblib_set_prop_rechg_soc_thresh(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;
	u8 new_thr = DIV_ROUND_CLOSEST(val->intval * 255, 100);

	rc = smblib_write(chg, CHARGE_RCHG_SOC_THRESHOLD_CFG_REG,
			new_thr);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write to RCHG_SOC_THRESHOLD_CFG_REG rc=%d\n",
				rc);
		return rc;
	}

	chg->auto_recharge_soc = val->intval;

	return rc;
}

int smblib_run_aicl(struct smb_charger *chg, int type)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
								rc);
		return rc;
	}

	/* USB is suspended so skip re-running AICL */
	if (stat & USBIN_SUSPEND_STS_BIT)
		return rc;

	smblib_dbg(chg, PR_MISC, "re-running AICL\n");

	stat = (type == RERUN_AICL) ? RERUN_AICL_BIT : RESTART_AICL_BIT;
	rc = smblib_masked_write(chg, AICL_CMD_REG, stat, stat);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to AICL_CMD_REG rc=%d\n",
				rc);
	return 0;
}

static int smblib_dp_pulse(struct smb_charger *chg)
{
	int rc;

	/* QC 3.0 increment */
	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, SINGLE_INCREMENT_BIT,
			SINGLE_INCREMENT_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

static int smblib_dm_pulse(struct smb_charger *chg)
{
	int rc;

	/* QC 3.0 decrement */
	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, SINGLE_DECREMENT_BIT,
			SINGLE_DECREMENT_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

int smblib_force_vbus_voltage(struct smb_charger *chg, u8 val)
{
	int rc;

	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, val, val);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

static void smblib_hvdcp_set_fsw(struct smb_charger *chg, int bit)
{
	switch (bit) {
	case QC_5V_BIT:
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_5V);
		break;
	case QC_9V_BIT:
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_9V);
		break;
	case QC_12V_BIT:
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_12V);
		break;
	default:
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_removal);
		break;
	}
}

#define QC3_PULSES_FOR_6V	5
#define QC3_PULSES_FOR_9V	20
#define QC3_PULSES_FOR_12V	35
static int smblib_hvdcp3_set_fsw(struct smb_charger *chg)
{
	int pulse_count, rc;

	rc = smblib_get_pulse_cnt(chg, &pulse_count);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
		return rc;
	}

	if (pulse_count < QC3_PULSES_FOR_6V)
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_5V);
	else if (pulse_count < QC3_PULSES_FOR_9V)
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_6V_8V);
	else if (pulse_count < QC3_PULSES_FOR_12V)
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_9V);
	else
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_12V);

	return 0;
}

static void smblib_hvdcp_adaptive_voltage_change(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP) {
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_CHANGE_STATUS rc=%d\n", rc);
			return;
		}

		smblib_hvdcp_set_fsw(chg, stat & QC_2P0_STATUS_MASK);
		vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER, false, 0);
	}

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3) {
		rc = smblib_hvdcp3_set_fsw(chg);
		if (rc < 0)
			smblib_err(chg, "Couldn't set QC3.0 Fsw rc=%d\n", rc);
	}

	power_supply_changed(chg->usb_main_psy);
}

int smblib_dp_dm(struct smb_charger *chg, int val)
{
	int target_icl_ua, rc = 0;
	union power_supply_propval pval;
	u8 stat;

	switch (val) {
	case POWER_SUPPLY_DP_DM_DP_PULSE:
		/*
		 * Pre-emptively increment pulse count to enable the setting
		 * of FSW prior to increasing voltage.
		 */
		chg->pulse_cnt++;

		rc = smblib_hvdcp3_set_fsw(chg);
		if (rc < 0)
			smblib_err(chg, "Couldn't set QC3.0 Fsw rc=%d\n", rc);

		rc = smblib_dp_pulse(chg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't increase pulse count rc=%d\n",
				rc);
			/*
			 * Increment pulse count failed;
			 * reset to former value.
			 */
			chg->pulse_cnt--;
		}

		smblib_dbg(chg, PR_PARALLEL, "DP_DM_DP_PULSE rc=%d cnt=%d\n",
				rc, chg->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_DM_PULSE:
		rc = smblib_dm_pulse(chg);
		if (!rc && chg->pulse_cnt)
			chg->pulse_cnt--;
		smblib_dbg(chg, PR_PARALLEL, "DP_DM_DM_PULSE rc=%d cnt=%d\n",
				rc, chg->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_ICL_DOWN:
		target_icl_ua = get_effective_result(chg->usb_icl_votable);
		if (target_icl_ua < 0) {
			/* no client vote, get the ICL from charger */
			rc = power_supply_get_property(chg->usb_psy,
					POWER_SUPPLY_PROP_HW_CURRENT_MAX,
					&pval);
			if (rc < 0) {
				smblib_err(chg, "Couldn't get max curr rc=%d\n",
					rc);
				return rc;
			}
			target_icl_ua = pval.intval;
		}

		/*
		 * Check if any other voter voted on USB_ICL in case of
		 * voter other than SW_QC3_VOTER reset and restart reduction
		 * again.
		 */
		if (target_icl_ua != get_client_vote(chg->usb_icl_votable,
							SW_QC3_VOTER))
			chg->usb_icl_delta_ua = 0;

		chg->usb_icl_delta_ua += 100000;
		vote(chg->usb_icl_votable, SW_QC3_VOTER, true,
						target_icl_ua - 100000);
		smblib_dbg(chg, PR_PARALLEL, "ICL DOWN ICL=%d reduction=%d\n",
				target_icl_ua, chg->usb_icl_delta_ua);
		break;
	case POWER_SUPPLY_DP_DM_FORCE_5V:
		smblib_dbg(chg, PR_MISC, "[DBG] Force 5V, qc2_unsupported_voltage(%d)\n", chg->qc2_unsupported_voltage);
		rc = smblib_force_vbus_voltage(chg, FORCE_5V_BIT);
		if (rc < 0)
			pr_err("Failed to force 5V\n");
		break;
	case POWER_SUPPLY_DP_DM_FORCE_9V:
		smblib_dbg(chg, PR_MISC, "[DBG] Force 9V, qc2_unsupported_voltage(%d)\n", chg->qc2_unsupported_voltage);
		if (chg->qc2_unsupported_voltage == QC2_NON_COMPLIANT_9V) {
			smblib_err(chg, "Couldn't set 9V: unsupported\n");
			return -EINVAL;
		}

		/* If we are increasing voltage to get to 9V, set FSW first */
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read QC_CHANGE_STATUS_REG rc=%d\n",
					rc);
			break;
		}

		if (stat & QC_5V_BIT) {
			/* Force 1A ICL before requesting higher voltage */
			vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER,
					true, 1000000);
			smblib_hvdcp_set_fsw(chg, QC_9V_BIT);
		}

		rc = smblib_force_vbus_voltage(chg, FORCE_9V_BIT);
		if (rc < 0)
			pr_err("Failed to force 9V\n");
		break;
	case POWER_SUPPLY_DP_DM_FORCE_12V:
		smblib_dbg(chg, PR_MISC, "[DBG] Force 12V, qc2_unsupported_voltage(%d)\n", chg->qc2_unsupported_voltage);
		if (chg->qc2_unsupported_voltage == QC2_NON_COMPLIANT_12V) {
			smblib_err(chg, "Couldn't set 12V: unsupported\n");
			return -EINVAL;
		}

		/* If we are increasing voltage to get to 12V, set FSW first */
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read QC_CHANGE_STATUS_REG rc=%d\n",
					rc);
			break;
		}

		if ((stat & QC_9V_BIT) || (stat & QC_5V_BIT)) {
			/* Force 1A ICL before requesting higher voltage */
			vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER,
					true, 1000000);
			smblib_hvdcp_set_fsw(chg, QC_12V_BIT);
		}

		rc = smblib_force_vbus_voltage(chg, FORCE_12V_BIT);
		if (rc < 0)
			pr_err("Failed to force 12V\n");
		break;
	case POWER_SUPPLY_DP_DM_ICL_UP:
	default:
		break;
	}

	return rc;
}

int smblib_disable_hw_jeita(struct smb_charger *chg, bool disable)
{
	int rc;
	u8 mask;

	/*
	 * Disable h/w base JEITA compensation if s/w JEITA is enabled
	 */
	mask = JEITA_EN_COLD_SL_FCV_BIT
		| JEITA_EN_HOT_SL_FCV_BIT
		| JEITA_EN_HOT_SL_CCC_BIT
		| JEITA_EN_COLD_SL_CCC_BIT,
	rc = smblib_masked_write(chg, JEITA_EN_CFG_REG, mask,
			disable ? 0 : mask);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure s/w jeita rc=%d\n",
				rc);
		return rc;
	}

	return 0;
}

static int smblib_set_sw_thermal_regulation(struct smb_charger *chg,
						bool enable)
{
	int rc = 0;

	if (!(chg->wa_flags & SW_THERM_REGULATION_WA))
		return rc;

	if (enable) {
		/*
		 * Configure min time to quickly address thermal
		 * condition.
		 */
		rc = smblib_masked_write(chg, SNARL_BARK_BITE_WD_CFG_REG,
			SNARL_WDOG_TIMEOUT_MASK, SNARL_WDOG_TMOUT_62P5MS);
		if (rc < 0) {
			smblib_err(chg, "Couldn't configure snarl wdog tmout, rc=%d\n",
					rc);
			return rc;
		}

		/*
		 * Schedule SW_THERM_REGULATION_WORK directly if USB input
		 * is suspended due to SW thermal regulation WA since WDOG
		 * IRQ won't trigger with input suspended.
		 */
		if (is_client_vote_enabled(chg->usb_icl_votable,
						SW_THERM_REGULATION_VOTER)) {
			vote(chg->awake_votable, SW_THERM_REGULATION_VOTER,
								true, 0);
			schedule_delayed_work(&chg->thermal_regulation_work, 0);
		}
	} else {
		cancel_delayed_work_sync(&chg->thermal_regulation_work);
		vote(chg->awake_votable, SW_THERM_REGULATION_VOTER, false, 0);
	}

	smblib_dbg(chg, PR_MISC, "WDOG SNARL INT %s\n",
				enable ? "Enabled" : "Disabled");

	return rc;
}

static int smblib_update_thermal_readings(struct smb_charger *chg)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	if (!chg->pl.psy)
		chg->pl.psy = power_supply_get_by_name("parallel");

	rc = smblib_read_iio_channel(chg, chg->iio.die_temp_chan,
				DIV_FACTOR_DECIDEGC, &chg->die_temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DIE TEMP channel, rc=%d\n", rc);
		return rc;
	}

	rc = smblib_read_iio_channel(chg, chg->iio.connector_temp_chan,
				DIV_FACTOR_DECIDEGC, &chg->connector_temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CONN TEMP channel, rc=%d\n", rc);
		return rc;
	}

	rc = smblib_read_iio_channel(chg, chg->iio.skin_temp_chan,
				DIV_FACTOR_DECIDEGC, &chg->skin_temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SKIN TEMP channel, rc=%d\n", rc);
		return rc;
	}

	if (chg->sec_chg_selected == POWER_SUPPLY_CHARGER_SEC_CP) {
		if (!chg->cp_psy)
			chg->cp_psy =
				power_supply_get_by_name("charge_pump_master");
		if (chg->cp_psy) {
			rc = power_supply_get_property(chg->cp_psy,
				POWER_SUPPLY_PROP_CP_DIE_TEMP, &pval);
			if (rc < 0) {
				smblib_err(chg, "Couldn't get smb1390 charger temp, rc=%d\n",
					rc);
				return rc;
			}
			chg->smb_temp = pval.intval;
		} else {
			smblib_dbg(chg, PR_MISC, "Coudln't find cp_psy\n");
			chg->smb_temp = -ENODATA;
		}
	} else if (chg->pl.psy && chg->sec_chg_selected ==
					POWER_SUPPLY_CHARGER_SEC_PL) {
		rc = power_supply_get_property(chg->pl.psy,
				POWER_SUPPLY_PROP_CHARGER_TEMP, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get smb1355 charger temp, rc=%d\n",
					rc);
			return rc;
		}
		chg->smb_temp = pval.intval;
	} else {
		chg->smb_temp = -ENODATA;
	}

	return rc;
}

/* SW thermal regulation thresholds in deciDegC */
#define DIE_TEMP_RST_THRESH		1000
#define DIE_TEMP_REG_H_THRESH		800
#define DIE_TEMP_REG_L_THRESH		600

#define CONNECTOR_TEMP_SHDN_THRESH	700
#define CONNECTOR_TEMP_RST_THRESH	600
#define CONNECTOR_TEMP_REG_H_THRESH	550
#define CONNECTOR_TEMP_REG_L_THRESH	500

#define SMB_TEMP_SHDN_THRESH		1400
#define SMB_TEMP_RST_THRESH		900
#define SMB_TEMP_REG_H_THRESH		800
#define SMB_TEMP_REG_L_THRESH		600

#define SKIN_TEMP_SHDN_THRESH		700
#define SKIN_TEMP_RST_THRESH		600
#define SKIN_TEMP_REG_H_THRESH		550
#define SKIN_TEMP_REG_L_THRESH		500

#define THERM_REG_RECHECK_DELAY_1S	1000	/* 1 sec */
#define THERM_REG_RECHECK_DELAY_8S	8000	/* 8 sec */
static int smblib_process_thermal_readings(struct smb_charger *chg)
{
	int rc = 0, wdog_timeout = SNARL_WDOG_TMOUT_8S;
	u32 thermal_status = TEMP_BELOW_RANGE;
	bool suspend_input = false, disable_smb = false;

	/*
	 * Following is the SW thermal regulation flow:
	 *
	 * TEMP_SHUT_DOWN_LEVEL: If either connector temp or skin temp
	 * exceeds their respective SHDN threshold. Need to suspend input
	 * and secondary charger.
	 *
	 * TEMP_SHUT_DOWN_SMB_LEVEL: If smb temp exceed its SHDN threshold
	 * but connector and skin temp are below it. Need to suspend SMB.
	 *
	 * TEMP_ALERT_LEVEL: If die, connector, smb or skin temp exceeds it's
	 * respective RST threshold. Stay put and monitor temperature closely.
	 *
	 * TEMP_ABOVE_RANGE or TEMP_WITHIN_RANGE or TEMP_BELOW_RANGE: If die,
	 * connector, smb or skin temp exceeds it's respective REG_H or REG_L
	 * threshold. Unsuspend input and SMB.
	 */
	if (chg->connector_temp > CONNECTOR_TEMP_SHDN_THRESH ||
		chg->skin_temp > SKIN_TEMP_SHDN_THRESH) {
		thermal_status = TEMP_SHUT_DOWN;
		wdog_timeout = SNARL_WDOG_TMOUT_1S;
		suspend_input = true;
		disable_smb = true;
		goto out;
	}

	if (chg->smb_temp > SMB_TEMP_SHDN_THRESH) {
		thermal_status = TEMP_SHUT_DOWN_SMB;
		wdog_timeout = SNARL_WDOG_TMOUT_1S;
		disable_smb = true;
		goto out;
	}

	if (chg->connector_temp > CONNECTOR_TEMP_RST_THRESH ||
			chg->skin_temp > SKIN_TEMP_RST_THRESH ||
			chg->smb_temp > SMB_TEMP_RST_THRESH ||
			chg->die_temp > DIE_TEMP_RST_THRESH) {
		thermal_status = TEMP_ALERT_LEVEL;
		wdog_timeout = SNARL_WDOG_TMOUT_1S;
		goto out;
	}

	if (chg->connector_temp > CONNECTOR_TEMP_REG_H_THRESH ||
			chg->skin_temp > SKIN_TEMP_REG_H_THRESH ||
			chg->smb_temp > SMB_TEMP_REG_H_THRESH ||
			chg->die_temp > DIE_TEMP_REG_H_THRESH) {
		thermal_status = TEMP_ABOVE_RANGE;
		wdog_timeout = SNARL_WDOG_TMOUT_1S;
		goto out;
	}

	if (chg->connector_temp > CONNECTOR_TEMP_REG_L_THRESH ||
			chg->skin_temp > SKIN_TEMP_REG_L_THRESH ||
			chg->smb_temp > SMB_TEMP_REG_L_THRESH ||
			chg->die_temp > DIE_TEMP_REG_L_THRESH) {
		thermal_status = TEMP_WITHIN_RANGE;
		wdog_timeout = SNARL_WDOG_TMOUT_8S;
	}
out:
	smblib_dbg(chg, PR_MISC, "Current temperatures: \tDIE_TEMP: %d,\tCONN_TEMP: %d,\tSMB_TEMP: %d,\tSKIN_TEMP: %d\nTHERMAL_STATUS: %d\n",
			chg->die_temp, chg->connector_temp, chg->smb_temp,
			chg->skin_temp, thermal_status);

	if (thermal_status != chg->thermal_status) {
		chg->thermal_status = thermal_status;
		/*
		 * If thermal level changes to TEMP ALERT LEVEL, don't
		 * enable/disable main/parallel charging.
		 */
		if (chg->thermal_status == TEMP_ALERT_LEVEL)
			goto exit;

		vote(chg->smb_override_votable, SW_THERM_REGULATION_VOTER,
				disable_smb, 0);

		/*
		 * Enable/disable secondary charger through votables to ensure
		 * that if SMB_EN pin get's toggled somehow, secondary charger
		 * remains enabled/disabled according to SW thermal regulation.
		 */
		if (!chg->cp_disable_votable)
			chg->cp_disable_votable = find_votable("CP_DISABLE");
		if (chg->cp_disable_votable)
			vote(chg->cp_disable_votable, SW_THERM_REGULATION_VOTER,
							disable_smb, 0);

		vote(chg->pl_disable_votable, SW_THERM_REGULATION_VOTER,
							disable_smb, 0);
		smblib_dbg(chg, PR_MISC, "Parallel %s as per SW thermal regulation\n",
				disable_smb ? "disabled" : "enabled");

		/*
		 * If thermal level changes to TEMP_SHUT_DOWN_SMB, don't
		 * enable/disable main charger.
		 */
		if (chg->thermal_status == TEMP_SHUT_DOWN_SMB)
			goto exit;

		/* Suspend input if SHDN threshold reached */
		vote(chg->dc_suspend_votable, SW_THERM_REGULATION_VOTER,
							suspend_input, 0);
		vote(chg->usb_icl_votable, SW_THERM_REGULATION_VOTER,
							suspend_input, 0);
		smblib_dbg(chg, PR_MISC, "USB/DC %s as per SW thermal regulation\n",
				suspend_input ? "suspended" : "unsuspended");
	}
exit:
	/*
	 * On USB suspend, WDOG IRQ stops triggering. To continue thermal
	 * monitoring and regulation until USB is plugged out, reschedule
	 * the SW thermal regulation work without releasing the wake lock.
	 */
	if (is_client_vote_enabled(chg->usb_icl_votable,
					SW_THERM_REGULATION_VOTER)) {
		schedule_delayed_work(&chg->thermal_regulation_work,
				msecs_to_jiffies(THERM_REG_RECHECK_DELAY_1S));
		return 0;
	}

	rc = smblib_masked_write(chg, SNARL_BARK_BITE_WD_CFG_REG,
			SNARL_WDOG_TIMEOUT_MASK, wdog_timeout);
	if (rc < 0)
		smblib_err(chg, "Couldn't set WD SNARL timer, rc=%d\n", rc);

	vote(chg->awake_votable, SW_THERM_REGULATION_VOTER, false, 0);
	return rc;
}

/*******************
 * DC PSY GETTERS *
 *******************/

int smblib_get_prop_voltage_wls_output(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->wls_psy) {
		chg->wls_psy = power_supply_get_by_name("wireless");
		if (!chg->wls_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->wls_psy,
				POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get POWER_SUPPLY_PROP_VOLTAGE_REGULATION, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_prop_dc_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	if (chg->smb_version == PMI632_SUBTYPE) {
		val->intval = 0;
		return 0;
	}

	rc = smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DCIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & DCIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_dc_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (chg->smb_version == PMI632_SUBTYPE) {
		val->intval = 0;
		return 0;
	}

	if (get_client_vote(chg->dc_suspend_votable, USER_VOTER)) {
		val->intval = false;
		return rc;
	}

	if (is_client_vote_enabled(chg->dc_suspend_votable,
						CHG_TERMINATION_VOTER)) {
		rc = smblib_get_prop_dc_present(chg, val);
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_DCIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_STS_BIT);

	return rc;
}

int smblib_get_prop_dc_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	return smblib_get_charge_param(chg, &chg->param.dc_icl, &val->intval);
}

int smblib_get_prop_dc_voltage_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = MICRO_12V;
	return 0;
}

int smblib_get_prop_dc_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->wls_psy) {
		chg->wls_psy = power_supply_get_by_name("wireless");
		if (!chg->wls_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->wls_psy,
				POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get POWER_SUPPLY_PROP_VOLTAGE_REGULATION, rc=%d\n",
				rc);
	return rc;
}

/*******************
 * DC PSY SETTERS *
 *******************/

int smblib_set_prop_dc_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	return smblib_set_charge_param(chg, &chg->param.dc_icl, val->intval);
}

int smblib_set_prop_voltage_wls_output(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	if (!chg->wls_psy) {
		chg->wls_psy = power_supply_get_by_name("wireless");
		if (!chg->wls_psy)
			return -ENODEV;
	}

	rc = power_supply_set_property(chg->wls_psy,
				POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set POWER_SUPPLY_PROP_VOLTAGE_REGULATION, rc=%d\n",
				rc);

	smblib_dbg(chg, PR_WLS, "Set WLS output voltage %d\n", val->intval);

	return rc;
}

int smblib_set_prop_dc_reset(struct smb_charger *chg)
{
	int rc;

	rc = vote(chg->dc_suspend_votable, VOUT_VOTER, true, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't suspend DC rc=%d\n", rc);
		return rc;
	}

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_EN_MASK,
				DCIN_EN_OVERRIDE_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set DCIN_EN_OVERRIDE_BIT rc=%d\n",
			rc);
		return rc;
	}

	rc = smblib_write(chg, DCIN_CMD_PON_REG, DCIN_PON_BIT | MID_CHG_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write %d to DCIN_CMD_PON_REG rc=%d\n",
			DCIN_PON_BIT | MID_CHG_BIT, rc);
		return rc;
	}

	/* Wait for 10ms to allow the charge to get drained */
	usleep_range(10000, 10010);

	rc = smblib_write(chg, DCIN_CMD_PON_REG, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't clear DCIN_CMD_PON_REG rc=%d\n", rc);
		return rc;
	}

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_EN_MASK, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't clear DCIN_EN_OVERRIDE_BIT rc=%d\n",
			rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, VOUT_VOTER, false, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't unsuspend  DC rc=%d\n", rc);
		return rc;
	}

	smblib_dbg(chg, PR_MISC, "Wireless charger removal detection successful\n");
	return rc;
}

/*******************
 * USB PSY GETTERS *
 *******************/

int smblib_get_prop_usb_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_usb_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (get_client_vote_locked(chg->usb_icl_votable, USER_VOTER) == 0) {
		val->intval = false;
		return rc;
	}

	if (is_client_vote_enabled(chg->usb_icl_votable,
					CHG_TERMINATION_VOTER)) {
		rc = smblib_get_prop_usb_present(chg, val);
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);
	if (asus_adapter_detecting_flag)
		val->intval = 1;
	else if (atomic_read(&feature_stop_need_release) && (stat & USE_USBIN_BIT))
		val->intval = 1;
	else if (g_Charger_mode && usb_thermal_once_flag && (stat & USE_USBIN_BIT))
		val->intval = 1;
	else
	val->intval = (stat & USE_USBIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_STS_BIT);
	return rc;
}

int smblib_get_prop_usb_voltage_max_design(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		if (chg->qc2_unsupported_voltage == QC2_NON_COMPLIANT_9V) {
			val->intval = MICRO_5V;
			break;
		} else if (chg->qc2_unsupported_voltage ==
				QC2_NON_COMPLIANT_12V) {
			val->intval = MICRO_9V;
			break;
		}
		/* else, fallthrough */
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
	case POWER_SUPPLY_TYPE_USB_PD:
		if (chg->smb_version == PMI632_SUBTYPE)
			val->intval = MICRO_9V;
		else
			val->intval = MICRO_9V;  //ASUS BSP : Kirin only supports adapters max to 9V
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

int smblib_get_prop_usb_voltage_max(struct smb_charger *chg,
					union power_supply_propval *val)
{
	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		if (chg->qc2_unsupported_voltage == QC2_NON_COMPLIANT_9V) {
			val->intval = MICRO_5V;
			break;
		} else if (chg->qc2_unsupported_voltage ==
				QC2_NON_COMPLIANT_12V) {
			val->intval = MICRO_9V;
			break;
		}
		/* else, fallthrough */
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		if (chg->smb_version == PMI632_SUBTYPE)
			val->intval = MICRO_9V;
		else
			val->intval = MICRO_12V;
		break;
	case POWER_SUPPLY_TYPE_USB_PD:
		val->intval = chg->voltage_max_uv;
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

#define HVDCP3_STEP_UV	200000
static int smblib_estimate_adaptor_voltage(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		val->intval = MICRO_12V;
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		val->intval = MICRO_5V + (HVDCP3_STEP_UV * chg->pulse_cnt);
		break;
	case POWER_SUPPLY_TYPE_USB_PD:
		/* Take the average of min and max values */
		val->intval = chg->voltage_min_uv +
			((chg->voltage_max_uv - chg->voltage_min_uv) / 2);
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

static int smblib_read_mid_voltage_chan(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.mid_chan)
		return -ENODATA;

	rc = iio_read_channel_processed(chg->iio.mid_chan, &val->intval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read MID channel rc=%d\n", rc);
		return rc;
	}

	/*
	 * If MID voltage < 1V, it is unreliable.
	 * Figure out voltage from registers and calculations.
	 */
	if (val->intval < 1000000)
		return smblib_estimate_adaptor_voltage(chg, val);

	return 0;
}

static int smblib_read_usbin_voltage_chan(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.usbin_v_chan)
		return -ENODATA;

	rc = iio_read_channel_processed(chg->iio.usbin_v_chan, &val->intval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN channel rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_get_prop_usb_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	int rc;

	rc = smblib_get_prop_usb_present(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb presence status rc=%d\n", rc);
		return -ENODATA;
	}

	/*
	 * For PM8150B, use MID_CHG ADC channel because overvoltage is observed
	 * to occur randomly in the USBIN channel, particularly at high
	 * voltages.
	 */
	if (chg->smb_version == PM8150B_SUBTYPE && pval.intval)
		return smblib_read_mid_voltage_chan(chg, val);
	else
		return smblib_read_usbin_voltage_chan(chg, val);
}

int smblib_get_prop_vph_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.vph_v_chan)
		return -ENODATA;

	rc = iio_read_channel_processed(chg->iio.vph_v_chan, &val->intval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read vph channel rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int g_r_sbu1 = 0;
int g_r_sbu2 = 0;
bool smblib_rsbux_low(struct smb_charger *chg, int r_thr)
{
	int r_sbu1, r_sbu2;
	bool ret = false;
	int rc;

	CHG_DBG("[usb_water] +++\n");

	if (!chg->iio.sbux_chan)
		return false;

	/* disable crude sensors */
	rc = smblib_masked_write(chg, TYPE_C_CRUDE_SENSOR_CFG_REG,
			EN_SRC_CRUDE_SENSOR_BIT | EN_SNK_CRUDE_SENSOR_BIT,
			0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable crude sensor rc=%d\n", rc);
		return false;
	}

	/* select SBU1 as current source */
	rc = smblib_write(chg, TYPE_C_SBU_CFG_REG, SEL_SBU1_ISRC_VAL);
	if (rc < 0) {
		smblib_err(chg, "Couldn't select SBU1 rc=%d\n", rc);
		goto cleanup;
	}

	rc = iio_read_channel_processed(chg->iio.sbux_chan, &r_sbu1);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SBU1 rc=%d\n", rc);
		goto cleanup;
	}

	if (r_sbu1 < r_thr) {
		ret = true;
		goto cleanup;
	}

	/* select SBU2 as current source */
	rc = smblib_write(chg, TYPE_C_SBU_CFG_REG, SEL_SBU2_ISRC_VAL);
	if (rc < 0) {
		smblib_err(chg, "Couldn't select SBU1 rc=%d\n", rc);
		goto cleanup;
	}

	rc = iio_read_channel_processed(chg->iio.sbux_chan, &r_sbu2);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SBU1 rc=%d\n", rc);
		goto cleanup;
	}

	if (r_sbu2 < r_thr)
		ret = true;
cleanup:
	CHG_DBG("[usb_water] ret=%d, r_sbu1(%d), r_sbu2(%d), r_thr(%d)\n", ret, r_sbu1, r_sbu2, r_thr);
	g_r_sbu1 = r_sbu1;
	g_r_sbu2 = r_sbu2;
	/* enable crude sensors */
	rc = smblib_masked_write(chg, TYPE_C_CRUDE_SENSOR_CFG_REG,
			EN_SRC_CRUDE_SENSOR_BIT | EN_SNK_CRUDE_SENSOR_BIT,
			EN_SRC_CRUDE_SENSOR_BIT | EN_SNK_CRUDE_SENSOR_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable crude sensor rc=%d\n", rc);

	/* disable current source */
	rc = smblib_write(chg, TYPE_C_SBU_CFG_REG, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't select SBU1 rc=%d\n", rc);

	CHG_DBG("[usb_water] ---\n");
	return ret;
}
EXPORT_SYMBOL_GPL(smblib_rsbux_low);

int smblib_get_prop_conn_temp(struct smb_charger *chg,
				 union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	bool usb_present, dc_present;
	int conn_temp, rc;

	rc = smblib_get_prop_usb_present(chg, &pval);
	if (rc < 0) {
		pr_err("Couldn't get usb presence status rc=%d\n", rc);
		return rc;
	}
	usb_present = pval.intval;

	rc = smblib_get_prop_dc_present(chg, &pval);
	if (rc < 0) {
		pr_err("Couldn't get dc presence status rc=%d\n", rc);
		return rc;
	}
	dc_present = pval.intval;

	if (!usb_present && !dc_present)
		return -ENODATA;

	if (chg->iio.connector_temp_chan) {
		rc = iio_read_channel_processed(chg->iio.connector_temp_chan,
				&conn_temp);
		if (rc < 0) {
			pr_err("Error in reading connector_temp channel, rc=%d", rc);
			return rc;
		}
		val->intval = conn_temp / 100;
	} else {
		CHG_DBG_E("no conn_temp io-channel-names\n");
		return -ENODATA;
	}

	return rc;
}

int smblib_get_prop_charger_temp(struct smb_charger *chg,
				 union power_supply_propval *val)
{
	int temp, rc;
	int input_present;

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return rc;

	if (input_present == INPUT_NOT_PRESENT)
		return -ENODATA;

	if (chg->iio.temp_chan) {
		rc = iio_read_channel_processed(chg->iio.temp_chan,
				&temp);
		if (rc < 0) {
			pr_err("Error in reading temp channel, rc=%d", rc);
			return rc;
		}
		val->intval = temp / 100;
	} else {
		CHG_DBG_E("no chg_temp io-channel-names\n");
		return -ENODATA;
	}

	return rc;
}

int smblib_get_prop_typec_cc_orientation(struct smb_charger *chg,
					 union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n", stat);

	if (stat & CC_ATTACHED_BIT)
		val->intval = (bool)(stat & CC_ORIENTATION_BIT) + 1;
	else
		val->intval = 0;

	return rc;
}

static const char * const smblib_typec_mode_name[] = {
	[POWER_SUPPLY_TYPEC_NONE]		  = "NONE",
	[POWER_SUPPLY_TYPEC_SOURCE_DEFAULT]	  = "SOURCE_DEFAULT",
	[POWER_SUPPLY_TYPEC_SOURCE_MEDIUM]	  = "SOURCE_MEDIUM",
	[POWER_SUPPLY_TYPEC_SOURCE_HIGH]	  = "SOURCE_HIGH",
	[POWER_SUPPLY_TYPEC_NON_COMPLIANT]	  = "NON_COMPLIANT",
	[POWER_SUPPLY_TYPEC_SINK]		  = "SINK",
	[POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE]   = "SINK_POWERED_CABLE",
	[POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY] = "SINK_DEBUG_ACCESSORY",
	[POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER]   = "SINK_AUDIO_ADAPTER",
	[POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY]   = "POWERED_CABLE_ONLY",
};

static int smblib_get_prop_ufp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_SNK_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_1 rc=%d\n", rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_1 = 0x%02x\n", stat);

	switch (stat & DETECTED_SRC_TYPE_MASK) {
	case SNK_RP_STD_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
	case SNK_RP_1P5_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_MEDIUM;
	case SNK_RP_3P0_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_HIGH;
	case SNK_RP_SHORT_BIT:
		return POWER_SUPPLY_TYPEC_NON_COMPLIANT;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

static int smblib_get_prop_dfp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	if (chg->lpd_stage == LPD_STAGE_COMMIT)
		return POWER_SUPPLY_TYPEC_NONE;

	rc = smblib_read(chg, TYPE_C_SRC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_SRC_STATUS_REG rc=%d\n",
				rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_SRC_STATUS_REG = 0x%02x\n", stat);

	switch (stat & DETECTED_SNK_TYPE_MASK) {
	case AUDIO_ACCESS_RA_RA_BIT:
		return POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER;
	case SRC_DEBUG_ACCESS_BIT:
		return POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY;
	case SRC_RD_RA_VCONN_BIT:
		return POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE;
	case SRC_RD_OPEN_BIT:
		return POWER_SUPPLY_TYPEC_SINK;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

static int smblib_get_prop_typec_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n",
				rc);
		return 0;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_MISC_STATUS_REG = 0x%02x\n", stat);

	if (stat & SNK_SRC_MODE_BIT)
		return smblib_get_prop_dfp_mode(chg);
	else
		return smblib_get_prop_ufp_mode(chg);
}

int smblib_get_prop_typec_power_role(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc = 0;
	u8 ctrl;

	rc = smblib_read(chg, TYPE_C_MODE_CFG_REG, &ctrl);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_MODE_CFG_REG rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_MODE_CFG_REG = 0x%02x\n",
		   ctrl);

	if (ctrl & TYPEC_DISABLE_CMD_BIT) {
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		return rc;
	}

	switch (ctrl & (EN_SRC_ONLY_BIT | EN_SNK_ONLY_BIT)) {
	case 0:
		val->intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		break;
	case EN_SRC_ONLY_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
		break;
	case EN_SNK_ONLY_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SINK;
		break;
	default:
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		smblib_err(chg, "unsupported power role 0x%02lx\n",
			ctrl & (EN_SRC_ONLY_BIT | EN_SNK_ONLY_BIT));
		return -EINVAL;
	}

	return rc;
}

static inline bool typec_in_src_mode(struct smb_charger *chg)
{
	return (chg->typec_mode > POWER_SUPPLY_TYPEC_NONE &&
		chg->typec_mode < POWER_SUPPLY_TYPEC_SOURCE_DEFAULT);
}

int smblib_get_prop_typec_select_rp(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc, rp;
	u8 stat;

	if (!typec_in_src_mode(chg))
		return -ENODATA;

	rc = smblib_read(chg, TYPE_C_CURRSRC_CFG_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_CURRSRC_CFG_REG rc=%d\n",
				rc);
		return rc;
	}

	switch (stat & TYPEC_SRC_RP_SEL_MASK) {
	case TYPEC_SRC_RP_STD:
		rp = POWER_SUPPLY_TYPEC_SRC_RP_STD;
		break;
	case TYPEC_SRC_RP_1P5A:
		rp = POWER_SUPPLY_TYPEC_SRC_RP_1P5A;
		break;
	case TYPEC_SRC_RP_3A:
	case TYPEC_SRC_RP_3A_DUPLICATE:
		rp = POWER_SUPPLY_TYPEC_SRC_RP_3A;
		break;
	default:
		return -EINVAL;
	}

	val->intval = rp;

	return 0;
}

int smblib_get_prop_usb_current_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	int rc = 0, buck_scale = 1, boost_scale = 1;

	if (chg->iio.usbin_i_chan) {
		rc = iio_read_channel_processed(chg->iio.usbin_i_chan,
				&val->intval);
		if (rc < 0) {
			pr_err("Error in reading USBIN_I channel, rc=%d", rc);
			return rc;
		}

		/*
		 * For PM8150B, scaling factor = reciprocal of
		 * 0.2V/A in Buck mode, 0.4V/A in Boost mode.
		 * For PMI632, scaling factor = reciprocal of
		 * 0.4V/A in Buck mode, 0.8V/A in Boost mode.
		 */
		switch (chg->smb_version) {
		case PMI632_SUBTYPE:
			buck_scale = 40;
			boost_scale = 80;
			break;
		default:
			buck_scale = 20;
			boost_scale = 40;
			break;
		}

		if (chg->otg_present || smblib_get_prop_dfp_mode(chg) !=
				POWER_SUPPLY_TYPEC_NONE) {
			val->intval = DIV_ROUND_CLOSEST(val->intval * 100,
								boost_scale);
			return rc;
		}

		rc = smblib_get_prop_usb_present(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get usb present status,rc=%d\n",
				rc);
			return -ENODATA;
		}

		/* If USB is not present, return 0 */
		if (!pval.intval)
			val->intval = 0;
		else
			val->intval = DIV_ROUND_CLOSEST(val->intval * 100,
								buck_scale);
	} else {
		val->intval = 0;
		rc = -ENODATA;
	}

	return rc;
}

int smblib_get_prop_low_power(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_SRC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_SRC_STATUS_REG rc=%d\n",
				rc);
		return rc;
	}

	val->intval = !(stat & SRC_HIGH_BATT_BIT);

	return 0;
}

int smblib_get_prop_input_current_settled(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	return smblib_get_charge_param(chg, &chg->param.icl_stat, &val->intval);
}

int smblib_get_prop_input_voltage_settled(struct smb_charger *chg,
						union power_supply_propval *val)
{
	int rc, pulses;

	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		rc = smblib_get_pulse_cnt(chg, &pulses);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
			return 0;
		}
		val->intval = MICRO_5V + HVDCP3_STEP_UV * pulses;
		break;
	case POWER_SUPPLY_TYPE_USB_PD:
		val->intval = chg->voltage_min_uv;
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

int smblib_get_prop_pd_in_hard_reset(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	val->intval = chg->pd_hard_reset;
	return 0;
}

int smblib_get_pe_start(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	val->intval = chg->ok_to_pd;
	return 0;
}

int smblib_get_prop_die_health(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	int input_present;

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return rc;

	if (input_present == INPUT_NOT_PRESENT)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	if (chg->wa_flags & SW_THERM_REGULATION_WA) {
		if (chg->die_temp == -ENODATA)
			return POWER_SUPPLY_HEALTH_UNKNOWN;

		if (chg->die_temp > DIE_TEMP_RST_THRESH)
			return POWER_SUPPLY_HEALTH_OVERHEAT;

		if (chg->die_temp > DIE_TEMP_REG_H_THRESH)
			return POWER_SUPPLY_HEALTH_HOT;

		if (chg->die_temp > DIE_TEMP_REG_L_THRESH)
			return POWER_SUPPLY_HEALTH_WARM;

		return POWER_SUPPLY_HEALTH_COOL;
	}

	rc = smblib_read(chg, DIE_TEMP_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DIE_TEMP_STATUS_REG, rc=%d\n",
				rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (stat & DIE_TEMP_RST_BIT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (stat & DIE_TEMP_UB_BIT)
		return POWER_SUPPLY_HEALTH_HOT;

	if (stat & DIE_TEMP_LB_BIT)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_COOL;
}

int smblib_get_prop_connector_health(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	if (chg->wa_flags & SW_THERM_REGULATION_WA) {
		if (chg->connector_temp == -ENODATA)
			return POWER_SUPPLY_HEALTH_UNKNOWN;

		if (chg->connector_temp > CONNECTOR_TEMP_RST_THRESH)
			return POWER_SUPPLY_HEALTH_OVERHEAT;

		if (chg->connector_temp > CONNECTOR_TEMP_REG_H_THRESH)
			return POWER_SUPPLY_HEALTH_HOT;

		if (chg->connector_temp > CONNECTOR_TEMP_REG_L_THRESH)
			return POWER_SUPPLY_HEALTH_WARM;

		return POWER_SUPPLY_HEALTH_COOL;
	}

	rc = smblib_read(chg, CONNECTOR_TEMP_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CONNECTOR_TEMP_STATUS_REG, rc=%d\n",
				rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (stat & CONNECTOR_TEMP_RST_BIT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (stat & CONNECTOR_TEMP_UB_BIT)
		return POWER_SUPPLY_HEALTH_HOT;

	if (stat & CONNECTOR_TEMP_LB_BIT)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_COOL;
}

static int get_rp_based_dcp_current(struct smb_charger *chg, int typec_mode)
{
	int rp_ua;

	switch (typec_mode) {
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		rp_ua = TYPEC_HIGH_CURRENT_UA;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
	/* fall through */
	default:
		rp_ua = DCP_CURRENT_UA;
	}

	return rp_ua;
}

/*******************
 * USB PSY SETTERS *
 * *****************/

int smblib_set_prop_pd_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, icl;

	if (chg->pd_active) {
		icl = get_client_vote(chg->usb_icl_votable, PD_VOTER);
		rc = vote(chg->usb_icl_votable, PD_VOTER, true, val->intval);
		if (val->intval != icl)
			power_supply_changed(chg->usb_psy);
	} else {
		rc = -EPERM;
	}

	return rc;
}

static int smblib_handle_usb_current(struct smb_charger *chg,
					int usb_current)
{
	int rc = 0, rp_ua, typec_mode;
	union power_supply_propval val = {0, };

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOAT) {
		if (usb_current == -ETIMEDOUT) {
			if ((chg->float_cfg & FLOAT_OPTIONS_MASK)
						== FORCE_FLOAT_SDP_CFG_BIT) {
				/*
				 * Confiugure USB500 mode if Float charger is
				 * configured for SDP mode.
				 */
				rc = set_sdp_current(chg, USBIN_500MA);
				if (rc < 0)
					smblib_err(chg,
						"Couldn't set SDP ICL rc=%d\n",
						rc);

				return rc;
			}

			if (chg->connector_type ==
					POWER_SUPPLY_CONNECTOR_TYPEC) {
				/*
				 * Valid FLOAT charger, report the current
				 * based of Rp.
				 */
				typec_mode = smblib_get_prop_typec_mode(chg);
				rp_ua = get_rp_based_dcp_current(chg,
								typec_mode);
				rc = vote(chg->usb_icl_votable,
						SW_ICL_MAX_VOTER, true, rp_ua);
				if (rc < 0)
					return rc;
			} else {
				rc = vote(chg->usb_icl_votable,
					SW_ICL_MAX_VOTER, true, DCP_CURRENT_UA);
				if (rc < 0)
					return rc;
			}
		} else {
			/*
			 * FLOAT charger detected as SDP by USB driver,
			 * charge with the requested current and update the
			 * real_charger_type
			 */
			chg->real_charger_type = POWER_SUPPLY_TYPE_USB;
			if (!asus_flow_done_flag && usb_current <= USBIN_25MA) {
				CHG_DBG("WA for SDP debounce 2mA suspend #1");
				rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
							true, USBIN_100MA);
			} else {
				rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
							true, usb_current);
			}
			if (rc < 0)
				return rc;
			rc = vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER,
							false, 0);
			if (rc < 0)
				return rc;
		}
	} else {
		rc = smblib_get_prop_usb_present(chg, &val);
		if (!rc && !val.intval)
			return 0;

		/* if flash is active force 500mA */
		if ((usb_current < SDP_CURRENT_UA) && is_flash_active(chg))
			usb_current = SDP_CURRENT_UA;

		if (!asus_flow_done_flag && usb_current <= USBIN_25MA) {
			CHG_DBG("WA for debounce SDP 2mA suspend #2");
			rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, true,
						USBIN_100MA);
		} else {
			rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, true,
						usb_current);
		}
		if (rc < 0) {
			pr_err("Couldn't vote ICL USB_PSY_VOTER rc=%d\n", rc);
			return rc;
		}

		rc = vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
		if (rc < 0) {
			pr_err("Couldn't remove SW_ICL_MAX vote rc=%d\n", rc);
			return rc;
		}

	}

	return 0;
}

int smblib_set_prop_sdp_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	union power_supply_propval pval;
	int rc = 0;

	if (!chg->pd_active) {
		rc = smblib_get_prop_usb_present(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get usb present rc = %d\n",
						rc);
			return rc;
		}

		/* handle the request only when USB is present */
		if (pval.intval)
			rc = smblib_handle_usb_current(chg, val->intval);
	} else if (chg->system_suspend_supported) {
		if (val->intval <= USBIN_25MA)
			rc = vote(chg->usb_icl_votable,
				PD_SUSPEND_SUPPORTED_VOTER, true, val->intval);
		else
			rc = vote(chg->usb_icl_votable,
				PD_SUSPEND_SUPPORTED_VOTER, false, 0);
	}
	return rc;
}

int smblib_set_prop_boost_current(struct smb_charger *chg,
					const union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher,
				val->intval <= chg->boost_threshold_ua ?
				chg->chg_freq.freq_below_otg_threshold :
				chg->chg_freq.freq_above_otg_threshold);
	if (rc < 0) {
		dev_err(chg->dev, "Error in setting freq_boost rc=%d\n", rc);
		return rc;
	}

	chg->boost_current_ua = val->intval;
	return rc;
}

int smblib_set_prop_usb_voltage_max_limit(struct smb_charger *chg,
					const union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };

	/* Exit if same value is re-configured */
	if (val->intval == chg->usbin_forced_max_uv)
		return 0;

	smblib_get_prop_usb_voltage_max_design(chg, &pval);

	if (val->intval >= MICRO_5V && val->intval <= pval.intval) {
		chg->usbin_forced_max_uv = val->intval;
		smblib_dbg(chg, PR_MISC, "Max VBUS limit changed to: %d\n",
				val->intval);
	} else if (chg->usbin_forced_max_uv) {
		chg->usbin_forced_max_uv = 0;
	} else {
		return 0;
	}

	power_supply_changed(chg->usb_psy);

	return 0;
}

int smblib_set_prop_typec_power_role(struct smb_charger *chg,
				     const union power_supply_propval *val)
{
	int rc = 0;
	u8 power_role;

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		return 0;

	switch (val->intval) {
	case POWER_SUPPLY_TYPEC_PR_NONE:
		power_role = TYPEC_DISABLE_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_DUAL:
		power_role = chg->typec_try_mode;
		break;
	case POWER_SUPPLY_TYPEC_PR_SINK:
		power_role = EN_SNK_ONLY_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_SOURCE:
		power_role = EN_SRC_ONLY_BIT;
		break;
	default:
		smblib_err(chg, "power role %d not supported\n", val->intval);
		return -EINVAL;
	}

	rc = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
				TYPEC_POWER_ROLE_CMD_MASK | TYPEC_TRY_MODE_MASK,
				power_role);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			power_role, rc);
		return rc;
	}

	return rc;
}

int smblib_set_prop_typec_select_rp(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	if (!typec_in_src_mode(chg)) {
		smblib_err(chg, "Couldn't set curr src: not in SRC mode\n");
		return -EINVAL;
	}

	if (val->intval < TYPEC_SRC_RP_MAX_ELEMENTS) {
		rc = smblib_masked_write(chg, TYPE_C_CURRSRC_CFG_REG,
				TYPEC_SRC_RP_SEL_MASK,
				val->intval);
		if (rc < 0)
			smblib_err(chg, "Couldn't write to TYPE_C_CURRSRC_CFG rc=%d\n",
					rc);
		return rc;
	}

	return -EINVAL;
}

int smblib_set_prop_pd_voltage_min(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, min_uv;

	min_uv = min(val->intval, chg->voltage_max_uv);
	if (chg->voltage_min_uv == min_uv)
		return 0;

	rc = smblib_set_usb_pd_allowed_voltage(chg, min_uv,
					       chg->voltage_max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid min voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_min_uv = min_uv;
	power_supply_changed(chg->usb_main_psy);

	return rc;
}

int smblib_set_prop_pd_voltage_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, max_uv;

	max_uv = max(val->intval, chg->voltage_min_uv);
	if (chg->voltage_max_uv == max_uv)
		return 0;

	rc = smblib_set_usb_pd_fsw(chg, max_uv);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set FSW for voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	rc = smblib_set_usb_pd_allowed_voltage(chg, chg->voltage_min_uv,
					       max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid max voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_max_uv = max_uv;
	power_supply_changed(chg->usb_main_psy);

	return rc;
}

int smblib_set_prop_pd_active(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	const struct apsd_result *apsd = smblib_get_apsd_result(chg);

	int rc = 0;
	int sec_charger;

	chg->pd_active = val->intval;

	smblib_apsd_enable(chg, !chg->pd_active);

	update_sw_icl_max(chg, apsd->pst);

	if (chg->pd_active) {
		vote(chg->limited_irq_disable_votable, CHARGER_TYPE_VOTER,
				false, 0);
		vote(chg->hdc_irq_disable_votable, CHARGER_TYPE_VOTER,
				false, 0);

		/*
		 * Enforce 100mA for PD until the real vote comes in later.
		 * It is guaranteed that pd_active is set prior to
		 * pd_current_max
		 */
		vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_100MA);
		vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);

		/*
		 * For PPS, Charge Pump is preferred over parallel charger if
		 * present.
		 */
		if (chg->pd_active == POWER_SUPPLY_PD_PPS_ACTIVE
						&& chg->sec_cp_present) {
			rc = smblib_select_sec_charger(chg,
						POWER_SUPPLY_CHARGER_SEC_CP,
						POWER_SUPPLY_CP_PPS, false);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't enable secondary charger rc=%d\n",
					rc);
		}
	} else {
		vote(chg->usb_icl_votable, PD_VOTER, false, 0);
		vote(chg->limited_irq_disable_votable, CHARGER_TYPE_VOTER,
				true, 0);
		vote(chg->hdc_irq_disable_votable, CHARGER_TYPE_VOTER,
				true, 0);

		sec_charger = chg->sec_pl_present ?
						POWER_SUPPLY_CHARGER_SEC_PL :
						POWER_SUPPLY_CHARGER_SEC_NONE;
		rc = smblib_select_sec_charger(chg, sec_charger,
						POWER_SUPPLY_CP_NONE, false);
		if (rc < 0)
			dev_err(chg->dev,
				"Couldn't enable secondary charger rc=%d\n",
					rc);

		/* PD hard resets failed, proceed to detect QC2/3 */
		if (chg->ok_to_pd) {
			chg->ok_to_pd = false;
			smblib_hvdcp_detect_enable(chg, true);
		}
	}

	smblib_update_usb_type(chg);
	power_supply_changed(chg->usb_psy);
	return rc;
}

int smblib_set_prop_ship_mode(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;

	smblib_dbg(chg, PR_MISC, "Set ship mode: %d!!\n", !!val->intval);

	rc = smblib_masked_write(chg, SHIP_MODE_REG, SHIP_MODE_EN_BIT,
			!!val->intval ? SHIP_MODE_EN_BIT : 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't %s ship mode, rc=%d\n",
				!!val->intval ? "enable" : "disable", rc);

	return rc;
}

int smblib_set_prop_pd_in_hard_reset(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc = 0;

	if (chg->pd_hard_reset == val->intval)
		return rc;

	chg->pd_hard_reset = val->intval;
	rc = smblib_masked_write(chg, TYPE_C_EXIT_STATE_CFG_REG,
			EXIT_SNK_BASED_ON_CC_BIT,
			(chg->pd_hard_reset) ? EXIT_SNK_BASED_ON_CC_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't set EXIT_SNK_BASED_ON_CC rc=%d\n",
				rc);

	return rc;
}

#define JEITA_SOFT			0
#define JEITA_HARD			1
static int smblib_update_jeita(struct smb_charger *chg, u32 *thresholds,
								int type)
{
	int rc;
	u16 temp, base;

	base = CHGR_JEITA_THRESHOLD_BASE_REG(type);

	temp = thresholds[1] & 0xFFFF;
	temp = ((temp & 0xFF00) >> 8) | ((temp & 0xFF) << 8);
	rc = smblib_batch_write(chg, base, (u8 *)&temp, 2);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't configure Jeita %s hot threshold rc=%d\n",
			(type == JEITA_SOFT) ? "Soft" : "Hard", rc);
		return rc;
	}

	temp = thresholds[0] & 0xFFFF;
	temp = ((temp & 0xFF00) >> 8) | ((temp & 0xFF) << 8);
	rc = smblib_batch_write(chg, base + 2, (u8 *)&temp, 2);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't configure Jeita %s cold threshold rc=%d\n",
			(type == JEITA_SOFT) ? "Soft" : "Hard", rc);
		return rc;
	}

	smblib_dbg(chg, PR_MISC, "%s Jeita threshold configured\n",
				(type == JEITA_SOFT) ? "Soft" : "Hard");

	return 0;
}

static int smblib_charge_inhibit_en(struct smb_charger *chg, bool enable)
{
	int rc;

	rc = smblib_masked_write(chg, CHGR_CFG2_REG,
					CHARGER_INHIBIT_BIT,
					enable ? CHARGER_INHIBIT_BIT : 0);
	return rc;
}

static int smblib_soft_jeita_arb_wa(struct smb_charger *chg)
{
	union power_supply_propval pval;
	int rc = 0;
	bool soft_jeita;

	rc = smblib_get_prop_batt_health(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get battery health rc=%d\n", rc);
		return rc;
	}

	/* Do nothing on entering hard JEITA condition */
	if (pval.intval == POWER_SUPPLY_HEALTH_COLD ||
		pval.intval == POWER_SUPPLY_HEALTH_HOT)
		return 0;

	if (chg->jeita_soft_fcc[0] < 0 || chg->jeita_soft_fcc[1] < 0 ||
		chg->jeita_soft_fv[0] < 0 || chg->jeita_soft_fv[1] < 0)
		return 0;

	soft_jeita = (pval.intval == POWER_SUPPLY_HEALTH_COOL) ||
			(pval.intval == POWER_SUPPLY_HEALTH_WARM);

	/* Do nothing on entering soft JEITA from hard JEITA */
	if (chg->jeita_arb_flag && soft_jeita)
		return 0;

	/* Do nothing, initial to health condition */
	if (!chg->jeita_arb_flag && !soft_jeita)
		return 0;

	/* Entering soft JEITA from normal state */
	if (!chg->jeita_arb_flag && soft_jeita) {
		vote(chg->chg_disable_votable, JEITA_ARB_VOTER, true, 0);

		rc = smblib_charge_inhibit_en(chg, true);
		if (rc < 0)
			smblib_err(chg, "Couldn't enable charge inhibit rc=%d\n",
					rc);

		rc = smblib_update_jeita(chg, chg->jeita_soft_hys_thlds,
					JEITA_SOFT);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't configure Jeita soft threshold rc=%d\n",
				rc);

		if (pval.intval == POWER_SUPPLY_HEALTH_COOL) {
			vote(chg->fcc_votable, JEITA_ARB_VOTER, true,
						chg->jeita_soft_fcc[0]);
			vote(chg->fv_votable, JEITA_ARB_VOTER, true,
						chg->jeita_soft_fv[0]);
		} else {
			vote(chg->fcc_votable, JEITA_ARB_VOTER, true,
						chg->jeita_soft_fcc[1]);
			vote(chg->fv_votable, JEITA_ARB_VOTER, true,
						chg->jeita_soft_fv[1]);
		}

		vote(chg->chg_disable_votable, JEITA_ARB_VOTER, false, 0);
		chg->jeita_arb_flag = true;
	} else if (chg->jeita_arb_flag && !soft_jeita) {
		/* Exit to health state from soft JEITA */

		vote(chg->chg_disable_votable, JEITA_ARB_VOTER, true, 0);

		rc = smblib_charge_inhibit_en(chg, false);
		if (rc < 0)
			smblib_err(chg, "Couldn't disable charge inhibit rc=%d\n",
					rc);

		rc = smblib_update_jeita(chg, chg->jeita_soft_thlds,
							JEITA_SOFT);
		if (rc < 0)
			smblib_err(chg, "Couldn't configure Jeita soft threshold rc=%d\n",
				rc);

		vote(chg->fcc_votable, JEITA_ARB_VOTER, false, 0);
		vote(chg->fv_votable, JEITA_ARB_VOTER, false, 0);
		vote(chg->chg_disable_votable, JEITA_ARB_VOTER, false, 0);
		chg->jeita_arb_flag = false;
	}

	smblib_dbg(chg, PR_MISC, "JEITA ARB status %d, soft JEITA status %d\n",
			chg->jeita_arb_flag, soft_jeita);
	return rc;
}

/************************
 * USB MAIN PSY GETTERS *
 ************************/
int smblib_get_prop_fcc_delta(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc, jeita_cc_delta_ua = 0;

	if (chg->sw_jeita_enabled) {
		val->intval = 0;
		return 0;
	}

	rc = smblib_get_jeita_cc_delta(chg, &jeita_cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get jeita cc delta rc=%d\n", rc);
		jeita_cc_delta_ua = 0;
	}

	val->intval = jeita_cc_delta_ua;
	return 0;
}

/************************
 * USB MAIN PSY SETTERS *
 ************************/
int smblib_get_charge_current(struct smb_charger *chg,
				int *total_current_ua)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);
	union power_supply_propval val = {0, };
	int rc = 0, typec_source_rd, current_ua;
	bool non_compliant;
	u8 stat;

	if (chg->pd_active) {
		*total_current_ua =
			get_client_vote_locked(chg->usb_icl_votable, PD_VOTER);
		return rc;
	}

	rc = smblib_read(chg, LEGACY_CABLE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_5 rc=%d\n", rc);
		return rc;
	}
	non_compliant = stat & TYPEC_NONCOMP_LEGACY_CABLE_STATUS_BIT;

	/* get settled ICL */
	rc = smblib_get_prop_input_current_settled(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get settled ICL rc=%d\n", rc);
		return rc;
	}

	typec_source_rd = smblib_get_prop_ufp_mode(chg);

	/* QC 2.0/3.0 adapter */
	if (apsd_result->bit & (QC_3P0_BIT | QC_2P0_BIT)) {
		if(!asus_flow_done_flag)
			*total_current_ua = 1000000;
		else
			*total_current_ua = HVDCP_CURRENT_UA;
		return 0;
	}

	if (non_compliant) {
		switch (apsd_result->bit) {
		case CDP_CHARGER_BIT:
			current_ua = CDP_CURRENT_UA;
			break;
		case DCP_CHARGER_BIT:
		case OCP_CHARGER_BIT:
		case FLOAT_CHARGER_BIT:
			current_ua = DCP_CURRENT_UA;
			break;
		default:
			current_ua = 0;
			break;
		}

		*total_current_ua = max(current_ua, val.intval);
		return 0;
	}

	switch (typec_source_rd) {
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
		switch (apsd_result->bit) {
		case CDP_CHARGER_BIT:
			current_ua = CDP_CURRENT_UA;
			break;
		case DCP_CHARGER_BIT:
		case OCP_CHARGER_BIT:
		case FLOAT_CHARGER_BIT:
			current_ua = chg->default_icl_ua;
			break;
		default:
			current_ua = 0;
			break;
		}
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
		current_ua = TYPEC_MEDIUM_CURRENT_UA;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		current_ua = TYPEC_HIGH_CURRENT_UA;
		break;
	case POWER_SUPPLY_TYPEC_NON_COMPLIANT:
	case POWER_SUPPLY_TYPEC_NONE:
	default:
		current_ua = 0;
		break;
	}

	*total_current_ua = max(current_ua, val.intval);
	return 0;
}

/************************
 * ASUS GET POWER_SUPPLY DATA *
 ************************/
int asus_get_prop_batt_temp(struct smb_charger *chg)
{
	union power_supply_propval temp_val = {0, };
	int rc;

	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_TEMP, &temp_val);

	return temp_val.intval;
}

int asus_get_prop_batt_volt(struct smb_charger *chg)
{
	union power_supply_propval volt_val = {0, };
	int rc;

	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_VOLTAGE_NOW, &volt_val);

	return volt_val.intval;
}

int asus_get_prop_batt_current(struct smb_charger *chg)
{
	union power_supply_propval current_val = {0, };
	int rc;

	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CURRENT_NOW, &current_val);

	return current_val.intval;
}

int asus_get_prop_batt_capacity(struct smb_charger *chg)
{
	union power_supply_propval capacity_val = {0, };
	int rc;

	rc = smblib_get_prop_batt_capacity(chg, &capacity_val);

	return capacity_val.intval;
}

#define AICL_STEP_MV		200
#define MAX_AICL_THRESHOLD_MV	4800

int asus_get_prop_batt_health(struct smb_charger *chg)
{
	union power_supply_propval health_val = {0, };
	int rc;

	rc = smblib_get_prop_batt_health(chg, &health_val);

	return health_val.intval;
}

int asus_get_prop_usb_present(struct smb_charger *chg)
{
	union power_supply_propval present_val = {0, };
	int rc;

	rc = smblib_get_prop_usb_present(chg, &present_val);

	return present_val.intval;
}

//[+++]ASUS : Add the interface for charging debug apk
int asus_get_prop_adapter_id(void)
{
	return ASUS_ADAPTER_ID;
}

int asus_get_prop_is_legacy_cable(void)
{
	if (LEGACY_CABLE_FLAG > 0)
		return 1;
	else
		return 0;
}

int asus_get_prop_total_fcc(void)
{
	return Total_FCC_Value/1000;
}

int asus_get_apsd_result_by_bit(void)
{
	const struct apsd_result *apsd_result;

	apsd_result = smblib_get_apsd_result(smbchg_dev);
	if (apsd_result->bit == (DCP_CHARGER_BIT | QC_3P0_BIT))
		return 7;
	else if (apsd_result->bit == (DCP_CHARGER_BIT | QC_2P0_BIT))
		return 6;
	else if (apsd_result->bit == FLOAT_CHARGER_BIT)
		return 5;
	else if (apsd_result->bit == OCP_CHARGER_BIT)
		return 4;
	else if (apsd_result->bit == DCP_CHARGER_BIT)
		return 3;
	else if (apsd_result->bit == CDP_CHARGER_BIT)
		return 2;
	else if (apsd_result->bit == SDP_CHARGER_BIT)
		return 1;
	else
		return 0;
}
//[---]ASUS : Add the interface for charging debug apk

/************************
 * ASUS FG GET CHARGER PARAMATER NAME *
 ************************/
const char *asus_get_apsd_result(void)
{
	const struct apsd_result *apsd_result;

	apsd_result = smblib_get_apsd_result(smbchg_dev);
	return apsd_result->name;
}

int asus_get_ufp_mode(void)
{
	int ufp_mode;

	ufp_mode = smblib_get_prop_ufp_mode(smbchg_dev);
	if (ufp_mode == POWER_SUPPLY_TYPEC_NONE)
		return 0;
	else if (ufp_mode == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT)
		return 1;
	else if (ufp_mode == POWER_SUPPLY_TYPEC_SOURCE_MEDIUM)
		return 2;
	else if (ufp_mode == POWER_SUPPLY_TYPEC_SOURCE_HIGH)
		return 3;
	else
		return 4;
}

int asus_get_batt_health(void)
{
	int bat_health;

	bat_health = asus_get_prop_batt_health(smbchg_dev);

	if (bat_health == POWER_SUPPLY_HEALTH_GOOD)
		return 0;
	else if (bat_health == POWER_SUPPLY_HEALTH_COLD) {
		//ASUSErclog(ASUS_JEITA_HARD_COLD, "JEITA Hard Cold is triggered");
		return 1;
	}
	else if (bat_health == POWER_SUPPLY_HEALTH_COOL)
		return 2;
	else if (bat_health == POWER_SUPPLY_HEALTH_WARM)
		return 3;
	else if (bat_health == POWER_SUPPLY_HEALTH_OVERHEAT) {
		//ASUSErclog(ASUS_JEITA_HARD_HOT, "JEITA Hard Hot is triggered");
		return 4;
	}
	else if (bat_health == POWER_SUPPLY_HEALTH_OVERVOLTAGE) {
		//ASUSErclog(ASUS_OUTPUT_OVP, "Battery OVP is triggered");
		return 5;
	}
	else
		return 6;
}

void asus_typec_removal_function(struct smb_charger *chg)
{
	int rc;

	rc = smblib_write(smbchg_dev, HVDCP_PULSE_COUNT_MAX_REG, 0x54);
	if (rc < 0)
		CHG_DBG_E("Failed to set HVDCP_PULSE_COUNT_MAX_REG\n");

	//Done by QCOM flow, starting BC1.2 Automatic Power Source Detection as early as after VBUS deglitch, followed by PD, and lastly QC2/3
	//smblib_apsd_enable(smbchg_dev, true);
	//smblib_hvdcp_detect_enable(smbchg_dev, true);

	cancel_delayed_work(&chg->asus_chg_flow_work);
	cancel_delayed_work(&chg->asus_adapter_adc_work);
	cancel_delayed_work(&chg->asus_min_monitor_work);
	cancel_delayed_work(&chg->asus_batt_RTC_work);
	cancel_delayed_work(&chg->asus_set_flow_flag_work);
	alarm_cancel(&bat_alarm);
	asus_flow_processing = 0;
	asus_CHG_TYPE = 0;
	ASUS_ADAPTER_ID = 0;
	HVDCP_FLAG = 0;
	UFP_FLAG = 0;
	asus_flow_done_flag = 0;
	asus_adapter_detecting_flag = 0;
	asus_quick_trigger_hvdcp = false;
	asus_set_icl = ICL_500mA;
	asus_smblib_relax(smbchg_dev);
	power_supply_changed(chg->usb_psy);
}

/************************
 * ASUS ADD BAT_ALARM *
 ************************/
static DEFINE_SPINLOCK(bat_alarm_slock);
static enum alarmtimer_restart batAlarm_handler(struct alarm *alarm, ktime_t now)
{
	CHG_DBG("batAlarm triggered\n");
	return ALARMTIMER_NORESTART;
}
void asus_batt_RTC_work(struct work_struct *dat)
{
	unsigned long batflags;
	struct timespec new_batAlarm_time;
	struct timespec mtNow;
	int RTCSetInterval = 60;

	if (!smbchg_dev) {
		CHG_DBG("driver not ready yet!\n");
		return;
	}

	if (!asus_get_prop_usb_present(smbchg_dev)) {
		alarm_cancel(&bat_alarm);
		CHG_DBG("usb not present, cancel\n");
		return;
	}
	mtNow = current_kernel_time();
	new_batAlarm_time.tv_sec = 0;
	new_batAlarm_time.tv_nsec = 0;

	RTCSetInterval = 60;

	new_batAlarm_time.tv_sec = mtNow.tv_sec + RTCSetInterval;
	spin_lock_irqsave(&bat_alarm_slock, batflags);
	alarm_start(&bat_alarm, timespec_to_ktime(new_batAlarm_time));
	spin_unlock_irqrestore(&bat_alarm_slock, batflags);
}

//[+++]ASUS : Add demo app read ADF function
int ADF_check_status(void)
{
	char buf[4] = {0, 0, 0, 0};
	struct file *fd;

	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fd = filp_open(ADF_PATH, O_RDONLY, 0);

	if (!IS_ERR(fd)) {
		kernel_read(fd, buf, 4, &(fd->f_pos));
		filp_close(fd, NULL);
	} else {
		set_fs(old_fs);
		CHG_DBG_E("Open %s failed\n", ADF_PATH);
		return 0;
	}

	filp_close(fd, NULL);
	set_fs(old_fs);

	if (buf[3] == 1 || buf[3] == 2) {
		return 1;
	} else {
		CHG_DBG_E("Check ADF failed, buf[3]=%d\n", buf[3]);
		return 0;
	}
}
//[---]ASUS : Add demo app read ADF function

/************************
 * ASUS CHARGER FLOW *
 ************************/

#define ICL_1000mA	0x14
#define ICL_1500mA	0x1E
#define ICL_1750mA	0x23
#define ICL_2000mA	0x28
#define ICL_2850mA	0x39

#define ASUS_MONITOR_CYCLE	60000
#define ADC_WAIT_TIME_HVDCP0	3000
#define ADC_WAIT_TIME_HVDCP23	100

#define EVB_750K_MIN	0xC2
#define EVB_750K_MAX	0xDE
#define EVB_200K_MIN	0x2F
#define EVB_200K_MAX	0x41
#define ER_750K_MIN		0x37
#define ER_750K_MAX		0x53
#define ER_200K_MIN		0x17
#define ER_200K_MAX		0x33
#define EVB_DMV_DPV_TH_LOW	0x2C
#define EVB_DMV_DPV_TH_HIGH	0x78

//[+++]ASUS : Add per min monitor jeita & thermal & typeC_DFP
void smblib_asus_monitor_start(struct smb_charger *chg, int time)
{
	asus_flow_done_flag = 1;
	cancel_delayed_work(&chg->asus_min_monitor_work);
	schedule_delayed_work(&chg->asus_min_monitor_work, msecs_to_jiffies(time));
	
	if(LEGACY_CABLE_FLAG == 0 && !(smbchg_dev->pd_active))
		schedule_delayed_work(&smbchg_dev->asus_cable_capability_check_work, msecs_to_jiffies(12000));
		
	schedule_delayed_work(&chg->asus_enable_inov_work, msecs_to_jiffies(60000));
}

static int SW_recharge(struct smb_charger *chg)
{
	int capacity;
	u8 termination_reg;
	bool termination_done = 0;
	int rc;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &termination_reg);
	if (rc < 0) {
		CHG_DBG_E("Couldn't read BATTERY_CHARGER_STATUS_1_REG\n");
		return rc;
	}

	if ((termination_reg & BATTERY_CHARGER_STATUS_MASK) == 0x05)
		termination_done = 1;

	rc = fg_get_msoc(g_fg, &capacity);

	CHG_DBG("capacity = %d, termination_done = %d, termination_reg = 0x%x\n", capacity, termination_done, termination_reg);

	if (capacity <= 98 && termination_done) {
		rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG, CHARGING_ENABLE_CMD_BIT, 0);  //Disabled = 0
		if (rc < 0) {
			CHG_DBG_E("Couldn't write charging_enable\n");
			return rc;
		}

		rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG, CHARGING_ENABLE_CMD_BIT, CHARGING_ENABLE_CMD_BIT);  //Enabled = 1
		if (rc < 0) {
			CHG_DBG_E("Couldn't write charging_enable\n");
			return rc;
		}
		CHG_DBG("done recharge\n");
	}
	return 0;
}

#define EN_BAT_CHG_EN_COMMAND_TRUE		BIT(0)
#define EN_BAT_CHG_EN_COMMAND_FALSE		0

enum JEITA_state {
	JEITA_STATE_INITIAL,
	JEITA_STATE_LESS_THAN_0,
	JEITA_STATE_RANGE_0_to_100,
	JEITA_STATE_RANGE_100_to_200,
	JEITA_STATE_RANGE_200_to_450,
	JEITA_STATE_RANGE_450_to_550,
	JEITA_STATE_LARGER_THAN_550,
};
int smbchg_jeita_judge_state(int old_State, int batt_tempr)
{
	int result_State;

	//decide value to set each reg (Vchg, Charging enable, Fast charge current)
	//batt_tempr < 0
	if (batt_tempr < 0) {
		result_State = JEITA_STATE_LESS_THAN_0;
	//0 <= batt_tempr < 10
	} else if (batt_tempr < 100) {
		result_State = JEITA_STATE_RANGE_0_to_100;
	//10 <= batt_tempr < 20
	} else if (batt_tempr < 200) {
		result_State = JEITA_STATE_RANGE_100_to_200;
	//20 <= batt_tempr < 45
	} else if (batt_tempr < 450) {
		result_State = JEITA_STATE_RANGE_200_to_450;
	//45 <= batt_tempr < 55
	} else if (batt_tempr < 550) {
		result_State = JEITA_STATE_RANGE_450_to_550;
	//55 <= batt_tempr
	} else{
		result_State = JEITA_STATE_LARGER_THAN_550;
	}

	//ASUS BSP : do 3 degree hysteresis
	if (old_State == JEITA_STATE_LESS_THAN_0 && result_State == JEITA_STATE_RANGE_0_to_100) {
		if (batt_tempr <= 30) {
			result_State = old_State;
		}
	}
	if (old_State == JEITA_STATE_RANGE_0_to_100 && result_State == JEITA_STATE_RANGE_100_to_200) {
		if (batt_tempr <= 130) {
			result_State = old_State;
		}
	}
	if (old_State == JEITA_STATE_RANGE_100_to_200 && result_State == JEITA_STATE_RANGE_200_to_450) {
		if (batt_tempr <= 230) {
			result_State = old_State;
		}
	}
	if (old_State == JEITA_STATE_RANGE_450_to_550 && result_State == JEITA_STATE_RANGE_200_to_450) {
		if (batt_tempr >= 420) {
			result_State = old_State;
		}
	}
	if (old_State == JEITA_STATE_LARGER_THAN_550 && result_State == JEITA_STATE_RANGE_450_to_550) {
		if (batt_tempr >= 520) {
			result_State = old_State;
		}
	}
	return result_State;
}

static int jeita_status_regs_write(u8 chg_en, int FV_uV, int FCC_uA)
{
	int rc;
	static int old_FV_uV = 0;

	CHG_DBG("old_FV = %duV, FV = %duV, FCC = %duA, chg_en = %d\n", old_FV_uV, FV_uV, FCC_uA, chg_en);

	if (old_FV_uV != FV_uV) {
		rc = smblib_masked_write(smbchg_dev, CHARGING_ENABLE_CMD_REG,  //Disabled = 0
				CHARGING_ENABLE_CMD_BIT, 0);
		if (rc < 0) {
			CHG_DBG_E("Couldn't write charging_enable rc = %d\n", rc);
			return rc;
		}
	}

	vote(smbchg_dev->fv_votable, BATT_PROFILE_VOTER, true, FV_uV);

	vote(smbchg_dev->fcc_votable, BATT_PROFILE_VOTER, true, FCC_uA);

	rc = smblib_masked_write(smbchg_dev, CHARGING_ENABLE_CMD_REG,  //Enabled = 1, Disabled = 0
			CHARGING_ENABLE_CMD_BIT, chg_en);
	if (rc < 0) {
		CHG_DBG_E("Couldn't write charging_enable rc = %d\n", rc);
		return rc;
	}

	old_FV_uV = FV_uV;
	return 0;
}

void jeita_rule(void)
{
	static int state = JEITA_STATE_INITIAL;
	int rc;
	int bat_volt;
	int bat_temp;
	int bat_health;
	int bat_capacity;
	u8 charging_enable;
	u8 FV_reg = 0x4C;  //0x1070 = 0x4C, FV_4p36V
	u8 ICL_reg = 0;
	int FV_uV;
	int FCC_uA;
	bool feature_stop_chg_flag = 0;

	CHG_DBG("+++\n");

	rc = smblib_write(smbchg_dev, JEITA_EN_CFG_REG, 0x10);
	//rc = smblib_write(smbchg_dev, JEITA_EN_CFG_REG, 0x00);  //ASUS: WA: Disable JEITA for Ara battery at EVB
	if (rc < 0)
		CHG_DBG_E("Failed to set JEITA_EN_CFG_REG\n");

	rc = smblib_read(smbchg_dev, CHGR_FLOAT_VOLTAGE_CFG_REG, &FV_reg);
	if (rc < 0)
		CHG_DBG_E("Couldn't read CHGR_FLOAT_VOLTAGE_CFG_REG\n");

	rc = smblib_read(smbchg_dev, USBIN_CURRENT_LIMIT_CFG_REG, &ICL_reg);
	if (rc < 0)
		CHG_DBG_E("Couldn't read USBIN_CURRENT_LIMIT_CFG_REG\n");

	bat_health = asus_get_batt_health();
	bat_temp = asus_get_prop_batt_temp(smbchg_dev);
	bat_volt = asus_get_prop_batt_volt(smbchg_dev);
	bat_capacity = asus_get_prop_batt_capacity(smbchg_dev);
	state = smbchg_jeita_judge_state(state, bat_temp);
	CHG_DBG("health = %s, temp = %d, capacity = %d, volt = %duV, ICL = %duA(0x%x)\n",
			health_type[bat_health], bat_temp, bat_capacity, bat_volt, (int)ICL_reg*50000, ICL_reg);

	switch (state) {
	case JEITA_STATE_LESS_THAN_0:
		charging_enable = EN_BAT_CHG_EN_COMMAND_FALSE;
		FV_uV = FV_JEITA_uV;
		FCC_uA = 1350000;
		CHG_DBG("temperature < 0\n");
		break;
	case JEITA_STATE_RANGE_0_to_100:
		charging_enable = EN_BAT_CHG_EN_COMMAND_TRUE;
		FV_uV = FV_JEITA_uV;
		FCC_uA = 1350000;
		CHG_DBG("0 <= temperature < 10\n");
		rc = SW_recharge(smbchg_dev);
		if (rc < 0) {
			CHG_DBG_E("SW_recharge failed rc = %d\n", rc);
		}
		break;
	case JEITA_STATE_RANGE_100_to_200:
		charging_enable = EN_BAT_CHG_EN_COMMAND_TRUE;
		FV_uV = FV_JEITA_uV;
		FCC_uA = 2300000;
		CHG_DBG("10 <= temperature < 20\n");
		rc = SW_recharge(smbchg_dev);
		if (rc < 0) {
			CHG_DBG_E("SW_recharge failed rc = %d\n", rc);
		}
		break;
	case JEITA_STATE_RANGE_200_to_450:
		charging_enable = EN_BAT_CHG_EN_COMMAND_TRUE;
		if (bat_volt <= 4250000) {
			FV_uV = FV_JEITA_uV;
			FCC_uA = 3800000;
		} else {
			FV_uV = FV_JEITA_uV;
			FCC_uA = 2300000;
		}
		CHG_DBG("20 <= temperature < 45\n");
		rc = SW_recharge(smbchg_dev);
		if (rc < 0) {
			CHG_DBG_E("SW_recharge failed rc = %d\n", rc);
		}
		break;
	case JEITA_STATE_RANGE_450_to_550:
		if (bat_volt >= 4100000 && FV_reg == 0x4C) {
			charging_enable = EN_BAT_CHG_EN_COMMAND_FALSE;
			FV_uV = FV_JEITA_uV;
		} else {
			charging_enable = EN_BAT_CHG_EN_COMMAND_TRUE;
			FV_uV = 4080000;
		}
		FCC_uA = 2300000;
		CHG_DBG("45 <= temperature < 55\n");
		break;
	case JEITA_STATE_LARGER_THAN_550:
		charging_enable = EN_BAT_CHG_EN_COMMAND_FALSE;
		FV_uV = FV_JEITA_uV;
		FCC_uA = 2300000;
		CHG_DBG("temperature >= 55\n");
		break;
	default:
		charging_enable = EN_BAT_CHG_EN_COMMAND_FALSE;
		FV_uV = FV_JEITA_uV;
		FCC_uA = 1350000;
		CHG_DBG("jeita judge failed, set default setting\n");
		break;
	}
	Total_FCC_Value = FCC_uA;  //Add the interface for charging debug apk

//ASUS : Stop charging - ftm limit +++
	if (charger_limit_enable_flag && (bat_capacity >= charger_limit_value)) {
		CHG_DBG("[stop_charging] ftm limit, capacity(%d) >= limit(%d)\n", bat_capacity, charger_limit_value);
		charging_enable = EN_BAT_CHG_EN_COMMAND_FALSE;
	}

//ASUS : Stop charging - demo_app limit & ultra_bat_life limit +++
	if ((demo_app_status_flag && ADF_check_status()) || ultra_bat_life_flag) {
		atomic_set(&feature_stop_need_release, 1);
		if (bat_capacity > 60) {
			smblib_set_usb_suspend(smbchg_dev, true);
			feature_stop_chg_flag = true;
		} else if (bat_capacity >= 58) {
			smblib_set_usb_suspend(smbchg_dev, false);
			feature_stop_chg_flag = true;
		} else {
			smblib_set_usb_suspend(smbchg_dev, false);
			feature_stop_chg_flag = false;
		}
	} else if (atomic_read(&feature_stop_need_release)) {
		atomic_set(&feature_stop_need_release, 0);
		smblib_set_usb_suspend(smbchg_dev, false);
		CHG_DBG("[stop_charging] leaving feautre limit, capacity(%d), demo(%d), ultra(%d), smart(%d)\n",
				bat_capacity, (demo_app_status_flag && ADF_check_status()), ultra_bat_life_flag, smartchg_stop_flag);
	}

	if (feature_stop_chg_flag || smartchg_stop_flag) {  //ASUS : Stop charging - support smartchg_stop +++
		CHG_DBG("[stop_charging] feature limit, capacity(%d), demo(%d), ultra(%d), smart(%d)\n",
				bat_capacity, (demo_app_status_flag && ADF_check_status()), ultra_bat_life_flag, smartchg_stop_flag);
		charging_enable = EN_BAT_CHG_EN_COMMAND_FALSE;
	}

	if (no_input_suspend_flag) {
		charging_enable = EN_BAT_CHG_EN_COMMAND_TRUE;
	}

	rc = jeita_status_regs_write(charging_enable, FV_uV, FCC_uA);
	if (rc < 0)
		CHG_DBG("Couldn't write jeita_status_register, rc = %d\n", rc);

	CHG_DBG("---\n");
}

void asus_cable_capability_check_work(struct work_struct *work)
{
	const struct apsd_result *apsd_result;
	u8 set_icl;
	u8 aicl_stat;
	u8 icl_stat;
	static u8 pre_icl = 0;
	static int count = 0;
	bool cable_change_to_legacy_medium = false;
	bool cable_change_to_legacy_high = false;
	bool cable_capabilit_changed = false;
	int rc;
	
	CHG_DBG("+++");
	
	if (!asus_get_prop_usb_present(smbchg_dev)) {
		asus_typec_removal_function(smbchg_dev);
		return;
	}
	
	rc = smblib_read(smbchg_dev, AICL_STATUS_REG, &aicl_stat);
	if (rc < 0) {
		smblib_err(smbchg_dev, "Couldn't read AICL_STATUS rc=%d\n", rc);
		return;
	}

	rc = smblib_read(smbchg_dev, 0x1107, &icl_stat);
	if (rc < 0) {
		smblib_err(smbchg_dev, "Couldn't read ICL_STATUS rc=%d\n", rc);
		return;
	}

	CHG_DBG("icl_stat = 0x%x \n", icl_stat);
	if(aicl_stat & AICL_DONE_BIT){
		if(icl_stat > pre_icl){
			pre_icl = icl_stat;
			schedule_delayed_work(&smbchg_dev->asus_cable_capability_check_work, msecs_to_jiffies(5000));
			return;
		}
		else{
			count++;
			if(count < 3){
				schedule_delayed_work(&smbchg_dev->asus_cable_capability_check_work, msecs_to_jiffies(5000));
				return;
			}
		}
	}
	else {
		schedule_delayed_work(&smbchg_dev->asus_cable_capability_check_work, msecs_to_jiffies(5000));
		return;
	}

	if(UFP_FLAG == 2){ //1.5A
		if((aicl_stat & AICL_DONE_BIT) && (icl_stat*50 <= 1350)){
			LEGACY_CABLE_FLAG = 1;
			cable_change_to_legacy_medium = true;
			CHG_DBG("change to legacy cable medium\n");
		}
	}
	else if(UFP_FLAG == 3) { //3A
		if((aicl_stat & AICL_DONE_BIT) && (icl_stat*50 <= 1800)){
			LEGACY_CABLE_FLAG = 1;
			cable_change_to_legacy_high = true;
			CHG_DBG("change to legacy cable high\n");
		}
	}
	
	if(!cable_change_to_legacy_medium && !cable_change_to_legacy_high){
		CHG_DBG("cable capability doesn't change\n");
		return;
	}
	
	apsd_result = smblib_update_usb_type(smbchg_dev);
	
	switch (apsd_result->bit) {

	case SDP_CHARGER_BIT:
	case FLOAT_CHARGER_BIT:
		set_icl = ICL_500mA;
		cable_capabilit_changed = true;
		break;
	case CDP_CHARGER_BIT:
		if (g_Charger_mode)
			set_icl = ICL_500mA;
		else
			set_icl = ICL_1500mA;

		cable_capabilit_changed = true;
		break;
	case OCP_CHARGER_BIT:
		set_icl = ICL_1000mA;
		cable_capabilit_changed = true;
		break;
	case DCP_CHARGER_BIT:
		if(ASUS_ADAPTER_ID == ASUS_750K || ASUS_ADAPTER_ID == PB)
			set_icl = ICL_2000mA;
		else if(ASUS_ADAPTER_ID == ASUS_200K)
			set_icl = ICL_1000mA;
		else if(ASUS_ADAPTER_ID == OTHERS)
			set_icl = ICL_1000mA;
		else if(ASUS_ADAPTER_ID == ADC_NOT_READY)
			set_icl = ICL_1000mA;
		
		cable_capabilit_changed = true;
		break;
	default:
		schedule_delayed_work(&smbchg_dev->asus_cable_capability_check_work, msecs_to_jiffies(60000));
		break;
	}
	
	if(cable_capabilit_changed){
		CHG_DBG("set icl to %dmA\n", set_icl*50);
		
		rc = asus_exclusive_vote(smbchg_dev->usb_icl_votable, ASUS_ICL_VOTER, true, (int)set_icl*50000);
		if (rc < 0)
			CHG_DBG_E("Failed to set ICL\n");

		asus_set_icl = set_icl;
	}
}

void update_inov_info(void)
{

	u8 die_temp_status = 0;
	u8 skin_temp_status = 0;
	
	smblib_read(smbchg_dev, DIE_TEMP_STATUS_REG, &die_temp_status);
	smblib_read(smbchg_dev, SKIN_TEMP_STATUS_REG, &skin_temp_status);

	CHG_DBG("INOV info: DIE_TEMP_STATUS_REG: 0x%x, SKIN_TEMP_STATUS_REG: 0x%x\n",
		die_temp_status, skin_temp_status);
}

void asus_min_monitor_work(struct work_struct *work)
{
	union power_supply_propval current_val;
	union power_supply_propval voltage_val;
	
	if (!smbchg_dev) {
		CHG_DBG_E("smbchg_dev is null due to driver probed isn't ready\n");
		return;
	}

	if (!asus_get_prop_usb_present(smbchg_dev)) {
		asus_typec_removal_function(smbchg_dev);
		return;
	}
	
	smblib_get_prop_usb_current_now(smbchg_dev, &current_val);
	smblib_get_prop_usb_voltage_now(smbchg_dev, &voltage_val);
	CHG_DBG("input_current: %d, input_voltage: %d\n", current_val.intval, voltage_val.intval);	
	update_inov_info();

	jeita_rule();

	if (asus_get_prop_usb_present(smbchg_dev)) {
		last_jeita_time = current_kernel_time();
		schedule_delayed_work(&smbchg_dev->asus_min_monitor_work, msecs_to_jiffies(ASUS_MONITOR_CYCLE));
		schedule_delayed_work(&smbchg_dev->asus_batt_RTC_work, 0);
	}
	//asus_smblib_relax(smbchg_dev);
}
//[---]ASUS : Add per min monitor jeita & thermal & typeC_DFP

void asus_chg_flow_work(struct work_struct *work)
{
	const struct apsd_result *apsd_result;
	int rc;
	u8 set_icl;
	u8 legacy_cable_reg = TYPEC_LEGACY_CABLE_STATUS_BIT;

	CHG_DBG("+++\n");

	if (!asus_get_prop_usb_present(smbchg_dev)) {
		asus_typec_removal_function(smbchg_dev);
		return;
	}

	apsd_result = smblib_update_usb_type(smbchg_dev);
	if (apsd_result->bit == (DCP_CHARGER_BIT | QC_3P0_BIT))
		HVDCP_FLAG = 3;
	else if (apsd_result->bit == (DCP_CHARGER_BIT | QC_2P0_BIT))
		HVDCP_FLAG = 2;
	else
		HVDCP_FLAG = 0;

	UFP_FLAG = asus_get_ufp_mode();

	rc = smblib_read(smbchg_dev, LEGACY_CABLE_STATUS_REG, &legacy_cable_reg);
	if (rc < 0)
		CHG_DBG_E("Couldn't read LEGACY_CABLE_STATUS_REG\n");

	if((legacy_cable_reg & TYPEC_LEGACY_CABLE_TYPE_MASK) == 0) //non-legacy
		LEGACY_CABLE_FLAG = 0;
	else if((legacy_cable_reg & TYPEC_LEGACY_CABLE_TYPE_MASK) == 3) //noncomp-legacy
		LEGACY_CABLE_FLAG = 1;
	else if((legacy_cable_reg & TYPEC_LEGACY_CABLE_TYPE_MASK) == 2) //legacy
		LEGACY_CABLE_FLAG = 2;
	else {
		CHG_DBG("LEGACY_CABLE_FLAG error, set LEGACY_CABLE_FLAG = 2\n");
		LEGACY_CABLE_FLAG = 2;
	}

	CHG_DBG("%s detected, UFP_FLAG = %s, LEGACY_CABLE_FLAG = %d\n",
			apsd_result->name, ufp_type[UFP_FLAG], LEGACY_CABLE_FLAG);

	if ((apsd_result->bit == 0) && (UFP_FLAG != 0)) {
		CHG_DBG("APSD not ready yet, delay 1s\n");
		msleep(1000);
		apsd_result = smblib_update_usb_type(smbchg_dev);
		if (apsd_result->bit == (DCP_CHARGER_BIT | QC_3P0_BIT))
			HVDCP_FLAG = 3;
		else if (apsd_result->bit == (DCP_CHARGER_BIT | QC_2P0_BIT))
			HVDCP_FLAG = 2;
		else
			HVDCP_FLAG = 0;
		CHG_DBG("Retry %s detected\n", apsd_result->name);
	}

	if (smbchg_dev->pd_active) {
		CHG_DBG("PD_active\n");
		asus_adapter_detecting_flag = 0;
		vote(smbchg_dev->pl_enable_votable_indirect, USBIN_I_VOTER, true, 0);
		smblib_asus_monitor_start(smbchg_dev, 0);  //ASUS BSP : Jeita start
		return;
	}

	switch (apsd_result->bit) {

	case SDP_CHARGER_BIT:
	case FLOAT_CHARGER_BIT:
		if (g_Charger_mode) {
			rc = smblib_masked_write(smbchg_dev, USBIN_ICL_OPTIONS_REG,
				USB51_MODE_BIT, USB51_MODE_BIT);
			if (rc < 0)
				CHG_DBG_E("Couldn't set ICL options\n");
		}

		if (UFP_FLAG == 3 && LEGACY_CABLE_FLAG == 0)
			set_icl = ICL_2000mA;
		else if (UFP_FLAG == 2 && LEGACY_CABLE_FLAG == 0)
			set_icl = ICL_1500mA;
		else
			set_icl = ICL_500mA;

		rc = asus_exclusive_vote(smbchg_dev->usb_icl_votable, ASUS_ICL_VOTER, true, (int)set_icl*50000);
		//rc = smblib_masked_write(smbchg_dev, USBIN_CURRENT_LIMIT_CFG_REG, USBIN_CURRENT_LIMIT_MASK, set_icl);
		if (rc < 0)
			CHG_DBG_E("Failed to set ICL\n");
		asus_set_icl = set_icl;
		asus_adapter_detecting_flag = 0;
		smblib_asus_monitor_start(smbchg_dev, 0);  //ASUS BSP : Jeita start
		break;
	case CDP_CHARGER_BIT:
		if (g_Charger_mode) {
			rc = smblib_masked_write(smbchg_dev, USBIN_ICL_OPTIONS_REG,
				USBIN_MODE_CHG_BIT | USB51_MODE_BIT, USBIN_MODE_CHG_BIT | USB51_MODE_BIT);
			if (rc < 0)
				CHG_DBG_E("Couldn't set ICL options\n");
		}

		if (UFP_FLAG == 3 && LEGACY_CABLE_FLAG == 0)
			set_icl = ICL_2000mA;
		else
			set_icl = ICL_1500mA;

		rc = asus_exclusive_vote(smbchg_dev->usb_icl_votable, ASUS_ICL_VOTER, true, (int)set_icl*50000);
		//rc = smblib_masked_write(smbchg_dev, USBIN_CURRENT_LIMIT_CFG_REG, USBIN_CURRENT_LIMIT_MASK, set_icl);
		if (rc < 0)
			CHG_DBG_E("Failed to set ICL\n");
		asus_set_icl = set_icl;
		asus_adapter_detecting_flag = 0;
		smblib_asus_monitor_start(smbchg_dev, 0);  //ASUS BSP : Jeita start
		break;
	case OCP_CHARGER_BIT:
		if (UFP_FLAG == 3 && LEGACY_CABLE_FLAG == 0)
			set_icl = ICL_2000mA;
		else if (UFP_FLAG == 2 && LEGACY_CABLE_FLAG == 0)
			set_icl = ICL_1500mA;
		else
			set_icl = ICL_1000mA;

		rc = asus_exclusive_vote(smbchg_dev->usb_icl_votable, ASUS_ICL_VOTER, true, (int)set_icl*50000);
		//rc = smblib_masked_write(smbchg_dev, USBIN_CURRENT_LIMIT_CFG_REG, USBIN_CURRENT_LIMIT_MASK, set_icl);
		if (rc < 0)
			CHG_DBG_E("Failed to set ICL\n");
		asus_set_icl = set_icl;
		asus_adapter_detecting_flag = 0;
		smblib_asus_monitor_start(smbchg_dev, 0);  //ASUS BSP : Jeita start
		break;

	case DCP_CHARGER_BIT | QC_3P0_BIT:
	case DCP_CHARGER_BIT | QC_2P0_BIT:
	case DCP_CHARGER_BIT:
		//#1: HVDCP_PULSE_COUNT_MAX = QC2_5V & QC3 PULSE COUNT=0
		//rc = smblib_write(smbchg_dev, HVDCP_PULSE_COUNT_MAX_REG, 0x0);
		//if (rc < 0)
		//	CHG_DBG_E("Failed to set HVDCP_PULSE_COUNT_MAX_REG\n");


		//#2: USBIN_OPTIONS1_CFG = Disable BC1.2 & Disable HVDCP
		smblib_apsd_enable(smbchg_dev, false);
		rc = smblib_masked_write(smbchg_dev, USBIN_OPTIONS_1_CFG_REG, HVDCP_EN_BIT, 0);
		if (rc < 0) {
			CHG_DBG_E("Couldn't set HVDCP_EN_BIT rc=%d\n", rc);
		}

		//#3: Re-run APSD
		CHG_DBG("Rerun APSD 1st\n");
		rc = smblib_masked_write(smbchg_dev, CMD_APSD_REG, APSD_RERUN_BIT, APSD_RERUN_BIT);
		if (rc < 0)
			CHG_DBG_E("Failed to set CMD_APSD_REG\n");

		//#1: USB DPDM swith to 2D(ADC), ADC_SW_EN-gpio101 = 1
		rc = gpio_direction_output(global_gpio->ADC_SW_EN, 1);
		if (rc) {
			CHG_DBG_E("Failed to swith to 2D, ADC_SW_EN-gpio101(%d)\n", gpio_get_value(global_gpio->ADC_SW_EN));
		} else {
			CHG_DBG("USB DPDM swith to 2D, ADC_SW_EN-gpio101(%d) = 1\n", gpio_get_value(global_gpio->ADC_SW_EN));
		}

		//#2: Delay 3s (HVDCP Flag= 0) / 0.1s (HVDCP Flag= others)
		if (asus_get_prop_usb_present(smbchg_dev)) {
			if (HVDCP_FLAG == 0) {
				CHG_DBG("HVDCP_FLAG = 0, ADC_WAIT_TIME = 3s\n");
				schedule_delayed_work(&smbchg_dev->asus_adapter_adc_work, msecs_to_jiffies(ADC_WAIT_TIME_HVDCP0));
			} else {
				CHG_DBG("HVDCP_FLAG = 2or3, ADC_WAIT_TIME = 0.1s\n");
				schedule_delayed_work(&smbchg_dev->asus_adapter_adc_work, msecs_to_jiffies(ADC_WAIT_TIME_HVDCP23));
			}
		}
		break;
	default:
		asus_adapter_detecting_flag = 0;
		//asus_smblib_relax(smbchg_dev);
		break;
	}

	CHG_DBG("---\n");
}

//[+++]ASUS : Add ASUS Adapter Detecting
#define KIRIN_750K_MIN 743
#define KIRIN_750K_MAX 892
#define KIRIN_200K_MIN 323
#define KIRIN_200K_MAX 423
#define VADC_THD_300MV  300
#define VADC_THD_1000MV  1000
void CHG_TYPE_judge(void)
{
	int rc;
	int vadc;
	int MIN_750K, MAX_750K, MIN_200K, MAX_200K;

	MIN_750K = KIRIN_750K_MIN;
	MAX_750K = KIRIN_750K_MAX;
	MIN_200K = KIRIN_200K_MIN;
	MAX_200K = KIRIN_200K_MAX;

	if (smbchg_dev->iio.asus_adapter_vadc_chan) {
		rc = iio_read_channel_processed(smbchg_dev->iio.asus_adapter_vadc_chan, &vadc);
		if (rc < 0) {
			CHG_DBG_E("Error in reading asus_adapter_vadc channel, rc=%d\n", rc);
			ASUS_ADAPTER_ID = ADC_NOT_READY;
			return;
		}
		vadc = vadc / 1000;  /* uV to mV */
		CHG_DBG("vadc(%dmV), first read\n", vadc);

		if (vadc <= VADC_THD_300MV) {
			//#1: Pull-high 620k ohm, ADCPWREN_PMI_GP1-gpio120 = 1
			rc = gpio_direction_output(global_gpio->ADCPWREN_PMI_GP1, 1);
			if (rc) {
				CHG_DBG_E("Failed to pull-high 620k ohm, ADCPWREN_PMI_GP1-gpio120(%d)\n", gpio_get_value(global_gpio->ADCPWREN_PMI_GP1));
			} else {
				CHG_DBG("Pull-high 620k ohm, ADCPWREN_PMI_GP1-gpio120(%d) = 1\n", gpio_get_value(global_gpio->ADCPWREN_PMI_GP1));
			}

			//#2: Delay 5ms
			msleep(5);

			rc = iio_read_channel_processed(smbchg_dev->iio.asus_adapter_vadc_chan, &vadc);
			if (rc < 0) {
				CHG_DBG_E("Error in reading asus_adapter_vadc channel, rc=%d\n", rc);
				ASUS_ADAPTER_ID = ADC_NOT_READY;
				return;
			}
			vadc = vadc / 1000;  /* uV to mV */
			CHG_DBG("vadc(%dmV), after pull-high 620k ohm\n", vadc);

			if (vadc >= VADC_THD_1000MV) {
				ASUS_ADAPTER_ID = OTHERS;
			} else {
				if (vadc >= MIN_750K && vadc <= MAX_750K)
					ASUS_ADAPTER_ID = ASUS_750K;
				else if (vadc >= MIN_200K && vadc <= MAX_200K)
					ASUS_ADAPTER_ID = ASUS_200K;
				else
					ASUS_ADAPTER_ID = OTHERS;
			}
		} else {
			if (vadc >= VADC_THD_1000MV)
				ASUS_ADAPTER_ID = PB;
			else
				ASUS_ADAPTER_ID = OTHERS;
		}
	} else {
		CHG_DBG_E("no asus_adapter_vadc io-channel-names\n");
		ASUS_ADAPTER_ID = ADC_NOT_READY;
	}
}

void asus_adapter_adc_work(struct work_struct *work)
{
	int rc;
	u8 usb_max_current = ICL_1000mA;
	u8 stat;

	if (!asus_get_prop_usb_present(smbchg_dev)) {
		asus_typec_removal_function(smbchg_dev);
		return;
	}

	//#3: Set USBIN_CURRENT_LIMIT to 50mA
	rc = asus_exclusive_vote(smbchg_dev->usb_icl_votable, ASUS_ICL_VOTER, true, 50000);
	if (rc < 0)
		CHG_DBG_E("Failed to set USBIN_CURRENT_LIMIT to 50mA\n");

	//#4: Delay 5ms
	msleep(5);

	CHG_TYPE_judge();
	CHG_DBG("ASUS_ADAPTER_ID(%s)\n", asus_id[ASUS_ADAPTER_ID]);

	//#1: USB DPDM Switch to 1D(PMIC), ADC_SW_EN-gpio101 = 0
	rc = gpio_direction_output(global_gpio->ADC_SW_EN, 0);
	if (rc) {
		CHG_DBG_E("Failed to swith to 1D, ADC_SW_EN-gpio101(%d)\n", gpio_get_value(global_gpio->ADC_SW_EN));
	} else {
		CHG_DBG("USB DPDM swith to 1D, ADC_SW_EN-gpio101(%d) = 0\n", gpio_get_value(global_gpio->ADC_SW_EN));
	}

	//#2: Pull-low 1M ohm, ADCPWREN_PMI_GP1-gpio120 = 0
	rc = gpio_direction_output(global_gpio->ADCPWREN_PMI_GP1, 0);
	if (rc) {
		CHG_DBG_E("Failed to pull-low 1M ohm, ADCPWREN_PMI_GP1-gpio120(%d)\n", gpio_get_value(global_gpio->ADCPWREN_PMI_GP1));
	} else {
		CHG_DBG("Pull-low 1M ohm, ADCPWREN_PMI_GP1-gpio120(%d) = 0\n", gpio_get_value(global_gpio->ADCPWREN_PMI_GP1));
	}

	switch (ASUS_ADAPTER_ID) {
	case ASUS_750K:
	case PB:
		if (HVDCP_FLAG == 0) {
			if (ASUS_ADAPTER_ID == ASUS_750K)
				asus_CHG_TYPE = 750;
			if (LEGACY_CABLE_FLAG || UFP_FLAG == 1)
				usb_max_current = ICL_2000mA;
			else
				usb_max_current = ICL_500mA;
		} else if (HVDCP_FLAG == 2)
			usb_max_current = ICL_1000mA;
		else
			usb_max_current = ICL_1500mA;
		break;
	case ASUS_200K:
		if (HVDCP_FLAG == 0) {
			if (LEGACY_CABLE_FLAG || UFP_FLAG == 1)
				usb_max_current = ICL_1000mA;
			else
				usb_max_current = ICL_500mA;
		} else if (HVDCP_FLAG == 2)
			usb_max_current = ICL_1000mA;
		else {
			asus_CHG_TYPE = 200;
			usb_max_current = ICL_1750mA;
		}
		break;
	case OTHERS:
		if (HVDCP_FLAG == 0) {
			if (UFP_FLAG == 3 && LEGACY_CABLE_FLAG == 0)
				usb_max_current = ICL_2000mA;
			else if (UFP_FLAG == 2 && LEGACY_CABLE_FLAG == 0)
				usb_max_current = ICL_1500mA;
			else
				usb_max_current = ICL_1000mA;
		} else if (HVDCP_FLAG == 2)
			usb_max_current = ICL_1000mA;
		else
			usb_max_current = ICL_1500mA;
		break;
	case ADC_NOT_READY:
		usb_max_current = ICL_1000mA;
		break;
	}

	if(LEGACY_CABLE_FLAG == 0 && UFP_FLAG == 3 && HVDCP_FLAG == 0){
		usb_max_current = ICL_2000mA;
	}
	else if(LEGACY_CABLE_FLAG == 0 && UFP_FLAG == 2 && HVDCP_FLAG == 0){
		usb_max_current = ICL_1500mA;
	}

	// if ICL is 50mA more than 5 senconds will cause some PB shutdown automatically, so set ICL to 500mA here
	rc = asus_exclusive_vote(smbchg_dev->usb_icl_votable, ASUS_ICL_VOTER, true, 500000);
	if (rc < 0)
		CHG_DBG_E("Failed to set USBIN_CURRENT_LIMIT to 500mA\n");
		
	//#1: HVDCP_PULSE_COUNT_MAX = [7:6] = 0x01(QC2_9V) & [5:0] = 0x14(QC3 PULSE COUNT=9V)
	rc = smblib_write(smbchg_dev, HVDCP_PULSE_COUNT_MAX_REG, 0x54);
	if (rc < 0)
		CHG_DBG_E("Failed to set HVDCP_PULSE_COUNT_MAX_REG\n");

	//#2: USBIN_OPTIONS1_CFG = Enable BC1.2 & Enable HVDCP
	smblib_apsd_enable(smbchg_dev, true);
	rc = smblib_masked_write(smbchg_dev, USBIN_OPTIONS_1_CFG_REG, HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0) {
		CHG_DBG_E("Couldn't set HVDCP_EN_BIT rc=%d\n", rc);
	}

	//#3: Re-run APSD
	CHG_DBG("Rerun APSD 2nd\n");
	rc = smblib_masked_write(smbchg_dev, CMD_APSD_REG, APSD_RERUN_BIT, APSD_RERUN_BIT);
	if (rc < 0)
		CHG_DBG_E("Failed to set CMD_APSD_REG\n");

	msleep(1000);

//Set current:
	CHG_DBG("ASUS_ADAPTER_ID = %s, set ICL = %duA(0x%x), HVDCP_FLAG = %d, UFP_FLAG = %s, LEGACY_CABLE_FLAG = %d\n",
			asus_id[ASUS_ADAPTER_ID], (int)usb_max_current*50000, usb_max_current, HVDCP_FLAG, ufp_type[UFP_FLAG], LEGACY_CABLE_FLAG);
	asus_set_icl = usb_max_current;
	rc = asus_exclusive_vote(smbchg_dev->usb_icl_votable, ASUS_ICL_VOTER, true, (int)usb_max_current*50000);
	//rc = smblib_masked_write(smbchg_dev, USBIN_CURRENT_LIMIT_CFG_REG, USBIN_CURRENT_LIMIT_MASK, usb_max_current);
	if (rc < 0)
		CHG_DBG_E("Failed to set ICL\n");

	//#1: 0x1365[4] = 1, Use SW to control Input Current Limit after APSD is completed
	rc = smblib_read(smbchg_dev, USBIN_LOAD_CFG_REG, &stat);
	if (rc < 0)
		CHG_DBG_E("Couldn't read USBIN_LOAD_CFG_REG rc=%d\n", rc);
	else
		CHG_DBG("Check setting, #1a: USBIN_LOAD_CFG_REG 0x1365 = 0x%x\n", stat);

	rc = smblib_masked_write(smbchg_dev, USBIN_LOAD_CFG_REG,
			ICL_OVERRIDE_AFTER_APSD_BIT, ICL_OVERRIDE_AFTER_APSD_BIT);
	if (rc < 0) {
		dev_err(smbchg_dev->dev, "Couldn't set default USBIN_LOAD_CFG_REG rc=%d\n", rc);
	}

	rc = smblib_read(smbchg_dev, USBIN_LOAD_CFG_REG, &stat);
	if (rc < 0)
		CHG_DBG_E("Couldn't read USBIN_LOAD_CFG_REG rc=%d\n", rc);
	else
		CHG_DBG("Check setting, #1b: USBIN_LOAD_CFG_REG 0x1365 = 0x%x\n", stat);


	//#2: 0x1342[0] = 1, Override ICL BC1.2 / Type-C APSD result with Command Register USBIN_CURRENT_LIMIT_CFG(0x1370)
	rc = smblib_read(smbchg_dev, CMD_ICL_OVERRIDE_REG, &stat);
	if (rc < 0)
		CHG_DBG_E("Couldn't read CMD_ICL_OVERRIDE_REG rc=%d\n", rc);
	else
		CHG_DBG("Check setting, #2a: CMD_ICL_OVERRIDE_REG 0x1342 = 0x%x\n", stat);

	rc = smblib_masked_write(smbchg_dev, CMD_ICL_OVERRIDE_REG,
			ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
	if (rc < 0) {
		dev_err(smbchg_dev->dev, "Couldn't set default CMD_ICL_OVERRIDE_REG rc=%d\n", rc);
	}

	rc = smblib_read(smbchg_dev, CMD_ICL_OVERRIDE_REG, &stat);
	if (rc < 0)
		CHG_DBG_E("Couldn't read CMD_ICL_OVERRIDE_REG rc=%d\n", rc);
	else
		CHG_DBG("Check setting, #2b: CMD_ICL_OVERRIDE_REG 0x1342 = 0x%x\n", stat);


	//#3: 0x1360 = 0x08, USBIN_ADAPTER_ALLOW_5V_TO_9V
	rc = smblib_read(smbchg_dev, USBIN_ADAPTER_ALLOW_CFG_REG, &stat);
	if (rc < 0)
		CHG_DBG_E("Couldn't read USBIN_ADAPTER_ALLOW_CFG_REG rc=%d\n", rc);
	else
		CHG_DBG("Check setting, #3a: USBIN_ADAPTER_ALLOW_CFG_REG 0x1360 = 0x%x\n", stat);

	rc = smblib_set_adapter_allowance(smbchg_dev, USBIN_ADAPTER_ALLOW_5V_TO_9V);
	if (rc < 0) {
		dev_err(smbchg_dev->dev, "Couldn't set default USBIN_ADAPTER_ALLOW_CFG_REG rc=%d\n", rc);
	}

	rc = smblib_read(smbchg_dev, USBIN_ADAPTER_ALLOW_CFG_REG, &stat);
	if (rc < 0)
		CHG_DBG_E("Couldn't read USBIN_ADAPTER_ALLOW_CFG_REG rc=%d\n", rc);
	else
		CHG_DBG("Check setting, #3b: USBIN_ADAPTER_ALLOW_CFG_REG 0x1360 = 0x%x\n", stat);


	asus_adapter_detecting_flag = 0;
	smblib_asus_monitor_start(smbchg_dev, 0);  //ASUS BSP : Jeita start
}
//[---]ASUS : Add ASUS Adapter Detecting

void asus_insertion_initial_settings(struct smb_charger *chg)
{
	int rc;

//#1: 0x1060 = 0x06, PCC_400mA
	rc = smblib_masked_write(chg, PRE_CHARGE_CURRENT_CFG_REG,
			PRE_CHARGE_CURRENT_SETTING_MASK, 0x06);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default PRE_CHARGE_CURRENT_CFG_REG rc=%d\n", rc);
	}

//#2: 0x1061 = 0x2E, FCC_2300mA
	rc = smblib_write(chg, CHGR_FAST_CHARGE_CURRENT_CFG_REG, 0x2E);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default CHGR_FAST_CHARGE_CURRENT_CFG_REG rc=%d\n", rc);
	}

//#3: 0x1070 = 0x4C, FV_4p36V
	rc = smblib_write(chg, CHGR_FLOAT_VOLTAGE_CFG_REG, 0x4C);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default CHGR_FLOAT_VOLTAGE_CFG_REG rc=%d\n", rc);
	}

//#4-1: 0x107E = 0x55, recharge threshold = 4.26V
	rc = smblib_write(chg, CHGR_ADC_RECHARGE_THRESHOLD_MSB_REG, 0x55);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default CHGR_ADC_RECHARGE_THRESHOLD_MSB_REG rc=%d\n", rc);
	}

//#4-2: 0x107F = 0x83, recharge threshold = 4.26V
	rc = smblib_write(chg, CHGR_ADC_RECHARGE_THRESHOLD_LSB_REG, 0x83);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default CHGR_ADC_RECHARGE_THRESHOLD_LSB_REG rc=%d\n", rc);
	}

//#4-3: 0x1051[1] = 0, HW recharge
	rc = smblib_masked_write(chg, CHGR_CFG2_REG, SOC_BASED, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default CHGR_CFG2_REG rc=%d\n", rc);
	}

//#5: 0x1366 = 0x02, USB_2P0_SEL, USB_500_MODE, USB_100_OR_500_MODE
	rc = smblib_write(chg, USBIN_ICL_OPTIONS_REG, 0x02);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default USBIN_ICL_OPTIONS_REG rc=%d\n", rc);
	}

//#6-1: 0x1067 = 0xFE, termination current = 150mA
	rc = smblib_write(chg, CHGR_ADC_ITERM_UP_THD_MSB_REG, 0xFE);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default CHGR_ADC_ITERM_UP_THD_MSB_REG rc=%d\n", rc);
	}

//#6-2: 0x1068 = 0x15, termination current = 150mA
	rc = smblib_write(chg, CHGR_ADC_ITERM_UP_THD_LSB_REG, 0x15);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default CHGR_ADC_ITERM_UP_THD_LSB_REG rc=%d\n", rc);
	}

//#7: 0x1360 = 0x08, USBIN_ADAPTER_ALLOW_5V_TO_9V
	rc = smblib_set_adapter_allowance(chg, USBIN_ADAPTER_ALLOW_5V_TO_9V);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default USBIN_ADAPTER_ALLOW_CFG_REG rc=%d\n", rc);
	}

/*
//#8: 0x1362[3:2] = 11, AUTO_SRC_DETECT_ENABLED, HVDCP_ENABLE, done by QCOM smblib_apsd_enable and smblib_hvdcp_detect_enable
	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
			HVDCP_EN_BIT | BC1P2_SRC_DETECT_BIT, HVDCP_EN_BIT | BC1P2_SRC_DETECT_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default USBIN_OPTIONS_1_CFG_REG rc=%d\n", rc);
	}
*/

//#9: 0x1051[0] = 0, CHG_INHIBIT_DIS
	rc = smblib_masked_write(chg, CHGR_CFG2_REG,
			CHARGER_INHIBIT_BIT, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default CHGR_CFG2_REG rc=%d\n", rc);
	}

//#10: 0x1042[0] = 0, CHARGING_DISABLED
	rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,  //Disabled = 0
			CHARGING_ENABLE_CMD_BIT, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default CHARGING_ENABLE_CMD_REG rc=%d\n", rc);
	}

//#11: 0x1042[0] = 1, CHARGING_ENABLED
	rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,  //Enabled = 1
			CHARGING_ENABLE_CMD_BIT, CHARGING_ENABLE_CMD_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default CHARGING_ENABLE_CMD_REG rc=%d\n", rc);
	}

//#12: 0x1183 = 0x05, VSYS_MIN_3P2V
	rc = smblib_write(chg, 0x1183, 0x05);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default DCDC_VSYSMIN_CFG rc=%d\n", rc);
	}

//#16: 0x1365[4] = 1, Use SW to control Input Current Limit after APSD is completed
	rc = smblib_masked_write(chg, USBIN_LOAD_CFG_REG,
			ICL_OVERRIDE_AFTER_APSD_BIT, ICL_OVERRIDE_AFTER_APSD_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default USBIN_LOAD_CFG_REG rc=%d\n", rc);
	}

//#17: 0x1342[0] = 1, Override ICL BC1.2 / Type-C APSD result with Command Register USBIN_CURRENT_LIMIT_CFG(0x1370)
	rc = smblib_masked_write(chg, CMD_ICL_OVERRIDE_REG,
			ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default CMD_ICL_OVERRIDE_REG rc=%d\n", rc);
	}

//#19: 0x155A = 0x01, LEGACY_CABLE_DET_WINDOW = 9ms
	rc = smblib_write(chg, 0x155A, 0x01);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set default LEGACY_CABLE_DET_WINDOW rc=%d\n", rc);
	}
}

void asus_set_flow_flag_work(struct work_struct *work)
{
	//[+++]ASUS : Check the VBUS status again before runing the work
	if (!asus_get_prop_usb_present(smbchg_dev)) {
		CHG_DBG_E("Try to run %s, but VBUS_IN is low\n", __func__);
		return;
	}
	//[---]ASUS : Check the VBUS status again before runing the work

	if (asus_flow_processing)
		asus_adapter_detecting_flag = 1;
}

void asus_reverse_charge_work(struct work_struct *work)
{
	int rc;
	int batt_current;
	int batt_capacity;
	
	CHG_DBG("+++\n");
	
	batt_current = asus_get_prop_batt_current(smbchg_dev);
	batt_capacity = asus_get_prop_batt_capacity(smbchg_dev);
	
	if ((batt_current <= 3300000 && batt_capacity > 30) && !cam_sensor_is_power_up()) {
		schedule_delayed_work(&smbchg_dev->asus_reverse_charge_work, msecs_to_jiffies(1000));
	}
	else {
		CHG_DBG("batt_current = %d, batt_capacity = %d, set otg current limit to 1A\n", batt_current, batt_capacity);
		
		//0x1152 = 0x1, OTG_ILIMIT_1000MA
		rc = smblib_masked_write(smbchg_dev, DCDC_OTG_CURRENT_LIMIT_CFG_REG,
			DCDC_OTG_CURRENT_LIMIT_MASK, 0x1);
		if (rc < 0) {
			CHG_DBG_E("Couldn't set default DCDC_OTG_CURRENT_LIMIT_CFG_REG rc=%d\n", rc);
		}
		
		default_src_caps[0] = 0x36019032;
		chg_set_src_cap();
	}
}

void asus_reverse_charge_check_camera(struct work_struct *work)
{
	int batt_capacity;
	
	CHG_DBG("+++\n");
	
	batt_capacity = asus_get_prop_batt_capacity(smbchg_dev);
	
	
	if(batt_capacity <= 15){
		cam_flash_battery_low(1);
		asus_extcon_set_state_sync(smbchg_dev->reversechg_extcon, 1);
	}
	else{
		schedule_delayed_work(&smbchg_dev->asus_reverse_charge_check_camera, msecs_to_jiffies(1000));
	}
}

extern int disable_inov_flag;
void asus_enable_inov(int enable)
{
	int rc;
	
	if(disable_inov_flag){
		enable = 0;
	}
	
	if(enable) {
		//inov setting: 0x1670 = 0x05, enable INOV
		rc = smblib_write(smbchg_dev, MISC_THERMREG_SRC_CFG_REG, 0x05);
		if (rc < 0) {
			dev_err(smbchg_dev->dev, "Couldn't set default MISC_THERMREG_SRC_CFG_REG rc=%d\n", rc);
		}
		CHG_DBG("enable inov\n");
	}
	else{
		//inov setting: 0x1670 = 0x0, disable INOV
		rc = smblib_write(smbchg_dev, MISC_THERMREG_SRC_CFG_REG, 0x0);
		if (rc < 0) {
			dev_err(smbchg_dev->dev, "Couldn't set default MISC_THERMREG_SRC_CFG_REG rc=%d\n", rc);
		}
		CHG_DBG("disable inov\n");
	}
}

void asus_enable_inov_work(struct work_struct *work)
{
	int usb_present;
	
	usb_present = asus_get_prop_usb_present(smbchg_dev);
	asus_enable_inov(usb_present);
}
/**********************
 * INTERRUPT HANDLERS *
 **********************/

irqreturn_t default_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	return IRQ_HANDLED;
}

irqreturn_t sdam_sts_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	mutex_lock(&chg->irq_status_lock);
	chg->irq_status |= PULSE_SKIP_IRQ_BIT;
	mutex_unlock(&chg->irq_status_lock);

	power_supply_changed(chg->usb_main_psy);
	return IRQ_HANDLED;
}

#define CHG_TERM_WA_ENTRY_DELAY_MS		300000		/* 5 min */
#define CHG_TERM_WA_EXIT_DELAY_MS		60000		/* 1 min */
static void smblib_eval_chg_termination(struct smb_charger *chg, u8 batt_status)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_REAL_CAPACITY, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SOC value, rc=%d\n", rc);
		return;
	}

	/*
	 * Post charge termination, switch to BSM mode triggers the risk of
	 * over charging as BATFET opening may take some time post the necessity
	 * of staying in supplemental mode, leading to unintended charging of
	 * battery. Trigger the charge termination WA once charging is completed
	 * to prevent overcharing.
	 */
	if ((batt_status == TERMINATE_CHARGE) && (pval.intval == 100)) {
		chg->cc_soc_ref = 0;
		chg->last_cc_soc = 0;
		alarm_start_relative(&chg->chg_termination_alarm,
				ms_to_ktime(CHG_TERM_WA_ENTRY_DELAY_MS));
	} else if (pval.intval < 100) {
		/*
		 * Reset CC_SOC reference value for charge termination WA once
		 * we exit the TERMINATE_CHARGE state and soc drops below 100%
		 */
		chg->cc_soc_ref = 0;
		chg->last_cc_soc = 0;
	}
}

irqreturn_t chg_state_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
				rc);
		return IRQ_HANDLED;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (chg->wa_flags & CHG_TERMINATION_WA)
		smblib_eval_chg_termination(chg, stat);

	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t batt_temp_changed_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->jeita_configured != JEITA_CFG_COMPLETE)
		return IRQ_HANDLED;

	rc = smblib_soft_jeita_arb_wa(chg);
	if (rc < 0) {
		smblib_err(chg, "Couldn't fix soft jeita arb rc=%d\n",
				rc);
		return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

irqreturn_t batt_psy_changed_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t usbin_uv_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct storm_watch *wdata;
	const struct apsd_result *apsd = smblib_get_apsd_result(chg);
	int rc;
	u8 stat = 0, max_pulses = 0;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	if (!chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data)
		return IRQ_HANDLED;

	wdata = &chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data->storm_data;
	reset_storm_count(wdata);

	/* Workaround for non-QC2.0-compliant chargers follows */
	if (!chg->qc2_unsupported_voltage &&
			apsd->pst == POWER_SUPPLY_TYPE_USB_HVDCP) {
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't read CHANGE_STATUS_REG rc=%d\n", rc);

		if (stat & QC_5V_BIT)
			return IRQ_HANDLED;

		rc = smblib_read(chg, HVDCP_PULSE_COUNT_MAX_REG, &max_pulses);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't read QC2 max pulses rc=%d\n", rc);

		chg->qc2_max_pulses = (max_pulses &
				HVDCP_PULSE_COUNT_MAX_QC2_MASK);

		if (stat & QC_12V_BIT) {
			chg->qc2_unsupported_voltage = QC2_NON_COMPLIANT_12V;
			rc = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
					HVDCP_PULSE_COUNT_MAX_QC2_MASK,
					HVDCP_PULSE_COUNT_MAX_QC2_9V);
			if (rc < 0)
				smblib_err(chg, "Couldn't force max pulses to 9V rc=%d\n",
						rc);

		} else if (stat & QC_9V_BIT) {
			chg->qc2_unsupported_voltage = QC2_NON_COMPLIANT_9V;
			rc = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
					HVDCP_PULSE_COUNT_MAX_QC2_MASK,
					HVDCP_PULSE_COUNT_MAX_QC2_5V);
			if (rc < 0)
				smblib_err(chg, "Couldn't force max pulses to 5V rc=%d\n",
						rc);

		}

		rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
				SUSPEND_ON_COLLAPSE_USBIN_BIT,
				0);
		if (rc < 0)
			smblib_err(chg, "Couldn't turn off SUSPEND_ON_COLLAPSE_USBIN_BIT rc=%d\n",
					rc);

		smblib_rerun_apsd(chg);
	}

	return IRQ_HANDLED;
}

#define USB_WEAK_INPUT_UA	1400000
#define ICL_CHANGE_DELAY_MS	1000
irqreturn_t icl_change_irq_handler(int irq, void *data)
{
	u8 stat;
	int rc, settled_ua, delay = ICL_CHANGE_DELAY_MS;
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	if (chg->mode == PARALLEL_MASTER) {
		/*
		 * Ignore if change in ICL is due to DIE temp mitigation.
		 * This is to prevent any further ICL split.
		 */
		if (chg->hw_die_temp_mitigation) {
			rc = smblib_read(chg, DIE_TEMP_STATUS_REG, &stat);
			if (rc < 0) {
				smblib_err(chg,
					"Couldn't read DIE_TEMP rc=%d\n", rc);
				return IRQ_HANDLED;
			}
			if (stat & (DIE_TEMP_UB_BIT | DIE_TEMP_LB_BIT)) {
				smblib_dbg(chg, PR_PARALLEL,
					"skip ICL change DIE_TEMP %x\n", stat);
				return IRQ_HANDLED;
			}
		}

		rc = smblib_read(chg, AICL_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n",
					rc);
			return IRQ_HANDLED;
		}

		rc = smblib_get_charge_param(chg, &chg->param.icl_stat,
					&settled_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
			return IRQ_HANDLED;
		}

		/* If AICL settled then schedule work now */
		if (settled_ua == get_effective_result(chg->usb_icl_votable))
			delay = 0;

		cancel_delayed_work_sync(&chg->icl_change_work);
		schedule_delayed_work(&chg->icl_change_work,
						msecs_to_jiffies(delay));
	}

	return IRQ_HANDLED;
}

static void smblib_micro_usb_plugin(struct smb_charger *chg, bool vbus_rising)
{
	if (!vbus_rising) {
		smblib_update_usb_type(chg);
		smblib_notify_device_mode(chg, false);
		smblib_uusb_removal(chg);
	}
}

void smblib_usb_plugin_hard_reset_locked(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	bool vbus_rising;
	struct smb_irq_data *data;
	struct storm_watch *wdata;

	CHG_DBG("+++\n");

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	if (vbus_rising) {
		/* Remove FCC_STEPPER 1.5A init vote to allow FCC ramp up */
		if (chg->fcc_stepper_enable)
			vote(chg->fcc_votable, FCC_STEPPER_VOTER, false, 0);
	} else {
		if (chg->wa_flags & BOOST_BACK_WA) {
			data = chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data;
			if (data) {
				wdata = &data->storm_data;
				update_storm_count(wdata,
						WEAK_CHG_STORM_COUNT);
				vote(chg->usb_icl_votable, BOOST_BACK_VOTER,
						false, 0);
				vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
						false, 0);
			}
		}

		/* Force 1500mA FCC on USB removal if fcc stepper is enabled */
		if (chg->fcc_stepper_enable)
			vote(chg->fcc_votable, FCC_STEPPER_VOTER,
							true, 1500000);
	}

//[+++]ASUS BSP charger
	CHG_DBG("vbus_rising = %d\n", vbus_rising);

	if (vbus_rising) {
		vbus_rising_count ++;
		asus_enable_inov(0);
		if (!asus_flow_processing) {
			asus_flow_processing = 1;
			schedule_delayed_work(&smbchg_dev->asus_set_flow_flag_work, msecs_to_jiffies(2000));
			asus_insertion_initial_settings(smbchg_dev);
			asus_smblib_stay_awake(smbchg_dev);
			if (g_Charger_mode)
				schedule_delayed_work(&smbchg_dev->asus_chg_flow_work, msecs_to_jiffies(13500));
			else
				schedule_delayed_work(&smbchg_dev->asus_chg_flow_work, msecs_to_jiffies(13000));
		}
	} else {
		rc = asus_exclusive_vote(smbchg_dev->usb_icl_votable, ASUS_ICL_VOTER, false, 0);
		if (rc < 0)
			CHG_DBG_E("Failed to disable USBIN_CURRENT_LIMIT\n");

		//#1: Write the ICL to the default 500mA, 0x1370 = 0x0A
		//Do here, avoid to be overrided by other function
		//Don't use Voter to set the ICL for keeping min ICL at 500mA. It would result in inpredicted ICL voting result
		//rc = asus_exclusive_vote(smbchg_dev->usb_icl_votable, ASUS_ICL_VOTER, true, 500000);
		rc = smblib_masked_write(smbchg_dev, USBIN_CURRENT_LIMIT_CFG_REG, USBIN_CURRENT_LIMIT_MASK, 0xA);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't set default USBIN_CURRENT_LIMIT_CFG_REG rc=%d\n", rc);
		}

		//#2: Write USB_500_MODE, 0x1366 = 0x02, USB_2P0_SEL, USB_500_MODE, USB_100_OR_500_MODE
		rc = smblib_write(smbchg_dev, USBIN_ICL_OPTIONS_REG, 0x02);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't set default USBIN_ICL_OPTIONS_REG rc=%d\n", rc);
		}

		//#3: USB DPDM Switch to 1D(PMIC), ADC_SW_EN-gpio101 = 0
		rc = gpio_direction_output(global_gpio->ADC_SW_EN, 0);
		if (rc) {
			CHG_DBG_E("Failed to swith to 1D, ADC_SW_EN-gpio101(%d)\n", gpio_get_value(global_gpio->ADC_SW_EN));
		} else {
			CHG_DBG("USB DPDM swith to 1D, ADC_SW_EN-gpio101(%d) = 0\n", gpio_get_value(global_gpio->ADC_SW_EN));
		}

		vbus_rising_count = 0;
		asus_typec_removal_function(smbchg_dev);
		cancel_delayed_work(&chg->asus_enable_inov_work);
		asus_enable_inov(0);
	}

	if (usb_thermal_once_flag) {
		if (cancel_delayed_work(&smbchg_dev->asus_usb_thermal_work))
			schedule_delayed_work(&smbchg_dev->asus_usb_thermal_work, msecs_to_jiffies(0));
	}
//[---]ASUS BSP charger

	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: usbin-plugin %s\n",
					vbus_rising ? "attached" : "detached");

	CHG_DBG("---\n");
}

#define PL_DELAY_MS	30000
void smblib_usb_plugin_locked(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	bool vbus_rising;
	struct smb_irq_data *data;
	struct storm_watch *wdata;

	CHG_DBG("+++\n");

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	smblib_set_opt_switcher_freq(chg, vbus_rising ? chg->chg_freq.freq_5V :
						chg->chg_freq.freq_removal);

	if (vbus_rising) {
		cancel_delayed_work_sync(&chg->pr_swap_detach_work);
		vote(chg->awake_votable, DETACH_DETECT_VOTER, false, 0);
		rc = smblib_request_dpdm(chg, true);
		if (rc < 0)
			smblib_err(chg, "Couldn't to enable DPDM rc=%d\n", rc);

		/* Enable SW Thermal regulation */
		rc = smblib_set_sw_thermal_regulation(chg, true);
		if (rc < 0)
			smblib_err(chg, "Couldn't start SW thermal regulation WA, rc=%d\n",
				rc);

		/* Remove FCC_STEPPER 1.5A init vote to allow FCC ramp up */
		if (chg->fcc_stepper_enable)
			vote(chg->fcc_votable, FCC_STEPPER_VOTER, false, 0);

		/* Schedule work to enable parallel charger */
		vote(chg->awake_votable, PL_DELAY_VOTER, true, 0);
		schedule_delayed_work(&chg->pl_enable_work,
					msecs_to_jiffies(PL_DELAY_MS));
	} else {
		/* Disable SW Thermal Regulation */
		rc = smblib_set_sw_thermal_regulation(chg, false);
		if (rc < 0)
			smblib_err(chg, "Couldn't stop SW thermal regulation WA, rc=%d\n",
				rc);

		if (chg->wa_flags & BOOST_BACK_WA) {
			data = chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data;
			if (data) {
				wdata = &data->storm_data;
				update_storm_count(wdata,
						WEAK_CHG_STORM_COUNT);
				vote(chg->usb_icl_votable, BOOST_BACK_VOTER,
						false, 0);
				vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
						false, 0);
			}
		}

		/* Force 1500mA FCC on removal if fcc stepper is enabled */
		if (chg->fcc_stepper_enable)
			vote(chg->fcc_votable, FCC_STEPPER_VOTER,
							true, 1500000);

		if (chg->wa_flags & WEAK_ADAPTER_WA) {
			chg->aicl_5v_threshold_mv =
					chg->default_aicl_5v_threshold_mv;
			chg->aicl_cont_threshold_mv =
					chg->default_aicl_cont_threshold_mv;

			smblib_set_charge_param(chg,
					&chg->param.aicl_5v_threshold,
					chg->aicl_5v_threshold_mv);
			smblib_set_charge_param(chg,
					&chg->param.aicl_cont_threshold,
					chg->aicl_cont_threshold_mv);
			chg->aicl_max_reached = false;

			if (chg->smb_version == PMI632_SUBTYPE)
				schgm_flash_torch_priority(chg,
						TORCH_BUCK_MODE);

			data = chg->irq_info[USBIN_UV_IRQ].irq_data;
			if (data) {
				wdata = &data->storm_data;
				reset_storm_count(wdata);
			}
			vote(chg->usb_icl_votable, AICL_THRESHOLD_VOTER,
					false, 0);
		}

		rc = smblib_request_dpdm(chg, false);
		if (rc < 0)
			smblib_err(chg, "Couldn't disable DPDM rc=%d\n", rc);

		smblib_update_usb_type(chg);
	}

//[+++]ASUS BSP charger
	CHG_DBG("vbus_rising = %d\n", vbus_rising);

	if (vbus_rising) {
		vbus_rising_count ++;
		asus_enable_inov(0);
		if (!asus_flow_processing) {
			asus_flow_processing = 1;
			schedule_delayed_work(&smbchg_dev->asus_set_flow_flag_work, msecs_to_jiffies(2000));
			asus_insertion_initial_settings(smbchg_dev);
			asus_smblib_stay_awake(smbchg_dev);
			if (g_Charger_mode)
				schedule_delayed_work(&smbchg_dev->asus_chg_flow_work, msecs_to_jiffies(13500));
			else
				schedule_delayed_work(&smbchg_dev->asus_chg_flow_work, msecs_to_jiffies(13000));
		}
	} else {
		rc = asus_exclusive_vote(smbchg_dev->usb_icl_votable, ASUS_ICL_VOTER, false, 0);
		if (rc < 0)
			CHG_DBG_E("Failed to disable USBIN_CURRENT_LIMIT\n");

		//#1: Write the ICL to the default 500mA, 0x1370 = 0x0A
		//Do here, avoid to be overrided by other function
		//Don't use Voter to set the ICL for keeping min ICL at 500mA. It would result in inpredicted ICL voting result
		//rc = asus_exclusive_vote(smbchg_dev->usb_icl_votable, ASUS_ICL_VOTER, true, 500000);
		rc = smblib_masked_write(smbchg_dev, USBIN_CURRENT_LIMIT_CFG_REG, USBIN_CURRENT_LIMIT_MASK, 0xA);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't set default USBIN_CURRENT_LIMIT_CFG_REG rc=%d\n", rc);
		}

		//#2: Write USB_500_MODE, 0x1366 = 0x02, USB_2P0_SEL, USB_500_MODE, USB_100_OR_500_MODE
		rc = smblib_write(smbchg_dev, USBIN_ICL_OPTIONS_REG, 0x02);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't set default USBIN_ICL_OPTIONS_REG rc=%d\n", rc);
		}

		//#3: USB DPDM Switch to 1D(PMIC), ADC_SW_EN-gpio101 = 0
		rc = gpio_direction_output(global_gpio->ADC_SW_EN, 0);
		if (rc) {
			CHG_DBG_E("Failed to swith to 1D, ADC_SW_EN-gpio101(%d)\n", gpio_get_value(global_gpio->ADC_SW_EN));
		} else {
			CHG_DBG("USB DPDM swith to 1D, ADC_SW_EN-gpio101(%d) = 0\n", gpio_get_value(global_gpio->ADC_SW_EN));
		}

		vbus_rising_count = 0;
		asus_typec_removal_function(smbchg_dev);
		cancel_delayed_work(&chg->asus_enable_inov_work);
		asus_enable_inov(0);
	}

	if (usb_thermal_once_flag) {
		if (cancel_delayed_work(&smbchg_dev->asus_usb_thermal_work))
			schedule_delayed_work(&smbchg_dev->asus_usb_thermal_work, msecs_to_jiffies(0));
	}
//[---]ASUS BSP charger

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		smblib_micro_usb_plugin(chg, vbus_rising);

	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: usbin-plugin %s\n",
					vbus_rising ? "attached" : "detached");

	CHG_DBG("---\n");
}

irqreturn_t usb_plugin_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	if (chg->pd_hard_reset)
		smblib_usb_plugin_hard_reset_locked(chg);
	else
		smblib_usb_plugin_locked(chg);

	return IRQ_HANDLED;
}

static void smblib_handle_slow_plugin_timeout(struct smb_charger *chg,
					      bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: slow-plugin-timeout %s\n",
		   rising ? "rising" : "falling");
	if (chg->asus_print_usb_src_change)
		CHG_DBG("IRQ: slow-plugin-timeout %s\n", rising ? "rising" : "falling");
}

static void smblib_handle_sdp_enumeration_done(struct smb_charger *chg,
					       bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: sdp-enumeration-done %s\n",
		   rising ? "rising" : "falling");
}

/* triggers when HVDCP 3.0 authentication has finished */
static void smblib_handle_hvdcp_3p0_auth_done(struct smb_charger *chg,
					      bool rising)
{
	const struct apsd_result *apsd_result;
	int rc;

	if (!rising)
		return;

	if (chg->mode == PARALLEL_MASTER)
		vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, true, 0);

	/* the APSD done handler will set the USB supply type */
	apsd_result = smblib_get_apsd_result(chg);

	/* for QC3, switch to CP if present */
	if ((apsd_result->bit & QC_3P0_BIT) && chg->sec_cp_present) {
		rc = smblib_select_sec_charger(chg, POWER_SUPPLY_CHARGER_SEC_CP,
					POWER_SUPPLY_CP_HVDCP3, false);
		if (rc < 0)
			dev_err(chg->dev,
			"Couldn't enable secondary chargers  rc=%d\n", rc);
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-3p0-auth-done rising; %s detected\n",
		   apsd_result->name);
	if (chg->asus_print_usb_src_change)
		CHG_DBG("IRQ: hvdcp-3p0-auth-done rising; %s detected\n", apsd_result->name);
}

static void smblib_handle_hvdcp_check_timeout(struct smb_charger *chg,
					      bool rising, bool qc_charger)
{
	if (rising) {

		if (qc_charger) {
			/* enable HDC and ICL irq for QC2/3 charger */
			vote(chg->limited_irq_disable_votable,
					CHARGER_TYPE_VOTER, false, 0);
			vote(chg->hdc_irq_disable_votable,
					CHARGER_TYPE_VOTER, false, 0);
			if(!asus_flow_done_flag)
				vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, 1000000);
			else
				vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, HVDCP_CURRENT_UA);
		} else {
			/* A plain DCP, enforce DCP ICL if specified */
			vote(chg->usb_icl_votable, DCP_VOTER,
				chg->dcp_icl_ua != -EINVAL, chg->dcp_icl_ua);
		}
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s %s\n", __func__,
		   rising ? "rising" : "falling");
}

/* triggers when HVDCP is detected */
static void smblib_handle_hvdcp_detect_done(struct smb_charger *chg,
					    bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-detect-done %s\n",
		   rising ? "rising" : "falling");
	if (chg->asus_print_usb_src_change)
		CHG_DBG("start, IRQ: hvdcp-detect-done %s\n", rising ? "rising" : "falling");
}

static void update_sw_icl_max(struct smb_charger *chg, int pst)
{
	int typec_mode;
	int rp_ua;

	/* while PD is active it should have complete ICL control */
	if (chg->pd_active)
		return;

	if (chg->typec_mode == POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER) {
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, 500000);
		return;
	}

	/*
	 * HVDCP 2/3, handled separately
	 */
	if (pst == POWER_SUPPLY_TYPE_USB_HVDCP
			|| pst == POWER_SUPPLY_TYPE_USB_HVDCP_3)
		return;

	/* TypeC rp med or high, use rp value */
	typec_mode = smblib_get_prop_typec_mode(chg);
	if (typec_rp_med_high(chg, typec_mode)) {
		rp_ua = get_rp_based_dcp_current(chg, typec_mode);
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, rp_ua);
		return;
	}

	/* rp-std or legacy, USB BC 1.2 */
	switch (pst) {
	case POWER_SUPPLY_TYPE_USB:
		/*
		 * USB_PSY will vote to increase the current to 500/900mA once
		 * enumeration is done.
		 */
		if (!is_client_vote_enabled(chg->usb_icl_votable,
						USB_PSY_VOTER)) {
			/* if flash is active force 500mA */
			vote(chg->usb_icl_votable, USB_PSY_VOTER, true,
					is_flash_active(chg) ?
					SDP_CURRENT_UA : SDP_100_MA);
		}
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					CDP_CURRENT_UA);
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					DCP_CURRENT_UA);
		break;
	case POWER_SUPPLY_TYPE_USB_FLOAT:
		/*
		 * limit ICL to 100mA, the USB driver will enumerate to check
		 * if this is a SDP and appropriately set the current
		 */
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					SDP_100_MA);
		break;
	case POWER_SUPPLY_TYPE_UNKNOWN:
	default:
		if (!asus_adapter_detecting_flag) {
			vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
						SDP_100_MA);
		}
		break;
	}
}

static void smblib_handle_apsd_done(struct smb_charger *chg, bool rising)
{
	const struct apsd_result *apsd_result;

	if (!rising)
		return;

	apsd_result = smblib_update_usb_type(chg);

	if (!asus_flow_done_flag)
		update_sw_icl_max(chg, apsd_result->pst);
	if (chg->asus_print_usb_src_change)
		CHG_DBG("apsd_result 0x1308 = 0x%x\n", apsd_result->bit);

	switch (apsd_result->bit) {
	case SDP_CHARGER_BIT:
	case CDP_CHARGER_BIT:
	case FLOAT_CHARGER_BIT:
		if (chg->use_extcon)
			smblib_notify_device_mode(chg, true);
		break;
	case OCP_CHARGER_BIT:
	case DCP_CHARGER_BIT:
		break;
	default:
		break;
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: apsd-done rising; %s detected\n",
		   apsd_result->name);
}

irqreturn_t usb_source_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc = 0;
	u8 stat;

	static u8 now_stat = 0x00, pre_stat = 0x00;
	pre_stat = now_stat;

	/* PD session is ongoing, ignore BC1.2 and QC detection */
	if (chg->pd_active)
		return IRQ_HANDLED;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_INTERRUPT, "APSD_STATUS = 0x%02x\n", stat);

	now_stat = stat;
	if (now_stat != pre_stat) {
		CHG_DBG("APSD_STATUS 0x1307 = 0x%x\n", stat);
		chg->asus_print_usb_src_change = true;
	} else {
		chg->asus_print_usb_src_change = false;
	}

	if ((chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		&& (stat & APSD_DTC_STATUS_DONE_BIT)
		&& !chg->uusb_apsd_rerun_done) {
		/*
		 * Force re-run APSD to handle slow insertion related
		 * charger-mis-detection.
		 */
		chg->uusb_apsd_rerun_done = true;
		smblib_rerun_apsd_if_required(chg);
		return IRQ_HANDLED;
	}

	smblib_handle_apsd_done(chg,
		(bool)(stat & APSD_DTC_STATUS_DONE_BIT));

	smblib_handle_hvdcp_detect_done(chg,
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_check_timeout(chg,
		(bool)(stat & HVDCP_CHECK_TIMEOUT_BIT),
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_3p0_auth_done(chg,
		(bool)(stat & QC_AUTH_DONE_STATUS_BIT));

	smblib_handle_sdp_enumeration_done(chg,
		(bool)(stat & ENUMERATION_DONE_BIT));

	smblib_handle_slow_plugin_timeout(chg,
		(bool)(stat & SLOW_PLUGIN_TIMEOUT_BIT));

	smblib_hvdcp_adaptive_voltage_change(chg);

	power_supply_changed(chg->usb_psy);

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_INTERRUPT, "APSD_STATUS = 0x%02x\n", stat);

	return IRQ_HANDLED;
}

enum alarmtimer_restart smblib_lpd_recheck_timer(struct alarm *alarm,
						ktime_t time)
{
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(alarm, struct smb_charger,
							lpd_recheck_timer);
	int rc;

	if (chg->lpd_reason == LPD_MOISTURE_DETECTED) {
		CHG_DBG("[usb_water] Recheck done, lpd_reason = moisture\n");
		pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		rc = smblib_set_prop_typec_power_role(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
				pval.intval, rc);
			return ALARMTIMER_NORESTART;
		}
		chg->moisture_present = false;
		power_supply_changed(chg->usb_psy);
	} else {
		CHG_DBG("[usb_water] Recheck done, lpd_reson = floating or none\n");
		rc = smblib_masked_write(chg, TYPE_C_INTERRUPT_EN_CFG_2_REG,
					TYPEC_WATER_DETECTION_INT_EN_BIT,
					TYPEC_WATER_DETECTION_INT_EN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set TYPE_C_INTERRUPT_EN_CFG_2_REG rc=%d\n",
					rc);
			return ALARMTIMER_NORESTART;
		}
	}

	chg->lpd_stage = LPD_STAGE_NONE;
	chg->lpd_reason = LPD_NONE;

	return ALARMTIMER_NORESTART;
}

#define RSBU_K_300K_UV	3000000
static bool smblib_src_lpd(struct smb_charger *chg)
{
	union power_supply_propval pval;
	bool lpd_flag = false;
	u8 stat;
	int rc;

	if (chg->lpd_disabled)
		return false;

	rc = smblib_read(chg, TYPE_C_SRC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_SRC_STATUS_REG rc=%d\n",
				rc);
		return false;
	}

	switch (stat & DETECTED_SNK_TYPE_MASK) {
	case SRC_DEBUG_ACCESS_BIT:
		if (smblib_rsbux_low(chg, RSBU_K_300K_UV))
			lpd_flag = true;
		break;
	case SRC_RD_RA_VCONN_BIT:
	case SRC_RD_OPEN_BIT:
	case AUDIO_ACCESS_RA_RA_BIT:
	default:
		break;
	}

	if (lpd_flag) {
		chg->lpd_stage = LPD_STAGE_COMMIT;
		pval.intval = POWER_SUPPLY_TYPEC_PR_SINK;
		rc = smblib_set_prop_typec_power_role(chg, &pval);
		if (rc < 0)
			smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
				pval.intval, rc);
		chg->lpd_reason = LPD_MOISTURE_DETECTED;
		chg->moisture_present =  true;
		alarm_start_relative(&chg->lpd_recheck_timer,
						ms_to_ktime(60000));
		power_supply_changed(chg->usb_psy);
	} else {
		chg->lpd_reason = LPD_NONE;
		//[+++]ASUS : Add usb water alert feature
		CHG_DBG("[usb_water] Not detect water\n");
		//[---]ASUS : Add usb water alert feature

		chg->typec_mode = smblib_get_prop_typec_mode(chg);
	}

	return lpd_flag;
}

static void typec_src_fault_condition_cfg(struct smb_charger *chg, bool src)
{
	int rc;
	u8 mask = USBIN_MID_COMP_FAULT_EN_BIT | USBIN_COLLAPSE_FAULT_EN_BIT;

	rc = smblib_masked_write(chg, OTG_FAULT_CONDITION_CFG_REG, mask,
					src ? 0 : mask);
	if (rc < 0)
		smblib_err(chg, "Couldn't write OTG_FAULT_CONDITION_CFG_REG rc=%d\n",
			rc);
}

static void typec_sink_insertion(struct smb_charger *chg)
{
	int rc;

	typec_src_fault_condition_cfg(chg, true);
	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher,
					chg->chg_freq.freq_above_otg_threshold);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_boost rc=%d\n", rc);

	if (chg->use_extcon) {
		smblib_notify_usb_host(chg, true);
		chg->otg_present = true;
	}

	if (!chg->pr_swap_in_progress)
		chg->ok_to_pd = (!(*chg->pd_disabled) || chg->early_usb_attach)
					&& !chg->pd_not_supported;
}

static void typec_src_insertion(struct smb_charger *chg)
{
	int rc = 0;
	u8 stat;

	if (chg->pr_swap_in_progress)
		return;

	rc = smblib_read(chg, LEGACY_CABLE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATE_MACHINE_STATUS_REG rc=%d\n",
			rc);
		return;
	}

	chg->typec_legacy = stat & TYPEC_LEGACY_CABLE_STATUS_BIT;
	chg->ok_to_pd = (!(chg->typec_legacy || *chg->pd_disabled)
			|| chg->early_usb_attach) && !chg->pd_not_supported;

	/* allow apsd proceed to detect QC2/3 */
	if (!chg->ok_to_pd)
		smblib_hvdcp_detect_enable(chg, true);
}

static void typec_ra_ra_insertion(struct smb_charger *chg)
{
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, 500000);
	vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	chg->ok_to_pd = false;
	smblib_hvdcp_detect_enable(chg, true);
}

static void typec_sink_removal(struct smb_charger *chg)
{
	int rc;

	typec_src_fault_condition_cfg(chg, false);
	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher,
					chg->chg_freq.freq_removal);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_removal rc=%d\n", rc);

	if (chg->use_extcon) {
		if (chg->otg_present)
			smblib_notify_usb_host(chg, false);
		chg->otg_present = false;
	}
}

static void typec_src_removal(struct smb_charger *chg)
{
	int rc;
	struct smb_irq_data *data;
	struct storm_watch *wdata;
	int sec_charger;

	CHG_DBG("start\n");

	sec_charger = chg->sec_pl_present ? POWER_SUPPLY_CHARGER_SEC_PL :
				POWER_SUPPLY_CHARGER_SEC_NONE;

	rc = smblib_select_sec_charger(chg, sec_charger, POWER_SUPPLY_CP_NONE,
					false);
	if (rc < 0)
		dev_err(chg->dev,
			"Couldn't disable secondary charger rc=%d\n", rc);

	typec_src_fault_condition_cfg(chg, false);
	smblib_hvdcp_detect_enable(chg, false);
	smblib_update_usb_type(chg);

	if (chg->wa_flags & BOOST_BACK_WA) {
		data = chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data;
		if (data) {
			wdata = &data->storm_data;
			update_storm_count(wdata, WEAK_CHG_STORM_COUNT);
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
			vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
					false, 0);
		}
	}

	cancel_delayed_work_sync(&chg->pl_enable_work);

	/* reset input current limit voters */
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
			is_flash_active(chg) ? SDP_CURRENT_UA : SDP_100_MA);
	vote(chg->usb_icl_votable, PD_VOTER, false, 0);
	vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	vote(chg->usb_icl_votable, SW_QC3_VOTER, false, 0);
	vote(chg->usb_icl_votable, CTM_VOTER, false, 0);
	vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER, false, 0);
	vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
	vote(chg->usb_icl_votable, THERMAL_THROTTLE_VOTER, false, 0);

	/* reset usb irq voters */
	vote(chg->limited_irq_disable_votable, CHARGER_TYPE_VOTER,
			true, 0);
	vote(chg->hdc_irq_disable_votable, CHARGER_TYPE_VOTER, true, 0);
	vote(chg->hdc_irq_disable_votable, HDC_IRQ_VOTER, false, 0);

	/* reset parallel voters */
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);
	vote(chg->pl_disable_votable, PL_FCC_LOW_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);

	/* Remove SW thermal regulation WA votes */
	vote(chg->usb_icl_votable, SW_THERM_REGULATION_VOTER, false, 0);
	vote(chg->pl_disable_votable, SW_THERM_REGULATION_VOTER, false, 0);
	vote(chg->dc_suspend_votable, SW_THERM_REGULATION_VOTER, false, 0);
	if (chg->cp_disable_votable)
		vote(chg->cp_disable_votable, SW_THERM_REGULATION_VOTER,
								false, 0);

	/* reset USBOV votes and cancel work */
	cancel_delayed_work_sync(&chg->usbov_dbc_work);
	vote(chg->awake_votable, USBOV_DBC_VOTER, false, 0);
	chg->dbc_usbov = false;

	chg->pulse_cnt = 0;
	chg->usb_icl_delta_ua = 0;
	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;
	chg->usbin_forced_max_uv = 0;

	/* write back the default FLOAT charger configuration */
	rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
				(u8)FLOAT_OPTIONS_MASK, chg->float_cfg);
	if (rc < 0)
		smblib_err(chg, "Couldn't write float charger options rc=%d\n",
			rc);

	/* reconfigure allowed voltage for HVDCP */
	rc = smblib_set_adapter_allowance(chg,
			USBIN_ADAPTER_ALLOW_5V_TO_9V);
	if (rc < 0)
		smblib_err(chg, "Couldn't set USBIN_ADAPTER_ALLOW_5V_TO_9V rc=%d\n",
			rc);

	/*
	 * if non-compliant charger caused UV, restore original max pulses
	 * and turn SUSPEND_ON_COLLAPSE_USBIN_BIT back on.
	 */
	if (chg->qc2_unsupported_voltage) {
		rc = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
				HVDCP_PULSE_COUNT_MAX_QC2_MASK,
				chg->qc2_max_pulses);
		if (rc < 0)
			smblib_err(chg, "Couldn't restore max pulses rc=%d\n",
					rc);

		rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
				SUSPEND_ON_COLLAPSE_USBIN_BIT,
				SUSPEND_ON_COLLAPSE_USBIN_BIT);
		if (rc < 0)
			smblib_err(chg, "Couldn't turn on SUSPEND_ON_COLLAPSE_USBIN_BIT rc=%d\n",
					rc);

		chg->qc2_unsupported_voltage = QC2_COMPLIANT;
	}

	if (chg->use_extcon)
		smblib_notify_device_mode(chg, false);

	chg->typec_legacy = false;
}

static void typec_mode_unattached(struct smb_charger *chg)
{
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, USBIN_100MA);
}

static void smblib_handle_rp_change(struct smb_charger *chg, int typec_mode)
{
	const struct apsd_result *apsd = smblib_get_apsd_result(chg);

	/*
	 * We want the ICL vote @ 100mA for a FLOAT charger
	 * until the detection by the USB stack is complete.
	 * Ignore the Rp changes unless there is a
	 * pre-existing valid vote or FLOAT is configured for
	 * SDP current.
	 */
	if (apsd->pst == POWER_SUPPLY_TYPE_USB_FLOAT) {
		if (get_client_vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER)
					<= USBIN_100MA
			|| (chg->float_cfg & FLOAT_OPTIONS_MASK)
					== FORCE_FLOAT_SDP_CFG_BIT)
			return;
	}

	update_sw_icl_max(chg, apsd->pst);

	smblib_dbg(chg, PR_MISC, "CC change old_mode=%d new_mode=%d\n",
						chg->typec_mode, typec_mode);
}

static void smblib_lpd_launch_ra_open_work(struct smb_charger *chg)
{
	u8 stat;
	int rc;

	if (chg->lpd_disabled)
		return;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n",
			rc);
		return;
	}

	if (!(stat & TYPEC_TCCDEBOUNCE_DONE_STATUS_BIT)
			&& chg->lpd_stage == LPD_STAGE_NONE) {
		chg->lpd_stage = LPD_STAGE_FLOAT;
		cancel_delayed_work_sync(&chg->lpd_ra_open_work);
		vote(chg->awake_votable, LPD_VOTER, true, 0);
		schedule_delayed_work(&chg->lpd_ra_open_work,
						msecs_to_jiffies(300));
	}
}

irqreturn_t typec_or_rid_detection_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB) {
		if (chg->uusb_moisture_protection_enabled) {
			/*
			 * Adding pm_stay_awake as because pm_relax is called
			 * on exit path from the work routine.
			 */
			pm_stay_awake(chg->dev);
			schedule_work(&chg->moisture_protection_work);
		}

		cancel_delayed_work_sync(&chg->uusb_otg_work);
		/*
		 * Skip OTG enablement if RID interrupt triggers with moisture
		 * protection still enabled.
		 */
		if (!chg->moisture_present) {
			vote(chg->awake_votable, OTG_DELAY_VOTER, true, 0);
			smblib_dbg(chg, PR_INTERRUPT, "Scheduling OTG work\n");
			schedule_delayed_work(&chg->uusb_otg_work,
				msecs_to_jiffies(chg->otg_delay_ms));
		}

		goto out;
	}

	if (chg->pr_swap_in_progress || chg->pd_hard_reset)
		goto out;

	smblib_lpd_launch_ra_open_work(chg);

	if (chg->usb_psy)
		power_supply_changed(chg->usb_psy);

out:
	return IRQ_HANDLED;
}

irqreturn_t typec_state_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int typec_mode;

	CHG_DBG("+++\n");

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB) {
		smblib_dbg(chg, PR_INTERRUPT,
				"Ignoring for micro USB\n");
		return IRQ_HANDLED;
	}

	typec_mode = smblib_get_prop_typec_mode(chg);
	if (chg->sink_src_mode != UNATTACHED_MODE
			&& (typec_mode != chg->typec_mode))
		smblib_handle_rp_change(chg, typec_mode);
	chg->typec_mode = typec_mode;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: cc-state-change; Type-C %s detected\n",
				smblib_typec_mode_name[chg->typec_mode]);
	CHG_DBG("IRQ: cc-state-change; Type-C %s detected\n",
				smblib_typec_mode_name[chg->typec_mode]);

	power_supply_changed(chg->usb_psy);

	return IRQ_HANDLED;
}

static void smblib_lpd_clear_ra_open_work(struct smb_charger *chg)
{
	if (chg->lpd_disabled)
		return;

	cancel_delayed_work_sync(&chg->lpd_detach_work);
	chg->lpd_stage = LPD_STAGE_FLOAT_CANCEL;
	cancel_delayed_work_sync(&chg->lpd_ra_open_work);
	vote(chg->awake_votable, LPD_VOTER, false, 0);
}

irqreturn_t typec_attach_detach_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	int rc;

	CHG_DBG("IRQ: %s\n", irq_data->name);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_read(chg, TYPE_C_STATE_MACHINE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATE_MACHINE_STATUS_REG rc=%d\n",
			rc);
		return IRQ_HANDLED;
	}

	if (stat & TYPEC_ATTACH_DETACH_STATE_BIT) {

		smblib_lpd_clear_ra_open_work(chg);

		rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n",
				rc);
			return IRQ_HANDLED;
		}

		if (smblib_get_prop_dfp_mode(chg) ==
				POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER) {
			chg->sink_src_mode = AUDIO_ACCESS_MODE;
			typec_ra_ra_insertion(chg);
		} else if (stat & SNK_SRC_MODE_BIT) {
			if (smblib_src_lpd(chg))
				return IRQ_HANDLED;
			chg->sink_src_mode = SRC_MODE;
			typec_sink_insertion(chg);
		} else {
			chg->sink_src_mode = SINK_MODE;
			typec_src_insertion(chg);
		}

	} else {
		switch (chg->sink_src_mode) {
		case SRC_MODE:
			typec_sink_removal(chg);
			break;
		case SINK_MODE:
		case AUDIO_ACCESS_MODE:
			typec_src_removal(chg);
			break;
		case UNATTACHED_MODE:
		default:
			typec_mode_unattached(chg);
			break;
		}

		if (!chg->pr_swap_in_progress) {
			chg->ok_to_pd = false;
			chg->sink_src_mode = UNATTACHED_MODE;
			chg->early_usb_attach = false;
			smblib_apsd_enable(chg, true);
		}

		if (chg->lpd_stage == LPD_STAGE_FLOAT_CANCEL)
			schedule_delayed_work(&chg->lpd_detach_work,
					msecs_to_jiffies(1000));
	}

	power_supply_changed(chg->usb_psy);

	return IRQ_HANDLED;
}

irqreturn_t dc_plugin_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	union power_supply_propval pval;
	int input_present;
	bool dcin_present, vbus_present;
	int rc, wireless_vout = 0;
	int sec_charger;

	rc = smblib_get_prop_vph_voltage_now(chg, &pval);
	if (rc < 0)
		return IRQ_HANDLED;

	/* 2*VPH, with a granularity of 100mV */
	wireless_vout = ((pval.intval * 2) / 100000) * 100000;

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return IRQ_HANDLED;

	dcin_present = input_present & INPUT_PRESENT_DC;
	vbus_present = input_present & INPUT_PRESENT_USB;

	if (dcin_present) {
		if (!vbus_present && chg->sec_cp_present) {
			pval.intval = wireless_vout;
			rc = smblib_set_prop_voltage_wls_output(chg, &pval);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't set dc voltage to 2*vph  rc=%d\n",
					rc);

			rc = smblib_select_sec_charger(chg,
					POWER_SUPPLY_CHARGER_SEC_CP,
					POWER_SUPPLY_CP_WIRELESS, false);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't enable secondary chargers  rc=%d\n",
					rc);
		}
	} else {
		if (chg->cp_reason == POWER_SUPPLY_CP_WIRELESS) {
			sec_charger = chg->sec_pl_present ?
					POWER_SUPPLY_CHARGER_SEC_PL :
					POWER_SUPPLY_CHARGER_SEC_NONE;
			rc = smblib_select_sec_charger(chg, sec_charger,
					POWER_SUPPLY_CP_NONE, false);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't disable secondary charger rc=%d\n",
					rc);
		}

		vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER, false, 0);
	}

	power_supply_changed(chg->dc_psy);

	smblib_dbg(chg, PR_WLS, "dcin_present= %d, usbin_present= %d, cp_reason = %d\n",
			dcin_present, vbus_present, chg->cp_reason);

	return IRQ_HANDLED;
}

irqreturn_t high_duty_cycle_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	chg->is_hdc = true;
	/*
	 * Disable usb IRQs after the flag set and re-enable IRQs after
	 * the flag cleared in the delayed work queue, to avoid any IRQ
	 * storming during the delays
	 */
	vote(chg->hdc_irq_disable_votable, HDC_IRQ_VOTER, true, 0);

	schedule_delayed_work(&chg->clear_hdc_work, msecs_to_jiffies(60));

	return IRQ_HANDLED;
}

static void smblib_bb_removal_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						bb_removal_work.work);

	vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
	vote(chg->awake_votable, BOOST_BACK_VOTER, false, 0);
}

#define BOOST_BACK_UNVOTE_DELAY_MS		750
#define BOOST_BACK_STORM_COUNT			3
#define WEAK_CHG_STORM_COUNT			8
irqreturn_t switcher_power_ok_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct storm_watch *wdata = &irq_data->storm_data;
	int rc, usb_icl;
	u8 stat;

	if (!(chg->wa_flags & BOOST_BACK_WA))
		return IRQ_HANDLED;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	/* skip suspending input if its already suspended by some other voter */
	usb_icl = get_effective_result(chg->usb_icl_votable);
	if ((stat & USE_USBIN_BIT) && usb_icl >= 0 && usb_icl <= USBIN_25MA)
		return IRQ_HANDLED;

	if (stat & USE_DCIN_BIT)
		return IRQ_HANDLED;

	if (is_storming(&irq_data->storm_data)) {
		/* This could be a weak charger reduce ICL */
		if (!is_client_vote_enabled(chg->usb_icl_votable,
						WEAK_CHARGER_VOTER)) {
			smblib_err(chg,
				"Weak charger detected: voting %dmA ICL\n",
				*chg->weak_chg_icl_ua / 1000);
			vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
					true, *chg->weak_chg_icl_ua);
			/*
			 * reset storm data and set the storm threshold
			 * to 3 for reverse boost detection.
			 */
			update_storm_count(wdata, BOOST_BACK_STORM_COUNT);
		} else {
			smblib_err(chg,
				"Reverse boost detected: voting 0mA to suspend input\n");
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, true, 0);
			vote(chg->awake_votable, BOOST_BACK_VOTER, true, 0);
			/*
			 * Remove the boost-back vote after a delay, to avoid
			 * permanently suspending the input if the boost-back
			 * condition is unintentionally hit.
			 */
			schedule_delayed_work(&chg->bb_removal_work,
				msecs_to_jiffies(BOOST_BACK_UNVOTE_DELAY_MS));
		}
	}

	return IRQ_HANDLED;
}

irqreturn_t wdog_snarl_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->wa_flags & SW_THERM_REGULATION_WA) {
		cancel_delayed_work_sync(&chg->thermal_regulation_work);
		vote(chg->awake_votable, SW_THERM_REGULATION_VOTER, true, 0);
		schedule_delayed_work(&chg->thermal_regulation_work, 0);
	}

	if (chg->step_chg_enabled)
		power_supply_changed(chg->batt_psy);

	return IRQ_HANDLED;
}

irqreturn_t wdog_bark_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_write(chg, BARK_BITE_WDOG_PET_REG, BARK_BITE_WDOG_PET_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't pet the dog rc=%d\n", rc);

	if (chg->step_chg_enabled)
		power_supply_changed(chg->batt_psy);

	return IRQ_HANDLED;
}

static void smblib_die_rst_icl_regulate(struct smb_charger *chg)
{
	int rc;
	u8 temp;

	rc = smblib_read(chg, DIE_TEMP_STATUS_REG, &temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DIE_TEMP_STATUS_REG rc=%d\n",
				rc);
		return;
	}

	/* Regulate ICL on die temp crossing DIE_RST threshold */
	vote(chg->usb_icl_votable, DIE_TEMP_VOTER,
				temp & DIE_TEMP_RST_BIT, 500000);
}

/*
 * triggered when DIE or SKIN or CONNECTOR temperature across
 * either of the _REG_L, _REG_H, _RST, or _SHDN thresholds
 */
irqreturn_t temp_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_die_rst_icl_regulate(chg);

	return IRQ_HANDLED;
}

static void smblib_usbov_dbc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						usbov_dbc_work.work);

	smblib_dbg(chg, PR_MISC, "Resetting USBOV debounce\n");
	chg->dbc_usbov = false;
	vote(chg->awake_votable, USBOV_DBC_VOTER, false, 0);
}

#define USB_OV_DBC_PERIOD_MS		1000
irqreturn_t usbin_ov_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (!(chg->wa_flags & USBIN_OV_WA))
		return IRQ_HANDLED;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	/*
	 * On specific PMICs, OV IRQ triggers for very small duration in
	 * interim periods affecting charging status reflection. In order to
	 * differentiate between OV IRQ glitch and real OV_IRQ, add a debounce
	 * period for evaluation.
	 */
	if (stat & USBIN_OV_RT_STS_BIT) {
		chg->dbc_usbov = true;
		vote(chg->awake_votable, USBOV_DBC_VOTER, true, 0);
		schedule_delayed_work(&chg->usbov_dbc_work,
				msecs_to_jiffies(USB_OV_DBC_PERIOD_MS));
	} else {
		cancel_delayed_work_sync(&chg->usbov_dbc_work);
		chg->dbc_usbov = false;
		vote(chg->awake_votable, USBOV_DBC_VOTER, false, 0);
	}

	smblib_dbg(chg, PR_MISC, "USBOV debounce status %d\n",
				chg->dbc_usbov);
	return IRQ_HANDLED;
}

/**************
 * Additional USB PSY getters/setters
 * that call interrupt functions
 ***************/

int smblib_get_prop_pr_swap_in_progress(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->pr_swap_in_progress;
	return 0;
}

#define DETACH_DETECT_DELAY_MS 20
int smblib_set_prop_pr_swap_in_progress(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;
	u8 stat = 0, orientation;

	smblib_dbg(chg, PR_MISC, "Requested PR_SWAP %d\n", val->intval);
	chg->pr_swap_in_progress = val->intval;

	/* check for cable removal during pr_swap */
	if (!chg->pr_swap_in_progress) {
		cancel_delayed_work_sync(&chg->pr_swap_detach_work);
		vote(chg->awake_votable, DETACH_DETECT_VOTER, true, 0);
		schedule_delayed_work(&chg->pr_swap_detach_work,
				msecs_to_jiffies(DETACH_DETECT_DELAY_MS));
	}

	rc = smblib_masked_write(chg, TYPE_C_DEBOUNCE_OPTION_REG,
			REDUCE_TCCDEBOUNCE_TO_2MS_BIT,
			val->intval ? REDUCE_TCCDEBOUNCE_TO_2MS_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't set tCC debounce rc=%d\n", rc);

	rc = smblib_masked_write(chg, TYPE_C_EXIT_STATE_CFG_REG,
			BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT,
			val->intval ? BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't set exit state cfg rc=%d\n", rc);

	if (chg->pr_swap_in_progress) {
		rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n",
				rc);
		}

		orientation =
			stat & CC_ORIENTATION_BIT ? TYPEC_CCOUT_VALUE_BIT : 0;
		rc = smblib_masked_write(chg, TYPE_C_CCOUT_CONTROL_REG,
			TYPEC_CCOUT_SRC_BIT | TYPEC_CCOUT_BUFFER_EN_BIT
					| TYPEC_CCOUT_VALUE_BIT,
			TYPEC_CCOUT_SRC_BIT | TYPEC_CCOUT_BUFFER_EN_BIT
					| orientation);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_CCOUT_CONTROL_REG rc=%d\n",
				rc);
		}
	} else {
		rc = smblib_masked_write(chg, TYPE_C_CCOUT_CONTROL_REG,
			TYPEC_CCOUT_SRC_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_CCOUT_CONTROL_REG rc=%d\n",
				rc);
		}

		/* enable DRP */
		rc = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
				 TYPEC_POWER_ROLE_CMD_MASK, 0);
		if (rc < 0)
			smblib_err(chg, "Couldn't enable DRP rc=%d\n", rc);
	}

	return 0;
}

/***************
 * Work Queues *
 ***************/
static void smblib_pr_swap_detach_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pr_swap_detach_work.work);
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATE_MACHINE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read STATE_MACHINE_STS rc=%d\n", rc);
		goto out;
	}
	smblib_dbg(chg, PR_REGISTER, "STATE_MACHINE_STS %x\n", stat);
	if (!(stat & TYPEC_ATTACH_DETACH_STATE_BIT)) {
		rc = smblib_request_dpdm(chg, false);
		if (rc < 0)
			smblib_err(chg, "Couldn't disable DPDM rc=%d\n", rc);
	}
out:
	vote(chg->awake_votable, DETACH_DETECT_VOTER, false, 0);
}

static void smblib_uusb_otg_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						uusb_otg_work.work);
	int rc;
	u8 stat;
	bool otg;

	rc = smblib_read(chg, TYPEC_U_USB_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_3 rc=%d\n", rc);
		goto out;
	}
	otg = !!(stat & U_USB_GROUND_NOVBUS_BIT);
	if (chg->otg_present != otg)
		smblib_notify_usb_host(chg, otg);
	else
		goto out;

	chg->otg_present = otg;
	if (!otg)
		chg->boost_current_ua = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher,
				otg ? chg->chg_freq.freq_below_otg_threshold
					: chg->chg_freq.freq_removal);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_boost rc=%d\n", rc);

	smblib_dbg(chg, PR_REGISTER, "TYPE_C_U_USB_STATUS = 0x%02x OTG=%d\n",
			stat, otg);
	power_supply_changed(chg->usb_psy);

out:
	vote(chg->awake_votable, OTG_DELAY_VOTER, false, 0);
}

static void bms_update_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						bms_update_work);

	smblib_suspend_on_debug_battery(chg);

	if (chg->batt_psy)
		power_supply_changed(chg->batt_psy);
}

static void pl_update_work(struct work_struct *work)
{
	union power_supply_propval prop_val;
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pl_update_work);
	int rc;

	if (chg->smb_temp_max == -EINVAL) {
		rc = smblib_get_thermal_threshold(chg,
					SMB_REG_H_THRESHOLD_MSB_REG,
					&chg->smb_temp_max);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't get charger_temp_max rc=%d\n",
					rc);
			return;
		}
	}

	prop_val.intval = chg->smb_temp_max;
	rc = power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
				&prop_val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set POWER_SUPPLY_PROP_CHARGER_TEMP_MAX rc=%d\n",
				rc);
		return;
	}

	if (chg->sec_chg_selected == POWER_SUPPLY_CHARGER_SEC_CP)
		return;

	smblib_select_sec_charger(chg, POWER_SUPPLY_CHARGER_SEC_PL,
				POWER_SUPPLY_CP_NONE, false);
}

static void clear_hdc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						clear_hdc_work.work);

	chg->is_hdc = 0;
	vote(chg->hdc_irq_disable_votable, HDC_IRQ_VOTER, false, 0);
}

static void smblib_icl_change_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							icl_change_work.work);
	int rc, settled_ua;

	rc = smblib_get_charge_param(chg, &chg->param.icl_stat, &settled_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
		return;
	}

	power_supply_changed(chg->usb_main_psy);

	smblib_dbg(chg, PR_INTERRUPT, "icl_settled=%d\n", settled_ua);
}

static void smblib_pl_enable_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							pl_enable_work.work);

	smblib_dbg(chg, PR_PARALLEL, "timer expired, enabling parallel\n");
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, false, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);
}

static void smblib_thermal_regulation_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						thermal_regulation_work.work);
	int rc;

	rc = smblib_update_thermal_readings(chg);
	if (rc < 0)
		smblib_err(chg, "Couldn't read current thermal values %d\n",
					rc);

	rc = smblib_process_thermal_readings(chg);
	if (rc < 0)
		smblib_err(chg, "Couldn't run sw thermal regulation %d\n",
					rc);
}

#define MOISTURE_PROTECTION_CHECK_DELAY_MS 300000		/* 5 mins */
static void smblib_moisture_protection_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						moisture_protection_work);
	int rc;
	bool usb_plugged_in;
	u8 stat;

	/*
	 * Disable 1% duty cycle on CC_ID pin and enable uUSB factory mode
	 * detection to track any change on RID, as interrupts are disable.
	 */
	rc = smblib_write(chg, ((chg->smb_version == PMI632_SUBTYPE) ?
			PMI632_TYPEC_U_USB_WATER_PROTECTION_CFG_REG :
			TYPEC_U_USB_WATER_PROTECTION_CFG_REG), 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable periodic monitoring of CC_ID rc=%d\n",
			rc);
		goto out;
	}

	rc = smblib_masked_write(chg, TYPEC_U_USB_CFG_REG,
					EN_MICRO_USB_FACTORY_MODE_BIT,
					EN_MICRO_USB_FACTORY_MODE_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable uUSB factory mode detection rc=%d\n",
			rc);
		goto out;
	}

	/*
	 * Add a delay of 100ms to allow change in rid to reflect on
	 * status registers.
	 */
	msleep(100);

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		goto out;
	}
	usb_plugged_in = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	/* Check uUSB status for moisture presence */
	rc = smblib_read(chg, TYPEC_U_USB_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_U_USB_STATUS_REG rc=%d\n",
				rc);
		goto out;
	}

	/*
	 * Factory mode detection happens in case of USB plugged-in by using
	 * a different current source of 2uA which can hamper moisture
	 * detection. Since factory mode is not supported in kernel, factory
	 * mode detection can be considered as equivalent to presence of
	 * moisture.
	 */
	if (stat == U_USB_STATUS_WATER_PRESENT || stat == U_USB_FMB1_BIT ||
			stat == U_USB_FMB2_BIT || (usb_plugged_in &&
			stat == U_USB_FLOAT1_BIT)) {
		smblib_set_moisture_protection(chg, true);
		alarm_start_relative(&chg->moisture_protection_alarm,
			ms_to_ktime(MOISTURE_PROTECTION_CHECK_DELAY_MS));
	} else {
		smblib_set_moisture_protection(chg, false);
		rc = alarm_cancel(&chg->moisture_protection_alarm);
		if (rc < 0)
			smblib_err(chg, "Couldn't cancel moisture protection alarm\n");
	}

out:
	pm_relax(chg->dev);
}

static enum alarmtimer_restart moisture_protection_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
	struct smb_charger *chg = container_of(alarm, struct smb_charger,
					moisture_protection_alarm);

	smblib_dbg(chg, PR_MISC, "moisture Protection Alarm Triggered %lld\n",
			ktime_to_ms(now));

	/* Atomic context, cannot use voter */
	pm_stay_awake(chg->dev);
	schedule_work(&chg->moisture_protection_work);

	return ALARMTIMER_NORESTART;
}

static void smblib_chg_termination_work(struct work_struct *work)
{
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(work, struct smb_charger,
						chg_termination_work);
	int rc, input_present, delay = CHG_TERM_WA_ENTRY_DELAY_MS;

	/*
	 * Hold awake votable to prevent pm_relax being called prior to
	 * completion of this work.
	 */
	vote(chg->awake_votable, CHG_TERMINATION_VOTER, true, 0);

	rc = smblib_is_input_present(chg, &input_present);
	if ((rc < 0) || !input_present)
		goto out;

	rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_REAL_CAPACITY, &pval);
	if ((rc < 0) || (pval.intval < 100)) {
		vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
		vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER, false, 0);
		goto out;
	}

	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CHARGE_FULL,
					&pval);
	if (rc < 0)
		goto out;

	/*
	 * On change in the value of learned capacity, re-initialize the
	 * reference cc_soc value due to change in cc_soc characteristic value
	 * at full capacity. Also, in case cc_soc_ref value is reset,
	 * re-initialize it.
	 */
	if (pval.intval != chg->charge_full_cc || !chg->cc_soc_ref) {
		chg->charge_full_cc = pval.intval;
		rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CC_SOC,
					&pval);
		if (rc < 0)
			goto out;

		chg->cc_soc_ref = pval.intval;
	} else {
		rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CC_SOC,
					&pval);
		if (rc < 0)
			goto out;
	}

	/*
	 * In BSM a sudden jump in CC_SOC is not expected. If seen, its a
	 * good_ocv or updated capacity, reject it.
	 */
	if (chg->last_cc_soc && pval.intval > (chg->last_cc_soc + 100)) {
		/* CC_SOC has increased by 1% from last time */
		chg->cc_soc_ref = pval.intval;
		smblib_dbg(chg, PR_MISC, "cc_soc jumped(%d->%d), reset cc_soc_ref\n",
				chg->last_cc_soc, pval.intval);
	}
	chg->last_cc_soc = pval.intval;

	/*
	 * Suspend/Unsuspend USB input to keep cc_soc within the 0.5% to 0.75%
	 * overshoot range of the cc_soc value at termination, to prevent
	 * overcharging.
	 */
	if (pval.intval < DIV_ROUND_CLOSEST(chg->cc_soc_ref * 10050, 10000)) {
		vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
		vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER, false, 0);
		delay = CHG_TERM_WA_ENTRY_DELAY_MS;
	} else if (pval.intval > DIV_ROUND_CLOSEST(chg->cc_soc_ref * 10075,
								10000)) {
		if (input_present & INPUT_PRESENT_USB)
			vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER,
					true, 0);
		if (input_present & INPUT_PRESENT_DC)
			vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER,
					true, 0);
		delay = CHG_TERM_WA_EXIT_DELAY_MS;
	}

	smblib_dbg(chg, PR_MISC, "Chg Term WA readings: cc_soc: %d, cc_soc_ref: %d, delay: %d\n",
			pval.intval, chg->cc_soc_ref, delay);
	alarm_start_relative(&chg->chg_termination_alarm, ms_to_ktime(delay));
out:
	vote(chg->awake_votable, CHG_TERMINATION_VOTER, false, 0);
}

static enum alarmtimer_restart chg_termination_alarm_cb(struct alarm *alarm,
								ktime_t now)
{
	struct smb_charger *chg = container_of(alarm, struct smb_charger,
							chg_termination_alarm);

	smblib_dbg(chg, PR_MISC, "Charge termination WA alarm triggered %lld\n",
			ktime_to_ms(now));

	/* Atomic context, cannot use voter */
	pm_stay_awake(chg->dev);
	schedule_work(&chg->chg_termination_work);

	return ALARMTIMER_NORESTART;
}

static void jeita_update_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						jeita_update_work);
	struct device_node *node = chg->dev->of_node;
	struct device_node *batt_node, *pnode;
	union power_supply_propval val;
	int rc, tmp[2], max_fcc_ma, max_fv_uv;
	u32 jeita_hard_thresholds[2];

	batt_node = of_find_node_by_name(node, "qcom,battery-data");
	if (!batt_node) {
		smblib_err(chg, "Batterydata not available\n");
		goto out;
	}

	/* if BMS is not ready, defer the work */
	if (!chg->bms_psy)
		return;

	rc = smblib_get_prop_from_bms(chg,
			POWER_SUPPLY_PROP_RESISTANCE_ID, &val);
	if (rc < 0) {
		smblib_err(chg, "Failed to get batt-id rc=%d\n", rc);
		goto out;
	}

	/* if BMS hasn't read out the batt_id yet, defer the work */
	if (val.intval <= 0)
		return;

	pnode = of_batterydata_get_best_profile(batt_node,
					val.intval / 1000, NULL);
	if (IS_ERR(pnode)) {
		rc = PTR_ERR(pnode);
		smblib_err(chg, "Failed to detect valid battery profile %d\n",
				rc);
		goto out;
	}

	rc = of_property_read_u32_array(pnode, "qcom,jeita-hard-thresholds",
				jeita_hard_thresholds, 2);
	if (!rc) {
		rc = smblib_update_jeita(chg, jeita_hard_thresholds,
					JEITA_HARD);
		if (rc < 0) {
			smblib_err(chg, "Couldn't configure Hard Jeita rc=%d\n",
					rc);
			goto out;
		}
	}

	rc = of_property_read_u32_array(pnode, "qcom,jeita-soft-thresholds",
				chg->jeita_soft_thlds, 2);
	if (!rc) {
		rc = smblib_update_jeita(chg, chg->jeita_soft_thlds,
					JEITA_SOFT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't configure Soft Jeita rc=%d\n",
					rc);
			goto out;
		}

		rc = of_property_read_u32_array(pnode,
					"qcom,jeita-soft-hys-thresholds",
					chg->jeita_soft_hys_thlds, 2);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get Soft Jeita hysteresis thresholds rc=%d\n",
					rc);
			goto out;
		}
	}

	chg->jeita_soft_fcc[0] = chg->jeita_soft_fcc[1] = -EINVAL;
	chg->jeita_soft_fv[0] = chg->jeita_soft_fv[1] = -EINVAL;
	max_fcc_ma = max_fv_uv = -EINVAL;

	of_property_read_u32(pnode, "qcom,fastchg-current-ma", &max_fcc_ma);
	of_property_read_u32(pnode, "qcom,max-voltage-uv", &max_fv_uv);

	if (max_fcc_ma <= 0 || max_fv_uv <= 0) {
		smblib_err(chg, "Incorrect fastchg-current-ma or max-voltage-uv\n");
		goto out;
	}

	rc = of_property_read_u32_array(pnode, "qcom,jeita-soft-fcc-ua",
					tmp, 2);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get fcc values for soft JEITA rc=%d\n",
				rc);
		goto out;
	}

	max_fcc_ma *= 1000;
	if (tmp[0] > max_fcc_ma || tmp[1] > max_fcc_ma) {
		smblib_err(chg, "Incorrect FCC value [%d %d] max: %d\n", tmp[0],
			tmp[1], max_fcc_ma);
		goto out;
	}
	chg->jeita_soft_fcc[0] = tmp[0];
	chg->jeita_soft_fcc[1] = tmp[1];

	rc = of_property_read_u32_array(pnode, "qcom,jeita-soft-fv-uv", tmp,
					2);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get fv values for soft JEITA rc=%d\n",
				rc);
		goto out;
	}

	if (tmp[0] > max_fv_uv || tmp[1] > max_fv_uv) {
		smblib_err(chg, "Incorrect FV value [%d %d] max: %d\n", tmp[0],
			tmp[1], max_fv_uv);
		goto out;
	}
	chg->jeita_soft_fv[0] = tmp[0];
	chg->jeita_soft_fv[1] = tmp[1];

	rc = smblib_soft_jeita_arb_wa(chg);
	if (rc < 0) {
		smblib_err(chg, "Couldn't fix soft jeita arb rc=%d\n",
				rc);
		goto out;
	}

	chg->jeita_configured = JEITA_CFG_COMPLETE;
	return;

out:
	chg->jeita_configured = JEITA_CFG_FAILURE;
}

static void smblib_lpd_ra_open_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							lpd_ra_open_work.work);
	union power_supply_propval pval;
	u8 stat;
	int rc;

	if (chg->pr_swap_in_progress || chg->pd_hard_reset) {
		chg->lpd_stage = LPD_STAGE_NONE;
		goto out;
	}

	if (chg->lpd_stage != LPD_STAGE_FLOAT)
		goto out;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n",
			rc);
		goto out;
	}

	/* quit if moisture status is gone or in attached state */
	if (!(stat & TYPEC_WATER_DETECTION_STATUS_BIT)
			|| (stat & TYPEC_TCCDEBOUNCE_DONE_STATUS_BIT)) {
		chg->lpd_stage = LPD_STAGE_NONE;
		goto out;
	}

	chg->lpd_stage = LPD_STAGE_COMMIT;

	/* Enable source only mode */
	pval.intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
	rc = smblib_set_prop_typec_power_role(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set typec source only mode rc=%d\n",
					rc);
		goto out;
	}

	/* Wait 1.5ms to get SBUx ready */
	usleep_range(1500, 1510);

	if (smblib_rsbux_low(chg, RSBU_K_300K_UV)) {
		/* Moisture detected, enable sink only mode */
		pval.intval = POWER_SUPPLY_TYPEC_PR_SINK;
		rc = smblib_set_prop_typec_power_role(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set typec sink only rc=%d\n",
				rc);
			goto out;
		}

		chg->lpd_reason = LPD_MOISTURE_DETECTED;
		chg->moisture_present =  true;

	} else {
		/* Floating cable, disable water detection irq temporarily */
		rc = smblib_masked_write(chg, TYPE_C_INTERRUPT_EN_CFG_2_REG,
					TYPEC_WATER_DETECTION_INT_EN_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set TYPE_C_INTERRUPT_EN_CFG_2_REG rc=%d\n",
					rc);
			goto out;
		}

		/* restore DRP mode */
		pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		rc = smblib_set_prop_typec_power_role(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
				pval.intval, rc);
			goto out;
		}

		chg->lpd_reason = LPD_FLOATING_CABLE;
		//[+++]ASUS : Add usb water alert feature
		CHG_DBG("[usb_water] Floating cable\n");
		//[---]ASUS : Add usb water alert feature

	}

	/* recheck in 60 seconds */
	alarm_start_relative(&chg->lpd_recheck_timer, ms_to_ktime(60000));
out:
	vote(chg->awake_votable, LPD_VOTER, false, 0);
}

static void smblib_lpd_detach_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							lpd_detach_work.work);

	if (chg->lpd_stage == LPD_STAGE_FLOAT_CANCEL)
		chg->lpd_stage = LPD_STAGE_NONE;
}

//[+++]ASUS : Add usb thermal alert feature
void asus_usb_thermal_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							asus_usb_thermal_work.work);
	int rc;
	u8 reg;
	int conn_temp, usb_present, otg_present;

	if (!g_usb_thermal_enable || no_input_suspend_flag)
		return;

	if (chg->iio.connector_temp_chan) {
		rc = iio_read_channel_processed(chg->iio.connector_temp_chan,
				&conn_temp);
		if (rc < 0) {
			pr_err("Error in reading connector_temp channel, rc=%d", rc);

			//retry if read fail
			rc = iio_read_channel_processed(chg->iio.connector_temp_chan,
				&conn_temp);
			if (rc < 0) {
				pr_err("Error in reading connector_temp channel retry, rc=%d", rc);
				return;
			}
			
		}
		conn_temp = conn_temp / 100;
	}
	if (g_usb_thermal_debug)
		conn_temp = g_usb_thermal_debug;

	rc = smblib_read(smbchg_dev, TYPE_C_MISC_STATUS_REG, &reg);
	if (rc < 0)
		CHG_DBG_E("Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n", rc);
	usb_present = reg & CC_ATTACHED_BIT;

	rc = smblib_read(smbchg_dev, DCDC_CMD_OTG_REG, &reg);
	if (rc < 0)
		CHG_DBG_E("Couldn't read DCDC_CMD_OTG_REG rc=%d\n", rc);
	otg_present = reg & OTG_EN_BIT;

	if (!usb_thermal_once_flag && conn_temp >= 700) {
		if (usb_present) {
			if (!otg_present) {
				//#1: 0x1340[0] = 1, Charger input suspend
				smblib_set_usb_suspend(smbchg_dev, 1);
			} else {
				//#1: Delay 100ms
				msleep(100);
			}
			//#2: 0x1140[0] = 0, Disable OTG
			rc = smblib_masked_write(smbchg_dev, DCDC_CMD_OTG_REG, OTG_EN_BIT, 0);
			if (rc < 0)
				CHG_DBG_E("Couldn't set DCDC_CMD_OTG_REG rc=%d\n", rc);

			asus_extcon_set_state_sync(chg->thermal_extcon, THERMAL_ALERT_WITH_AC);
		} else {
			asus_extcon_set_state_sync(chg->thermal_extcon, THERMAL_ALERT_NO_AC);
		}
		usb_thermal_once_flag = true;

		CHG_DBG("conn_temp(%d) >= 700, usb thermal alert\n", conn_temp);
	}
	else if (!usb_present && conn_temp <= 600) {
		smblib_set_usb_suspend(smbchg_dev, 0);
		usb_thermal_once_flag = 0;
		
		asus_extcon_set_state_sync(chg->thermal_extcon, THERMAL_ALERT_NONE);
		CHG_DBG("conn_temp(%d) <= 600, disable usb suspend\n", conn_temp);
	}
	
	CHG_DBG("conn_temp(%d), usb(%d), otg(%d), usb_connector = %d\n",
			conn_temp, usb_present, otg_present, asus_extcon_get_state(smbchg_dev->thermal_extcon));

	schedule_delayed_work(&smbchg_dev->asus_usb_thermal_work, msecs_to_jiffies(THERMAL_ALERT_CYCLE));
}
//[---]ASUS : Add usb thermal alert feature

//[+++]ASUS : Show "+" on charging icon
#define SWITCH_QC_NOT_QUICK_CHARGING        4
#define SWITCH_QC_QUICK_CHARGING            3
#define SWITCH_QC_NOT_QUICK_CHARGING_PLUS   2
#define SWITCH_QC_QUICK_CHARGING_PLUS       1
#define SWITCH_QC_OTHER	             0
static int pre_set = -1;
void set_qc_stat(union power_supply_propval *val)
{
	int bat_status;
	int qc_status;
	int set = SWITCH_QC_OTHER;

	if (g_Charger_mode)
		return;

	if(smbchg_dev->pd_active){
		if(final_pd_mw > 10000000){
			if (asus_get_prop_batt_capacity(smbchg_dev) <= 70)
				set = SWITCH_QC_QUICK_CHARGING_PLUS;
			else
				set = SWITCH_QC_NOT_QUICK_CHARGING_PLUS;
		}
		else if(final_pd_mw == 10000000){
			if (asus_get_prop_batt_capacity(smbchg_dev) <= 70)
				set = SWITCH_QC_QUICK_CHARGING;
			else
				set = SWITCH_QC_NOT_QUICK_CHARGING;
		}
		else
			set = SWITCH_QC_OTHER;
			
		asus_extcon_set_state_sync(smbchg_dev->quickchg_extcon, set);
		return;
	}

	bat_status = val->intval;
	qc_status = asus_get_batt_status();

	if (qc_status == NORMAL) {
		set = SWITCH_QC_OTHER;
		asus_extcon_set_state_sync(smbchg_dev->quickchg_extcon, set);

		if (pre_set != set) {
			CHG_DBG("Batt_status = %s, quick_charging = %d\n", bat_status_text[bat_status], set);
			pre_set = set;
		}
		return;
	}

	switch (bat_status) {
	//QUICK_CHARGING and QUICK_CHARGING_PLUS happen in charger mode only, refer to smblib_get_prop_batt_status
	case POWER_SUPPLY_STATUS_CHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
	case POWER_SUPPLY_STATUS_QUICK_CHARGING:
	case POWER_SUPPLY_STATUS_QUICK_CHARGING_PLUS:
		if (asus_get_prop_batt_capacity(smbchg_dev) <= 70) {
			if (qc_status == QC)
				set = SWITCH_QC_QUICK_CHARGING;
			else if (qc_status == QC_PLUS)
				set = SWITCH_QC_QUICK_CHARGING_PLUS;
		} else {
			if (qc_status == QC)
				set = SWITCH_QC_NOT_QUICK_CHARGING;
			else if (qc_status == QC_PLUS)
				set = SWITCH_QC_NOT_QUICK_CHARGING_PLUS;
		}
		asus_extcon_set_state_sync(smbchg_dev->quickchg_extcon, set);
		break;
	default:
		set = SWITCH_QC_OTHER;
		asus_extcon_set_state_sync(smbchg_dev->quickchg_extcon, set);
		break;
	}

	if (pre_set != set) {
		CHG_DBG("Batt_status = %s, quick_charging = %d\n", bat_status_text[bat_status], set);
		pre_set = set;
	}
	return;
}
//[---]ASUS : Show "+" on charging icon

static int smblib_create_votables(struct smb_charger *chg)
{
	int rc = 0;

	chg->fcc_votable = find_votable("FCC");
	if (chg->fcc_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find FCC votable rc=%d\n", rc);
		return rc;
	}

	chg->fv_votable = find_votable("FV");
	if (chg->fv_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find FV votable rc=%d\n", rc);
		return rc;
	}

	chg->usb_icl_votable = find_votable("USB_ICL");
	if (chg->usb_icl_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find USB_ICL votable rc=%d\n", rc);
		return rc;
	}

	chg->pl_disable_votable = find_votable("PL_DISABLE");
	if (chg->pl_disable_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find votable PL_DISABLE rc=%d\n", rc);
		return rc;
	}

	chg->pl_enable_votable_indirect = find_votable("PL_ENABLE_INDIRECT");
	if (chg->pl_enable_votable_indirect == NULL) {
		rc = -EINVAL;
		smblib_err(chg,
			"Couldn't find votable PL_ENABLE_INDIRECT rc=%d\n",
			rc);
		return rc;
	}

	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);

	chg->smb_override_votable = create_votable("SMB_EN_OVERRIDE",
				VOTE_SET_ANY,
				smblib_smb_disable_override_vote_callback, chg);
	if (IS_ERR(chg->smb_override_votable)) {
		rc = PTR_ERR(chg->smb_override_votable);
		chg->smb_override_votable = NULL;
		return rc;
	}

	chg->dc_suspend_votable = create_votable("DC_SUSPEND", VOTE_SET_ANY,
					smblib_dc_suspend_vote_callback,
					chg);
	if (IS_ERR(chg->dc_suspend_votable)) {
		rc = PTR_ERR(chg->dc_suspend_votable);
		chg->dc_suspend_votable = NULL;
		return rc;
	}

	chg->awake_votable = create_votable("AWAKE", VOTE_SET_ANY,
					smblib_awake_vote_callback,
					chg);
	if (IS_ERR(chg->awake_votable)) {
		rc = PTR_ERR(chg->awake_votable);
		chg->awake_votable = NULL;
		return rc;
	}

	chg->chg_disable_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY,
					smblib_chg_disable_vote_callback,
					chg);
	if (IS_ERR(chg->chg_disable_votable)) {
		rc = PTR_ERR(chg->chg_disable_votable);
		chg->chg_disable_votable = NULL;
		return rc;
	}

	chg->limited_irq_disable_votable = create_votable(
				"USB_LIMITED_IRQ_DISABLE",
				VOTE_SET_ANY,
				smblib_limited_irq_disable_vote_callback,
				chg);
	if (IS_ERR(chg->limited_irq_disable_votable)) {
		rc = PTR_ERR(chg->limited_irq_disable_votable);
		chg->limited_irq_disable_votable = NULL;
		return rc;
	}

	chg->hdc_irq_disable_votable = create_votable("USB_HDC_IRQ_DISABLE",
					VOTE_SET_ANY,
					smblib_hdc_irq_disable_vote_callback,
					chg);
	if (IS_ERR(chg->hdc_irq_disable_votable)) {
		rc = PTR_ERR(chg->hdc_irq_disable_votable);
		chg->hdc_irq_disable_votable = NULL;
		return rc;
	}

	chg->icl_irq_disable_votable = create_votable("USB_ICL_IRQ_DISABLE",
					VOTE_SET_ANY,
					smblib_icl_irq_disable_vote_callback,
					chg);
	if (IS_ERR(chg->icl_irq_disable_votable)) {
		rc = PTR_ERR(chg->icl_irq_disable_votable);
		chg->icl_irq_disable_votable = NULL;
		return rc;
	}

	return rc;
}

static void smblib_destroy_votables(struct smb_charger *chg)
{
	if (chg->dc_suspend_votable)
		destroy_votable(chg->dc_suspend_votable);
	if (chg->usb_icl_votable)
		destroy_votable(chg->usb_icl_votable);
	if (chg->awake_votable)
		destroy_votable(chg->awake_votable);
	if (chg->chg_disable_votable)
		destroy_votable(chg->chg_disable_votable);
}

static void smblib_iio_deinit(struct smb_charger *chg)
{
	if (!IS_ERR_OR_NULL(chg->iio.usbin_v_chan))
		iio_channel_release(chg->iio.usbin_v_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_i_chan))
		iio_channel_release(chg->iio.usbin_i_chan);
	if (!IS_ERR_OR_NULL(chg->iio.temp_chan))
		iio_channel_release(chg->iio.temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.sbux_chan))
		iio_channel_release(chg->iio.sbux_chan);
	if (!IS_ERR_OR_NULL(chg->iio.vph_v_chan))
		iio_channel_release(chg->iio.vph_v_chan);
	if (!IS_ERR_OR_NULL(chg->iio.asus_adapter_vadc_chan))
		iio_channel_release(chg->iio.asus_adapter_vadc_chan);
	if (!IS_ERR_OR_NULL(chg->iio.die_temp_chan))
		iio_channel_release(chg->iio.die_temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.connector_temp_chan))
		iio_channel_release(chg->iio.connector_temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.skin_temp_chan))
		iio_channel_release(chg->iio.skin_temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.smb_temp_chan))
		iio_channel_release(chg->iio.smb_temp_chan);
}

int smblib_init(struct smb_charger *chg)
{
	union power_supply_propval prop_val;
	int rc = 0;

	mutex_init(&chg->smb_lock);
	mutex_init(&chg->irq_status_lock);
	INIT_WORK(&chg->bms_update_work, bms_update_work);
	INIT_WORK(&chg->pl_update_work, pl_update_work);
	INIT_WORK(&chg->jeita_update_work, jeita_update_work);
	INIT_DELAYED_WORK(&chg->clear_hdc_work, clear_hdc_work);
//[+++]ASUS work
	INIT_DELAYED_WORK(&chg->asus_chg_flow_work, asus_chg_flow_work);
	INIT_DELAYED_WORK(&chg->asus_adapter_adc_work, asus_adapter_adc_work);
	INIT_DELAYED_WORK(&chg->asus_min_monitor_work, asus_min_monitor_work);
	INIT_DELAYED_WORK(&chg->asus_batt_RTC_work, asus_batt_RTC_work);
	INIT_DELAYED_WORK(&chg->asus_set_flow_flag_work, asus_set_flow_flag_work);
	alarm_init(&bat_alarm, ALARM_REALTIME, batAlarm_handler);
	INIT_DELAYED_WORK(&chg->asus_usb_thermal_work, asus_usb_thermal_work);
	INIT_DELAYED_WORK(&chg->asus_reverse_charge_work, asus_reverse_charge_work);
	INIT_DELAYED_WORK(&chg->asus_cable_capability_check_work, asus_cable_capability_check_work);
	INIT_DELAYED_WORK(&chg->asus_reverse_charge_check_camera, asus_reverse_charge_check_camera);
	INIT_DELAYED_WORK(&chg->asus_enable_inov_work, asus_enable_inov_work);
//[---]ASUS work

	INIT_DELAYED_WORK(&chg->icl_change_work, smblib_icl_change_work);
	INIT_DELAYED_WORK(&chg->pl_enable_work, smblib_pl_enable_work);
	INIT_DELAYED_WORK(&chg->uusb_otg_work, smblib_uusb_otg_work);
	INIT_DELAYED_WORK(&chg->bb_removal_work, smblib_bb_removal_work);
	INIT_DELAYED_WORK(&chg->lpd_ra_open_work, smblib_lpd_ra_open_work);
	INIT_DELAYED_WORK(&chg->lpd_detach_work, smblib_lpd_detach_work);
	INIT_DELAYED_WORK(&chg->thermal_regulation_work,
					smblib_thermal_regulation_work);
	INIT_DELAYED_WORK(&chg->usbov_dbc_work, smblib_usbov_dbc_work);
	INIT_DELAYED_WORK(&chg->pr_swap_detach_work,
					smblib_pr_swap_detach_work);

	if (chg->wa_flags & CHG_TERMINATION_WA) {
		INIT_WORK(&chg->chg_termination_work,
					smblib_chg_termination_work);

		if (alarmtimer_get_rtcdev()) {
			alarm_init(&chg->chg_termination_alarm, ALARM_BOOTTIME,
						chg_termination_alarm_cb);
		} else {
			smblib_err(chg, "Couldn't get rtc device\n");
			return -ENODEV;
		}
	}

	if (chg->uusb_moisture_protection_enabled) {
		INIT_WORK(&chg->moisture_protection_work,
					smblib_moisture_protection_work);

		if (alarmtimer_get_rtcdev()) {
			alarm_init(&chg->moisture_protection_alarm,
				ALARM_BOOTTIME, moisture_protection_alarm_cb);
		} else {
			smblib_err(chg, "Failed to initialize moisture protection alarm\n");
			return -ENODEV;
		}
	}

	chg->fake_capacity = -EINVAL;
	chg->fake_input_current_limited = -EINVAL;
	chg->fake_batt_status = -EINVAL;
	chg->sink_src_mode = UNATTACHED_MODE;
	chg->jeita_configured = false;
	chg->sec_chg_selected = POWER_SUPPLY_CHARGER_SEC_NONE;
	chg->cp_reason = POWER_SUPPLY_CP_NONE;
	chg->thermal_status = TEMP_BELOW_RANGE;

//[+++]ASUS extcon registration
	chg->thermal_extcon = extcon_dev_allocate(asus_extcon_cable);
	if (IS_ERR(chg->thermal_extcon)) {
		rc = PTR_ERR(chg->thermal_extcon);
		dev_err(chg->dev, "[BAT][CHG] failed to allocate ASUS thermal extcon device rc=%d\n", rc);
	}

	asus_extcon_set_fnode_name(chg->thermal_extcon, "usb_connector");
	rc = extcon_dev_register(chg->thermal_extcon);
	if (rc < 0) {
		dev_err(chg->dev, "[BAT][CHG] failed to register ASUS thermal extcon device rc=%d\n", rc);
	}

	chg->water_extcon = extcon_dev_allocate(asus_extcon_cable);
	if (IS_ERR(chg->water_extcon)) {
		rc = PTR_ERR(chg->water_extcon);
		dev_err(chg->dev, "[BAT][CHG] failed to allocate ASUS water extcon device rc=%d\n", rc);
	}

	asus_extcon_set_fnode_name(chg->water_extcon, "vbus_liquid");
	rc = extcon_dev_register(chg->water_extcon);
	if (rc < 0) {
		dev_err(chg->dev, "[BAT][CHG] failed to register ASUS water extcon device rc=%d\n", rc);
	}

	chg->quickchg_extcon = extcon_dev_allocate(asus_extcon_cable);
	if (IS_ERR(chg->quickchg_extcon)) {
		rc = PTR_ERR(chg->quickchg_extcon);
		dev_err(chg->dev, "[BAT][CHG] failed to allocate ASUS quickchg extcon device rc=%d\n", rc);
	}

	asus_extcon_set_fnode_name(chg->quickchg_extcon, "quick_charging");
	rc = extcon_dev_register(chg->quickchg_extcon);
	if (rc < 0) {
		dev_err(chg->dev, "[BAT][CHG] failed to register ASUS quickchg extcon device rc=%d\n", rc);
	} else {
		qc_stat_registed = true;
	}
	
	chg->reversechg_extcon = extcon_dev_allocate(asus_extcon_cable);
	if (IS_ERR(chg->reversechg_extcon)) {
		rc = PTR_ERR(chg->reversechg_extcon);
		dev_err(chg->dev, "[BAT][CHG] failed to allocate ASUS reversechg extcon device rc=%d\n", rc);
	}

	asus_extcon_set_fnode_name(chg->reversechg_extcon, "reverse_charging");
	rc = extcon_dev_register(chg->reversechg_extcon);
	if (rc < 0) {
		dev_err(chg->dev, "[BAT][CHG] failed to register ASUS reversechg extcon device rc=%d\n", rc);
	}
//[---]ASUS extcon registration

	switch (chg->mode) {
	case PARALLEL_MASTER:
		rc = qcom_batt_init(chg->smb_version);
		if (rc < 0) {
			smblib_err(chg, "Couldn't init qcom_batt_init rc=%d\n",
				rc);
			return rc;
		}

		rc = qcom_step_chg_init(chg->dev, chg->step_chg_enabled,
						chg->sw_jeita_enabled, false);
		if (rc < 0) {
			smblib_err(chg, "Couldn't init qcom_step_chg_init rc=%d\n",
				rc);
			return rc;
		}

		rc = smblib_create_votables(chg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't create votables rc=%d\n",
				rc);
			return rc;
		}

		chg->bms_psy = power_supply_get_by_name("bms");

		if (chg->sec_pl_present) {
			chg->pl.psy = power_supply_get_by_name("parallel");
			if (chg->pl.psy) {
				if (chg->sec_chg_selected
					!= POWER_SUPPLY_CHARGER_SEC_CP) {
					rc = smblib_select_sec_charger(chg,
						POWER_SUPPLY_CHARGER_SEC_PL,
						POWER_SUPPLY_CP_NONE, false);
					if (rc < 0)
						smblib_err(chg, "Couldn't config pl charger rc=%d\n",
							rc);
				}

				if (chg->smb_temp_max == -EINVAL) {
					rc = smblib_get_thermal_threshold(chg,
						SMB_REG_H_THRESHOLD_MSB_REG,
						&chg->smb_temp_max);
					if (rc < 0) {
						dev_err(chg->dev, "Couldn't get charger_temp_max rc=%d\n",
								rc);
						return rc;
					}
				}

				prop_val.intval = chg->smb_temp_max;
				rc = power_supply_set_property(chg->pl.psy,
					POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
					&prop_val);
				if (rc < 0) {
					dev_err(chg->dev, "Couldn't set POWER_SUPPLY_PROP_CHARGER_TEMP_MAX rc=%d\n",
							rc);
					return rc;
				}
			}
		}

		rc = smblib_register_notifier(chg);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't register notifier rc=%d\n", rc);
			return rc;
		}
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	return rc;
}

int smblib_deinit(struct smb_charger *chg)
{
	switch (chg->mode) {
	case PARALLEL_MASTER:
		if (chg->uusb_moisture_protection_enabled) {
			alarm_cancel(&chg->moisture_protection_alarm);
			cancel_work_sync(&chg->moisture_protection_work);
		}
		if (chg->wa_flags & CHG_TERMINATION_WA) {
			alarm_cancel(&chg->chg_termination_alarm);
			cancel_work_sync(&chg->chg_termination_work);
		}
		cancel_work_sync(&chg->bms_update_work);
		cancel_work_sync(&chg->jeita_update_work);
		cancel_work_sync(&chg->pl_update_work);
		cancel_delayed_work_sync(&chg->clear_hdc_work);
		cancel_delayed_work_sync(&chg->icl_change_work);
		cancel_delayed_work_sync(&chg->pl_enable_work);
		cancel_delayed_work_sync(&chg->uusb_otg_work);
		cancel_delayed_work_sync(&chg->bb_removal_work);
		cancel_delayed_work_sync(&chg->lpd_ra_open_work);
		cancel_delayed_work_sync(&chg->lpd_detach_work);
		cancel_delayed_work_sync(&chg->thermal_regulation_work);
		cancel_delayed_work_sync(&chg->usbov_dbc_work);
		cancel_delayed_work_sync(&chg->pr_swap_detach_work);
		power_supply_unreg_notifier(&chg->nb);
		smblib_destroy_votables(chg);
		qcom_step_chg_deinit();
		qcom_batt_deinit();
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	smblib_iio_deinit(chg);

	return 0;
}
