/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/power_supply.h>
#include <oem-chg_control.h>
#include <oem-hkadc.h>
#include <oem-chg_param.h>
#include <mach/oem_fact.h>
#include <linux/reboot.h>
#include <oem-chg_smem.h>
#include <linux/dnand_cdev_driver.h>
#include <mach/board.h>
#include <linux/string.h>
#include <linux/qpnp/power-on.h>
#include <linux/gpio.h>

static void oem_chg_set_charging(void);
static void oem_chg_set_charger_power_supply(void);

extern int oem_option_item1_bit0;
extern int oem_option_item1_bit3;

static int current_batt_status;
static int current_batt_health;
static int current_batt_voltage;
static int current_capacity;
static int current_batt_temp;
static int current_pa_therm;
static int current_chg_connect;

static struct wake_lock oem_chg_wake_lock;
struct delayed_work oem_chg_battery_removal_work;

static oem_chg_state_type current_chg_state = OEM_CHG_ST_IDLE;
static oem_chg_state_type new_chg_state = OEM_CHG_ST_IDLE;


static int current_chg_volt_status = OEM_ST_VOLT_LOW;
static int current_chg_temp_status = OEM_ST_TEMP_NORM;
static bool usb_alert_status = false;

static int battery_limit_volt_status	= OEM_ST_TEMP_NORM;
static int battery_temp_status	= OEM_ST_TEMP_NORM;
static int battery_limit_soc_status	= OEM_ST_TEMP_NORM;

static int limit_chg_timer = 0;
static bool limit_chg_disable = false;

static uint32_t total_charging_time = 0;

static atomic_t wchg_eoc_status = ATOMIC_INIT(0);


static void oem_chg_wake_lock_control(void)
{
	bool sleep_disable = false;

	switch (current_chg_state) {
		case OEM_CHG_ST_FAST:
		case OEM_CHG_ST_LIMIT:
		case OEM_CHG_ST_ERR_BATT_TEMP:
			sleep_disable = true;
			break;

		default:
			break;
	}

	if (sleep_disable) {
		wake_lock(&oem_chg_wake_lock);
		pr_info("wake_lock set\n");
	} else if (wake_lock_active(&oem_chg_wake_lock)) {
		wake_lock_timeout(&oem_chg_wake_lock, msecs_to_jiffies(CHG_WAKE_LOCK_TIMEOUT_MS));
		pr_info("wake_lock_timeout set:%d[ms]\n", CHG_WAKE_LOCK_TIMEOUT_MS);
	} else {
		pr_info("wake_lock is not active already\n");
	}
}

static int oem_chg_get_batt_property(enum power_supply_property psp)
{
	struct power_supply *batt_psy;
	union power_supply_propval ret = {0,};

	batt_psy = power_supply_get_by_name("battery");
	batt_psy->get_property(batt_psy, psp, &ret);

	return ret.intval;
}
static bool is_current_property_init = false;
static void oem_chg_update_batt_property(void)
{
	current_batt_status		= oem_chg_get_batt_property(POWER_SUPPLY_PROP_STATUS);
	current_batt_health		= oem_chg_get_batt_property(POWER_SUPPLY_PROP_HEALTH);
	current_batt_voltage	= oem_chg_get_batt_property(POWER_SUPPLY_PROP_VOLTAGE_NOW);
	current_capacity		= oem_chg_get_batt_property(POWER_SUPPLY_PROP_CAPACITY);
	current_batt_temp		= oem_chg_get_batt_property(POWER_SUPPLY_PROP_TEMP);
	current_pa_therm		= oem_chg_get_batt_property(POWER_SUPPLY_PROP_OEM_PA_THERM);
	current_chg_connect		= oem_chg_get_batt_property(POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE);
	is_current_property_init = true;

	pr_debug("st:%d h:%d bv:%d cap:%d bt:%d pa:%d con:%d\n",
		current_batt_status, current_batt_health, current_batt_voltage,
		current_capacity, current_batt_temp, current_pa_therm,
		current_chg_connect);

	oem_chg_smem_write_vbat_mv(current_batt_voltage / 1000);
}


static void oem_chg_check_battery_removal(void)
{
	if (oem_option_item1_bit0) {
		return;
	}

	if (oem_chg_get_current_chg_connect() == POWER_SUPPLY_OEM_CONNECT_NONE) {
		return;
	}

	if (oem_chg_get_batt_property(POWER_SUPPLY_PROP_PRESENT)) {
		return;
	}
	schedule_delayed_work(&oem_chg_battery_removal_work,
			msecs_to_jiffies(BATT_REMOVAL_CHECK_TIME));
	pr_info("set oem_chg_battery_removal_work:%dms\n",BATT_REMOVAL_CHECK_TIME);
}

static void oem_chg_battery_removal(struct work_struct *work)
{
	if (oem_chg_get_batt_property(POWER_SUPPLY_PROP_PRESENT)) {
		return;
	}
	pr_err("power off, detected a battery removal\n");
	kernel_power_off();
}


static int oem_chg_check_timer(void)
{
	bool ret = false;

	if (total_charging_time < CHG_ERR_TIME_SEC) {
		if (!(total_charging_time % 60)) {
			pr_info("total:%d[min] timer:%d[min]\n",
				total_charging_time/60, CHG_ERR_TIME_SEC/60);
		}
		total_charging_time += OEM_HKADC_MONITOR_TIME_MS / 1000;
	} else {
		pr_info("expire total:%d[min] timer:%d[min]\n",
			total_charging_time/60, CHG_ERR_TIME_SEC/60);
		ret = true;
	}
	return ret;
}

static void oem_chg_reset_timer(void)
{
	total_charging_time = 0;
}



/*
static bool oem_chg_is_voice_call(void)
{
	bool ret = false;

	if (CDMA_ST_TALK == oem_chg_get_cdma_status()) {
		ret = true;
	}
	return ret;
}

static bool oem_chg_is_data_communication(void)
{
	bool ret = false;

	if ((CDMA_ST_DATA == oem_chg_get_cdma_status()) ||
		(EVDO_ST_ON == oem_chg_get_evdo_status()) ||
		(LTE_ST_ON == oem_chg_get_lte_status())) {
		ret = true;
	}
	return ret;
}
*/



static int oem_chg_check_volt_status(void)
{
	int ret = false;
	int new_chg_volt_status = OEM_ST_VOLT_NORM;
	int batt_voltage = oem_chg_get_current_batt_voltage();

	switch (oem_chg_get_current_volt_status()) {

	case OEM_ST_VOLT_LOW:
		if (batt_voltage >= BATT_VOLT_ERR_CHG_OFF) {
			new_chg_volt_status = OEM_ST_VOLT_HIGH;
		} else if (batt_voltage >= BATT_VOLT_POWER_ON_OK) {
			new_chg_volt_status = OEM_ST_VOLT_NORM;
		} else {
			new_chg_volt_status = OEM_ST_VOLT_LOW;
		}
		break;

	case OEM_ST_VOLT_NORM:
		if (batt_voltage >= BATT_VOLT_ERR_CHG_OFF) {
			new_chg_volt_status = OEM_ST_VOLT_HIGH;
		} else if (batt_voltage <= BATT_VOLT_POWER_ON_NG) {
			new_chg_volt_status = OEM_ST_VOLT_LOW;
		} else {
			new_chg_volt_status = OEM_ST_VOLT_NORM;
		}
		break;

	case OEM_ST_VOLT_HIGH:
		new_chg_volt_status = OEM_ST_VOLT_HIGH;
		break;

	default:
		break;
	}

	if (oem_chg_get_current_volt_status() != new_chg_volt_status) {
		pr_info("chg_volt_status current:%d new:%d\n",
			current_chg_volt_status, new_chg_volt_status);
		current_chg_volt_status = new_chg_volt_status;
		ret = true;
	}
	return ret;
}

static void oem_chg_battery_temp_check(void)
{
	int batt_temp_hot_chg_off	= BATT_TEMP_HOT_CHG_OFF;
	int batt_temp_hot_chg_on	= BATT_TEMP_HOT_CHG_ON;
	int batt_temp_cold_chg_off	= BATT_TEMP_COLD_CHG_OFF;
	int batt_temp_cold_chg_on	= BATT_TEMP_COLD_CHG_ON;
	int batt_temp_limit_chg_on	= FACTORY_TEMP_LIMIT_CHG_ON;
	int batt_temp_limit_chg_off	= FACTORY_TEMP_LIMIT_CHG_ON;
	int temp = oem_chg_get_current_batt_temp();


	if (oem_is_off_charge()) {
		batt_temp_limit_chg_on =  FACTORY_TEMP_LIMIT_CHG_ON;
		batt_temp_limit_chg_off = FACTORY_TEMP_LIMIT_CHG_ON;
	}


	if (oem_option_item1_bit3) {
		batt_temp_hot_chg_off = FACTORY_TEMP_CHG_OFF;
		batt_temp_limit_chg_on = FACTORY_TEMP_LIMIT_CHG_ON;
	} else if (oem_chg_get_current_chg_connect() == POWER_SUPPLY_OEM_CONNECT_CHARGE_STAND_1) {
		batt_temp_hot_chg_off	= BATT_TEMP_W_CHG_HOT_CHG_OFF;
		batt_temp_hot_chg_on	= BATT_TEMP_W_CHG_HOT_CHG_ON;
	}

	switch (oem_chg_get_current_temp_status()) {

	case OEM_ST_TEMP_NORM:
		if (batt_temp_cold_chg_off >= temp) {
			battery_temp_status = OEM_ST_TEMP_STOP;
		} else if (batt_temp_hot_chg_off <= temp) {
			battery_temp_status = OEM_ST_TEMP_STOP;
		} else if (batt_temp_limit_chg_on <= temp) {
			battery_temp_status = OEM_ST_TEMP_LIMIT;
		} else {
			battery_temp_status = OEM_ST_TEMP_NORM;
		}
		break;

	case OEM_ST_TEMP_LIMIT:
		if (batt_temp_hot_chg_off <= temp) {
			battery_temp_status = OEM_ST_TEMP_STOP;
		} else if (batt_temp_limit_chg_off < temp) {
			battery_temp_status = OEM_ST_TEMP_LIMIT;
		} else {
			battery_temp_status = OEM_ST_TEMP_NORM;
		}
		break;

	case OEM_ST_TEMP_STOP:
		if (batt_temp_cold_chg_on > temp) {
			battery_temp_status = OEM_ST_TEMP_STOP;
		} else if (batt_temp_hot_chg_on < temp) {
			battery_temp_status = OEM_ST_TEMP_STOP;
		} else if (batt_temp_limit_chg_off < temp) {
			battery_temp_status = OEM_ST_TEMP_LIMIT;
		} else {
			battery_temp_status = OEM_ST_TEMP_NORM;
		}
		break;

	default:
		break;
	}
}

static void oem_chg_battery_volt_limit_check(void)
{
	int batt_volt_chg_limit_on = BATT_VOLT_LIMIT_CHG_ON;
	int batt_volt_chg_limit_off = BATT_VOLT_LIMIT_CHG_OFF;
	int batt_volt = oem_chg_get_current_batt_voltage();

	switch (oem_chg_get_current_temp_status()) {

	case OEM_ST_TEMP_NORM:
		if (batt_volt_chg_limit_on <= batt_volt) {
			battery_limit_volt_status = OEM_ST_TEMP_LIMIT;
		} else {
			battery_limit_volt_status = OEM_ST_TEMP_NORM;
		}
		break;

	case OEM_ST_TEMP_LIMIT:
		if (batt_volt_chg_limit_off >= batt_volt) {
			battery_limit_volt_status = OEM_ST_TEMP_NORM;
		} else {
			battery_limit_volt_status = OEM_ST_TEMP_LIMIT;
		}
		break;

	case OEM_ST_TEMP_STOP:
		break;

	default:
		break;
	}
}


static void oem_chg_battery_soc_limit_check(void)
{
	int batt_soc = oem_chg_get_current_capacity();

	if (batt_soc >= BATT_SOC_LIMIT_OFF) {
		battery_limit_soc_status = OEM_ST_TEMP_NORM;
	} else {
		battery_limit_soc_status = OEM_ST_TEMP_LIMIT;
	}
	pr_debug("SOC = %d, battery_limit_soc_status = %d\n" ,batt_soc ,battery_limit_soc_status);
}

#define ENABLE_USB_ALERT_TEST	0
#if ENABLE_USB_ALERT_TEST
#include <asm-generic/fcntl.h>
#include <linux/syscalls.h>
#define PATH_USB_ALERT_TEST		"/data/tmp/usb_alert"
int read_from_test_file(int* usb, int* pa, int* cam)
{
	static int usb_last = -1;
	static int pa_last = -1;
	static int cam_last = -1;
	int fd = -1;
	char buf[64];
	int rc;

	fd = sys_open(PATH_USB_ALERT_TEST, O_RDONLY, 0);

	if (fd < 0) {
		return -1;
	}

	sys_read(fd, buf, sizeof(buf));
	sys_close(fd);

	rc = sscanf(buf, "%d %d %d", usb, pa, cam);

	if (rc != 3) {
		printk("%s(%d):'%s' invalid data. rc=%d\n", __func__, __LINE__, PATH_USB_ALERT_TEST, rc);
		return -1;
	}

	if (usb_last != *usb || pa_last != *pa || cam_last != *cam) {
		printk("%s(%d):usb=%d->%d pa=%d->%d cam=%d->%d\n", __func__, __LINE__,
			usb_last, *usb,
			pa_last, *pa,
			cam_last, *cam);
		usb_last = *usb;
		pa_last  = *pa;
		cam_last = *cam;
	}

	return 0;
}
#endif/* ENABLE_USB_ALERT_TEST */

static int oem_chg_usb_therm_monitor(void)
{
	int usb_therm;
	int pa_therm;
	int cam_therm;
	int req = false;

	usb_therm = oem_chg_get_batt_property(POWER_SUPPLY_PROP_OEM_USB_THERM);
	pa_therm = oem_chg_get_batt_property(POWER_SUPPLY_PROP_OEM_PA_THERM);
	cam_therm = oem_chg_get_batt_property(POWER_SUPPLY_PROP_OEM_CAMERA_THERM);

#if ENABLE_USB_ALERT_TEST
	read_from_test_file(&usb_therm, &pa_therm, &cam_therm);
#endif/* ENABLE_USB_ALERT_TEST */

	if (usb_alert_status) {
		if((usb_therm < USB_THERM_ALERT_OFF)) {
			usb_alert_status = false;
			req = true;
			pr_err("usb alert off therm usb:%d pa:%d cam:%d\n", usb_therm, pa_therm, cam_therm);
		}
	} else {
		if((usb_therm >= USB_THERM_ALERT_ON) &&
			((pa_therm < PA_THERM_USB_ALERT) ||
			(cam_therm < CAM_THERM_USB_ALERT))) {
			usb_alert_status = true;

			
			if (!oem_is_off_charge()) {
				pr_err("%s(%d):>>oem_notify_power_key_event(KEY_POWER)\n", __func__, __LINE__);
				oem_notify_power_key_event(KEY_POWER, 1);
			}
			

			req = true;
			pr_err("usb alert on therm usb:%d pa:%d cam:%d\n", usb_therm, pa_therm, cam_therm);
		}
	}
	return req;
}


static int oem_chg_check_temp_status(void)
{
	int ret = false;
	int new_chg_temp_status = OEM_ST_TEMP_NORM;
	oem_chg_update_modem_status();
	oem_chg_battery_volt_limit_check();
	oem_chg_battery_temp_check();
	oem_chg_battery_soc_limit_check();

	if ((POWER_SUPPLY_HEALTH_OVERHEAT == oem_chg_get_current_batt_health()) ||
		(POWER_SUPPLY_HEALTH_COLD == oem_chg_get_current_batt_health()) ||
		(OEM_ST_TEMP_STOP == battery_temp_status)) {
		new_chg_temp_status = OEM_ST_TEMP_STOP;
	} else if ((OEM_ST_TEMP_LIMIT == battery_limit_volt_status) &&

		(OEM_ST_TEMP_LIMIT == battery_limit_soc_status) &&

		(OEM_ST_TEMP_LIMIT == battery_temp_status)) {
		new_chg_temp_status = OEM_ST_TEMP_LIMIT;
	}

	if (oem_chg_get_current_temp_status() != new_chg_temp_status) {
		pr_info("chg_temp_status current:%d new:%d\n",current_chg_temp_status, new_chg_temp_status);
		current_chg_temp_status = new_chg_temp_status;
		pr_info("volt:%d batt:%d 1x:%d evdo:%d lte:%d\n",
			oem_chg_get_current_batt_voltage(),
			oem_chg_get_current_batt_temp(),
			oem_chg_get_cdma_status(),
			oem_chg_get_evdo_status(),
			oem_chg_get_lte_status());
		ret = true;
	}
	return ret;
}


static void oem_chg_limit_control(void)
{
	int oem_chg_limit_check_time = LIMIT_CHG_ON_TIME;

	limit_chg_timer += OEM_HKADC_MONITOR_TIME_MS;

	if (limit_chg_disable) {
		oem_chg_limit_check_time = LIMIT_CHG_OFF_TIME;
	}

	if (limit_chg_timer >= oem_chg_limit_check_time ) {
		limit_chg_timer = 0;
		limit_chg_disable ^= 1;
		oem_chg_buck_ctrl(limit_chg_disable);
		pr_info("limit_chg_disable=%d time_out=%d\n",
			limit_chg_disable, oem_chg_limit_check_time);
	}
}


static void oem_chg_init_limit_control(void)
{
	oem_chg_buck_ctrl(1);
	limit_chg_timer = 0;
	limit_chg_disable = true;
}

static void oem_chg_set_charging(void)
{
	oem_chg_batt_fet_ctrl(1);
	oem_chg_buck_ctrl(0);
}

static void oem_chg_set_charger_power_supply(void)
{
	oem_chg_batt_fet_ctrl(0);
	oem_chg_buck_ctrl(0);
}


void oem_chg_set_battery_power_supply(void)
{
	oem_chg_batt_fet_ctrl(0);
	oem_chg_buck_ctrl(1);
}
EXPORT_SYMBOL(oem_chg_set_battery_power_supply);


void oem_chg_stop_charging(void)
{
	oem_chg_batt_fet_ctrl(1);
	oem_chg_buck_ctrl(0);
	current_chg_state = OEM_CHG_ST_IDLE;
	new_chg_state = OEM_CHG_ST_IDLE;
	current_chg_volt_status = OEM_ST_TEMP_NORM;
	current_chg_temp_status = OEM_ST_TEMP_NORM;
	oem_chg_reset_timer();
	oem_chg_wake_lock_control();
	atomic_set(&wchg_eoc_status, 0);
}
EXPORT_SYMBOL(oem_chg_stop_charging);


int oem_chg_get_current_batt_status(void)
{
	int ret = current_batt_status;
	return ret;
}
EXPORT_SYMBOL(oem_chg_get_current_batt_status);

int oem_chg_get_current_batt_health(void)
{
	int ret = current_batt_health;
	return ret;
};
EXPORT_SYMBOL(oem_chg_get_current_batt_health);

int oem_chg_get_current_batt_voltage(void)
{
	int ret = current_batt_voltage;
	return ret;
}
EXPORT_SYMBOL(oem_chg_get_current_batt_voltage);

int oem_chg_get_current_capacity(void)
{
	int ret = current_capacity;
	return ret;
}
EXPORT_SYMBOL(oem_chg_get_current_capacity);

int oem_chg_get_current_batt_temp(void)
{

	if (!is_current_property_init) {
		current_batt_temp = oem_chg_get_batt_property(POWER_SUPPLY_PROP_TEMP);
		pr_info("is_current_property_init:false %s%dC\n",
			current_batt_temp < 0 ? "-" : "+", (int)abs(current_batt_temp));
	}
	return current_batt_temp;

}
EXPORT_SYMBOL(oem_chg_get_current_batt_temp);

int oem_chg_get_current_pa_therm(void)
{
	int ret = current_pa_therm;
	return ret;
}
EXPORT_SYMBOL(oem_chg_get_current_pa_therm);

int oem_chg_get_current_chg_connect(void)
{
	int ret = current_chg_connect;
	return ret;
}
EXPORT_SYMBOL(oem_chg_get_current_chg_connect);

int oem_chg_get_current_temp_status(void)
{
	int ret = current_chg_temp_status;
	return ret;
}
EXPORT_SYMBOL(oem_chg_get_current_temp_status);

int oem_chg_get_current_volt_status(void)
{
	int ret = current_chg_volt_status;
	return ret;
}
EXPORT_SYMBOL(oem_chg_get_current_volt_status);


int oem_chg_get_usb_alert_status(void)
{
	int ret = usb_alert_status;
	return ret;
}
EXPORT_SYMBOL(oem_chg_get_usb_alert_status);


int oem_chg_control(void)
{
	int req_ps_changed_batt = false;

	oem_chg_update_batt_property();

	oem_chg_check_battery_removal();

	if (1 == oem_param_share.test_mode_chg_disable) {
		return false;
	}

	req_ps_changed_batt = oem_chg_usb_therm_monitor();

	switch (current_chg_state) {
		case OEM_CHG_ST_IDLE:
			if (oem_chg_get_current_chg_connect() == POWER_SUPPLY_OEM_CONNECT_NONE) {
				break;
			}
			req_ps_changed_batt = oem_chg_check_volt_status();
			req_ps_changed_batt |= oem_chg_check_temp_status();
			if (oem_chg_get_current_volt_status() == OEM_ST_VOLT_HIGH) {
				new_chg_state = OEM_CHG_ST_ERR_BATT_VOLT;
			} else if (oem_chg_get_current_temp_status() == OEM_ST_TEMP_STOP) {
				new_chg_state = OEM_CHG_ST_ERR_BATT_TEMP;
			} else if (oem_chg_get_current_temp_status() == OEM_ST_TEMP_LIMIT) {
				new_chg_state = OEM_CHG_ST_LIMIT;
			} else if (oem_chg_get_current_batt_status() == POWER_SUPPLY_STATUS_FULL) {
				new_chg_state = OEM_CHG_ST_FULL;
			} else if (oem_chg_get_current_batt_status() == POWER_SUPPLY_STATUS_CHARGING) {
				new_chg_state = OEM_CHG_ST_FAST;
			}
			break;

		case OEM_CHG_ST_FAST:
			req_ps_changed_batt = oem_chg_check_volt_status();
			req_ps_changed_batt |= oem_chg_check_temp_status();
			if (oem_chg_get_current_volt_status() == OEM_ST_VOLT_HIGH) {
				new_chg_state = OEM_CHG_ST_ERR_BATT_VOLT;
			} else if (oem_chg_get_current_temp_status() == OEM_ST_TEMP_STOP) {
				new_chg_state = OEM_CHG_ST_ERR_BATT_TEMP;
			} else if (oem_chg_get_current_temp_status() == OEM_ST_TEMP_LIMIT) {
				new_chg_state = OEM_CHG_ST_LIMIT;
			} else if (oem_chg_get_current_batt_status() == POWER_SUPPLY_STATUS_FULL) {
				new_chg_state = OEM_CHG_ST_FULL;

			} else if (oem_chg_check_timer()) {
				new_chg_state = OEM_CHG_ST_ERR_TIME_OVER;

			}
			break;

		case OEM_CHG_ST_LIMIT:
			req_ps_changed_batt = oem_chg_check_volt_status();
			req_ps_changed_batt |= oem_chg_check_temp_status();
			if (oem_chg_get_current_volt_status() == OEM_ST_VOLT_HIGH) {
				new_chg_state = OEM_CHG_ST_ERR_BATT_VOLT;
			} else if (oem_chg_get_current_temp_status() == OEM_ST_TEMP_STOP) {
				new_chg_state = OEM_CHG_ST_ERR_BATT_TEMP;
			} else if (oem_chg_get_current_temp_status() == OEM_ST_TEMP_NORM) {
				new_chg_state = OEM_CHG_ST_FAST;
			} else if (oem_chg_get_current_batt_status() == POWER_SUPPLY_STATUS_FULL) {
				new_chg_state = OEM_CHG_ST_FULL;
			} else {
				oem_chg_limit_control();
			}
			break;

		case OEM_CHG_ST_FULL:
			if (oem_chg_get_current_batt_status() == POWER_SUPPLY_STATUS_CHARGING) {
				new_chg_state = OEM_CHG_ST_FAST;
			}
			break;

		case OEM_CHG_ST_ERR_BATT_TEMP:
			req_ps_changed_batt = oem_chg_check_temp_status();
			if (oem_chg_get_current_temp_status() != OEM_ST_TEMP_STOP) {
				new_chg_state = OEM_CHG_ST_FAST;
				atomic_set(&wchg_eoc_status, 0);
			}
			break;

		case OEM_CHG_ST_ERR_BATT_VOLT:
            /* no processing */
			break;

		case OEM_CHG_ST_ERR_TIME_OVER:
            /* no processing */
			break;

		case OEM_CHG_ST_ERR_CHARGER:
            /* no processing */
			break;

		default:
			break;
	}

	if (new_chg_state != current_chg_state) {
		switch (new_chg_state) {
			case OEM_CHG_ST_IDLE:
				oem_chg_stop_charging();
				break;

			case OEM_CHG_ST_FAST:
				oem_chg_set_charging();
				break;

			case OEM_CHG_ST_LIMIT:
				oem_chg_init_limit_control();
				break;

			case OEM_CHG_ST_FULL:
				break;

			case OEM_CHG_ST_ERR_BATT_TEMP:
				oem_chg_set_charger_power_supply();
				break;

			case OEM_CHG_ST_ERR_BATT_VOLT:
				oem_chg_set_battery_power_supply();
				break;

			case OEM_CHG_ST_ERR_TIME_OVER:
				oem_chg_set_charger_power_supply();
				break;

			case OEM_CHG_ST_ERR_CHARGER:
				oem_chg_set_battery_power_supply();
				break;

			default:
				break;
		}
		pr_info("chg_state current:%d new:%d\n",current_chg_state, new_chg_state);
		current_chg_state = new_chg_state;
		req_ps_changed_batt |= true;
		oem_chg_reset_timer();
		oem_chg_wake_lock_control();
	}
	return req_ps_changed_batt;
}
EXPORT_SYMBOL(oem_chg_control);


void oem_chg_control_init(void)
{
	wake_lock_init(&oem_chg_wake_lock, WAKE_LOCK_SUSPEND, "oem_chg_control");
	INIT_DELAYED_WORK(&oem_chg_battery_removal_work,
		oem_chg_battery_removal);
	pr_info("CHG_ERR_TIME_SEC:%d[min]\n", CHG_ERR_TIME_SEC/60);
}
EXPORT_SYMBOL(oem_chg_control_init);


