/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/leds.h>
#include <oem-chg_control.h>
#include <mach/board.h>
#include <linux/qpnp/power-on.h>

#define LOW_BATT_CAPACITY           15
#define LOW_BATT_LED_ON_TIME        250
#define LOW_BATT_LED_OFF_TIME       4750
#define USB_ALERT_LED_ON_TIME       250
#define USB_ALERT_LED_OFF_TIME      250

enum oem_batt_led_status
{
	OEM_BATT_LED_ST_INVALID,
	OEM_BATT_LED_ST_OFF,
	OEM_BATT_LED_ST_LOW_BATTERY,
	OEM_BATT_LED_ST_CHG_FAST,
	OEM_BATT_LED_ST_USB_ALERT,
	OEM_BATT_LED_ST_CHG_FULL
};

static struct delayed_work oem_batt_led_work;

static void set_led_param_off(struct led_control_ex_data *led_ctrl,
				enum oem_batt_led_status *led_st)
{
	led_ctrl->priority   = LED_PRIORITY_BATTERY;
	led_ctrl->color      = LED_COLOR_OFF;
	led_ctrl->mode       = LED_BLINK_OFF;
	led_ctrl->pattern[0] = 0;
	*led_st = OEM_BATT_LED_ST_OFF;
}

static void set_led_param_chg_fast(struct led_control_ex_data *led_ctrl,
				enum oem_batt_led_status *led_st)
{
	led_ctrl->priority   = LED_PRIORITY_BATTERY;
	led_ctrl->color      = LED_COLOR_RED;
	led_ctrl->mode       = LED_BLINK_OFF;
	led_ctrl->pattern[0] = 0;
	*led_st = OEM_BATT_LED_ST_CHG_FAST;
}

static void set_led_param_chg_full(struct led_control_ex_data *led_ctrl,
				enum oem_batt_led_status *led_st)
{
	led_ctrl->priority   = LED_PRIORITY_BATTERY;
	led_ctrl->color      = LED_COLOR_GREEN;
	led_ctrl->mode       = LED_BLINK_OFF;
	led_ctrl->pattern[0] = 0;
	*led_st = OEM_BATT_LED_ST_CHG_FULL;
}

static void set_led_param_low_batt(struct led_control_ex_data *led_ctrl,
				enum oem_batt_led_status *led_st)
{
	led_ctrl->priority   = LED_PRIORITY_BATTERY;
	led_ctrl->color      = LED_COLOR_RED;
	led_ctrl->mode       = LED_BLINK_ON;
	led_ctrl->pattern[0] = LOW_BATT_LED_ON_TIME;
	led_ctrl->pattern[1] = LOW_BATT_LED_OFF_TIME;
	led_ctrl->pattern[2] = 0;
	*led_st = OEM_BATT_LED_ST_LOW_BATTERY;
}

static void set_led_param_usb_alert(struct led_control_ex_data *led_ctrl,
				enum oem_batt_led_status *led_st)
{
	led_ctrl->priority   = LED_PRIORITY_BATTERY;
	led_ctrl->color      = LED_COLOR_RED;
	led_ctrl->mode       = LED_BLINK_ON;
	led_ctrl->pattern[0] = USB_ALERT_LED_ON_TIME;
	led_ctrl->pattern[1] = USB_ALERT_LED_OFF_TIME;
	led_ctrl->pattern[2] = 0;
	*led_st = OEM_BATT_LED_ST_USB_ALERT;
}

static void oem_batt_led_control(void)
{
	struct led_control_ex_data led_ctrl;
	static enum oem_batt_led_status current_led_st = OEM_BATT_LED_ST_INVALID;
	enum oem_batt_led_status new_led_st = OEM_BATT_LED_ST_INVALID;
	int batt_status;
	int batt_capacity;
	int batt_volt_status;

	batt_status = oem_chg_get_current_batt_status();
	batt_capacity = oem_chg_get_current_capacity();
	batt_volt_status = oem_chg_get_current_volt_status();

	switch (batt_status)
	{
		case POWER_SUPPLY_STATUS_DISCHARGING:
			if (oem_is_off_charge())
			{
				set_led_param_off(&led_ctrl, &new_led_st);
			}
			else
			{
				if (LOW_BATT_CAPACITY > batt_capacity)
				{
					set_led_param_low_batt(&led_ctrl, &new_led_st);
				}
				else
				{
					set_led_param_off(&led_ctrl, &new_led_st);
				}
			}
			break;

		case POWER_SUPPLY_STATUS_CHARGING:
			set_led_param_chg_fast(&led_ctrl, &new_led_st);
			break;

		case POWER_SUPPLY_STATUS_FULL:
			set_led_param_chg_full(&led_ctrl, &new_led_st);
			break;

		case POWER_SUPPLY_STATUS_NOT_CHARGING:

			/* set_led_param_off(&led_ctrl, &new_led_st); */
			current_led_st = OEM_BATT_LED_ST_INVALID;
			new_led_st = OEM_BATT_LED_ST_INVALID;
			pr_debug("Skipped LED control for not charging state.\n");

			break;

		default:
			new_led_st = current_led_st;
			pr_err("batt_status = %d\n" ,batt_status);
			break;
	}

	
	if (oem_chg_get_usb_alert_status() && oem_is_off_charge()) {
		set_led_param_usb_alert(&led_ctrl, &new_led_st);
	}

	if (current_led_st != new_led_st)
	{
		qpnp_set_leddata_ex(&led_ctrl);
		current_led_st = new_led_st;
		pr_info("led_st=%d batt_st=%d cap=%d vbatt_st=%d\n",
			current_led_st, batt_status, batt_capacity, batt_volt_status);

		
		if (new_led_st == OEM_BATT_LED_ST_CHG_FULL && oem_is_off_charge()) {
			pr_err("%s(%d):>>oem_notify_power_key_event(KEY_POWER)\n", __func__, __LINE__);
			oem_notify_power_key_event(KEY_POWER, 1);
		}
		
	}
}

static void oem_batt_led_monitor(struct work_struct *work)
{
	oem_batt_led_control();
	schedule_delayed_work(&oem_batt_led_work,
			round_jiffies_relative(msecs_to_jiffies(1000)));
}

void oem_batt_led_init(void)
{
	INIT_DELAYED_WORK(&oem_batt_led_work, oem_batt_led_monitor);
	schedule_delayed_work(&oem_batt_led_work,
			round_jiffies_relative(msecs_to_jiffies(0)));
}
EXPORT_SYMBOL(oem_batt_led_init);
