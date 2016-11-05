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
#ifndef OEM_CHG_COMTROL_H
#define OEM_CHG_COMTROL_H

#include <linux/qpnp/qpnp-adc.h>

#define CHG_WAKE_LOCK_TIMEOUT_MS	(2000)

#define CHG_ERR_TIME_SEC		(12 * 60 * 60)



#define BATT_VOLT_POWER_ON_OK	(3300 * 1000)
#define BATT_VOLT_POWER_ON_NG	(3200 * 1000)
#define BATT_VOLT_ERR_CHG_OFF	(4100 * 1000)


#define BATT_REMOVAL_CHECK_TIME	50

#define LIMIT_CHG_ON_TIME	(15 * 1000)
#define LIMIT_CHG_OFF_TIME	(20 * 1000)

#define BATT_VOLT_LIMIT_CHG_ON		(3800 * 1000)
#define BATT_VOLT_LIMIT_CHG_OFF		(3500 * 1000)

#define FACTORY_TEMP_CHG_OFF		(95 * 10)
#define FACTORY_TEMP_LIMIT_CHG_ON	(90 * 10)

#define BATT_TEMP_HOT_CHG_OFF		(52 * 10)
#define BATT_TEMP_HOT_CHG_ON		(50 * 10)
#define BATT_TEMP_COLD_CHG_OFF		(-5)
#define BATT_TEMP_COLD_CHG_ON		(0)
#define BATT_TEMP_W_CHG_HOT_CHG_OFF	(56 * 10)
#define BATT_TEMP_W_CHG_HOT_CHG_ON	(54 * 10)

#define BATT_TEMP_LIMIT_ON			(45 * 10)
#define BATT_TEMP_LIMIT_OFF			(43 * 10)

#define PMA_TEMP_HOT_EOC			(57 * 10)

#define USB_THERM_ALERT_ON	(80)
#define USB_THERM_ALERT_OFF	(68)
#define PA_THERM_USB_ALERT	(50)
#define CAM_THERM_USB_ALERT	(50)


#define CHG_DNAND_CMD			18


#define BATT_SOC_LIMIT_OFF		90

typedef enum {
	OEM_CHG_ST_IDLE,
	OEM_CHG_ST_FAST,
	OEM_CHG_ST_FULL,
	OEM_CHG_ST_LIMIT,
	OEM_CHG_ST_ERR_BATT_TEMP,
	OEM_CHG_ST_ERR_BATT_VOLT,
	OEM_CHG_ST_ERR_TIME_OVER,
	OEM_CHG_ST_ERR_CHARGER,
	OEM_CHG_ST_INVALID
} oem_chg_state_type;



enum oem_chg_volt_status {
	OEM_ST_VOLT_LOW = 0,
	OEM_ST_VOLT_NORM,
	OEM_ST_VOLT_HIGH
};

enum oem_chg_temp_status {
	OEM_ST_TEMP_NORM = 0,
	OEM_ST_TEMP_LIMIT,
	OEM_ST_TEMP_STOP
};

extern void oem_chg_control_init(void);

extern int oem_chg_control(void);

extern void oem_chg_set_battery_power_supply(void);

extern void oem_chg_stop_charging(void);

extern int oem_chg_get_current_batt_status(void);

extern int oem_chg_get_current_batt_health(void);

extern int oem_chg_get_current_batt_voltage(void);

extern int oem_chg_get_current_capacity(void);

extern int oem_chg_get_current_batt_temp(void);

extern int oem_chg_get_current_pa_therm(void);

extern int oem_chg_get_current_chg_connect(void);

extern int oem_chg_get_current_temp_status(void);

extern int oem_chg_get_current_volt_status(void);

extern int oem_chg_get_usb_alert_status(void);

extern void oem_chg_ps_changed_batt(void);

extern int oem_chg_batt_fet_ctrl(int enable);

extern int oem_chg_buck_ctrl(int disable);

extern int oem_chg_vadc_read(enum qpnp_vadc_channels channel,
				struct qpnp_vadc_result *result);

extern void oem_batt_led_init(void);

extern void oem_chg_set_pma_eoc(bool value);

extern u8 oem_chg_get_bat_fet_status(void);
#endif
