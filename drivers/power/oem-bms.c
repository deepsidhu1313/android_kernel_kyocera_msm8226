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
#define pr_fmt(fmt)	"BMS: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/power_supply.h>
#include <oem-bms.h>

#undef BMS_DEBUG
#ifdef BMS_DEBUG
	#define BMS_LOG pr_err
#else
	#define BMS_LOG pr_debug
#endif

#define LOWER_LIMIT_SOC_CHARGING 0
#define UPPER_LIMIT_SOC_CHARGING 99
#define LOWER_LIMIT_SOC_DISCHARGING 0
#define UPPER_LIMIT_SOC_DISCHARGING 100
#define OEM_SOC_FULL 100


#define VBATT_SAMPLES			20
#define POWER_OFF_SOC			0
#define LOW_BATTERY_SOC			1
#define LOW_BATTERY_CHECK_SOC	10

#define POWER_OFF_THRESHOLD_UV		(2940 * 1000)
#define LOW_BATTERY_THRESHOLD_UV	(2980 * 1000)

#define VBATT_INITIAL_UV			(4200 * 1000)

static int vbatt_current_uv = VBATT_INITIAL_UV;
static int vbatt_buf_index = 0;
static int vbatt_buf[VBATT_SAMPLES] ={

	VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV,
	VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV,
	VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV,
	VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV

};

static struct wake_lock oem_bms_wake_lock;


static int oem_bms_get_batt_property(enum power_supply_property psp)
{
	struct power_supply *psy;
	union power_supply_propval result = {0,};

	psy = power_supply_get_by_name("battery");
	psy->get_property(psy, psp, &result);
	return result.intval;
}


void oem_bms_init(void)
{
	wake_lock_init(&oem_bms_wake_lock, WAKE_LOCK_SUSPEND, "oem_bms");
}

static int oem_bms_bound_soc_in_charging(int soc)
{
	soc = max(LOWER_LIMIT_SOC_CHARGING, soc);
	soc = min(UPPER_LIMIT_SOC_CHARGING, soc);
	return soc;
}

static int oem_bms_bound_soc_in_discharging(int soc)
{
	int in_soc = soc;
	soc = max(LOWER_LIMIT_SOC_DISCHARGING, soc);
	soc = min(UPPER_LIMIT_SOC_DISCHARGING, soc);
	if (in_soc != soc)
		pr_err("soc convert %d to %d\n", in_soc, soc);
	return soc;
}


static atomic_t catch_up_full = ATOMIC_INIT(0);
static int oem_bms_get_catch_up_full(void)
{
	return atomic_read(&catch_up_full);
}

void oem_bms_set_catch_up_full(int on_off)
{
	atomic_set(&catch_up_full, on_off);
}
EXPORT_SYMBOL(oem_bms_set_catch_up_full);

static int oem_bms_get_average(int *buf, int num)
{
	int ave, cnt;
	int max = 0, min = 0, sum = 0;

	max = buf[0];
	min = buf[0];

	for (cnt = 0; cnt < num; cnt++) {
		if (max < buf[cnt]) {
			max = buf[cnt];
		} else if (min > buf[cnt]) {
			min = buf[cnt];
		}
		sum += buf[cnt];
	}

	if (num < 3) {
		ave = sum / num;
	} else {
		ave = (sum - max - min) / (num - 2);
	}

	pr_debug("ave:%d sum:%d max:%d min:%d num:%d\n",
		ave, sum, max, min, num);

	return ave;
}

static void oem_bms_update_vbatt_current(void)
{
	int vbatt_now_uv = oem_bms_get_batt_property(POWER_SUPPLY_PROP_VOLTAGE_NOW);

	if (vbatt_now_uv == -EINVAL) {
		pr_err("invalid value vbatt_now_uv:%d\n", vbatt_now_uv);
		return;
	}
	vbatt_buf_index++;

	if (vbatt_buf_index == VBATT_SAMPLES) {
		vbatt_buf_index = 0;
	}
	vbatt_buf[vbatt_buf_index] = vbatt_now_uv;
	vbatt_current_uv = oem_bms_get_average(vbatt_buf, VBATT_SAMPLES);

	pr_debug("vbatt_buf [0]=%d [1]=%d [2]=%d [3]=%d [4]=%d vbatt_buf_index:%d\n",
		vbatt_buf[0], vbatt_buf[1], vbatt_buf[2], vbatt_buf[3], vbatt_buf[4], vbatt_buf_index);
}

static int oem_bms_check_low_vbatt(int soc, int battery_status)
{
	int result_soc = soc;

	if(vbatt_current_uv <= POWER_OFF_THRESHOLD_UV) {
		result_soc = POWER_OFF_SOC;
		pr_err("set auto power off soc:%d vbatt_uv:%d\n", soc, vbatt_current_uv);
	} else if ((vbatt_current_uv <= LOW_BATTERY_THRESHOLD_UV) &&
				(battery_status != POWER_SUPPLY_STATUS_CHARGING) &&
				(soc <= LOW_BATTERY_CHECK_SOC)) {
		result_soc = LOW_BATTERY_SOC;
	}
	pr_debug("result_soc:%d vbatt_current_uv:%d\n", result_soc, vbatt_current_uv);
	return result_soc;
}



static int pre_state = POWER_SUPPLY_STATUS_DISCHARGING;
static int pre_state_result_soc = 0;
static int new_state_in_soc = 0;
static int pre_result_soc = 0;
static int pre_in_soc = 0;
static int pre_calc_soc = 0;
static int base_calc_soc = 0;
static uint16_t pre_ocv_raw = 0;
static bool catch_up_in_soc_monitor = false;
static int check_in_soc = 0;
static int in_soc_match_cnt = 0;
static int pre_elapsed_time = 0;
static int filter_base_time = 0;
static bool is_initialized = false;
int oem_bms_correct_soc(int in_soc, int calc_soc, uint16_t ocv_raw)
{
	struct timespec ts;
	static struct timespec last_ts;
	int elapsed_time;
	int filter_check_time = SOC_FILTER_TIME_SEC;
	int result_soc = in_soc;
	int pre_state_result_soc_work = pre_state_result_soc;
	int new_state_in_soc_work = new_state_in_soc;
	int alpha = 0;
	int round;
	int batt_uv = oem_bms_get_batt_property(POWER_SUPPLY_PROP_VOLTAGE_NOW);
	int state = oem_bms_get_batt_property(POWER_SUPPLY_PROP_STATUS);

	if (!is_initialized) {
		pre_state_result_soc = in_soc;
		new_state_in_soc = in_soc;
		pre_result_soc = in_soc;
		pre_in_soc = in_soc;
		pre_calc_soc = calc_soc;
		pre_ocv_raw = ocv_raw;
		check_in_soc = in_soc;
		getnstimeofday(&last_ts);
		filter_base_time = last_ts.tv_sec;
		is_initialized = true;
		pr_info("initialized in_soc:%d\n", in_soc);
	}

	if (pre_state != state) {
		pr_info("update pre_state_result_soc %d to %d new_state_in_soc %d to %d\n",
			pre_state_result_soc, pre_result_soc, new_state_in_soc, in_soc);

		pre_state_result_soc = pre_result_soc;
		new_state_in_soc = in_soc;

		if ((state == POWER_SUPPLY_STATUS_DISCHARGING) ||
			(state == POWER_SUPPLY_STATUS_CHARGING)) {
			in_soc_match_cnt = 0;
			pre_elapsed_time = 0;
			check_in_soc = in_soc;
			getnstimeofday(&last_ts);
			filter_base_time = last_ts.tv_sec;
			oem_bms_set_catch_up_full(0);
			catch_up_in_soc_monitor = false;
			wake_lock_timeout(&oem_bms_wake_lock, msecs_to_jiffies((IN_SOC_MONITOR_TIME_SEC + 1) * 1000));
			pr_info("in_soc monitor start wake_lock_timeout:%dsec\n", IN_SOC_MONITOR_TIME_SEC + 1);
		} else {
			wake_unlock(&oem_bms_wake_lock);
		}
	}

	if ((state == POWER_SUPPLY_STATUS_DISCHARGING) ||
		(state == POWER_SUPPLY_STATUS_CHARGING)) {

		if (wake_lock_active(&oem_bms_wake_lock)) {
			getnstimeofday(&ts);
			elapsed_time = ts.tv_sec - last_ts.tv_sec;

			if (elapsed_time > pre_elapsed_time) {
				if(check_in_soc == in_soc) {
					in_soc_match_cnt++;
				} else {
					in_soc_match_cnt = 0;
				}

				if ((in_soc_match_cnt >= IN_SOC_MATCH_CNT) ||
					(elapsed_time >= IN_SOC_MONITOR_TIME_SEC)) {
					pr_info("new_state_in_soc update:%d to %d\n", new_state_in_soc, in_soc);

					new_state_in_soc = in_soc;
					wake_unlock(&oem_bms_wake_lock);
				}
				pr_info("in_soc_monitor\n state old:%d new:%d batt_mv:%d\n result_soc pre_st:%d old:%d\n in_soc new_state:%d old:%d new:%d\n check_in_soc:%d match_cnt:%d elapsed_time:%d wake_lock:%d\n",
					pre_state, state, batt_uv/1000,
					pre_state_result_soc, pre_result_soc,
					new_state_in_soc, pre_in_soc, in_soc,
					check_in_soc, in_soc_match_cnt, elapsed_time,
					wake_lock_active(&oem_bms_wake_lock));

				pre_elapsed_time = elapsed_time;
				check_in_soc = in_soc;
			}
		} else {
			if (state == POWER_SUPPLY_STATUS_DISCHARGING) {
				if (catch_up_in_soc_monitor) {
					if (calc_soc <= in_soc) {
						pr_info("catch up in_soc\n pre_state_result_soc %d to %d\n new_state_in_soc %d to %d\n",
							pre_state_result_soc, pre_result_soc, new_state_in_soc, in_soc);

						pre_state_result_soc = pre_result_soc;
						new_state_in_soc = in_soc;
						catch_up_in_soc_monitor = false;
					} else if (calc_soc > base_calc_soc) {
						/* calc_soc larger than base_calc_soc */
					} else if ((calc_soc - base_calc_soc) <= CALC_SOC_DOWN_DIFF) {
						pr_info("calc_soc change\n pre_state_result_soc %d to %d\n new_state_in_soc %d to %d\n",
							pre_state_result_soc, pre_result_soc, new_state_in_soc, calc_soc);
						pr_info("base_calc_soc %d to %d\n", base_calc_soc, calc_soc);

						pre_state_result_soc = pre_result_soc;
						new_state_in_soc = calc_soc;
						base_calc_soc = calc_soc;
					} else {
						base_calc_soc = calc_soc;
					}
				} else if (calc_soc > in_soc) {
					pr_info("calc_soc larger than in_soc\n pre_state_result_soc %d to %d\n new_state_in_soc %d to %d\n",
						pre_state_result_soc, pre_result_soc, new_state_in_soc, calc_soc);
					pr_info("base_calc_soc %d to %d\n", base_calc_soc, calc_soc);

					pre_state_result_soc = pre_result_soc;
					new_state_in_soc = calc_soc;
					base_calc_soc = calc_soc;
					catch_up_in_soc_monitor = true;

					pr_info("ocv_raw old:%d new:%d calc_soc old:%d new:%d in_soc:%d\n",
						pre_ocv_raw, ocv_raw, pre_calc_soc, calc_soc, in_soc);
				}
			}

			if ((!catch_up_in_soc_monitor) &&
				((pre_in_soc - in_soc) >= IN_SOC_CHECK_DIFF)) {
				pr_info("diff is larger than %d update\n pre_state_result_soc %d to %d\n new_state_in_soc %d to %d pre_in_soc:%d\n",
					IN_SOC_CHECK_DIFF, pre_state_result_soc, pre_result_soc, new_state_in_soc, in_soc, pre_in_soc);

				pre_state_result_soc = pre_result_soc;
				new_state_in_soc = in_soc;
			}
		}
	}

	switch (state) {

	case POWER_SUPPLY_STATUS_FULL:
		result_soc = OEM_SOC_FULL;
		break;

	case POWER_SUPPLY_STATUS_CHARGING:
		if (pre_state_result_soc > CORRECT_SOC_MAX_CHG) {
			pre_state_result_soc_work = CORRECT_SOC_MAX_CHG;
		} else {
			pre_state_result_soc_work = pre_state_result_soc;
		}

		if (new_state_in_soc > CORRECT_SOC_MAX_CHG) {
			new_state_in_soc_work = CORRECT_SOC_MAX_CHG;
		} else {
			new_state_in_soc_work = new_state_in_soc;
		}

		alpha = abs(pre_state_result_soc_work - new_state_in_soc_work);

		if (wake_lock_active(&oem_bms_wake_lock)) {
			result_soc = pre_state_result_soc;
		} else if (alpha > CORRECT_DIFF_MAX) {
			result_soc = in_soc;
			pr_err("alpha is invalid %d\n", alpha);
		} else if (new_state_in_soc_work > in_soc) {
			round = new_state_in_soc_work / 2;
			result_soc = (pre_state_result_soc_work * in_soc + round) / new_state_in_soc_work;
		} else {
			round = (SOC_MAX - new_state_in_soc_work) / 2;
			result_soc = SOC_MAX - ((SOC_MAX - pre_state_result_soc_work) * (SOC_MAX - in_soc) + round) / (SOC_MAX - new_state_in_soc_work);
		}

		if (oem_bms_get_catch_up_full()) {
			getnstimeofday(&ts);
			elapsed_time = ts.tv_sec - filter_base_time;

			if (elapsed_time >= filter_check_time) {
				pr_info("filter_soc:%d result_soc:%d pre_result_soc:%d elapsed_time:%d check_time:%d\n",
					pre_result_soc + 1, result_soc, pre_result_soc, elapsed_time, filter_check_time);
				result_soc = pre_result_soc + 1;
				filter_base_time = ts.tv_sec;

				pr_info("catch_up_full update\n pre_state_result_soc %d to %d\n new_state_in_soc %d to %d\n",
					pre_state_result_soc, result_soc, new_state_in_soc, in_soc);
				oem_bms_set_catch_up_full(0);
				pre_state_result_soc = result_soc;
				new_state_in_soc = in_soc;
			} else {
				result_soc = pre_result_soc;
			}
		} else if (result_soc > pre_result_soc) {
			getnstimeofday(&ts);
			elapsed_time = ts.tv_sec - filter_base_time;

			if (pre_result_soc >= CV_CHG_CHECK_SOC) {
				filter_check_time = CV_SOC_FILTER_TIME_SEC;
			}

			if (elapsed_time >= filter_check_time) {
				pr_info("filter_soc:%d result_soc:%d pre_result_soc:%d elapsed_time:%d check_time:%d\n",
					pre_result_soc + 1, result_soc, pre_result_soc, elapsed_time, filter_check_time);
				result_soc = pre_result_soc + 1;
				filter_base_time = ts.tv_sec;
			} else {
				result_soc = pre_result_soc;
			}
		}

		result_soc = oem_bms_bound_soc_in_charging(result_soc);
		break;

	case POWER_SUPPLY_STATUS_DISCHARGING:
		if (pre_state_result_soc < CORRECT_SOC_MAX_DISCHG) {
			pre_state_result_soc_work = CORRECT_SOC_MAX_DISCHG;
		} else {
			pre_state_result_soc_work = pre_state_result_soc;
		}

		if (new_state_in_soc < CORRECT_SOC_MAX_DISCHG) {
			new_state_in_soc_work = CORRECT_SOC_MAX_DISCHG;
		} else {
			new_state_in_soc_work = new_state_in_soc;
		}

		alpha = abs(pre_state_result_soc_work - new_state_in_soc_work);

		if (wake_lock_active(&oem_bms_wake_lock)) {
			result_soc = pre_state_result_soc;
		} else if (alpha > CORRECT_DIFF_MAX) {
			result_soc = in_soc;
			pr_err("alpha is invalid %d\n", alpha);
		} else if (catch_up_in_soc_monitor) {
			if (calc_soc > base_calc_soc) {
				result_soc = pre_result_soc;
			} else {
				round = new_state_in_soc_work / 2;
				result_soc = (pre_state_result_soc_work * calc_soc + round) / new_state_in_soc_work;
			}
		} else {
			round = new_state_in_soc_work / 2;
			result_soc = (pre_state_result_soc_work * in_soc + round) / new_state_in_soc_work;
		}

		result_soc = oem_bms_bound_soc_in_discharging(result_soc);
		break;

	default:
		pr_err("unknown POWER_SUPPLY_STATUS:%d\n", state);
		break;
	}

	oem_bms_update_vbatt_current();

	result_soc = oem_bms_check_low_vbatt(result_soc, state);

	if ((pre_result_soc != result_soc)	||
		(pre_in_soc != in_soc)	||
		(pre_calc_soc != calc_soc)	||
		(pre_ocv_raw != ocv_raw)	||
		(pre_state != state)) {
		pr_info("result_soc:%d A:%d B:%d X:%d alpha:%d\n",
			result_soc, pre_state_result_soc_work, new_state_in_soc_work, in_soc, alpha);
		pr_info("change\n state old:%d new:%d batt_mv:%d\n result_soc pre_st:%d old:%d new:%d\n in_soc new_state:%d old:%d new:%d\n",
			pre_state, state, batt_uv/1000,
			pre_state_result_soc, pre_result_soc, result_soc,
			new_state_in_soc, pre_in_soc, in_soc);
		pr_info("ocv_raw old:%d new:%d calc_soc old:%d new:%d in_soc:%d catch_up_in_soc_monitor:%d\n",
			pre_ocv_raw, ocv_raw, pre_calc_soc, calc_soc, in_soc, catch_up_in_soc_monitor);
	}
	pre_result_soc = result_soc;
	pre_in_soc = in_soc;
	pre_calc_soc = calc_soc;
	pre_ocv_raw = ocv_raw;
	pre_state = state;

	return result_soc;
}
EXPORT_SYMBOL(oem_bms_correct_soc);


