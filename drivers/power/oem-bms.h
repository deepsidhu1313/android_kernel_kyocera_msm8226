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

#ifndef __OEM_BMS_H
#define __OEM_BMS_H


#define SOC_MAX		100
#define CORRECT_SOC_MAX_CHG			99
#define CORRECT_SOC_MAX_DISCHG		1

#define CORRECT_DIFF_MAX			100


#define IN_SOC_MONITOR_TIME_SEC		60
#define IN_SOC_MATCH_CNT			5
#define IN_SOC_CHECK_DIFF			2

#define SOC_FILTER_TIME_SEC			60
#define FULL_CHG_CHECK_SOC			99
#define CV_SOC_FILTER_TIME_SEC		(7 * 60)
#define CV_CHG_CHECK_SOC			96
#define CALC_SOC_DOWN_DIFF			(-2)

/**
 * oem_bms_correct_soc - readjustment oem soc
 */
extern int oem_bms_correct_soc(int in_soc, int calc_soc, uint16_t ocv_raw);

extern void oem_bms_init(void);
extern void oem_bms_set_catch_up_full(int on_off);
#endif
