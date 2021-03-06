/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 */
/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#include <linux/batterydata-lib.h>

static struct single_row_lut fcc_temp = {
	
	.x		= {-20, -10, 0, 10, 20, 44, 45, 60},
	.y		= {1000, 2000, 3400, 3500, 3700, 3700, 3400, 3400},
	.cols	= 8
	
};

static struct pc_temp_ocv_lut pc_temp_ocv = {
	.rows		= 29,
	
	.cols		= 8,
	.temp		= {-20, -10, 0, 10, 20, 44, 45, 60},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40,
					35, 30, 25, 20, 15, 10, 9, 8, 7, 6, 5,
					4, 3, 2, 1, 0},
	.ocv		= {
		
				{3960, 3960, 3960, 3960, 3960, 3960, 3860, 3860},
				{3913, 3910, 3908, 3905, 3905, 3905, 3829, 3829},
				{3886, 3880, 3876, 3870, 3870, 3870, 3798, 3798},
				{3859, 3850, 3843, 3835, 3835, 3835, 3767, 3767},
				{3832, 3820, 3810, 3800, 3800, 3800, 3736, 3736},
				{3805, 3790, 3778, 3765, 3765, 3765, 3705, 3705},
				{3778, 3760, 3746, 3730, 3730, 3730, 3674, 3674},
				{3751, 3730, 3714, 3695, 3695, 3695, 3643, 3643},
				{3724, 3700, 3682, 3660, 3660, 3660, 3612, 3612},
				{3697, 3670, 3650, 3625, 3625, 3625, 3580, 3580},
				{3670, 3640, 3618, 3590, 3590, 3590, 3549, 3549},
				{3643, 3610, 3585, 3555, 3555, 3555, 3518, 3518},
				{3615, 3580, 3553, 3520, 3520, 3520, 3487, 3487},
				{3588, 3550, 3520, 3485, 3485, 3485, 3456, 3456},
				{3561, 3520, 3488, 3450, 3450, 3450, 3425, 3425},
				{3534, 3490, 3455, 3415, 3415, 3415, 3394, 3394},
				{3507, 3460, 3422, 3380, 3380, 3380, 3362, 3362},
				{3480, 3430, 3390, 3345, 3345, 3345, 3331, 3331},
				{3450, 3400, 3350, 3300, 3300, 3300, 3300, 3300},
				{3415, 3370, 3325, 3276, 3276, 3276, 3276, 3276},
				{3380, 3340, 3300, 3254, 3254, 3254, 3254, 3254},
				{3345, 3310, 3275, 3232, 3232, 3232, 3232, 3232},
				{3310, 3280, 3250, 3210, 3210, 3210, 3210, 3210},
				{3275, 3250, 3225, 3188, 3188, 3188, 3188, 3188},
				{3240, 3220, 3200, 3166, 3166, 3166, 3166, 3166},
				{3205, 3190, 3175, 3144, 3144, 3144, 3144, 3144},
				{3170, 3160, 3150, 3122, 3122, 3122, 3122, 3122},
				{3100, 3100, 3100, 3100, 3100, 3100, 3100, 3100},
				{3050, 3050, 3050, 3050, 3050, 3050, 3050, 3050}
		
	
	}
};

static struct sf_lut rbatt_sf = {
	.rows		= 29,
	.cols		= 5,
	/* row_entries are temperature */
	.row_entries	= {-20, 0, 20, 40, 65},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40,
					35, 30, 25, 20, 15, 10, 9, 8, 7, 6, 5,
					4, 3, 2, 1, 0},
	.sf		= {
	
				{731, 261, 100, 92, 92},
				{731, 261, 100, 92, 92},
				{726, 254, 102, 92, 90},
				{722, 254, 102, 92, 90},
				{721, 249, 103, 90, 92},
				{715, 250, 102, 90, 92},
				{714, 250, 102, 92, 92},
				{713, 250, 103, 92, 92},
				{712, 252, 103, 92, 92},
				{716, 254, 101, 92, 92},
				{715, 256, 103, 92, 93},
				{714, 258, 103, 93, 92},
				{717, 265, 105, 92, 93},
				{720, 265, 103, 94, 93},
				{721, 271, 105, 93, 93},
				{722, 279, 104, 92, 94},
				{723, 286, 105, 93, 94},
				{727, 297, 106, 94, 95},
				{727, 314, 108, 92, 95},
				{728, 315, 106, 95, 93},
				{729, 320, 107, 92, 93},
				{731, 325, 110, 93, 95},
				{729, 327, 109, 94, 94},
				{731, 329, 109, 95, 95},
				{731, 331, 112, 96, 95},
				{731, 335, 113, 97, 94},
				{729, 336, 116, 97, 95},
				{732, 341, 116, 97, 95},
				{733, 344, 118, 99, 97},
	
	}
};

struct bms_battery_data oem_batt_data = {
	.fcc			= 3700, 
	.fcc_temp_lut		= &fcc_temp,
	.pc_temp_ocv_lut	= &pc_temp_ocv,
	.rbatt_sf_lut		= &rbatt_sf,
	.default_rbatt_mohm	= 153, 
	.flat_ocv_threshold_uv	= 3600000, 
};
