/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 */
/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/msm_mdp.h>
#include "disp_ext.h"
#include "mdss_dsi.h"

#define LCDBL_DEBUG         0

#if LCDBL_DEBUG
#define LCDBL_DEBUG_LOG( msg, ... ) \
	pr_notice("[LCDBL][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define LCDBL_DEBUG_LOG( msg, ... ) \
	pr_debug("[LCDBL][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#endif

#define LCDBL_ERR_LOG( msg, ... ) \
	pr_err("[LCDBL][%s][E](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define LCDBL_WARN_LOG( msg, ... ) \
	pr_warn("[LCDBL][%s][W](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define LCDBL_NOTICE_LOG( msg, ... ) \
	pr_notice("[LCDBL][%s][N](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)


#define LCDBL_LEVEL_MAX 255
#define LCDBL_IDX_PWM   0
#define LCDBL_IDX_S2C   1

static const u8 disp_ext_lcdbl_level_conv_table[256][2] =
{
	{	0,     0	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    23	},
	{	29,    22	},
	{	29,    22	},
	{	29,    22	},
	{	29,    22	},
	{	29,    22	},
	{	29,    22	},
	{	29,    21	},
	{	29,    21	},
	{	29,    21	},
	{	29,    21	},
	{	29,    21	},
	{	29,    21	},
	{	29,    21	},
	{	29,    21	},
	{	29,    21	},
	{	29,    21	},
	{	29,    20	},
	{	29,    20	},
	{	29,    20	},
	{	29,    20	},
	{	29,    20	},
	{	29,    20	},
	{	29,    20	},
	{	29,    20	},
	{	29,    20	},
	{	29,    19	},
	{	29,    19	},
	{	29,    19	},
	{	29,    19	},
	{	29,    19	},
	{	29,    19	},
	{	29,    19	},
	{	29,    18	},
	{	29,    18	},
	{	29,    18	},
	{	29,    18	},
	{	29,    18	},
	{	29,    18	},
	{	29,    17	},
	{	29,    17	},
	{	29,    17	},
	{	29,    17	},
	{	29,    16	},
	{	29,    16	},
	{	29,    16	},
	{	29,    16	},
	{	29,    16	},
	{	29,    15	},
	{	29,    15	},
	{	29,    15	},
	{	29,    15	},
	{	29,    14	},
	{	29,    14	},
	{	29,    14	},
	{	29,    14	},
	{	29,    13	},
	{	29,    13	},
	{	29,    13	},
	{	29,    12	},
	{	29,    12	},
	{	29,    12	},
	{	29,    12	},
	{	29,    12	},
	{	29,    12	},
	{	29,    11	},
	{	29,    11	},
	{	29,    11	},
	{	29,    10	},
	{	29,    10	},
	{	29,    10	},
	{	29,    9	},
	{	29,    9	},
	{	29,    8	},
	{	29,    8	},
	{	29,    7	},
	{	29,    7	},
	{	29,    6	},
	{	29,    6	},
	{	29,    5	},
	{	29,    5	},
	{	29,    4	},
	{	29,    4	},
	{	29,    3	},
	{	29,    3	},
	{	29,    2	},
	{	29,    2	},
	{	29,    1	},
	{	29,    1	},
	{	29,    1	},
	{	30,    1	},
	{	31,    1	},
	{	32,    1	},
	{	33,    1	},
	{	34,    1	},
	{	34,    1	},
	{	35,    1	},
	{	35,    1	},
	{	36,    1	},
	{	36,    1	},
	{	37,    1	},
	{	37,    1	},
	{	38,    1	},
	{	38,    1	},
	{	39,    1	},
	{	40,    1	},
	{	41,    1	},
	{	42,    1	},
	{	43,    1	},
	{	44,    1	},
	{	45,    1	},
	{	46,    1	},
	{	47,    1	},
	{	48,    1	},
	{	49,    1	},
	{	50,    1	},
	{	51,    1	},
	{	52,    1	},
	{	53,    1	},
	{	54,    1	},
	{	55,    1	},
	{	56,    1	},
	{	57,    1	},
	{	58,    1	},
	{	59,    1	},
	{	60,    1	},
	{	61,    1	},
	{	62,    1	},
	{	63,    1	},
	{	64,    1	},
	{	65,    1	},
	{	66,    1	},
	{	67,    1	},
	{	68,    1	},
	{	69,    1	},
	{	70,    1	},
	{	71,    1	},
	{	72,    1	},
	{	73,    1	},
	{	74,    1	},
	{	76,    1	},
	{	78,    1	},
	{	79,    1	},
	{	81,    1	},
	{	83,    1	},
	{	85,    1	},
	{	87,    1	},
	{	88,    1	},
	{	90,    1	},
	{	92,    1	},
	{	94,    1	},
	{	96,    1	},
	{	98,    1	},
	{	100,   1	},
	{	101,   1	},
	{	103,   1	},
	{	105,   1	},
	{	107,   1	},
	{	109,   1	},
	{	111,   1	},
	{	113,   1	},
	{	115,   1	},
	{	117,   1	},
	{	119,   1	},
	{	121,   1	},
	{	123,   1	},
	{	125,   1	},
	{	127,   1	},
	{	129,   1	},
	{	131,   1	},
	{	133,   1	},
	{	135,   1	},
	{	137,   1	},
	{	139,   1	},
	{	141,   1	},
	{	143,   1	},
	{	145,   1	},
	{	147,   1	},
	{	149,   1	},
	{	151,   1	},
	{	153,   1	},
	{	155,   1	},
	{	157,   1	},
	{	159,   1	},
	{	161,   1	},
	{	163,   1	},
	{	165,   1	},
	{	167,   1	},
	{	169,   1	},
	{	171,   1	},
	{	173,   1	},
	{	175,   1	},
	{	177,   1	},
	{	179,   1	},
	{	181,   1	},
	{	183,   1	},
	{	185,   1	},
	{	187,   1	},
	{	189,   1	},
	{	191,   1	},
	{	193,   1	},
	{	195,   1	},
	{	197,   1	},
	{	199,   1	},
	{	201,   1	},
	{	203,   1	},
	{	205,   1	},
	{	207,   1	},
	{	209,   1	},
	{	211,   1	},
	{	213,   1	},
	{	215,   1	},
	{	217,   1	},
	{	219,   1	},
	{	221,   1	},
	{	223,   1	},
	{	225,   1	},
	{	227,   1	},
	{	229,   1	},
	{	231,   1	},
	{	233,   1	},
};

static DEFINE_SPINLOCK(lcdbl_s2c_lock);
static void disp_ext_lcdbl_gpio_pulse(int gpio, int pulse)
{
	int i;
	unsigned long flags;

	LCDBL_DEBUG_LOG("[IN] gpio=%d pulse=%d", gpio, pulse);

	spin_lock_irqsave(&lcdbl_s2c_lock, flags);
	for (i = 0; i < pulse; i++) {
		gpio_set_value(gpio ,0);
		gpio_set_value(gpio ,1);
	}
	spin_unlock_irqrestore(&lcdbl_s2c_lock, flags);
	udelay(750);

	LCDBL_DEBUG_LOG("[OUT]");
}

#ifdef CONFIG_DISP_EXT_LCDBL_DEBUG
static int disp_ext_lcdbl_lcdbl_en_gpio_saved = -EINVAL;
static void disp_ext_lcdbl_save_lcdbl_en_gpio(int gpio)
{
	LCDBL_DEBUG_LOG("[IN/OUT]");
	disp_ext_lcdbl_lcdbl_en_gpio_saved = gpio;
}

static int disp_ext_lcdbl_s2c_out_ctrl(const char *val, struct kernel_param *kp)
{
	int ret;
	long s2c = 0;
	int gpio = disp_ext_lcdbl_lcdbl_en_gpio_saved;

	LCDBL_DEBUG_LOG("[IN]");

	ret = kstrtol(val, 0, &s2c);
	if (ret) {
		LCDBL_WARN_LOG("invalid param. ret=%d", ret);
		return ret;
	}

	if (s2c < 1 || 32 < s2c) {
		LCDBL_WARN_LOG("invalid range. s2c=%ld", s2c);
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio)) {
		LCDBL_WARN_LOG("lcdbl en gpio is not specified. gpio=%d", gpio);
		return -EINVAL;
	}

	disp_ext_lcdbl_gpio_pulse(gpio, s2c);

	LCDBL_DEBUG_LOG("[OUT]");

	return ret;
}
module_param_call(lcdbl_s2c_out, disp_ext_lcdbl_s2c_out_ctrl,
					NULL, NULL, 0200);
#else
static inline void disp_ext_lcdbl_save_lcdbl_en_gpio(int gpio) {}
#endif

int disp_ext_lcdbl_get_lcdbl_en_gpio(struct device_node *node)
{
	int gpio;

	LCDBL_DEBUG_LOG("[IN]");

	gpio = of_get_named_gpio(node, "kc,lcdbl-en-gpio", 0);
	if (!gpio_is_valid(gpio)) {
		LCDBL_NOTICE_LOG("lcdbl en gpio is not specified");
	} else {
		LCDBL_DEBUG_LOG("lcdbl en gpio[%d] found, but not assigned here.", gpio);
	}

	disp_ext_lcdbl_save_lcdbl_en_gpio(gpio);
	LCDBL_DEBUG_LOG("[OUT]");

	return gpio;
}

u32 disp_ext_lcdbl_gpio_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, u32 level)
{
	int rc ;
	u8 s2c = 0, pwm = 0;
	static u8 pre_s2c = 0;
	static u32 pre_level = 0;

	LCDBL_DEBUG_LOG("[IN]");

#ifdef CONFIG_DISP_EXT_BOARD
	if (disp_ext_board_get_panel_detect() != 1) {
		return(level);
	}
#endif /* CONFIG_DISP_EXT_BOARD */
	if (!ctrl || !gpio_is_valid(ctrl->lcdbl_en_gpio)) {
		LCDBL_DEBUG_LOG("lcdbl en gpio is not specified");
		return(level);
	}

	if (level == 0) {
		gpio_set_value(ctrl->lcdbl_en_gpio, 0);
		pre_s2c = 0;
		if (pre_level != 0) {
			gpio_free(ctrl->lcdbl_en_gpio);
			LCDBL_DEBUG_LOG("lcdbl en gpio[%d], free.", ctrl->lcdbl_en_gpio);
		}
	} else {
		if (level > LCDBL_LEVEL_MAX) {
			level = LCDBL_LEVEL_MAX;
		}
		if (pre_level == 0) {
			rc = gpio_request(ctrl->lcdbl_en_gpio, "lcdbl_en");
			if (rc) {
				LCDBL_ERR_LOG("request lcdbl en gpio failed, rc=%d", rc);
				return(level);
			}
			LCDBL_DEBUG_LOG("lcdbl en gpio[%d], assigned.", ctrl->lcdbl_en_gpio);
		}
		s2c = disp_ext_lcdbl_level_conv_table[level][LCDBL_IDX_S2C];
		pwm = disp_ext_lcdbl_level_conv_table[level][LCDBL_IDX_PWM];
		if (s2c != pre_s2c) {
			disp_ext_lcdbl_gpio_pulse(ctrl->lcdbl_en_gpio, s2c);
			pre_s2c = s2c;
		}
		if (!pre_level) {
			usleep_range(15000, 16000);
		}
	}
	pre_level = level;

	LCDBL_DEBUG_LOG("[OUT] level:%d s2c:%d pwm:%d", level, s2c, pwm);

	return pwm;
}
