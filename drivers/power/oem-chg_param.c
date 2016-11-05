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

#include <linux/debugfs.h>
#include <linux/slab.h>
#include <mach/msm_smsm.h>
#include <oem-chg_param.h>
#include <mach/oem_fact.h>


int oem_option_item1_bit0 = 0;
int oem_option_item1_bit1 = 0;
int oem_option_item1_bit2 = 0;
int oem_option_item1_bit3 = 0;
int oem_option_item1_bit4 = 0;

static void oem_get_fact_option(void)
{
	oem_option_item1_bit0 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 0);
	oem_option_item1_bit1 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 1);
	oem_option_item1_bit2 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 2);
	oem_option_item1_bit3 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 3);
	oem_option_item1_bit4 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 4);

	pr_info("factory option item1 bit 0:%d 1:%d 2:%d 3:%d 4:%d\n",
		oem_option_item1_bit0, oem_option_item1_bit1, oem_option_item1_bit2,
		oem_option_item1_bit3, oem_option_item1_bit4);
}

oem_chg_param_hkadc_type oem_param_hkadc = {
	.cal_vbat1 = 0x0010C8E0,
	.cal_vbat2 = 0x00155CC0
};

oem_chg_param_charger_type oem_param_charger = {
	.reserve1_1 = 0xFFFF,
	.reserve1_2 = 0xFFFF,
	.reserve1_3 = 0xFFFF,
	.reserve1_4 = 0xFFFF,
	.reserve1_5 = 0xFFFF,
	.reserve1_6 = 0xFFFF,
	.reserve1_7 = 0xFFFF,
	.reserve1_8 = 0xFFFF,
	.reserve2_1 = 0xFFFF,
	.reserve2_2 = 0xFFFF,
	.reserve2_3 = 0xFFFF,
	.reserve2_4 = 0xFFFF,
	.reserve2_5 = 0xFFFF,
	.reserve2_6 = 0xFFFF,
	.reserve2_7 = 0xFFFF,
	.reserve2_8 = 0xFFFF,
	.reserve3_1 = 0xFFFF,
	.reserve3_2 = 0xFFFF,
	.reserve3_3 = 0xFFFF,
	.reserve3_4 = 0xFFFF,
	.reserve3_5 = 0xFFFF,
	.reserve3_6 = 0xFFFF,
	.reserve3_7 = 0xFFFF,
	.reserve3_8 = 0xFFFF,
	.reserve4_1 = 0xFFFF,
	.reserve4_2 = 0xFFFF,
	.reserve4_3 = 0xFFFF,
	.reserve4_4 = 0xFFFF,
	.reserve4_5 = 0xFFFF,
	.reserve4_6 = 0xFFFF,
	.reserve4_7 = 0xFFFF,
	.reserve4_8 = 0xFFFF,
	.reserve5_1 = 0xFFFF,
	.reserve5_2 = 0xFFFF,
	.reserve5_3 = 0xFFFF,
	.reserve5_4 = 0xFFFF,
	.reserve5_5 = 0xFFFF,
	.reserve5_6 = 0xFFFF,
	.reserve5_7 = 0xFFFF,
	.reserve5_8 = 0xFFFF,
	.reserve6_1 = 0xFFFF,
	.reserve6_2 = 0xFFFF,
	.reserve6_3 = 0xFFFF,
	.reserve6_4 = 0xFFFF,
	.reserve6_5 = 0xFFFF,
	.reserve6_6 = 0xFFFF,
	.reserve6_7 = 0xFFFF,
	.reserve6_8 = 0xFFFF,
	.reserve7_1 = 0xFFFF,
	.reserve7_2 = 0xFFFF,
	.reserve7_3 = 0xFFFF,
	.reserve7_4 = 0xFFFF,
	.reserve7_5 = 0xFFFF,
	.reserve7_6 = 0xFFFF,
	.reserve7_7 = 0xFFFF,
	.reserve7_8 = 0xFFFF,
	.reserve8_1 = 0xFFFF,
	.reserve8_2 = 0xFFFF,
	.reserve8_3 = 0xFFFF,
	.reserve8_4 = 0xFFFF,
	.reserve8_5 = 0xFFFF,
	.reserve8_6 = 0xFFFF,
	.reserve8_7 = 0xFFFF,
	.reserve8_8 = 0xFFFF,
	.reserve9_1 = 0xFFFF,
	.reserve9_2 = 0xFFFF,
	.reserve9_3 = 0xFFFF,
	.reserve9_4 = 0xFFFF,
	.reserve9_5 = 0xFFFF,
	.reserve9_6 = 0xFFFF,
	.reserve9_7 = 0xFFFF,
	.reserve9_8 = 0xFFFF
};

oem_chg_param_share_type oem_param_share = {
	.factory_mode_1 = 0x00,
	.test_mode_chg_disable = 0x00,
	.cool_down_mode_disable = 0x00,	
	.reserve1_4 = 0xFF,
	.reserve1_5 = 0xFF,
	.reserve1_6 = 0xFF,
	.reserve1_7 = 0xFF,
	.reserve1_8 = 0xFF
};

typedef struct {
	oem_chg_param_hkadc_type		oem_chg_param_hkadc;
	oem_chg_param_charger_type		oem_chg_param_charger;
	oem_chg_param_share_type		oem_chg_param_share;
} oem_chg_param_type;

uint8_t oem_cmp_zero_flag = 0;
static void oem_param_hkadc_init(oem_chg_param_type *ptr)
{
	if (ptr == NULL){
		pr_err("chg param hkadc read error.\n");
		return;
	}

	SET_CHG_PARAM2(0x00000000, 0xFFFFFFFF, hkadc, cal_vbat1);
	SET_CHG_PARAM2(0x00000000, 0xFFFFFFFF, hkadc, cal_vbat2);
}

static void oem_param_charger_init(oem_chg_param_type *ptr)
{

	if (ptr == NULL){
		pr_err("chg param charger read error.\n");
		return;
	}
}

static void oem_param_share_init(oem_chg_param_type *ptr)
{
	if (ptr == NULL){
		pr_err("chg param share read error.\n");
		return;
	}

	memcpy(&oem_param_share, &ptr->oem_chg_param_share,
				sizeof(oem_chg_param_share_type));
}

static int init_flag = 0;
void oem_chg_param_init(void)
{
	uint32_t *smem_ptr = NULL;
	uint32_t *cmp_ptr  = NULL;

	if (init_flag) {
		return;
	}

	smem_ptr = (uint32_t *)kc_smem_alloc(SMEM_CHG_PARAM, CHG_PARAM_SIZE);
	if (smem_ptr == NULL) {
		pr_err("chg param read error.\n");
		init_flag = 1;
		return;
	}

	cmp_ptr = kmalloc((CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE), GFP_KERNEL);
	if (cmp_ptr == NULL) {
		pr_err("kmalloc error.\n");
		init_flag = 1;
		return;
	}

	memset(cmp_ptr, 0x00, (CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE));
	if (0 == memcmp(smem_ptr, cmp_ptr, (CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE))) {
		pr_err("smem data all '0'\n");
		memset(smem_ptr, 0xFF, (CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE));
		oem_cmp_zero_flag = 1;
	}

	oem_param_hkadc_init((oem_chg_param_type*)smem_ptr);
	oem_param_charger_init((oem_chg_param_type*)smem_ptr);
	oem_param_share_init((oem_chg_param_type*)smem_ptr);

	kfree(cmp_ptr);
	init_flag = 1;

	oem_get_fact_option();

}
