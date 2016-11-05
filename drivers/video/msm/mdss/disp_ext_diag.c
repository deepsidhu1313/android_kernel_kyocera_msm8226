/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 *
 * drivers/video/msm/mdss/disp_ext_diag.c
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include "disp_ext.h"

#define ACK_ERROR_BIT_MASK0 0x0000FF00
#define ACK_ERROR_BIT_MASK1 0x000000FF

#define CHECK_ERROR_BITS 16

static char error_count[CHECK_ERROR_BITS];
static char *img_error_data = &(error_count[0]);
static char err_status_buf[2];
static char *reg_error_data = &(err_status_buf[0]);

static u8 diag_event_flag = 0x00;

static bool counter_init_flag  = false;

static char disp_on_cmd[1] = {0x29};
static struct dsi_cmd_desc dsi_on_cmds = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(disp_on_cmd)},
	disp_on_cmd
};
static char disp_off_cmd[1] = {0x28};
static struct dsi_cmd_desc dsi_off_cmds = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(disp_off_cmd)},
	disp_off_cmd
};


void disp_ext_diag_count_err_status(u32 ack_err_status)
{
	int check_bit = 0x01;
	int radix = 2;
	int index;

	if (!counter_init_flag) {
		memset(&error_count, 0x00, sizeof(error_count));
		counter_init_flag = true;
	}

	for(index = 0 ; index < CHECK_ERROR_BITS ; index++) {
		if(ack_err_status & check_bit) {
			error_count[index]++;
		}
		check_bit = check_bit * radix;
	}

}

void disp_ext_diag_set_ack_err_stat(u32 ack_err_status)
{
	memset(&err_status_buf, 0x00, sizeof(err_status_buf));

	err_status_buf[0] = ((ack_err_status & ACK_ERROR_BIT_MASK0) >> 8);
	err_status_buf[1] = (ack_err_status & ACK_ERROR_BIT_MASK1);
}

u8 disp_ext_diag_event_flag_check(void)
{
	return diag_event_flag;
}

void disp_ext_diag_set_mipi_err_chk(bool flag)
{
	if (flag) {
		diag_event_flag |= MIPI_ERR_CHECK;
	} else {
		diag_event_flag &= ~MIPI_ERR_CHECK;
	}
}

int disp_ext_diag_reg_write(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	void __user *argp = (void __user *)arg;
	int ret;
	struct dsi_cmd_desc dm_dsi_cmds;
	struct disp_diag_mipi_write_reg_type mipi_reg_data;
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ret = copy_from_user(&mipi_reg_data, argp, sizeof(mipi_reg_data));
	if (ret) {
		pr_err("MSMFB_MIPI_REG_WRITE: error 1[%d] \n", ret);
		return ret;
	}

	if ((mipi_reg_data.type != DTYPE_DCS_WRITE)
	 && (mipi_reg_data.type != DTYPE_DCS_WRITE1)
	 && (mipi_reg_data.type != DTYPE_DCS_LWRITE)
	 && (mipi_reg_data.type != DTYPE_GEN_WRITE)
	 && (mipi_reg_data.type != DTYPE_GEN_WRITE1)
	 && (mipi_reg_data.type != DTYPE_GEN_WRITE2)
	 && (mipi_reg_data.type != DTYPE_GEN_LWRITE)) {
		pr_err("MSMFB_MIPI_REG_WRITE: error 2[%d] \n", mipi_reg_data.type);
		return -EINVAL;
	}

	memset(&dm_dsi_cmds, 0x00, sizeof(dm_dsi_cmds));
	dm_dsi_cmds.dchdr.dtype = mipi_reg_data.type;
	dm_dsi_cmds.dchdr.last = 1;
	dm_dsi_cmds.dchdr.vc = 0;
	dm_dsi_cmds.dchdr.ack = 0;
	dm_dsi_cmds.dchdr.wait = mipi_reg_data.wait;
	dm_dsi_cmds.dchdr.dlen = mipi_reg_data.len;
	dm_dsi_cmds.payload = (char *)mipi_reg_data.data;
	pr_info("@@@ Tx command\n");
	pr_info("    - dtype = 0x%08X\n", dm_dsi_cmds.dchdr.dtype);
	pr_info("    - last  = %d\n", dm_dsi_cmds.dchdr.last);
	pr_info("    - vc    = %d\n", dm_dsi_cmds.dchdr.vc);
	pr_info("    - ack   = %d\n", dm_dsi_cmds.dchdr.ack);
	pr_info("    - wait  = %d\n", dm_dsi_cmds.dchdr.wait);
	pr_info("    - dlen  = %d\n", dm_dsi_cmds.dchdr.dlen);
	pr_info("    - payload:%02X %02X %02X %02X\n",
		dm_dsi_cmds.payload[0], dm_dsi_cmds.payload[1],
		dm_dsi_cmds.payload[2], dm_dsi_cmds.payload[3]);
	pr_info("    -         %02X %02X %02X %02X\n",
		dm_dsi_cmds.payload[4], dm_dsi_cmds.payload[5],
		dm_dsi_cmds.payload[6], dm_dsi_cmds.payload[7]);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dm_dsi_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	if (mipi_reg_data.speed == 1) {
		mdss_dsi_set_tx_power_mode(0, pdata);
	} else {
		mdss_dsi_set_tx_power_mode(1, pdata);
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (mipi_reg_data.bta) {
		diag_event_flag |= REG_ERR_CHECK;
	} else {
		diag_event_flag &= ~REG_ERR_CHECK;
	}

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	if (mipi_reg_data.bta) {
		if (err_status_buf[0] == 0x00 &&
				err_status_buf[1] == 0x00) {
			memset(&mipi_reg_data.ack_err_status, 0x00, 2);
		} else {
			memcpy(mipi_reg_data.ack_err_status, reg_error_data, 2);
		}
	} else {
		memset(&mipi_reg_data.ack_err_status, 0x00, 2);
	}

	ret = copy_to_user(argp, &mipi_reg_data, sizeof(mipi_reg_data));
	if (ret) {
		pr_err("MSMFB_MIPI_REG_WRITE: error [%d] \n", ret);
	}

	diag_event_flag &= ~REG_ERR_CHECK;

	return ret;
}

int disp_ext_diag_reg_read(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	void __user *argp = (void __user *)arg;
	int ret;
	struct dsi_cmd_desc dm_dsi_cmds;
	struct disp_diag_mipi_read_reg_type mipi_reg_data;
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	char rbuf[25];

	ret = copy_from_user(&mipi_reg_data, argp, sizeof(mipi_reg_data));
	if (ret) {
		pr_err("MSMFB_MIPI_REG_READ: error 1[%d] \n", ret);
		return ret;
	}

	if ((mipi_reg_data.type != DTYPE_DCS_READ)
	 && (mipi_reg_data.type != DTYPE_GEN_READ)
	 && (mipi_reg_data.type != DTYPE_GEN_READ1)
	 && (mipi_reg_data.type != DTYPE_GEN_READ2)) {
		pr_err("MSMFB_MIPI_REG_READ: error 2[%d] \n", mipi_reg_data.type);
		return -EINVAL;
	}

	memset(&dm_dsi_cmds, 0x00, sizeof(dm_dsi_cmds));
	dm_dsi_cmds.dchdr.dtype = mipi_reg_data.type;
	dm_dsi_cmds.dchdr.last = 1;
	dm_dsi_cmds.dchdr.vc = 0;
	dm_dsi_cmds.dchdr.ack = 1;
	dm_dsi_cmds.dchdr.wait = mipi_reg_data.wait;
	dm_dsi_cmds.dchdr.dlen = mipi_reg_data.len;
	dm_dsi_cmds.payload = (char *)mipi_reg_data.data;
	pr_info("@@@ Rx command\n");
	pr_info("    - dtype = 0x%08X\n", dm_dsi_cmds.dchdr.dtype);
	pr_info("    - last  = %d\n", dm_dsi_cmds.dchdr.last);
	pr_info("    - vc    = %d\n", dm_dsi_cmds.dchdr.vc);
	pr_info("    - ack   = %d\n", dm_dsi_cmds.dchdr.ack);
	pr_info("    - wait  = %d\n", dm_dsi_cmds.dchdr.wait);
	pr_info("    - dlen  = %d\n", dm_dsi_cmds.dchdr.dlen);
	pr_info("    - payload:%02X %02X %02X %02X\n",
		dm_dsi_cmds.payload[0], dm_dsi_cmds.payload[1],
		dm_dsi_cmds.payload[2], dm_dsi_cmds.payload[3]);
	pr_info("    -         %02X %02X %02X %02X\n",
		dm_dsi_cmds.payload[4], dm_dsi_cmds.payload[5],
		dm_dsi_cmds.payload[6], dm_dsi_cmds.payload[7]);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dm_dsi_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = (int)mipi_reg_data.rlen;
	cmdreq.cb = NULL;
	cmdreq.rbuf = rbuf;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	memcpy(mipi_reg_data.data, ctrl_pdata->rx_buf.data, ctrl_pdata->rx_buf.len);

	ret = copy_to_user(argp, &mipi_reg_data, sizeof(mipi_reg_data));
	if (ret) {
		pr_err("MSMFB_MIPI_REG_READ: error 3[%d] \n", ret);
	}

	return ret;
}

int disp_ext_diag_tx_rate(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	void __user *argp = (void __user *)arg;
	int ret;
	u32 input_data;
	u32 msmfb_rate;
	int new_fps;

	ret = copy_from_user(&input_data, argp, sizeof(uint));
	if (ret) {
		pr_err("MSMFB_CHANGE_TRANSFER_RATE: error 1[%d] \n", ret);
		return ret;
	}
	msmfb_rate = ((input_data << 8) & 0xFF00) | ((input_data >> 8) & 0x00FF) * 1000000;

	new_fps = mfd->panel_info->mipi.frame_rate * msmfb_rate / mfd->panel_info->clk_rate;

	pr_info("%s: new_fps =%d\n", __func__, new_fps);

	ret = mdss_dsi_clk_div_config(mfd->panel_info, new_fps);
	if (ret) {
		pr_err("MSMFB_CHANGE_TRANSFER_RATE: error 2[%d] \n", ret);
		return ret;
	}

	return 0;
}

int disp_ext_diag_err_check_start(void)
{
	diag_event_flag |= IMG_ERR_CHECK;

	return 0;
}

int disp_ext_diag_err_check_stop(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct disp_diag_err_check_type err_check_data;
	void __user *argp = (void __user *)arg;
	int ret;
	diag_event_flag &= ~IMG_ERR_CHECK;

	ret = copy_from_user(&err_check_data, argp, sizeof(err_check_data));
	if (ret) {
		pr_err("MSMFB_ERR_CHK_STOP: error 1[%d] \n", ret);
		return ret;
	}

	memcpy(err_check_data.count_err_status, img_error_data, CHECK_ERROR_BITS);

	ret = copy_to_user(argp, &err_check_data, sizeof(err_check_data));
	if (ret) {
		pr_err("MSMFB_ERR_CHK_STOP: error 2[%d] \n", ret);
	}

	counter_init_flag = false;

	return ret;
}

int disp_ext_diag_current_err_stat(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct disp_diag_err_check_type err_check_data;
	void __user *argp = (void __user *)arg;
	int ret;

	ret = copy_from_user(&err_check_data, argp, sizeof(err_check_data));
	if (ret) {
		pr_err("MSMFB_CURRENT_ERR_STAT: error 1[%d] \n", ret);
		return ret;
	}

	memcpy(err_check_data.count_err_status, img_error_data, CHECK_ERROR_BITS);

	ret = copy_to_user(argp, &err_check_data, sizeof(err_check_data));
	if (ret) {
		pr_err("MSMFB_CURRENT_ERR_STAT: error 2[%d] \n", ret);
	}

	return ret;
}

int disp_ext_diag_img_transfer_sw(struct fb_info *info, unsigned int cmd)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	u32 status;
	int ret = 0;

	memset(&cmdreq, 0, sizeof(cmdreq));
	switch(cmd) {
	case MSMFB_IMG_TRANSFER_ON:
		cmdreq.cmds = &dsi_on_cmds;
		break;
	case MSMFB_IMG_TRANSFER_OFF:
		cmdreq.cmds = &dsi_off_cmds;
		break;
	default:
		pr_err("%s:cmd value is different. cmd = %d\n", __func__, cmd);
		ret = -EINVAL;
		break;
	}
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	status = MIPI_INP((ctrl_pdata->ctrl_base) + 0x3c);
	if ((status & 0x4000000) == 0) {
		mdss_dsi_set_tx_power_mode(1, pdata);
	}

	return ret;
}

void disp_ext_diag_init(void)
{
}
