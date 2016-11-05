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
#define pr_fmt(fmt)	"OEM_CHG_DET %s: " fmt, __func__

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/oem-chg_api.h>
#include <oem-chg_control.h>
#include <oem-chg_det.h>
#include <oem-chg_param.h>
#include <mach/msm_smsm.h>

static void oem_chg_set_usb_det_n_irq(void);
static void oem_chg_set_pma_det_n_irq(void);
static irqreturn_t oem_chg_usb_det_n_isr(int irq, void *ptr);
static irqreturn_t oem_chg_pma_det_n_isr(int irq, void *ptr);

#define GPIO_USB_DET_N   35

#define GPIO_EXT_DET_N   64
#define GPIO_EXT_CHG_EN_N 120
#define GPIO_PMA_DET_N   GPIO_EXT_DET_N
#define GPIO_PMA_EN     GPIO_EXT_CHG_EN_N

#define GPIO_H_CHG_ON    69
#define GPIO_USB_PATH_EN 78

#define IS_USB_DET() (0 == gpio_get_value(GPIO_USB_DET_N))
#define IS_PMA_DET() (0 == gpio_get_value(GPIO_PMA_DET_N))

#define SET_PMA_EN(val) gpio_set_value(GPIO_PMA_EN, val)
#define SET_USB_PATH_EN(val)  gpio_set_value(GPIO_USB_PATH_EN, val) 

#define USB_CHECK_COUNT   2
#define PMA_CHECK_COUNT   2
#define CHECK_CYCLE_MS   50
#define H_CHG_ON_WAIT_MS 5000


#define SMEM_CHG_PARAM_SIZE				160
#define SMEM_CHG_CONNECT_STATE_POINT	151


DEFINE_SPINLOCK(oem_chg_det_master_lock);
#define MASTERDATA_SPINLOCK_INIT() spin_lock_init(&oem_chg_det_master_lock);

#define MASTERDATA_LOCK() \
	{ \
	unsigned long oem_chg_det_irq_flag; \
	spin_lock_irqsave(&oem_chg_det_master_lock, oem_chg_det_irq_flag);

#define MASTERDATA_UNLOCK() \
	spin_unlock_irqrestore(&oem_chg_det_master_lock, oem_chg_det_irq_flag); \
	}

#undef CHG_DET_DEBUG
#ifdef CHG_DET_DEBUG
	#define CHG_DET_LOG pr_err
#else
	#define CHG_DET_LOG pr_debug
#endif

static void *the_chip;

struct delayed_work oem_chg_usb_det_work;
struct delayed_work oem_chg_pma_det_work;
struct delayed_work oem_h_chg_on_dwork;
struct work_struct  oem_h_chg_on_work;
static atomic_t oem_h_chg_on_req = ATOMIC_INIT(0);

struct wake_lock usb_det_wake_lock;
struct wake_lock pma_det_wake_lock;

static int current_usb_det_status = 0;
static int current_pma_det_status = 0;


static int oem_chg_connect_state = POWER_SUPPLY_OEM_CONNECT_NONE;
static atomic_t is_chg_connect_initialized=ATOMIC_INIT(0);

static void oem_chg_master_data_write(enum power_supply_property psp, int val)
{
	int* masterdata;

	switch(psp){
	case POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE:
		masterdata
		 = &oem_chg_connect_state;
		break;

	default:
		masterdata = NULL;
		break;
	}

	if (masterdata != NULL){
		MASTERDATA_LOCK();
		*masterdata = val;
		MASTERDATA_UNLOCK();
	}
}

static int oem_chg_master_data_read(enum power_supply_property psp, int *intval)
{
	int ret = 0;
	int initialized;
	int* masterdata;

	initialized = atomic_read(&is_chg_connect_initialized);

	if (!initialized) {
		pr_err("called before init\n");
		*intval = 0;
		return -EAGAIN;
	}

	switch(psp){
	case POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE:
		masterdata = &oem_chg_connect_state;
		break;

	default:
		masterdata = NULL;
		break;
	}

	if (masterdata != NULL){
		MASTERDATA_LOCK();
		*intval = *masterdata;
		MASTERDATA_UNLOCK();
	}else{
		CHG_DET_LOG("Out of range psp %d \n", psp);
		ret = -EINVAL;
	}
	return ret;
}

static int oem_chg_determine_connect_state(void)
{
	int state = POWER_SUPPLY_OEM_CONNECT_NONE;

	if (current_usb_det_status) {
		state = POWER_SUPPLY_OEM_CONNECT_USB;
	}else if (current_pma_det_status) {
		state = POWER_SUPPLY_OEM_CONNECT_CHARGE_STAND_1;
	}
	CHG_DET_LOG("connect state=%d usb=%d pma=%d\n", state, current_usb_det_status, current_pma_det_status);
	return state;
}


static void oem_chg_set_smem_connect_state(int state)
{
	uint8_t *smem_ptr = NULL;

	smem_ptr = (uint8_t *)kc_smem_alloc(SMEM_CHG_PARAM, SMEM_CHG_PARAM_SIZE);
	if (smem_ptr == NULL) {
		pr_err("smem_ptr is NULL\n");
		return;
	}

	if (1 == oem_param_share.test_mode_chg_disable) {
		state = POWER_SUPPLY_OEM_CONNECT_NONE;
		pr_info("set state to not connected in test mode\n");
	}

	smem_ptr[SMEM_CHG_CONNECT_STATE_POINT] = (uint8_t)state;
}

extern int msm_otg_change_charge_device(int, int);


static void oem_chg_set_connect_state(void)
{
	int state;

	state = oem_chg_determine_connect_state();
	CHG_DET_LOG("charger connect state = %d \n", state);

	if (-1 < state) {
		oem_chg_master_data_write(POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE, state);
		oem_chg_set_smem_connect_state(state);
	}

	if (state == POWER_SUPPLY_OEM_CONNECT_NONE) {
		oem_chg_stop_charging();
	}

	msm_otg_change_charge_device(current_usb_det_status, current_pma_det_status);
}

int oem_chg_pm_batt_power_get_property(enum power_supply_property psp, int *intval)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE:
		ret = oem_chg_master_data_read(psp, intval);
		CHG_DET_LOG("charger connect state = %d\n", *intval);
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(oem_chg_pm_batt_power_get_property);



void oem_chg_set_usb_path_en(int usb_path_en_f)
{
	SET_USB_PATH_EN(usb_path_en_f);
}
EXPORT_SYMBOL(oem_chg_set_usb_path_en);


void oem_chg_set_pma_en(int pma_en_f)
{
	SET_PMA_EN(pma_en_f);
}
EXPORT_SYMBOL(oem_chg_set_pma_en);


static void oem_chg_usb_detection(struct work_struct *work)
{
	static int check_cnt = 0;
	int new_usb_det_status = IS_USB_DET();

	if (current_usb_det_status == new_usb_det_status) {
		check_cnt = 0;
		CHG_DET_LOG("equ usb_status=%d\n", new_usb_det_status);
		goto set_irq;
	}
	check_cnt++;

	if (USB_CHECK_COUNT <= check_cnt) {
		pr_info("change usb_status old=%d new=%d\n",
			current_usb_det_status, new_usb_det_status);
		current_usb_det_status = new_usb_det_status;
		oem_chg_set_connect_state();
		check_cnt = 0;
		if (current_usb_det_status) {
			oem_chg_set_pma_en(current_usb_det_status);
			oem_chg_set_usb_path_en(current_usb_det_status);
		} else {
			oem_chg_set_usb_path_en(current_usb_det_status);
			oem_chg_set_pma_en(current_usb_det_status);
		}
	}

	if (check_cnt) {
		schedule_delayed_work(&oem_chg_usb_det_work,
				msecs_to_jiffies(CHECK_CYCLE_MS));
		CHG_DET_LOG("set oem_chg_usb_det_work=%dms cnt=%d\n", CHECK_CYCLE_MS, check_cnt);
		return;
	}

set_irq:
	oem_chg_set_usb_det_n_irq();
	wake_unlock(&usb_det_wake_lock);
	CHG_DET_LOG("unlock usb_det_wake_lock\n");
	return;
}

static void oem_chg_pma_detection(struct work_struct *work)
{
	static int check_cnt = 0;
	int new_pma_det_status = IS_PMA_DET();

	if (current_pma_det_status == new_pma_det_status) {
		check_cnt = 0;
		CHG_DET_LOG("equ pma_status=%d\n", new_pma_det_status);
		goto set_irq;
	}
	check_cnt++;

	if (PMA_CHECK_COUNT <= check_cnt) {
		pr_info("change pma_status old=%d new=%d\n",
			current_pma_det_status, new_pma_det_status);
		current_pma_det_status = new_pma_det_status;
		oem_chg_set_connect_state();
		check_cnt = 0;
	}

	if (check_cnt) {
		schedule_delayed_work(&oem_chg_pma_det_work,
				msecs_to_jiffies(CHECK_CYCLE_MS));
		CHG_DET_LOG("set oem_chg_pma_det_work cnt=%d\n", check_cnt);
		return;
	}

set_irq:
	oem_chg_set_pma_det_n_irq();
	wake_unlock(&pma_det_wake_lock);
	CHG_DET_LOG("unlock pma_det_wake_lock\n");
	return;
}

static void oem_chg_set_usb_det_n_irq(void)
{
	int rc = 0;

	free_irq(gpio_to_irq(GPIO_USB_DET_N), 0);
	if (current_usb_det_status) {
		rc = request_irq(gpio_to_irq(GPIO_USB_DET_N), oem_chg_usb_det_n_isr, IRQF_TRIGGER_HIGH, "USB_DET_N", 0);
	} else {
		rc = request_irq(gpio_to_irq(GPIO_USB_DET_N), oem_chg_usb_det_n_isr, IRQF_TRIGGER_LOW, "USB_DET_N", 0);
	}
	if (rc) {
		pr_err("couldn't register interrupts rc=%d\n", rc);
	}
}

static void oem_chg_set_pma_det_n_irq(void)
{
	int rc = 0;

	free_irq(gpio_to_irq(GPIO_PMA_DET_N), 0);
	if (current_pma_det_status) {
		rc = request_irq(gpio_to_irq(GPIO_PMA_DET_N), oem_chg_pma_det_n_isr, IRQF_TRIGGER_HIGH, "PMA_DET_N", 0);
	} else {
		rc = request_irq(gpio_to_irq(GPIO_PMA_DET_N), oem_chg_pma_det_n_isr, IRQF_TRIGGER_LOW, "PMA_DET_N", 0);
	}
	if (rc) {
		pr_err("couldn't register interrupts rc=%d\n", rc);
	}
}

static irqreturn_t oem_chg_usb_det_n_isr(int irq, void *ptr)
{
	disable_irq_nosync(gpio_to_irq(GPIO_USB_DET_N));

	CHG_DET_LOG("USB_DET_N irq detected!! GPIO=%d\n", gpio_get_value(GPIO_USB_DET_N));

	wake_lock(&usb_det_wake_lock);
	schedule_delayed_work(&oem_chg_usb_det_work,
			msecs_to_jiffies(CHECK_CYCLE_MS));

	return IRQ_HANDLED;
}

static irqreturn_t oem_chg_pma_det_n_isr(int irq, void *ptr)
{
	disable_irq_nosync(gpio_to_irq(GPIO_PMA_DET_N));

	CHG_DET_LOG("PMA_DET_N irq detected!! GPIO=%d\n", gpio_get_value(GPIO_PMA_DET_N));

	wake_lock(&pma_det_wake_lock);
	schedule_delayed_work(&oem_chg_pma_det_work,
			msecs_to_jiffies(CHECK_CYCLE_MS));

	return IRQ_HANDLED;
}

int oem_chg_get_usb_det_status(void)
{
	return current_usb_det_status;
}
EXPORT_SYMBOL(oem_chg_get_usb_det_status);

int oem_chg_get_pma_det_status(void)
{
	return current_pma_det_status;
}
EXPORT_SYMBOL(oem_chg_get_pma_det_status);

void oem_chg_set_h_chg_on(enum power_supply_type psp)
{
	int value = 0;


	if (psp == POWER_SUPPLY_TYPE_USB_DCP) {

		value = 1;
	}
	pr_info("H_CHG_ON=%d POWER_SUPPLY_PROP_TYPE=%d\n", value, psp);
}
EXPORT_SYMBOL(oem_chg_set_h_chg_on);



void oem_chg_h_chg_on_ctrl(int value)
{
	if(value == 0 || value == 1) {
		atomic_set(&oem_h_chg_on_req, value);
		schedule_work(&oem_h_chg_on_work);
		pr_info("set oem_h_chg_on_work value:%d\n", value);
	}
}
EXPORT_SYMBOL(oem_chg_h_chg_on_ctrl);


static void oem_h_chg_on_ctrl_work(struct work_struct *work)
{
	if (atomic_read(&oem_h_chg_on_req)) {
		schedule_delayed_work(&oem_h_chg_on_dwork, msecs_to_jiffies(H_CHG_ON_WAIT_MS));
		pr_info("set oem_h_chg_on_dwork:%dms\n", H_CHG_ON_WAIT_MS);
	} else {
		cancel_delayed_work_sync(&oem_h_chg_on_dwork);
		gpio_set_value(GPIO_H_CHG_ON, 0);
		pr_info("set GPIO_H_CHG_ON:0\n");
	}
}

static void oem_h_chg_on_set_dwork(struct work_struct *work)
{
	gpio_set_value(GPIO_H_CHG_ON, 1);
	pr_info("set GPIO_H_CHG_ON:1\n");
}

void oem_chg_det_init(void *chip)
{
	int gpio_trigger_pma_det_n;
	int gpio_trigger_usb_det_n;
	int rc;

	the_chip = chip;

	wake_lock_init(&usb_det_wake_lock, WAKE_LOCK_SUSPEND, "usb_detect");
	wake_lock_init(&pma_det_wake_lock, WAKE_LOCK_SUSPEND, "pma_detect");

	INIT_DELAYED_WORK(&oem_chg_usb_det_work, oem_chg_usb_detection);
	INIT_DELAYED_WORK(&oem_chg_pma_det_work, oem_chg_pma_detection);
	INIT_DELAYED_WORK(&oem_h_chg_on_dwork, oem_h_chg_on_set_dwork);
	INIT_WORK(&oem_h_chg_on_work, oem_h_chg_on_ctrl_work);

	gpio_tlmm_config(GPIO_CFG(GPIO_USB_DET_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_PMA_DET_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	gpio_request( GPIO_PMA_EN, "GPIO_PMA_EN PORT");
	gpio_tlmm_config(GPIO_CFG(GPIO_PMA_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	
	gpio_request( GPIO_USB_PATH_EN, "GPIO_USB_PATH_EN PORT" );
	gpio_tlmm_config(GPIO_CFG(GPIO_USB_PATH_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE); 
	

	if (IS_USB_DET()) {
		gpio_trigger_usb_det_n = IRQF_TRIGGER_HIGH;
		current_usb_det_status = 1;
	} else {
		gpio_trigger_usb_det_n = IRQF_TRIGGER_LOW;
		current_usb_det_status = 0;
	}

	if (IS_PMA_DET()) {
		gpio_trigger_pma_det_n = IRQF_TRIGGER_HIGH;
		current_pma_det_status = 1;
	} else {
		gpio_trigger_pma_det_n = IRQF_TRIGGER_LOW;
		current_pma_det_status = 0;
	}

	pr_info("current usb_status=%d pma_status=%d\n",current_usb_det_status, current_pma_det_status);
	if (current_usb_det_status) {
		oem_chg_set_pma_en(current_usb_det_status);
		oem_chg_set_usb_path_en(current_usb_det_status);
	} else {
		oem_chg_set_usb_path_en(current_usb_det_status);
		oem_chg_set_pma_en(current_usb_det_status);
	}


	oem_chg_set_connect_state();
	MASTERDATA_SPINLOCK_INIT();
	atomic_set(&is_chg_connect_initialized, 1);

	rc = request_irq(gpio_to_irq(GPIO_USB_DET_N), oem_chg_usb_det_n_isr, gpio_trigger_usb_det_n, "USB_DET_N", 0);
	if (rc) {
		pr_err("usb couldn't register interrupts rc=%d\n", rc);
		return;
	}

	rc = request_irq(gpio_to_irq(GPIO_PMA_DET_N), oem_chg_pma_det_n_isr, gpio_trigger_pma_det_n, "PMA_DET_N", 0);
	if (rc) {
		pr_err("pma couldn't register interrupts rc=%d\n", rc);
		return;
	}
	enable_irq_wake(gpio_to_irq(GPIO_USB_DET_N));
	enable_irq_wake(gpio_to_irq(GPIO_PMA_DET_N));

	msm_otg_change_charge_device(current_usb_det_status, current_pma_det_status);
	pr_info("initialization is completed!!\n");
}
EXPORT_SYMBOL(oem_chg_det_init);
