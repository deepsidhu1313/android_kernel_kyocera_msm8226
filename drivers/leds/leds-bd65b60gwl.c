/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#undef ENABLE_CABC_PWM

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <mach/vreg.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/leds.h>
#include <linux/time.h>
#include "oem_wled.h" 


#include <linux/workqueue.h>
static unsigned long I2C_DELAY_JIFFIES = 1;


#include <mach/kc_board.h>

#define FORCE_WLED_ON        0
#define DISABLE_DISP_DETECT        1

#define BD65B60GWL_DEBUG        0
#if BD65B60GWL_DEBUG
#define BD65B60GWL_DEBUG_LOG( msg... )	printk(KERN_ERR "[WLED]:" msg)
#else
#define BD65B60GWL_DEBUG_LOG( msg... )
#endif

#define BD65B60GWL_ERR_LOG( msg... ) 	printk(KERN_ERR "[WLED]:" msg)
#define BD65B60GWL_INFO_LOG( msg... ) 	printk(KERN_INFO "[WLED]:" msg)


#define BD65B60GWL_DRV_NAME "BD65B60GWL"

#define I2C_RETRIES_NUM					(5)
#define BD65B60GWL_RESET_GPIO			(55)
#define BD65B60GWL_I2C_WRITE_MSG_NUM	(1)
#define BD65B60GWL_I2C_READ_MSG_NUM		(2)
#define BD65B60GWL_WRITE_BUF_LEN		(2)

#define BD65B60GWL_LED_ENABLE			0x01
#define BD65B60GWL_LED_DISABLE			0x00
#define BD65B60GWL_LED_BRIGHT_MASK		0xff

#define BACKLIGHT_ON					0x01
#define BACKLIGHT_OFF					0x00
#define BACKLIGHT_THRESHOLD_TEMPVAL		0x3C
#define BACKLIGHT_THRESHOLD_CURVAL		0xFF

#define GPIO_HIGH_VAL					1
#define GPIO_LOW_VAL					0

struct bd65b60gwl_data {
	struct led_classdev	st_cdev;
	struct i2c_client	*client;
	uint32_t			ul_value;

	struct delayed_work		dwork;
	struct workqueue_struct* wq;

	struct mutex		lock;
	struct wake_lock	work_wake_lock;
	int					wled_reset_gpio;
};

#define BACKLIGHT_INFO			"backlightinfo"

/* Local Variables */
static struct i2c_client *client_bd65b60gwl = NULL;
static int32_t backlight_status = BACKLIGHT_OFF;
static unsigned int *g_wled_brightness = NULL;

/* Local Function */
static int bd65b60gwl_i2c_read(struct i2c_client *client, uint8_t uc_reg, uint8_t *rbuf, int len);
static int bd65b60gwl_i2c_write(struct i2c_client *client, uint8_t uc_reg, uint8_t uc_val);
static void bd65b60gwl_work_handler(struct work_struct *work);
#ifdef CONFIG_PM
static int bd65b60gwl_suspend(struct i2c_client *client, pm_message_t mesg);
static int bd65b60gwl_resume(struct i2c_client *client);
#endif
static int bd65b60gwl_remove(struct i2c_client *client);
static int bd65b60gwl_init_client(struct i2c_client *client);
static int bd65b60gwl_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int32_t __init bd65b60gwl_wled_init(void);
static void __exit bd65b60gwl_wled_exit(void);

static int bd65b60gw_activate_gpios(struct bd65b60gwl_data *data)
{
	// GPIO[55]	HARDWARE Reset H
	BD65B60GWL_DEBUG_LOG("%s() set gpio(%d) = %d\n", __func__, data->wled_reset_gpio, GPIO_HIGH_VAL);
	gpio_set_value_cansleep(data->wled_reset_gpio, GPIO_HIGH_VAL);
	usleep(200);

	return 0;
}

static int bd65b60gw_suspend_gpios(struct bd65b60gwl_data *data)
{
	// GPIO[55]	HARDWARE Reset L
	BD65B60GWL_DEBUG_LOG("%s() set gpio(%d) = %d\n", __func__, data->wled_reset_gpio, GPIO_LOW_VAL);
	gpio_set_value_cansleep(data->wled_reset_gpio, GPIO_LOW_VAL);
	usleep(200);

	return 0;
}

static int bd65b60gwl_i2c_read(struct i2c_client *client, uint8_t uc_reg, uint8_t *rbuf, int len)
{
	int ret = 0;
	int retry = 0;
	struct i2c_msg i2cMsg[BD65B60GWL_I2C_READ_MSG_NUM];
	u8 reg = 0;

	BD65B60GWL_DEBUG_LOG("%s() [IN] client=0x%p reg=0x%02X len=%d\n", __func__, client, uc_reg, len);

	if (client == NULL)
	{
		BD65B60GWL_ERR_LOG("%s() fail client=0x%p\n",__func__, client);
		return -ENODEV;
	}

	reg = uc_reg;

	i2cMsg[0].addr = client->addr;
	i2cMsg[0].flags = 0;
	i2cMsg[0].len = 1;
	i2cMsg[0].buf = &reg;

	i2cMsg[1].addr = client->addr;
	i2cMsg[1].flags = I2C_M_RD;
	i2cMsg[1].len = len;
	i2cMsg[1].buf = rbuf;

	do {
		ret = i2c_transfer(client->adapter, &i2cMsg[0], BD65B60GWL_I2C_READ_MSG_NUM);
		BD65B60GWL_DEBUG_LOG("%s(): i2c_transfer() call end ret=%d\n", __func__, ret);
	} while ((ret != BD65B60GWL_I2C_READ_MSG_NUM) && (++retry < I2C_RETRIES_NUM));

	if (ret != BD65B60GWL_I2C_READ_MSG_NUM) {
		BD65B60GWL_ERR_LOG("%s(): fail (try:%d) uc_reg=0x%02x, rbuf=0x%02x ret=%d\n", __func__, retry, uc_reg, *rbuf, ret);
		ret = -1;
	}else{
		ret = 0;
	}
	BD65B60GWL_DEBUG_LOG("%s(): [OUT] rbuf=0x%02x\n", __func__, *rbuf);

	return ret;
}

static int bd65b60gwl_i2c_write(struct i2c_client *client, uint8_t uc_reg, uint8_t uc_val)
{
	int ret = 0;
	int retry = 0;
	struct i2c_msg i2cMsg;
	u8 ucwritebuf[BD65B60GWL_WRITE_BUF_LEN];

	BD65B60GWL_DEBUG_LOG("%s() [IN] client=0x%p reg=0x%02x val=0x%02X\n", __func__, client, uc_reg, uc_val);

	if (client == NULL)
	{
		BD65B60GWL_ERR_LOG("%s() fail client=0x%p\n",__func__, client);
		return -ENODEV;
	}

	ucwritebuf[0] = uc_reg;
	ucwritebuf[1] = uc_val;
	i2cMsg.addr  = client->addr;
	i2cMsg.flags = 0;
	i2cMsg.len   =  sizeof(ucwritebuf);
	i2cMsg.buf   =  &ucwritebuf[0];

	do {
		ret = i2c_transfer(client->adapter, &i2cMsg, BD65B60GWL_I2C_WRITE_MSG_NUM);
		BD65B60GWL_DEBUG_LOG("%s() i2c_transfer() call end ret=%d\n", __func__, ret);
	} while ((ret != BD65B60GWL_I2C_WRITE_MSG_NUM) && (++retry < I2C_RETRIES_NUM));

	if (ret != BD65B60GWL_I2C_WRITE_MSG_NUM) {
		BD65B60GWL_ERR_LOG("%s(): fail (try:%d) uc_reg=0x%02x, uc_val=0x%02x ret=%d\n", __func__, retry, ucwritebuf[0], ucwritebuf[1], ret);
		ret = -1;
	}else{
		ret = 0;
	}
	BD65B60GWL_DEBUG_LOG("%s(): [OUT]\n", __func__);

	return ret;
}

static void wled_led_set(struct led_classdev *pst_cdev, enum led_brightness value)
{
	struct bd65b60gwl_data *data;

	data = container_of(pst_cdev, struct bd65b60gwl_data, st_cdev);


	if (data->wq) {
		data->ul_value = value;
		cancel_delayed_work_sync(&data->dwork);
		queue_delayed_work(data->wq, &data->dwork, I2C_DELAY_JIFFIES);
	}


	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);
}

static enum led_brightness wled_led_get(struct led_classdev *pst_cdev)
{
	int32_t lret = 0;
	struct bd65b60gwl_data *data;
	BD65B60GWL_DEBUG_LOG("%s() [IN]\n", __func__);

	data = container_of(pst_cdev, struct bd65b60gwl_data, st_cdev);
	lret = data->ul_value;
	BD65B60GWL_DEBUG_LOG("%s() [OUT] ret=0x%02x\n", __func__, lret);

	return lret;
}

static void bd65b60gwl_work_handler(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bd65b60gwl_data *data = container_of(dwork, struct bd65b60gwl_data, dwork);
	int level = 0;
	struct i2c_client *client = data->client;
	uint8_t reg = 0;
	int err = 0;
	int val;
	int gval;

	level = data->st_cdev.brightness;

	BD65B60GWL_DEBUG_LOG("%s() [IN] level=0x%x\n", __func__, level);
	gval = gpio_get_value_cansleep(data->wled_reset_gpio);
	BD65B60GWL_DEBUG_LOG("%s() gpio(%d) = %d \n", __func__, data->wled_reset_gpio, gval);

	mutex_lock(&data->lock);

	if( BACKLIGHT_OFF == backlight_status ){
		bd65b60gw_activate_gpios( data );

		err = bd65b60gwl_init_client(client);
		if (err)
		{
			BD65B60GWL_ERR_LOG("%s() Failed bd65b60gwl_init_client\n", __func__);
			return;
		}
	}


	if( g_wled_brightness ) {
		val = g_wled_brightness[level & BD65B60GWL_LED_BRIGHT_MASK];
	} else {
		val = level & BD65B60GWL_LED_BRIGHT_MASK;
	}

	BD65B60GWL_DEBUG_LOG("%s() level=0x%x val=0x%x\n", __func__, level, val);

    /* program brightness control registers */
	bd65b60gwl_i2c_write(client, 0x05, val);

	if (level == 0) {
	    BD65B60GWL_DEBUG_LOG("%s(): OFF\n", __func__);

		{
			bd65b60gwl_i2c_read(client, 0x05, &reg, 1);
			bd65b60gwl_i2c_write(client, 0x0E, BD65B60GWL_LED_DISABLE);		// WLED OFF
		}

		bd65b60gw_suspend_gpios( data );

		backlight_status = BACKLIGHT_OFF;
	} else {
	    BD65B60GWL_DEBUG_LOG("%s(): ON (0x%0x)\n", __func__, level);

		{
			bd65b60gwl_i2c_write(client, 0x0E, BD65B60GWL_LED_ENABLE);		// WLED ON
		}

		backlight_status = BACKLIGHT_ON;
	}

	mutex_unlock(&data->lock);

	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);


	return;
}

#ifdef CONFIG_PM
static int bd65b60gwl_suspend(struct i2c_client *client, pm_message_t mesg)
{
	BD65B60GWL_DEBUG_LOG("%s() [IN]\n", __func__);
	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);
	return 0;
}

static int bd65b60gwl_resume(struct i2c_client *client)
{
	BD65B60GWL_DEBUG_LOG("%s() [IN]\n", __func__);
	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);
	return 0;
}
#else
#define bd65b60gwl_suspend    NULL
#define bd65b60gwl_resume     NULL
#endif /* CONFIG_PM */

static int bd65b60gwl_remove(struct i2c_client *client)
{
	struct bd65b60gwl_data *data = i2c_get_clientdata(client);

	BD65B60GWL_DEBUG_LOG("%s() [IN]\n", __func__);

	kfree(data);

	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);

	return 0;
}

static int bd65b60gwl_init_client(struct i2c_client *client)
{
	struct bd65b60gwl_data *data = i2c_get_clientdata(client);
	int gval;

	BD65B60GWL_DEBUG_LOG("%s() [IN] client=0x%p\n", __func__, client);
	gval = gpio_get_value_cansleep(data->wled_reset_gpio);
	BD65B60GWL_DEBUG_LOG("%s() gpio(%d) = %d \n", __func__, data->wled_reset_gpio, gval);
	if( !gval ){
		bd65b60gw_activate_gpios( data );
	}
	bd65b60gwl_i2c_write(client, 0x01, 0x01);
	bd65b60gwl_i2c_write(client, 0x02, 0x02);
	bd65b60gwl_i2c_write(client, 0x03, 0x05);
	bd65b60gwl_i2c_write(client, 0x05, 0x60);
#ifdef ENABLE_CABC_PWM
	bd65b60gwl_i2c_write(client, 0x07, 0x26);
#else
	bd65b60gwl_i2c_write(client, 0x07, 0x06);
#endif
	bd65b60gwl_i2c_write(client, 0x08, 0x00);

#ifdef FORCE_WLED_ON
	// force WLED ON
	BD65B60GWL_DEBUG_LOG("%s() force WLED ON\n", __func__);
	bd65b60gwl_i2c_write(client, 0x0E, 0x01);		// WLED ON
#endif
	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);

    return 0;
}

static int bd65b60gwl_probe(struct i2c_client *client,
                   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct bd65b60gwl_data *data;
	int err = 0;
	int gval;

	BD65B60GWL_DEBUG_LOG("%s() [IN] client=0x%p\n", __func__, client);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct bd65b60gwl_data), GFP_KERNEL);
	if (!data) {
		printk("%s: Failed kzalloc\n",__func__);
		err = -ENOMEM;
		goto exit;
	}

	data->st_cdev.brightness_set    = wled_led_set;
	data->st_cdev.brightness_get    = wled_led_get;

	mutex_init(&data->lock);

	data->client = client;
	i2c_set_clientdata(client, data);


	data->wq = create_workqueue("bd65b60gwl");
	if (!data->wq) {
		printk("%s: !!create_workqueue(bd65b60gwl) failed.\n",__func__);
	}
	INIT_DELAYED_WORK(&data->dwork, (work_func_t)bd65b60gwl_work_handler);


	data->st_cdev.name = BACKLIGHT_INFO;

	BD65B60GWL_DEBUG_LOG("%s(): led_classdev_register cdev.name=%s\n", __func__, data->st_cdev.name);
	err = led_classdev_register(&data->client->dev, &data->st_cdev);
	if (err) {
		BD65B60GWL_ERR_LOG("%s() unable to register led %s\n", __func__, data->st_cdev.name);
		goto fail_id_check;
	}

	data->wled_reset_gpio = of_get_named_gpio(client->dev.of_node, "kc,reset-gpio", 0);
	if (!gpio_is_valid(data->wled_reset_gpio)) {
		BD65B60GWL_ERR_LOG("%s() No valid RESET GPIO specified %d\n", __func__, data->wled_reset_gpio);
		goto fail_id_check;
	}

	err = gpio_request(data->wled_reset_gpio, BD65B60GWL_DRV_NAME);
	BD65B60GWL_DEBUG_LOG("%s() gpio_request GPIO=%d err=%d\n", __func__, data->wled_reset_gpio, err);
	if (err < 0) {
		BD65B60GWL_ERR_LOG("%s() failed to request GPIO=%d, ret=%d\n",
				__func__,
				data->wled_reset_gpio,
				err);
		goto fail_id_check;
	}
	gval = gpio_get_value_cansleep(data->wled_reset_gpio);
	BD65B60GWL_DEBUG_LOG("%s() gpio(%d) = %d \n", __func__, data->wled_reset_gpio, gval);


	if (OEM_get_board() == OEM_BOARD_PP_TYPE)
		g_wled_brightness = wled_brightness_panel_ppp;
	else

		/* set default table */
		g_wled_brightness = wled_brightness_panel0;


#ifdef DISABLE_DISP_DETECT
	err = bd65b60gwl_init_client(client);
	if (err)
	{
		BD65B60GWL_ERR_LOG("%s() Failed bd65b60gwl_init_client\n", __func__);
		goto fail_id_check;
	}
#endif
	client_bd65b60gwl = client;

	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);

	return 0;

fail_id_check:
	mutex_destroy(&data->lock);
	led_classdev_unregister(&data->st_cdev);
	kfree(data);
exit:
	BD65B60GWL_DEBUG_LOG("%s() [OUT] err=%d\n", __func__, err);
	return err;
}


static const struct i2c_device_id bd65b60gwl_id[] = {
	{ BD65B60GWL_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bd65b60gwl_id);

static struct of_device_id bd65b60gwl_match_table[] = {
	{ .compatible = BD65B60GWL_DRV_NAME,},
	{ },
};

static struct i2c_driver bd65b60gwl_driver = {
	.driver = {
		.name   = BD65B60GWL_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = bd65b60gwl_match_table,
	},
	.suspend = bd65b60gwl_suspend,
	.resume = bd65b60gwl_resume,
	.probe  = bd65b60gwl_probe,
	.remove = bd65b60gwl_remove,
	.id_table = bd65b60gwl_id,
};

static int32_t __init bd65b60gwl_wled_init(void)
{
	int32_t rc = 0;

	 BD65B60GWL_DEBUG_LOG("%s() [IN]\n", __func__);

	rc = i2c_add_driver(&bd65b60gwl_driver);
	BD65B60GWL_DEBUG_LOG("%s() i2c_add_driver() ret=%d\n", __func__, rc);
	if (rc != 0) {
		BD65B60GWL_ERR_LOG("%s() can't add i2c driver\n", __func__);
		rc = -ENOTSUPP;
		return rc;
	}

	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);

	return rc;

}

static void __exit bd65b60gwl_wled_exit(void)
{
	BD65B60GWL_DEBUG_LOG("%s() [IN]\n", __func__);

	i2c_del_driver(&bd65b60gwl_driver);
	i2c_unregister_device(client_bd65b60gwl);
	client_bd65b60gwl = NULL;

	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);
}

module_init(bd65b60gwl_wled_init);
module_exit(bd65b60gwl_wled_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("WLED");
MODULE_LICENSE("GPL");
