/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */
/******************************************************************************
 * MODULE       : rohm_bm1383_i2c.c
 * FUNCTION     : Driver source for BM1383 in Pressure Sensor IC.
 * AUTHOR       : Atsushi Momota
 * PROGRAMMED   : Sensing Solution Group, ROHM CO.,LTD.
 * MODIFICATION : Modified by ROHM, JAN/05/2015
 * REMARKS      :
 * COPYRIGHT    : Copyright (C) 2015 ROHM CO.,LTD.
 *              : This program is free software; you can redistribute it and/or
 *              : modify it under the terms of the GNU General Public License
 *              : as published by the Free Software Foundation; either version 2
 *              : of the License, or (at your option) any later version.
 *              :
 *              : This program is distributed in the hope that it will be useful,
 *              : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *              : GNU General Public License for more details.
 *              :
 *              : You should have received a copy of the GNU General Public License
 *              : along with this program; if not, write to the Free Software
 *              : Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *****************************************************************************/
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
#include <linux/version.h>

#include "rohm_bm1383_i2c.h"

/******************************* define *******************************/
/* structure of peculiarity to use by system */
typedef struct {
    struct i2c_client  *client;      /* structure pointer for i2c bus            */
    struct hrtimer     timer;        /* structure for timer handler              */
    struct work_struct work;         /* structure for work queue                 */
    struct input_dev   *input_dev;   /* structure pointer for input device       */
    unsigned long      delay_time;   /* delay time to set from application       */
    unsigned char      measure_state;
} PRESS_DATA;

/* logical functions */
static int __devinit        press_init(void);
static void __exit          press_exit(void);
static int                  press_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int                  press_remove(struct i2c_client *client);
static int                  press_input_open(struct input_dev *input);
static void                 press_input_close(struct input_dev *input);
static void                 press_work_func(struct work_struct *work);
static enum hrtimer_restart press_timer_func(struct hrtimer *timer);

/* file system */
static ssize_t bm1383_store_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t bm1383_show_enable(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t bm1383_store_delaytime(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t bm1383_show_delaytime(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t bm1383_show_data(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t bm1383_show_version(struct device *dev, struct device_attribute *attr, char *buf);
/* access function */
static int press_access_init(struct i2c_client *client);
static int press_access_write_measurement_state(struct i2c_client *client, unsigned char val);
static int press_access_read_measurement_data(struct i2c_client *client, unsigned int *val);

/* driver function */
static void make_timer_val(unsigned long data, unsigned long *sec, unsigned long *nsec);

/**************************** variable declaration ****************************/
static const char              bm1383_driver_ver[] = BM1383_DRIVER_VER;
static struct workqueue_struct *rohm_workqueue;

/**************************** structure declaration ****************************/
/* I2C device IDs supported by this driver */
static const struct i2c_device_id press_id[] = {
    { BM1383_I2C_NAME, 0 }, /* rohm bm1383 driver */
    { }
};

static struct of_device_id match_table[] = {
	{ .compatible = "bm1383",},
	{ },
};

/* represent an I2C device driver */
static struct i2c_driver bm1383_driver = {
    .driver = {                      /* device driver model driver */
        .name  = BM1383_I2C_NAME,
        .owner = THIS_MODULE,
        .of_match_table = match_table,
    },
    .probe    = press_probe,         /* callback for device binding */
    .remove   = press_remove,        /* callback for device unbinding */
    .shutdown = NULL,
    .suspend  = NULL,
    .resume   = NULL,
    .id_table = press_id,            /* list of I2C devices supported by this driver */
};

static DEVICE_ATTR(enable,S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP , bm1383_show_enable, bm1383_store_enable);
static DEVICE_ATTR(delay, S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP , bm1383_show_delaytime, bm1383_store_delaytime);
static DEVICE_ATTR(data,  S_IRUGO, bm1383_show_data, NULL);
static DEVICE_ATTR(version,  S_IRUGO, bm1383_show_version, NULL);

static struct attribute *bm1383_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_data.attr,
    &dev_attr_delay.attr,
    &dev_attr_version.attr,
    NULL  /* need to NULL terminate the list of attributes */
};

static const struct attribute_group bm1383_attr_group = {
    .attrs = bm1383_attributes,
};

/************************************************************
 *                      logic function                      *
 ***********************************************************/
/******************************************************************************
 * NAME       : press_init
 * FUNCTION   : register driver to kernel
 * REMARKS    :
 *****************************************************************************/
static int __devinit press_init(void)
{
    int result;
    printk(KERN_INFO "[PRESS] %s: init\n",__func__);

    rohm_workqueue = create_singlethread_workqueue("rohm_workqueue");
    if (!rohm_workqueue) {
        result = -ENOMEM;
        return (result);
    }
    result = i2c_add_driver(&bm1383_driver);

    return (result);
}

/******************************************************************************
 * NAME       : press_exit
 * FUNCTION   : remove driver from kernel
 * REMARKS    :
 *****************************************************************************/
static void __exit press_exit(void)
{
    printk(KERN_INFO "[PRESS] %s: exit\n",__func__);
    i2c_del_driver(&bm1383_driver);
    if (rohm_workqueue) {
        destroy_workqueue(rohm_workqueue);
    }

    return;
}

/******************************************************************************
 * NAME       : press_probe
 * FUNCTION   : initialize system
 * REMARKS    :
 *****************************************************************************/
static int press_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    PRESS_DATA *press;
    int        result;

    printk(KERN_INFO "[PRESS] called %s of BM1383!!\n", __func__);
    result = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
    if (!result) {
        printk(KERN_ERR "[PRESS] %s: need I2C_FUNC_I2C\n", __func__);
        result = -ENODEV;
        goto err_first;
    }
    press = kzalloc(sizeof(PRESS_DATA), GFP_KERNEL);
    if (press == NULL) {
        printk(KERN_ERR "[PRESS] %s: no memory\n", __func__);
        result = -ENOMEM;
        goto err_first;
    }
    INIT_WORK(&press->work, press_work_func);
    press->client = client;
    i2c_set_clientdata(client, press);

    /* set input device */
    press->input_dev = input_allocate_device();
    if (!press->input_dev) {
        result = -ENOMEM;
        printk(KERN_ERR "[PRESS] %s: input_device allocate failed\n", __func__);
        dev_err(&press->client->dev, "input device allocate failed\n");
        goto err_second;
    }
    /* set input event */
    input_set_drvdata(press->input_dev, press);
    set_bit(EV_MSC, press->input_dev->evbit);
    set_bit(EV_SYN, press->input_dev->evbit);
    set_bit(PRESS, press->input_dev->mscbit);

    /* set event name */
    press->input_dev->name  = BM1383_INPUT_NAME;
    press->input_dev->open  = press_input_open;
    press->input_dev->close = press_input_close;

    /* register the device */
    result = input_register_device(press->input_dev);
    if (result) {
        printk(KERN_ERR "[PRESS] unable to register input polled device %s: %d\n", press->input_dev->name, result);
        goto err_third;
    }

    /* Register sysfs hooks */
    result = sysfs_create_group(&press->input_dev->dev.kobj, &bm1383_attr_group);
    if (result) {
        printk(KERN_ERR "[PRESS] %s sysfs_create_groupX\n", __func__);
        goto err_fourth;
    }

    /* timer process */
    hrtimer_init(&press->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    press->timer.function = press_timer_func;

    /* initialize static variable */
    press->delay_time    = BM1383_DEFAULT_DELAY_TIME;
    press->measure_state = MEASURE_OFF;
    printk(KERN_INFO "[PRESS] %s probe exit\n", __func__);

    result = press_access_init(press->client);
    if (result < 0) {
        printk(KERN_ERR "[PRESS] %s: Can't initialize IC. result = 0x%x\n", __func__, result);
        goto err_fourth;
    }
    return (0);

err_fourth:
    input_unregister_device(press->input_dev);
err_third:
    input_free_device(press->input_dev);
err_second:
    kfree(press);
err_first:

    return (result);

}

/******************************************************************************
 * NAME       : press_remove
 * FUNCTION   : close system
 * REMARKS    :
 *****************************************************************************/
static int press_remove(struct i2c_client *client)
{
    PRESS_DATA *press;
    int        result;

    press  = i2c_get_clientdata(client);
    result = hrtimer_cancel(&press->timer);
    if (result == 0) {
        cancel_work_sync(&press->work);
        sysfs_remove_group(&client->dev.kobj, &bm1383_attr_group);
        input_unregister_device(press->input_dev);
        input_free_device(press->input_dev);
        kfree(press);
    } else {
        result = -result;
        printk(KERN_ERR "[PRESS] %s: Can't cancel timer.\n", __func__);
    }


    return (result);
}

/******************************************************************************
 * NAME       : press_work_func
 * FUNCTION   : periodically reads the data from sensor(thread of work)
 * REMARKS    :
 *****************************************************************************/
static void press_work_func(struct work_struct *work)
{
    int           result;
    PRESS_DATA    *press;
    long          wait_sec;
    unsigned long wait_nsec;
    unsigned int  data;


    press = container_of(work, PRESS_DATA, work);

    if (press->measure_state == MEASURE_OFF) {
        return;
    }

    result = press_access_read_measurement_data(press->client, &data);
    if (result < 0) {
        printk(KERN_ERR "[PRESS] %s: ERROR! read data function.\n", __func__);
        return;
    }

    //printk(KERN_INFO "[PRESS] %s: data(%d) \n",__func__,data);

    input_event(press->input_dev, EV_MSC, PRESS, DUMMY_VALUE);
    input_event(press->input_dev, EV_MSC, PRESS, data);
    input_sync(press->input_dev);

    make_timer_val(press->delay_time, &wait_sec, &wait_nsec);
    result = hrtimer_start(&press->timer, ktime_set(wait_sec, wait_nsec), HRTIMER_MODE_REL);
    if (result != 0) {
        printk(KERN_ERR "[PRESS] %s: Can't start timer\n", __func__);
        return;
    }

    return;
}

/******************************************************************************
 * NAME       : press_timer_func
 * FUNCTION   : call work function (thread of timer)
 * REMARKS    :
 *****************************************************************************/
static enum hrtimer_restart press_timer_func(struct hrtimer *timer)
{
    PRESS_DATA *press;
    int        result;

    press  = container_of(timer, PRESS_DATA, timer);
    result = queue_work(rohm_workqueue, &press->work);
    if (result == 0) {
        printk(KERN_ERR "[PRESS] %s: Can't register que.\n", __func__);
        printk(KERN_ERR "[PRESS] %s: result = 0x%x\n", __func__, result);
    }


    return (HRTIMER_NORESTART);
}

/******************************************************************************
 * NAME       : press_input_open
 * FUNCTION   : initialize device and open process
 * REMARKS    :
 *****************************************************************************/
static int press_input_open(struct input_dev *input)
{
    int        result = 0;

    printk(KERN_INFO "[PRESS] %s: input device open\n",__func__);

//    press = input_get_drvdata(input);

    /* initialize IC */
/*
    result = press_access_init(press->client);
    if (result < 0) {
        printk(KERN_ERR "%s: Can't initialize IC. result = 0x%x\n", __func__, result);
    }
*/


    return (result);
}

/******************************************************************************
 * NAME       : press_input_close
 * FUNCTION   : stop device and close process
 * REMARKS    :
 *****************************************************************************/
static void press_input_close(struct input_dev *input)
{
//    PRESS_DATA *press;

    printk(KERN_INFO "[PRESS] %s: input device close\n",__func__);

    /* copy client data */
//    press = input_get_drvdata(input);

    /* close IC */
//    press_access_exit(press->client);

    return;
}

/************************************************************
 *                   file system function                   *
 ***********************************************************/
/******************************************************************************
 * NAME       : bm1383_store_enable
 * FUNCTION   : write to power on or off BM1383
 * REMARKS    :
 *****************************************************************************/
static ssize_t bm1383_store_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_data;
    PRESS_DATA        *press;
    unsigned long     val;
    int               result;
    long              wait_sec;
    unsigned long     wait_nsec;

    /* initialize data */
    input_data = to_input_dev(dev);
    press  = input_get_drvdata(input_data);
    val    = simple_strtoul(buf, NULL, 10);

    printk(KERN_INFO "[PRESS] value(%ld) %s \n",val,__func__);

    result = -1;

    if ((val == MEASURE_ON) || (val == MEASURE_OFF)) {
        result = press_access_write_measurement_state(press->client, (unsigned char)val);
        if (result < 0) {
            printk(KERN_ERR "[PRESS] %s: Can't write press_access_write_measurement_state function. Result = 0x%x.\n", __func__, result);
        } else {
            if (val == MEASURE_ON) {
                if (press->measure_state == MEASURE_OFF) {
                    /* set to start timer to que */
                    if (press->delay_time < COMPLETE_MEASURE_WAIT_TIME) {
                        make_timer_val(COMPLETE_MEASURE_WAIT_TIME, &wait_sec, &wait_nsec);
                    } else {
                        make_timer_val(press->delay_time, &wait_sec, &wait_nsec);
                    }
                    result = hrtimer_start(&press->timer, ktime_set(wait_sec, wait_nsec), HRTIMER_MODE_REL);
                    if (result != 0) {
                        printk(KERN_ERR "[PRESS] %s: Can't start timer\n", __func__);
                    }
                }
                press->measure_state = MEASURE_ON;
            } else { // val == MEASURE_OFF
                /* process which stop measurement */
                /* clear to timer que */
                hrtimer_cancel(&press->timer);
                cancel_work_sync(&press->work);
                press->measure_state = MEASURE_OFF;
            }
        }
    } else {
        printk(KERN_INFO "%s: Can't execute.", __func__);
    }

    return (count);
}


/******************************************************************************
 * NAME       : bm1383_show_enable
 * FUNCTION   : read to power on or off BM1383
 * REMARKS    :
 *****************************************************************************/
static ssize_t bm1383_show_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct input_dev *input_data;
    PRESS_DATA        *press;
    int               result;

    /* initialize data */
    input_data = to_input_dev(dev);
    press  = input_get_drvdata(input_data);

    result = sprintf(buf, "%d\n", press->measure_state);

    return (result);
}

/******************************************************************************
 * NAME       : bm1383_store_delaytime
 * FUNCTION   : write delay time of BM1383
 * REMARKS    :
 *****************************************************************************/
static ssize_t bm1383_store_delaytime(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_data;
    PRESS_DATA        *press;
    unsigned long     val;
    /* initialize data */
    input_data = to_input_dev(dev);
    press  = input_get_drvdata(input_data);
    val    = simple_strtoul(buf, NULL, 10);
    val    = val * 1000000;  // ms -> ns
    printk(KERN_INFO "[PRESS] delay (%ld) %s \n",val,__func__);

    if (val < MIN_DELAY_TIME) {
        val = MIN_DELAY_TIME;
    }
    press->delay_time = val;

    return (count);
}

/******************************************************************************
 * NAME       : bm1383_show_delaytime
 * FUNCTION   : read to delay time of BM1383
 * REMARKS    :
 *****************************************************************************/
static ssize_t bm1383_show_delaytime(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct input_dev *input_data;
    PRESS_DATA        *press;
    int               result;

    /* initialize data */
    input_data = to_input_dev(dev);
    press  = input_get_drvdata(input_data);

    result = sprintf(buf, "%d\n", (int)(press->delay_time/1000000));  // ns -> ms

    return (result);
}

/******************************************************************************
 * NAME       : bm1383_show_data
 * FUNCTION   : read pressure data from BM1383
 * REMARKS    :
 *****************************************************************************/
static ssize_t bm1383_show_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct input_dev *input_data;
    PRESS_DATA        *press;
    int               result;
    unsigned int      pdata;

    /* initialize data */
    input_data = to_input_dev(dev);
    press  = input_get_drvdata(input_data);

    result = press_access_read_measurement_data(press->client, &pdata);
    if (result >= 0) {
        result = sprintf(buf, "0x%08X\n", pdata);
    }
    return (result);
}

/******************************************************************************
 * NAME       : bm1383_show_version
 * FUNCTION   : read version
 * REMARKS    :
 *****************************************************************************/
static ssize_t bm1383_show_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    int result;
    result = sprintf(buf, "driver version %s\n", bm1383_driver_ver);

    return (result);
}

/************************************************************
 *                     access function                      *
 ***********************************************************/
/******************************************************************************
 * NAME       : press_access_init
 * FUNCTION   : initialize BM1383
 * REMARKS    :
 *****************************************************************************/
static int press_access_init(struct i2c_client *client)
{
    int result;

    result = i2c_smbus_write_byte_data(client, BM1383_POWER_DOWN, 0x01);
    if (result < 0) {
        printk(KERN_ERR "[PRESS] %s: Can't write POWER_DOWN Register(0x12)\n", __func__);
        return (result);
    }

    result = i2c_smbus_write_byte_data(client, BM1383_SLEEP, 0x00);
    if (result < 0) {
        printk(KERN_ERR "[PRESS] %s: Can't write SLEEP Register(0x13)\n", __func__);
        return (result);
    }

    result = i2c_smbus_write_byte_data(client, BM1383_SLEEP, 0x01);
    if (result < 0) {
        printk(KERN_ERR "[PRESS] %s: Can't write SLEEP Register(0x13)\n", __func__);
        return (result);
    }

    return (result);
}

/******************************************************************************
 * NAME       : press_access_exit
 * FUNCTION   : shutdown BM1383
 * REMARKS    :
 *****************************************************************************/
/*
static int press_access_exit(struct i2c_client *client)
{
    int result;

    result = i2c_smbus_write_byte_data(client, BM1383_SLEEP, 0x00);
    if (result < 0) {
        printk(KERN_ERR "%s: Can't write SLEEP Register(0x13)\n", __func__);
        return (result);
    }

    result = i2c_smbus_write_byte_data(client, BM1383_POWER_DOWN, 0x00);
    if (result < 0) {
        printk(KERN_ERR "%s: Can't write POWER_DOWN Register(0x12)\n", __func__);
        return (result);
    }

    return (result);
}
*/
/******************************************************************************
 * NAME       : press_access_write_measurement_state
 * FUNCTION   : measurement on or off to BM1383
 * REMARKS    : 1 is measurement on, 0 is measurement off
 *****************************************************************************/
static int press_access_write_measurement_state(struct i2c_client *client, unsigned char val)
{
    int result;

    if (val == MEASURE_ON) {
        result = i2c_smbus_write_byte_data(client, BM1383_MODE_CONTROL, BM1383_MODE_CONTROL_ON);        
    } else if (val == MEASURE_OFF) {
        result = i2c_smbus_write_byte_data(client, BM1383_MODE_CONTROL, BM1383_MODE_CONTROL_OFF);
    } else {
        result = -EINVAL;
    }

    return (result);
}


/******************************************************************************
 * NAME       : press_access_read_measurement_data
 * FUNCTION   : Read measurement data from BM1383
 * REMARKS    : 
 *****************************************************************************/
static int press_access_read_measurement_data(struct i2c_client *client, unsigned int *val)
{
    int           result;
    unsigned char buf[3] = {0};

    /* read pressure data */
    result = i2c_smbus_read_i2c_block_data(client, BM1383_PRESSURE_MSB, sizeof(buf), buf);
    if (result < 0) {
        printk(KERN_ERR "[PRESS] %s: error result = 0x%08X.\n", __func__, result);
        return (result);
    } else {
        *val = ((unsigned int)buf[0] << 16) | ((unsigned int)buf[1] << 8) | (buf[2] & 0xFC);
    }

    return (result);
}

/******************************************************************************
 * NAME       : make_timer_val
 * FUNCTION   : Convert from NS time to ktime_set function value. 
 * REMARKS    :
 *****************************************************************************/
static void make_timer_val(unsigned long data, unsigned long *sec, unsigned long *nsec)
{
    /* the setting value from application */
    *sec  = (data / SN_TIME_UNIT);
    *nsec = (data - (*sec * SN_TIME_UNIT));

    return;
}

MODULE_DESCRIPTION("ROHM BM1383 Pressure Sensor Driver");
MODULE_LICENSE("GPL");

module_init(press_init);
module_exit(press_exit);

