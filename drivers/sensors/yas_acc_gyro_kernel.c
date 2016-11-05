 /*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */

/*
 * Copyright (c) 2015 Yamaha Corporation
 *
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/android_alarm.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "buffer.h"
#include "iio.h"
#include "ring_sw.h"
#include "sysfs.h"
#include "trigger.h"
#include "trigger_consumer.h"
#include "yas.h"

static struct i2c_client *this_client;

enum {
	YAS_SCAN_ACCEL_X,
	YAS_SCAN_ACCEL_Y,
	YAS_SCAN_ACCEL_Z,
	YAS_SCAN_ACCEL_TIMESTAMP,
};
enum {
	YAS_SCAN_GYRO_X,
	YAS_SCAN_GYRO_Y,
	YAS_SCAN_GYRO_Z,
	YAS_SCAN_GYRO_TIMESTAMP,
};
struct yas_state {
	struct mutex lock;
	struct i2c_client *client;
	spinlock_t gyro_spin_lock;
	struct yas_acc_gyro_driver ag;
	spinlock_t accel_spin_lock;
	struct iio_dev *accel_dev;
	struct iio_dev *gyro_dev;
	struct iio_trigger *accel_trig;
	struct iio_trigger *gyro_trig;
	struct delayed_work accel_work;
	struct delayed_work gyro_work;
	int16_t accel_sampling_frequency;
	int16_t gyro_sampling_frequency;
	atomic_t accel_pseudo_irq_enable;
	atomic_t gyro_pseudo_irq_enable;
	int32_t gyro_data[3];
	int32_t gyro_bias[3];
	int32_t accel_data[3];
	int32_t accel_bias[3];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend sus;
#endif
};

static int yas_device_open(int32_t type)
{
	return 0;
}

static int yas_device_close(int32_t type)
{
	return 0;
}

static int yas_device_write(int32_t type, uint8_t addr, const uint8_t *buf,
		int len)
{
	uint8_t tmp[2];
	if (sizeof(tmp) - 1 < len)
		return -1;
	tmp[0] = addr;
	memcpy(&tmp[1], buf, len);
	if (i2c_master_send(this_client, tmp, len + 1) < 0)
		return -1;
	return 0;
}

static int yas_device_read(int32_t type, uint8_t addr, uint8_t *buf, int len)
{
	struct i2c_msg msg[2];
	int err;
	msg[0].addr = this_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &addr;
	msg[1].addr = this_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;
	err = i2c_transfer(this_client->adapter, msg, 2);
	if (err != 2) {
		dev_err(&this_client->dev,
				"i2c_transfer() read error: "
				"slave_addr=%02x, reg_addr=%02x, err=%d\n",
				this_client->addr, addr, err);
		return err;
	}
	return 0;
}

static void yas_usleep(int us)
{
	usleep_range(us, us + 1000);
}

static int64_t yas_current_time(void)
{
    return ktime_to_ns(alarm_get_elapsed_realtime());
	//return jiffies_to_msecs(jiffies);
}

static int yas_pseudo_irq_enable(int32_t type)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	if (type & YAS_TYPE_ACC) {
		if (!atomic_cmpxchg(&st->accel_pseudo_irq_enable, 0, 1)) {
			mutex_lock(&st->lock);
			st->ag.set_enable(YAS_TYPE_ACC, 1);
			mutex_unlock(&st->lock);
			schedule_delayed_work(&st->accel_work, 0);
		}
	}
	if (type & YAS_TYPE_GYRO) {
		if (!atomic_cmpxchg(&st->gyro_pseudo_irq_enable, 0, 1)) {
			mutex_lock(&st->lock);
			st->ag.set_enable(YAS_TYPE_GYRO, 1);
			mutex_unlock(&st->lock);
			schedule_delayed_work(&st->gyro_work,
					msecs_to_jiffies(10));
		}
	}
	return 0;
}

static int yas_pseudo_irq_disable(int32_t type)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	if (type & YAS_TYPE_ACC) {
		if (atomic_cmpxchg(&st->accel_pseudo_irq_enable, 1, 0)) {
			cancel_delayed_work_sync(&st->accel_work);
			mutex_lock(&st->lock);
			st->ag.set_enable(YAS_TYPE_ACC, 0);
			mutex_unlock(&st->lock);
		}
	}
	if (type & YAS_TYPE_GYRO) {
		if (atomic_cmpxchg(&st->gyro_pseudo_irq_enable, 1, 0)) {
			cancel_delayed_work_sync(&st->gyro_work);
			mutex_lock(&st->lock);
			st->ag.set_enable(YAS_TYPE_GYRO, 0);
			mutex_unlock(&st->lock);
		}
	}
	return 0;
}

static int yas_set_pseudo_irq(int enable, int32_t type)
{
	if (enable)
		yas_pseudo_irq_enable(type);
	else
		yas_pseudo_irq_disable(type);
	return 0;
}

static irqreturn_t yas_accel_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct iio_buffer *buffer = indio_dev->buffer;
	struct yas_state *st = i2c_get_clientdata(this_client);
	int len = 0, i, j;
	size_t datasize = buffer->access->get_bytes_per_datum(buffer);
	int32_t *acc;

	acc = kmalloc(datasize, GFP_KERNEL);
	if (acc == NULL) {
		dev_err(indio_dev->dev.parent,
				"memory alloc failed in buffer bh");
		goto done;
	}
	if (!bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength)) {
		j = 0;
		for (i = 0; i < 3; i++) {
			if (test_bit(i, indio_dev->active_scan_mask)) {
				acc[j] = st->accel_data[i];
				j++;
			}
		}
		len = j * 4;
	}

	/* Guaranteed to be aligned with 8 byte boundary */
	if (buffer->scan_timestamp)
		*(s64 *)(((phys_addr_t)acc + len
					+ sizeof(s64) - 1) & ~(sizeof(s64) - 1))
			= pf->timestamp;
	buffer->access->store_to(buffer, (u8 *)acc, pf->timestamp);
	iio_trigger_notify_done(st->accel_trig);
	kfree(acc);
done:
	return IRQ_HANDLED;
}

static irqreturn_t yas_gyro_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct iio_buffer *buffer = indio_dev->buffer;
	struct yas_state *st = i2c_get_clientdata(this_client);
	int len = 0, i, j;
	size_t datasize = buffer->access->get_bytes_per_datum(buffer);
	int32_t *gyro;

	gyro = (int32_t *) kmalloc(datasize, GFP_KERNEL);
	if (gyro == NULL) {
		dev_err(indio_dev->dev.parent,
				"memory alloc failed in buffer bh");
		goto done;
	}
	if (!bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength)) {
		j = 0;
		for (i = 0; i < 3; i++) {
			if (test_bit(i, indio_dev->active_scan_mask)) {
				gyro[j] = st->gyro_data[i];
				j++;
			}
		}
		len = j * 4;
	}

	/* Guaranteed to be aligned with 8 byte boundary */
	if (buffer->scan_timestamp)
		*(s64 *)(((phys_addr_t)gyro + len
					+ sizeof(s64) - 1) & ~(sizeof(s64) - 1))
			= pf->timestamp;
	buffer->access->store_to(buffer, (u8 *)gyro, pf->timestamp);

	iio_trigger_notify_done(st->gyro_trig);
	kfree(gyro);
done:
	return IRQ_HANDLED;
}

static int yas_accel_rdy_trigger_set_state(struct iio_trigger *trig, bool state)
{
	yas_set_pseudo_irq(state, YAS_TYPE_ACC);
	return 0;
}

static int yas_gyro_rdy_trigger_set_state(struct iio_trigger *trig, bool state)
{
	yas_set_pseudo_irq(state, YAS_TYPE_GYRO);
	return 0;
}

static const struct iio_trigger_ops yas_accel_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &yas_accel_rdy_trigger_set_state,
};

static const struct iio_trigger_ops yas_gyro_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &yas_gyro_rdy_trigger_set_state,
};

static irqreturn_t iio_pollfunc_store_boottime(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	pf->timestamp = ktime_to_ns(ktime_get_boottime());
	return IRQ_WAKE_THREAD;
}

static struct iio_trigger *yas_probe_trigger(struct iio_dev *indio_dev,
		irqreturn_t (*handler)(int, void *),
		const struct iio_trigger_ops *ops)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	struct iio_trigger *trig = NULL;
	int ret;
	indio_dev->pollfunc = iio_alloc_pollfunc(&iio_pollfunc_store_boottime,
			handler, IRQF_ONESHOT, indio_dev, "%s_consumer%d",
			indio_dev->name, indio_dev->id);
	if (indio_dev->pollfunc == NULL)
		goto error_ret;
	trig = iio_allocate_trigger("%s-dev%d", indio_dev->name,
			indio_dev->id);
	if (!trig)
		goto error_dealloc_pollfunc;
	trig->dev.parent = &st->client->dev;
	trig->ops = ops;
	trig->private_data = indio_dev;
	ret = iio_trigger_register(trig);
	if (ret)
		goto error_free_trig;
	return trig;

error_free_trig:
	iio_free_trigger(trig);
error_dealloc_pollfunc:
	iio_dealloc_pollfunc(indio_dev->pollfunc);
error_ret:
	return NULL;
}

static void yas_remove_trigger(struct iio_dev *indio_dev,
		struct iio_trigger *trig)
{
	iio_trigger_unregister(trig);
	iio_free_trigger(trig);
	iio_dealloc_pollfunc(indio_dev->pollfunc);
}

static const struct iio_buffer_setup_ops yas_buffer_setup_ops = {
	.preenable = &iio_sw_buffer_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
};

static void yas_remove_buffer(struct iio_dev *indio_dev)
{
	iio_buffer_unregister(indio_dev);
	iio_sw_rb_free(indio_dev->buffer);
};

static int yas_probe_buffer(struct iio_dev *indio_dev)
{
	int ret;
	struct iio_buffer *buffer;

	buffer = iio_sw_rb_allocate(indio_dev);
	if (!buffer) {
		ret = -ENOMEM;
		goto error_ret;
	}
	buffer->scan_timestamp = true;
	indio_dev->buffer = buffer;
	indio_dev->setup_ops = &yas_buffer_setup_ops;
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_buffer_register(indio_dev, indio_dev->channels,
			indio_dev->num_channels);
	if (ret)
		goto error_free_buf;
	return 0;

error_free_buf:
	iio_sw_rb_free(indio_dev->buffer);
error_ret:
	return ret;
}

static ssize_t yas_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret;
	mutex_lock(&st->lock);
	ret = st->ag.get_position();
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d\n", ret);
}

static ssize_t yas_position_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret, position;
	ret = kstrtoint(buf, 10, &position);
	if (ret)
		return -EINVAL;
	mutex_lock(&st->lock);
	ret = st->ag.set_position(position);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}

static ssize_t yas_accel_sampling_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	return sprintf(buf, "%d\n", st->accel_sampling_frequency);
}

static ssize_t yas_accel_sampling_frequency_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret, data, delay;
	ret = kstrtoint(buf, 10, &data);
	if (ret)
		return -EINVAL;
	if (data <= 0)
		return -EINVAL;
	mutex_lock(&st->lock);
	st->accel_sampling_frequency = data;
	delay = MSEC_PER_SEC / st->accel_sampling_frequency;
	st->ag.set_delay(YAS_TYPE_ACC, delay);
	mutex_unlock(&st->lock);
	return count;
}

static ssize_t yas_gyro_sampling_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	return sprintf(buf, "%d\n", st->gyro_sampling_frequency);
}

static ssize_t yas_gyro_sampling_frequency_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret, data, delay;
	ret = kstrtoint(buf, 10, &data);
	if (ret)
		return ret;
	if (data <= 0)
		return -EINVAL;
	mutex_lock(&st->lock);
	st->gyro_sampling_frequency = data;
	delay = MSEC_PER_SEC / st->gyro_sampling_frequency;
	st->ag.set_delay(YAS_TYPE_GYRO, delay);
	mutex_unlock(&st->lock);
	return count;
}

static int yas_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret = -EINVAL;

	mutex_lock(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_CALIBBIAS:
		if (chan->type == IIO_ACCEL) {
			st->accel_bias[chan->channel2 - IIO_MOD_X] = val;
			ret = 0;
		} else if (chan->type == IIO_ANGL_VEL) {
			st->gyro_bias[chan->channel2 - IIO_MOD_X] = val;
			ret = 0;
		} else {
			ret = -EINVAL;
		}
		break;
	}

	mutex_unlock(&st->lock);

	return ret;
}

static int yas_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val, int *val2,
		long mask)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret = -EINVAL;

	if (chan->type != IIO_ACCEL && chan->type != IIO_ANGL_VEL)
		return -EINVAL;

	mutex_lock(&st->lock);

	switch (mask) {
	case 0:
		if (chan->type == IIO_ACCEL)
			*val = st->accel_data[chan->channel2 - IIO_MOD_X];
		else
			*val = st->gyro_data[chan->channel2 - IIO_MOD_X];
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
	case IIO_CHAN_INFO_CALIBSCALE:
		if (chan->type == IIO_ACCEL) {
			/* Gain : counts / m/s^2 = 1000000 [um/s^2] */
			/* Scaling factor : 1000000 / Gain = 1 */
			*val = 0;
			*val2 = 1;
			ret = IIO_VAL_INT_PLUS_MICRO;
		} else {
			/* Gain : counts / dps = 1000 [mdps] */
			/* Scaling factor : 1000000 / Gain = 1000 */
			*val = 0;
			*val2 = 1000;
			ret = IIO_VAL_INT_PLUS_MICRO;
		}
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		if (chan->type == IIO_ACCEL) {
			*val = st->accel_bias[chan->channel2 - IIO_MOD_X];
			ret = IIO_VAL_INT;
		} else if (chan->type == IIO_ANGL_VEL) {
			*val = st->gyro_bias[chan->channel2 - IIO_MOD_X];
			ret = IIO_VAL_INT;
		} else
			ret = -EINVAL;
		break;
	}

	mutex_unlock(&st->lock);

	return ret;
}

static void yas_accel_work_func(struct work_struct *work)
{
	struct yas_data acc[1];
	struct yas_state *st = i2c_get_clientdata(this_client);
	uint32_t time_before, time_after;
	int32_t delay;
	unsigned long flags;
	int ret, i;

	time_before = jiffies_to_msecs(jiffies);
	mutex_lock(&st->lock);
	ret = st->ag.measure(YAS_TYPE_ACC, acc, 1);
	if (ret == 1) {
		for (i = 0; i < 3; i++)
			st->accel_data[i] = acc[0].xyz.v[i] - st->accel_bias[i];
	}
	mutex_unlock(&st->lock);
	if (ret == 1) {
		spin_lock_irqsave(&st->accel_spin_lock, flags);
		iio_trigger_poll(st->accel_trig, iio_get_time_ns());
		spin_unlock_irqrestore(&st->accel_spin_lock, flags);
	}
	time_after = jiffies_to_msecs(jiffies);
	delay = MSEC_PER_SEC / st->accel_sampling_frequency
		- (time_after - time_before);
	if (delay <= 0)
		delay = 1;
	schedule_delayed_work(&st->accel_work, msecs_to_jiffies(delay));
}

static void yas_gyro_work_func(struct work_struct *work)
{
	struct yas_data gyro[1];
	struct yas_state *st = i2c_get_clientdata(this_client);
	uint32_t time_before, time_after;
	int32_t delay;
	unsigned long flags;
	int ret, i;

	time_before = jiffies_to_msecs(jiffies);
	mutex_lock(&st->lock);
	ret = st->ag.measure(YAS_TYPE_GYRO, gyro, 1);
	if (ret == 1) {
		for (i = 0; i < 3; i++)
			st->gyro_data[i] = gyro[0].xyz.v[i] - st->gyro_bias[i];
	}
	mutex_unlock(&st->lock);
	if (ret == 1) {
		spin_lock_irqsave(&st->gyro_spin_lock, flags);
		iio_trigger_poll(st->gyro_trig, iio_get_time_ns());
		spin_unlock_irqrestore(&st->gyro_spin_lock, flags);
	}
	time_after = jiffies_to_msecs(jiffies);
	delay = MSEC_PER_SEC / st->gyro_sampling_frequency
		- (time_after - time_before);
	if (delay <= 0)
		delay = 1;
	schedule_delayed_work(&st->gyro_work, msecs_to_jiffies(delay));
}

#define YAS_INFO_MASK					\
	(IIO_CHAN_INFO_SCALE_SHARED_BIT |		\
	 IIO_CHAN_INFO_CALIBSCALE_SEPARATE_BIT |	\
	 IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT)

#define YAS_ACCELEROMETER_CHANNEL(axis)			\
{							\
	.type = IIO_ACCEL,				\
	.modified = 1,					\
	.channel2 = IIO_MOD_##axis,			\
	.info_mask = YAS_INFO_MASK,			\
	.scan_index = YAS_SCAN_ACCEL_##axis,		\
	.scan_type = IIO_ST('s', 32, 32, 0)		\
}

#define YAS_GYROSCOPE_CHANNEL(axis)			\
{							\
	.type = IIO_ANGL_VEL,				\
	.modified = 1,					\
	.channel2 = IIO_MOD_##axis,			\
	.info_mask = YAS_INFO_MASK,			\
	.scan_index = YAS_SCAN_GYRO_##axis,		\
	.scan_type = IIO_ST('s', 32, 32, 0)		\
}

static const struct iio_chan_spec yas_accel_channels[] = {
	YAS_ACCELEROMETER_CHANNEL(X),
	YAS_ACCELEROMETER_CHANNEL(Y),
	YAS_ACCELEROMETER_CHANNEL(Z),
	IIO_CHAN_SOFT_TIMESTAMP(YAS_SCAN_ACCEL_TIMESTAMP)
};

static const struct iio_chan_spec yas_gyro_channels[] = {
	YAS_GYROSCOPE_CHANNEL(X),
	YAS_GYROSCOPE_CHANNEL(Y),
	YAS_GYROSCOPE_CHANNEL(Z),
	IIO_CHAN_SOFT_TIMESTAMP(YAS_SCAN_GYRO_TIMESTAMP)
};

static IIO_DEVICE_ATTR_NAMED(accel_sampling_frequency, sampling_frequency,
		S_IRUSR|S_IWUSR, yas_accel_sampling_frequency_show,
		yas_accel_sampling_frequency_store, 0);
static IIO_DEVICE_ATTR_NAMED(gyro_sampling_frequency, sampling_frequency,
		S_IRUSR|S_IWUSR, yas_gyro_sampling_frequency_show,
		yas_gyro_sampling_frequency_store, 0);
static IIO_DEVICE_ATTR(position, S_IRUSR|S_IWUSR,
		yas_position_show, yas_position_store, 0);

static struct attribute *yas_accel_attributes[] = {
	&iio_dev_attr_accel_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_position.dev_attr.attr,
	NULL
};
static struct attribute *yas_gyro_attributes[] = {
	&iio_dev_attr_gyro_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_position.dev_attr.attr,
	NULL
};
static const struct attribute_group yas_accel_attribute_group = {
	.attrs = yas_accel_attributes,
};

static const struct attribute_group yas_gyro_attribute_group = {
	.attrs = yas_gyro_attributes,
};

static const struct iio_info yas_accel_info = {
	.read_raw = &yas_read_raw,
	.write_raw = &yas_write_raw,
	.attrs = &yas_accel_attribute_group,
	.driver_module = THIS_MODULE,
};

static const struct iio_info yas_gyro_info = {
	.read_raw = &yas_read_raw,
	.write_raw = &yas_write_raw,
	.attrs = &yas_gyro_attribute_group,
	.driver_module = THIS_MODULE,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void yas_early_suspend(struct early_suspend *h)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	if (atomic_read(&st->accel_pseudo_irq_enable)) {
		cancel_delayed_work_sync(&st->accel_work);
		st->ag.set_enable(YAS_TYPE_ACC, 0);
	}
	if (atomic_read(&st->gyro_pseudo_irq_enable)) {
		cancel_delayed_work_sync(&st->gyro_work);
		st->ag.set_enable(YAS_TYPE_ACC, 0);
	}
}

static void yas_late_resume(struct early_suspend *h)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	if (atomic_read(&st->accel_pseudo_irq_enable)) {
		st->ag.set_enable(YAS_TYPE_ACC, 1);
		schedule_delayed_work(&st->accel_work, 0);
	}
	if (atomic_read(&st->gyro_pseudo_irq_enable)) {
		st->ag.set_enable(YAS_TYPE_GYRO, 1);
		schedule_delayed_work(&st->gyro_work, 0);
	}
}
#endif

static int yas_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct iio_trigger *trig;
	struct yas_state *st;
	struct iio_dev *accel_dev, *gyro_dev;
	int ret, i;

	this_client = i2c;
	st = kzalloc(sizeof(struct yas_state), GFP_KERNEL);
	if (st == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	i2c_set_clientdata(i2c, st);

	accel_dev = iio_allocate_device(0);
	if (!accel_dev) {
		ret = -ENOMEM;
		goto error_free;
	}
	accel_dev->name = YAS_ACC_GYRO_NAME_A;
	accel_dev->dev.parent = &i2c->dev;
	accel_dev->info = &yas_accel_info;
	accel_dev->channels = yas_accel_channels;
	accel_dev->num_channels = ARRAY_SIZE(yas_accel_channels);
	accel_dev->modes = INDIO_DIRECT_MODE;
	st->accel_dev = accel_dev;

	gyro_dev = iio_allocate_device(0);
	if (!gyro_dev) {
		ret = -ENOMEM;
		goto error_free_accel;
	}
	gyro_dev->name = YAS_ACC_GYRO_NAME_G;
	gyro_dev->dev.parent = &i2c->dev;
	gyro_dev->info = &yas_gyro_info;
	gyro_dev->channels = yas_gyro_channels;
	gyro_dev->num_channels = ARRAY_SIZE(yas_gyro_channels);
	gyro_dev->modes = INDIO_DIRECT_MODE;
	st->gyro_dev = gyro_dev;

	st->client = i2c;
	st->accel_sampling_frequency = 20;
	st->gyro_sampling_frequency = 20;
	st->ag.callback.device_open = yas_device_open;
	st->ag.callback.device_close = yas_device_close;
	st->ag.callback.device_read = yas_device_read;
	st->ag.callback.device_write = yas_device_write;
	st->ag.callback.usleep = yas_usleep;
	st->ag.callback.current_time = yas_current_time;
	INIT_DELAYED_WORK(&st->accel_work, yas_accel_work_func);
	INIT_DELAYED_WORK(&st->gyro_work, yas_gyro_work_func);
	mutex_init(&st->lock);
#ifdef CONFIG_HAS_EARLYSUSPEND
	st->sus.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	st->sus.suspend = yas_early_suspend;
	st->sus.resume = yas_late_resume;
	register_early_suspend(&st->sus);
#endif
	for (i = 0; i < 3; i++) {
		st->gyro_data[i] = 0;
		st->gyro_bias[i] = 0;
		st->accel_data[i] = 0;
		st->accel_bias[i] = 0;
	}

	ret = yas_probe_buffer(accel_dev);
	if (ret)
		goto error_free_gyro;
	iio_scan_mask_set(accel_dev, accel_dev->buffer, YAS_SCAN_ACCEL_X);
	iio_scan_mask_set(accel_dev, accel_dev->buffer, YAS_SCAN_ACCEL_Y);
	iio_scan_mask_set(accel_dev, accel_dev->buffer, YAS_SCAN_ACCEL_Z);
	trig = yas_probe_trigger(accel_dev, yas_accel_handler, &yas_accel_ops);
	if (trig == NULL)
		goto error_remove_accel_buffer;
	st->accel_trig = trig;
	ret = iio_device_register(accel_dev);
	if (ret)
		goto error_remove_accel_trigger;

	ret = yas_probe_buffer(gyro_dev);
	if (ret)
		goto error_unregister_accel;
	iio_scan_mask_set(gyro_dev, gyro_dev->buffer, YAS_SCAN_GYRO_X);
	iio_scan_mask_set(gyro_dev, gyro_dev->buffer, YAS_SCAN_GYRO_Y);
	iio_scan_mask_set(gyro_dev, gyro_dev->buffer, YAS_SCAN_GYRO_Z);
	trig = yas_probe_trigger(gyro_dev, yas_gyro_handler, &yas_gyro_ops);
	if (trig == NULL)
		goto error_remove_gyro_buffer;
	st->gyro_trig = trig;
	ret = iio_device_register(gyro_dev);
	if (ret)
		goto error_remove_gyro_trigger;

	ret = yas_acc_gyro_driver_init(&st->ag);
	if (ret < 0) {
		ret = -EFAULT;
		goto error_unregister_gyro;
	}
	ret = st->ag.init();
	if (ret < 0) {
		ret = -EFAULT;
		goto error_unregister_gyro;
	}
	spin_lock_init(&st->accel_spin_lock);
	spin_lock_init(&st->gyro_spin_lock);
	return 0;

error_unregister_gyro:
	iio_device_unregister(gyro_dev);
error_remove_gyro_trigger:
	yas_remove_trigger(gyro_dev, st->gyro_trig);
error_remove_gyro_buffer:
	yas_remove_buffer(gyro_dev);
error_unregister_accel:
	iio_device_unregister(accel_dev);
error_remove_accel_trigger:
	yas_remove_trigger(accel_dev, st->accel_trig);
error_remove_accel_buffer:
	yas_remove_buffer(accel_dev);
error_free_gyro:
	iio_free_device(gyro_dev);
error_free_accel:
	iio_free_device(accel_dev);
error_free:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&st->sus);
#endif
	kfree(st);
error_ret:
	i2c_set_clientdata(i2c, NULL);
	this_client = NULL;
	return ret;
}

static int yas_remove(struct i2c_client *i2c)
{
	struct yas_state *st = NULL;
	if (this_client)
		st = i2c_get_clientdata(this_client);
	if (st) {
		yas_pseudo_irq_disable(YAS_TYPE_ACC | YAS_TYPE_GYRO);
		st->ag.term();

		iio_device_unregister(st->accel_dev);
		yas_remove_trigger(st->accel_dev, st->accel_trig);
		yas_remove_buffer(st->accel_dev);
		iio_free_device(st->accel_dev);

		iio_device_unregister(st->gyro_dev);
		yas_remove_trigger(st->gyro_dev, st->gyro_trig);
		yas_remove_buffer(st->gyro_dev);
		iio_free_device(st->gyro_dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&st->sus);
#endif
		kfree(st);
		i2c_set_clientdata(i2c, NULL);
		this_client = NULL;
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int yas_suspend(struct device *dev)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	if (atomic_read(&st->accel_pseudo_irq_enable)) {
		cancel_delayed_work_sync(&st->accel_work);
		st->ag.set_enable(YAS_TYPE_ACC, 0);
	}
	if (atomic_read(&st->gyro_pseudo_irq_enable)) {
		cancel_delayed_work_sync(&st->gyro_work);
		st->ag.set_enable(YAS_TYPE_GYRO, 0);
	}
	return 0;
}

static int yas_resume(struct device *dev)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	if (atomic_read(&st->accel_pseudo_irq_enable)) {
		st->ag.set_enable(YAS_TYPE_ACC, 1);
		schedule_delayed_work(&st->accel_work, 0);
	}
	if (atomic_read(&st->gyro_pseudo_irq_enable)) {
		st->ag.set_enable(YAS_TYPE_GYRO, 1);
		schedule_delayed_work(&st->gyro_work, 0);
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(yas_pm_ops, yas_suspend, yas_resume);
#define YAS_PM_OPS (&yas_pm_ops)
#else
#define YAS_PM_OPS NULL
#endif

static const struct i2c_device_id yas_id[] = {
	{YAS_ACC_GYRO_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, yas_id);

static struct i2c_driver yas_driver = {
	.driver = {
		.name	= YAS_ACC_GYRO_NAME,
		.owner	= THIS_MODULE,
		.pm	= YAS_PM_OPS,
	},
	.probe		= yas_probe,
	.remove		= __devexit_p(yas_remove),
	.id_table	= yas_id,
};
static int __init yas_init(void)
{
	return i2c_add_driver(&yas_driver);
}

static void __exit yas_exit(void)
{
	i2c_del_driver(&yas_driver);
}

module_init(yas_init);
module_exit(yas_exit);

MODULE_DESCRIPTION("Yamaha Acceleration and Gyroscope I2C driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("6.1.1101");
