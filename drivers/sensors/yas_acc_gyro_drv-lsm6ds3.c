 /*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */

/*
 * Copyright (c) 2015 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
#include "yas.h"
#if YAS_ACC_DRIVER == YAS_ACC_DRIVER_LSM6DS3 \
	|| YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_LSM6DS3
#define YAS_ACC_FS_2G                                                        (0)
#define YAS_ACC_FS_4G                                                        (1)
#define YAS_ACC_FS_8G                                                        (2)
#define YAS_ACC_FS_16G                                                       (3)
#define YAS_ACC_FS                                                 YAS_ACC_FS_4G
#if YAS_ACC_FS == YAS_ACC_FS_2G
#define YAS_ACC_RESOLUTION                                               (16393)
#elif YAS_ACC_FS == YAS_ACC_FS_4G
#define YAS_ACC_RESOLUTION                                                (8197)
#elif YAS_ACC_FS == YAS_ACC_FS_8G
#define YAS_ACC_RESOLUTION                                                (4098)
#elif YAS_ACC_FS == YAS_ACC_FS_16G
#define YAS_ACC_RESOLUTION                                                (2049)
#else
#define YAS_ACC_RESOLUTION                                               (16393)
#endif
#define YAS_GYRO_RESOLUTION                                                 (70)
#define YAS_GRAVITY_EARTH                                              (9806550)
#define YAS_WHO_AM_I                                                      (0x0f)
#define YAS_WHO_AM_I_ID                                                   (0x69)
#define YAS_CTRL1_XL                                                      (0x10)
#if YAS_ACC_FS == YAS_ACC_FS_2G
#define YAS_CTRL1_XL_FS                                                   (0x00)
#elif YAS_ACC_FS == YAS_ACC_FS_4G
#define YAS_CTRL1_XL_FS                                                   (0x08)
#elif YAS_ACC_FS == YAS_ACC_FS_8G
#define YAS_CTRL1_XL_FS                                                   (0x0c)
#elif YAS_ACC_FS == YAS_ACC_FS_16G
#define YAS_CTRL1_XL_FS                                                   (0x04)
#else
#define YAS_CTRL1_XL_FS                                                   (0x00)
#endif
#define YAS_CTRL2_G                                                       (0x11)
#define YAS_CTRL2_G_FS                                                    (0x0c)
#define YAS_OUTX_L_G                                                      (0x22)
#define YAS_OUTX_L_XL                                                     (0x28)
#define YAS_CTRL3_C                                                       (0x12)
#define YAS_CTRL3_C_BDU                                                   (0x40)
#define YAS_CTRL3_C_IF_INC                                                (0x04)
#define YAS_CTRL3_C_SW_RESET                                              (0x01)
#define YAS_ODR_1660HZ                                                    (0x80)
#define YAS_ODR_833HZ                                                     (0x70)
#define YAS_ODR_416HZ                                                     (0x60)
#define YAS_ODR_208HZ                                                     (0x50)
#define YAS_ODR_104HZ                                                     (0x40)
#define YAS_ODR_52HZ                                                      (0x30)
#define YAS_ODR_26HZ                                                      (0x20)
#define YAS_ODR_13HZ                                                      (0x10)
#define BUF_SIZE                                                            (12)
#define YAS_DEFAULT_POSITION                                                 (4)
/*
   YAS_STARTUP_TIME

   Startup time is required after powerup, but t,su Startup time is unknown.
   So, Startup time is assumed here as 100 [msec].
*/
#define YAS_STARTUP_TIME                                                (100000)
#define YAS_RESET_WAIT_TIME                                               (1000)
#define YAS_RESET_RETRY_CNT                                                 (10)

struct yas_odr {
	int delay;
	uint8_t odr;
};

struct yas_sensor {
	uint8_t odr;
	int enable;
	int delay;
};

struct yas_driver {
	int initialized;
	int position;
	struct yas_sensor a;
	struct yas_sensor g;
	struct yas_driver_callback cbk;
};

static const struct yas_odr yas_odr_tbl[] = {
	{1,  YAS_ODR_1660HZ},
	{2,  YAS_ODR_833HZ},
	{3,  YAS_ODR_416HZ},
	{5,  YAS_ODR_208HZ},
	{10, YAS_ODR_104HZ},
#if 0
	{20, YAS_ODR_52HZ},
	{40, YAS_ODR_26HZ},
	{80, YAS_ODR_13HZ},
#endif
};

static const int yas_position_map[][3][3] = {
	{ { 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1} }, /* top/upper-left */
	{ { 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1} }, /* top/upper-right */
	{ {-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1} }, /* top/lower-right */
	{ { 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1} }, /* top/lower-left */
	{ {-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1} }, /* bottom/upper-left */
	{ { 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1} }, /* bottom/upper-right */
	{ { 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1} }, /* bottom/lower-right */
	{ { 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1} }, /* bottom/lower-left */
};

static struct yas_driver driver;

static int yas_open_dev(void);
static int yas_close_dev(void);
static int yas_read_reg(uint8_t adr, uint8_t *val);
static int yas_write_reg(uint8_t adr, uint8_t val);
static int yas_read_reg_data6(uint8_t adr, uint8_t *val);
static int yas_read_reg_data12(uint8_t adr, uint8_t *val);
static void yas_usleep(int delay);
static uint32_t yas_current_time(void);
static void yas_set_odr(int32_t type, int delay);
static int yas_reset(void);
static int yas_power_up(int32_t type);
static int yas_power_down(int32_t type);
static int yas_init(void);
static int yas_term(void);
static int yas_get_delay(int32_t type);
static int yas_set_delay(int32_t type, int delay);
static int yas_get_enable(int32_t type);
static int yas_set_enable(int32_t type, int enable);
static int yas_get_position(void);
static int yas_set_position(int position);
static int yas_measure(int32_t type, struct yas_data *raw, int num);
static int yas_ext(int32_t cmd, void *p);

static int
yas_open_dev(void)
{
	return driver.cbk.device_open(YAS_TYPE_ACC | YAS_TYPE_GYRO);
}

static int
yas_close_dev(void)
{
	return driver.cbk.device_close(YAS_TYPE_ACC | YAS_TYPE_GYRO);
}

static int
yas_read_reg(uint8_t adr, uint8_t *val)
{
	return driver.cbk.device_read(YAS_TYPE_ACC | YAS_TYPE_GYRO
				      , adr, val, 1);
}

static int
yas_write_reg(uint8_t adr, uint8_t val)
{
	int rt;
	rt = driver.cbk.device_write(YAS_TYPE_ACC | YAS_TYPE_GYRO
				     , adr, &val, 1);
	return rt;
}

static int
yas_read_reg_data6(uint8_t adr, uint8_t *val)
{
	return driver.cbk.device_read(YAS_TYPE_ACC | YAS_TYPE_GYRO
				      , adr, val, 6);
}

static int
yas_read_reg_data12(uint8_t adr, uint8_t *val)
{
	return driver.cbk.device_read(YAS_TYPE_ACC | YAS_TYPE_GYRO
				      , adr, val, 12);
}

static void
yas_usleep(int delay)
{
	driver.cbk.usleep(delay);
}

static uint32_t
yas_current_time(void)
{
	if (driver.cbk.current_time == NULL)
		return 0;
	return driver.cbk.current_time();
}

static void
yas_set_odr(int32_t type, int delay)
{
	int i;
	if (type & YAS_TYPE_ACC) {
		for (i = 1; i < NELEMS(yas_odr_tbl) &&
			     delay >= yas_odr_tbl[i].delay; i++)
			;
		driver.a.delay = delay;
		driver.a.odr = yas_odr_tbl[i-1].odr;
	}
	if (type & YAS_TYPE_GYRO) {
		for (i = 1; i < NELEMS(yas_odr_tbl) &&
			     delay >= yas_odr_tbl[i].delay; i++)
			;
		driver.g.delay = delay;
		driver.g.odr = yas_odr_tbl[i-1].odr;
	}
}

static int
yas_reset(void)
{
	uint8_t reg;
	int cnt;
	if (!(driver.a.enable | driver.g.enable)) {
		if (yas_write_reg(YAS_CTRL3_C
				  , YAS_CTRL3_C_SW_RESET) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		for (cnt = 0; cnt < YAS_RESET_RETRY_CNT; cnt++) {
			if (yas_read_reg(YAS_CTRL3_C, &reg) < 0)
				return YAS_ERROR_DEVICE_COMMUNICATION;
			if (!(reg & YAS_CTRL3_C_SW_RESET))
				break;
			yas_usleep(YAS_RESET_WAIT_TIME);
		}
		if (cnt == YAS_RESET_RETRY_CNT)
			return YAS_ERROR_INITIALIZE;
	}
	return YAS_NO_ERROR;
}

static int
yas_power_up(int32_t type)
{
	int er;
	er = yas_reset();
	if (er != YAS_NO_ERROR)
		return er;
	if (yas_write_reg(YAS_CTRL3_C, YAS_CTRL3_C_BDU|YAS_CTRL3_C_IF_INC) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (type & YAS_TYPE_ACC) {
		if (yas_write_reg(YAS_CTRL1_XL
				  , YAS_CTRL1_XL_FS | driver.a.odr) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	if (type & YAS_TYPE_GYRO) {
		if (yas_write_reg(YAS_CTRL2_G
				  , YAS_CTRL2_G_FS | driver.g.odr) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	yas_usleep(YAS_STARTUP_TIME);
	return YAS_NO_ERROR;
}

static int
yas_power_down(int32_t type)
{
	if ((type & YAS_TYPE_ACC) && driver.a.enable) {
		if (yas_write_reg(YAS_CTRL1_XL, 0) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	if ((type & YAS_TYPE_GYRO) && driver.g.enable) {
		if (yas_write_reg(YAS_CTRL2_G, 0) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	return YAS_NO_ERROR;
}

static int
yas_init(void)
{
	uint8_t id;
	if (driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (yas_open_dev() < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_read_reg(YAS_WHO_AM_I, &id) < 0) {
		yas_close_dev();
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	if (id != YAS_WHO_AM_I_ID) {
		yas_close_dev();
		return YAS_ERROR_CHIP_ID;
	}
	driver.a.enable = 0;
	driver.g.enable = 0;
	driver.a.delay = YAS_DEFAULT_SENSOR_DELAY;
	driver.g.delay = YAS_DEFAULT_SENSOR_DELAY;
	driver.position = YAS_DEFAULT_POSITION;
	yas_set_odr(YAS_TYPE_ACC | YAS_TYPE_GYRO, YAS_DEFAULT_SENSOR_DELAY);
	yas_power_down(YAS_TYPE_ACC | YAS_TYPE_GYRO);
	yas_close_dev();
	driver.initialized = 1;
	return YAS_NO_ERROR;
}

static int
yas_term(void)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	yas_set_enable(YAS_TYPE_ACC | YAS_TYPE_GYRO, 0);
	driver.initialized = 0;
	return YAS_NO_ERROR;
}

static int
yas_get_delay(int32_t type)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (type & YAS_TYPE_ACC)
		return driver.a.delay;
	if (type & YAS_TYPE_GYRO)
		return driver.g.delay;
	return YAS_ERROR_ARG;
}

static int
yas_set_delay(int32_t type, int delay)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (delay < 0)
		return YAS_ERROR_ARG;
	type &= (YAS_TYPE_ACC | YAS_TYPE_GYRO);
	if (!type)
		return YAS_ERROR_ARG;
	yas_set_odr(type, delay);
	if ((type & YAS_TYPE_ACC) && driver.a.enable) {
		if (yas_write_reg(YAS_CTRL1_XL
				  , YAS_CTRL1_XL_FS | driver.a.odr) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	if ((type & YAS_TYPE_GYRO) && driver.g.enable) {
		if (yas_write_reg(YAS_CTRL2_G
				  , YAS_CTRL2_G_FS | driver.g.odr) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	yas_usleep(YAS_STARTUP_TIME);
	return YAS_NO_ERROR;
}

static int
yas_get_enable(int32_t type)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (type & YAS_TYPE_ACC)
		return driver.a.enable;
	if (type & YAS_TYPE_GYRO)
		return driver.g.enable;
	return YAS_ERROR_ARG;
}

static int
yas_set_enable(int32_t type, int enable)
{
	int enable_a, enable_g;
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	enable = !!enable;
	type &= (YAS_TYPE_ACC | YAS_TYPE_GYRO);
	if (!type)
		return YAS_ERROR_ARG;
	if (type & YAS_TYPE_ACC)
		enable_a = enable;
	else
		enable_a = driver.a.enable;
	if (type & YAS_TYPE_GYRO)
		enable_g = enable;
	else
		enable_g = driver.g.enable;
	if ((enable_a + enable_g) == (driver.a.enable + driver.g.enable)) {
		driver.a.enable = enable_a;
		driver.g.enable = enable_g;
		return YAS_NO_ERROR;
	}
	if (!driver.a.enable && enable_a) {
		if (!driver.g.enable)
			if (yas_open_dev() < 0)
				return YAS_ERROR_DEVICE_COMMUNICATION;
		if (yas_power_up(YAS_TYPE_ACC) < 0) {
			if (!driver.g.enable)
				yas_close_dev();
			return YAS_ERROR_DEVICE_COMMUNICATION;
		}
	}
	if (driver.a.enable && !enable_a) {
		yas_power_down(YAS_TYPE_ACC);
		if (!driver.g.enable)
			yas_close_dev();
	}
	driver.a.enable = enable_a;
	if (!driver.g.enable && enable_g) {
		if (!driver.a.enable)
			if (yas_open_dev() < 0)
				return YAS_ERROR_DEVICE_COMMUNICATION;
		if (yas_power_up(YAS_TYPE_GYRO) < 0) {
			if (!driver.a.enable)
				yas_close_dev();
			return YAS_ERROR_DEVICE_COMMUNICATION;
		}
	}
	if (driver.g.enable && !enable_g) {
		yas_power_down(YAS_TYPE_GYRO);
		if (!driver.a.enable)
			yas_close_dev();
	}
	driver.g.enable = enable_g;
	return YAS_NO_ERROR;
}

static int
yas_get_position(void)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	return driver.position;
}

static int
yas_set_position(int position)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (position < 0 || 7 < position)
		return YAS_ERROR_ARG;
	driver.position = position;
	return YAS_NO_ERROR;
}

static int
yas_measure(int32_t type, struct yas_data *raw, int num)
{
	uint8_t buf[BUF_SIZE];
	int16_t dat[3];
	int num_received;
	int i, j;
	num_received = 0;
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (raw == NULL || num < 0)
		return YAS_ERROR_ARG;
	if (num == 0)
		return 0;
	type &= (YAS_TYPE_ACC | YAS_TYPE_GYRO);
	if ((type & YAS_TYPE_ACC) && !driver.a.enable)
		type &= ~YAS_TYPE_ACC;
	if ((type & YAS_TYPE_GYRO) && !driver.g.enable)
		type &= ~YAS_TYPE_GYRO;
	if (!type)
		return 0;
	if (type & YAS_TYPE_ACC)
		num_received++;
	if (type & YAS_TYPE_GYRO) {
		if (num == num_received)
			type &= ~YAS_TYPE_GYRO;
		else
			num_received++;
	}
	if (type == (YAS_TYPE_ACC | YAS_TYPE_GYRO)) {
		if (yas_read_reg_data12(YAS_OUTX_L_G, buf) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		for (i = 0; i < 3; i++)
			dat[i] = (int16_t)(((int16_t)((buf[i*2+1+6] << 8))
					    | buf[i*2+6]));
		for (i = 0; i < 3; i++) {
			raw[0].xyz.v[i] = 0;
			for (j = 0; j < 3; j++)
				raw[0].xyz.v[i] += dat[j] *
					yas_position_map[driver.position][i][j];
			raw[0].xyz.v[i] *=
				(YAS_GRAVITY_EARTH / YAS_ACC_RESOLUTION);
		}
		raw[0].type = YAS_TYPE_ACC;
		raw[0].timestamp = yas_current_time();
		raw[0].accuracy = 0;
		for (i = 0; i < 3; i++)
			dat[i] = (int16_t)(((int16_t)((buf[i*2+1] << 8))
					    | buf[i*2]));
		for (i = 0; i < 3; i++) {
			raw[1].xyz.v[i] = 0;
			for (j = 0; j < 3; j++)
				raw[1].xyz.v[i] += dat[j] *
					yas_position_map[driver.position][i][j];
			raw[1].xyz.v[i] *= YAS_GYRO_RESOLUTION;
		}
		raw[1].type = YAS_TYPE_GYRO;
		raw[1].timestamp = yas_current_time();
		raw[1].accuracy = 0;
	}
	if (type == YAS_TYPE_ACC) {
		if (yas_read_reg_data6(YAS_OUTX_L_XL, buf) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		for (i = 0; i < 3; i++)
			dat[i] = (int16_t)(((int16_t)((buf[i*2+1] << 8))
					    | buf[i*2]));
		for (i = 0; i < 3; i++) {
			raw[0].xyz.v[i] = 0;
			for (j = 0; j < 3; j++)
				raw[0].xyz.v[i] += dat[j] *
					yas_position_map[driver.position][i][j];
			raw[0].xyz.v[i] *=
				(YAS_GRAVITY_EARTH / YAS_ACC_RESOLUTION);
		}
		raw[0].type = YAS_TYPE_ACC;
		raw[0].timestamp = yas_current_time();
		raw[0].accuracy = 0;
	}
	if (type == YAS_TYPE_GYRO) {
		if (yas_read_reg_data6(YAS_OUTX_L_G, buf) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		for (i = 0; i < 3; i++)
			dat[i] = (int16_t)(((int16_t)((buf[i*2+1] << 8))
					    | buf[i*2]));
		for (i = 0; i < 3; i++) {
			raw[0].xyz.v[i] = 0;
			for (j = 0; j < 3; j++)
				raw[0].xyz.v[i] += dat[j] *
					yas_position_map[driver.position][i][j];
			raw[0].xyz.v[i] *= YAS_GYRO_RESOLUTION;
		}
		raw[0].type = YAS_TYPE_GYRO;
		raw[0].timestamp = yas_current_time();
		raw[0].accuracy = 0;
	}
	return num_received;
}

static int
yas_ext(int32_t cmd, void *p)
{
	(void)cmd;
	(void)p;
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	return YAS_NO_ERROR;
}

int
yas_acc_gyro_driver_init(struct yas_acc_gyro_driver *f)
{
	if (f == NULL || f->callback.device_open == NULL
	    || f->callback.device_close == NULL
	    || f->callback.device_read == NULL
	    || f->callback.device_write == NULL
	    || f->callback.usleep == NULL)
		return YAS_ERROR_ARG;
	f->init = yas_init;
	f->term = yas_term;
	f->get_delay = yas_get_delay;
	f->set_delay = yas_set_delay;
	f->get_enable = yas_get_enable;
	f->set_enable = yas_set_enable;
	f->get_position = yas_get_position;
	f->set_position = yas_set_position;
	f->measure = yas_measure;
	f->ext = yas_ext;
	driver.cbk = f->callback;
	yas_term();
	return YAS_NO_ERROR;
}
#endif
