# This software is contributed or developed by KYOCERA Corporation.
# (C) 2015 KYOCERA Corporation

obj-$(CONFIG_SENSORS) += sensors_class.o
obj-$(CONFIG_INPUT_SENSOR_PRESSURE) += rohm_bm1383_i2c.o
obj-$(CONFIG_INPUT_SENSOR_PROXIMITY) += gp2ap030a.o
obj-$(CONFIG_SENSORS_YAS_MAGNETOMETER) += yas_mag_drv-yas537.o yas_mag_kernel.o
CFLAGS_yas_mag_kernel.o += -Idrivers/staging/iio
obj-$(CONFIG_SENSORS_YAS_ACC_GYRO_6AXIS_SENSOR)+=yas_acc_gyro_drv-lsm6ds3.o yas_acc_gyro_kernel.o
CFLAGS_yas_acc_gyro_kernel.o += -Idrivers/staging/iio
