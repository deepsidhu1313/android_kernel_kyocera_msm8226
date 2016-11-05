/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */
/******************************************************************************
 * MODULE     : rohm_bm1383_i2c.h
 * FUNCTION   : Driver header for BM1383, Pressure Sensor IC
 * AUTHOR     : Atsushi Momota
 * PROGRAMMED : Sensing Solution Group
 * REMARKS    :
 * COPYRIGHT  : Copyright (C) 2014 ROHM CO.,LTD.
 *****************************************************************************/
#ifndef _ROHM_BM1383_I2C_H_
#define _ROHM_BM1383_I2C_H_

/******************************************************************************
   Define which system use
 *****************************************************************************/
#define BM1383_I2C_NAME               ("bm1383")
#define BM1383_I2C_ADDRESS            (0x5D)
#define BM1383_INPUT_NAME             ("bm1383_inputdev")
#define BM1383_DRIVER_VER             ("1.0.0")
#define SN_TIME_UNIT                  (1000000000)
#define MIN_DELAY_TIME                (10 * 1000 * 1000)

#define PRESS                         (MSC_SERIAL)


/******************************************************************************
   Macro
 *****************************************************************************/


/******************************************************************************
   Use user Interface
 *****************************************************************************/
#define MEASURE_OFF                   (0)
#define MEASURE_ON                    (1)


/******************************************************************************
   Typedef struct
 *****************************************************************************/


/******************************************************************************
   define Address
 *****************************************************************************/
#define BM1383_RESET_CONTROL          (0x11)
#define BM1383_POWER_DOWN             (0x12)
#define BM1383_SLEEP                  (0x13)
#define BM1383_MODE_CONTROL           (0x14)
#define BM1383_PRESSURE_MSB           (0x1C)


/******************************************************************************
   define VAL
 *****************************************************************************/
#define BM1383_MODE_CONTROL_AVE_NUM0  (0 << 5)
#define BM1383_MODE_CONTROL_AVE_NUM2  (1 << 5)
#define BM1383_MODE_CONTROL_AVE_NUM4  (2 << 5)
#define BM1383_MODE_CONTROL_AVE_NUM8  (3 << 5)
#define BM1383_MODE_CONTROL_AVE_NUM16 (4 << 5)
#define BM1383_MODE_CONTROL_AVE_NUM32 (5 << 5)
#define BM1383_MODE_CONTROL_AVE_NUM64 (6 << 5)
#define BM1383_MODE_CONTROL_STNBY     (0)
#define BM1383_MODE_CONTROL_50MS      (2)
#define BM1383_MODE_CONTROL_100MS     (3)
#define BM1383_MODE_CONTROL_200MS     (4)

/******************
   User setting value
 ******************/
#define BM1383_MODE_CONTROL_ON        (BM1383_MODE_CONTROL_AVE_NUM32 | BM1383_MODE_CONTROL_50MS)
#define BM1383_MODE_CONTROL_OFF       (BM1383_MODE_CONTROL_STNBY)

#define BM1383_DEFAULT_DELAY_TIME     (50 * 1000 * 1000)  // ns

#define DUMMY_VALUE                   (-1)
#define COMPLETE_MEASURE_WAIT_TIME    (100 * 1000 * 1000)

#endif /* _ROHM_BM1383_I2C_H_ */

