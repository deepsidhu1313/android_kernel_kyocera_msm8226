/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */

#ifndef KC_BOARD_H
#define KC_BOARD_H

/* ----------------------------------------------------*/
/* HW ID                                               */
/* ----------------------------------------------------*/
typedef enum {
    OEM_BOARD_WS0_TYPE     = 0,
    OEM_BOARD_WS1_TYPE     = 1,
    OEM_BOARD_WS1_1_TYPE   = OEM_BOARD_WS1_TYPE,
    OEM_BOARD_WS1_2_TYPE   = 2,
    OEM_BOARD_WS1_3_TYPE   = OEM_BOARD_WS1_2_TYPE,
    OEM_BOARD_WS2_TYPE     = 3,
    OEM_BOARD_WS2_1_TYPE   = OEM_BOARD_WS2_TYPE,
    OEM_BOARD_WS2_2_TYPE   = 4,
    OEM_BOARD_WS2_3_TYPE   = 5,
    OEM_BOARD_LAB_TYPE     = 6,
    OEM_BOARD_PP_TYPE      = 7,
    OEM_BOARD_FAIL_TYPE    = 99,
} oem_board_type;

/* ----------------------------------------------------*/
/* VENDOR CEL                                          */
/* ----------------------------------------------------*/
typedef enum {
    OEM_VENDOR_SP_TYPE     = 0,
    OEM_VENDOR_BT_TYPE     = 1,
    OEM_VENDOR_FAIL_TYPE   = 99,
} oem_vendor_type;

extern void OEM_board_judgement(void);
extern oem_board_type OEM_get_board(void);

extern void OEM_vendor_judgement(void);
extern oem_vendor_type OEM_get_vendor(void);
#endif /* KC_BOARD_H */
