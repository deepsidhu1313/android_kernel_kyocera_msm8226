#ifndef _RESET_LOG_H
#define _RESET_LOG_H
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <resetlog.h>
DESCRIPTION

EXTERNALIZED FUNCTIONS

This software is contributed or developed by KYOCERA Corporation.
(C) 2014 KYOCERA Corporation
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

#define HEADER_VERION           "v1.0.5"
#define HEADER_VERION_SIZE      (8)

#define MSM_KCJLOG_BASE         (MSM_UNINIT_RAM_BASE)

#define SIZE_SMEM_ALLOC         (512)
#define SIZE_CONTROL_INFO       (SIZE_SMEM_ALLOC)
#define SIZE_KERNEL_LOG         (512 * 1024)
#define SIZE_LOGCAT_MAIN        (512 * 1024)
#define SIZE_LOGCAT_SYSTEM      (512 * 1024)
#define SIZE_LOGCAT_EVENTS      (256 * 1024)
#define SIZE_LOGCAT_RADIO       (512 * 1024)

#define ADDR_CONTROL_INFO       (MSM_KCJLOG_BASE)
#define ADDR_KERNEL_LOG         (ADDR_CONTROL_INFO  + SIZE_CONTROL_INFO )
#define ADDR_LOGCAT_MAIN        (ADDR_KERNEL_LOG    + SIZE_KERNEL_LOG   )
#define ADDR_LOGCAT_SYSTEM      (ADDR_LOGCAT_MAIN   + SIZE_LOGCAT_MAIN  )
#define ADDR_LOGCAT_EVENTS      (ADDR_LOGCAT_SYSTEM + SIZE_LOGCAT_SYSTEM)
#define ADDR_LOGCAT_RADIO       (ADDR_LOGCAT_EVENTS + SIZE_LOGCAT_EVENTS)

#define CRASH_MAGIC_CODE        "KC ERROR"

#define CRASH_SYSTEM_KERNEL     "KERNEL"
#define CRASH_SYSTEM_MODEM      "MODEM"
#define CRASH_SYSTEM_RIVA       "RIVA"
#define CRASH_SYSTEM_LPASS      "LPASS"
#define CRASH_SYSTEM_ANDROID    "ANDROID"
#define CRASH_SYSTEM_UNKNOWN    "UNKNOWN"

#define CRASH_KIND_PANIC        "KERNEL PANIC"
#define CRASH_KIND_FATAL        "ERR FATAL"
#define CRASH_KIND_EXEPTION     "EXEPTION"
#define CRASH_KIND_WDOG_HW      "HW WATCH DOG"
#define CRASH_KIND_WDOG_SW      "SW WATCH DOG"
#define CRASH_KIND_SYS_SERVER   "SYSTEM SERVER CRASH"
#define CRASH_KIND_UNKNOWN      "UNKNOWN"

/* RAM_CONSOLE Contol */
typedef struct {
    unsigned char               magic[4];       // RAM_CONSOLE MAGIC("DEBG")
    unsigned long               start;          // RAM_CONSOLE start
    unsigned long               size;           // RAM_CONSOLE size
} ram_console_header_type;

typedef struct {
    ram_console_header_type     header;         // RAM_CONSOLE header
    unsigned char               msg[1];         // RAM_CONSOLE message
} ram_console_type;

enum {
	LOGGER_INFO_MAIN,
	LOGGER_INFO_SYSTEM,
	LOGGER_INFO_EVENTS,
	LOGGER_INFO_RADIO,
	LOGGER_INFO_MAX,
};

/* Log Control */
struct logger_log_info {
	unsigned long               w_off;
	unsigned long               head;
};

#define MAGIC_CODE_SIZE         (16)
#define CRASH_SYSTEM_SIZE       (16)
#define CRASH_KIND_SIZE         (32)
#define CRASH_TIME_SIZE         (48)
#define LINUX_BANNER_SIZE       (160)
#define PRODUCT_NUMBER_SIZE     (16)
#define VERSION_SIZE            (16)
#define BUILD_DATE_SIZE         (32)
#define SETTING_OPTION_SIZE     (44)
#define RESERVED_SIZE           (12)

typedef struct {
	unsigned char               magic_code[MAGIC_CODE_SIZE];            // Magic Code(16byte)
	unsigned char               crash_system[CRASH_SYSTEM_SIZE];        // Crash System(16byte)
	unsigned char               crash_kind[CRASH_KIND_SIZE];            // Crash Kind(32byte)
	unsigned char               crash_time[CRASH_TIME_SIZE];            // Crash Time(48byte)
	unsigned char               linux_banner[LINUX_BANNER_SIZE];        // Linux banner(160byte)
	unsigned char               product_number[PRODUCT_NUMBER_SIZE];
	unsigned char               software_version[VERSION_SIZE];
	unsigned char               baseband_version[VERSION_SIZE];
	unsigned char               android_build_date[BUILD_DATE_SIZE];
	unsigned char               modem_build_date[BUILD_DATE_SIZE];
	struct logger_log_info      info[LOGGER_INFO_MAX];
	unsigned long               reserved[RESERVED_SIZE];
} ram_log_info_type;
#endif /* _RESET_LOG_H */
