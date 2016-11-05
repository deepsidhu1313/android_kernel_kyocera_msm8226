/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <kcjlog.c>
DESCRIPTION

EXTERNALIZED FUNCTIONS

This software is contributed or developed by KYOCERA Corporation.
(C) 2015 KYOCERA Corporation

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 and
only version 2 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
02110-1301, USA.

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * INCLUDE FILES
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

#include <linux/module.h>
#include <asm/io.h>
#include <mach/msm_iomap.h>
#include "resetlog.h"

static int sw_version_changed_handler( const char *kmessage, struct kernel_param *kp );
static int build_date_changed_handler( const char *kmessage, struct kernel_param *kp );

static struct kparam_string sw_version = {
	.string = ((ram_log_info_type *)ADDR_CONTROL_INFO)->software_version,
	.maxlen = VERSION_SIZE,
};

static struct kparam_string build_date = {
	.string = ((ram_log_info_type *)ADDR_CONTROL_INFO)->android_build_date,
	.maxlen = BUILD_DATE_SIZE,
};

module_param_call(sw_version, sw_version_changed_handler, param_get_string, &sw_version,
                  S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);

module_param_call(build_date, build_date_changed_handler, param_get_string, &build_date,
                  S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);


static int sw_version_changed_handler( const char *kmessage,
                                       struct kernel_param *kp )
{
	return param_set_copystring( kmessage, kp );
}

static int build_date_changed_handler( const char *kmessage,
                                       struct kernel_param *kp )
{
	return param_set_copystring( kmessage, kp );
}

