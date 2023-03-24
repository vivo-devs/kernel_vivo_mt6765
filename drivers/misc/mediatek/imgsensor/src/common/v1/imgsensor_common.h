/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#ifndef __IMGSENSOR_COMMON_H__
#define __IMGSENSOR_COMMON_H__
#include "kd_camera_feature.h"
#include "kd_imgsensor_define.h"

/************************************************************************
 * Debug configuration
 ************************************************************************/
#define PREFIX "[imgsensor]"
#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG(fmt, arg...)  pr_info(PREFIX fmt, ##arg)
#define PK_PR_ERR(fmt, arg...)  pr_err(fmt, ##arg)
#define PK_INFO(fmt, arg...) pr_info(PREFIX fmt, ##arg)
#else
#define PK_DBG(fmt, arg...)
#define PK_PR_ERR(fmt, arg...)  pr_err(fmt, ##arg)
#define PK_INFO(fmt, arg...) pr_debug(PREFIX fmt, ##arg)
#endif

#define PLATFORM_POWER_SEQ_NAME "platform_power_seq"
#define DEBUG_CAMERA_HW_K

#define IMGSENSOR_LEGACY_COMPAT
#if defined(CONFIG_MTK_CAM_PD1987)
#define MIPI_SWITCH
#endif
#define IMGSENSOR_TOSTRING(value)           #value
#define IMGSENSOR_STRINGIZE(stringizedName) IMGSENSOR_TOSTRING(stringizedName)

enum IMGSENSOR_ARCH {
	IMGSENSOR_ARCH_V1 = 0,
	IMGSENSOR_ARCH_V2,
	IMGSENSOR_ARCH_V3
};

enum IMGSENSOR_RETURN {
	IMGSENSOR_RETURN_SUCCESS = 0,
	IMGSENSOR_RETURN_ERROR   = -1,
//IIC is grounded, the camera can be opend ,check iic timeout
	IMGSENSOR_I2C_TIMEOUT = -2,
//IIC is grounded, the camera can be opend ,check iic timeout
};

#define LENGTH_FOR_SNPRINTF 256
#endif

