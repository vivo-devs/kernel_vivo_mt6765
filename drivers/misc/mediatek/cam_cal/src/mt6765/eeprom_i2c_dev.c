// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#include "eeprom_i2c_dev.h"

#if defined(CONFIG_MTK_CAM_PD2140F_EX) ||defined(CONFIG_MTK_CAM_PD2140IF_EX)|| defined(CONFIG_MTK_CAM_PD2140)
static enum EEPROM_I2C_DEV_IDX gi2c_dev_sel[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {
	I2C_DEV_IDX_1, /* main */
	I2C_DEV_IDX_2, /* sub */
	I2C_DEV_IDX_2, /* main2 */ //PD2140 sub and main2 using the same I2C_DEV
	I2C_DEV_IDX_3, /* sub2 */
	I2C_DEV_IDX_3, /* main3 */
};
#else
static enum EEPROM_I2C_DEV_IDX gi2c_dev_sel[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {
	I2C_DEV_IDX_1, /* main */
	I2C_DEV_IDX_2, /* sub */
	I2C_DEV_IDX_3, /* main2 */
	I2C_DEV_IDX_3, /* sub2 */
	I2C_DEV_IDX_3, /* main3 */
};
#endif

enum EEPROM_I2C_DEV_IDX get_i2c_dev_sel(enum IMGSENSOR_SENSOR_IDX idx)
{
	if (idx >= IMGSENSOR_SENSOR_IDX_MIN_NUM &&
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM)
		return gi2c_dev_sel[idx];
	return I2C_DEV_IDX_1;
}

int gi2c_dev_timing[I2C_DEV_IDX_MAX] = {
	100, /* dev1, 100k */
	100, /* dev2, 100k */
	100, /* dev3, 100k */
};

