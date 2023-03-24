// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "imgsensor_sensor.h"

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/string.h>

#include "kd_camera_typedef.h"
#include "kd_camera_feature.h"


#include "imgsensor_hw.h"

#if defined(CONFIG_MTK_CAM_PD2140F_EX) ||defined(CONFIG_MTK_CAM_PD2140IF_EX)|| defined(CONFIG_MTK_CAM_PD2140)
extern char *get_board_version(void);
extern char debug_info[8];

enum BOARD_VERSION_BIT{
	BOARD_VERSION_BIT_START				= 0,
	BOARD_VERSION_BIT_RESERVED0			= BOARD_VERSION_BIT_START,
	BOARD_VERSION_BIT_RESERVED1			= 1,
	BOARD_VERSION_BIT_LOWORHIGH_CONFIGURATION	= 2,
	BOARD_VERSION_BIT_HARDWARE_VERSION		= 3,

	BOARD_VERSION_BIT_REGION_START			= 4,
	BOARD_VERSION_BIT_REGION_VERSION_0		= BOARD_VERSION_BIT_REGION_START,
	BOARD_VERSION_BIT_REGION_VERSION_1		= 5,
	BOARD_VERSION_BIT_REGION_VERSION_2		= 6,
	BOARD_VERSION_BIT_REGION_VERSION_3		= 7,
	BOARD_VERSION_BIT_REGION_END			= BOARD_VERSION_BIT_REGION_VERSION_3,

	BOARD_VERSION_BIT_MAX				= 8,
};

enum HARDWARE_REGION_VERSION {				// GPIO172 GPIO53 GPIO52 GPIO42
	HARDWARE_REGION_VERSION_LATINAMERICA 		= 0B0000,
	HARDWARE_REGION_VERSION_INDIA			= 0B1000,
	HARDWARE_REGION_VERSION_INDONESIA		= 0B0100,
	HARDWARE_REGION_VERSION_MALAYSIA		= 0B1100,
	HARDWARE_REGION_VERSION_PHILIPPINE		= 0B0010,
	HARDWARE_REGION_VERSION_FULLFREQUENCY		= 0B1010,
	HARDWARE_REGION_VERSION_EUROPE			= 0B0110,
	HARDWARE_REGION_VERSION_CIS			= 0B1110,
};
#endif

char *imgsensor_sensor_idx_name[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {
	IMGSENSOR_SENSOR_IDX_NAME_MAIN,
	IMGSENSOR_SENSOR_IDX_NAME_SUB,
	IMGSENSOR_SENSOR_IDX_NAME_MAIN2,
	IMGSENSOR_SENSOR_IDX_NAME_SUB2,
	IMGSENSOR_SENSOR_IDX_NAME_MAIN3,
};
extern int hConflag;
enum IMGSENSOR_RETURN imgsensor_hw_release_all(struct IMGSENSOR_HW *phw)
{
	int i;

	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (phw->pdev[i]->release != NULL)
			(phw->pdev[i]->release)(phw->pdev[i]->pinstance);
	}
	return IMGSENSOR_RETURN_SUCCESS;
}
enum IMGSENSOR_RETURN imgsensor_hw_init(struct IMGSENSOR_HW *phw)
{
	struct IMGSENSOR_HW_SENSOR_POWER      *psensor_pwr;
	struct IMGSENSOR_HW_CFG               *pcust_pwr_cfg;
	struct IMGSENSOR_HW_CUSTOM_POWER_INFO *ppwr_info;
	int i, j;
	char str_prop_name[LENGTH_FOR_SNPRINTF];
	struct device_node *of_node
		= of_find_compatible_node(NULL, NULL, "mediatek,camera_hw");

	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (hw_open[i] != NULL)
			(hw_open[i])(&phw->pdev[i]);

		if (phw->pdev[i]->init != NULL)
			(phw->pdev[i]->init)(phw->pdev[i]->pinstance);
	}

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		psensor_pwr = &phw->sensor_pwr[i];

		pcust_pwr_cfg = imgsensor_custom_config;
		while (pcust_pwr_cfg->sensor_idx != i &&
		       pcust_pwr_cfg->sensor_idx != IMGSENSOR_SENSOR_IDX_NONE)
			pcust_pwr_cfg++;

		if (pcust_pwr_cfg->sensor_idx == IMGSENSOR_SENSOR_IDX_NONE)
			continue;

		ppwr_info = pcust_pwr_cfg->pwr_info;
		while (ppwr_info->pin != IMGSENSOR_HW_PIN_NONE) {
			for (j = 0; j < IMGSENSOR_HW_ID_MAX_NUM; j++)
				if (ppwr_info->id == phw->pdev[j]->id)
					break;

			psensor_pwr->id[ppwr_info->pin] = j;
			ppwr_info++;
		}
	}

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		memset(str_prop_name, 0, sizeof(str_prop_name));
		snprintf(str_prop_name,
					sizeof(str_prop_name),
					"cam%d_%s",
					i,
					"enable_sensor");
		if (of_property_read_string(
			of_node,
			str_prop_name,
			&phw->enable_sensor_by_index[i]) < 0) {
			pr_info("Property cust-sensor not defined\n");
			phw->enable_sensor_by_index[i] = NULL;
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN imgsensor_hw_power_sequence(
	struct IMGSENSOR_HW             *phw,
	enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
	enum   IMGSENSOR_HW_POWER_STATUS pwr_status,
	struct IMGSENSOR_HW_POWER_SEQ   *ppower_sequence,
	char *pcurr_idx)
{
	struct IMGSENSOR_HW_SENSOR_POWER *psensor_pwr =
	    &phw->sensor_pwr[sensor_idx];

	struct IMGSENSOR_HW_POWER_SEQ    *ppwr_seq = ppower_sequence;
	struct IMGSENSOR_HW_POWER_INFO   *ppwr_info;
	struct IMGSENSOR_HW_DEVICE       *pdev;
	int                               pin_cnt = 0;

	while (ppwr_seq < ppower_sequence + IMGSENSOR_HW_SENSOR_MAX_NUM &&
		ppwr_seq->name != NULL) {
		if (!strcmp(ppwr_seq->name, PLATFORM_POWER_SEQ_NAME)) {
			if (sensor_idx == ppwr_seq->_idx)
				break;
		} else {
			if (!strcmp(ppwr_seq->name, pcurr_idx))
				break;
		}
		ppwr_seq++;
	}

	if (ppwr_seq->name == NULL)
		return IMGSENSOR_RETURN_ERROR;

	ppwr_info = ppwr_seq->pwr_info;

	while (ppwr_info->pin != IMGSENSOR_HW_PIN_NONE &&
		ppwr_info < ppwr_seq->pwr_info + IMGSENSOR_HW_POWER_INFO_MAX) {

		if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON &&
		   ppwr_info->pin != IMGSENSOR_HW_PIN_UNDEF) {
			pdev = phw->pdev[psensor_pwr->id[ppwr_info->pin]];
		pr_info(
		  "sensor_idx = %d, pin=%d, pin_state_on=%d, hw_id =%d\n",
		 sensor_idx,
		 ppwr_info->pin,
		 ppwr_info->pin_state_on,
		 psensor_pwr->id[ppwr_info->pin]);


			if (pdev->set != NULL)
				pdev->set(
				    pdev->pinstance,
				    sensor_idx,
				    ppwr_info->pin,
				    ppwr_info->pin_state_on);

			mdelay(ppwr_info->pin_on_delay);
		}

		ppwr_info++;
		pin_cnt++;
	
		}
		
		if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
			if (phw->pdev[1]->set != NULL){
	
		if((sensor_idx == 3)&&(hConflag == 1)){
#if 1
			phw->pdev[1]->set(
			phw->pdev[1]->pinstance,
			2,
			IMGSENSOR_HW_PIN_PDN,
			IMGSENSOR_HW_PIN_STATE_LEVEL_0);
#endif
		}
		else if((sensor_idx == 2)&&(hConflag == 1)){
#if 1
			phw->pdev[1]->set(
			phw->pdev[1]->pinstance,
			3,
			IMGSENSOR_HW_PIN_PDN,
			IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH);
#endif	
		}
		}
	}

	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_OFF) {
		while (pin_cnt) {
			ppwr_info--;
			pin_cnt--;

			if (ppwr_info->pin != IMGSENSOR_HW_PIN_UNDEF) {
				pdev =
				    phw->pdev[psensor_pwr->id[ppwr_info->pin]];
				mdelay(ppwr_info->pin_on_delay);

				if (pdev->set != NULL)
					pdev->set(
					    pdev->pinstance,
					    sensor_idx,
					    ppwr_info->pin,
					    ppwr_info->pin_state_off);
			}
		}
        if (phw->pdev[1]->set != NULL){
		if((sensor_idx == 3)&&(hConflag == 0)){
#if 1
			phw->pdev[1]->set(
			phw->pdev[1]->pinstance,
			2,
			IMGSENSOR_HW_PIN_PDN,
			IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH);
#endif
		}

	}
	}

	/* wait for power stable */
	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON)
		mdelay(5);
	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_hw_power(
	struct IMGSENSOR_HW     *phw,
	struct IMGSENSOR_SENSOR *psensor,
	char *curr_sensor_name,
	enum IMGSENSOR_HW_POWER_STATUS pwr_status)
{
	enum IMGSENSOR_SENSOR_IDX sensor_idx = psensor->inst.sensor_idx;
	char str_index[LENGTH_FOR_SNPRINTF];
	int ret = 0;

#if defined(CONFIG_MTK_CAM_PD2140F_EX) ||defined(CONFIG_MTK_CAM_PD2140IF_EX)|| defined(CONFIG_MTK_CAM_PD2140)
	char *board_version			= NULL;
	unsigned char u8_board_version		= 0;
	unsigned char u8_region_version		= 0;
	unsigned char is_hardware_v2		= 0;
	unsigned char is_highlevel_hw_ver	= 0;
	unsigned char is_rearcamera_8M		= 0;
	int  i;
#endif

	pr_info(
		"wayoung debug V1 sensor_idx %d, power %d curr_sensor_name %s, enable list %s\n",
		sensor_idx,
		pwr_status,
		curr_sensor_name,
		phw->enable_sensor_by_index[(uint32_t)sensor_idx] == NULL
		? "NULL"
		: phw->enable_sensor_by_index[(uint32_t)sensor_idx]);
       //add by vivo changcheng 2021.6.4 start
       if(curr_sensor_name == NULL) {
		pr_err("curr_sensor_name = NULL");
		return IMGSENSOR_RETURN_ERROR;
       }
       //add by vivo changcheng 2021.6.4 end
	if (phw->enable_sensor_by_index[(uint32_t)sensor_idx] &&
	!strstr(phw->enable_sensor_by_index[(uint32_t)sensor_idx], curr_sensor_name))
		return IMGSENSOR_RETURN_ERROR;


	ret = snprintf(str_index, sizeof(str_index), "%d", sensor_idx);
	if (ret == 0) {
		pr_info("Error! snprintf allocate 0");
		ret = IMGSENSOR_RETURN_ERROR;
		return ret;
	}
	imgsensor_hw_power_sequence(
	    phw,
	    sensor_idx,
	    pwr_status,
	    platform_power_sequence,
	    str_index);

#if defined(CONFIG_MTK_CAM_PD2140F_EX) ||defined(CONFIG_MTK_CAM_PD2140IF_EX)|| defined(CONFIG_MTK_CAM_PD2140)
	board_version = get_board_version();
	for (i = BOARD_VERSION_BIT_START; i < BOARD_VERSION_BIT_MAX; i++) {
		u8_board_version <<= 1;
		u8_board_version |= (board_version[i] == '1');
	}

	is_highlevel_hw_ver	= (u8_board_version >> BOARD_VERSION_BIT_LOWORHIGH_CONFIGURATION)	& 0x01;
	is_hardware_v2		= (u8_board_version >> BOARD_VERSION_BIT_HARDWARE_VERSION)		& 0x01;
	u8_region_version	= (u8_board_version >> BOARD_VERSION_BIT_REGION_START)			& 0x0F;

	if (is_highlevel_hw_ver) {
		is_rearcamera_8M = false;
		pr_info("set is_rearcamera_8M [false]. is_highlevel_hw_ver[true]");
	} else {
		if (u8_region_version == HARDWARE_REGION_VERSION_LATINAMERICA	||
			u8_region_version == HARDWARE_REGION_VERSION_INDIA	||
			u8_region_version == HARDWARE_REGION_VERSION_EUROPE) {
			is_rearcamera_8M = false;
			pr_info("set is_rearcamera_8M [false]. u8_region_version[%x]", u8_region_version);
		} else {
			is_rearcamera_8M = true;
			pr_info("set is_rearcamera_8M [true]. is_highlevel_hw_ver[false]");
		}
	}

	if (debug_info[0] == '1') {								//debug enable
		if (debug_info[1] == '0') {
			is_rearcamera_8M = false;
		} else {
			is_rearcamera_8M = true;
		}

		if (debug_info[2] == '0') {
			is_hardware_v2 = false;
		} else {
			is_hardware_v2 = true;
		}

		pr_info("enable debug info, and set is_rearcamera_8M: %d, is_hardware_v2: %d", is_rearcamera_8M, is_hardware_v2);
	} else {
		pr_info("disable debug info");
	}

	pr_info("Get board version: %s[0x%02x], is_hardware_v2[%d], is_highlevel_hw_ver[%d], u8_region_version[0x%02x], is_rearcamera_8M[%d]",
			board_version,
			u8_board_version,
			is_hardware_v2,
			is_highlevel_hw_ver,
			u8_region_version,
			is_rearcamera_8M);

	if (is_hardware_v2) {
		if (is_rearcamera_8M) {
			imgsensor_hw_power_sequence(
				phw,
				sensor_idx,
				pwr_status,
				sensor_power_sequence_1_2V,
			curr_sensor_name);
		} else {
			imgsensor_hw_power_sequence(
				phw,
				sensor_idx,
				pwr_status,
				sensor_power_sequence_1_1V,
				curr_sensor_name);
		}
	} else {
		imgsensor_hw_power_sequence(
			phw,
			sensor_idx,
			pwr_status,
			sensor_power_sequence,
			curr_sensor_name);
	}
#else
	imgsensor_hw_power_sequence(
	    phw,
	    sensor_idx,
	    pwr_status,
	    sensor_power_sequence,
	    curr_sensor_name);
#endif

	return IMGSENSOR_RETURN_SUCCESS;
}

