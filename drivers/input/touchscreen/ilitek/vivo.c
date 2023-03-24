/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ilitek.h"

extern int mdss_dsi_panel_reset_and_powerctl(int enable);

enum {
	GET_FW_VER = 0,
	GET_FW_CONFIG_VER,
};

enum {
	CHARGER_OFF = 0,
	CHARGER_ON,
};

enum {
	EDGE_PALM_LEFT_UP_RIGHT_DOWN = 0,
	EDGE_PALM_STRAIGHT,
	EDGE_PALM_LEFT_DOWN_RIGHT_UP,
};

enum {
	ILI_RAWDATA = 0,
	ILI_DIFFDATA,
	ILI_BASEDATA,
};

int bbk_ili_get_rawordiff_data(int which, int *data)
{
	int i, row, col;
	int ret = 0, count = 0, timeout = 50;
	int frame_count = 0;
	u8 cmd[2] = {0xFA, 0x0};
	u8 tp_mode = 0;
	s16 temp;
	unsigned char *ptr;

	switch (which) {
	case ILI_RAWDATA:
		ipio_info("get raw data");
		cmd[1] = P5_X_FW_RAW_DATA_MODE;
		frame_count = 1;
		break;
	case ILI_DIFFDATA:
		ipio_info("get delta data\n");
		cmd[1] = P5_X_FW_DELTA_DATA_MODE;
		frame_count = 1;
		break;
	case ILI_BASEDATA:
		ipio_info("get baseline data\n");
		cmd[1] = P5_X_FW_RAW_DATA_MODE;
		frame_count = 1;
		break;
	default:
		ipio_err("Unknown type, %d\n", which);
		return -1;
	}

	tp_mode = P5_X_FW_DEBUG_MODE;
	ret = ilitek_tddi_switch_mode(&tp_mode);
	if (ret < 0)
		goto out;

	mdelay(50);
    idev->debug_node_open = DISABLE;

	row = idev->ych_num;
	col = idev->xch_num;

	ipio_info("row = %d, col = %d\n", row, col);
	ipio_info("cmd = 0x%x, 0x%x\n", cmd[0], cmd[1]);

	mutex_lock(&idev->touch_mutex);
	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ret = idev->write(cmd, sizeof(cmd));
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
	mutex_unlock(&idev->touch_mutex);

	idev->debug_data_frame = 0;
	idev->debug_node_open = ENABLE;

	while ((count < frame_count) && (timeout > 0)) {
		ipio_info("count = %d, over = %d, frame = %d\n",
				count, count % 1024, idev->debug_data_frame);

		if ((count % 1024) < idev->debug_data_frame) {
			ilitek_plat_irq_disable();
			mutex_lock(&idev->debug_mutex);
			ptr = &idev->debug_buf[count % 1024][35];
			for (i = 0; i < row * col; i++, ptr += 2) {
				temp = (*ptr << 8) + *(ptr + 1);
				if (which != P5_X_FW_RAW_DATA_MODE)
					temp -= 0x10000;

				*((data + (row*(i % col))) + (i /col)) = temp;
			}
			mutex_unlock(&idev->debug_mutex);
			ilitek_plat_irq_enable();
			count++;
			timeout = 50;
		}

		if (count % 1024 == 0 && idev->debug_data_frame == 1024)
			idev->debug_data_frame = 0;

		/* get one frame data take around 130ms */
		mdelay(100);

		timeout-- ;
		if (timeout == 0)
			ipio_err("debug mode get data timeout!\n");
	}

out:
	idev->debug_node_open = DISABLE;
	tp_mode = P5_X_FW_DEMO_MODE;
	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	mutex_lock(&idev->touch_mutex);
	ilitek_tddi_switch_mode(&tp_mode);
	mutex_unlock(&idev->touch_mutex);	
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
	return ret;
}

int bbk_ili_fw_update(const struct firmware *fw)
{
	int ret = 0;

	if (!fw) {
		ipio_err("firmware is null\n");
		return -ENOMEM;
	}

	if (atomic_read(&idev->fw_stat)) {
		ipio_err("fw update is still running\n");
		return -1;
	}

	idev->vivo_fw_size = fw->size;
	ipio_info("##### fw size = %d\n", idev->vivo_fw_size);
	memcpy(idev->vivo_fw_data, fw->data, idev->vivo_fw_size);
	ret = ilitek_tddi_fw_upgrade_handler(NULL);
	return ret;
}

int iliIdleEnableOrDisable(int state)
{
	int ret = 0;
	u8 cmd[2] = {0};

	ipio_info("idle %s\n", (state == ENABLE) ? "ON" : "OFF");
	mutex_lock(&idev->touch_mutex);
	cmd[0] = 0x14;
	cmd[1] = 0xFF & state;
	ipio_info("idle cmd = 0x%x, 0x%x\n", cmd[0], cmd[1]);
	ret = idev->write(cmd, sizeof(cmd));
	mutex_unlock(&idev->touch_mutex);
	return ret;
}

int bbk_ili_readUdd(unsigned char *udd)
{
	return 0;
}

int bbk_ili_writeUdd(unsigned char *udd)
{
	return 0;
}

int bbk_ili_mode_change(int which)
{
	int ret = 0;
	static int mode;

	switch (which) {
	case TOUCHSCREEN_SLEEP:
		VTI("deep sleep mode");
		if (mode == TOUCHSCREEN_NORMAL) {
			idev->gesture = DISABLE;
			ret = ilitek_tddi_sleep_handler(TP_DEEP_SLEEP);
		} else {
			idev->gesture = DISABLE;
			idev->tp_suspend = true;
			ilitek_tddi_ic_func_ctrl("sleep", DEEP_SLEEP_IN);
			idev->tp_suspend = false;
		}
		ilitek_plat_irq_disable();
		mdelay(35);
		mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		mdss_dsi_panel_reset_and_powerctl(0);
		mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		mdelay(10);
		mode = TOUCHSCREEN_SLEEP;
		//idev->tp_suspend = true;
		break;
	case TOUCHSCREEN_NORMAL:
		//idev->tp_suspend = false;
		VTI("resume to normal mode");
		ret = ilitek_tddi_sleep_handler(TP_RESUME);
		mode = TOUCHSCREEN_NORMAL;
		break;
	case TOUCHSCREEN_GESTURE:
		VTI("suspend with gesture mode");
		if (mode ==  TOUCHSCREEN_GESTURE) {
			VTI("Already in gesture mode break");
			break;
		}
		if (mode == TOUCHSCREEN_NORMAL) {
			idev->gesture = ENABLE;
			ret = ilitek_tddi_sleep_handler(TP_SUSPEND);
		} else {
			mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
			mdss_dsi_panel_reset_and_powerctl(1);
			mdelay(50);
			mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
			idev->gesture = ENABLE;
			ilitek_tddi_gesture_recovery();
			enable_irq_wake(idev->irq_num);
			ilitek_plat_irq_enable();
		}
		mode = TOUCHSCREEN_GESTURE;
		//idev->tp_suspend = true;
		break;
	}
	return ret;
}

int bbk_ili_get_fw_version(int which)
{
	int ret = 0;

	ipio_info("which = %d\n", which);

	switch (which) {
	case GET_FW_CONFIG_VER:
		ipio_info("no config version\n");
		ret = 0;
		break;
	case GET_FW_VER:
		ipio_info("get fw version\n");
		mutex_lock(&idev->touch_mutex);
		ilitek_tddi_ic_get_fw_ver();
		mutex_unlock(&idev->touch_mutex);
		ret = idev->chip->fw_ver & 0xffff;
		break;
	default:
		ipio_err("Unknown type, %d\n", which);
		ret = -1;
		break;
	}
	ipio_info("firmware version = %x\n", ret);
	return ret;
}

int bbk_ili_get_ic_mode(void)
{
	int res = 0;
	uint8_t cmd[2] = { 0 };

	mutex_lock(&idev->touch_mutex);
	cmd[0] = 0x4b;
	cmd[1] = 0x0;
	idev->write(cmd, 1);

	mdelay(10);

	res = idev->read(cmd, 2);
	mutex_unlock(&idev->touch_mutex);
	if (res < 0) {
		ipio_err("Failed to read ic mode \n");
		return -1;
	}
	if (cmd[1]) {
		ipio_info("gesutre mode = 1\n");
		return 1;
	} else {
		ipio_info("normal mode = 0\n");
		return 0;
	}
}

struct point_xy {
	u16 x;
	u16 y;
};

struct gesture_buf {
	struct point_xy Point_start;
	struct point_xy Point_end;
	struct point_xy Point_1st;
	struct point_xy Point_2st;
	struct point_xy Point_3st;
	struct point_xy Point_4st;
};

int bbk_ili_gesture_point_get(u16 *data)
{
	int ret = 0;
	u16 xx[6];
	u16 yy[6];
	uint8_t gesture_id = 0;
	uint8_t score = 0;
	int nlength = 0;
	u16 x1, y1, x2, y2;
	struct gesture_buf gesture;

	if (!data) {
		ipio_err("data is null\n");
		return -1;
	}

	if (idev->debug_buf == NULL) {
		VTI("idev->debug_buf NULL");
		return -1;
	}

	ilitek_dump_data(idev->debug_buf, 8, P5_X_GESTURE_INFO_LENGTH + 1, 0, "VIVO_TS gesture before");

	gesture_id = idev->debug_buf[0][1];
	score = idev->debug_buf[0][36];

	x1 = (((idev->debug_buf[0][4] & 0xF0) << 4) | (idev->debug_buf[0][5]));
	y1 = (((idev->debug_buf[0][4] & 0x0F) << 8) | (idev->debug_buf[0][6]));
	x2 = (((idev->debug_buf[0][7] & 0xF0) << 4) | (idev->debug_buf[0][8]));
	y2 = (((idev->debug_buf[0][7] & 0x0F) << 8) | (idev->debug_buf[0][9]));

	xx[0] = (((idev->debug_buf[0][28] & 0xF0) << 4) | (idev->debug_buf[0][29]));
	yy[0] = (((idev->debug_buf[0][28] & 0x0F) << 8) | (idev->debug_buf[0][30]));
	xx[1] = (((idev->debug_buf[0][31] & 0xF0) << 4) | (idev->debug_buf[0][32]));
	yy[1] = (((idev->debug_buf[0][31] & 0x0F) << 8) | (idev->debug_buf[0][33]));

	xx[2] = xx[0] + ((xx[1] - xx[0])/2);
	yy[2] = yy[0];
	xx[3] = xx[0];
	yy[3] = yy[0] + ((yy[1] - yy[0])/2);
	xx[4] = xx[0] + ((xx[1] - xx[0])/2);
	yy[4] = yy[1];
	xx[5] = xx[1];
	yy[5] = yy[0] + ((yy[1] - yy[0])/2);

	gesture.Point_start.x = x1*idev->panel_wid/TPD_WIDTH;
	gesture.Point_start.y = y1*idev->panel_hei/TPD_HEIGHT;
	gesture.Point_end.x  = x2*idev->panel_wid/TPD_WIDTH;
	gesture.Point_end.y  = y2*idev->panel_hei/TPD_HEIGHT;
	gesture.Point_1st.x   = xx[2]*idev->panel_wid/TPD_WIDTH;
	gesture.Point_1st.y   = yy[2]*idev->panel_hei/TPD_HEIGHT;
	gesture.Point_2st.x   = xx[3]*idev->panel_wid/TPD_WIDTH;
	gesture.Point_2st.y   = yy[3]*idev->panel_hei/TPD_HEIGHT;
	gesture.Point_3st.x   = xx[4]*idev->panel_wid/TPD_WIDTH;
	gesture.Point_3st.y   = yy[4]*idev->panel_hei/TPD_HEIGHT;
	gesture.Point_4st.x   = xx[5]*idev->panel_wid/TPD_WIDTH;
	gesture.Point_4st.y   = yy[5]*idev->panel_hei/TPD_HEIGHT;

/*
	gesture.Point_4st.x   = (((point_data[25] & 0xF0) << 4) | (point_data[26]));
	gesture.Point_4st.y   = (((point_data[25] & 0x0F) << 8) | (point_data[27]));
*/
     /*judge gesture type  */
	switch (gesture_id) {
	case GESTURE_UP:
		*data       = gesture.Point_start.x;
		*(data + 1) = gesture.Point_start.y;
		*(data + 2) = gesture.Point_end.x;
		*(data + 3) = gesture.Point_end.y;
		nlength = 2;
		break;
	case GESTURE_W:
		gesture.Point_1st.x = (((idev->debug_buf[0][16] & 0xF0) << 4) | (idev->debug_buf[0][17]))*idev->panel_wid/TPD_WIDTH;
		gesture.Point_1st.y = (((idev->debug_buf[0][16] & 0x0F) << 8) | (idev->debug_buf[0][18]))*idev->panel_hei/TPD_HEIGHT;
		gesture.Point_2st.x = (((idev->debug_buf[0][19] & 0xF0) << 4) | (idev->debug_buf[0][20]))*idev->panel_wid/TPD_WIDTH;
		gesture.Point_2st.y = (((idev->debug_buf[0][19] & 0x0F) << 8) | (idev->debug_buf[0][21]))*idev->panel_hei/TPD_HEIGHT;
		gesture.Point_3st.x = (((idev->debug_buf[0][22] & 0xF0) << 4) | (idev->debug_buf[0][23]))*idev->panel_wid/TPD_WIDTH;
		gesture.Point_3st.y = (((idev->debug_buf[0][22] & 0x0F) << 8) | (idev->debug_buf[0][24]))*idev->panel_hei/TPD_HEIGHT;
		*data     = gesture.Point_start.x;
		*(data + 1)  = gesture.Point_start.y;
		*(data + 2)  = gesture.Point_1st.x;
		*(data + 3)  = gesture.Point_1st.y;
		*(data + 4)  = gesture.Point_2st.x;
		*(data + 5)  = gesture.Point_2st.y;
		*(data + 6)  = gesture.Point_3st.x;
		*(data + 7)  = gesture.Point_3st.y;
		*(data + 8)  = gesture.Point_end.x;
		*(data + 9)  = gesture.Point_end.y;
		nlength = 5;
		break;
	case GESTURE_E:
		*data        = gesture.Point_start.x;
		*(data + 1)  = gesture.Point_start.y;
		*(data + 2)  = gesture.Point_end.x;
		*(data + 3)  = gesture.Point_end.y;
		*(data + 4)  = gesture.Point_1st.x;
		*(data + 5)  = gesture.Point_1st.y;
		*(data + 6)  = gesture.Point_2st.x;
		*(data + 7)  = gesture.Point_2st.y;
		*(data + 8)  = gesture.Point_3st.x;
		*(data + 9)  = gesture.Point_3st.y;
		*(data + 10) = gesture.Point_4st.x;
		*(data + 11) = gesture.Point_4st.y;
		nlength = 6;
		break;
	case GESTURE_C:
		*data        = gesture.Point_start.x;
		*(data + 1)  = gesture.Point_start.y;
		*(data + 2)  = gesture.Point_2st.x;
		*(data + 3)  = gesture.Point_2st.y;
		*(data + 4)  = gesture.Point_end.x;
		*(data + 5)  = gesture.Point_end.y;
		*(data + 6)  = gesture.Point_1st.x;
		*(data + 7)  = gesture.Point_1st.y;
		*(data + 8)  = gesture.Point_3st.x;
		*(data + 9)  = gesture.Point_3st.y;
		*(data + 10) = gesture.Point_4st.x;
		*(data + 11) = gesture.Point_4st.y;
		nlength = 3;
		break;
	case GESTURE_O:
		*data        = gesture.Point_start.x;
		*(data + 1)  = gesture.Point_start.y;
		*(data + 2)  = gesture.Point_end.x;
		*(data + 3)  = gesture.Point_end.y;
		*(data + 4)  = gesture.Point_1st.x;
		*(data + 5)  = gesture.Point_1st.y;
		*(data + 6)  = gesture.Point_2st.x;
		*(data + 7)  = gesture.Point_2st.y;
		*(data + 8)  = gesture.Point_3st.x;
		*(data + 9)  = gesture.Point_3st.y;
		*(data + 10) = gesture.Point_4st.x;
		*(data + 11) = gesture.Point_4st.y;
		if (idev->debug_buf[0][34]) {/*point_data[34] = 1 clockwise*/
			vtsGesturePointsReport(VTS_GESTURE_O_DIR,1,-1,-1); 
		}
		nlength = 6;
		break;
	case GESTURE_F:
		*data        = gesture.Point_start.x;
		*(data + 1)  = gesture.Point_start.y;
		*(data + 2)  = gesture.Point_end.x;
		*(data + 3)  = gesture.Point_end.y;
		*(data + 4)  = gesture.Point_1st.x;
		*(data + 5)  = gesture.Point_1st.y;
		*(data + 6)  = gesture.Point_2st.x;
		*(data + 7)  = gesture.Point_2st.y;
		*(data + 8)  = gesture.Point_3st.x;
		*(data + 9)  = gesture.Point_3st.y;
		*(data + 10) = gesture.Point_4st.x;
		*(data + 11) = gesture.Point_4st.y;
		nlength = 6;
		break;
	case GESTURE_A:
		*data        = gesture.Point_start.x;
		*(data + 1)  = gesture.Point_start.y;
		*(data + 2)  = gesture.Point_end.x;
		*(data + 3)  = gesture.Point_end.y;
		*(data + 4)  = gesture.Point_1st.x;
		*(data + 5)  = gesture.Point_1st.y;
		*(data + 6)  = gesture.Point_2st.x;
		*(data + 7)  = gesture.Point_2st.y;
		*(data + 8)  = gesture.Point_3st.x;
		*(data + 9)  = gesture.Point_3st.y;
		*(data + 10) = gesture.Point_4st.x;
		*(data + 11) = gesture.Point_4st.y;
		nlength = 6;
		break;
	default:
		nlength = -1;
		break;
	}
	ipio_info("Point_start (x = %d,y = %d),Point_end (x = %d,y = %d)\n", gesture.Point_start.x, gesture.Point_start.y, gesture.Point_end.x, gesture.Point_end.y);
	ipio_info("Point_1st (x = %d,y = %d),Point_2st (x = %d,y = %d)\n", gesture.Point_1st.x, gesture.Point_1st.y, gesture.Point_2st.x, gesture.Point_2st.y);
	ipio_info("Point_3st (x = %d,y = %d),Point_4st (x = %d,y = %d)\n", gesture.Point_3st.x, gesture.Point_3st.y, gesture.Point_4st.x, gesture.Point_4st.y);
	ipio_info("idev->panel_wid = %d ,  idev->panel_hei = %d \n", idev->panel_wid, idev->panel_hei);
	idev->debug_node_open = DISABLE;
	ret = nlength;
	return ret;
}

int bbk_ili_set_charger_bit(int state)
{
	int ret = 0;

	ipio_info("state = %d\n", state);	
	mutex_lock(&idev->touch_mutex);

	switch (state) {
	case CHARGER_OFF:
		ipio_info("set charger off\n");
		ret = ilitek_tddi_ic_func_ctrl("plug", ENABLE);/* plug out*/
		break;
	case CHARGER_ON:
		ipio_info("set charger on\n");
		ret = ilitek_tddi_ic_func_ctrl("plug", DISABLE);/* plug in*/
		break;
	default:
		ipio_err("Unknown type, %d\n", state);
		ret = -1;
		break;
	}	
	
	mutex_unlock(&idev->touch_mutex);
	return ret;
}

int bbk_ili_read_charger_bit(void)
{
	int res = 0;
	uint8_t cmd[2] = { 0 };
	mutex_lock(&idev->touch_mutex);
	cmd[0] = 0x48;
	cmd[1] = 0x0;
	idev->write(cmd, 1);

	mdelay(10);

	res = idev->read(cmd, 2);
	mutex_unlock(&idev->touch_mutex);
	if (res < 0) {
		ipio_err("Failed to read fw charger %d\n", res);
		return -1;
	}
	if (cmd[1]) {
		ipio_info("cmd[1] = 1\n");
		return 1;
	} else {
		ipio_info("cmd[1] = 0\n");	
		return 0;
	}
}

int iliSetEdgeRestainSwitch(int on)
{
	int ret = 0;

	ipio_info("on = %d\n", on);	
	mutex_lock(&idev->touch_mutex);
	
	switch (on) {
	case EDGE_PALM_LEFT_UP_RIGHT_DOWN:
		ipio_info("Edge: left rotation\n");
		ret = ilitek_tddi_ic_func_ctrl("edge_palm", 0x2);/*ilitek_tddi_edge_palm_ctrl(2);*/
		break;
	case EDGE_PALM_STRAIGHT:
		ipio_info("Edge: straight\n");
		ret = ilitek_tddi_ic_func_ctrl("edge_palm", 0x1);/*ilitek_tddi_edge_palm_ctrl(1);*/
		break;
	case EDGE_PALM_LEFT_DOWN_RIGHT_UP:
		ipio_info("Edge: right rotation\n");
		ret = ilitek_tddi_ic_func_ctrl("edge_palm", 0x0);/*ilitek_tddi_edge_palm_ctrl(0);*/
		break;
	default:
		ipio_err("Unknown type, %d\n", on);
		ret = -1;
		break;
	}
	
	mutex_unlock(&idev->touch_mutex);
	return ret;
}

int bbk_ili_get_header_file_version(int which, unsigned char *fw)
{
	int ret = 0;

	switch (which) {
	case GET_FW_CONFIG_VER:
		ipio_info("no config version\n");
		ret = 0;
		break;
	case GET_FW_VER:
		ipio_info("get header fw version\n");
		ret = ((fw[FW_VER_ADDR] << 24) | (fw[FW_VER_ADDR + 1] << 16) |
				(fw[FW_VER_ADDR + 2] << 8) | (fw[FW_VER_ADDR + 3])) & 0x7fffffff ;
		break;
	default:
		ipio_err("Unknown type, %d\n", which);
		ret = -1;
		break;
	}
	ipio_info("Header firmware version = %x\n", ret);
	return ret;
}

struct fw_debug_info {
	u8 id;
	u8 app_sys_powr_state_e : 3;
	u8 app_sys_state_e : 3;
	u8 tp_state_e : 2;
	u8 touch_palm_state_e : 2;
	u8 app_an_statu_e : 3;
	u8 app_sys_check_bg_abnormal : 1;
	u8 g_b_wrong_bg: 1;
	u8 reserved0 : 1;

        u8 status_of_dynamic_normal : 1;
        u8 status_of_dynamic_charger : 1;
        u8 status_of_dynamic_noise : 1;
	u8 reserved1 : 5;

	u32 algo_pt_status0 : 3;
	u32 algo_pt_status1 : 3;
	u32 algo_pt_status2 : 3;
	u32 algo_pt_status3 : 3;
	u32 algo_pt_status4 : 3;
	u32 algo_pt_status5 : 3;
	u32 algo_pt_status6 : 3;
	u32 algo_pt_status7 : 3;
	u32 algo_pt_status8 : 3;
	u32 algo_pt_status9 : 3;
	u32 reserved2 : 2;

        u8 nodp : 5;
        u8 high_byte_frequecy : 3;

        u8 low_byte_freqeucy;
};
int bbk_ili_get_fw_debug_info(unsigned char *buf)
{
		int ret = 0, len = 0;
        struct fw_debug_info fdi;
        u8 cmd[1] = {0};
        u8 fw_buf[10] = {0};

        memset(&fdi, 0x0, sizeof(fdi));

        mutex_lock(&idev->touch_mutex);

        cmd[0] = 0x47;
        ret = idev->write(cmd, 1);
        if (ret < 0) {
                ipio_err("write 0x47 cmd error\n");
                ret = -1;
                goto out;
        }

        ret = idev->read(fw_buf, sizeof(fw_buf));
        if (ret < 0) {
                ipio_err("read fw debug info error\n");
                ret = -1;
                goto out;
        }

        ilitek_dump_data(fw_buf, 8, sizeof(fw_buf), 0, "fw debug info");

        memcpy(&fdi, fw_buf, sizeof(fdi));

        len += sprintf(buf, "header = %d\n", fdi.id);
        len += sprintf(buf + len, "app_sys_powr_state_e = %d\n", fdi.app_sys_powr_state_e);
        len += sprintf(buf + len, "app_sys_state_e = %d\n", fdi.app_sys_state_e);
        len += sprintf(buf + len, "tp_state_e = %d\n", fdi.tp_state_e);
        len += sprintf(buf + len, "touch_palm_state_e = %d\n", fdi.touch_palm_state_e);
        len += sprintf(buf + len, "app_an_statu_e = %d\n", fdi.app_an_statu_e);
        len += sprintf(buf + len, "app_sys_check_bg_abnormal = %d\n", fdi.app_sys_check_bg_abnormal);
        len += sprintf(buf + len, "g_b_wrong_bg = %d\n", fdi.g_b_wrong_bg);

        len += sprintf(buf + len, "status_of_dynamic_normal = %d\n", fdi.status_of_dynamic_normal);
        len += sprintf(buf + len, "status_of_dynamic_charger = %d\n", fdi.status_of_dynamic_charger);
        len += sprintf(buf + len, "status_of_dynamic_noise = %d\n", fdi.status_of_dynamic_noise);

        len += sprintf(buf + len, "algo_pt_status0 = %d\n", fdi.algo_pt_status0);
        len += sprintf(buf + len, "algo_pt_status1 = %d\n", fdi.algo_pt_status1);
        len += sprintf(buf + len, "algo_pt_status2 = %d\n", fdi.algo_pt_status2);
        len += sprintf(buf + len, "algo_pt_status3 = %d\n", fdi.algo_pt_status3);
        len += sprintf(buf + len, "algo_pt_status4 = %d\n", fdi.algo_pt_status4);
        len += sprintf(buf + len, "algo_pt_status5 = %d\n", fdi.algo_pt_status5);
        len += sprintf(buf + len, "algo_pt_status6 = %d\n", fdi.algo_pt_status6);
        len += sprintf(buf + len, "algo_pt_status7 = %d\n", fdi.algo_pt_status7);
        len += sprintf(buf + len, "algo_pt_status8 = %d\n", fdi.algo_pt_status8);
        len += sprintf(buf + len, "algo_pt_status9 = %d\n", fdi.algo_pt_status9);

        len += sprintf(buf + len, "nodp = %d\n", fdi.nodp);
        len += sprintf(buf + len, "high_byte_frequecy = %d\n", fdi.high_byte_frequecy);
        len += sprintf(buf + len, "low_byte_freqeucy = %d\n", fdi.low_byte_freqeucy);

	ipio_info("id = %d\n", fdi.id);
	ipio_info("app_sys_powr_state_e = %d\n", fdi.app_sys_powr_state_e);
	ipio_info("app_sys_state_e = %d\n", fdi.app_sys_state_e);
	ipio_info("tp_state_e = %d\n", fdi.tp_state_e);
	ipio_info("touch_palm_state_e = %d\n", fdi.touch_palm_state_e);
	ipio_info("app_an_statu_e = %d\n", fdi.app_an_statu_e);
	ipio_info("app_sys_check_bg_abnormal = %d\n", fdi.app_sys_check_bg_abnormal);
	ipio_info("g_b_wrong_bg = %d\n", fdi.g_b_wrong_bg);
        ipio_info("status_of_dynamic_normal = %d\n", fdi.status_of_dynamic_normal);
        ipio_info("status_of_dynamic_charger = %d\n", fdi.status_of_dynamic_charger);
        ipio_info("status_of_dynamic_noise = %d\n", fdi.status_of_dynamic_noise);
	ipio_info("algo_pt_status0 = %d\n", fdi.algo_pt_status0);
	ipio_info("algo_pt_status1 = %d\n", fdi.algo_pt_status1);
	ipio_info("algo_pt_status2 = %d\n", fdi.algo_pt_status2);
	ipio_info("algo_pt_status3 = %d\n", fdi.algo_pt_status3);
	ipio_info("algo_pt_status4 = %d\n", fdi.algo_pt_status4);
	ipio_info("algo_pt_status5 = %d\n", fdi.algo_pt_status5);
	ipio_info("algo_pt_status6 = %d\n", fdi.algo_pt_status6);
	ipio_info("algo_pt_status7 = %d\n", fdi.algo_pt_status7);
	ipio_info("algo_pt_status8 = %d\n", fdi.algo_pt_status8);
	ipio_info("algo_pt_status9 = %d\n", fdi.algo_pt_status9);
	ipio_info("node = %d\n", fdi.nodp);
	ipio_info("low byte frequecy = %d\n", fdi.high_byte_frequecy);
	ipio_info("high byte frequecy = %d\n", fdi.low_byte_freqeucy);

out:
        mutex_unlock(&idev->touch_mutex);
        return (ret < 0) ? ret : len;

}

int vivo_ili_module_id(void)
{
	/*just for display in *#225# view*/
	return VTS_MVC_TRY; 
}

extern unsigned int report_lcm_id(void);
int get_lcm_id(void)
{

	int lcm_id;

	lcm_id = report_lcm_id();

	VTI("lcm id: 0x%x", lcm_id);

	return lcm_id;
}

int bbk_ili_process_by_package(unsigned char *package_name) 
{
	struct vivo_ts_struct *vtsData = vivoTsGetVtsData();

	if (!strcmp(package_name, "vts_gs:vivo_ts:0")) {
		VTI("sleepMode:0");    
		vtsData->gestureMode = 0;
    } else if (strstr(package_name, "vts_gs:vivo_ts")) {
		VTI("gestureMode:1");    
		vtsData->gestureMode = 1;
	}

	return 0;
}