/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 ov13855pd1901mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *----------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#define PFX "OV13855PD1901_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "../imgsensor_i2c.h"
#include "ov13855pd1901mipiraw_Sensor.h"
#define LOG_INF(format, args...)    \
	pr_debug(PFX "[%s] " format, __func__, ##args)



#define MULTI_WRITE 1
#define ORINGNAL_VERSION 0

#if MULTI_WRITE
#define I2C_BUFFER_LEN 1020	/* trans# max is 255, each 4 bytes */
#else
#define I2C_BUFFER_LEN 4
#endif


static DEFINE_SPINLOCK(imgsensor_drv_lock);
static bool bIsLongExposure = KAL_FALSE;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV13855PD1901_SENSOR_ID,

	.checksum_value = 0xb1f1b3cc,

	.pre = {
		.pclk = 108000000,      /*record different mode's pclk*/
		.linelength = 1122,	/*record different mode's linelength*/
		.framelength = 3214,    /*record different mode's framelength*/
		.startx = 0,	/*record different mode's startx of grabwindow*/
		.starty = 0,	/*record different mode's starty of grabwindow*/

		/*record different mode's width of grabwindow*/
		.grabwindow_width = 2112,
		/*record different mode's height of grabwindow*/
		.grabwindow_height = 1568,

		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount
		 * by different scenario
		 */
		.mipi_data_lp2hs_settle_dc = 90,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 225600000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 108000000,
		.linelength = 1122,
		.framelength = 3240,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4224,
		.grabwindow_height = 3136,
		.mipi_data_lp2hs_settle_dc = 90,
		.mipi_pixel_rate = 451200000,
		.max_framerate = 297,
	},
	.normal_video = {
		.pclk = 108000000,
		.linelength = 1122,
		.framelength = 3214,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2112,
		.grabwindow_height = 1568,
		.mipi_data_lp2hs_settle_dc = 90,
		.mipi_pixel_rate = 225600000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 108000000,
		.linelength = 1122,
		.framelength = 874,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1024,
		.grabwindow_height = 768,
		.mipi_data_lp2hs_settle_dc = 90,
		.mipi_pixel_rate = 225600000,
		.max_framerate = 1100,
	},
	.slim_video = {
		.pclk = 108000000,	/*record different mode's pclk*/
		.linelength = 1122,	/*record different mode's linelength*/
		.framelength = 3216,	/*record different mode's framelength*/
		.startx = 0,	/*record different mode's startx of grabwindow*/
		.starty = 0,	/*record different mode's starty of grabwindow*/

		/*record different mode's width of grabwindow*/
		.grabwindow_width = 1024,
		/*record different mode's height of grabwindow*/
		.grabwindow_height = 768,

		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount
		 * by different scenario
		 */
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 225600000,
		.max_framerate = 300,

	},
	.custom1 = {
		.pclk = 108000000,
		.linelength = 1122,
		.framelength = 4010,//3240,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,//4224,
		.grabwindow_height = 2448,//3136,
		.mipi_data_lp2hs_settle_dc = 90,
		.mipi_pixel_rate = 451200000,
		.max_framerate = 240,
	},
	.custom2 = {
		.pclk = 108000000,
		.linelength = 1122,
		.framelength = 4812,//3240,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,//4224,
		.grabwindow_height = 2448,//3136,
		.mipi_data_lp2hs_settle_dc = 90,
		.mipi_pixel_rate = 451200000,
		.max_framerate = 200,
	},
	.margin = 8,                    /*sensor framelength & shutter margin*/
	.min_shutter = 4,               /*min shutter*/

	/*max framelength by sensor register's limitation*/
	.max_frame_length = 0x7fff,
	/*shutter delay frame for AE cycle, 2 frame*/
	.ae_shut_delay_frame = 0,
	/*sensor gain delay frame for AE cycle,2 frame*/
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,    /*isp gain delay frame for AE cycle*/
	.frame_time_delay_frame = 2,	/* The delay frame of setting frame length  */
	.ihdr_support = 0,	            /*1, support; 0,not support*/
	.ihdr_le_firstline = 0,         /*1,le first ; 0, se first*/
	.sensor_mode_num = 7,	        /*support sensor mode num*/

	.cap_delay_frame = 2,           /*enter capture delay frame num*/
	.pre_delay_frame = 2,           /*enter preview delay frame num*/
	.video_delay_frame = 2,         /*enter video delay frame num*/
	.hs_video_delay_frame = 2,   /*enter high speed video  delay frame num*/
	.slim_video_delay_frame = 2, /*enter slim video delay frame num*/
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_2MA,     /*mclk driving current*/

	/*sensor_interface_type*/
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	/*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	/*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
	.mipi_settle_delay_mode = 1,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.mclk = 24,         /*mclk value, suggest 24 or 26 for 24Mhz or 26Mhz*/
	.mipi_lane_num = SENSOR_MIPI_4_LANE,

	/*record sensor support all write id addr*/
	.i2c_addr_table = {0x6c, 0xff},
	.i2c_speed = 400,
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,		/*mirrorflip information*/

	/*IMGSENSOR_MODE enum value,record current sensor mode*/
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,			/*current shutter*/
	.gain = 0x100,				/*current gain*/
	.dummy_pixel = 0,			/*current dummypixel*/
	.dummy_line = 0,			/*current dummyline*/

	/*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.current_fps = 0,
	/*auto flicker enable: KAL_FALSE for disable auto flicker*/
	.autoflicker_en = KAL_FALSE,
	/*test pattern mode or not. KAL_FALSE for in test pattern mode*/
	.test_pattern = KAL_FALSE,
	/*current scenario id*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	/*sensor need support LE, SE with HDR feature*/
	.ihdr_mode = 0,

	.i2c_write_id = 0x6c,  /*record current sensor's i2c write id*/

};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[7] = {
{ 4256, 3168, 0,   8, 4256, 3152, 2128, 1576,  8,	4,	 2112, 1568, 0,   0,   2112, 1568}, // Preview
{ 4256, 3168, 0,   8, 4256, 3152, 4256, 3152,  16,	8,	 4224, 3136, 0,   0,   4224, 3136},//capture
{ 4256, 3168, 0,   8, 4256, 3152, 2128, 1576,  8,	4,	 2112, 1568, 0,   0,   2112, 1568},//normal-video
#if 1
{ 4256, 3168, 64, 32, 4128, 3104, 1032,  776,  4,	4,	 1024,	768, 0,   0,   1024,  768},/*hs-video*/
#else
{ 4256, 3168, 64, 32, 4128, 3104, 1032,  776,  196, 148,	 640,	480, 0,   0,	640,  480},/*hs-video*/
#endif
{ 4256, 3168, 64, 32, 4128, 3104, 1032,  776,  4,	4,	 1024,	768, 0,   0,   1024,  768},/*slim-video*/
{ 4256, 3168, 0,   8, 4256, 3152, 4256, 3152,  496, 352,  3264, 2448, 0,   0,   3264, 2448},//custom1
{ 4256, 3168, 0,   8, 4256, 3152, 4256, 3152,  496,	352,  3264, 2448, 0,   0,   3264, 2448},//custom2
};

/*PD information update*/
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	 .i4OffsetX = 0,
	 .i4OffsetY = 0,
	 .i4PitchX	= 32,
	 .i4PitchY	= 32,
	 .i4PairNum  =8,//32*32 is cropped into 16*8 sub block of 8 pairnum, per sub block includes just only one pd pair pixels
	 .i4SubBlkW  =16,
	 .i4SubBlkH  =8,
	 .i4PosL = {{14, 6},{30, 6},{6, 10},{22, 10},{14, 22},{30, 22},{6, 26},{22, 26},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}},
	 .i4PosR = {{14, 2},{30, 2},{6, 14},{22, 14},{14, 18},{30, 18},{6, 30},{22, 30},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}},
	 .i4BlockNumX = 132,
	 .i4BlockNumY = 98,

};

/*vivo hope add for Camera otp errorcode*/
extern int ov13855pd1901_vivo_otp_read(void);
static int vivo_otp_read_when_power_on = 0;
extern otp_error_code_t OV13855PD1901_OTP_ERROR_CODE;
extern unsigned int is_atboot;
extern unsigned  int ov13855pd1901_flag;
/*vivo hope add end*/
#if 0
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte << 8) & 0xff00) | ((get_byte >> 8) & 0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {
		(char)(addr >> 8), (char)(addr & 0xFF),
		(char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}
#endif
static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {
		(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	pr_debug("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	/*return; //for test*/
    write_cmos_sensor_8(0x380c, imgsensor.line_length >> 8);
    write_cmos_sensor_8(0x380d, imgsensor.line_length & 0xFF);
    write_cmos_sensor_8(0x380e, imgsensor.frame_length >> 8);
    write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
}	/*	set_dummy  */
#if 0
static kal_uint16 table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;

	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE

	if (tosend >= I2C_BUFFER_LEN || IDX == len || addr != addr_last) {
		iBurstWriteReg_multi(puSendCmd, tosend,
			imgsensor.i2c_write_id, 3, imgsensor_info.i2c_speed);

			tosend = 0;
		}
#else
		iWriteRegI2CTiming(puSendCmd, 3,
			imgsensor.i2c_write_id, imgsensor_info.i2c_speed);

		tosend = 0;

#endif
	}
	return 0;
}
#endif
static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	pr_debug("framerate = %d, min framelength should enable %d\n",
		framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	imgsensor.dummy_line =
		imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;

		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
    //check
	kal_uint16 realtime_fps = 0;
    pr_debug("write shutter :%d\n",shutter);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;//increase current frame_length that makes shutter <= frame_length - margin.
    else
        imgsensor.frame_length = imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;

	if (imgsensor.autoflicker_en == KAL_TRUE)
	{
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305){
			realtime_fps = 296;
            set_max_framerate(realtime_fps,0);
		}
        else if(realtime_fps >= 147 && realtime_fps <= 150){
			realtime_fps = 146;
            set_max_framerate(realtime_fps ,0);
		}
        else
        {
        	imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
            write_cmos_sensor_8(0x380e, imgsensor.frame_length >> 8);
            write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
        }
    }
    else
    {
    	imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
        write_cmos_sensor_8(0x380e, imgsensor.frame_length >> 8);
        write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
    }

	if(shutter > 32759) {
		/*enter long exposure mode */
		kal_uint32 exposure_time;
		kal_uint32 long_shutter;
		pr_debug("enter long exposure mode\n");
		pr_debug("Calc long exposure  +\n");
		//exposure_time = (double)(shutter*imgsensor.line_length)/imgsensor.pclk;//108000000
		//long_shutter = (kal_uint32)((exposure_time-0.0345)*1000000000/9.26/256);
		exposure_time = shutter*imgsensor_info.cap.linelength/108;//us
        long_shutter = ((exposure_time - 34500)/256)*(100000/926);
        pr_debug("Calc long exposure exposure_time:%d long_shutter %d -\n",exposure_time,long_shutter);
		long_shutter = (long_shutter >> 1) << 1;
		pr_debug("Calc long exposure  -\n");

		write_cmos_sensor_8(0x3208, 0x00);  /* group hold */
		write_cmos_sensor_8(0x3018, 0xfd);
		write_cmos_sensor_8(0x3019, 0xf2);
		write_cmos_sensor_8(0x3c80, 0x0c);
		write_cmos_sensor_8(0x3c82, 0x00);
		write_cmos_sensor_8(0x3c83, 0x7c);
		write_cmos_sensor_8(0x3107, 0x23);
		write_cmos_sensor_8(0x3c99, 0x03);  /* enable long exposure mode */
		write_cmos_sensor_8(0x3c9a, (long_shutter >> 24) & 0xff);
		write_cmos_sensor_8(0x3c9b, (long_shutter >> 16) & 0xff);
		write_cmos_sensor_8(0x3c9c, (long_shutter >> 8) & 0xff);
		write_cmos_sensor_8(0x3c9d, long_shutter & 0xff); /* wirte long shtter */
		/* settting for CLK and IQ */
		write_cmos_sensor_8(0x380e, 0x0E);
		write_cmos_sensor_8(0x380f, 0x00);
		write_cmos_sensor_8(0x3500, 0x00);
		write_cmos_sensor_8(0x3501, 0xD0);
		write_cmos_sensor_8(0x3502, 0x00);
		write_cmos_sensor_8(0x3508, 0x00);
		write_cmos_sensor_8(0x3509, 0x80); /*1xgain*/
		write_cmos_sensor_8(0x5203, 0x04);
		write_cmos_sensor_8(0x5204, 0x00);
		write_cmos_sensor_8(0x5205, 0x40);
		write_cmos_sensor_8(0x520D, 0xf5);
		write_cmos_sensor_8(0x520e, 0xf5);
		write_cmos_sensor_8(0x5207, 0x84);
		write_cmos_sensor_8(0x5208, 0xFF);
		write_cmos_sensor_8(0x5201, 0x9c);
		write_cmos_sensor_8(0x5200, 0x19);
		write_cmos_sensor_8(0x5003, 0x04);
		write_cmos_sensor_8(0x5006, 0x00);
		write_cmos_sensor_8(0x5007, 0xF8);
		write_cmos_sensor_8(0x5000, 0xED);
		write_cmos_sensor_8(0x5001, 0x05);
		write_cmos_sensor_8(0x3018, 0xf0);
		write_cmos_sensor_8(0x3019, 0xf0);
		write_cmos_sensor_8(0x3208, 0x10);  /*enter long exp; group 0 hold end*/
		write_cmos_sensor_8(0x3208, 0x02);  /*group 2*/
		write_cmos_sensor_8(0x3018, 0xf9);
		write_cmos_sensor_8(0x3019, 0xf2);
		write_cmos_sensor_8(0x3c80, 0x00);
		write_cmos_sensor_8(0x3c82, 0x00);
		write_cmos_sensor_8(0x3c83, 0xf9);
		write_cmos_sensor_8(0x3107, 0x23);
		write_cmos_sensor_8(0x3c99, 0x00);
		write_cmos_sensor_8(0x3c9a, 0x00);
		write_cmos_sensor_8(0x3c9b, 0x00);
		write_cmos_sensor_8(0x3c9c, 0x00);
		write_cmos_sensor_8(0x3c9d, 0x10);
		write_cmos_sensor_8(0x380e, 0x0c);
		write_cmos_sensor_8(0x380f, 0x8e);
		write_cmos_sensor_8(0x3500, 0x00);
		write_cmos_sensor_8(0x3501, 0x80);
		write_cmos_sensor_8(0x3502, 0x00);
		write_cmos_sensor_8(0x5203, 0x24);
		write_cmos_sensor_8(0x5204, 0x12);
		write_cmos_sensor_8(0x5205, 0x41);
		write_cmos_sensor_8(0x520D, 0x0f);
		write_cmos_sensor_8(0x520e, 0xfd);
		write_cmos_sensor_8(0x5207, 0x84);
		write_cmos_sensor_8(0x5208, 0x40);
		write_cmos_sensor_8(0x5201, 0x94);
		write_cmos_sensor_8(0x5200, 0x1b);
		write_cmos_sensor_8(0x5003, 0x00);
		write_cmos_sensor_8(0x5006, 0x00);
		write_cmos_sensor_8(0x5007, 0x10);
		write_cmos_sensor_8(0x5000, 0xff);
		write_cmos_sensor_8(0x5001, 0x07);
		write_cmos_sensor_8(0x3018, 0xf0);
		write_cmos_sensor_8(0x3019, 0xf0);
		write_cmos_sensor_8(0x3208, 0x12); /*exit long exp; group 2 hold end*/
		write_cmos_sensor_8(0x3209, 0x42); /*stay 2 frame; auto exit long exp*/
		write_cmos_sensor_8(0x320b, 0x80);
		write_cmos_sensor_8(0x3208, 0xa0); /*group launch*/
		bIsLongExposure = KAL_TRUE;
	} else {
	    #if 0
		/*normal exposure mode*/
		if(bIsLongExposure) {
			/*exit long exposure mode if use short normal exposure*/
			pr_debug("exit long exposure mode\n");
			write_cmos_sensor_8(0x3208, 0x00);  /* group hold */
			write_cmos_sensor_8(0x3018, 0xf9);
			write_cmos_sensor_8(0x3019, 0xf2);
			write_cmos_sensor_8(0x3c80, 0x00);
			write_cmos_sensor_8(0x3c82, 0x00);
			write_cmos_sensor_8(0x3c83, 0xF9);
			write_cmos_sensor_8(0x3107, 0x23);
			write_cmos_sensor_8(0x3c99, 0x00);
			write_cmos_sensor_8(0x3c9a, 0x00);
			write_cmos_sensor_8(0x3c9b, 0x00);
			write_cmos_sensor_8(0x3c9c, 0x00);
			write_cmos_sensor_8(0x3c9d, 0x10);
			write_cmos_sensor_8(0x380E, 0x0C);
			write_cmos_sensor_8(0x380F, 0x8E);
			write_cmos_sensor_8(0x3500, 0x00);
			write_cmos_sensor_8(0x3501, 0x80);
			write_cmos_sensor_8(0x3502, 0x00);
			write_cmos_sensor_8(0x3508, 0x00);
			write_cmos_sensor_8(0x3509, 0x80);
			write_cmos_sensor_8(0x5203, 0x24);
			write_cmos_sensor_8(0x5204, 0x12);
			write_cmos_sensor_8(0x5205, 0x41);
			write_cmos_sensor_8(0x520D, 0x0F);
			write_cmos_sensor_8(0x520E, 0xFD);
			write_cmos_sensor_8(0x5207, 0x84);
			write_cmos_sensor_8(0x5208, 0x40);
			write_cmos_sensor_8(0x5201, 0x94);
			write_cmos_sensor_8(0x5200, 0x1B);
			write_cmos_sensor_8(0x5003, 0x00);
			write_cmos_sensor_8(0x5006, 0x00);
			write_cmos_sensor_8(0x5007, 0x10);
			write_cmos_sensor_8(0x5000, 0xFF);
			write_cmos_sensor_8(0x5001, 0x07);
			write_cmos_sensor_8(0x3018, 0xf0);
			write_cmos_sensor_8(0x3019, 0xf0);
			write_cmos_sensor_8(0x3208, 0x10); /*group 0 hold end */
			write_cmos_sensor_8(0x3208, 0xA0); /*group 0 launch */
			bIsLongExposure = KAL_FALSE;
		}
	    #endif
	    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	    //frame_length and shutter should be an even number.
	    shutter = (shutter >> 1) << 1;
	    imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
		/*Warning : shutter must be even. Odd might happen Unexpected Results */
		write_cmos_sensor_8(0x3500, (shutter >> 12) & 0x0F);
		write_cmos_sensor_8(0x3501, (shutter >> 4) & 0xFF);
		write_cmos_sensor_8(0x3502, (shutter<<4)  & 0xF0);
	}

    pr_debug("shutter =%d, framelength =%d, realtime_fps =%d\n", shutter,imgsensor.frame_length, realtime_fps);
}

static void set_shutter_frame_length(
	kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
#if ORINGNAL_VERSION
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
    /*Change frame time*/
	dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
#else
	 spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/*Change frame time*/
	if (frame_length > 1)
		imgsensor.frame_length = frame_length;
/* */
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
#endif
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;

	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		shutter =
		(imgsensor_info.max_frame_length - imgsensor_info.margin);

	if (imgsensor.autoflicker_en) {

		realtime_fps =
	   imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length*/
            write_cmos_sensor_8(0x380e, imgsensor.frame_length >> 8);
            write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
		/* Extend frame length*/
            write_cmos_sensor_8(0x380e, imgsensor.frame_length >> 8);
            write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
	}
	/* Update Shutter*/
		write_cmos_sensor_8(0x3500, (shutter >> 12) & 0x0F);
		write_cmos_sensor_8(0x3501, (shutter >> 4) & 0xFF);
		write_cmos_sensor_8(0x3502, (shutter<<4)  & 0xF0);

	pr_debug("Add for N3D! shutterlzl =%d, framelength =%d\n",
		shutter, imgsensor.frame_length);

}


/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;

	//platform 1xgain = 64, sensor driver 1*gain = 0x80
	iReg = gain*128/BASEGAIN;

	if(iReg < 0x80)// sensor 1xGain
	{
		iReg = 0X80;
	}
	if(iReg > 0x7c0)// sensor 15.5xGain
	{
		iReg = 0X7C0;
	}

	return iReg;
}

/*************************************************************************
 * FUNCTION
 * set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	pr_debug("gain = %d , reg_gain = 0x%x\n", gain, reg_gain);

    write_cmos_sensor_8(0x03508,(reg_gain >> 8)); 
    write_cmos_sensor_8(0x03509,(reg_gain&0xff));

	return gain;
}	/*	set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{

/*	kal_uint8 itemp;*/

	pr_debug("image_mirror = %d\n", image_mirror);

}

/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
#endif

static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		if (0) {
			/*ensure exit long exposure mode when chaning scenario*/
			write_cmos_sensor_8(0x3c80, 0x00);
			write_cmos_sensor_8(0x3c82, 0x00);
			write_cmos_sensor_8(0x3c83, 0xF9);
			write_cmos_sensor_8(0x3107, 0x23);
			write_cmos_sensor_8(0x3c99, 0x00);
			write_cmos_sensor_8(0x3c9a, 0x00);
			write_cmos_sensor_8(0x3c9b, 0x00);
			write_cmos_sensor_8(0x3c9c, 0x00);
			write_cmos_sensor_8(0x3c9d, 0x10);
			write_cmos_sensor_8(0x380E, 0x0C);
			write_cmos_sensor_8(0x380F, 0x8E);
			write_cmos_sensor_8(0x3500, 0x00);
			write_cmos_sensor_8(0x3501, 0x80);
			write_cmos_sensor_8(0x3502, 0x00);
			write_cmos_sensor_8(0x3508, 0x00);
			write_cmos_sensor_8(0x3509, 0x80);
			write_cmos_sensor_8(0x5203, 0x24);
			write_cmos_sensor_8(0x5204, 0x12);
			write_cmos_sensor_8(0x5205, 0x41);
			write_cmos_sensor_8(0x520D, 0x0F);
			write_cmos_sensor_8(0x520E, 0xFD);
			write_cmos_sensor_8(0x5207, 0x84);
			write_cmos_sensor_8(0x5208, 0x40);
			write_cmos_sensor_8(0x5201, 0x94);
			write_cmos_sensor_8(0x5200, 0x1B);
			write_cmos_sensor_8(0x5003, 0x00);
			write_cmos_sensor_8(0x5006, 0x00);
			write_cmos_sensor_8(0x5007, 0x10);
			write_cmos_sensor_8(0x5000, 0xFF);
			write_cmos_sensor_8(0x5001, 0x07);
			bIsLongExposure = KAL_FALSE;
		}
		write_cmos_sensor_8(0x0100, 0x01);
	}
	else
		write_cmos_sensor_8(0x0100, 0x00);
	mdelay(5);
	return ERROR_NONE;
}

static void sensor_init(void)
{
	pr_debug("sensor_init() E\n");
	
	write_cmos_sensor_8(0x0103, 0x01);//SW Reset, need delay 
	mdelay(5);

	write_cmos_sensor_8(0x0300, 0x02);
	write_cmos_sensor_8(0x0301, 0x00);
	write_cmos_sensor_8(0x0302, 0x5d); 
	write_cmos_sensor_8(0x0304, 0x00);
	write_cmos_sensor_8(0x0305, 0x01);
	write_cmos_sensor_8(0x030b, 0x06);
	write_cmos_sensor_8(0x030c, 0x02);
	write_cmos_sensor_8(0x030d, 0x88);
	write_cmos_sensor_8(0x0312, 0x11);
	write_cmos_sensor_8(0x3022, 0x01);
	write_cmos_sensor_8(0x3012, 0x40);
	write_cmos_sensor_8(0x3013, 0x72);
	write_cmos_sensor_8(0x3016, 0x72);
	write_cmos_sensor_8(0x301b, 0xf0);
	write_cmos_sensor_8(0x301f, 0xd0);
	write_cmos_sensor_8(0x3106, 0x15);
	write_cmos_sensor_8(0x3107, 0x23);
	write_cmos_sensor_8(0x3500, 0x00);
	write_cmos_sensor_8(0x3502, 0x00);
	write_cmos_sensor_8(0x3508, 0x02);
	write_cmos_sensor_8(0x3509, 0x00);
	write_cmos_sensor_8(0x350a, 0x00);
	write_cmos_sensor_8(0x350e, 0x00);
	write_cmos_sensor_8(0x3510, 0x00);
	write_cmos_sensor_8(0x3511, 0x02);
	write_cmos_sensor_8(0x3512, 0x00);
	write_cmos_sensor_8(0x3600, 0x2b);
	write_cmos_sensor_8(0x3601, 0x52);
	write_cmos_sensor_8(0x3602, 0x60);
	write_cmos_sensor_8(0x3612, 0x05);
	write_cmos_sensor_8(0x3613, 0xa4);
	write_cmos_sensor_8(0x3620, 0x80);
	write_cmos_sensor_8(0x3621, 0x10);
	write_cmos_sensor_8(0x3622, 0x30);
	write_cmos_sensor_8(0x3624, 0x1c);
	write_cmos_sensor_8(0x3640, 0x10);
	#if defined(CONFIG_MTK_CAM_PD1732)
	write_cmos_sensor_8(0x3641, 0xF0);
	#endif 
	#if defined(CONFIG_MTK_CAM_PD1803)
	write_cmos_sensor_8(0x3641, 0x70);
	#endif
	write_cmos_sensor_8(0x3661, 0x80);
	write_cmos_sensor_8(0x3664, 0x73);
	write_cmos_sensor_8(0x3665, 0xa7);
	write_cmos_sensor_8(0x366e, 0xff);
	write_cmos_sensor_8(0x366f, 0xf4);
	write_cmos_sensor_8(0x3674, 0x00);
	write_cmos_sensor_8(0x3679, 0x0c);
	write_cmos_sensor_8(0x367f, 0x01);
	write_cmos_sensor_8(0x3680, 0x0c);
	write_cmos_sensor_8(0x3681, 0x50);
	write_cmos_sensor_8(0x3682, 0x50);
	write_cmos_sensor_8(0x3683, 0xa9);
	write_cmos_sensor_8(0x3684, 0xa9);
	write_cmos_sensor_8(0x3709, 0x5f);
	write_cmos_sensor_8(0x371a, 0x3e);
	write_cmos_sensor_8(0x3738, 0xcc);
	write_cmos_sensor_8(0x373d, 0x26);
	write_cmos_sensor_8(0x3764, 0x20);
	write_cmos_sensor_8(0x3765, 0x20);
	write_cmos_sensor_8(0x37a1, 0x36);
	write_cmos_sensor_8(0x37a8, 0x3b);
	write_cmos_sensor_8(0x37ab, 0x31);
	write_cmos_sensor_8(0x37c3, 0xf1);
	write_cmos_sensor_8(0x37c5, 0x00);
	write_cmos_sensor_8(0x37d8, 0x03);
	write_cmos_sensor_8(0x37da, 0xc2);
	write_cmos_sensor_8(0x37dc, 0x02);
	write_cmos_sensor_8(0x37e0, 0x00);
	write_cmos_sensor_8(0x37e1, 0x0a);
	write_cmos_sensor_8(0x37e2, 0x14);
	write_cmos_sensor_8(0x37e5, 0x03);
	write_cmos_sensor_8(0x3800, 0x00);
	write_cmos_sensor_8(0x3804, 0x10);
	write_cmos_sensor_8(0x3815, 0x01);
	write_cmos_sensor_8(0x3817, 0x01);
	write_cmos_sensor_8(0x3821, 0x00);
	write_cmos_sensor_8(0x3822, 0xc2);
	write_cmos_sensor_8(0x3823, 0x18);
	write_cmos_sensor_8(0x3832, 0x00);
	write_cmos_sensor_8(0x3c80, 0x00);
	write_cmos_sensor_8(0x3c87, 0x01);
	write_cmos_sensor_8(0x3c8c, 0x19);
	write_cmos_sensor_8(0x3c8d, 0x1c);
	write_cmos_sensor_8(0x3c90, 0x00);
	write_cmos_sensor_8(0x3c91, 0x00);
	write_cmos_sensor_8(0x3c92, 0x00);
	write_cmos_sensor_8(0x3c93, 0x00);
	write_cmos_sensor_8(0x3c94, 0x40);
	write_cmos_sensor_8(0x3c95, 0x54);
	write_cmos_sensor_8(0x3c96, 0x34);
	write_cmos_sensor_8(0x3c97, 0x04);
	write_cmos_sensor_8(0x3c98, 0x00);
	write_cmos_sensor_8(0x3d8c, 0x73);
	write_cmos_sensor_8(0x3d8d, 0xc0);
	write_cmos_sensor_8(0x3f00, 0x0b);
	write_cmos_sensor_8(0x3f03, 0x00);
	write_cmos_sensor_8(0x4001, 0xe0);
	write_cmos_sensor_8(0x4008, 0x00);
	write_cmos_sensor_8(0x4011, 0xf0);
	write_cmos_sensor_8(0x4017, 0x08);
	write_cmos_sensor_8(0x4052, 0x00);
	write_cmos_sensor_8(0x4053, 0x80);
	write_cmos_sensor_8(0x4054, 0x00);
	write_cmos_sensor_8(0x4055, 0x80);
	write_cmos_sensor_8(0x4056, 0x00);
	write_cmos_sensor_8(0x4057, 0x80);
	write_cmos_sensor_8(0x4058, 0x00);
	write_cmos_sensor_8(0x4059, 0x80);
	write_cmos_sensor_8(0x405e, 0x20);
	write_cmos_sensor_8(0x4500, 0x07);
	write_cmos_sensor_8(0x4503, 0x00);
	write_cmos_sensor_8(0x450a, 0x04);
	write_cmos_sensor_8(0x4809, 0x04);
	write_cmos_sensor_8(0x480c, 0x12);
	write_cmos_sensor_8(0x481f, 0x30);
	write_cmos_sensor_8(0x4826, 0x3f);
	write_cmos_sensor_8(0x4833, 0x10);
	write_cmos_sensor_8(0x4d00, 0x03);
	write_cmos_sensor_8(0x4d01, 0xc9);
	write_cmos_sensor_8(0x4d02, 0xbc);
	write_cmos_sensor_8(0x4d03, 0xd7);
	write_cmos_sensor_8(0x4d04, 0xf0);
	write_cmos_sensor_8(0x4d05, 0xa2);
	write_cmos_sensor_8(0x5000, 0xff);
	write_cmos_sensor_8(0x5001, 0x07);
	write_cmos_sensor_8(0x5040, 0x39);
	write_cmos_sensor_8(0x5041, 0x10);
	write_cmos_sensor_8(0x5042, 0x10);
	write_cmos_sensor_8(0x5043, 0x84);
	write_cmos_sensor_8(0x5044, 0x62);
	write_cmos_sensor_8(0x5180, 0x00);
	write_cmos_sensor_8(0x5181, 0x10);
	write_cmos_sensor_8(0x5182, 0x02);
	write_cmos_sensor_8(0x5183, 0x0f);
	write_cmos_sensor_8(0x5200, 0x1b);
	write_cmos_sensor_8(0x520b, 0x07);
	write_cmos_sensor_8(0x520c, 0x0f);
	write_cmos_sensor_8(0x5300, 0x04);
	write_cmos_sensor_8(0x5301, 0x0c);
	write_cmos_sensor_8(0x5302, 0x0c);
	write_cmos_sensor_8(0x5303, 0x0f);
	write_cmos_sensor_8(0x5304, 0x00);
	write_cmos_sensor_8(0x5305, 0x70);
	write_cmos_sensor_8(0x5306, 0x00);
	write_cmos_sensor_8(0x5307, 0x80);
	write_cmos_sensor_8(0x5308, 0x00);
	write_cmos_sensor_8(0x5309, 0xa5);
	write_cmos_sensor_8(0x530a, 0x00);
	write_cmos_sensor_8(0x530b, 0xd3);
	write_cmos_sensor_8(0x530c, 0x00);
	write_cmos_sensor_8(0x530d, 0xf0);
	write_cmos_sensor_8(0x530e, 0x01);
	write_cmos_sensor_8(0x530f, 0x10);
	write_cmos_sensor_8(0x5310, 0x01);
	write_cmos_sensor_8(0x5311, 0x20);
	write_cmos_sensor_8(0x5312, 0x01);
	write_cmos_sensor_8(0x5313, 0x20);
	write_cmos_sensor_8(0x5314, 0x01);
	write_cmos_sensor_8(0x5315, 0x20);
	write_cmos_sensor_8(0x5316, 0x08);
	write_cmos_sensor_8(0x5317, 0x08);
	write_cmos_sensor_8(0x5318, 0x10);
	write_cmos_sensor_8(0x5319, 0x88);
	write_cmos_sensor_8(0x531a, 0x88);
	write_cmos_sensor_8(0x531b, 0xa9);
	write_cmos_sensor_8(0x531c, 0xaa);
	write_cmos_sensor_8(0x531d, 0x0a);
	write_cmos_sensor_8(0x5405, 0x02);
	write_cmos_sensor_8(0x5406, 0x67);
	write_cmos_sensor_8(0x5407, 0x01);
	write_cmos_sensor_8(0x5408, 0x4a);
	write_cmos_sensor_8(0x0100, 0x00);	
	pr_debug("sensor_init() end\n");
}	/*	sensor_init  */

static void preview_setting(void)
{
	/*Preview 2320*1744 30fps 24M MCLK 4lane 1488Mbps/lane*/
	/*preview 30.01fps*/
	pr_debug("preview_setting() E\n");
	write_cmos_sensor_8(0x0303, 0x01);
	write_cmos_sensor_8(0x3501, 0x40);
	write_cmos_sensor_8(0x3662, 0x10);
	write_cmos_sensor_8(0x3714, 0x28);
	write_cmos_sensor_8(0x3737, 0x08);
	write_cmos_sensor_8(0x3739, 0x20);
	write_cmos_sensor_8(0x37c2, 0x14);
	write_cmos_sensor_8(0x37d9, 0x0c);
	write_cmos_sensor_8(0x37e3, 0x08);
	write_cmos_sensor_8(0x37e4, 0x38);
	write_cmos_sensor_8(0x37e6, 0x08);
	write_cmos_sensor_8(0x3801, 0x00);
	write_cmos_sensor_8(0x3802, 0x00);
	write_cmos_sensor_8(0x3803, 0x08);
	write_cmos_sensor_8(0x3805, 0x9f);
	write_cmos_sensor_8(0x3806, 0x0c);
	write_cmos_sensor_8(0x3807, 0x4f);
	write_cmos_sensor_8(0x3808, 0x08);
	write_cmos_sensor_8(0x3809, 0x40);
	write_cmos_sensor_8(0x380a, 0x06);
	write_cmos_sensor_8(0x380b, 0x20);
	write_cmos_sensor_8(0x380c, 0x04);
	write_cmos_sensor_8(0x380d, 0x62);
	write_cmos_sensor_8(0x380e, 0x0c);
	write_cmos_sensor_8(0x380f, 0x8e);
	write_cmos_sensor_8(0x3810, 0x00);
	write_cmos_sensor_8(0x3811, 0x08);
	write_cmos_sensor_8(0x3812, 0x00);
	write_cmos_sensor_8(0x3813, 0x02);
	write_cmos_sensor_8(0x3814, 0x03);
	write_cmos_sensor_8(0x3816, 0x03);
	write_cmos_sensor_8(0x3820, 0xab);
	write_cmos_sensor_8(0x3826, 0x04);
	write_cmos_sensor_8(0x3827, 0x90);
	write_cmos_sensor_8(0x3829, 0x07);
	write_cmos_sensor_8(0x4009, 0x0d);
	write_cmos_sensor_8(0x4050, 0x04);
	write_cmos_sensor_8(0x4051, 0x0b);
	write_cmos_sensor_8(0x4837, 0x1c);
	write_cmos_sensor_8(0x4902, 0x01);	
	pr_debug("preview_setting() end\n");
}	/*	preview_setting  */


static void capture_setting(kal_uint16 currefps)
{
	pr_debug("capture_setting() E! currefps:%d\n", currefps);
	
	write_cmos_sensor_8(0x0303, 0x00);
	write_cmos_sensor_8(0x3501, 0x80);
	write_cmos_sensor_8(0x3662, 0x12);
	write_cmos_sensor_8(0x3714, 0x24);
	write_cmos_sensor_8(0x3737, 0x04);
	write_cmos_sensor_8(0x3739, 0x12);
	write_cmos_sensor_8(0x37c2, 0x04);
	write_cmos_sensor_8(0x37d9, 0x0c);
	write_cmos_sensor_8(0x37e3, 0x04);
	write_cmos_sensor_8(0x37e4, 0x2A);
	write_cmos_sensor_8(0x37e6, 0x04);
	write_cmos_sensor_8(0x3801, 0x00);
	write_cmos_sensor_8(0x3802, 0x00);
	write_cmos_sensor_8(0x3803, 0x08);
	write_cmos_sensor_8(0x3805, 0x9f);
	write_cmos_sensor_8(0x3806, 0x0c);
	write_cmos_sensor_8(0x3807, 0x57);
	write_cmos_sensor_8(0x3808, 0x10);
	write_cmos_sensor_8(0x3809, 0x80);
	write_cmos_sensor_8(0x380a, 0x0c);
	write_cmos_sensor_8(0x380b, 0x40);
	write_cmos_sensor_8(0x380c, 0x04);
	write_cmos_sensor_8(0x380d, 0x62);
	write_cmos_sensor_8(0x380e, 0x0C);
	write_cmos_sensor_8(0x380f, 0xa8);
	write_cmos_sensor_8(0x3810, 0x00);
	write_cmos_sensor_8(0x3811, 0x10);
	write_cmos_sensor_8(0x3812, 0x00);
	write_cmos_sensor_8(0x3813, 0x08);
	write_cmos_sensor_8(0x3814, 0x01);
	write_cmos_sensor_8(0x3816, 0x01);
	write_cmos_sensor_8(0x3820, 0xa8);
	write_cmos_sensor_8(0x3826, 0x11);
	write_cmos_sensor_8(0x3827, 0x1c);
	write_cmos_sensor_8(0x3829, 0x03);
	write_cmos_sensor_8(0x4009, 0x0f);
	write_cmos_sensor_8(0x4050, 0x04);
	write_cmos_sensor_8(0x4051, 0x0b);
	write_cmos_sensor_8(0x4837, 0x0e);
	write_cmos_sensor_8(0x4902, 0x01);
}


static void normal_video_setting(kal_uint16 currefps)
{
	pr_debug("normal_video_setting() E! currefps:%d\n", currefps);
	
	preview_setting();
	pr_debug("normal_video_setting() end! currefps:%d\n", currefps);
}

static void hs_video_setting(void)
{
	pr_debug("hs_video_setting() E\n");
	
	write_cmos_sensor_8(0x0303, 0x01);
	write_cmos_sensor_8(0x3501, 0x20);
	write_cmos_sensor_8(0x3662, 0x08);
	write_cmos_sensor_8(0x3714, 0x30);
	write_cmos_sensor_8(0x3737, 0x08);
	write_cmos_sensor_8(0x3739, 0x20);
	write_cmos_sensor_8(0x37c2, 0x2c);
	write_cmos_sensor_8(0x37d9, 0x06);
	write_cmos_sensor_8(0x37e3, 0x08);
	write_cmos_sensor_8(0x37e4, 0x36);
	write_cmos_sensor_8(0x37e6, 0x08);
	write_cmos_sensor_8(0x3801, 0x40);
	write_cmos_sensor_8(0x3802, 0x00);
	write_cmos_sensor_8(0x3803, 0x40);
	write_cmos_sensor_8(0x3805, 0x5f);
	write_cmos_sensor_8(0x3806, 0x0c);
	write_cmos_sensor_8(0x3807, 0x5f);
	write_cmos_sensor_8(0x3808, 0x04);
	write_cmos_sensor_8(0x3809, 0x00);
	write_cmos_sensor_8(0x380a, 0x03);
	write_cmos_sensor_8(0x380b, 0x00);
	write_cmos_sensor_8(0x380c, 0x04);
	write_cmos_sensor_8(0x380d, 0x62);
	write_cmos_sensor_8(0x380e, 0x03);
	write_cmos_sensor_8(0x380f, 0x6a);
	write_cmos_sensor_8(0x3808, 0x04);
	write_cmos_sensor_8(0x3809, 0x00);
	write_cmos_sensor_8(0x380a, 0x03);
	write_cmos_sensor_8(0x380b, 0x00);
	write_cmos_sensor_8(0x3810, 0x00);
	write_cmos_sensor_8(0x3811, 0x04);
	write_cmos_sensor_8(0x3812, 0x00);
	write_cmos_sensor_8(0x3813, 0x04);
	write_cmos_sensor_8(0x3814, 0x07);
	write_cmos_sensor_8(0x3816, 0x07);
	write_cmos_sensor_8(0x3820, 0xac);
	write_cmos_sensor_8(0x3826, 0x04);
	write_cmos_sensor_8(0x3827, 0x48);
	write_cmos_sensor_8(0x3829, 0x03);
	write_cmos_sensor_8(0x4009, 0x05);
	write_cmos_sensor_8(0x4050, 0x02);
	write_cmos_sensor_8(0x4051, 0x05);
	write_cmos_sensor_8(0x4837, 0x1c);
	write_cmos_sensor_8(0x4902, 0x02);
}


static void slim_video_setting(void)
{
	pr_debug("slim_video_setting() E\n");
	write_cmos_sensor_8(0x0303, 0x01);
	write_cmos_sensor_8(0x3501, 0x20);
	write_cmos_sensor_8(0x3662, 0x08);
	write_cmos_sensor_8(0x3714, 0x30);
	write_cmos_sensor_8(0x3737, 0x08);
	write_cmos_sensor_8(0x3739, 0x20);
	write_cmos_sensor_8(0x37c2, 0x2c);
	write_cmos_sensor_8(0x37d9, 0x06);
	write_cmos_sensor_8(0x37e3, 0x08);
	write_cmos_sensor_8(0x37e4, 0x36);
	write_cmos_sensor_8(0x37e6, 0x08);
	write_cmos_sensor_8(0x3801, 0x40);
	write_cmos_sensor_8(0x3802, 0x00);
	write_cmos_sensor_8(0x3803, 0x40);
	write_cmos_sensor_8(0x3805, 0x5f);
	write_cmos_sensor_8(0x3806, 0x0c);
	write_cmos_sensor_8(0x3807, 0x5f);
	write_cmos_sensor_8(0x3808, 0x04);
	write_cmos_sensor_8(0x3809, 0x00);
	write_cmos_sensor_8(0x380a, 0x03);
	write_cmos_sensor_8(0x380b, 0x00);
	write_cmos_sensor_8(0x380c, 0x04);
	write_cmos_sensor_8(0x380d, 0x62);
	write_cmos_sensor_8(0x380e, 0x0c);
	write_cmos_sensor_8(0x380f, 0x90);
	write_cmos_sensor_8(0x3810, 0x00);
	write_cmos_sensor_8(0x3811, 0x04);
	write_cmos_sensor_8(0x3812, 0x00);
	write_cmos_sensor_8(0x3813, 0x04);
	write_cmos_sensor_8(0x3814, 0x07);
	write_cmos_sensor_8(0x3816, 0x07);
	write_cmos_sensor_8(0x3820, 0xac);
	write_cmos_sensor_8(0x3826, 0x04);
	write_cmos_sensor_8(0x3827, 0x48);
	write_cmos_sensor_8(0x3829, 0x03);
	write_cmos_sensor_8(0x4009, 0x05);
	write_cmos_sensor_8(0x4050, 0x02);
	write_cmos_sensor_8(0x4051, 0x05);
	write_cmos_sensor_8(0x4837, 0x1c);
	write_cmos_sensor_8(0x4902, 0x02);
		
	pr_debug("slim_video_setting() end\n");

}

static void custom1_setting(kal_uint16 currefps)
{
	capture_setting(currefps);
	write_cmos_sensor_8(0x3808, 0x0c);
	write_cmos_sensor_8(0x3809, 0xc0);
	write_cmos_sensor_8(0x380a, 0x09);
	write_cmos_sensor_8(0x380b, 0x90);
	write_cmos_sensor_8(0x380e, 0x0f);
	write_cmos_sensor_8(0x380f, 0xaa); //4010
	write_cmos_sensor_8(0x3810, 0x01);
	write_cmos_sensor_8(0x3811, 0xF0);
	write_cmos_sensor_8(0x3812, 0x01);
	write_cmos_sensor_8(0x3813, 0x68);
	pr_debug("custom1_setting() end! \n");
}

static void custom2_setting(kal_uint16 currefps)
{
	capture_setting(currefps);
	write_cmos_sensor_8(0x3808, 0x0c);
	write_cmos_sensor_8(0x3809, 0xc0);
	write_cmos_sensor_8(0x380a, 0x09);
	write_cmos_sensor_8(0x380b, 0x90);
	write_cmos_sensor_8(0x380e, 0x12);
	write_cmos_sensor_8(0x380f, 0xcc); //4812
	write_cmos_sensor_8(0x3810, 0x01);
	write_cmos_sensor_8(0x3811, 0xF0);
	write_cmos_sensor_8(0x3812, 0x01);
	write_cmos_sensor_8(0x3813, 0x68);
	pr_debug("custom2_setting() end! \n");
}

static kal_uint32 return_sensor_id(void)
{
    return ( (read_cmos_sensor_8(0x300a) << 16) | (read_cmos_sensor_8(0x300b) << 8) | read_cmos_sensor_8(0x300c) + 2);
}

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 * we should detect the module used i2c address
	 */
	int I2C_BUS = -1 ;
	I2C_BUS = i2c_adapter_id(pgi2c_cfg_legacy->pinst->pi2c_client->adapter);
	pr_debug(" I2C_BUS = %d\n",I2C_BUS);
	if(I2C_BUS != 2){	
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
				*sensor_id = return_sensor_id();
				if (*sensor_id == imgsensor_info.sensor_id) {
				pr_info("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				/*vivo hope add for AT cammand start*/
				if (is_atboot == 1) {
					pr_info("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
					pr_info("AT mode skip now return\n");
					return ERROR_NONE;
				}
				/*vivo hope add for AT cammand end*/
				/*vivo hope add for CameraEM otp errorcode*/
				pr_debug("add:start read eeprom  - when_power_on  = %d\n",vivo_otp_read_when_power_on);
				vivo_otp_read_when_power_on = ov13855pd1901_vivo_otp_read();
				pr_debug("add:end read eeprom - when_power_on = %d\n",vivo_otp_read_when_power_on);
			    /*vivo hope add end*/
				if (ov13855pd1901_flag == 1){
					pr_debug("i2c write id: 0x%x, sensor id: 0x%x, ov13855pd1901_flag = %d\n", imgsensor.i2c_write_id,*sensor_id, ov13855pd1901_flag);
			return ERROR_NONE;
				}
			}
			pr_debug("Read sensor id fail, id: 0x%x, ov13855pd1901_flag =%d\n",
					imgsensor.i2c_write_id, ov13855pd1901_flag);
				retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id || ov13855pd1901_flag != 1) {
	/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF*/
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

    pr_debug("PLATFORM:MT6797,MIPI 4LANE +++++ ++++ \n");
    pr_debug("preview 2112*1568@30fps,1080Mbps/lane; video 1024*768@120fps,864Mbps/lane; capture 4224*3136@30fps,1080Mbps/lane\n");

	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 * we should detect the module used i2c address
	 */

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			pr_debug("Read sensor id fail: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	pr_debug("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *  *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate/10);

		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	 capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	slim_video	 */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom1.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use custom1's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom1.max_framerate/10);

		imgsensor.pclk = imgsensor_info.custom1.pclk;
		imgsensor.line_length = imgsensor_info.custom1.linelength;
		imgsensor.frame_length = imgsensor_info.custom1.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom1_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom2.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use custom2's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom2.max_framerate/10);

		imgsensor.pclk = imgsensor_info.custom1.pclk;
		imgsensor.line_length = imgsensor_info.custom2.linelength;
		imgsensor.frame_length = imgsensor_info.custom2.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom2_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32 get_resolution(
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT(*sensor_resolution))
{
	pr_debug("E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width = 
		imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height =
		imgsensor_info.custom1.grabwindow_height;
		
	sensor_resolution->SensorCustom2Width = 
		imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height =
		imgsensor_info.custom2.grabwindow_height;

	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
				  MSDK_SENSOR_INFO_STRUCT *sensor_info,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	/* inverse with datasheet*/
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;

	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
		
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;

	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 1; /*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode*/
	/* 0: NO PDAF, 1: PDAF Raw Data mode,
	 * 2:PDAF VC mode(Full),
	 * 3:PDAF VC mode(Binning)
	 */
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x*/
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x*/
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

	    sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;
    case MSDK_SCENARIO_ID_CUSTOM1:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

			break;
    case MSDK_SCENARIO_ID_CUSTOM2:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;

			break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/* get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;
    case MSDK_SCENARIO_ID_CUSTOM1:
			custom1(image_window, sensor_config_data);
			break;
    case MSDK_SCENARIO_ID_CUSTOM2:
			custom2(image_window, sensor_config_data);
			break;
	default:
			pr_debug("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	pr_debug("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate*/
	if (framerate == 0)
		/* Dynamic frame rate*/
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	pr_debug("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /*enable auto flicker*/
		imgsensor.autoflicker_en = KAL_TRUE;
	else /*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	pr_debug("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk /
			framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.pre.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.pre.framelength);
		else
			imgsensor.dummy_line = 0;
		imgsensor.frame_length =
		  imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;

		frame_length = imgsensor_info.normal_video.pclk /
			framerate * 10 / imgsensor_info.normal_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.normal_video.framelength)
			imgsensor.dummy_line =
		      (frame_length - imgsensor_info.normal_video.framelength);

		else
			imgsensor.dummy_line = 0;
		imgsensor.frame_length =
		 imgsensor_info.normal_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if (imgsensor.current_fps !=
				imgsensor_info.cap.max_framerate)
				pr_debug(
					"current_fps %d is not support, so use cap's setting: %d fps!\n",
					framerate,
					imgsensor_info.cap.max_framerate/10);

				frame_length = imgsensor_info.cap.pclk /
				 framerate * 10 / imgsensor_info.cap.linelength;
				spin_lock(&imgsensor_drv_lock);

			if (frame_length > imgsensor_info.cap.framelength)
				imgsensor.dummy_line =
				  frame_length - imgsensor_info.cap.framelength;
			else
				imgsensor.dummy_line = 0;

			imgsensor.frame_length =
			 imgsensor_info.cap.framelength + imgsensor.dummy_line;

			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk /
			framerate * 10 / imgsensor_info.hs_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.hs_video.framelength)
			imgsensor.dummy_line =
		  (frame_length - imgsensor_info.hs_video.framelength);
		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		    imgsensor_info.hs_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk /
			framerate * 10 / imgsensor_info.slim_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.slim_video.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.slim_video.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.slim_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk /
			framerate * 10 / imgsensor_info.custom1.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom1.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.custom1.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.custom1.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk /
			framerate * 10 / imgsensor_info.custom1.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom2.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.custom2.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.custom2.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default:  /*coding with  preview scenario by default*/

		frame_length = imgsensor_info.pre.pclk /
			framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.pre.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.pre.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		  imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		pr_debug(
		    "error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	pr_debug("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
	case MSDK_SCENARIO_ID_CUSTOM1:
			*framerate = imgsensor_info.custom1.max_framerate;	
			break;
	case MSDK_SCENARIO_ID_CUSTOM2:
			*framerate = imgsensor_info.custom2.max_framerate;	
			break;
	default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	pr_debug("enable: %d\n", enable);

	if (enable) {
        write_cmos_sensor_8(0x4503, 0x80);
	} else {
        write_cmos_sensor_8(0x4503, 0x00);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	/*struct SENSOR_VC_INFO_STRUCT *pvcinfo;*/
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;


	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	pr_debug("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len = 4;
			break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len = 4;
			break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
			break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	    /*night_mode((BOOL) *feature_data); no need to implement this mode*/
			break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
			break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
	case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor_8(
			    sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
	case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData =
				read_cmos_sensor_8(sensor_reg_data->RegAddr);
			break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or
		 * just return LENS_DRIVER_ID_DO_NOT_CARE
		 */

		/* if EEPROM does not exist in camera module.*/
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode(
			(BOOL)*feature_data_16, *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
		  (enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;

		/*for factory mode auto testing*/
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len = 4;
			break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		spin_lock(&imgsensor_drv_lock);
			imgsensor.current_fps = (UINT32)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
			break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			*feature_data_32);

		wininfo =
	    (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[3],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[4],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[5],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[6],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		/*pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1), (UINT16)*(feature_data+2));*/

		/* ihdr_write_shutter_gain((UINT16)*feature_data,
		 * (UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
		 */
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		pr_debug("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		/* ihdr_write_shutter(
		 * (UINT16)*feature_data,(UINT16)*(feature_data+1));
		 */
		break;
		/******************** PDAF START >>> *********/
	case SENSOR_FEATURE_GET_PDAF_INFO:
			pr_debug("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
			PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG: /*full*/
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
		break;
		}
		break;

		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			pr_debug("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
			//PDAF capacity enable or not, 2p8 only full size support PDAF
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CUSTOM1:
				case MSDK_SCENARIO_ID_CUSTOM2:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;// xxx is not supported
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
			break;
#if 0	/*hope modify*/	
	case SENSOR_FEATURE_GET_PDAF_DATA:	/*get cal data from eeprom*/
			pr_debug("SENSOR_FEATURE_GET_PDAF_DATA\n");
			pr_debug("SENSOR_FEATURE_GET_PDAF_DATA success\n");
			break;
	
	case SENSOR_FEATURE_SET_PDAF:
			pr_debug("PDAF mode :%d\n", imgsensor.pdaf_mode);
			break;
#endif			
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:/*lzl*/
			set_shutter_frame_length((UINT16)*feature_data,
					(UINT16)*(feature_data+1));
			break;
        case SENSOR_FEATURE_GET_CUSTOM_INFO:
            switch (*feature_data) {
                case 0:    //info type: otp state
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = OV13855PD1901_OTP_ERROR_CODE;//otp_state
                    break;
            }
            break;
	/******************** STREAMING RESUME/SUSPEND *********/
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom1.mipi_pixel_rate;
			break;
    	case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;

	default:
			break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV13855PD1901_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
