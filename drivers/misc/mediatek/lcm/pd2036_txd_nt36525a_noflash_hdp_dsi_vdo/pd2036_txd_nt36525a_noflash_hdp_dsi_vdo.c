/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#include "mtkfb.h"


#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>

#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>

#ifndef CONFIG_FPGA_EARLY_PORTING
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#endif

#endif
#endif

static const unsigned int BL_MIN_LEVEL = 20;
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))



#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define lcm_vddi_setting(cmd) \
			lcm_util.lcm_vddi_setting(cmd)
#define lcm_reset_setting(cmd) \
			lcm_util.lcm_reset_setting(cmd)
#define lcm_enp_setting(cmd) \
			lcm_util.lcm_enp_setting(cmd)
#define lcm_enn_setting(cmd) \
			lcm_util.lcm_enn_setting(cmd)
#define lcm_bkg_setting(cmd) \
			lcm_util.lcm_bkg_setting(cmd)
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>

/*****************************************************************************
 * Define
 *****************************************************************************/
extern unsigned int panel_reset_state;
extern unsigned int panel_off_state;
extern unsigned int phone_shutdown_state;
extern unsigned int is_atboot;
extern unsigned int pre_bklt_level;

#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE	0
#define FRAME_WIDTH		(720)
#define FRAME_HEIGHT		(1600)
#define LCM_PHYSICAL_WIDTH		(67932)
#define LCM_PHYSICAL_HEIGHT		(150960)


#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
extern unsigned int rf_hw_id;
#define CONFIG_VIVO_LCM_PD1732DF_CONTROL
#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
extern int vivo_lcm_cabc_backlight;
static unsigned int dimming_enable;
extern unsigned int lcm_id_version;
extern unsigned int dimming_for_camera;
extern void lcm_bias_set_avdd_n_avee(int value);
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 80, {} },
};

#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
static struct LCM_setting_table init_setting_12bit[] = {

	{0xFF,1,{0x20}},
	{0xFB,1,{0x01}},
	{0x01,1,{0x55}},
	{0x03,1,{0x55}},
	{0x05,1,{0x99}},
	{0x07,1,{0x5F}},
	{0x08,1,{0xDF}},
	{0x0E,1,{0x7D}},
	{0x0F,1,{0x7D}},
	{0x94,1,{0x40}},
	{0x95,1,{0xF5}},
	{0x96,1,{0x1D}},
	{0x69,1,{0x91}},
	{0x1F,1,{0x02}},
	{0xB0,16,{0x00,0x36,0x00,0x44,0x00,0x55,0x00,0x68,0x00,0x78,0x00,0x87,0x00,0x98,0x00,0xA4}},
	{0xB1,16,{0x00,0xB0,0x00,0xDD,0x01,0x06,0x01,0x41,0x01,0x71,0x01,0xC0,0x02,0x01,0x02,0x03}},
	{0xB2,16,{0x02,0x42,0x02,0x88,0x02,0xB4,0x02,0xEE,0x03,0x14,0x03,0x42,0x03,0x51,0x03,0x61}},
	{0xB3,12,{0x03,0x74,0x03,0x8A,0x03,0xA6,0x03,0xC4,0x03,0xDB,0x03,0xDF}},
	{0xB4,16,{0x00,0x08,0x00,0x14,0x00,0x2D,0x00,0x42,0x00,0x57,0x00,0x68,0x00,0x79,0x00,0x89}},
	{0xB5,16,{0x00,0x97,0x00,0xC9,0x00,0xF1,0x01,0x33,0x01,0x66,0x01,0xB8,0x01,0xFB,0x01,0xFD}},
	{0xB6,16,{0x02,0x3C,0x02,0x83,0x02,0xB0,0x02,0xEA,0x03,0x10,0x03,0x3F,0x03,0x4E,0x03,0x5E}},
	{0xB7,12,{0x03,0x71,0x03,0x87,0x03,0x99,0x03,0xB8,0x03,0xDB,0x03,0xDF}},
	{0xB8,16,{0x00,0x1A,0x00,0x27,0x00,0x3F,0x00,0x55,0x00,0x67,0x00,0x79,0x00,0x88,0x00,0x98}},
	{0xB9,16,{0x00,0xA6,0x00,0xD6,0x00,0xFD,0x01,0x3D,0x01,0x6E,0x01,0xBE,0x01,0xFF,0x02,0x01}},
	{0xBA,16,{0x02,0x3F,0x02,0x86,0x02,0xB3,0x02,0xF0,0x03,0x18,0x03,0x3E,0x03,0x4E,0x03,0x5E}},
	{0xBB,12,{0x03,0x70,0x03,0x84,0x03,0x9F,0x03,0xBD,0x03,0xD9,0x03,0xDF}},
	{0xFF,1,{0x21}},
	{0xFB,1,{0x01}},
	{0xB0,16,{0x00,0x2E,0x00,0x3C,0x00,0x4D,0x00,0x60,0x00,0x70,0x00,0x7F,0x00,0x90,0x00,0x9C}},
	{0xB1,16,{0x00,0xA8,0x00,0xD5,0x00,0xFE,0x01,0x39,0x01,0x69,0x01,0xB8,0x01,0xF9,0x01,0xFB}},
	{0xB2,16,{0x02,0x3A,0x02,0x80,0x02,0xAC,0x02,0xE6,0x03,0x0C,0x03,0x3A,0x03,0x49,0x03,0x59}},
	{0xB3,12,{0x03,0x6C,0x03,0x82,0x03,0x9E,0x03,0xBC,0x03,0xD3,0x03,0xD7}},
	{0xB4,16,{0x00,0x00,0x00,0x0C,0x00,0x25,0x00,0x3A,0x00,0x4F,0x00,0x60,0x00,0x71,0x00,0x81}},
	{0xB5,16,{0x00,0x8F,0x00,0xC1,0x00,0xE9,0x01,0x2B,0x01,0x5E,0x01,0xB0,0x01,0xF3,0x01,0xF5}},
	{0xB6,16,{0x02,0x34,0x02,0x7B,0x02,0xA8,0x02,0xE2,0x03,0x08,0x03,0x37,0x03,0x46,0x03,0x56}},
	{0xB7,12,{0x03,0x69,0x03,0x7F,0x03,0x91,0x03,0xB0,0x03,0xD3,0x03,0xD7}},
	{0xB8,16,{0x00,0x12,0x00,0x1F,0x00,0x37,0x00,0x4D,0x00,0x5F,0x00,0x71,0x00,0x80,0x00,0x90}},
	{0xB9,16,{0x00,0x9E,0x00,0xCE,0x00,0xF5,0x01,0x35,0x01,0x66,0x01,0xB6,0x01,0xF7,0x01,0xF9}},
	{0xBA,16,{0x02,0x37,0x02,0x7E,0x02,0xAB,0x02,0xE8,0x03,0x10,0x03,0x36,0x03,0x46,0x03,0x56}},
	{0xBB,12,{0x03,0x68,0x03,0x7C,0x03,0x97,0x03,0xB5,0x03,0xD1,0x03,0xD7}},

	{0xFF,1,{0x22}},
	{0xFB,1,{0x01}},
	{0x1A,1,{0x00}},
	{0x53,1,{0x00}},
	{0x56,1,{0x77}},
	{0x58,1,{0x10}},
	{0x59,1,{0x10}},
	{0x5E,1,{0x10}},
	{0x5F,1,{0x10}},
	{0x68,1,{0x01}},
	{0x65,1,{0x80}},
	{0x66,1,{0x80}},
	{0x67,1,{0x80}},
	{0x6A,1,{0x00}},
        
	{0xFF,1,{0x23}},
	{0xFB,1,{0x01}},
	{0x00,1,{0x80}},//CABC 12BIT SETTING
	{0x04,1,{0x05}},
	{0x05,1,{0x2D}},
	{0x06,1,{0x01}},
	{0x07,1,{0x00}},
	{0x08,1,{0x01}},
	{0x12,1,{0xB4}},
	{0x15,1,{0xE9}},
	{0x16,1,{0x0B}},
	{0x30,1,{0xFF}},
	{0x31,1,{0xFE}},
	{0x32,1,{0xEB}},
	{0x33,1,{0xE5}},
	{0x34,1,{0xDD}},
	{0x35,1,{0xDA}},
	{0x36,1,{0xD5}},
	{0x37,1,{0xD0}},
	{0x38,1,{0xCE}},
	{0x39,1,{0xCD}},
	{0x3A,1,{0xCD}},
	{0x3B,1,{0xCD}},
	{0x3D,1,{0xCB}},
	{0x3F,1,{0xCB}},
	{0x40,1,{0xC6}},
	{0x41,1,{0xBF}},
	{0x45,1,{0xFF}},
	{0x46,1,{0xF0}},
	{0x47,1,{0xE8}},
	{0x48,1,{0xCE}},
	{0x49,1,{0xBC}},
	{0x4A,1,{0xB8}},
	{0x4B,1,{0xB5}},
	{0x4C,1,{0xB0}},
	{0x4D,1,{0xA8}},
	{0x4E,1,{0xA0}},
	{0x4F,1,{0x9B}},
	{0x50,1,{0x98}},
	{0x51,1,{0x98}},
	{0x52,1,{0x88}},
	{0x53,1,{0x80}},
	{0x54,1,{0x7F}},
	
	{0xFF,1,{0x24}},
	{0xFB,1,{0x01}},
	{0x00,1,{0x00}},
	{0x01,1,{0x1E}},
	{0x02,1,{0x1D}},
	{0x03,1,{0x21}},
	{0x04,1,{0x04}},
	{0x05,1,{0x05}},
	{0x06,1,{0x1F}},
	{0x07,1,{0x20}},
	{0x08,1,{0x0C}},
	{0x09,1,{0x0D}},
	{0x0A,1,{0x0E}},
	{0x0B,1,{0x0F}},
	{0x0C,1,{0x00}},
	{0x0D,1,{0x00}},
	{0x0E,1,{0x00}},
	{0x0F,1,{0x00}},
	{0x10,1,{0x00}},
	{0x11,1,{0x00}},
	{0x12,1,{0x00}},
	{0x13,1,{0x00}},
	{0x14,1,{0x00}},
	{0x15,1,{0x00}},
	{0x16,1,{0x00}},
	{0x17,1,{0x1E}},
	{0x18,1,{0x1D}},
	{0x19,1,{0x21}},
	{0x1A,1,{0x04}},
	{0x1B,1,{0x05}},
	{0x1C,1,{0x1F}},
	{0x1D,1,{0x20}},
	{0x1E,1,{0x0C}},
	{0x1F,1,{0x0D}},
	{0x20,1,{0x0E}},
	{0x21,1,{0x0F}},
	{0x22,1,{0x00}},
	{0x23,1,{0x00}},
	{0x24,1,{0x00}},
	{0x25,1,{0x00}},
	{0x26,1,{0x00}},
	{0x27,1,{0x00}},
	{0x28,1,{0x00}},
	{0x29,1,{0x00}},
	{0x2A,1,{0x00}},
	{0x2B,1,{0x00}},
	{0x2F,1,{0x08}},
	{0x30,1,{0x06}},
	{0x31,1,{0x00}},
	{0x32,1,{0x00}},
	{0x33,1,{0x06}},
	{0x34,1,{0x08}},
	{0x35,1,{0x00}},
	{0x36,1,{0x00}},
	{0x37,1,{0x44}},
	{0x38,1,{0x44}},
	{0x39,1,{0x00}},
	{0x3A,1,{0x24}},
	{0x3B,1,{0x90}},
	{0x3D,1,{0x52}},
	{0x4D,1,{0x34}},
	{0x4E,1,{0x12}},
	{0x4F,1,{0x00}},
	{0x50,1,{0x00}},
	{0x51,1,{0x21}},
	{0x52,1,{0x43}},
	{0x53,1,{0x00}},
	{0x54,1,{0x00}},
	{0x55,1,{0x47}},
	{0x56,1,{0x44}},
	{0x58,1,{0x21}},
	{0x59,1,{0x00}},
	{0x5A,1,{0x24}},
	{0x5B,1,{0x90}},
	{0x5C,1,{0x88}},
	{0x5D,1,{0x08}},
	{0x5E,1,{0x04}},
	{0x5F,1,{0x00}},
	{0x60,1,{0x80}},
	{0x61,1,{0x90}},
	{0x64,1,{0x10}},
	{0x80,1,{0x0A}},
	{0x81,1,{0x06}},
	{0x82,1,{0x0A}},
	{0x83,1,{0x06}},
	{0x84,1,{0x80}},
	{0x85,1,{0x01}},
	{0x86,1,{0x04}},
	{0x91,1,{0x40}},
	{0x98,1,{0x80}},
	{0xAB,1,{0x00}},
	{0xAD,1,{0x00}},
	{0xAF,1,{0x04}},
	{0xB0,1,{0x05}},
	
	{0xFF,1,{0x25}},
	{0xFB,1,{0x01}},
	{0x17,1,{0x82}},
	{0x18,1,{0x06}},
	{0x19,1,{0x0F}},
	{0xC0,1,{0x07}},
	{0xC2,1,{0x84}},
	{0xC6,1,{0x92}},
	{0xCA,1,{0x1C}},
	{0xCC,1,{0x1C}},
	{0xCD,1,{0x00}},
	{0xCE,1,{0x00}},
	{0xD3,1,{0x11}},
	{0xD4,1,{0xCC}},
	{0xD5,1,{0x11}},
	{0xD6,1,{0x1C}},
	{0xD7,1,{0x11}},
	
	{0xFF,1,{0x26}},
	{0xFB,1,{0x01}},
	{0x00,1,{0xA0}},
	{0xFF,1,{0x27}},
	{0xFB,1,{0x01}},
	{0x13,1,{0x00}},
	{0x15,1,{0xB4}},
	{0x1F,1,{0x55}},
	{0x26,1,{0x0F}},
	{0xC0,1,{0x18}},
	{0xC1,1,{0xF2}},
	{0xC7,1,{0x03}},
	{0xC2,1,{0x00}},
	{0xC3,1,{0x00}},
	{0xC4,1,{0xF2}},
	{0xC5,1,{0x00}},
	{0xC6,1,{0x00}},
	
	{0xFF,1,{0x25}},
	{0xFB,1,{0x01}},
	{0x0A,1,{0x82}},
	{0x0B,1,{0x1F}},
	{0x0C,1,{0x01}},
	
	{0xFF,1,{0x24}},
	{0xFB,1,{0x01}},
	{0x92,1,{0xB1}},
	{0xB1,1,{0xAD}},
	{0x93,1,{0x0C}},
	{0x94,1,{0x0C}},
	
	{0xFF,1,{0xE0}},
	{0xFB,1,{0x01}},
	{0x9E,1,{0x00}},
	
	{0xFF,1,{0x10}},
	{0xFB,1,{0x01}},
	{0xBA,1,{0x02}},

	//Backlight setting
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x35, 1, {0x00} },
	{0x51, 2, {0x00, 0x00} },
	{0x68, 2, {0x03, 0x01} },/* set backlight and cabc diming */
	{0x53, 1, {0x24} },
    	{0x55, 1, {0x00} },
	//02为3lane，03为4lane
	{0xBA, 1, {0x02} }, 
	//Sleep out&Display on
	{0x29, 0, {} },
	//{REGFLAG_DELAY, 20, {} },
	{0x11, 0, {} },
	{REGFLAG_DELAY, 96, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table bl_level_12bit[] = {
	{0x51, 2, {0x0F, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_cmd_backlight_dimming_enable[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2C} },
};

static struct LCM_setting_table lcm_cmd_backlight_dimming_disable[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x24} },
};
#endif

static struct LCM_setting_table lcm_cmd_cabc_level1[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x01} },
};

static struct LCM_setting_table lcm_cmd_cabc_level2[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x02} },
};
static struct LCM_setting_table lcm_cmd_cabc_level3[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x02} },
};

static struct LCM_setting_table lcm_cmd_cabc_off[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2C} },
	{0x55, 1, {0x00} },
};

static void push_table(struct LCM_setting_table *table,
				unsigned int count,
				unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
				table[i].para_list, force_update);
		}
	}
}

static void cabc_push_table(void *cmdq, struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;
	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}


static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
	params->dsi.mode = SYNC_EVENT_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
#endif
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_THREE_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 250;
	params->dsi.vertical_frontporch = 12;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4; //14
	params->dsi.horizontal_backporch = 88;   //24
	params->dsi.horizontal_frontporch = 96;  //36
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.ssc_disable = 1;

	params->dsi.data_rate = 860;
	params->dsi.PLL_CLOCK = 430; //300

/*#ifdef CONFIG_MTK_LCM_MIPI_CLK_CONTROL*/
	if (rf_hw_id == 1) {
		params->dsi.horizontal_backporch = 60;
		params->dsi.horizontal_frontporch = 60;
		params->dsi.data_rate = 774;
		params->dsi.PLL_CLOCK = 387;
		LCM_INFO("%s PD1987 rf_hw_id PLL_CLOCK %d\n", __func__, params->dsi.PLL_CLOCK);
	}
/*#endif*/

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	if (is_atboot)
		params->dsi.esd_check_enable = 0;

	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
	params->dsi.HS_PRPR = 0x8;
	params->dsi.HS_TRAIL = 0x9;
	/*add for mipi_dyn_clk*/
	params->dsi.PLL_CLOCK_dyn = 424;
	params->dsi.horizontal_backporch_dyn = 76;

}


static void lcm_init_power(void)
{
	LCM_INFO("%s nt36525a -> enter power on begin, panel_reset_state=%d\n", __func__, panel_reset_state);
	if (panel_reset_state == 0) {
		panel_reset_state = 1;
		lcm_enp_setting(1);
		MDELAY(3);
		lcm_enn_setting(1);
		MDELAY(5);
	/***********************************
	KTD2151 is 1th manufacture, SM3109 is 2nd
	default voltage is 5v5, so we need to set again
	It will lead to a raising on the waveform.
	************************************/
		lcm_bias_set_avdd_n_avee(60);
		LCM_INFO("%s nt36525a -> enter power on end\n", __func__);
	}
}

static void lcm_suspend_power(void)
{
	if (phone_shutdown_state) {
		LCM_INFO("[LCM]lcm_suspend_power reset keep low level\n");
		lcm_reset_setting(0);
	}
	MDELAY(10);
	lcm_enn_setting(0);
	MDELAY(5);
	lcm_enp_setting(0);
	MDELAY(5);
	LCM_INFO("[LCM]lcm_suspend_power nt36525a- vddi keep high level\n");
	panel_reset_state = 0;// clear reset state
}

static void lcm_init(void)
{

	LCM_INFO("[LCM]nt36525a- init begin\n");
	lcm_reset_setting(1);
	MDELAY(3);
	lcm_reset_setting(0);
	MDELAY(3);
	lcm_reset_setting(1);
	MDELAY(3);
	lcm_reset_setting(0);
	MDELAY(3);
	lcm_reset_setting(1);
	/*****************************************************************
	lrz modify it to 15ms from 90ms.	90ms is used to ensure flash download initial code.
	But PD1987 use no-flash version nt36525, so 15ms is enough, NolenCheng confirmed 0304
	******************************************************************/
	MDELAY(15);
	panel_reset_state = 1; // set reset state
	panel_off_state = 0;
	push_table(init_setting_12bit, sizeof(init_setting_12bit) / sizeof(struct LCM_setting_table), 1);
	LCM_INFO("[LCM]nt36525a- init end\n");
}

static void lcm_suspend(void)
{
	push_table(lcm_suspend_setting,
		sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
		1);
	panel_off_state = 1;
	LCM_INFO("[LCM]nt36525- lcm_suspend display off\n");
}

static void lcm_resume(void)
{
	lcm_init();
}


static void lcm_reset_for_touch(void)
{
	lcm_reset_setting(0);
	MDELAY(2);
	lcm_reset_setting(1);
	MDELAY(12);
	panel_reset_state = 1;
}
static void lcm_update(unsigned int x, unsigned int y, unsigned int width,
	unsigned int height)
{
#if LCM_DSI_CMD_MODE
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
#endif
}


/* return FALSE: No need recovery */
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char buffer[3];
	int array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);

	if (buffer[0] != 0x9c) {
		LCM_INFO("[LCM][LCM ERROR] [0x53]=0x%02x\n", buffer[0]);
		return TRUE;
	}
	LCM_INFO("[LCM][LCM NORMAL] [0x53]=0x%02x\n", buffer[0]);
	return FALSE;
#else
	return FALSE;
#endif

}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];

	LCM_INFO("[LCM]ATA check size = 0x%x, 0x%x, 0x%x, 0x%x\n",
		x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	/* read id return two byte,version and id */
	data_array[0] = 0x00043700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x2A, read_buf, 4);

	if ((read_buf[0] == x0_MSB) && (read_buf[1] == x0_LSB)
	    && (read_buf[2] == x1_MSB) && (read_buf[3] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	x0 = 0;
	x1 = FRAME_WIDTH - 1;

	x0_MSB = ((x0 >> 8) & 0xFF);
	x0_LSB = (x0 & 0xFF);
	x1_MSB = ((x1 >> 8) & 0xFF);
	x1_LSB = (x1 & 0xFF);

	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	return ret;
#else
	return 0;
#endif
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	/*level =level*16;*/
	if ((level != 0) && (dimming_enable == 0) && (vivo_lcm_cabc_backlight != 0) && (dimming_for_camera == 1)) {
		cabc_push_table(handle, lcm_cmd_backlight_dimming_enable, sizeof(lcm_cmd_backlight_dimming_enable) / sizeof(struct LCM_setting_table), 1);
		dimming_enable = 1;
		LCM_INFO("LastBacklight = %d, dimming_enable=%d, high_bit=0x%x, low_bit=0x%x\n", vivo_lcm_cabc_backlight, dimming_enable, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);

	}
	if (level == 0) {
		cabc_push_table(handle, lcm_cmd_backlight_dimming_disable, sizeof(lcm_cmd_backlight_dimming_disable) / sizeof(struct LCM_setting_table), 1);
		dimming_enable = 0;
		LCM_INFO("LastBacklight = %d, dimming_enable=%d, high_bit=0x%x, low_bit=0x%x\n", vivo_lcm_cabc_backlight, dimming_enable, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
	}
	bl_level_12bit[0].para_list[0] = (unsigned char)((level>>8)&0xFF);
	bl_level_12bit[0].para_list[1] = (unsigned char)(level&0xFFFF);
	LCM_INFO("LastBacklight = %d, level=%d, high_bit=0x%x, low_bit=0x%x\n", vivo_lcm_cabc_backlight, level, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
	cabc_push_table(handle, bl_level_12bit, sizeof(bl_level_12bit) / sizeof(struct LCM_setting_table), 1);
	pre_bklt_level = level;
}

static unsigned int lcm_get_id(unsigned char *lcm_flag)
{
	if (lcm_id_version == 0x61)
		return 0x3E;
	else if (lcm_id_version == 0x62)
		return 0x51;
	else
		return 0x3A;
}

static void lcm_cabc_vivo_open(void *handle, unsigned char leveldimming, unsigned char levelsetting)
{
	/*add for lcm_diming*/
	if (leveldimming == 0x2d) {
		if ((levelsetting == 0) && (sizeof(lcm_cmd_backlight_dimming_disable) > 0)) {
			dimming_enable = 0;
			cabc_push_table(handle, lcm_cmd_backlight_dimming_disable, sizeof(lcm_cmd_backlight_dimming_disable) / sizeof(struct LCM_setting_table), 1);
		} else if ((levelsetting == 1) && (sizeof(lcm_cmd_backlight_dimming_enable) > 0)) {
			dimming_enable = 1;
			cabc_push_table(handle, lcm_cmd_backlight_dimming_enable, sizeof(lcm_cmd_backlight_dimming_enable) / sizeof(struct LCM_setting_table), 1);
		}
	/*add end*/
	} else if ((levelsetting == 0) && (sizeof(lcm_cmd_cabc_off)) > 0)
		cabc_push_table(handle, lcm_cmd_cabc_off, sizeof(lcm_cmd_cabc_off) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 1) && (sizeof(lcm_cmd_cabc_level1)) > 0)
		cabc_push_table(handle, lcm_cmd_cabc_level1, sizeof(lcm_cmd_cabc_level1) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 2) && (sizeof(lcm_cmd_cabc_level2)) > 0)
		cabc_push_table(handle, lcm_cmd_cabc_level2, sizeof(lcm_cmd_cabc_level2) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 3) && (sizeof(lcm_cmd_cabc_level3)) > 0)
		cabc_push_table(handle, lcm_cmd_cabc_level3, sizeof(lcm_cmd_cabc_level3) / sizeof(struct LCM_setting_table), 1);
      LCM_INFO("vincent=----lcm_cabc_vivo_open dim:%d,level:%d\n", leveldimming, levelsetting);
}

static void lcm_cabc_vivo_close(void *handle, unsigned char level)
{
	if (sizeof(lcm_cmd_cabc_off))
		cabc_push_table(handle, lcm_cmd_cabc_off, sizeof(lcm_cmd_cabc_off) / sizeof(struct LCM_setting_table), 1);
	 LCM_INFO("vincent=----lcm_cabc_vivo_colose level:%d\n", level);
}

struct LCM_DRIVER pd2036_txd_nt36525a_noflash_hdp_dsi_vdo_lcm_drv = {
	.name = "pd2036_txd_nt36525a_noflash_hdp_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_init_power,
	.suspend_power = lcm_suspend_power,
	.esd_check = lcm_esd_check,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = lcm_ata_check,
	.update = lcm_update,
	.get_id	    = lcm_get_id,
	.lcm_cabc_open  = lcm_cabc_vivo_open,
	.lcm_cabc_close = lcm_cabc_vivo_close,
	.lcm_reset = lcm_reset_for_touch,
};
