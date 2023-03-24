#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "../imgsensor_i2c.h"
#include "s5k4h7yxpd1901mipiraw_Sensor.h"

#define PFX "S5K4H7YXPD1901_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo cfx add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_SUB_OTP_DATA_SIZE 0x0F80 /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
/*#define AF_RANGE_GOT*/  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_SUB_EEPROM_WRITE_ID 0x20
#define VIVO_SUB_I2C_SPEED 400
#define VIVO_SUB_MAX_OFFSET 0x0F7F
#define VIVO_SUB_VENDOR_SUNNY 0x01
#define VIVO_SUB_VENDOR_TRULY 0x02
#define VIVO_SUB_VENDOR_QTECH 0x05
#define VIVO_SUB_VENDOR_OFILM 0x09
#define VIVO_SUB_VENDOR_LENS_ID 0x011
/*#define VIVO_SUB_VENDOR_VCM_ID 0x00*/
/*#define VIVO_SUB_VENDOR_DRIVERIC_ID 0x00*/
#define VIVO_SUB_VENDOR_PLATFORM_ID 0x01

static unsigned char vivo_sub_otp_data[VIVO_SUB_OTP_DATA_SIZE];
unsigned char vivo_sub_otp_data_s5k4h7yxpd1901[1886];
static unsigned const int ModuleInfo_group1_flag = 0x0000;
static unsigned const int ModuleInfo_group2_flag = 0x01+(0xA43 - 0xA04);
static unsigned int ModuleInfo_sub_addr;
static unsigned int ModuleInfo_sub_checksum_addr;
static unsigned const int Fuse_id_group1_flag = 0x0A0C - 0x0A04;  /*0x1929 - 0x0401*/
static unsigned const int Fuse_id_group2_flag = (0x0A0C - 0x0A04) +0x01+(0xA43 - 0xA04);  /*0x1929 - 0x0401*/
static unsigned int Fuse_id_sub_addr;
static unsigned int Fuse_id_sub_checksum_addr;
static unsigned const int Sn_group1_flag = 0x0A1A - 0x0A04;  /*0x1929 - 0x0401*/
static unsigned const int Sn_group2_flag = (0x0A1A - 0x0A04) +0x01+(0xA43 - 0xA04);  /*0x1929 - 0x0401*/
static unsigned int Sn_sub_addr;
static unsigned int Sn_sub_checksum_addr;
static unsigned const int Awb_group1_flag = 0x0A28 - 0x0A04;
static unsigned const int Awb_group2_flag = (0x0A28 - 0x0A04) +0x01+(0xA43 - 0xA04);
static unsigned int Awb_sub_addr;
static unsigned int Awb_sub_checksum_addr;
static unsigned const int Lsc_group1_flag = (0x0A04 - 0x0A04) +2*0x40;
static unsigned const int Lsc_group2_flag = (0x0A04 - 0x0A04) +32*0x40;
static unsigned int Lsc_sub_addr;
static unsigned int Lsc_sub_checksum_addr;


/*static unsigned const int Awb_low_group1_flag = 0x0A36 - 0x0A04;*/
/*static unsigned const int Awb_low_group2_flag = (0x0A36 - 0x0A04) +0x01+(0xA35 - 0xA04);*/
/*static unsigned int Awb_low_sub_addr;*/
/*static unsigned int Awb_low_sub_checksum_addr;*/





static int checksum;
otp_error_code_t S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
extern MUINT32  sn_inf_sub_s5k4h7yxpd1901[13];

static void write_cmos_sensor_16_8(kal_uint32 addr, kal_uint32 para)
{
  char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
  //	kdSetI2CSpeed(400); 
  iWriteRegI2C(pu_send_cmd, 3, VIVO_SUB_EEPROM_WRITE_ID);
}

static kal_uint16 read_cmos_sensor_16_8(kal_uint16 addr)
{
  kal_uint16 get_byte = 0;
  char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

  iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, VIVO_SUB_EEPROM_WRITE_ID);
  return get_byte;
}

static int s5k4h7yxpd1901_otp_read_setting_init_continuous_read(kal_uint16 addr)
{
	int i = 0, j = 0;
	write_cmos_sensor_16_8(0x0100, 0x01);
	mdelay(50);
	for(j = 0; j < 62; j++){
		write_cmos_sensor_16_8(0x0A02, (0x15 + j));
		write_cmos_sensor_16_8(0x0A00, 0x01);
		mdelay(3);//page == 64 bytes
		for (i = 0; i < 0x40; i++) {
			vivo_sub_otp_data[i +0x40*j] = read_cmos_sensor_16_8(0x0A04 +i);
		}
	}
	write_cmos_sensor_16_8(0x0A00, 0x04);	
	write_cmos_sensor_16_8(0x0A00, 0x00);
	return 0;
}

extern unsigned int is_atboot;/*guojunzheng add*/
int S5K4H7YXPD1901_otp_read(void)
{
	int i = 0 ;
	//int j = 0;
	int offset = 0x0401;
	int check_if_ModuleInfo_group_valid = 1;
	int check_if_AWB_group_valid = 1;
	int check_if_Lsc_group_valid = 1;
	int check_if_SN_group_valid = 1;
	int check_if_Fuse_id_group_valid = 1;
	int check_if_group_valid = 0;

	
	int R_unit = 0, B_unit = 0, G_unit = 0, R_golden = 0, B_golden = 0, G_golden = 0;
	/*int R_unit_low = 0, B_unit_low = 0, G_unit_low = 0, R_golden_low = 0, B_golden_low = 0, G_golden_low = 0;*/
	long long t1, t2, t3, t4, t5, t6, t, temp;
	S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
	
	/* This operation takes a long time, we need to skip it. guojunzheng add begin */
	if (is_atboot == 1) {
		LOG_INF("AT mode skip vivo_sub_otp_read\n");
		return 1;
	}
	/* guojunzheng add end */

	s5k4h7yxpd1901_otp_read_setting_init_continuous_read(offset);
	LOG_INF("read_s5k4h7yxpd1901_data\n");

	#if 0
	for(j = 0; j < 62; j++){
		for (i = 0 ; i < 0x40; i++) {
			LOG_INF("read_s5k4h7yxpd1901_data[%d][0x%x][%d] = 0x%x\n", (j + 21), (i + 0x0A04), (i +0x40*j), vivo_sub_otp_data[i +0x40*j]);
		}
	}
	#endif
	
	/*moduleinfo  */
	if (0x01 == vivo_sub_otp_data[ModuleInfo_group1_flag]) { /*Group 1*/
		ModuleInfo_sub_addr = 0x0A05 - 0x0A04;
		ModuleInfo_sub_checksum_addr = ModuleInfo_sub_addr + 6;
		LOG_INF("ModuleInfo Group 1 !!!\n");
	} else if (0x01 == vivo_sub_otp_data[ModuleInfo_group2_flag]) { /*Group 2*/
		ModuleInfo_sub_addr = (0x0A05 - 0x0A04) + 0x01+(0xA43 - 0xA04);
		ModuleInfo_sub_checksum_addr = ModuleInfo_sub_addr + 6;
		LOG_INF("ModuleInfo Group 2 !!!\n");
	} else {
		check_if_ModuleInfo_group_valid = 0;
		LOG_INF("ModuleInfo flag error!!!    ModuleInfo group1 flag =%d, group 2 =%d \n", vivo_sub_otp_data[ModuleInfo_group1_flag], vivo_sub_otp_data[ModuleInfo_group2_flag]);
	}

	/*awb 5100K  */
	if (0x01 == vivo_sub_otp_data[Awb_group1_flag]) { /*Group 1*/
		Awb_sub_addr = 0x0A29 - 0x0A04;
		Awb_sub_checksum_addr = Awb_sub_addr + 12;
		LOG_INF("AWB Group 1 !!!\n");
	} else if (0x01== vivo_sub_otp_data[Awb_group2_flag]) { /*Group 2*/
		Awb_sub_addr = (0x0A29 - 0x0A04)  + 0x01+(0xA43 - 0xA04) ;
		Awb_sub_checksum_addr = Awb_sub_addr + 12;
		LOG_INF("AWB Group 2 !!!\n");
	} else {
		check_if_AWB_group_valid = 0;
		LOG_INF("AWB flag error!!!    AWB group1 flag = %d, group2 flag = %d\n", vivo_sub_otp_data[Awb_group1_flag], vivo_sub_otp_data[Awb_group2_flag]);
	}
	#if 0
	/*awb 3100K  */
	if (0x01 == vivo_sub_otp_data[Awb_low_group1_flag]) { /*Group 1*/
		Awb_low_sub_addr = 0x0A37 - 0x0A04;
		Awb_low_sub_checksum_addr = Awb_low_sub_addr + 12;
		LOG_INF("AWB Group 1 !!!\n");
	} else if (0x01 == vivo_sub_otp_data[Awb_low_group2_flag]) { /*Group 2*/
		Awb_low_sub_addr = (0x0A37 - 0x0A04)  + 0x01+(0xA43 - 0xA04);
		Awb_low_sub_checksum_addr = Awb_low_sub_addr + 12;
		LOG_INF("AWB Group 2 !!!\n");
	} else {
		check_if_AWB_low_group_valid = 0;
		LOG_INF("AWB flag error!!!    AWB group1 flag  = %d, group2 flag  = %d \n", vivo_sub_otp_data[Awb_low_group1_flag], vivo_sub_otp_data[Awb_low_group2_flag]);
	}
	#endif
	
	/*fuse id  */
	if (0x01 == vivo_sub_otp_data[Fuse_id_group1_flag]) { /*Group 1*/
		Fuse_id_sub_addr = 0x0A0D - 0x0A04;
		Fuse_id_sub_checksum_addr = Fuse_id_sub_addr + 12;
		LOG_INF("Fuse id Group 1 !!!\n");
	} else if (0x01 == vivo_sub_otp_data[Fuse_id_group2_flag]) { /*Group 2*/
		Fuse_id_sub_addr = (0x0A0D - 0x0A04) + 0x01+(0xA43 - 0xA04);
		Fuse_id_sub_checksum_addr = Fuse_id_sub_addr + 12;
		LOG_INF("Fuse id Group 2 !!!\n");
	} else {
		check_if_Fuse_id_group_valid = 0;
		LOG_INF(" flag error!!!    Fuse id  group 1 flag = %d, group 2 flag = %d \n", vivo_sub_otp_data[Fuse_id_group1_flag], vivo_sub_otp_data[Fuse_id_group2_flag]);
	}
	
	/*sn  */
	if (0x01 == vivo_sub_otp_data[Sn_group1_flag]) { /*Group 1*/
		Sn_sub_addr = 0x0A1B - 0x0A04;
		Sn_sub_checksum_addr = Sn_sub_addr + 12;
		LOG_INF("Sn Group 1 !!!\n");
	} else if (0x01 == vivo_sub_otp_data[Sn_group2_flag]) { /*Group 2*/
		Sn_sub_addr = (0x0A1B - 0x0A04) + 0x01+(0xA43 - 0xA04);
		Sn_sub_checksum_addr = Sn_sub_addr + 12;
		LOG_INF("Sn Group 2 !!!\n");
	} else {
		check_if_SN_group_valid = 0;
		LOG_INF("Sn flag error!!!    Sn  group 1 flag = %d, group 2 flag = %d \n", vivo_sub_otp_data[Sn_group1_flag], vivo_sub_otp_data[Sn_group2_flag]);
	}

	/*lsc  */
	if (0x01 == vivo_sub_otp_data[Lsc_group1_flag]) { /*Group 1*/
		Lsc_sub_addr = (0x0A05 - 0x0A04) +2*0x40;
		Lsc_sub_checksum_addr = Lsc_sub_addr + 1868;
		LOG_INF("Lsc Group 1 !!!\n");
	} else if (0x01 == vivo_sub_otp_data[Lsc_group2_flag]) { /*Group 2*/
		Lsc_sub_addr = (0x0A05 - 0x0A04) +32*0x40;
		Lsc_sub_checksum_addr = Lsc_sub_addr + 1868;
		LOG_INF("Lsc Group 2 !!!\n");
	} else {
		check_if_Lsc_group_valid = 0;
		LOG_INF("Lsc flag error!!!    Lsc  group 1 flag = %d, group 2 flag = %d \n", vivo_sub_otp_data[Lsc_group1_flag], vivo_sub_otp_data[Lsc_group2_flag]);
	}
	
	LOG_INF("read_s5k4h7yxpd1901_data,moduleinfo_addr=%d,awb_addr= %d, Lsc_sub_addr = %d, sn = %d, fuse id addr = %d\n", ModuleInfo_sub_addr, Awb_sub_addr, Lsc_sub_addr, Sn_sub_addr, Fuse_id_sub_addr);

	check_if_group_valid =check_if_Fuse_id_group_valid &  check_if_Lsc_group_valid & check_if_AWB_group_valid & check_if_ModuleInfo_group_valid & check_if_SN_group_valid;
	if (check_if_group_valid == 0x01) { /****all the data is valid****/
		/****ModuleInfo****/
		if ((VIVO_SUB_VENDOR_TRULY != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x00])&&(VIVO_SUB_VENDOR_SUNNY != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x00])&&(VIVO_SUB_VENDOR_OFILM != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x00])&&( VIVO_SUB_VENDOR_QTECH != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x00]) ) {
			S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[cfx++]Module ID error!!!    otp_error_code:%d\n", S5K4H7YXPD1901_OTP_ERROR_CODE);
			return 0;
		} else if ((VIVO_SUB_VENDOR_LENS_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x05]) || (VIVO_SUB_VENDOR_PLATFORM_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x01]) /*|| (VIVO_SUB_VENDOR_DRIVERIC_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x07])*/) {
			S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("Platform ID or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code =%d\n", S5K4H7YXPD1901_OTP_ERROR_CODE);
			return 0;
		} /*else if ((0xff != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x08]) || (0x00 != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x09]) || (0x0b != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x0a]) || (0x01 != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x0b])) {
			S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[cfx++]: calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n", vivo_sub_otp_data[ModuleInfo_sub_addr + 0x08], vivo_sub_otp_data[ModuleInfo_sub_addr + 0x09], vivo_sub_otp_data[ModuleInfo_sub_addr + 0x0a], vivo_sub_otp_data[ModuleInfo_sub_addr + 0x0b]);
			return 0;
		}*/
		
		/****ModuleInfo_checksum****/
		checksum = 0;
		for (i = ModuleInfo_sub_addr; i < ModuleInfo_sub_checksum_addr; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[ModuleInfo_sub_checksum_addr] != checksum) {
			S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[cfx++]ModuleInfo_checksum error!!!   otp_error_code:%d\n", S5K4H7YXPD1901_OTP_ERROR_CODE);
			return 0;
			}
		/****AWB_checksum 5100K****/
		checksum = 0;
		for (i = Awb_sub_addr; i < Awb_sub_checksum_addr; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Awb_sub_checksum_addr] != checksum) {
			S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AWB_checksum error!!!   otp_error_code:%d\n", S5K4H7YXPD1901_OTP_ERROR_CODE);
			return 0;
		}
		#if 0
		/****AWB_checksum 3100K****/
		checksum = 0;
		for (i = Awb_low_sub_addr; i < Awb_low_sub_checksum_addr; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Awb_low_sub_checksum_addr] != checksum) {
			S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AWB_checksum error!!!   otp_error_code:%d\n", S5K4H7YXPD1901_OTP_ERROR_CODE);
			return 0;
		}
		#endif
		/****SN_checksum****/
		checksum = 0;
		for (i = Sn_sub_addr; i < Sn_sub_checksum_addr; i++) {
			/*LOG_INF("vivo_sub_otp_data[0x%x] = 0x%x\n", i, vivo_sub_otp_data[i]);*/
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Sn_sub_checksum_addr] != checksum) {
			S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n", S5K4H7YXPD1901_OTP_ERROR_CODE);
			return 0;
		}
		
		sn_inf_sub_s5k4h7yxpd1901[0] = 0x01;
		
		for (i = 0; i < 12; i++) {
			sn_inf_sub_s5k4h7yxpd1901[i +1] = (MUINT32)vivo_sub_otp_data[i + Sn_sub_addr];
			LOG_INF("sn_inf_sub_s5k4h7yxpd1901[%d] = 0x%x, vivo_sub_otp_data[0x%x] = 0x%x\n", i+1  , sn_inf_sub_s5k4h7yxpd1901[i+1],  i +Sn_sub_addr, vivo_sub_otp_data[i + Sn_sub_addr]);
		}

		
		/****Fuse id_checksum****/
		checksum = 0;
		for (i = Fuse_id_sub_addr; i < Fuse_id_sub_checksum_addr; i++) {
			/*LOG_INF("vivo_sub_otp_data[0x%x] = 0x%x\n", i, vivo_sub_otp_data[i]);*/
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Fuse_id_sub_checksum_addr] != checksum) {
			S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("Fuse id _checksum error!!!   otp_error_code:%d\n", S5K4H7YXPD1901_OTP_ERROR_CODE);
			return 0;
		}

		
		/****Lsc_checksum****/
		checksum = 0;
		for (i = Lsc_sub_addr; i < Lsc_sub_checksum_addr; i++) {
			/*LOG_INF("vivo_sub_otp_data[0x%x] = 0x%x\n", i, vivo_sub_otp_data[i]);*/
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Lsc_sub_checksum_addr] != checksum) {
			S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("Lsc _checksum error!!!   otp_error_code:%d\n", S5K4H7YXPD1901_OTP_ERROR_CODE);
			return 0;
		}
		
		/****check if awb out of range[high cct]****/
		R_unit = vivo_sub_otp_data[Awb_sub_addr];
		R_unit = (R_unit << 8) | (vivo_sub_otp_data[Awb_sub_addr +1]);
		B_unit = vivo_sub_otp_data[Awb_sub_addr+2];
		B_unit = (B_unit << 8) | (vivo_sub_otp_data[Awb_sub_addr+3]);
		G_unit = vivo_sub_otp_data[Awb_sub_addr+4];
		G_unit = (G_unit << 8) | (vivo_sub_otp_data[Awb_sub_addr+5]);

		R_golden = vivo_sub_otp_data[Awb_sub_addr+6];
		R_golden = (R_golden << 8) | (vivo_sub_otp_data[Awb_sub_addr+7]);
		B_golden = vivo_sub_otp_data[Awb_sub_addr+8];
		B_golden = (B_golden << 8) | (vivo_sub_otp_data[Awb_sub_addr+9]);
		G_golden = vivo_sub_otp_data[Awb_sub_addr+10];
		G_golden = (G_golden << 8) | (vivo_sub_otp_data[Awb_sub_addr+11]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("R_unit=%d, R_golden=%d, B_unit=%d, B_golden=%d, G_unit=%d, G_golden=%d\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
		t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
		t2 = R_golden*R_golden;
		t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
		t4 = G_golden*G_golden;
		t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
		t6 = B_golden*B_golden;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("[cfx++]AWB[high cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, S5K4H7YXPD1901_OTP_ERROR_CODE);
			return 0;
		}

		#if 0
		/****check if awb out of range[low cct]****/
		R_unit_low = vivo_sub_otp_data[Awb_low_sub_addr];
		R_unit_low = (R_unit_low << 8) | (vivo_sub_otp_data[Awb_low_sub_addr + 1]);
		B_unit_low = vivo_sub_otp_data[Awb_low_sub_addr+2];
		B_unit_low = (B_unit_low << 8) | (vivo_sub_otp_data[Awb_low_sub_addr+3]);
		G_unit_low = vivo_sub_otp_data[Awb_low_sub_addr+4];
		G_unit_low = (G_unit_low << 8) | (vivo_sub_otp_data[Awb_low_sub_addr+ 5]);

		R_golden_low = vivo_sub_otp_data[Awb_low_sub_addr+6];
		R_golden_low = (R_golden_low << 8) | (vivo_sub_otp_data[Awb_low_sub_addr+7]);
		B_golden_low = vivo_sub_otp_data[Awb_low_sub_addr+8];
		B_golden_low = (B_golden_low << 8) | (vivo_sub_otp_data[Awb_low_sub_addr+9]);
		G_golden_low = vivo_sub_otp_data[Awb_low_sub_addr+10];
		G_golden_low = (G_golden_low << 8) | (vivo_sub_otp_data[Awb_low_sub_addr+11]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("R_unit_low=%d, R_golden_low=%d, B_unit_low=%d, B_golden_low=%d, G_unit_low=%d, G_golden_low=%d\n", R_unit_low, R_golden_low, B_unit_low, B_golden_low, G_unit_low, G_golden_low);
		t1 = 1024*1024*(R_unit_low-R_golden_low)*(R_unit_low-R_golden_low);
		t2 = R_golden_low*R_golden_low;
		t3 = 1048576*(G_unit_low-G_golden_low)*(G_unit_low-G_golden_low);
		t4 = G_golden_low*G_golden_low;
		t5 = 1048576*(B_unit_low-B_golden_low)*(B_unit_low-B_golden_low);
		t6 = B_golden_low*B_golden_low;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("cfx_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("[cfx++]AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, S5K4H7YXPD1901_OTP_ERROR_CODE);
			return 0;
		}
		#endif
		#if 0
		/****LSC_checksum****/
		checksum = 0;
		for (i = Lsc_sub_addr; i < Lsc_sub_checksum_addr; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Lsc_sub_checksum_addr] != checksum) {
			S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("LSC_checksum error!!!   otp_error_code:%d\n", S5K4H7YXPD1901_OTP_ERROR_CODE);
			return 0;
		}
		#endif
		memcpy(vivo_sub_otp_data_s5k4h7yxpd1901, &vivo_sub_otp_data[ModuleInfo_sub_addr], 6);
		memcpy(&vivo_sub_otp_data_s5k4h7yxpd1901[6], &vivo_sub_otp_data[Awb_sub_addr], 12);
		memcpy(&vivo_sub_otp_data_s5k4h7yxpd1901[18], &vivo_sub_otp_data[Lsc_sub_addr], 1868);
		//add for printk S5K4H7YXPD1901
		#if 0
		for (i = 0; i < 1886; i++) {
			LOG_INF("read_otp_data_for_hal[%d] = 0x%x\n", i, vivo_sub_otp_data_s5k4h7yxpd1901[i]);
		}
		#endif
		return 1;
	} else {
		S5K4H7YXPD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("invalid otp data. error!!!   otp_error_code:%d\n", S5K4H7YXPD1901_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo cfx add end*/
