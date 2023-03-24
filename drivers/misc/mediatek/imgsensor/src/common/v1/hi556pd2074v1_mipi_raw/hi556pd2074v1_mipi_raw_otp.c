#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>


#include "../imgsensor_i2c.h"
#include "hi556pd2074v1mipiraw_Sensor.h"

#define PFX "hi556pd2074v1_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo cfx add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_SUB_OTP_DATA_SIZE 0x16E7 /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
/*#define AF_RANGE_GOT*/  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_SUB_EEPROM_WRITE_ID 0x40
#define VIVO_SUB_I2C_SPEED 400
#define VIVO_SUB_MAX_OFFSET 0x16E7
/*#define VIVO_SUB_VENDOR_SUNNY 0x01*/
/*#define VIVO_SUB_VENDOR_TRULY 0x02*/
#define VIVO_SUB_VENDOR_QTECH 0x05
/*#define VIVO_SUB_VENDOR_OFILM 0x09*/
#define VIVO_SUB_VENDOR_LENS_ID 0x10
#define VIVO_SUB_VENDOR_PLATFROM_ID 0x01
/*#define VIVO_SUB_VENDOR_VCM_ID 0x00*/
/*#define VIVO_SUB_VENDOR_DRIVERIC_ID 0x00*/
/*#define VIVO_SUB_VENDOR_PLATFORM_ID 0x00*/

static unsigned char vivo_sub_otp_data[VIVO_SUB_OTP_DATA_SIZE];
unsigned char vivo_sub_otp_data_hi556pd2074v1[1891];
unsigned const int ModuleInfo_sub_flag_hi556pd2074v1 = 0x0451 - 0x0401;   /*0x190d - 0x0401*/
unsigned int ModuleInfo_sub_addr_hi556pd2074v1;
unsigned int ModuleInfo_sub_checksum_addr_hi556pd2074v1;
unsigned const int Awb_sub_flag_hi556pd2074v1 = 0;
unsigned int Awb_sub_addr_hi556pd2074v1;
unsigned int Awb_sub_checksum_addr_hi556pd2074v1;
unsigned const int Sn_sub_flag_hi556pd2074v1 = 0x0476 - 0x0401;  /*0x1929 - 0x0401*/
unsigned int Sn_sub_addr_hi556pd2074v1;
unsigned int Sn_sub_checksum_addr_hi556pd2074v1;
unsigned const int Fuseid_sub_flag_hi556pd2074v1 = 0x049E - 0x0401;  /*0x1929 - 0x0401*/
unsigned int Fuseid_sub_addr_hi556pd2074v1;
unsigned int Fuseid_sub_checksum_addr_hi556pd2074v1;

#if 1
static unsigned const int Lsc_sub_flag = 0x0500 - 0x0401;
static unsigned int Lsc_sub_addr;
static unsigned int Lsc_sub_checksum_addr;
#endif

static int checksum;
otp_error_code_t SUB_OTP_ERROR_CODE_HI556PD2074V1 = OTP_ERROR_CODE_NORMAL;
extern MUINT32  sn_inf_sub_hi556pd2074v1[13];


static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	kdSetI2CSpeed(VIVO_SUB_I2C_SPEED); 
	iWriteRegI2C(pu_send_cmd, 3, VIVO_SUB_EEPROM_WRITE_ID);
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	
	kdSetI2CSpeed(400); 

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, VIVO_SUB_EEPROM_WRITE_ID);

	return get_byte;
}

static int HI556PD2074V1_otp_read_setting_init_continuous_read(kal_uint16 addr)
{
	int i;
	write_cmos_sensor_8(0x0a02, 0x01); /*Fast sleep on*/
	write_cmos_sensor_8(0x0a00, 0x00); /*stand by on*/
	mdelay(10);
	write_cmos_sensor_8(0x0f02, 0x00); /*pll disable*/
	write_cmos_sensor_8(0x011a, 0x01); /*CP TRIM_H*/
	write_cmos_sensor_8(0x011b, 0x09); /*IPGM TRIM_H*/
	write_cmos_sensor_8(0x0d04, 0x01); /*Fsync(otp busy) Output Enable*/
	write_cmos_sensor_8(0x0d00, 0x07); /*Fsync(otp busy) Output Drivability*/
	write_cmos_sensor_8(0x003e, 0x10); /*OTP R/W mode*/
	write_cmos_sensor_8(0x0a00, 0x01); /*stand by off*/

	write_cmos_sensor_8(0x010a, ((addr)>>8)&0xff);/*start address H*/
	write_cmos_sensor_8(0x010b, (addr)&0xff);/*start address L*/
	write_cmos_sensor_8(0x0102, 0x01);/*single read*/
	for (i = 0; i < VIVO_SUB_OTP_DATA_SIZE; i++) {
		vivo_sub_otp_data[i] = read_cmos_sensor(0x108);
	}
	write_cmos_sensor_8(0x0a00, 0x00);
	mdelay(10);
	write_cmos_sensor_8(0x003e, 0x00);
	write_cmos_sensor_8(0x0a00, 0x01);
	return 0;
}

/* extern unsigned int is_atboot;guojunzheng add*/
int vivo_sub_otp_read_hi556pd2074v1(void)
{
	int i = 0;
	int offset = 0x0401;
	int check_if_ModuleInfo_group_valid = 1;
	int check_if_AWB_group_valid = 1;
	int check_if_Lsc_group_valid = 1;
	int check_if_SN_group_valid = 1;	
	int check_if_Fuseid_group_valid = 1;
	int check_if_group_valid = 0;
    long  R_unit = 0, B_unit = 0, R_golden = 0, B_golden = 0, G_unit = 0, G_golden = 0;
	long  t1, t2, t3, t4, t5, t6, t, temp;
	/* This operation takes a long time, we need to skip it. guojunzheng add begin
	if (is_atboot == 1) {
		LOG_INF("AT mode skip vivo_sub_otp_read_hi556pd2074v1\n");
		return 1;
	}
	 guojunzheng add end */

	HI556PD2074V1_otp_read_setting_init_continuous_read(offset);
	LOG_INF("read_hi556_data,moduleinfo_flag = 0x%x,awb_flag = 0x%x , sn_flag = 0x%x, fuse id = 0x%x\n", vivo_sub_otp_data[ModuleInfo_sub_flag_hi556pd2074v1], vivo_sub_otp_data[Awb_sub_flag_hi556pd2074v1], vivo_sub_otp_data[Sn_sub_flag_hi556pd2074v1], vivo_sub_otp_data[Fuseid_sub_flag_hi556pd2074v1]);
	LOG_INF("read_hi556_data\n");

	#if 0
	for (i = 0; i < VIVO_SUB_OTP_DATA_SIZE; i++) {
		LOG_INF("read_hi556pd2074v1_data[0x%x][%d] = 0x%x\n", (0x0401+i),i, vivo_sub_otp_data[i]);
	}
	#endif
	/*module info*/
	if (0x01 == vivo_sub_otp_data[ModuleInfo_sub_flag_hi556pd2074v1]) { /*Group 1*/
		ModuleInfo_sub_addr_hi556pd2074v1 = 0x0452 - 0x0401;
		ModuleInfo_sub_checksum_addr_hi556pd2074v1 = ModuleInfo_sub_addr_hi556pd2074v1 + 11;
		LOG_INF("ModuleInfo Group 1 !!!\n");
	} else if (0x13 == vivo_sub_otp_data[ModuleInfo_sub_flag_hi556pd2074v1]) { /*Group 2*/
		ModuleInfo_sub_addr_hi556pd2074v1 = 0x045E - 0x0401;
		ModuleInfo_sub_checksum_addr_hi556pd2074v1 = ModuleInfo_sub_addr_hi556pd2074v1 + 11;
		LOG_INF("ModuleInfo Group 2 !!!\n");
	} else if (0x37 == vivo_sub_otp_data[ModuleInfo_sub_flag_hi556pd2074v1]) { /*Group 3*/
		ModuleInfo_sub_addr_hi556pd2074v1 = 0x046A - 0x0401;
		ModuleInfo_sub_checksum_addr_hi556pd2074v1 = ModuleInfo_sub_addr_hi556pd2074v1 + 11;
		LOG_INF("ModuleInfo Group 3 !!!\n");
	} else {
		check_if_ModuleInfo_group_valid = 0;
		LOG_INF("ModuleInfo flag error!!!    ModuleInfo flag :%d\n", vivo_sub_otp_data[ModuleInfo_sub_flag_hi556pd2074v1]);
	}
	/*awb*/
	if (0x01 == vivo_sub_otp_data[Awb_sub_flag_hi556pd2074v1]) { /*Group 1*/
		Awb_sub_addr_hi556pd2074v1 = 0x0402 - 0x0401;
		Awb_sub_checksum_addr_hi556pd2074v1 = Awb_sub_addr_hi556pd2074v1 + 12;
		LOG_INF("AWB Group 1 !!!\n");
		LOG_INF("lilin Awb_sub_addr_hi556pd2074v1:%d\n", Awb_sub_addr_hi556pd2074v1);
	} else if (0x13 == vivo_sub_otp_data[Awb_sub_flag_hi556pd2074v1]) { /*Group 2*/
		Awb_sub_addr_hi556pd2074v1 = 0x040F - 0x0401;
		Awb_sub_checksum_addr_hi556pd2074v1 = Awb_sub_addr_hi556pd2074v1 + 12;
		LOG_INF("AWB Group 2 !!!\n");
		LOG_INF("lilin Awb_sub_addr_hi556pd2074v1:%d\n", Awb_sub_addr_hi556pd2074v1);
	} else if (0x37 == vivo_sub_otp_data[Awb_sub_flag_hi556pd2074v1]) { /*Group 3*/
		Awb_sub_addr_hi556pd2074v1 = 0x41C - 0x0401;
		Awb_sub_checksum_addr_hi556pd2074v1 = Awb_sub_addr_hi556pd2074v1 + 12;
		LOG_INF("AWB Group 3 !!!\n");
		LOG_INF("lilin Awb_sub_addr_hi556pd2074v1:%d\n", Awb_sub_addr_hi556pd2074v1);
	} else {
		check_if_AWB_group_valid = 0;
		LOG_INF("AWB flag error!!!    AWB flag :%d\n", vivo_sub_otp_data[Awb_sub_flag_hi556pd2074v1]);
	}
      
	/*sn inf*/
	if (0x01 == vivo_sub_otp_data[Sn_sub_flag_hi556pd2074v1]) { /*Group 1*/
		Sn_sub_addr_hi556pd2074v1 = 0x0477 - 0x0401;
		Sn_sub_checksum_addr_hi556pd2074v1 = Sn_sub_addr_hi556pd2074v1 + 12;
		LOG_INF("Sn Group 1 !!!\n");
	} else if (0x13 == vivo_sub_otp_data[Sn_sub_flag_hi556pd2074v1]) { /*Group 2*/
		Sn_sub_addr_hi556pd2074v1 = 0x0484 - 0x0401;
		Sn_sub_checksum_addr_hi556pd2074v1 = Sn_sub_addr_hi556pd2074v1 + 12;
		LOG_INF("Sn Group 2 !!!\n");
	} else if (0x37 == vivo_sub_otp_data[Sn_sub_flag_hi556pd2074v1]) { /*Group 3*/
		Sn_sub_addr_hi556pd2074v1 = 0x0491 - 0x0401;
		Sn_sub_checksum_addr_hi556pd2074v1 = Sn_sub_addr_hi556pd2074v1 + 12;
		LOG_INF("Sn Group 3 !!!\n");
	} else {
		check_if_SN_group_valid = 0;
		LOG_INF("Sn flag error!!!    Sn  flag :%d\n", vivo_sub_otp_data[Sn_sub_flag_hi556pd2074v1]);
	}

	
	/*Fuse id inf*/
	if (0x01 == vivo_sub_otp_data[Fuseid_sub_flag_hi556pd2074v1]) { /*Group 1*/
		Fuseid_sub_addr_hi556pd2074v1 = 0x049F - 0x0401;
		Fuseid_sub_checksum_addr_hi556pd2074v1 = Fuseid_sub_addr_hi556pd2074v1 + 13;
		LOG_INF("Fuse id Group 1 !!!\n");
	} else if (0x13 == vivo_sub_otp_data[Fuseid_sub_flag_hi556pd2074v1]) { /*Group 2*/
		Fuseid_sub_addr_hi556pd2074v1 = 0x04AD - 0x0401;
		Fuseid_sub_checksum_addr_hi556pd2074v1 = Fuseid_sub_addr_hi556pd2074v1 + 13;
		LOG_INF("Fuse id Group 2 !!!\n");
	} else if (0x37 == vivo_sub_otp_data[Fuseid_sub_flag_hi556pd2074v1]) { /*Group 3*/
		Fuseid_sub_addr_hi556pd2074v1 = 0x04BB - 0x0401;
		Fuseid_sub_checksum_addr_hi556pd2074v1 = Fuseid_sub_addr_hi556pd2074v1 + 13;
		LOG_INF("Fuse id Group 3 !!!\n");
	} else {
		check_if_Fuseid_group_valid = 0;
		LOG_INF("Fuse id flag error!!!    Fuse id  flag :%d\n", vivo_sub_otp_data[Fuseid_sub_addr_hi556pd2074v1]);
	}
	/*lsc info*/
	if(0x04 == vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x00]){
    
	if (0x01 == vivo_sub_otp_data[Lsc_sub_flag]) { /*Group 1*/
		Lsc_sub_addr = 0x501 - 0x401;
		Lsc_sub_checksum_addr = Lsc_sub_addr + 1868;
		LOG_INF("Lsc Group 1 !!!\n");
	} else if (0x13 == vivo_sub_otp_data[Lsc_sub_flag]) { /*Group 2*/
		Lsc_sub_addr = 0xc4e - 0x401;
		Lsc_sub_checksum_addr = Lsc_sub_addr + 1868;
		LOG_INF("Lsc Group 2 !!!\n");
	} else if (0x37 == vivo_sub_otp_data[Lsc_sub_flag]) { /*Group 2*/
		Lsc_sub_addr = 0x139b - 0x401;
		Lsc_sub_checksum_addr = Lsc_sub_addr + 1868;
		LOG_INF("Lsc Group 3 !!!\n");
	} else {
		check_if_Lsc_group_valid = 0;
		LOG_INF("Lsc flag error!!!   Lsc  group flag = %d \n", Lsc_sub_flag);
	}
	}
	LOG_INF("read_hi556_data,moduleinfo_addr=0x%x,awb_addr= 0x%x, sn = 0x%x, Fuseid_sub_addr_hi556pd2074v1 = 0x%x\n", ModuleInfo_sub_addr_hi556pd2074v1, Awb_sub_addr_hi556pd2074v1,Sn_sub_addr_hi556pd2074v1, Fuseid_sub_addr_hi556pd2074v1);
     if(0x04 == vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x00]){
	 check_if_group_valid = check_if_AWB_group_valid & check_if_ModuleInfo_group_valid & check_if_SN_group_valid & check_if_Fuseid_group_valid ;
	}else {
	 check_if_group_valid = check_if_AWB_group_valid & check_if_ModuleInfo_group_valid & check_if_SN_group_valid & check_if_Fuseid_group_valid & check_if_Lsc_group_valid;
	}
	if (check_if_group_valid == 0x01) { /****all the data is valid****/
		/****ModuleInfo****/
		if ((VIVO_SUB_VENDOR_QTECH != vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x00])&&(0x04 != vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x00])) {
			SUB_OTP_ERROR_CODE_HI556PD2074V1 = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("Module ID error!!!    otp_error_code:%d\n", SUB_OTP_ERROR_CODE_HI556PD2074V1);
			return 0;
		} else if ((VIVO_SUB_VENDOR_LENS_ID != vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x05])&(0x10 != vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x04])) {
			SUB_OTP_ERROR_CODE_HI556PD2074V1 = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("Platform ID or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code =%d\n", SUB_OTP_ERROR_CODE_HI556PD2074V1);
			return 0;
		} else if ((VIVO_SUB_VENDOR_PLATFROM_ID != vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x01])&&(0x04 != vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x00])) {
			SUB_OTP_ERROR_CODE_HI556PD2074V1 = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("Platform ID  Error!!!    otp_error_code =%d\n", SUB_OTP_ERROR_CODE_HI556PD2074V1);
			return 0;
		} 
		/*else if ((0xff != vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x08]) || (0x00 != vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x09]) || (0x0b != vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x0a]) || (0x01 != vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x0b])) {
			SUB_OTP_ERROR_CODE_HI556PD2074V1 = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF(": calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n", vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x08], vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x09], vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x0a], vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x0b]);
			return 0;
		}*/
		/****ModuleInfo_checksum****/
		checksum = 0;
		for (i = ModuleInfo_sub_addr_hi556pd2074v1; i < ModuleInfo_sub_checksum_addr_hi556pd2074v1; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
			LOG_INF("checksum value:%d, OTP checksum value:%d\n", checksum, vivo_sub_otp_data[ModuleInfo_sub_checksum_addr_hi556pd2074v1]);
		if (vivo_sub_otp_data[ModuleInfo_sub_checksum_addr_hi556pd2074v1] != checksum) {
			SUB_OTP_ERROR_CODE_HI556PD2074V1 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("ModuleInfo_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE_HI556PD2074V1);
			return 0;
			}
		/****AWB_checksum****/
		checksum = 0;
		for (i = Awb_sub_addr_hi556pd2074v1; i < Awb_sub_checksum_addr_hi556pd2074v1; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
			LOG_INF("checksum value:%d, OTP checksum value:%d\n", checksum, vivo_sub_otp_data[ModuleInfo_sub_checksum_addr_hi556pd2074v1]);
		if (vivo_sub_otp_data[Awb_sub_checksum_addr_hi556pd2074v1] != checksum) {
			SUB_OTP_ERROR_CODE_HI556PD2074V1 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AWB_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE_HI556PD2074V1);
			return 0;
		}
		R_unit = vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1];
		R_unit = (R_unit << 8) | (vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1+1]);
		B_unit = vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1+2];
		B_unit = (B_unit << 8) | (vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1+3]);
		G_unit = vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1+4];
		G_unit = (G_unit << 8) | (vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1+5]);

		R_golden = vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1+6];
		R_golden = (R_golden << 8) | (vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1+7]);
		B_golden = vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1+8];
		B_golden = (B_golden << 8) | (vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1+9]);
		G_golden = vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1+10];
		G_golden = (G_golden << 8) | (vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1+11]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("lilin_add:R_unit=%lld, R_golden=%lld, B_unit=%lld, B_golden=%lld, G_unit=%lld, G_golden=%lld\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
		t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
		t2 = R_golden*R_golden;
		t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
		t4 = G_golden*G_golden;
		t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
		t6 = B_golden*B_golden;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 17721;
		LOG_INF("lilin_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
		SUB_OTP_ERROR_CODE_HI556PD2074V1 = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
		LOG_INF("lilin AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%	otp_error_code:%d\n",	temp,	SUB_OTP_ERROR_CODE_HI556PD2074V1);
		return 0;
		}
		
		
		/****SN_checksum****/
		checksum = 0;
		for (i = Sn_sub_addr_hi556pd2074v1; i < Sn_sub_checksum_addr_hi556pd2074v1; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Sn_sub_checksum_addr_hi556pd2074v1] != checksum) {
			SUB_OTP_ERROR_CODE_HI556PD2074V1 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE_HI556PD2074V1);
			return 0;
		}
		sn_inf_sub_hi556pd2074v1[0] = 0x01;
		for (i = 0; i < 12; i++) {
			sn_inf_sub_hi556pd2074v1[i + 1] = (MUINT32)vivo_sub_otp_data[i + Sn_sub_addr_hi556pd2074v1];
			LOG_INF("sn_inf_sub_hi556pd2074v1[%d] = 0x%x, vivo_sub_otp_data[0x%x] = 0x%x\n", i + 1, sn_inf_sub_hi556pd2074v1[i + 1],  i + Sn_sub_addr_hi556pd2074v1, vivo_sub_otp_data[i + Sn_sub_addr_hi556pd2074v1]);
		}		
		/****Fuse id _checksum****/
		checksum = 0;
		for (i = Fuseid_sub_addr_hi556pd2074v1; i < Fuseid_sub_checksum_addr_hi556pd2074v1; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Fuseid_sub_checksum_addr_hi556pd2074v1] != checksum) {
			SUB_OTP_ERROR_CODE_HI556PD2074V1 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("Fuse id_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE_HI556PD2074V1);
			return 0;
		}
		#if 1
		/****LSC_checksum****/
		if(0x04 == vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x00]){
		checksum = 0;
		for (i = Lsc_sub_addr; i < Lsc_sub_checksum_addr; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Lsc_sub_checksum_addr] != checksum) {
			SUB_OTP_ERROR_CODE_HI556PD2074V1 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("LSC_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE_HI556PD2074V1);
			return 0;
		}
		}
		#endif
	    /****check if awb out of range[high cct]****/
		memcpy(vivo_sub_otp_data_hi556pd2074v1, &vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1], 8);
		memcpy(&vivo_sub_otp_data_hi556pd2074v1[8], &vivo_sub_otp_data[0], 1);
		memcpy(&vivo_sub_otp_data_hi556pd2074v1[9], &vivo_sub_otp_data[Awb_sub_addr_hi556pd2074v1], 12);
		if(0x04 == vivo_sub_otp_data[ModuleInfo_sub_addr_hi556pd2074v1 + 0x00]){
		 memcpy(&vivo_sub_otp_data_hi556pd2074v1[21], &vivo_sub_otp_data[Lsc_sub_addr -1], 1869);
		}
		
		/* add for print hi556 data*/
		#if 0
		for (i = 0; i < 21; i++) {
			LOG_INF("read_hi556_data[%d] = 0x%x\n", i, vivo_sub_otp_data_hi556pd2074v1[i]);
		}
		#endif
		return 1;
	} else {
		SUB_OTP_ERROR_CODE_HI556PD2074V1 = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("invalid otp data. error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE_HI556PD2074V1);
		return 0;
	}
}
/*vivo hope add end*/
