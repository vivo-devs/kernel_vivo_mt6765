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
#include "ov13855mipiraw_Sensor.h"

#define PFX "OV13855_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo cfx add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_OTP_DATA_SIZE 0x0D68 /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
#define AF_RANGE_GOT  //if got the range of AF_inf and AF_mac , open this define ,the default AF range is 10%!!!!
#define VIVO_EEPROM_WRITE_ID 0xA0
#define VIVO_I2C_SPEED 400
#define VIVO_MAX_OFFSET 0x0D68
/*#define VIVO_VENDOR_SUNNY 0x01*/
#define VIVO_VENDOR_TRULY 0x02
/*#define VIVO_VENDOR_QTECH 0x05*/
#define VIVO_VENDOR_OFILM 0x09
#define VIVO_VENDOR_LENS_ID 0x01
#define VIVO_VENDOR_VCM_ID 0x04
#define VIVO_VENDOR_DRIVERIC_ID 0x04
#define VIVO_VENDOR_PLATFORM_ID 0x01

static unsigned char vivo_otp_data[VIVO_OTP_DATA_SIZE];
static unsigned const int ModuleInfo_addr = 0x0000;
static unsigned const int ModuleInfo_checksum_addr = 0x0001;
static unsigned const int Af_addr = 0x07f7;
static unsigned const int Af_checksum_addr = 0x0807;
static unsigned const int Awb_addr = 0x000f;
static unsigned const int Awb_checksum_addr = 0x001c;
static unsigned const int Lsc_addr = 0x0021;
static unsigned const int Lsc_checksum_addr = 0x076e;
static unsigned const int PD_Proc1_addr = 0x0808;
static unsigned const int PD_Proc1_checksum_addr = 0x09f9;
static unsigned const int PD_Proc2_addr = 0x9fa;
static unsigned const int PD_Proc2_checksum_addr = 0x0d67;
#ifdef AF_RANGE_GOT
static unsigned const int AF_inf_golden = 320;
static unsigned const int AF_mac_golden = 700;
#endif
static int checksum = 0;
otp_error_code_t OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
unsigned  int ov13855_flag;
static bool vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{

	char pu_send_cmd[2] = {(char)(addr >> 8),  (char)(addr & 0xFF)};
    if (addr > VIVO_MAX_OFFSET) { /*VIVO_MAX_OFFSET*/
		return false;
	}
	kdSetI2CSpeed(VIVO_I2C_SPEED);

	if (iReadRegI2C(pu_send_cmd,  2,  (u8 *)data,  1,  VIVO_EEPROM_WRITE_ID) < 0) {
		return false;
	}
    return true;
}
int ov13855_vivo_otp_read(void)
{
	int i = 0;
	int offset = ModuleInfo_addr;
	int check_if_group_valid = 0;
	int R_unit = 0,B_unit = 0,R_golden = 0,B_golden = 0,G_unit = 0,G_golden = 0;
	#ifdef AF_RANGE_GOT
	int diff_inf = 0, diff_mac = 0, diff_inf_macro = 0;
	int AF_inf = 0, AF_mac = 0;
	#endif
	
	long long t1,t2,t3,t4,t5,t6,t,temp;
	OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		if(!vivo_read_eeprom(offset, &vivo_otp_data[i])){
			LOG_INF("read_vivo_eeprom 0x%0x %d fail \n",offset, vivo_otp_data[i]);
			return 0;
		}
		/*LOG_INF("[xyf++1]read_vivo_eeprom vivo_otp_data[0x%0x] =0x%x\n",offset, vivo_otp_data[i]);*/
		offset++;
	}
	//check_if_group_valid = vivo_otp_data[ModuleInfo_addr] | vivo_otp_data[Awb_addr] | vivo_otp_data[Af_addr] | vivo_otp_data[Lsc_addr] | vivo_otp_data[PD_Proc1_addr] | vivo_otp_data[PD_Proc2_addr];
	if((0x01 == vivo_otp_data[ModuleInfo_addr]) && (0x01 == vivo_otp_data[Awb_addr]) && (0x01 == vivo_otp_data[Af_addr]) && (0x01 == vivo_otp_data[Lsc_addr]) && (0x01 == vivo_otp_data[PD_Proc1_addr]) && (0x01 == vivo_otp_data[PD_Proc2_addr]))
	{
		check_if_group_valid = 0x01;
		LOG_INF("[xyf++]0x01 is valid.check_if_group_valid:%d\n",check_if_group_valid);
	}
	
	if (vivo_otp_data[0x07] != 0x01 ){
		OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
		ov13855_flag = 0;
		LOG_INF("VCM ID Error!!!    ov13855_flag =%d\n", ov13855_flag);
	}else{
		ov13855_flag = 1;
		LOG_INF("ov13855_flag = %d\n", ov13855_flag);
	}
	
	if (check_if_group_valid == 0x01) //all the data is valid
	{
		/////ModuleInfo 
		if((VIVO_VENDOR_TRULY != vivo_otp_data[0x05]) && (VIVO_VENDOR_OFILM != vivo_otp_data[0x05]))///xuyongfu need check 2nd vendor
		{
			OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[xyf++]ModuleInfo_info error!!!    otp_error_code:%d\n",OV13855_OTP_ERROR_CODE);
			return 0;
		} else if ((vivo_otp_data[0x06] != 0x05) || (vivo_otp_data[0x08] != 0x03) || (vivo_otp_data[0x09] != 0x01))
		{
			OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[xyf++]: Platform ID or Sensor or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code:%d\n",OV13855_OTP_ERROR_CODE);
			return 0;
		} else if ((vivo_otp_data[0x07] != 0x01) && (vivo_otp_data[0x07] != 0x02)) {
			OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[xyf++]:  VCM ID Error!!!    otp_error_code:%d\n", OV13855_OTP_ERROR_CODE);
			return 0;		
		}
		else if((0xff != vivo_otp_data[0x0a]) || (0x00 != vivo_otp_data[0x0b]) || (0x0b != vivo_otp_data[0x0c]) || (0x01 != vivo_otp_data[0x0d]))
		{
			OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[xyf++]: calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n",vivo_otp_data[0x0a],vivo_otp_data[0x0b],vivo_otp_data[0x0c],vivo_otp_data[0x0d]);
			return 0;
		}
		//////ModuleInfo_checksum
		checksum = 0;
        for(i = ModuleInfo_addr + 2; i < ModuleInfo_checksum_addr + 13; i ++)
        {              
			checksum += vivo_otp_data[i];						
        }  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[ModuleInfo_checksum_addr] != checksum )
        {
			OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
            LOG_INF("[xyf++]ModuleInfo_checksum error!!!   otp_error_code:%d\n",OV13855_OTP_ERROR_CODE);
			return 0;
        }
		
		//////AWB_checksum
		checksum = 0;
        for(i = Awb_addr+1; i < Awb_checksum_addr; i ++)
        {              
			checksum += vivo_otp_data[i];						
        }  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[Awb_checksum_addr] != checksum )
        {
			OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
            LOG_INF("[xyf++]AWB_checksum error!!!   otp_error_code:%d\n",OV13855_OTP_ERROR_CODE);
			return 0;
        }
		
		//////AF_checksum
		checksum = 0;
        for(i = Af_addr+1; i < Af_checksum_addr; i ++)
        {              
			checksum += vivo_otp_data[i];						
        }  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[Af_checksum_addr] != checksum )
        {
			OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
            LOG_INF("[xyf++]AF_checksum error!!!   otp_error_code:%d\n",OV13855_OTP_ERROR_CODE);
			return 0;
        }
		
		//////LSC_checksum
		checksum = 0;
        for(i = Lsc_addr+1; i < Lsc_checksum_addr; i ++)
        {              
			checksum += vivo_otp_data[i];						
        }  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[Lsc_checksum_addr] != checksum )
        {
			OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("LSC_checksum error!!!   otp_error_code:%d\n", OV13855_OTP_ERROR_CODE);
			return 0;
		}

		/****PDAF_checksum****/
		checksum = 0;
        for(i = PD_Proc1_addr+1; i < PD_Proc1_checksum_addr; i ++)
        {              
			checksum += vivo_otp_data[i];						
        }  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[PD_Proc1_checksum_addr] != checksum )
        {
			OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
            LOG_INF("[xyf++]PD_Proc1_data_checksum error!!!   otp_error_code:%d\n",OV13855_OTP_ERROR_CODE);
			return 0;
        }
		checksum = 0;//PD_Proc2_data
		for(i = PD_Proc2_addr+1; i < PD_Proc2_checksum_addr; i ++)
        {              
			checksum += vivo_otp_data[i];						
        }  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[PD_Proc2_checksum_addr] != checksum )
        {
			OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
            LOG_INF("[xyf++]PD_Proc2_data_checksum error!!!   otp_error_code:%d\n",OV13855_OTP_ERROR_CODE);
			return 0;
        }
		
		/****check if awb out of range[high cct]****/
		R_unit = vivo_otp_data[Awb_addr+1];
		R_unit = (R_unit << 8) | (vivo_otp_data[Awb_addr+2]);
		B_unit = vivo_otp_data[Awb_addr+3];
		B_unit = (B_unit << 8) | (vivo_otp_data[Awb_addr+4]);
		G_unit = vivo_otp_data[Awb_addr+5];
		G_unit = (G_unit << 8) | (vivo_otp_data[Awb_addr+6]);

		R_golden = vivo_otp_data[Awb_addr+7];
		R_golden = (R_golden << 8) | (vivo_otp_data[Awb_addr+8]);
		B_golden = vivo_otp_data[Awb_addr+9];
		B_golden = (B_golden << 8) | (vivo_otp_data[Awb_addr+10]);
		G_golden = vivo_otp_data[Awb_addr+11];
		G_golden = (G_golden << 8) | (vivo_otp_data[Awb_addr+12]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("cfx_add:R_unit=%d, R_golden=%d, B_unit=%d, B_golden=%d, G_unit=%d, G_golden=%d\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
		t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
		t2 = R_golden*R_golden;
		t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
		t4 = G_golden*G_golden;
		t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
		t6 = B_golden*B_golden;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("xyf_add:t1 = %lld ,t2 = %lld ,t3 = %lld ,t4 = %lld ,t5 = %lld ,t6 = %lld ,temp = %lld ,t = %lld\n",t1,t2,t3,t4,t5,t6,temp,t);
		if(t > 0) 
		{
			OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, OV13855_OTP_ERROR_CODE);
			return 0;
		}

		#ifdef AF_RANGE_GOT
		/*******check if AF out of range******/

		AF_inf  =  vivo_otp_data[Af_addr+6];
		AF_inf = (AF_inf << 8) | (vivo_otp_data[Af_addr+7]);
				
		AF_mac = vivo_otp_data[Af_addr+8];
		AF_mac = (AF_mac << 8) | (vivo_otp_data[Af_addr+9]);
        diff_inf_macro = AF_mac - AF_inf;
        
		diff_inf = (AF_inf - AF_inf_golden) > 0 ? (AF_inf - AF_inf_golden) : (AF_inf_golden - AF_inf);
		diff_mac = (AF_mac - AF_mac_golden) > 0 ? (AF_mac - AF_mac_golden) : (AF_mac_golden - AF_mac);
   
        /* tyj add for distinguishing VCM */
        if(vivo_otp_data[0x07] == 0x02){    /* 0x02 Shicoh, 0x01 TDK */ 
			if (diff_inf > 100 || diff_mac > 120 || diff_inf_macro > 500 || diff_inf_macro < 260) {  /*AF code out of range*/
			    OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			    LOG_INF("AF out of range error!!!   otp_error_code:%d   VCM ID = 0x%x AF_inf = %d AF_mac = %d\n", OV13855_OTP_ERROR_CODE, vivo_otp_data[0x07], AF_inf, AF_mac);
			    return 0;
		    }
        }
        /* tyj add for distinguishing end */
        else{
            LOG_INF("[tyj++]distinguish vcm VCM ID = 0x%x inf = %d mac = %d\n", vivo_otp_data[0x07], AF_inf, AF_mac);
		    if (diff_inf > 100 || diff_mac > 110 || diff_inf_macro > 500 || diff_inf_macro < 260) {  /*AF code out of range*/
			    OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			    LOG_INF("AF out of range error!!!   otp_error_code:%d   VCM ID = 0x%x AF_inf = %d AF_mac = %d\n", OV13855_OTP_ERROR_CODE, vivo_otp_data[0x07], AF_inf, AF_mac);
			    return 0;
		    }
        }
		#endif

		/*cfx add for pdaf data start 20161223*/
		/*for(i = 0;i < 1372;i++)
		{
			if(i == 0)
				LOG_INF("read_OV13855_PDAF_data");
			if(i < 496)
				PDAF_data[i] = vivo_otp_data[PD_Proc1_addr+i+1];
			else //i >= 496
				PDAF_data[i] = vivo_otp_data[PD_Proc1_addr+i+3];
		}*/
		/*copy pdaf data*/
		/*memcpy(PDAF_data, &vivo_otp_data[PDAF_addr+1], PDAF_checksum_addr-PDAF_addr-1);*/
		/*cfx add for pdaf data end*/

		return 1;
	} else {
		OV13855_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("invalid otp data. error!!!   otp_error_code:%d\n", OV13855_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo cfx add end*/
