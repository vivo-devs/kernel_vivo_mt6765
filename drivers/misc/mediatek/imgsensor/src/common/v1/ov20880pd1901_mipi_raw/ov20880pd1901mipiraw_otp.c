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
#include "ov20880pd1901mipiraw_Sensor.h"

#define PFX "OV20880PD1901_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo cfx add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_OTP_DATA_SIZE 0x17FA /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
/*#define AF_RANGE_GOT*/  //if got the range of AF_inf and AF_mac , open this define ,the default AF range is 10%!!!!
#define VIVO_EEPROM_WRITE_ID 0xA0
#define VIVO_I2C_SPEED 400
#define VIVO_MAX_OFFSET 0x17FA
/*#define VIVO_VENDOR_SUNNY 0x01*/
/*#define VIVO_VENDOR_TRULY 0x02*/
/*#define VIVO_VENDOR_QTECH 0x05*/
#define VIVO_VENDOR_OFILM 0x09
/*#define VIVO_VENDOR_LENS_ID 0x01*/
#define VIVO_VENDOR_LENS_ID 0x07
#define VIVO_VENDOR_VCM_ID 0x00
#define VIVO_VENDOR_DRIVERIC_ID 0x00
#define VIVO_VENDOR_PLATFORM_ID 0x01
static unsigned char vivo_otp_data[VIVO_OTP_DATA_SIZE];
static unsigned const int ModuleInfo_addr = 0x1000;
static unsigned const int Crosstalk_addr = 0x0B40;
static unsigned const int Dpc_addr = 0x07FA;
static unsigned const int Crosstalk_checksum_addr = 0x0D9F;
static unsigned const int Dpc_checksum_addr = 0x0B3F;
static unsigned const int ModuleInfo_checksum_addr = 0x101F;
static unsigned const int FuseId_addr = 0x1020;
static unsigned const int FuseId_checksum_addr = 0x1044;
static unsigned const int SN_addr = 0x1045;
static unsigned const int SN_checksum_addr = 0x1065;
static unsigned const int Awb_addr = 0x1066;
static unsigned const int Awb_checksum_addr = 0x109B;
static unsigned const int Awb_low_addr = 0x1073;

static unsigned const int Lsc_addr = 0x109C;
static unsigned const int Lsc_checksum_addr = 0x17F9;

#ifdef AF_RANGE_GOT
static unsigned const int AF_inf_golden = 320;
static unsigned const int AF_mac_golden = 700;
#endif
static int checksum = 0;
otp_error_code_t OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;

extern MUINT32 sn_inf_main_ov20880pd1901pd[13];  /*0 flag   1-12 data*/
extern MUINT32 crosstalk_data[607];  /*0 flag   1-606 data*/
extern MUINT32 dpc_data[837];  /*0 flag   1-836 data*/
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
int ov20880_read_sn_id_v1(void)
{
   int i = 0;
   int offset = 0x46;
   unsigned char sn_id[4] ={0x00};
   for(i = 0; i < 4; i++ ){
   	if (!vivo_read_eeprom(offset,  &sn_id[i])) {
			LOG_INF("read_vivo_eeprom 0x%0x %d fail \n", offset,  sn_id[i]);
			return 0;
	}
		/*LOG_INF("read_vivo_eeprom Data[0x%0x]:0x%x\n", offset,  vivo_otp_data[i]);*/
	offset++;	
	}
	return sn_id[0]<<24|sn_id[1]<<16|sn_id[2]<<8|sn_id[3];
}
int ov20880pd1901_vivo_otp_read(void)
{
	int i = 0;
	int offset = 0x0000;
	int check_if_group_valid = 0;
	int R_unit = 0, B_unit = 0, G_unit = 0, R_golden = 0, B_golden = 0, G_golden = 0;
	int R_unit_low = 0, B_unit_low = 0, G_unit_low = 0, R_golden_low = 0, B_golden_low = 0, G_golden_low = 0;
	
	#ifdef AF_RANGE_GOT
	int diff_inf = 0, diff_mac = 0, diff_inf_macro = 0;
	int AF_inf = 0, AF_mac = 0;
	#endif
	
	long long t1,t2,t3,t4,t5,t6,t,temp;
	OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		if(!vivo_read_eeprom(offset, &vivo_otp_data[i])){
			LOG_INF("read_vivo_eeprom 0x%0x %d fail \n",offset, vivo_otp_data[i]);
			return 0;
		}
		/*LOG_INF("read_vivo_eeprom vivo_otp_data[0x%0x] =0x%x\n",offset, vivo_otp_data[i]);*/
		offset++;
	}
	//check_if_group_valid = vivo_otp_data[ModuleInfo_addr] | vivo_otp_data[Awb_addr] | vivo_otp_data[Lsc_addr] | ;
	if((0x01 == vivo_otp_data[ModuleInfo_addr]) && (0x01 == vivo_otp_data[FuseId_addr])&& (0x01 == vivo_otp_data[SN_addr]) && (0x01 == vivo_otp_data[Awb_addr]) && (0x01 == vivo_otp_data[Lsc_addr])&&(0x01 == vivo_otp_data[Crosstalk_addr])&&(0x01 == vivo_otp_data[Dpc_addr]))
	{
		check_if_group_valid = 0x01;
		LOG_INF("0x01 is valid.check_if_group_valid:%d\n",check_if_group_valid);
	}
	if (check_if_group_valid == 0x01) //all the data is valid
	{
		/////ModuleInfo 
		if((VIVO_VENDOR_OFILM != vivo_otp_data[0x1001]) )///xuyongfu need check 2nd vendor
		{
			OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("ModuleInfo_info error!!!    otp_error_code:%d\n",OV20880PD1901_OTP_ERROR_CODE);
			return 0;
		} else if ((vivo_otp_data[0x1008] != 0x07) || (vivo_otp_data[0x100A] != 0x00) || (vivo_otp_data[0x1002] != 0x01))
		{
			OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("Platform ID or Sensor or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code:%d\n",OV20880PD1901_OTP_ERROR_CODE);
			return 0;
		} else if ((vivo_otp_data[0x1009] != 0x00) ) {
			OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("VCM ID Error!!!    otp_error_code:%d\n", OV20880PD1901_OTP_ERROR_CODE);
			return 0;		
		}
		else if((0xff != vivo_otp_data[0x100B]) || (0x00 != vivo_otp_data[0x100C]) || (0x0b != vivo_otp_data[0x100D]) || (0x01 != vivo_otp_data[0x100E]))
		{
			OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n",vivo_otp_data[0x0a],vivo_otp_data[0x0b],vivo_otp_data[0x0c],vivo_otp_data[0x0d]);
			return 0;
		}
		
			/*crosstalk_checksum*/
		   checksum = 0;
	        for(i = Crosstalk_addr + 1; i < Crosstalk_checksum_addr; i ++)
	        {              
				checksum += vivo_otp_data[i];						
	        }  
				checksum = checksum % 0xff + 1;
			if( vivo_otp_data[Crosstalk_checksum_addr] != checksum )
	        {
				OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
	            LOG_INF("crosstalk_checksum error!!!   otp_error_code:%d\n",OV20880PD1901_OTP_ERROR_CODE);
				return 0;
	        }
			for (i = 0; i < 607; i++) {	
			crosstalk_data[i] = (MUINT32)vivo_otp_data[i + Crosstalk_addr];
			LOG_INF("crosstalk_data[%d] = 0x%x, vivo_otp_data[0x%x] = 0x%x\n", i, crosstalk_data[i],	i + Crosstalk_addr, vivo_otp_data[i + Crosstalk_addr]);
			}				

			/*dpc_checksum*/
		   checksum = 0;
	        for(i = Dpc_addr + 1; i < Dpc_checksum_addr; i ++)
	        {              
				checksum += vivo_otp_data[i];						
	        }  
				checksum = checksum % 0xff + 1;
			if( vivo_otp_data[Dpc_checksum_addr] != checksum )
	        {
				OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
	            LOG_INF("dpc_checksum error!!!   otp_error_code:%d\n",OV20880PD1901_OTP_ERROR_CODE);
				return 0;
	        }
			for (i = 0; i < 837; i++) {	
			dpc_data[i] = (MUINT32)vivo_otp_data[i + Dpc_addr];
			LOG_INF("dpc_data[%d] = 0x%x, vivo_otp_data[0x%x] = 0x%x\n", i, dpc_data[i],	i + Dpc_addr, vivo_otp_data[i + Dpc_addr]);
			}					
		/*ModuleInfo_checksum*/
		checksum = 0;
	        for(i = ModuleInfo_addr + 1; i < ModuleInfo_checksum_addr; i ++)
	        {              
				checksum += vivo_otp_data[i];						
	        }  
				checksum = checksum % 0xff + 1;
			if( vivo_otp_data[ModuleInfo_checksum_addr] != checksum )
	        {
				OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
	            LOG_INF("ModuleInfo_checksum error!!!   otp_error_code:%d\n",OV20880PD1901_OTP_ERROR_CODE);
				return 0;
	        }

		/*Fuse Id information*/
		checksum = 0;
		for(i = FuseId_addr + 1; i < FuseId_checksum_addr; i ++)
		{			   
			checksum += vivo_otp_data[i];						
		}  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[FuseId_checksum_addr] != checksum )
		{
			OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("FuseId_checksum error!!!   otp_error_code:%d\n",OV20880PD1901_OTP_ERROR_CODE);
			return 0;
		}

		/*SN information*/
		checksum = 0;
		for(i = SN_addr + 1; i < SN_checksum_addr; i ++)
		{			   
			checksum += vivo_otp_data[i];						
		}  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[SN_checksum_addr] != checksum )
		{
			OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n",OV20880PD1901_OTP_ERROR_CODE);
			return 0;
		}
		sn_inf_main_ov20880pd1901pd[0] = 0x01;	
		#if 1	
		for (i = 0; i < 12; i++) {	
			sn_inf_main_ov20880pd1901pd[i + 1] = (MUINT32)vivo_otp_data[i + 1 + SN_addr];
			LOG_INF("sn_inf_main_ov20880pd1901pd[%d] = 0x%x, vivo_otp_data[0x%x] = 0x%x\n", i + 1, sn_inf_main_ov20880pd1901pd[i + 1],	i + SN_addr, vivo_otp_data[i + 1 + SN_addr]);	}	
	    #endif
		/*AWB_checksum*/
		checksum = 0;
		for(i = Awb_addr+1; i < Awb_checksum_addr; i ++)
		{              
			checksum += vivo_otp_data[i];						
		}  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[Awb_checksum_addr] != checksum )
		{
			OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
		    LOG_INF("AWB_checksum error!!!   otp_error_code:%d\n",OV20880PD1901_OTP_ERROR_CODE);
			return 0;
		}
		
	
		/*LSC_checksum*/
		checksum = 0;
		for(i = Lsc_addr+1; i < Lsc_checksum_addr; i ++)
		{              
			checksum += vivo_otp_data[i];						
		}  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[Lsc_checksum_addr] != checksum )
		{
			OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("LSC_checksum error!!!   otp_error_code:%d\n", OV20880PD1901_OTP_ERROR_CODE);
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
			OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, OV20880PD1901_OTP_ERROR_CODE);
			return 0;
		}
        /****check if awb out of range[low cct]****/
		R_unit_low = vivo_otp_data[Awb_low_addr];
		R_unit_low = (R_unit_low << 8) | (vivo_otp_data[Awb_low_addr+2]);
		B_unit_low = vivo_otp_data[Awb_low_addr+3];
		B_unit_low = (B_unit_low << 8) | (vivo_otp_data[Awb_low_addr+4]);
		G_unit_low = vivo_otp_data[Awb_low_addr+5];
		G_unit_low = (G_unit_low << 8) | (vivo_otp_data[Awb_low_addr+6]);

		R_golden_low = vivo_otp_data[Awb_low_addr+6];
		R_golden_low = (R_golden_low << 8) | (vivo_otp_data[Awb_low_addr+8]);
		B_golden_low = vivo_otp_data[Awb_low_addr+9];
		B_golden_low = (B_golden_low << 8) | (vivo_otp_data[Awb_low_addr+10]);
		G_golden_low = vivo_otp_data[Awb_low_addr+11];
		G_golden_low = (G_golden_low << 8) | (vivo_otp_data[Awb_low_addr+12]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("cfx_add:R_unit_low=%d, R_golden_low=%d, B_unit_low=%d, B_golden_low=%d, G_unit_low=%d, G_golden_low=%d\n", R_unit_low, R_golden_low, B_unit_low, B_golden_low, G_unit_low, G_golden_low);
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
			OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, OV20880PD1901_OTP_ERROR_CODE);
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
			    OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			    LOG_INF("AF out of range error!!!   otp_error_code:%d   VCM ID = 0x%x AF_inf = %d AF_mac = %d\n", OV20880PD1901_OTP_ERROR_CODE, vivo_otp_data[0x07], AF_inf, AF_mac);
			    return 0;
		    }
		}
		/* tyj add for distinguishing end */
		else{
		    LOG_INF("[tyj++]distinguish vcm VCM ID = 0x%x inf = %d mac = %d\n", vivo_otp_data[0x07], AF_inf, AF_mac);
		    if (diff_inf > 100 || diff_mac > 110 || diff_inf_macro > 500 || diff_inf_macro < 260) {  /*AF code out of range*/
			    OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			    LOG_INF("AF out of range error!!!   otp_error_code:%d   VCM ID = 0x%x AF_inf = %d AF_mac = %d\n", OV20880PD1901_OTP_ERROR_CODE, vivo_otp_data[0x07], AF_inf, AF_mac);
			    return 0;
		    }
		}
		#endif

		/*cfx add for pdaf data start 20161223*/
		/*for(i = 0;i < 1372;i++)
		{
			if(i == 0)
				LOG_INF("read_OV20880PD1901_PDAF_data");
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
		OV20880PD1901_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("invalid otp data. error!!!   otp_error_code:%d\n", OV20880PD1901_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo cfx add end*/
