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
#include "hi556lcepd2140frontmipiraw_Sensor.h"

#define PFX "hi556lcepd2140front_EEPROM_OTP"

#define LOG_INF(format,  args...)        pr_debug(PFX "[%s] " format,  __func__,  ##args)

#define UNUSED_DATA_LENGTH                 0x0400
#define COMMON_CHECKSUM_LENGTH           1

#define AWB_VALUE_LENGTH                 16
#define AWB_GROUP_FLAG_SEG               (0x0426 - UNUSED_DATA_LENGTH)
#define AWB_HEAD_ADDR_GROUP_1            (0x0427 - UNUSED_DATA_LENGTH)
#define AWB_CHECKSUM_ADDR_GROUP_1        (0x0437 - UNUSED_DATA_LENGTH)
#define AWB_CHECKSUM_ADDR_MAKEUP_OFFSET  5

#define INFO_VALUE_LENGTH                11     // 5 reserve bits cannot be caculate in checksum
#define INFO_GROUP_FLAG_SEG              (0x0401 - UNUSED_DATA_LENGTH)
#define INFO_HEAD_ADDR_GROUP_1           (0x0402 - UNUSED_DATA_LENGTH)
#define INFO_CHECKSUM_ADDR_GROUP_1       (0x040D - UNUSED_DATA_LENGTH)


#define LSC_VALUE_LENGTH                 1868
#define LSC_GROUP_FLAG_SEG               (0x045A - UNUSED_DATA_LENGTH)
#define LSC_HEAD_ADDR_GROUP_1            (0x045B - UNUSED_DATA_LENGTH)
#define LSC_CHECKSUM_ADDR_GROUP_1        (0x0BA7 - UNUSED_DATA_LENGTH)


#define SN_VALUE_LENGTH                  12
#define SN_GROUP_FLAG_SEG                (0x1A42 - UNUSED_DATA_LENGTH)
#define SN_HEAD_ADDR_GROUP_1             (0x1A43 - UNUSED_DATA_LENGTH)
#define SN_CHECKSUM_ADDR_GROUP_1         (0x1A4F - UNUSED_DATA_LENGTH)


#define MAX_MEM_BLOCK_SIZE              (INFO_VALUE_LENGTH+AWB_VALUE_LENGTH+LSC_VALUE_LENGTH+SN_VALUE_LENGTH+4+4)
#define MAX_GROUP_INDEX                 3

static unsigned char otp_data_kernel_read_hi556lcepd2140front[MAX_MEM_BLOCK_SIZE*MAX_GROUP_INDEX];
unsigned char otp_data_vendor_read_hi556lcepd2140front[INFO_VALUE_LENGTH+AWB_VALUE_LENGTH+LSC_VALUE_LENGTH+SN_VALUE_LENGTH+2];
/**************/
/****vivo cfx add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_SUB_OTP_DATA_SIZE 0x1669 /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
/*#define AF_RANGE_GOT*/  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_SUB_EEPROM_WRITE_ID 0x40
#define VIVO_SUB_I2C_SPEED 400
/*#define VIVO_SUB_VENDOR_SUNNY 0x01*/
/*#define VIVO_SUB_VENDOR_TRULY 0x02*/
/*#define VIVO_SUB_VENDOR_QTECH 0x57*/
#define VIVO_SUB_VENDOR_LCE 0x57
/*#define VIVO_SUB_VENDOR_OFILM 0x09*/
#define VIVO_SUB_VENDOR_LENS_ID 0x24
#define VIVO_SUB_VENDOR_PLATFROM_ID 0x01
/*#define VIVO_SUB_VENDOR_VCM_ID 0x00*/
/*#define VIVO_SUB_VENDOR_DRIVERIC_ID 0x00*/
/*#define VIVO_SUB_VENDOR_PLATFORM_ID 0x00*/

otp_error_code_t SUB_OTP_ERROR_CODE_HI556PD2140 = OTP_ERROR_CODE_NORMAL;

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

static int HI556PD2140_otp_read_setting_init_continuous_read(kal_uint16 addr)
{
    kal_uint32 i;

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
    for (i = 0; i < MAX_MEM_BLOCK_SIZE*MAX_GROUP_INDEX ; i++) {
	otp_data_kernel_read_hi556lcepd2140front[i] = read_cmos_sensor(0x108);
    }
    write_cmos_sensor_8(0x0a00, 0x00);
    mdelay(10);
    write_cmos_sensor_8(0x003e, 0x00);
    write_cmos_sensor_8(0x0a00, 0x01);
    return 0;
}

typedef enum{
    OTP_TYPE_INFO = 0,
    OTP_TYPE_AWB = 1,
    OTP_TYPE_LSC = 2,
    OTP_TYPE_SN = 3,
    OTP_TYPE_MAX = 4,
} OTP_DATA_TYPE;

typedef struct OTPMapUnit {
    kal_uint32 ValueLength;
    kal_uint32 GroupFlagSeg;
    kal_uint32 DataHeadAddr;
    kal_uint32 CheckSumAddrGroup1;
} OTPMapUnit;

kal_uint32 HI556_GROUP_SEG[3] = {0x01, 0x13, 0x37};

OTPMapUnit OtpMapInfo[OTP_TYPE_MAX] = {
    {INFO_VALUE_LENGTH,      INFO_GROUP_FLAG_SEG,  INFO_HEAD_ADDR_GROUP_1, INFO_CHECKSUM_ADDR_GROUP_1 },
    {AWB_VALUE_LENGTH,       AWB_GROUP_FLAG_SEG,   AWB_HEAD_ADDR_GROUP_1,  AWB_CHECKSUM_ADDR_GROUP_1  },
    {LSC_VALUE_LENGTH,       LSC_GROUP_FLAG_SEG,   LSC_HEAD_ADDR_GROUP_1,  LSC_CHECKSUM_ADDR_GROUP_1  },
    {SN_VALUE_LENGTH,        SN_GROUP_FLAG_SEG,    SN_HEAD_ADDR_GROUP_1,   SN_CHECKSUM_ADDR_GROUP_1   },
};

kal_int16 check_checksum_info(OTP_DATA_TYPE ReadOTPType)
{
    kal_int16  IsValid           = 0x0;
    kal_uint32 CaculatedChecksum = 0x0;
    kal_int16  GroupIndex        = 0x0;
    kal_uint32 KernelReadHeadAddr   = 0x0;
    kal_uint32 TmpCheckSumAddr   = 0x0;
    kal_uint32 ReadOffset             = 0x0;

    LOG_INF("ReadOTPType is [%d]", ReadOTPType);
    for (GroupIndex = 0; GroupIndex < MAX_GROUP_INDEX; GroupIndex++) {
	if (HI556_GROUP_SEG[GroupIndex] == otp_data_kernel_read_hi556lcepd2140front[OtpMapInfo[ReadOTPType].GroupFlagSeg]) {
	    CaculatedChecksum = 0x0;
	    KernelReadHeadAddr = OtpMapInfo[ReadOTPType].DataHeadAddr \
			+ GroupIndex * (OtpMapInfo[ReadOTPType].ValueLength + COMMON_CHECKSUM_LENGTH);
	for (ReadOffset = 0; ReadOffset < OtpMapInfo[ReadOTPType].ValueLength; ReadOffset++) {
		CaculatedChecksum += otp_data_kernel_read_hi556lcepd2140front[KernelReadHeadAddr + ReadOffset];
	}
	    CaculatedChecksum = CaculatedChecksum % 0xFF + 1;
	    TmpCheckSumAddr = 0x0;
	    TmpCheckSumAddr = OtpMapInfo[ReadOTPType].CheckSumAddrGroup1 \
		    + GroupIndex * (OtpMapInfo[ReadOTPType].ValueLength);
	if (CaculatedChecksum == otp_data_kernel_read_hi556lcepd2140front[TmpCheckSumAddr +  GroupIndex * COMMON_CHECKSUM_LENGTH]) {
		IsValid = 1;
		LOG_INF("ReadOTPType is [%d],GroupSelected[%d]", ReadOTPType, GroupIndex+1);
		if (OtpMapInfo[ReadOTPType].ValueLength == SN_VALUE_LENGTH) {
		    otp_data_vendor_read_hi556lcepd2140front[INFO_VALUE_LENGTH+AWB_VALUE_LENGTH+LSC_VALUE_LENGTH+SN_VALUE_LENGTH+1] =
			    otp_data_kernel_read_hi556lcepd2140front[TmpCheckSumAddr +  GroupIndex * COMMON_CHECKSUM_LENGTH];
		    otp_data_vendor_read_hi556lcepd2140front[INFO_VALUE_LENGTH+AWB_VALUE_LENGTH+LSC_VALUE_LENGTH+SN_VALUE_LENGTH] =
			    HI556_GROUP_SEG[GroupIndex-1];
		}
		return GroupIndex + 1;
	} else {
		IsValid = 0;
	}
		    LOG_INF("Read fail! CheckSum is not matched!!");
		    LOG_INF("CaculatedChecksum is [0x%x], ReadedFromKernelChecksum = [0x%x]",
		    CaculatedChecksum, otp_data_kernel_read_hi556lcepd2140front[TmpCheckSumAddr +  GroupIndex * 1]);
	break;
	}
    }
    LOG_INF("ReadOTPType is [%d],GroupSelected[%d]", ReadOTPType, GroupIndex+1);
    return IsValid;
}

bool get_otp_single_unit_map(unsigned char *pOutput, OTP_DATA_TYPE ReadOTPType)
{
	kal_uint32 KernelReadHeadAddr = 0x0;
	kal_int16 GroupIndex = 0x0;
    if (pOutput == NULL) {
	return false;
    }
    if (ReadOTPType > (kal_uint32)OTP_TYPE_MAX) {
	return false;

    }
	// |flag|valuelength|checksumlength|valuelength|checksumlength|
    KernelReadHeadAddr = OtpMapInfo[ReadOTPType].DataHeadAddr;
    GroupIndex = check_checksum_info(ReadOTPType);
	LOG_INF("GroupIndex read from check_checksum_info is [%d]", GroupIndex);
    if (GroupIndex) {
	GroupIndex--;
	KernelReadHeadAddr += GroupIndex*(OtpMapInfo[ReadOTPType].ValueLength + COMMON_CHECKSUM_LENGTH);
	memcpy(pOutput, &otp_data_kernel_read_hi556lcepd2140front[KernelReadHeadAddr],
		    OtpMapInfo[ReadOTPType].ValueLength);
	return true;
    }
    return false;
    }

int vivo_sub_otp_read_hi556lcepd2140front(void)
{
    kal_int16 OTPType = 0;
    kal_bool result = true;
    kal_uint32 HEADPoint = 0;

    HI556PD2140_otp_read_setting_init_continuous_read(UNUSED_DATA_LENGTH);

    for (OTPType = 0; OTPType < OTP_TYPE_MAX; OTPType++) {
	LOG_INF("OTPType neet to read is [%d]", OTPType);
	result &= \
	get_otp_single_unit_map((unsigned char *)&otp_data_vendor_read_hi556lcepd2140front[HEADPoint], (kal_uint32)OTPType);
	HEADPoint += OtpMapInfo[OTPType].ValueLength;
	LOG_INF("Checksum of part [%d] = [%d]", OTPType, result);
    }
    return true;
}