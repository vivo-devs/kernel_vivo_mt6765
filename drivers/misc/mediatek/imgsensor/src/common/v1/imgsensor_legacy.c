// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "kd_camera_typedef.h"
#include "imgsensor_i2c.h"

#ifdef IMGSENSOR_LEGACY_COMPAT
void kdSetI2CSpeed(u16 i2cSpeed)
{

}

int iReadRegI2C(
	u8 *a_pSendData,
	u16 a_sizeSendData,
	u8 *a_pRecvData,
	u16 a_sizeRecvData,
	u16 i2cId)
{
	return imgsensor_i2c_read(
	    pgi2c_cfg_legacy,
	    a_pSendData,
	    a_sizeSendData,
	    a_pRecvData,
	    a_sizeRecvData,
	    i2cId,
	    IMGSENSOR_I2C_SPEED);
}

//IIC is grounded, the camera can be opend ,check iic timeout
int check_i2c_timeout(u16 addr, u16 i2cid)
{
	int ret, temp;
	u8 pu_send_cmd[2];
	u16 get_byte;
	struct IMGSENSOR_I2C_CFG *pdevice = imgsensor_i2c_get_device();
	if ( pdevice == NULL){
	    pr_err("[%s] check_i2c_timeout pdevice =null \n",__FUNCTION__);
	    return IMGSENSOR_RETURN_ERROR;
	    }

	temp = pdevice->pinst->pi2c_client->adapter->timeout;
	pdevice->pinst->pi2c_client->adapter->timeout = HZ/20;

	if(addr&0xff00){
	    get_byte = 2;
	    pu_send_cmd[0] = (addr>> 8);
	    pu_send_cmd[1] = (addr & 0xff);
	} else {
	    get_byte = 1;
	    pu_send_cmd[0] = (addr & 0xff);
	}
	ret = iReadRegI2C(pu_send_cmd,get_byte,(u8*)&get_byte,1,i2cid);

	pdevice->pinst->pi2c_client->adapter->timeout = temp;
	pr_info("[%s]  read get_byte = 0x%x  ret =%d \n",__FUNCTION__,get_byte,ret);
	return (ret==-2)?1:0;
}
//IIC is grounded, the camera can be opend ,check iic timeout

int iReadRegI2CTiming(
	u8 *a_pSendData,
	u16 a_sizeSendData,
	u8 *a_pRecvData,
	u16 a_sizeRecvData,
	u16 i2cId,
	u16 timing)
{
	return imgsensor_i2c_read(
	    pgi2c_cfg_legacy,
	    a_pSendData,
	    a_sizeSendData,
	    a_pRecvData,
	    a_sizeRecvData,
	    i2cId,
	    timing);
}

int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId)
{
	return imgsensor_i2c_write(
	    pgi2c_cfg_legacy,
	    a_pSendData,
	    a_sizeSendData,
	    a_sizeSendData,
	    i2cId,
	    IMGSENSOR_I2C_SPEED);
}

int iWriteRegI2CTiming(
	u8 *a_pSendData,
	u16 a_sizeSendData,
	u16 i2cId,
	u16 timing)
{
	return imgsensor_i2c_write(
	    pgi2c_cfg_legacy,
	    a_pSendData,
	    a_sizeSendData,
	    a_sizeSendData,
	    i2cId,
	    timing);
}

int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId)
{
	return imgsensor_i2c_write(
	    pgi2c_cfg_legacy,
	    pData,
	    bytes,
	    bytes,
	    i2cId,
	    IMGSENSOR_I2C_SPEED);
}

int iBurstWriteReg_multi(
	u8 *pData,
	u32 bytes,
	u16 i2cId,
	u16 transfer_length,
	u16 timing)
{
	return imgsensor_i2c_write(
	    pgi2c_cfg_legacy,
	    pData,
	    bytes,
	    transfer_length,
	    i2cId,
	    timing);
}


#endif

