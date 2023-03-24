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

/*
 * FP5516WE4AF voice coil motor driver
 *
 *
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>

#include "lens_info.h"

#define AF_DRVNAME "FP5516WE4AF_DRV"
#define AF_I2C_SLAVE_ADDR 0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...)                                               \
	pr_info(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;

static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4CurrPosition;
static unsigned long g_u4TargetPosition;

int s4AF_i2c_write_32(struct i2c_client *client, u16 addr, u32 data)
{
	struct i2c_msg msg[1]  = {0,};
	char           buf[6]  = {0,};
	int            ret     = 0;

	buf[0] = (char)(addr >> 8);
	buf[1] = (char)(addr & 0xFF);
	buf[2] = (char)((data >> 24) & 0xFF);
	buf[3] = (char)((data >> 16) & 0xFF);
	buf[4] = (char)((data >> 8) & 0xFF);
	buf[5] = (char)(data & 0xFF);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 6;
	//msg[0].len = ARRAY_SIZE(buf);
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (1 != ret) {
		ret = -1;
		LOG_INF("ois i2c write fail(%d)", ret);
	} else {
		ret = 0;
	}

	return ret;

}

int s4AF_i2c_write_16(struct i2c_client *client, u16 addr, u16 data)//16
{
	struct i2c_msg msg[1]  = {0,};
	char           buf[4]  = {0,};
	int            ret     = 0;

	buf[0] = (char)(addr >> 8);
	buf[1] = (char)(addr & 0xFF);
	buf[2] = (char)(data >> 8);
	buf[3] = (char)(data & 0xFF);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = ARRAY_SIZE(buf);
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
    		LOG_INF("s4AF_i2c_write_16 (%d)", ret);
	if (1 != ret) {
		LOG_INF("s4AF_i2c_write_16 failed(%d)", ret);
	} else {
		ret = 0;
	}

	return ret;

}

int s4AF_i2c_write_8(struct i2c_client *client, u8 addr, u8 data)//6
{
	struct i2c_msg msg[1]  = {0,};
	char           buf[2]  = {0,};
	int            ret     = 0;

	buf[0] = (char)(addr & 0xFF);
	buf[1] = (char)(data & 0xFF);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = ARRAY_SIZE(buf);
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
    	LOG_INF("s4AF_i2c_write_8(%d)", ret);
	if (1 != ret) {
		LOG_INF("s4AF_i2c_write_8 fail(%d)", ret);
	} else {
		ret = 0;
	}

	return ret;

}

int s4AF_i2c_read_32(struct i2c_client *client, u16 addr, u32 *data)
{
	int            ret          = 0;
	struct i2c_msg msg[2];
	u8             addr_buf[2]  = {0,};
	//u8             read_buf[4]  = {0,};
	u32 vRcvBuff = 0;

	addr_buf[0] = (char)(addr >> 8);
	addr_buf[1] = (char)(addr & 0xFF);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = addr_buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 4;
	msg[1].buf = (u8 *)&vRcvBuff;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ARRAY_SIZE(msg) != ret) {
		LOG_INF("ois i2c read fail(%d)", ret);
	} else {
		ret = 0;
	}
	//LOG_OIS_ERR("zycdebug : %d   %d   %d    %d", read_buf[0],read_buf[1],read_buf[2],read_buf[3]);
	*data = vRcvBuff;

	//*data =(((read_buf[0] << 24)  | (read_buf[1] << 16) )| (read_buf[2] << 8) )| read_buf[3];
	//dataTemp = ((dataTemp & 0xFF) << 24 ) + ((dataTemp & 0xFF00) << 8) + ((dataTemp & 0xFF0000) >> 8) + ((dataTemp & 0xFF000000) >> 24)

	return ret;
}


int s4AF_i2c_read_16(struct i2c_client *client, u16 addr, u16 *data)//16
{
	int            ret          = 0;
	struct i2c_msg msg[2];
	u8             addr_buf[2]  = {0,};
	u8             read_buf[2]  = {0,};

	addr_buf[0] = (char)(addr >> 8);
	addr_buf[1] = (char)(addr & 0xFF);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = addr_buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = ARRAY_SIZE(read_buf);
	msg[1].buf = read_buf;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ARRAY_SIZE(msg) != ret) {
		LOG_INF("ois i2c read fail(%d)", ret);
	} else {
		ret = 0;
	}

	*data = (read_buf[1] << 8) | read_buf[0];
        LOG_INF("s4AF_i2c_read_16 =%d  read_buf[0]=0x%x read_buf[1]=0x%x  data=0x%x\n",*data,read_buf[0],read_buf[1],*data);
	return ret;
}

int s4AF_i2c_read_8(struct i2c_client *client, u8 addr, u8 *data)//8
{
	int i4RetValue;
	struct i2c_msg msg[2];
    	u8    addr_buf[1]  = {0,};
	u8    read_buf[1]  = {0,};
	addr_buf[0] = (char)addr;
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = addr_buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = ARRAY_SIZE(read_buf);
	msg[1].buf = read_buf;

	i4RetValue =
		i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	*data = read_buf[0] ;

	if (i4RetValue != 2) {
		LOG_INF("I2C Read failed!!\n");
		return -1;
	}
    
	return 0;
}

static int s4AF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[3] = { 0x03, (char)(a_u2Data >> 8),
		(char)(a_u2Data & 0xFF) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

	return 0;
}



static int i2c_read(u8 a_u2Addr, u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puReadCmd[1] = { (char)(a_u2Addr) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puReadCmd, 1);
	if (i4RetValue != 1) {
		LOG_INF(" I2C write failed!!\n");
		return -1;
	}

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, (char *)a_puBuff, 1);
	if (i4RetValue != 1) {
		LOG_INF(" I2C read failed!!\n");
		return -1;
	}

	return 0;
}

static u8 read_data(u8 addr)
{
	u8 get_byte = 0;

	i2c_read(addr, &get_byte);

	return get_byte;
}

static int s4FP5516WEAF_ReadReg(unsigned short *a_pu2Result)
{
	*a_pu2Result = (read_data(0x03) << 8) + (read_data(0x04) & 0xff);

	return 0;
}
#if 0
static int s4AF_ReadReg(unsigned short *a_pu2Result)
{
	int i4RetValue = 0;
	char pBuff[2];

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, pBuff, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C read failed!!\n");
		return -1;
	}

	*a_pu2Result = (((u16)pBuff[0]) << 4) + (pBuff[1] >> 4);

	return 0;
}

static int s4AF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[3] = {0x03,(char)(a_u2Data >> 8), (char)(a_u2Data & 0xFF)};

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

	return 0;
}
#endif
static inline int getAFInfo(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo,
			 sizeof(struct stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

/* initAF include driver initialization and standby mode */
static int initAF(void)
{
       //u8 codevalue0 = 0;
       //u8 codevalue1 = 0;

	LOG_INF("+  g_pAF_Opened =%d\n",*g_pAF_Opened);

	if (*g_pAF_Opened == 1) {

		int i4RetValue;
   
//		char puSendCmd1[2] = {(char)(0x02), (char)(0x01)};//Power down
		char puSendCmd2[2] = {(char)(0x02), (char)(0x00)};//Power up and Delay 100us
		char puSendCmd3[2] = {(char)(0x02), (char)(0x02)};//Select ringing control=AESC mode
		char puSendCmd4[2] = {(char)(0x06), (char)(0x40)};// AESC Setting.  sle=1*acc
		char puSendCmd5[2] = {(char)(0x07), (char)(0x65)};//Set Tres=10.0mS. 
		char puSendCmd6[2] = {(char)(0x08), (char)(0x01)};//Set Decay=0.8mS.

		g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
		g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;
#if 0
		i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd1, 2);
		if (i4RetValue < 0)
			LOG_INF(" puSendCmd1 send failed.\n");
             s4AF_i2c_read_8(g_pstAF_I2Cclient,0x02,&codevalue);
             	LOG_INF("initAF Power down =%d\n",codevalue);
#endif
		i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
		if (i4RetValue < 0)
			LOG_INF(" puSendCmd2 send failed.\n");
             mdelay(1);
             //s4AF_i2c_read_8(g_pstAF_I2Cclient,0x02,&codevalue0);
             	//LOG_INF("initAF Power up =%d\n",codevalue0);

		i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
		if (i4RetValue < 0)
			LOG_INF(" puSendCmd3 send failed.\n");
             //s4AF_i2c_read_8(g_pstAF_I2Cclient,0x02,&codevalue0);
             	//LOG_INF("initAF Select ringing control =%d\n",codevalue0);

		i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd4, 2);
		if (i4RetValue < 0)
			LOG_INF(" puSendCmd4 send failed.\n");
             //s4AF_i2c_read_8(g_pstAF_I2Cclient,0x06,&codevalue0);
             	//LOG_INF("initAF AESC Setting =%d\n",codevalue0);

		i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd5, 2);
		if (i4RetValue < 0)
			LOG_INF(" puSendCmd5 send failed.\n");
             //s4AF_i2c_read_8(g_pstAF_I2Cclient,0x07,&codevalue0);
             	//LOG_INF("initAF Set Tres =%d\n",codevalue0);

		i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd6, 2);
		if (i4RetValue < 0)
			LOG_INF(" puSendCmd6 send failed.\n");
             //s4AF_i2c_read_8(g_pstAF_I2Cclient,0x08,&codevalue0);
             	//LOG_INF("initAF Set Decay =%d\n",codevalue0);

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}
    
       //s4AF_i2c_read_8(g_pstAF_I2Cclient,0x05,&codevalue1);
       //LOG_INF("initAF 05 =0x%x\n",codevalue1);

       //s4AF_i2c_read_8(g_pstAF_I2Cclient,0x03,&codevalue1);
       //s4AF_i2c_read_8(g_pstAF_I2Cclient,0x04,&codevalue0);
       //LOG_INF("initAF code  0304 =%d\n",(codevalue1<<8|codevalue0));

	LOG_INF("-\n");

	return 0;
}

/* moveAF only use to control moving the motor */
static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;

       u8 codevalue0 = 0;
       //u8 codevalue1 = 0;
        unsigned short readPos;
    	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	LOG_INF("g_u4AF_INF =%d  g_u4AF_MACRO=%d\n",g_u4AF_INF,g_u4AF_MACRO);
	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)  ) {
		printk("out of range\n");
		return -EINVAL;
	}

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

 	LOG_INF("moveAF a_u4Position =%d  MSB=0x%x LSB=0x%x  a_u4Position=0x%x\n",g_u4TargetPosition,g_u4TargetPosition>>8,g_u4TargetPosition&0xff,g_u4TargetPosition);
#if 0
	if ((s4AF_i2c_write_8(g_pstAF_I2Cclient,0x03,(g_u4TargetPosition>>8)) == 0) && (s4AF_i2c_write_8(g_pstAF_I2Cclient,0x04,(g_u4TargetPosition&0xFF)) == 0)) {
        	LOG_INF("moving the motor sucess\n");
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
		ret = 0;
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
		ret = -1;
	}
       s4AF_i2c_read_8(g_pstAF_I2Cclient,0x03,&codevalue1);
       s4AF_i2c_read_8(g_pstAF_I2Cclient,0x04,&codevalue0);
 	LOG_INF("moveAF read code =%d  0x%x 0x%x  0x%x\n",(codevalue1<<8|codevalue0),(codevalue1<<8|codevalue0),codevalue1,codevalue0);
       mdelay(20);
#else
       s4AF_i2c_read_8(g_pstAF_I2Cclient,0x05,&codevalue0);
       LOG_INF("s4FP5516WEAF_ReadReg 0x05 %d \n",codevalue0);
	if (s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
        	LOG_INF("moving the motor sucess\n");
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
		ret = 0;
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
		ret = -1;
	}
       //mdelay(100);
       ret = s4FP5516WEAF_ReadReg(&readPos);

       LOG_INF("s4FP5516WEAF_ReadReg  -0x0304 %d  0x%x\n",readPos,readPos);
#endif

	return ret;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long FP5516WE4AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
		    unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue =
			getAFInfo((__user struct stAF_MotorInfo *)(a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int FP5516WE4AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
       //u8 codevalue1 = 0;
	LOG_INF("Start\n");


	if (*g_pAF_Opened == 2) {
		LOG_INF("Wait\n");
            	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	       g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;
		s4AF_i2c_write_8(g_pstAF_I2Cclient,0x02,0x01); /* Power down mode */

        //s4AF_i2c_read_8(g_pstAF_I2Cclient,0x02,&codevalue1);
        //LOG_INF("codevalue1 =%d \n",codevalue1);
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

int FP5516WE4AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
			  spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	initAF();

	return 1;
}

int FP5516WE4AF_GetFileName(unsigned char *pFileName)
{
	#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char FilePath[256];
	char *FileString;

	sprintf(FilePath, "%s", __FILE__);
	FileString = strrchr(FilePath, '/');
	*FileString = '\0';
	FileString = (strrchr(FilePath, '/') + 1);
	strncpy(pFileName, FileString, AF_MOTOR_NAME);
	LOG_INF("FileName : %s\n", pFileName);
	#else
	pFileName[0] = '\0';
	#endif
	return 1;
}
