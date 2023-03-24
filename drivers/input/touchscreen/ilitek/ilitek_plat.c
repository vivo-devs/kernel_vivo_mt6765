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
#include "../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
#include "ilitek_fw.h"



extern unsigned int is_atboot;

/* by tp team for bianyi
extern unsigned int os_boot_puresys;
static bool boot_mode_normal = true;
extern enum boot_mode_t get_boot_mode(void);
*/
extern unsigned int report_lcm_id(void);



#define DTS_INT_GPIO	"ilitek,irq-gpio"
#define DTS_RESET_GPIO	"ilitek,reset-gpio"
#define DTS_OF_NAME		"vivo,ts-spi"

void ilitek_plat_tp_reset(void)
{
	ipio_info("edge delay = %d\n", idev->rst_edge_delay);
	gpio_direction_output(idev->tp_rst, 1);
	mdelay(1);
	gpio_set_value(idev->tp_rst, 0);
	mdelay(5);
	gpio_set_value(idev->tp_rst, 1);
	mdelay(idev->rst_edge_delay);
}

void ilitek_plat_input_register(void)
{
	ipio_info();

	idev->input = input_allocate_device();
	if (ERR_ALLOC_MEM(idev->input)) {
		ipio_err("Failed to allocate touch input device\n");
		input_free_device(idev->input);
		return;
	}

	idev->input->name = idev->hwif->name;
	idev->input->phys = idev->phys;
	idev->input->dev.parent = idev->dev;
	idev->input->id.bustype = idev->hwif->bus_type;

	/* set the supported event type for input device */
	set_bit(EV_ABS, idev->input->evbit);
	set_bit(EV_SYN, idev->input->evbit);
	set_bit(EV_KEY, idev->input->evbit);
	set_bit(BTN_TOUCH, idev->input->keybit);
	set_bit(BTN_TOOL_FINGER, idev->input->keybit);
	set_bit(INPUT_PROP_DIRECT, idev->input->propbit);

	input_set_abs_params(idev->input, ABS_MT_POSITION_X, TOUCH_SCREEN_X_MIN, idev->panel_wid - 1, 0, 0);
	input_set_abs_params(idev->input, ABS_MT_POSITION_Y, TOUCH_SCREEN_Y_MIN, idev->panel_hei - 1, 0, 0);
	input_set_abs_params(idev->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(idev->input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	if (MT_PRESSURE)
		input_set_abs_params(idev->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	if (MT_B_TYPE) {
#if KERNEL_VERSION(3, 7, 0) <= LINUX_VERSION_CODE
		input_mt_init_slots(idev->input, MAX_TOUCH_NUM, INPUT_MT_DIRECT);
#else
		input_mt_init_slots(idev->input, MAX_TOUCH_NUM);
#endif /* LINUX_VERSION_CODE */
	} else {
		input_set_abs_params(idev->input, ABS_MT_TRACKING_ID, 0, MAX_TOUCH_NUM, 0, 0);
	}

	/* Gesture keys register */
	input_set_capability(idev->input, EV_KEY, KEY_POWER);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_UP);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_LEFT);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_RIGHT);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_O);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_E);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_M);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_W);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_S);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_V);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_C);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_F);

	__set_bit(KEY_GESTURE_POWER, idev->input->keybit);
	__set_bit(KEY_GESTURE_UP, idev->input->keybit);
	__set_bit(KEY_GESTURE_DOWN, idev->input->keybit);
	__set_bit(KEY_GESTURE_LEFT, idev->input->keybit);
	__set_bit(KEY_GESTURE_RIGHT, idev->input->keybit);
	__set_bit(KEY_GESTURE_O, idev->input->keybit);
	__set_bit(KEY_GESTURE_E, idev->input->keybit);
	__set_bit(KEY_GESTURE_M, idev->input->keybit);
	__set_bit(KEY_GESTURE_W, idev->input->keybit);
	__set_bit(KEY_GESTURE_S, idev->input->keybit);
	__set_bit(KEY_GESTURE_V, idev->input->keybit);
	__set_bit(KEY_GESTURE_Z, idev->input->keybit);
	__set_bit(KEY_GESTURE_C, idev->input->keybit);
	__set_bit(KEY_GESTURE_F, idev->input->keybit);

	/* register the input device to input sub-system */
	if (input_register_device(idev->input) < 0) {
		ipio_err("Failed to register touch input device\n");
		input_unregister_device(idev->input);
		input_free_device(idev->input);
	}
}

void ilitek_plat_regulator_power_on(bool status)
{
	ipio_info("%s\n", status ? "POWER ON" : "POWER OFF");

	if (status) {
		if (idev->vdd) {
			if (regulator_enable(idev->vdd) < 0)
				ipio_err("regulator_enable VDD fail\n");
		}
		if (idev->vcc) {
			if (regulator_enable(idev->vcc) < 0)
				ipio_err("regulator_enable VCC fail\n");
		}
	} else {
		if (idev->vdd) {
			if (regulator_disable(idev->vdd) < 0)
				ipio_err("regulator_enable VDD fail\n");
		}
		if (idev->vcc) {
			if (regulator_disable(idev->vcc) < 0)
				ipio_err("regulator_enable VCC fail\n");
		}
	}
	atomic_set(&idev->ice_stat, DISABLE);
	mdelay(5);
}

static void ilitek_plat_regulator_power_init(void)
{
	const char *vdd_name = "vdd";
	const char *vcc_name = "vcc";

	idev->vdd = regulator_get(idev->dev, vdd_name);
	if (ERR_ALLOC_MEM(idev->vdd)) {
		ipio_err("regulator_get VDD fail\n");
		idev->vdd = NULL;
	}
	if (regulator_set_voltage(idev->vdd, VDD_VOLTAGE, VDD_VOLTAGE) < 0)
		ipio_err("Failed to set VDD %d\n", VDD_VOLTAGE);

	idev->vcc = regulator_get(idev->dev, vcc_name);
	if (ERR_ALLOC_MEM(idev->vcc)) {
		ipio_err("regulator_get VCC fail.\n");
		idev->vcc = NULL;
	}
	if (regulator_set_voltage(idev->vcc, VCC_VOLTAGE, VCC_VOLTAGE) < 0)
		ipio_err("Failed to set VCC %d\n", VCC_VOLTAGE);

	ilitek_plat_regulator_power_on(true);
}

static int ilitek_plat_gpio_register(void)
{
	int ret = 0;
	u32 flag;
	struct device_node *dev_node = idev->dev->of_node;

	idev->tp_int = of_get_named_gpio_flags(dev_node, DTS_INT_GPIO, 0, &flag);
	idev->tp_rst = of_get_named_gpio_flags(dev_node, DTS_RESET_GPIO, 0, &flag);

	ipio_info("TP INT: %d\n", idev->tp_int);
	ipio_info("TP RESET: %d\n", idev->tp_rst);

	if (!gpio_is_valid(idev->tp_int)) {
		ipio_err("Invalid INT gpio: %d\n", idev->tp_int);
		return -EBADR;
	}

	if (!gpio_is_valid(idev->tp_rst)) {
		ipio_err("Invalid RESET gpio: %d\n", idev->tp_rst);
		return -EBADR;
	}

	ret = gpio_request(idev->tp_int, "TP_INT");
	if (ret < 0) {
		ipio_err("Request IRQ GPIO failed, ret = %d\n", ret);
		gpio_free(idev->tp_int);
		ret = gpio_request(idev->tp_int, "TP_INT");
		if (ret < 0) {
			ipio_err("Retrying request INT GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

	ret = gpio_request(idev->tp_rst, "TP_RESET");
	if (ret < 0) {
		ipio_err("Request RESET GPIO failed, ret = %d\n", ret);
		gpio_free(idev->tp_rst);
		ret = gpio_request(idev->tp_rst, "TP_RESET");
		if (ret < 0) {
			ipio_err("Retrying request RESET GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

out:
	gpio_direction_input(idev->tp_int);
	return ret;
}

void ilitek_plat_irq_disable(void)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (atomic_read(&idev->irq_stat) == DISABLE)
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	disable_irq_nosync(idev->irq_num);
	atomic_set(&idev->irq_stat, DISABLE);
	VTI("Disable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

void ilitek_plat_irq_enable(void)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (atomic_read(&idev->irq_stat) == ENABLE)
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	enable_irq(idev->irq_num);
	atomic_set(&idev->irq_stat, ENABLE);
	ipio_debug("Enable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

static irqreturn_t ilitek_plat_isr_top_half(int irq, void *dev_id)
{
	/*ipio_debug("report: %d, rst: %d, fw: %d, switch: %d, mp: %d, sleep: %d, esd: %d\n",
			idev->report,
			atomic_read(&idev->tp_reset),
			atomic_read(&idev->fw_stat),
			atomic_read(&idev->tp_sw_mode),
			atomic_read(&idev->mp_stat),
			atomic_read(&idev->tp_sleep),
			atomic_read(&idev->esd_stat));*/
	VTD("Enter interrupter");
	if (irq != idev->irq_num) {
		ipio_err("Incorrect irq number (%d)\n", irq);
		return IRQ_NONE;
	}

	if (atomic_read(&idev->mp_int_check) == ENABLE) {
		atomic_set(&idev->mp_int_check, DISABLE);
		ipio_info("Get an INT for mp, ignore\n");
		wake_up(&(idev->inq));
		//return IRQ_HANDLED;
	}

	/*if (!idev->report || atomic_read(&idev->tp_reset) ||
		atomic_read(&idev->fw_stat) || atomic_read(&idev->tp_sw_mode) ||
		atomic_read(&idev->mp_stat) || atomic_read(&idev->tp_sleep) ||
		atomic_read(&idev->esd_stat)) {
			ipio_debug("ignore interrupt !\n");
			return IRQ_HANDLED;
	}
	return IRQ_WAKE_THREAD;*/
	vtsIrqBtmThreadWake();
	
	return IRQ_HANDLED;
}

/*static irqreturn_t ilitek_plat_isr_bottom_half(int irq, void *dev_id)
{
	if (mutex_is_locked(&idev->touch_mutex)) {
		ipio_debug("touch is locked, ignore\n");
		return IRQ_HANDLED;
	}
	mutex_lock(&idev->touch_mutex);
	ilitek_tddi_report_handler();
	mutex_unlock(&idev->touch_mutex);
	return IRQ_HANDLED;
}*/

void ilitek_vivo_isr_bottom_half(void *work)
{
	mutex_lock(&idev->touch_mutex);
	ilitek_tddi_report_handler();
	mutex_unlock(&idev->touch_mutex);
	return;
}

static int ilitek_plat_irq_register(void)
{
	int ret = 0;

	idev->irq_num  = gpio_to_irq(idev->tp_int);

	ipio_info("idev->irq_num = %d\n", idev->irq_num);

	/*ret = devm_request_threaded_irq(idev->dev, idev->irq_num,
				   ilitek_plat_isr_top_half,
				   ilitek_plat_isr_bottom_half,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ilitek", NULL);*/
	ret = vivoTsInterruptRegister(idev->irq_num, NULL, ilitek_plat_isr_top_half, IRQF_TRIGGER_FALLING/* | IRQF_NO_SUSPEND |IRQF_ONESHOT*/, idev);			   

	if (ret != 0)
		ipio_err("Failed to register irq handler, irq = %d, ret = %d\n", idev->irq_num, ret);

	atomic_set(&idev->irq_stat, ENABLE);
	return ret;
}

/*#ifdef CONFIG_FB
static int ilitek_plat_notifier_fb(struct notifier_block *self, unsigned long event, void *data)
{
	int *blank;
	struct fb_event *evdata = data;

	ipio_info("Notifier's event = %ld\n", event);

	
		//FB_EVENT_BLANK(0x09): A hardware display blank change occurred.
		//FB_EARLY_EVENT_BLANK(0x10): A hardware display blank early change occurred.
	
	if (evdata && evdata->data) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_POWERDOWN:
			if (TP_SUSPEND_PRIO) {
				if (event != FB_EARLY_EVENT_BLANK)
					return NOTIFY_DONE;
			} else {
				if (event != FB_EVENT_BLANK)
					return NOTIFY_DONE;
			}
			ilitek_tddi_sleep_handler(TP_SUSPEND);
			break;
		case FB_BLANK_UNBLANK:
		case FB_BLANK_NORMAL:
			if (event == FB_EVENT_BLANK)
				ilitek_tddi_sleep_handler(TP_RESUME);
			break;
		default:
			ipio_err("Unknown event, blank = %d\n", *blank);
			break;
		}
	}
	return NOTIFY_OK;
}
#else
static void ilitek_plat_early_suspend(struct early_suspend *h)
{
	ilitek_tddi_sleep_handler(TP_SUSPEND);
}

static void ilitek_plat_late_resume(struct early_suspend *h)
{
	ilitek_tddi_sleep_handler(TP_RESUME);
}
#endif

static void ilitek_plat_sleep_init(void)
{
#ifdef CONFIG_FB
	ipio_info("Init notifier_fb struct\n");
	idev->notifier_fb.notifier_call = ilitek_plat_notifier_fb;

	if (fb_register_client(&idev->notifier_fb))
		ipio_err("Unable to register notifier_fb\n");

#else
	ipio_info("Init eqarly_suspend struct\n");
	idev->early_suspend.suspend = ilitek_plat_early_suspend;
	idev->early_suspend.resume = ilitek_plat_late_resume;
	idev->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	register_early_suspend(&idev->early_suspend);
#endif
}*/

static int ilitek_plat_probe(void)
{
	struct vivo_ts_struct *vts_data = NULL;
	int ret;
	
	VTI("start..");

	vts_data = vivoTsAlloc();	
	if(vts_data == NULL) {	
		VTE("vts_data alloc fail");
		goto free_vtsdata;
	}
	
	ret = report_lcm_id();
	vts_data->busType = VTS_BUS_SPI;
	vts_data->spiClient = idev->spi;
	vts_data->tsDimensionX = 720;		//pdata->x_max; 
	vts_data->tsDimensionY = 1544;		//pdata->y_max; 
	if (ret == 0x60)
		vts_data->tsDimensionY = 1560;
	vivoTsSetTxRxNum(18, 32);	
	vts_data->isIndependentKeyIc = 0;
	vts_data->hasFingerPrint = 1;	
	vts_data->imeiWriteSupport = UNSUPPORT;

	vts_data->irqBtmType = VTS_IRQ_BTM_TYPE_RT_THREAD;
	vts_data->irqBtmThreadHandler = ilitek_vivo_isr_bottom_half;
	//vts_data->irqBtmType = VTS_IRQ_BTM_TYPE_ONESHOOT_THREAD;	
	
	vts_data->icNum = VTS_IC_ILI9881;
	vts_data->mtkOrQcom = VTS_PLATFORM_MTK_NEW;
	vts_data->sensorTestKey =  "com.ilitek.touchscreen.tptest:MainActivity:com.ilitek.touchscreen.action.tptest:0:ILITEK_TpTestResult";
	vts_data->lcmNoiseTestKey = "com.ilitek.touchscreen.noise:MainActivity:null:null:null"; 
	vts_data->rawdataTestKey = "com.ilitek.touchscreen.noise:DataShowActivity:null:null:null";
	vts_data->RFTestKey = "com.ilitek.touchscreen.noise:BspTestActivity:com.ilitek.touchscreen.action.noisetest:0:ILITEK_TpTestResult";

	vts_data->getIcMode = bbk_ili_get_ic_mode;
	vts_data->writeImei= NULL;
	vts_data->readImei= NULL;
	vts_data->icModeChange = bbk_ili_mode_change;
	vts_data->updateFirmware = bbk_ili_fw_update;
	vts_data->setGestureSwitch = NULL;		//if all gusture switch default on,this function no use 
	vts_data->getRawOrDiffData = bbk_ili_get_rawordiff_data;
	vts_data->getIcFirmwareOrConfigVersion = bbk_ili_get_fw_version;	
	vts_data->getModuleId = vivo_ili_module_id;
	vts_data->getGesturePointData = bbk_ili_gesture_point_get; 
	vts_data->setChargerFlagSwitch = bbk_ili_set_charger_bit;	
	vts_data->getTsPinValue = NULL; 
	vts_data->setEdgeRestainSwitch = iliSetEdgeRestainSwitch;	
	vts_data->getLcmId = get_lcm_id;	
	vts_data->idleEnableOrDisable = iliIdleEnableOrDisable;	
	vts_data->getHeaderFileFWVersionOrConfig = bbk_ili_get_header_file_version;	
	vts_data->proFcFilterSwitch = 1;	
	vts_data->otherInfo = bbk_ili_get_fw_debug_info;
	vts_data->processByPackage = bbk_ili_process_by_package;

	ret = vivoTsInit((void *)idev->spi, NULL, -1);
	if(ret < 0) {
		VTE("vivoTsInit fail");
		goto free_vivo_ts;
	}
	
	vts_data->resumeEventBlank = FB_EVENT_BLANK;
	vts_data->suspendEventBlank = FB_EVENT_BLANK; 

	vivoTsFwAdd(VTS_FW_TYPE_FW, CTPM_FW1, sizeof(CTPM_FW1), "PD1901", VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x62);
	vivoTsFwAdd(VTS_FW_TYPE_FW, CTPM_FW2, sizeof(CTPM_FW2), "PD1987F_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x60);

	if (REGULATOR_POWER)
		ilitek_plat_regulator_power_init();

	ilitek_plat_gpio_register();

	if (ilitek_tddi_init() < 0) {
		ipio_err("platform probe failed\n");
		goto free_vivo_ts;
	}

	idev->ili_suspended = false;
	ilitek_plat_irq_register();
	//ilitek_plat_sleep_init();

	VTI("end");
	vivoTsAfterProbeCompleteCall(NULL, NULL, -1);
	
	return 0;

free_vivo_ts:
	VTE("free_vivo_ts");
	vivoTsDeInit();

free_vtsdata:
	VTE("free_vivo_ts");
	vivoTsFree();

	return 0;
}

static int ilitek_plat_remove(void)
{
	ipio_info("remove plat dev\n");
	ilitek_tddi_dev_remove();
	return 0;
}

static const struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};

static struct ilitek_hwif_info hwif = {
	.bus_type = TDDI_INTERFACE,
	.plat_type = TP_PLAT_QCOM,
	.owner = THIS_MODULE,
	.name = TDDI_DEV_ID,
	.of_match_table = of_match_ptr(tp_match_table),
	.plat_probe = ilitek_plat_probe,
	.plat_remove = ilitek_plat_remove,	
	
};

static void vivo_ilitek_plat_dev_init(void)
{
	ipio_info("vivo_ILITEK TP driver init\n");
	if (ilitek_tddi_dev_init(&hwif) < 0) {
		ipio_err("Failed to register spi bus driver\n");
		return ;
	}
	return;
}

static int __init ilitek_plat_dev_init(void)
{
	//enum boot_mode_t boot_mode;
	int ret;
	
	ipio_info("ILITEK TP driver\n");
	ret = report_lcm_id();
	VTI("ilitek module lcm id is 0x%x", ret);
	if (0x62 != ret && 0x60 != ret) {
		VTI("not ilitek TP");
		return 0;
	}
	
	// by vivo tp team start
	/*
	boot_mode = get_boot_mode();
	VTI("boot_mode = %d", boot_mode);
	
	if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT ||
		boot_mode == LOW_POWER_OFF_CHARGING_BOOT ||
		boot_mode == META_BOOT ||
		boot_mode == FACTORY_BOOT ||
		boot_mode == ADVMETA_BOOT) {
		VTI("in (%d) mode, do not load driver", boot_mode);
		boot_mode_normal = false;
	} else {
		boot_mode_normal = true;
	}

	
	if (is_atboot == 1 || !boot_mode_normal) {
		VTI("TS is in at mode of power off charging mode.");
		return 0;
	}

	if (os_boot_puresys == 1) {
		VTI("boot puresys, don't register driver.");
		return 0;
	}
	*/
	// by vivo tp team end
	ret = vivo_touchscreen_new_module_init(vivo_ilitek_plat_dev_init, "vivo_ts_nt_spi_driver_init");
	
	VTI("finished...");
	
	return ret;
}

static void __exit ilitek_plat_dev_exit(void)
{
	ipio_info("remove plat dev\n");
	ilitek_tddi_dev_remove();
}

//module_init(ilitek_plat_dev_init);
late_initcall(ilitek_plat_dev_init);
module_exit(ilitek_plat_dev_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
