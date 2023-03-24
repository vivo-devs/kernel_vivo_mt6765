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

/* Debug level */
bool ipio_debug_level = DEBUG_OUTPUT;
EXPORT_SYMBOL(ipio_debug_level);

static struct workqueue_struct *esd_wq;
static struct workqueue_struct *bat_wq;
static struct workqueue_struct *spi_recover_wq;
static struct workqueue_struct *esd_gesture_wq;
static struct delayed_work esd_work;
static struct delayed_work bat_work;
static struct delayed_work spi_recover_work;
static struct delayed_work esd_gesture_work;
u8 *readbuf = NULL;

int ilitek_tddi_mp_test_handler(char *apk, bool lcm_on)
{
	int ret = 0;
	u8 tp_mode = P5_X_FW_TEST_MODE;

	if (atomic_read(&idev->fw_stat)) {
		ipio_err("fw upgrade processing, ignore\n");
		return 0;
	}

	if (!idev->chip->open_c_formula ||
		!idev->chip->open_sp_formula) {
		ipio_err("formula is null\n");
		return -1;
	}

	atomic_set(&idev->mp_stat, ENABLE);
	ilitek_plat_irq_disable();
	if (idev->actual_tp_mode != P5_X_FW_TEST_MODE) {
		if (ilitek_tddi_switch_mode(&tp_mode) < 0)
			goto out;
	}

	ret = ilitek_tddi_mp_test_main(apk, lcm_on);

out:
	/*
	 * If there's running mp test with lcm off, we suspose that
	 * users will soon call resume from suspend. TP mode will be changed
	 * from MP to AP mode until resume finished.
	 */
	if (!lcm_on) {
		atomic_set(&idev->mp_stat, DISABLE);
		ilitek_plat_irq_enable();
		return ret;
	}

	idev->actual_tp_mode = P5_X_FW_DEMO_MODE;
	if (idev->fw_upgrade_mode == UPGRADE_IRAM)
		ilitek_tddi_fw_upgrade_handler(NULL);
	else
		ilitek_tddi_reset_ctrl(idev->reset);

	atomic_set(&idev->mp_stat, DISABLE);
	ilitek_plat_irq_enable();
	return ret;
}

int ilitek_tddi_switch_mode(u8 *data)
{
	int ret = 0, mode;
	u8 cmd[4] = {0};

	if (!data) {
		ipio_err("data is null\n");
		return -EINVAL;
	}

	atomic_set(&idev->tp_sw_mode, START);
	ilitek_plat_irq_disable();
	mode = data[0];
	idev->actual_tp_mode = mode;

	switch (idev->actual_tp_mode) {
	case P5_X_FW_DEMO_MODE:
		ipio_info("Switch to Demo mode\n");
		cmd[0] = P5_X_MODE_CONTROL;
		cmd[1] = mode;
		ret = idev->write(cmd, 2);
		if (ret < 0) {
			ipio_err("Failed to switch demo mode, do reset/reload instead\n");
			if (idev->fw_upgrade_mode == UPGRADE_IRAM)
				ilitek_tddi_fw_upgrade_handler(NULL);
			else
				ilitek_tddi_reset_ctrl(idev->reset);
		}
		break;
	case P5_X_FW_DEBUG_MODE:
		ipio_info("Switch to Debug mode\n");
		cmd[0] = P5_X_MODE_CONTROL;
		cmd[1] = mode;
		ret = idev->write(cmd, 2);
		if (ret < 0)
			ipio_err("Failed to switch Debug mode\n");
		break;
	case P5_X_FW_GESTURE_MODE:
		ipio_info("Switch to Gesture mode, lpwg cmd = %d\n",  idev->gesture_mode);
		ret = ilitek_tddi_ic_func_ctrl("lpwg", idev->gesture_mode);
		break;
	case P5_X_FW_TEST_MODE:
		ipio_info("Switch to Test mode\n");
		ret = idev->mp_move_code();
		break;
	case P5_X_FW_DEMO_DEBUG_INFO_MODE:
		ipio_info("Switch to demo debug info mode\n");
		cmd[0] = P5_X_MODE_CONTROL;
		cmd[1] = mode;
		ret = idev->write(cmd, 2);
		if (ret < 0)
			ipio_err("Failed to switch debug info mode\n");
		break;
	case P5_X_FW_SOP_FLOW_MODE:
		ipio_info("Not implemented SOP flow mode yet\n");
		break;
	case P5_X_FW_ESD_MODE:
		ipio_info("Not implemented ESD mode yet\n");
		break;
	default:
		ipio_err("Unknown TP mode: %x\n", mode);
		ret = -1;
		break;
	}

	if (ret < 0)
		ipio_err("Switch mode failed\n");

	ipio_debug("Actual TP mode = %d\n", idev->actual_tp_mode);
	atomic_set(&idev->tp_sw_mode, END);
	ilitek_plat_irq_enable();	
	return ret;
}

void ilitek_tddi_gesture_recovery(void)
{
	bool lock = mutex_is_locked(&idev->touch_mutex);

	atomic_set(&idev->esd_stat, START);

	if (!lock)
		mutex_lock(&idev->touch_mutex);

	ipio_info("Doing gesture recovery\n");
	idev->ges_recover();

	if (!lock)
		mutex_unlock(&idev->touch_mutex);

	atomic_set(&idev->esd_stat, END);
}

void ilitek_tddi_spi_recovery(void)
{
	bool lock = mutex_is_locked(&idev->touch_mutex);

	atomic_set(&idev->esd_stat, START);

	if (!lock)
		mutex_lock(&idev->touch_mutex);

	ipio_info("Doing spi recovery\n");
	if (ilitek_tddi_fw_upgrade_handler(NULL) < 0)
		ipio_err("FW upgrade failed\n");

	if (!lock)
		mutex_unlock(&idev->touch_mutex);

	atomic_set(&idev->esd_stat, END);
}

int ilitek_tddi_wq_esd_spi_check(void)
{
	u8 tx = SPI_WRITE, rx = 0;

	idev->spi_write_then_read(idev->spi, &tx, 1, &rx, 1);
	ipio_debug("spi esd check = 0x%x\n", rx);
	if (rx != SPI_ACK) {
		ipio_err("rx = 0x%x\n", rx);
		return -1;
	}
	return 0;
}

static void ilitek_tddi_wq_esd_check(struct work_struct *work)
{
	if (idev->esd_recover() < 0) {
		ipio_err("SPI ACK failed, doing spi recovery\n");
		ilitek_tddi_spi_recovery();
		return;
	}
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
}

static int read_power_status(u8 *buf)
{
	struct file *f = NULL;
	mm_segment_t old_fs;
	ssize_t byte = 0;

	old_fs = get_fs();
	set_fs(get_ds());

	f = filp_open(POWER_STATUS_PATH, O_RDONLY, 0);
	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open %s\n", POWER_STATUS_PATH);
		return -1;
	}

	f->f_op->llseek(f, 0, SEEK_SET);
	byte = f->f_op->read(f, buf, 20, &f->f_pos);

	ipio_debug("Read %d bytes\n", (int)byte);

	set_fs(old_fs);
	filp_close(f, NULL);
	return 0;
}

static void ilitek_tddi_wq_bat_check(struct work_struct *work)
{
	u8 str[20] = {0};
	static int charge_mode;

	read_power_status(str);
	ipio_debug("Batter Status: %s\n", str);

	if (strstr(str, "Charging") != NULL || strstr(str, "Full") != NULL
		|| strstr(str, "Fully charged") != NULL) {
		if (charge_mode != 1) {
			ipio_debug("Charging mode\n");
			ilitek_tddi_ic_func_ctrl("plug", DISABLE);/* plug in*/
			charge_mode = 1;
		}
	} else {
		if (charge_mode != 2) {
			ipio_debug("Not charging mode\n");
			ilitek_tddi_ic_func_ctrl("plug", ENABLE);/* plug out*/
			charge_mode = 2;
		}
	}
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
}

void ilitek_tddi_wq_ctrl(int type, int ctrl)
{
	switch (type) {
	case WQ_ESD:
		if (ENABLE_WQ_ESD) {
			if (!esd_wq) {
				ipio_err("wq esd is null\n");
				break;
			}
			idev->wq_esd_ctrl = ctrl;
			if (ctrl == ENABLE) {
				ipio_debug("execute esd check\n");
				if (!queue_delayed_work(esd_wq, &esd_work, msecs_to_jiffies(WQ_ESD_DELAY)))
					ipio_debug("esd check was already on queue\n");
			} else {
				cancel_delayed_work_sync(&esd_work);
				flush_workqueue(esd_wq);
				ipio_debug("cancel esd wq\n");
			}
		}
		break;
	case WQ_BAT:
		if (ENABLE_WQ_BAT) {
			if (!bat_wq) {
				ipio_err("WQ BAT is null\n");
				break;
			}
			idev->wq_bat_ctrl = ctrl;
			if (ctrl == ENABLE) {
				ipio_debug("execute bat check\n");
				if (!queue_delayed_work(bat_wq, &bat_work, msecs_to_jiffies(WQ_BAT_DELAY)))
					ipio_debug("bat check was already on queue\n");
			} else {
				cancel_delayed_work_sync(&bat_work);
				flush_workqueue(bat_wq);
				ipio_debug("cancel bat wq\n");
			}
		}
		break;
	case WQ_SPI_RECOVER:
		if (!spi_recover_wq) {
			ipio_err("wq spi recovery is null\n");
			break;
		}
		ipio_info("execute spi recovery\n");
		if (!queue_delayed_work(spi_recover_wq, &spi_recover_work, msecs_to_jiffies(HZ)))
			ipio_info("spi recovery was already on queue\n");
		break;
	case WQ_GES_RECOVER:
		if (!esd_gesture_wq) {
			ipio_err("wq gesture recovery is null\n");
			break;
		}
		ipio_info("execute geture recovery\n");
		if (!queue_delayed_work(esd_gesture_wq, &esd_gesture_work, msecs_to_jiffies(HZ)))
			ipio_info("geture recovery was already on queue\n");
		break;
	case WQ_SUSPEND:
		ipio_err("Not implement yet\n");
		break;
	default:
		ipio_err("Unknown WQ type, %d\n", type);
		break;
	}
}

static void ilitek_tddi_wq_init(void)
{
	esd_wq = alloc_workqueue("esd_check", WQ_MEM_RECLAIM, 0);
	bat_wq = alloc_workqueue("bat_check", WQ_MEM_RECLAIM, 0);

	WARN_ON(!esd_wq);
	WARN_ON(!bat_wq);

	INIT_DELAYED_WORK(&esd_work, ilitek_tddi_wq_esd_check);
	INIT_DELAYED_WORK(&bat_work, ilitek_tddi_wq_bat_check);
}

int ilitek_tddi_sleep_handler(int mode)
{
	int ret = 0;

	mutex_lock(&idev->touch_mutex);
	atomic_set(&idev->tp_sleep, START);

	if (atomic_read(&idev->fw_stat) ||
		atomic_read(&idev->mp_stat)) {
		ipio_info("fw upgrade or mp still running, ignore sleep requst\n");
		atomic_set(&idev->tp_sleep, END);
		mutex_unlock(&idev->touch_mutex);
		return 0;
	}

	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);
	ilitek_plat_irq_disable();

	ipio_info("Sleep Mode = %d\n", mode);

	switch (mode) {
	case TP_SUSPEND:
		ipio_info("TP suspend start\n");
		ilitek_tddi_ic_func_ctrl("sense", DISABLE);
		ilitek_tddi_ic_check_busy(50, 50);

		if (idev->gesture) {
			ipio_info("prepare to getsture mode\n");
			idev->gesture_move_code(idev->gesture_mode);
			enable_irq_wake(idev->irq_num);
			ilitek_plat_irq_enable();
		} else {
			ilitek_tddi_ic_func_ctrl("sleep", SLEEP_IN);
		}
		ipio_info("TP suspend end\n");
		break;
	case TP_DEEP_SLEEP:
		ipio_info("TP deep suspend start\n");
		ilitek_tddi_ic_func_ctrl("sense", DISABLE);
		ilitek_tddi_ic_check_busy(50, 50);

		if (idev->gesture) {
			idev->gesture_move_code(idev->gesture_mode);
			enable_irq_wake(idev->irq_num);
			ilitek_plat_irq_enable();
		} else {
			ilitek_tddi_ic_func_ctrl("sleep", DEEP_SLEEP_IN);
		}
		ipio_info("TP deep suspend end\n");
		break;
	case TP_RESUME:
		ipio_info("TP resume start\n");
		if (idev->gesture)
			disable_irq_wake(idev->irq_num);

		/* Set tp as demo mode and reload code if it's iram. */
		idev->actual_tp_mode = P5_X_FW_DEMO_MODE;
		if (idev->fw_upgrade_mode == UPGRADE_IRAM)
			ilitek_tddi_fw_upgrade_handler(NULL);
		else
			ilitek_tddi_reset_ctrl(idev->reset);
		ilitek_plat_irq_enable();
		ipio_info("TP resume end\n");
		ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
		ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
		break;
	default:
		ipio_err("Unknown sleep mode, %d\n", mode);
		ret = -EINVAL;
		break;
	}

	ilitek_tddi_touch_release_all_point();
	atomic_set(&idev->tp_sleep, END);
	mutex_unlock(&idev->touch_mutex);
	return ret;
}

int ilitek_tddi_fw_upgrade_handler(void *data)
{
	int ret = 0;
	static bool input_reg_once;

	atomic_set(&idev->fw_stat, START);

	idev->fw_update_stat = 0;
	ret = ilitek_tddi_fw_upgrade(idev->fw_upgrade_mode, HEX_FILE, idev->fw_open);
	if (ret != 0)
		idev->fw_update_stat = -1;
	else
		idev->fw_update_stat = 100;

	// ipio_info("Flash FW completed ... update TP/FW info\n");
	// ilitek_tddi_ic_get_protocl_ver();
	// ilitek_tddi_ic_get_fw_ver();
	// ilitek_tddi_ic_get_core_ver();
	// ilitek_tddi_ic_get_tp_info();
	// ilitek_tddi_ic_get_panel_info();

	if (!input_reg_once) {
		input_reg_once = true;
		ipio_info("Registre touch to input subsystem\n");
		ilitek_plat_input_register();
	}

	atomic_set(&idev->fw_stat, END);
	complete(&idev->fw_update_done);
	return ret;
}

void ilitek_tddi_report_handler(void)
{
	int ret = 0, pid = 0;
	u8 checksum = 0;
	int rlen = 0, buf_size = 0;
	u16 self_key = 2;
	int tmp = ipio_debug_level;

	/* Just in case these stats couldn't be blocked in top half context */
	/*if (!idev->report || atomic_read(&idev->tp_reset) ||
		atomic_read(&idev->fw_stat) || atomic_read(&idev->tp_sw_mode) ||
		atomic_read(&idev->mp_stat) || atomic_read(&idev->tp_sleep)) {
		ipio_info("ignore report request\n");
		return;
	}*/
	
	if (atomic_read(&vivoTsGetVtsData()->tsState) == TOUCHSCREEN_GESTURE) { //avoid spi slee cause gesture no function
			int retry = 0;
			VTI("wait spi bus wakeup %d", idev->ili_suspended);
			while(idev->ili_suspended) {
				msleep(10);
				if (++retry > 50) {
					VTI("after 500ms delay, device is stil in suspend mode!\n");
					ilitek_plat_irq_enable();
					return ;
				}
			}
		}

	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);

	switch (idev->actual_tp_mode) {
	case P5_X_FW_DEMO_MODE:
		rlen = P5_X_DEMO_MODE_PACKET_LENGTH;
		break;
	case P5_X_FW_DEBUG_MODE:
		rlen = (2 * idev->xch_num * idev->ych_num) + (idev->stx * 2) + (idev->srx * 2);
		rlen += 2 * self_key + (8 * 2) + 1 + 35;
		break;
	case P5_X_FW_GESTURE_MODE:
		__pm_stay_awake(idev->ws);
		/* Waiting for pm resume completed */
		
		if (idev->gesture_mode == P5_X_FW_GESTURE_INFO_MODE)
			rlen = P5_X_GESTURE_INFO_LENGTH;
		else
			rlen = P5_X_GESTURE_NORMAL_LENGTH;
		break;
	case P5_X_FW_DEMO_DEBUG_INFO_MODE:
		/*only suport SPI interface now, so defult use size 1024 buffer*/
		rlen = 1024;
		break;
	default:
		ipio_err("Unknown fw mode, %d\n", idev->actual_tp_mode);
		rlen = 0;
		break;
	}

	ipio_debug("Packget length = %d\n", rlen);

	if (!rlen) {
		ipio_err("Length of packet is invaild\n");
		goto out;
	}

	buf_size = (idev->fw_uart_en == DISABLE) ? rlen : 2048;

	memset(readbuf,0,2048);
	ret = idev->read(readbuf, rlen);
	if (ret < 0) {
//		ilitek_tddi_ic_get_pc_counter();
		ipio_err("Read report packet failed, ret = %d,idev->reset_state = %d \n", ret, idev->reset_state);
		if (ret == DO_SPI_RECOVER && idev->actual_tp_mode == P5_X_FW_GESTURE_MODE && idev->gesture) {
			ilitek_tddi_ic_get_recover_pc_counter();
			ipio_err("Gesture failed, doing gesture recovery\n");
			ilitek_tddi_gesture_recovery();
			goto recover;
		} else if (ret == DO_SPI_RECOVER) {
			ilitek_tddi_ic_get_recover_pc_counter();
			ipio_err("SPI ACK failed, doing spi recovery\n");
			ilitek_tddi_spi_recovery();
			goto recover;
		}
		goto out;
	}

	ipio_debug("Read length = %d\n", (ret));

	rlen = ret;

	ilitek_dump_data(readbuf, 8, rlen, 0, "finger report");

	checksum = ilitek_calc_packet_checksum(readbuf, rlen - 1);

	if (checksum != readbuf[rlen-1] && idev->fw_uart_en == DISABLE) {
		ipio_err("Wrong checksum, checksum = %x, buf = %x\n", checksum, readbuf[rlen-1]);
		ipio_debug_level = DEBUG_ALL;
		ilitek_dump_data(readbuf, 8, rlen, 0, "finger report with wrong");
		ipio_debug_level = tmp;
		goto out;
	}

	pid = readbuf[0];
	ipio_debug("Packet ID = %x\n", pid);

	switch (pid) {
	case P5_X_DEMO_PACKET_ID:
		ilitek_tddi_report_ap_mode(readbuf, rlen);
		break;
	case P5_X_DEBUG_PACKET_ID:
		ilitek_tddi_report_debug_mode(readbuf, rlen);
		break;
	case P5_X_I2CUART_PACKET_ID:
		ilitek_tddi_report_i2cuart_mode(readbuf, rlen);
		break;
	case P5_X_GESTURE_PACKET_ID:
		ilitek_tddi_report_gesture_mode(readbuf, rlen);
		break;
	case P5_X_PALM_PACKET_ID:
		VTI("got PalmOn");
		vivoTsCoverMute(0, 1);
		break;
	default:
		ipio_err("Unknown packet id, %x\n", pid);
		break;
	}

out:
	idev->reset_state = DISABLE;
	if (!(idev->actual_tp_mode == P5_X_FW_GESTURE_MODE)) {
		ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
		ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
	}
recover:
	if (idev->actual_tp_mode == P5_X_FW_GESTURE_MODE)
	__pm_relax(idev->ws);
	
}

int ilitek_tddi_reset_ctrl(int mode)
{
	int ret = 0;

	atomic_set(&idev->tp_reset, START);
	ilitek_plat_irq_disable();
	if (mode != TP_IC_CODE_RST)
		ilitek_tddi_ic_check_otp_prog_mode();

	switch (mode) {
	case TP_IC_CODE_RST:
		ipio_info("TP IC Code RST \n");
		ret = ilitek_tddi_ic_code_reset();
		break;
	case TP_IC_WHOLE_RST:
		ipio_info("TP IC whole RST\n");
		ret = ilitek_tddi_ic_whole_reset();
		break;
	case TP_HW_RST_ONLY:
		ipio_info("TP HW RST\n");
		ilitek_plat_tp_reset();
		break;
	default:
		ipio_err("Unknown reset mode, %d\n", mode);
		ret = -EINVAL;
		break;
	}

	/*
	 * Since OTP must be folloing with reset, except for code rest,
	 * the stat of ice mode should be set as 0.
	 */
	if (mode != TP_IC_CODE_RST)
		atomic_set(&idev->ice_stat, DISABLE);
	idev->fw_uart_en = DISABLE;
	atomic_set(&idev->tp_reset, END);
	idev->reset_state = ENABLE;
	ilitek_plat_irq_enable();	
	return ret;
}

int ilitek_tddi_init(void)
{
	//struct task_struct *fw_boot_th;

	ipio_info("ilitek tddi main init\n");

	mutex_init(&idev->touch_mutex);
	mutex_init(&idev->debug_mutex);
	mutex_init(&idev->debug_read_mutex);
	init_waitqueue_head(&(idev->inq));
	spin_lock_init(&idev->irq_spin);
	init_completion(&idev->fw_update_done);
	init_completion(&idev->touch_debug_done);

	atomic_set(&idev->irq_stat, DISABLE);
	atomic_set(&idev->ice_stat, DISABLE);
	atomic_set(&idev->tp_reset, END);
	atomic_set(&idev->fw_stat, END);
	atomic_set(&idev->mp_stat, DISABLE);
	atomic_set(&idev->tp_sleep, END);
	atomic_set(&idev->mp_int_check, DISABLE);
	atomic_set(&idev->esd_stat, END);

	readbuf = kcalloc(2048, sizeof(u8), GFP_ATOMIC);
	if (ERR_ALLOC_MEM(readbuf)) {
		ipio_err("Failed to allocate packet memory, %ld\n", PTR_ERR(readbuf));
		return -ENODEV;
	}

	ilitek_tddi_ic_init();
	ilitek_tddi_wq_init();
	ilitek_tddi_node_init();

	/* Must do hw reset once in first time for work normally if tp reset is avaliable */
	if (!TDDI_RST_BIND)
		ilitek_tddi_reset_ctrl(idev->reset);

	idev->do_otp_check = ENABLE;
	idev->fw_uart_en = DISABLE;
	idev->force_fw_update = DISABLE;
	idev->reset_state = DISABLE;
	
	/* Compare version with fw info in boot stage */
	if (idev->fw_upgrade_mode == UPGRADE_FLASH) {
		ilitek_tddi_ic_get_protocl_ver();
		ilitek_tddi_ic_get_fw_ver();
		ilitek_tddi_ic_get_core_ver();
	}

	/*
	 * This status of ice enable will be reset until process of fw upgrade runs.
	 * it might cause unknown problems if we disable ice mode without any
	 * codes inside touch ic.
	 */
	ilitek_ice_mode_ctrl(ENABLE, OFF);

	if (ilitek_tddi_ic_get_info() < 0) {
		ipio_err("Not found ilitek chips\n");
		return -ENODEV;
	}

	ilitek_tddi_fw_read_flash_info(idev->fw_upgrade_mode);
/*
	fw_boot_th = kthread_run(ilitek_tddi_fw_upgrade_handler, NULL, "ili_fw_boot");
	if (fw_boot_th == (struct task_struct *)ERR_PTR) {
		fw_boot_th = NULL;
		WARN_ON(!fw_boot_th);
		ipio_err("Failed to create fw upgrade thread\n");
	}
*/	
	ilitek_tddi_fw_upgrade_handler(NULL);
	ilitek_tddi_ic_get_tp_info();
	ilitek_tddi_ic_get_panel_info();

	wait_for_completion(&idev->fw_update_done);
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);

	idev->ws = wakeup_source_register(NULL,"ili_wakelock");
	if (!idev->ws)
		ipio_err("wakeup source request failed\n");
	
	return 0;
}

void ilitek_tddi_dev_remove(void)
{
	ipio_info("remove ilitek dev\n");

	ipio_kfree((void **)&readbuf);
	if (!idev)
		return;

	gpio_free(idev->tp_int);
	gpio_free(idev->tp_rst);

	if (esd_wq != NULL) {
		cancel_delayed_work_sync(&esd_work);
		flush_workqueue(esd_wq);
		destroy_workqueue(esd_wq);
	}
	if (bat_wq != NULL) {
		cancel_delayed_work_sync(&bat_work);
		flush_workqueue(bat_wq);
		destroy_workqueue(bat_wq);
	}
	if (spi_recover_wq != NULL) {
		cancel_delayed_work_sync(&spi_recover_work);
		flush_workqueue(spi_recover_wq);
		destroy_workqueue(spi_recover_wq);
	}
	if (esd_gesture_wq != NULL) {
		cancel_delayed_work_sync(&esd_gesture_work);
		flush_workqueue(esd_gesture_wq);
		destroy_workqueue(esd_gesture_wq);
	}
	ilitek_tddi_interface_dev_exit(idev->hwif);
}

int ilitek_tddi_dev_init(struct ilitek_hwif_info *hwif)
{	
	return ilitek_tddi_interface_dev_init(hwif);
}
