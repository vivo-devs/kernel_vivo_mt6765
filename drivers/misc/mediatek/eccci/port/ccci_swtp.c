// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015 MediaTek Inc.
 */
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#include "ccci_debug.h"
#include "ccci_config.h"
#include "ccci_common_config.h"
#include "ccci_modem.h"
#include "ccci_swtp.h"
#include "ccci_fsm.h"

#include <linux/input/mt.h>
#include <linux/input.h>
#include <mt-plat/mtk_boot_common.h>
#include <linux/printk.h>
#include <linux/proc_fs.h>

static struct input_dev *swtp_input_dev;
const struct of_device_id swtp_of_match[] = {
	{ .compatible = SWTP_COMPATIBLE_DEVICE_ID, },
	{ .compatible = SWTP1_COMPATIBLE_DEVICE_ID,},
	{},
};
#define SWTP_MAX_SUPPORT_MD 1
struct swtp_t swtp_data[SWTP_MAX_SUPPORT_MD];

#define GOIP_STATUS "gpio_status"
static struct proc_dir_entry  *gpio_status;
int swtp_int_gpio = -1;
int input_data = -1;
static int gpio_proc_show(struct seq_file *file, void *data)
{
	input_data = gpio_get_value(swtp_int_gpio);
	seq_printf(file, "%d\n", input_data);

	return 0;
}

static int gpio_proc_open (struct inode *inode, struct file *file)
{
	return single_open(file, gpio_proc_show, inode->i_private);
}

static const struct file_operations gpio_status_ops = {
	.open = gpio_proc_open,
	.read = seq_read,
};

static int switch_Tx_Power(int md_id, unsigned int mode)
{
	int ret = 0;
	unsigned int resv = mode;

	ret = exec_ccci_kern_func_by_md_id(md_id, ID_UPDATE_TX_POWER,
		(char *)&resv, sizeof(resv));

	pr_debug("[swtp] switch_MD%d_Tx_Power(%d): ret[%d]\n",
		md_id + 1, resv, ret);

	CCCI_DEBUG_LOG(md_id, "ctl", "switch_MD%d_Tx_Power(%d): %d\n",
		md_id + 1, resv, ret);

	return ret;
}

int switch_MD1_Tx_Power(unsigned int mode)
{
	return switch_Tx_Power(0, mode);
}

int switch_MD2_Tx_Power(unsigned int mode)
{
	return switch_Tx_Power(1, mode);
}

static int swtp_switch_mode(int irq, struct swtp_t *swtp)
{
	unsigned long flags;
	int val;

	if (swtp == NULL) {
		CCCI_LEGACY_ERR_LOG(-1, SYS, "%s data is null\n", __func__);
		return -1;
	}

	spin_lock_irqsave(&swtp->spinlock, flags);
	val = swtp->irq[0] == irq ? 0:1;

	if (swtp->eint_type[val] == IRQ_TYPE_LEVEL_LOW) {
		irq_set_irq_type(swtp->irq[val], IRQ_TYPE_LEVEL_HIGH);
		swtp->eint_type[val] = IRQ_TYPE_LEVEL_HIGH;
		input_report_key(swtp_input_dev, KEY_TABLE0, 1);
		input_sync(swtp_input_dev);
		input_report_key(swtp_input_dev, KEY_TABLE0, 0);
		input_sync(swtp_input_dev);
		printk("[swtp]input keycode = %d", KEY_TABLE0);
	} else {
		irq_set_irq_type(swtp->irq[val], IRQ_TYPE_LEVEL_LOW);
		swtp->eint_type[val] = IRQ_TYPE_LEVEL_LOW;
		input_report_key(swtp_input_dev, KEY_TABLE1, 1);
		input_sync(swtp_input_dev);
		input_report_key(swtp_input_dev, KEY_TABLE1, 0);
		input_sync(swtp_input_dev);
		printk("[swtp]input keycode = %d", KEY_TABLE1);
	}
	if (swtp->curr_mode[val] == SWTP_EINT_PIN_PLUG_IN)
		swtp->curr_mode[val] = SWTP_EINT_PIN_PLUG_OUT;
	  else
		swtp->curr_mode[val] = SWTP_EINT_PIN_PLUG_IN;
/*
	if (swtp->curr_mode[val] == SWTP_EINT_PIN_PLUG_IN) {
		if (swtp->eint_type[val] == IRQ_TYPE_LEVEL_HIGH)
			irq_set_irq_type(swtp->irq[val], IRQ_TYPE_LEVEL_HIGH);
		else
			irq_set_irq_type(swtp->irq[val], IRQ_TYPE_LEVEL_LOW);
	} else {
		if (swtp->eint_type[val] == IRQ_TYPE_LEVEL_HIGH)
			irq_set_irq_type(swtp->irq[val], IRQ_TYPE_LEVEL_LOW);
		else
			irq_set_irq_type(swtp->irq[val], IRQ_TYPE_LEVEL_HIGH);
	}
*/
	swtp->curr_mode[val] = !swtp->curr_mode[val];

	if (swtp->curr_mode[0] | swtp->curr_mode[1])
		swtp->final_mode = SWTP_EINT_PIN_PLUG_IN;
	else
		swtp->final_mode = SWTP_EINT_PIN_PLUG_OUT;
	CCCI_LEGACY_ALWAYS_LOG(swtp->md_id, SYS, "%s mode %d\n",
		__func__, swtp->final_mode);
	spin_unlock_irqrestore(&swtp->spinlock, flags);

	return swtp->final_mode;
}

static int swtp_send_tx_power_mode(struct swtp_t *swtp)
{
	unsigned long flags;
	unsigned int md_state;
	int ret = 0;

	md_state = ccci_fsm_get_md_state(swtp->md_id);
	if (md_state != BOOT_WAITING_FOR_HS1 &&
		md_state != BOOT_WAITING_FOR_HS2 &&
		md_state != READY) {
		CCCI_LEGACY_ERR_LOG(swtp->md_id, SYS,
			"%s md_state=%d no ready\n", __func__, md_state);
		ret = 1;
		goto __ERR_HANDLE__;
	}
	if (swtp->md_id == 0)
		ret = switch_MD1_Tx_Power(swtp->final_mode);
	else {
		CCCI_LEGACY_ERR_LOG(swtp->md_id, SYS,
			"%s md is no support\n", __func__);
		ret = 2;
		goto __ERR_HANDLE__;
	}

	if (ret >= 0)
		CCCI_LEGACY_ALWAYS_LOG(swtp->md_id, SYS,
			"%s send swtp to md ret=%d, mode=%d, rety_cnt=%d\n",
			__func__, ret, swtp->final_mode, swtp->retry_cnt);
	spin_lock_irqsave(&swtp->spinlock, flags);
	if (ret >= 0)
		swtp->retry_cnt = 0;
	else
		swtp->retry_cnt++;
	spin_unlock_irqrestore(&swtp->spinlock, flags);

__ERR_HANDLE__:

	if (ret < 0) {
		CCCI_LEGACY_ERR_LOG(swtp->md_id, SYS,
			"%s send tx power failed, ret=%d,rety_cnt=%d schedule delayed work\n",
			__func__, ret, swtp->retry_cnt);
		schedule_delayed_work(&swtp->delayed_work, 5 * HZ);
	}

	return ret;
}


static irqreturn_t swtp_irq_func(int irq, void *data)
{
	struct swtp_t *swtp = (struct swtp_t *)data;
	int ret = 0;

	ret = swtp_switch_mode(irq, swtp);
	if (ret < 0) {
		CCCI_LEGACY_ERR_LOG(swtp->md_id, SYS,
			"%s swtp_switch_mode failed in irq, ret=%d\n",
			__func__, ret);
	} else {
		ret = swtp_send_tx_power_mode(swtp);
		if (ret < 0)
			CCCI_LEGACY_ERR_LOG(swtp->md_id, SYS,
				"%s send tx power failed in irq, ret=%d,and retry late\n",
				__func__, ret);
	}

	return IRQ_HANDLED;
}

static void swtp_tx_work(struct work_struct *work)
{
	struct swtp_t *swtp = container_of(to_delayed_work(work),
		struct swtp_t, delayed_work);
	int ret = 0;

	ret = swtp_send_tx_power_mode(swtp);
}
void ccci_swtp_test(int irq)
{
	swtp_irq_func(irq, &swtp_data[0]);
}

int swtp_md_tx_power_req_hdlr(int md_id, int data)
{
	int ret = 0;
	struct swtp_t *swtp = NULL;

	if (md_id < 0 || md_id >= SWTP_MAX_SUPPORT_MD)
		return -1;
	swtp = &swtp_data[md_id];
	ret = swtp_send_tx_power_mode(swtp);
	return 0;
}

int swtp_init(int md_id)
{
	int i, ret = 0;
#ifdef CONFIG_MTK_EIC
	u32 ints[2] = { 0, 0 };
#else
	u32 ints[1] = { 0 };
#endif
	u32 ints1[2] = { 0, 0 };
	struct device_node *node = NULL;

	if (md_id < 0 || md_id >= SWTP_MAX_SUPPORT_MD) {
		CCCI_LEGACY_ERR_LOG(-1, SYS,
			"invalid md_id = %d\n", md_id);
		return -1;
	}
	swtp_data[md_id].md_id = md_id;
	spin_lock_init(&swtp_data[md_id].spinlock);
	INIT_DELAYED_WORK(&swtp_data[md_id].delayed_work, swtp_tx_work);
	for (i = 0; i < 1; i++) {
		swtp_data[md_id].curr_mode[i] = SWTP_EINT_PIN_PLUG_OUT;
		node = of_find_matching_node(NULL, &swtp_of_match[i]);
	if (node) {
		/*ret = of_property_read_u32_array(node, "debounce",
				ints, ARRAY_SIZE(ints));
		ret |= of_property_read_u32_array(node, "interrupts",
				ints1, ARRAY_SIZE(ints1));
		if (ret) {
			CCCI_LEGACY_ERR_LOG(md_id, SYS,
				"%s get property fail.\n", __func__);
			goto prop_fail;
		}*/
		swtp_input_dev = input_allocate_device();
		swtp_input_dev->name = "swtp";
		__set_bit(EV_KEY, swtp_input_dev->evbit);

		input_set_capability(swtp_input_dev, EV_KEY, KEY_TABLE0);
		input_set_capability(swtp_input_dev, EV_KEY, KEY_TABLE1);
		ret = input_register_device(swtp_input_dev);
		if (ret)
			printk("[SWTP]input device register fail \n");


#ifdef CONFIG_MTK_EIC /* for chips before mt6739 */
			swtp_data[md_id].gpiopin[i] = ints[0];
			swtp_data[md_id].setdebounce[i] = ints[1];
			swtp_data[md_id].eint_type[i] = ints1[1];
#else /* for mt6739,and chips after mt6739 */
			swtp_data[md_id].setdebounce[i] = ints[0];
			swtp_data[md_id].gpiopin[i] =
				of_get_named_gpio(node, "deb-gpios", 0);
			swtp_data[md_id].eint_type[i] = ints1[1];
#endif
			swtp_data[md_id].eint_type[i] = ints1[1];
			gpio_set_debounce(swtp_data[md_id].gpiopin[i],
			swtp_data[md_id].setdebounce[i]);
			swtp_data[md_id].irq[i] = irq_of_parse_and_map(node, 0);
			swtp_int_gpio = of_get_named_gpio(node, "swtp-gpio", 0);
			ret = request_irq(swtp_data[md_id].irq[i],
				swtp_irq_func, IRQF_TRIGGER_LOW,
				 "swtp0-eint", &swtp_data[md_id]);
		if (ret != 0) {
			CCCI_LEGACY_ERR_LOG(md_id, SYS,
				"swtp%d-eint IRQ LINE NOT AVAILABLE\n", i);
		} else {
			CCCI_LEGACY_ALWAYS_LOG(md_id, SYS,
				"swtp%d-eint set EINT finished, irq=%d, setdebounce=%d, eint_type=%d\n",
				i, swtp_data[md_id].irq[i],
				swtp_data[md_id].setdebounce[i],
				swtp_data[md_id].eint_type[i]);
		}
	} else {
		CCCI_LEGACY_ERR_LOG(md_id, SYS,
			"%s can't find compatible node\n", __func__);
		ret = -1;
		}
	}

	gpio_status = proc_create(GOIP_STATUS, 0644, NULL, &gpio_status_ops);
	if (gpio_status == NULL) {
		printk("tpd, create_proc_entry gpio_status_ops failed\n");
	}
	return ret;
/*prop_fail:*/
	register_ccci_sys_call_back(md_id, MD_SW_MD1_TX_POWER_REQ,
		swtp_md_tx_power_req_hdlr);
	return ret;
}

