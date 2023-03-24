#define pr_fmt(fmt)		"[FP_KERN] " KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>

#include "fp_id.h"

static uint8_t support_soft_fpid;
const char *fp_soft_id;
static char *fp_frame_buff;
#define FRAME_BUFF_LEN 128
#define MAX_TIMES		7

struct kobject kobj;

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};
static const struct vreg_config  vreg_conf[] = {
	{ "vcc_fpc", 1800000UL, 1800000UL, 10, },
	{ "vcc_goodix", 2800000UL, 2800000UL, 10, },
};

static const char * const pctl_names[] = {
	"fp_gpio_pull_up",
	"fp_gpio_pull_down",
};

int get_fp_id(void);
static int fp_id;
static int fp_up;
static int fp_down;
const char *fp_project_name;
struct regulator *fp_vreg[ARRAY_SIZE(vreg_conf)];
struct pinctrl *fingerprint_pinctrl;
struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
extern char *get_board_version(void);

static int vreg_setup(struct device *dev, const char *name,
	bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;

	pr_info("vreg_setup start");
	for (i = 0; i < ARRAY_SIZE(fp_vreg); i++) {
		const char *n = vreg_conf[i].name;

		if (!strncmp(n, name, strlen(n)))
			goto found;
	}
	pr_err("Regulator %s not found", name);
	return -EINVAL;
found:
	vreg = fp_vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = regulator_get(dev, name);
			if (IS_ERR(vreg)) {
				pr_err("Unable to get  %s", name);
				return -ENODEV;
			}
		}
		pr_info("vreg_setup vfp");
		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc)
				pr_err("Unable to set voltage on %s, %d",
					name, rc);
		}
		/*rc = regulator_set_optimum_mode(vreg, vreg_conf[i].ua_load);
		if (rc < 0)
			dev_err(dev, "Unable to set current on %s, %d\n",
					name, rc);*/
		rc = regulator_enable(vreg);
		if (rc) {
			pr_err("error enabling %s: %d", name, rc);
			regulator_put(vreg);
			vreg = NULL;
		}
		fp_vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
				pr_debug("disabled %s", name);
			}
			regulator_put(vreg);
			fp_vreg[i] = NULL;
		}
		rc = 0;
	}
	return rc;
}

static int select_pin_ctl(struct device *dev, const char *name) /*down */
{
	size_t i;
	int rc;
	for (i = 0; i < ARRAY_SIZE(pinctrl_state); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fingerprint_pinctrl, pinctrl_state[i]);
			if (rc)
				pr_err("bio_fp_error cannot select '%s'\n", name);
			else
				pr_debug("Selected '%s'\n", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	pr_err("bio_fp_error %s:'%s' not found\n", __func__, name);
exit:
	return rc;
}

#define DEVFS_MODE_RO (S_IRUSR|S_IRGRP|S_IROTH)
struct attribute fp_id_attr = {
	.name = "fp_id",
	.mode = DEVFS_MODE_RO,
};

static struct attribute *our_own_sys_attrs[] = {
	&fp_id_attr,
	NULL,
};

int get_fp_id(void)
{
	return fp_id;
}
EXPORT_SYMBOL(get_fp_id);

static void fp_id_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static ssize_t fp_id_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	char *fp_frame_id;

	if (support_soft_fpid) {
		if (NULL == fp_soft_id) {
			fp_frame_id = "default";
		} else {
			pr_info("fp_frame_id=%s.\n", fp_soft_id);
			return snprintf(buf, strlen(fp_soft_id) + 2, "%s\n", fp_soft_id);
		}
	} else {
		if (fp_id == FPC_FPC1229) {
			fp_frame_id = "fpc_1229";
		} else if (fp_id == GOODIX_GF5126M) {
			fp_frame_id = "goodix_5126m";
		} else if (fp_id == GOODIX_GF5216C) {
			fp_frame_id = "goodix_5216c";
		} else if (fp_id == GOODIX_GF5269) {
			fp_frame_id = "goodix_5269";
		} else if (fp_id == GOODIX_GF3208) {
			fp_frame_id = "goodix_3208";
		} else if (fp_id == GOODIX_GF318M) {
			fp_frame_id = "goodix_318m";
		} else if (fp_id == GOODIX_GF5288) {
			fp_frame_id = "goodix_5288";
		} else if (fp_id == GOODIX_GF3658) {
			fp_frame_id = "goodix_3658";
		} else if (fp_id == GOODIX_GF3626) {
			fp_frame_id = "sidefp_goodix_3626";
		} else if (fp_id == FPC_FPC1511) {
			fp_frame_id = "fpc_1511";
		} else if (fp_id == FPC_FPC1540) {
			fp_frame_id = "sidefp_fpc_1540";
		} else if (fp_id == SILEAD_GSL6165) {
			fp_frame_id = "silead_6165";
		} else if (fp_id == FOCALTECH_FT9391) {
			fp_frame_id = "sidefp_focaltech_ft9391";
		} else if (fp_id == CHIPONE_ICNF7312) {
			fp_frame_id = "sidefp_chipone_icnf7312";
		}
	}
	pr_info("fp_id_int get_fp_id=%d, fp_frame_id=%s, fp_down=%d, fp_up=%d.", get_fp_id(), fp_frame_id, fp_down, fp_up);
	return snprintf(buf, strlen(fp_frame_id) + 2, "%s\n", fp_frame_id);
}

static ssize_t fp_id_object_store(struct kobject *k, struct attribute *attr, const char *buf, size_t count)
{
	if (support_soft_fpid) {
		memset(fp_frame_buff, 0, FRAME_BUFF_LEN);
		memcpy(fp_frame_buff, buf, count);
		// snprintf(fp_frame_buff, FRAME_BUFF_LEN, "%s", buf);
		fp_frame_buff[FRAME_BUFF_LEN - 1] = '\0';
		fp_soft_id = fp_frame_buff;
		pr_info("set fp_id: %s.", fp_soft_id);
		return count;
	} else {
		/* nothing to do temply */
		pr_info("fp_id cannot be writed.");
		return 0;
	}
}

static const struct sysfs_ops fp_id_object_sysfs_ops = {
	.show = fp_id_object_show,
	.store = fp_id_object_store,
};

static struct kobj_type fp_id_object_type = {
	.sysfs_ops	= &fp_id_object_sysfs_ops,
	.release	= fp_id_object_release,
	.default_attrs = our_own_sys_attrs,
};

static int fp_id_probe(struct platform_device *pdev)
{
	int ret;
	int fp_gpio = -1;
	int i;
	char *board_version = NULL;
	fp_frame_buff = kmalloc(FRAME_BUFF_LEN, GFP_KERNEL);
	if (!fp_frame_buff) {
		pr_err("%s: alloc fp_frame_buff failed!", __func__);
		ret = -ENOMEM;
	}

	ret = kobject_init_and_add(&kobj, &fp_id_object_type, NULL, "fp_id");
	if (ret) {
		pr_err("%s: Create fp_id error!", __func__);
		return -EINVAL;
	}
	ret = of_property_read_string(pdev->dev.of_node, "vivo,project-name", &fp_project_name);
	if (ret) {
		pr_err(KERN_ERR "bio_fp_error %s:vivo,project-name property do not find", __func__);
		fp_project_name = "default";
	}
	pr_info("%s:vivo,project-name = %s", __func__, fp_project_name);
	
	/*if (!strncmp(fp_project_name, "PD2140F_EX", 10)) {
		fp_id = FOCALTECH_FT9391;
		pr_info("%s:return directly!!!", __func__);
		return 0;
	}*/

	support_soft_fpid = of_property_read_bool(pdev->dev.of_node, "vivo,support_soft_fingerprint_id");
	if (support_soft_fpid) {
		ret = of_property_read_string(pdev->dev.of_node, "vivo,soft_fingerprint_id", &fp_soft_id);
		if (ret) {
			pr_err("%s:vivo,soft_fingerprint_id property do not find", __func__);
			return -EINVAL;
		}
		pr_info("%s:vivo,soft_fingerprint_id is %s", __func__, fp_soft_id);
		return 0;
	} else {
		pr_info("%s:vivo,support_soft_fingerprint_id property do not find", __func__);
	}

	fp_gpio = of_get_named_gpio(pdev->dev.of_node, "fp_id,gpios", 0);
	if (fp_gpio < 0) {
		pr_err("%s: get fp_id gpio failed!", __func__);
		return -1;
	}
	pr_info("%s:fp gpio: %d", __func__, fp_gpio);

	ret = devm_gpio_request(&pdev->dev, fp_gpio, "fp_id,gpios");
	if (ret)  {
		pr_err("%s: request fp_id gpio failed!", __func__);
		return -EINVAL;
	}

	fingerprint_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(fingerprint_pinctrl)) {
		if (PTR_ERR(fingerprint_pinctrl) == -EPROBE_DEFER) {
			pr_err("bio_fp_error %s: pinctrl not ready!", __func__);
			return -EINVAL;
		}
		pr_err("%s: Target does not use pinctrl", __func__);
		fingerprint_pinctrl = NULL;
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			pr_err("bio_fp_error %s: cannot find '%s'", __func__, n);
			return -EINVAL;
		}
		pr_info("%s: found pin control %s", __func__, n);
		pinctrl_state[i] = state;
	}


	if (!strncmp(fp_project_name, "PD1801", 6)) {
		ret = vreg_setup(&pdev->dev, "vcc_fpc", true);
		if (ret) {
			pr_info("%s:can not set vreg", __func__);
			return -EINVAL;
		}
		mdelay(5);
	}
	ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_up");
	mdelay(5);
	if (ret)
		return -EINVAL;
	fp_up = gpio_get_value(fp_gpio);
	/*pr_info("%s: set fp-id pull up,get gpio value = %d", __func__, fp_up); */
	ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
	mdelay(5);
	if (ret)
		return -EINVAL;
	fp_down = gpio_get_value(fp_gpio);

	if (!strncmp(fp_project_name, "VTD1901", 7) || !strncmp(fp_project_name, "VTD1902", 7)) {
		if ((fp_up == 1) && (fp_down == 0)) {
			fp_id = GOODIX_GF3658;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
		} else {
			fp_id = GOODIX_GF5288;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
		}
	} else if (!strncmp(fp_project_name, "PD1901", 6) || !strncmp(fp_project_name, "PD1901F_EX", 10)) {
		if ((fp_up == 1) && (fp_down == 0)) {
			fp_id = GOODIX_GF5288;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
		} else {
			fp_id = GOODIX_GF3658;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
		}
	} else if (!strncmp(fp_project_name, "PD1987F_EX", 10)) {
		if ((fp_up == 0) && (fp_down == 0)) {
			fp_id = GOODIX_GF3658;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
		} else {
			fp_id = FPC_FPC1511;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
		}
	} else if (!strncmp(fp_project_name, "PD2036F_EX", 10)) {
		board_version = get_board_version();
		for (i = 0; i < 6; i++) {
			pr_info("%s board_info i:%d value:%c", __func__, i, *(board_version + i));
		}
		if (*(board_version) == '0') {
			fp_id = -1;
		} else {
			if ((fp_up == 0) && (fp_down == 0)) {
				fp_id = GOODIX_GF3626;
				ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
			} else {
				fp_id = FPC_FPC1540;
				ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
			}
		}
	} else if (!strncmp(fp_project_name, "PD2036", 6) || !strncmp(fp_project_name, "PD2139F_EX", 10)) {
		if ((fp_up == 0) && (fp_down == 0)) {
			fp_id = GOODIX_GF3626;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
		} else {
			fp_id = FPC_FPC1540;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
		}
	} else if (!strncmp(fp_project_name, "PD2014F_EX", 10)) {
		board_version = get_board_version();
		for (i = 0; i < 8; i++) {
			pr_info("%s board_info i:%d value:%c", __func__, i, *(board_version + i));
		}
		if (*(board_version) == '0') {
			fp_id = GOODIX_GF3658;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
		} else {
			fp_id = -1;
		}
	}
	gpio_free(fp_gpio);

	if (!strncmp(fp_project_name, "PD1801", 6)) {
	ret = vreg_setup(&pdev->dev, "vcc_fpc", false);
	if (ret) {
		pr_info("%s:can not set vreg", __func__);
		return -EINVAL;
	}
	msleep(10);
	}
	return 0;
}

static int fp_id_remove(struct platform_device *pdev)
{
	pr_info("fp_id  remove.");
	if (fp_frame_buff) {
		kfree(fp_frame_buff);
	}
	kobject_del(&kobj);
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id fp_id_match_table[] = {
	{ .compatible = "fp-id",},
	{},
};
#endif

static struct platform_driver fp_id_driver = {
	.probe      = fp_id_probe,
	.remove     = fp_id_remove,
	.driver = {
		.name   = "fp_id",
		.owner  = THIS_MODULE,
		.of_match_table = fp_id_match_table,
	},
};

static int __init fp_id_init(void)
{
	pr_info("fp_id  fp_id_init.");
	return platform_driver_register(&fp_id_driver);
}
late_initcall(fp_id_init);

static void __exit fp_id_exit(void)
{
	platform_driver_unregister(&fp_id_driver);
}
module_exit(fp_id_exit);

MODULE_AUTHOR("Xiaot BBK Corporation");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
