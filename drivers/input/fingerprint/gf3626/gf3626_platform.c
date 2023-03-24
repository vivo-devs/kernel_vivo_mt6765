/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#define pr_fmt(fmt)		"[FP_KERN]" KBUILD_MODNAME ": " fmt
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf3626_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf3626_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;

	gf_dev->vdd_use_gpio = of_property_read_bool(gf_dev->spi->dev.of_node, "fp,vdd_use_gpio");
	gf_dev->vdd_use_pmic = of_property_read_bool(gf_dev->spi->dev.of_node, "fp,vdd_use_pmic");

	pr_info("%s vdd_use_gpio %d\n", __func__, gf_dev->vdd_use_gpio);
	pr_info("%s vdd_use_pmic %d\n", __func__, gf_dev->vdd_use_pmic);

	if (gf_dev->vdd_use_gpio) {
		gf_dev->vdd_en_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_vdd_en", 0);
		if (!gpio_is_valid(gf_dev->vdd_en_gpio)) {
			pr_info("VDD_EN GPIO is invalid.\n");
			return -EINVAL;
		}

	}

	/*get reset resource */
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		pr_info("RESET GPIO is invalid.\n");
		return -EINVAL;
	}


	/*get irq resourece */
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_irq", 0);
	pr_info("gf::irq_gpio:%d\n", gf_dev->irq_gpio);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		pr_info("IRQ GPIO is invalid.\n");
		return -EINVAL;
	}

	return rc;
}

static gf3626_set_spi_status(struct gf_dev *gf_dev, int enable)
{
	int rc = 0;
	if (enable) {
		rc = pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_miso_spi);
		rc = pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_mosi_spi);
		rc = pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_cs_spi);
		rc = pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_clk_spi);
	} else {
		rc = pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_miso_pulllow);
		rc = pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_mosi_pulllow);
		rc = pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_cs_pulllow);
		rc = pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_clk_pulllow);
	}
	return rc;
}

int gf3626_request_resource(struct gf_dev *gf_dev, irq_handler_t thread_fn)
{
	int rc = 0;
	pr_info("%s: enter\n", __func__);

	if (!gf_dev->resource_requested) {
				gf_dev->pinctrl_gpios = devm_pinctrl_get (&gf_dev->spi->dev);
		if (IS_ERR (gf_dev->pinctrl_gpios)) {
			rc = PTR_ERR (gf_dev->pinctrl_gpios);
			pr_err("%s can't find fingerprint pinctrl", __func__);
			return rc;
		}

		gf_dev->pins_miso_spi = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "miso_spi");
		if (IS_ERR(gf_dev->pins_miso_spi)) {
			rc = PTR_ERR(gf_dev->pins_miso_spi);
			pr_err("%s can't find fingerprint pinctrl miso_spi\n", __func__);
			return rc;
		}
		gf_dev->pins_miso_pullhigh = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "miso_pullhigh");
		if (IS_ERR(gf_dev->pins_miso_pullhigh)) {
			rc = PTR_ERR(gf_dev->pins_miso_pullhigh);
			pr_err("%s can't find fingerprint pinctrl miso_pullhigh\n", __func__);
			return rc;
		}
		gf_dev->pins_miso_pulllow = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "miso_pulllow");
		if (IS_ERR(gf_dev->pins_miso_pulllow)) {
			rc = PTR_ERR(gf_dev->pins_miso_pulllow);
			pr_err("%s can't find fingerprint pinctrl miso_pulllow\n", __func__);
			return rc;
		}

		gf_dev->pins_mosi_spi = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "mosi_spi");
		if (IS_ERR(gf_dev->pins_mosi_spi)) {
			rc = PTR_ERR(gf_dev->pins_mosi_spi);
			pr_err("%s can't find fingerprint pinctrl pins_mosi_spi\n", __func__);
			return rc;
		}

		gf_dev->pins_mosi_pullhigh = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "mosi_pullhigh");
		if (IS_ERR(gf_dev->pins_mosi_pullhigh)) {
			rc = PTR_ERR(gf_dev->pins_mosi_pullhigh);
			pr_err("%s can't find fingerprint pinctrl mosi_pullhigh\n", __func__);
			return rc;
		}
		gf_dev->pins_mosi_pulllow = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "mosi_pulllow");
		if (IS_ERR(gf_dev->pins_mosi_pulllow)) {
			rc = PTR_ERR(gf_dev->pins_mosi_pulllow);
			pr_err("%s can't find fingerprint pinctrl mosi_pulllow\n", __func__);
			return rc;
		}
		gf_dev->pins_cs_spi = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "cs_spi");
		if (IS_ERR(gf_dev->pins_cs_spi)) {
			rc = PTR_ERR(gf_dev->pins_cs_spi);
			pr_err("%s can't find fingerprint pinctrl pins_cs_spi\n", __func__);
			return rc;
		}
		gf_dev->pins_cs_pullhigh = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "cs_pullhigh");
		if (IS_ERR(gf_dev->pins_cs_pullhigh)) {
			rc = PTR_ERR(gf_dev->pins_cs_pullhigh);
			pr_err("%s can't find fingerprint pinctrl cs_pullhigh\n", __func__);
			return rc;
		}
		gf_dev->pins_cs_pulllow = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "cs_pulllow");
		if (IS_ERR(gf_dev->pins_cs_pulllow)) {
			rc = PTR_ERR(gf_dev->pins_cs_pulllow);
			pr_err("%s can't find fingerprint pinctrl cs_pulllow\n", __func__);
			return rc;
		}
		gf_dev->pins_clk_spi = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "clk_spi");
		if (IS_ERR(gf_dev->pins_clk_spi)) {
			rc = PTR_ERR(gf_dev->pins_clk_spi);
			pr_err("%s can't find fingerprint pinctrl pins_clk_spi\n", __func__);
			return rc;
		}
		gf_dev->pins_clk_pullhigh = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "clk_pullhigh");
		if (IS_ERR(gf_dev->pins_clk_pullhigh)) {
			rc = PTR_ERR(gf_dev->pins_clk_pullhigh);
			pr_err("%s can't find fingerprint pinctrl clk_pullhigh\n", __func__);
			return rc;
		}
		gf_dev->pins_clk_pulllow = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "clk_pulllow");
		if (IS_ERR(gf_dev->pins_clk_pulllow)) {
			rc = PTR_ERR(gf_dev->pins_clk_pulllow);
			pr_err("%s can't find fingerprint pinctrl clk_pulllow\n", __func__);
			return rc;
		}

		//VDD
		if (gf_dev->vdd_use_pmic) {
			gf_dev->vreg = regulator_get(&gf_dev->spi->dev, "vfp");
			if (IS_ERR(gf_dev->vreg)) {
				pr_info("%s Unable to get vfp\n", __func__);
				return -EINVAL;
			}
		}

		//VDD
		if (gf_dev->vdd_use_gpio) {
			rc = gpio_request(gf_dev->vdd_en_gpio, "goodix_vdd_en");
			if (rc) {
				pr_info("%s Failed to request VDD GPIO. rc = %d\n", __func__, rc);
				goto error_gpio_vdd;
			}
		}

		/*get reset resource */
		rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
		if (rc) {
			pr_info("Failed to request RESET GPIO. rc = %d\n", rc);
			goto error_gpio_reset;
		}
		gpio_direction_output(gf_dev->reset_gpio, 0);

		/*get irq resourece */
		rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
		if (rc) {
			pr_info("Failed to request IRQ GPIO. rc = %d\n", rc);
			goto error_gpio_irq;
		}
		gpio_direction_input(gf_dev->irq_gpio);

		/*register irq*/
		gf_dev->irq = gf3626_irq_num(gf_dev);

		rc = request_threaded_irq(gf_dev->irq, NULL, thread_fn,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "gf", gf_dev);

		if (rc) {
			pr_err("failed to request IRQ:%d\n", gf_dev->irq);
			goto error_irq;
		}
		enable_irq_wake(gf_dev->irq);

		gf_dev->resource_requested = true;
	} else {
		 pr_info("%s resource is requested already.", __func__);
	}

	/*power on and enable irq*/
	rc = gf3626_power_on(gf_dev);
	if (rc) {
		pr_err("gf9518 power on fail.\n");
		goto error_power;
	}

	gf_enable_irq(gf_dev);
	gf3626_hw_reset(gf_dev, 3);
	gf_dev->device_available = 1;

	pr_info("%s: exit\n", __func__);
	return 0;

error_power:
	gf_disable_irq(gf_dev);
error_irq:
	free_irq(gf_dev->irq, gf_dev);

	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
	}
error_gpio_irq:
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
	}
error_gpio_reset:
	if (gf_dev->vdd_use_gpio) {
		if (gpio_is_valid(gf_dev->vdd_en_gpio)) {
			gpio_free(gf_dev->vdd_en_gpio);
		}
	}
error_gpio_vdd:
	pr_err("%s requset source failed.\n", __func__);
	return rc;
}

void gf3626_release_resource(struct gf_dev *gf_dev)
{
	pr_info("%s: enter\n", __func__);
	if (gf_dev->resource_requested) {
		gf_disable_irq(gf_dev);
		if (gf_dev->irq)
			free_irq(gf_dev->irq, gf_dev);
		gf3626_power_off(gf_dev);

		if (gpio_is_valid(gf_dev->irq_gpio)) {
			gpio_free(gf_dev->irq_gpio);
			pr_info("remove irq_gpio success\n");
		}

		if (gpio_is_valid(gf_dev->reset_gpio)) {
			gpio_free(gf_dev->reset_gpio);
			pr_info("remove reset_gpio success\n");
		}

		if (gf_dev->vdd_use_pmic) {
			regulator_put(gf_dev->vreg);
			pr_info("%s regulator put vcc_spi\n", __func__);
		}

		if (gf_dev->vdd_use_gpio) {
			if (gpio_is_valid(gf_dev->vdd_en_gpio)) {
				gpio_free(gf_dev->vdd_en_gpio);
				pr_info("remove vdd_en_gpio success\n");
			}
		}

		if (!IS_ERR (gf_dev->pinctrl_gpios)) {
			devm_pinctrl_put(gf_dev->pinctrl_gpios);
			pr_info("%s devm_pinctrl_put", __func__);
		}
		gf_dev->resource_requested = false;
	} else {
		pr_warn("resource not request, can not release resource.\n");
	}
	pr_info("%s: exit\n", __func__);
}

void gf3626_cleanup(struct gf_dev	 *gf_dev)
{
	pr_info("[info] %s\n", __func__);
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

int gf3626_power_on(struct gf_dev *gf_dev)
{
    int rc = 0;
	pr_info("gf:%s, entry\n", __func__);

	if (!gf_dev->resource_requested) {
		pr_info("%s, resource_requested: %d.\n", __func__, gf_dev->resource_requested);
		return 0;
	}

	if (gf_dev->vdd_use_pmic) {
		if (regulator_is_enabled(gf_dev->vreg)) {
			pr_info("%s,power state:on,don't set repeatedly!\n", __func__);
			return rc;
		}

		rc = regulator_set_load(gf_dev->vreg, 600000);
		if (rc < 0) {
			pr_err("%s: regulator_set_load(uA_load=%d) failed. rc=%d\n",
				__func__, 1000, rc);
		}

		if (regulator_count_voltages(gf_dev->vreg) > 0) {
			rc = regulator_set_voltage(gf_dev->vreg, 3000000, 3000000);
				if (rc) {
					pr_info(KERN_ERR "gf3658:Unable to set voltage on vcc_spi");
				}
		}
		rc = regulator_enable(gf_dev->vreg);
	}

	if (gf_dev->vdd_use_gpio) {
		pr_info("gf:%s, entry\n", __func__);
		rc = gpio_direction_output(gf_dev->vdd_en_gpio, 1);
		if (rc) {
			pr_info ("gf5288 power on fail.\n");
			return -EINVAL;
		}
	}
    gf3626_set_spi_status (gf_dev, 1);

	pr_info("gf:%s, exit\n", __func__);
    return rc;
}

int gf3626_power_off(struct gf_dev *gf_dev)
{
    int rc = 0;
	pr_info("gf:%s, entry\n", __func__);

	if (!gf_dev->resource_requested) {
		pr_info("%s, resource_requested: %d.\n", __func__, gf_dev->resource_requested);
		return 0;
	}

	gf3626_set_spi_status (gf_dev, 0);

	if (gf_dev->vdd_use_pmic) {
		if (gf_dev->vreg) {
			rc = regulator_set_load(gf_dev->vreg, 0);
			if (rc < 0) {
				pr_err("%s: regulator_set_load(uA_load=%d) failed. rc=%d\n",
					__func__, 0, rc);
			}
			if (regulator_is_enabled(gf_dev->vreg)) {
				gpio_direction_output(gf_dev->reset_gpio, 0);
				regulator_disable(gf_dev->vreg);
			}
			pr_info(KERN_ERR "gf3658: disable  vcc_spi %d\n", rc);
		}
	}

	if (gf_dev->vdd_use_gpio) {
		rc = gpio_direction_output(gf_dev->reset_gpio, 0);
		if (rc) {
			pr_info("gf3658 set reset gpio output fail.\n");
			return -EIO;
		}

		rc = gpio_direction_output(gf_dev->vdd_en_gpio, 0);
		if (rc) {
			pr_info ("gf5288 power off fail.\n");
			return -EINVAL;
		}
	}

	pr_info("gf:%s, exit\n", __func__);
    return rc;
}

int gf_hw_get_power_state(struct gf_dev *gf_dev)
{

	int retval = 0;
	retval = gpio_get_value(gf_dev->vdd_en_gpio);
	pr_info("gf:%s, retval=%d\n", __func__, retval);
	return retval;
}

int gf3626_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -EINVAL;
	}
	//gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(10);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf3626_irq_num(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -EINVAL;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

