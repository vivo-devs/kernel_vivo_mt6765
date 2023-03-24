/**
 * plat-msm8916.c
 *
 **/

#include <linux/stddef.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/platform_data/spi-mt65xx.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include "ff_log.h"
#include "ff_err.h"
#include "ff_ctl.h"

# undef LOG_TAG
#define LOG_TAG "mt6765"

/* TODO: */
#define FF_COMPATIBLE_NODE "mediatek,focaltech-fp"

/* Define pinctrl state types. */
typedef enum {
	FF_PINCTRL_STATE_CS_ACT = 0,
	FF_PINCTRL_STATE_CLK_ACT,
	FF_PINCTRL_STATE_MOSI_ACT,
	FF_PINCTRL_STATE_MISO_ACT,
	FF_PINCTRL_STATE_CS_CLR,
	FF_PINCTRL_STATE_CLK_CLR,
	FF_PINCTRL_STATE_MOSI_CLR,
	FF_PINCTRL_STATE_MISO_CLR,
	FF_PINCTRL_STATE_MAXIMUM /* Array size */
} ff_pinctrl_state_t;

static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
	"cs_spi", "clk_spi", "mosi_spi", "miso_spi", "cs_pulllow", "clk_pulllow", "mosi_pulllow", "miso_pulllow",
};

/*
 * GPIO context definition and its singleton instance.
 */
typedef struct {
	int32_t vdd_use_gpio;
	int32_t vdd_use_pmic;
	struct regulator *vreg;
	struct device_node *dev_node;
	struct pinctrl *g_pinctrl;
	struct pinctrl_state *g_pin_states[FF_PINCTRL_STATE_MAXIMUM];
	int32_t gpio_rst_pin;
	int32_t gpio_int_pin;
	int32_t gpio_power_pin;
	bool b_spiclk_enabled;
} ff_plat_context_t;

static ff_plat_context_t ff_plat_context = {
	.gpio_rst_pin   = -1,
	.gpio_int_pin   = -1,
	.gpio_power_pin = -1,
	.b_spiclk_enabled = false,
}, *g_context = &ff_plat_context;

#if USE_PLATFORM_BUS
extern struct platform_device *g_dev;
#elif USE_SPI_BUS
extern struct spi_device *g_dev;
#endif

extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);
extern void vfp_spi_clk_enable(uint8_t bonoff);

int ff_ctl_parse_dts(void)
{
	int err = 0, gpio;
	bool b_config_dirtied = false;

	if (unlikely(!g_context)) {
		return (-ENOSYS);
	}

	/* Find device tree node. */
	g_context->dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE);
	if (!g_context->dev_node) {
		FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE);
		return (-ENODEV);
	}

	/* Parse PWR/VDD pin. */
	g_context->vdd_use_gpio = of_property_read_bool(g_context->dev_node, "fp,vdd_use_gpio");
	g_context->vdd_use_pmic = of_property_read_bool(g_context->dev_node, "fp,vdd_use_pmic");

	if (g_context->vdd_use_gpio) {
		gpio = of_get_named_gpio(g_context->dev_node, "fp,vdd_gpio", 0);
		if (gpio > 0) {
			g_context->gpio_power_pin = gpio;
			b_config_dirtied = true;
		}
		if (!gpio_is_valid(g_context->gpio_power_pin)) {
			FF_LOGE("g_context->gpio_power_pin(%d) is invalid.", g_context->gpio_power_pin);
			return (-ENODEV);
		}
	}

	/*Parse RST pin. */
	gpio = of_get_named_gpio(g_context->dev_node, "fp,reset_gpio", 0);
	if (gpio > 0) {
		g_context->gpio_rst_pin = gpio;
		b_config_dirtied = true;
	}
	if (!gpio_is_valid(g_context->gpio_rst_pin)) {
		FF_LOGE("g_context->gpio_rst_pin(%d) is invalid.", g_context->gpio_rst_pin);
		return (-ENODEV);
	}

	/* Parse INT pin. */
	gpio = of_get_named_gpio(g_context->dev_node, "fp,irq-gpio", 0);
	if (gpio > 0) {
		g_context->gpio_int_pin = gpio;
		b_config_dirtied = true;
	}
	if (!gpio_is_valid(g_context->gpio_int_pin)) {
		FF_LOGE("g_context->gpio_int_pin(%d) is invalid.", g_context->gpio_int_pin);
		return (-ENODEV);
	}

	/* Configuration is dirty, must sync back to HAL. */
	if (!err && b_config_dirtied) {
		err = 1;
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

int ff_ctl_init_pins(int *irq_num)
{
	int err = 0, i = 0;
	struct platform_device *pdev = NULL;
	FF_LOGV("'%s' enter.", __func__);

	if (unlikely(!g_context)) {
		return (-ENOSYS);
	}

	/* Convert to platform device. */
	pdev = of_find_device_by_node(g_context->dev_node);
	if (!pdev) {
		FF_LOGE("of_find_device_by_node(..) failed.");
		return (-ENODEV);
	}

	/* Retrieve the pinctrl handler. */
	g_context->g_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (!g_context->g_pinctrl) {
		FF_LOGE("devm_pinctrl_get(..) failed.");
		return (-ENODEV);
	}

	/* Initialize all spi pins. */
	for (i = 0; i < FF_PINCTRL_STATE_MAXIMUM; ++i) {
		g_context->g_pin_states[i] = pinctrl_lookup_state(g_context->g_pinctrl, g_pinctrl_state_names[i]);
		if (!g_context->g_pin_states[i]) {
			FF_LOGE("can't find pinctrl state for '%s'.", g_pinctrl_state_names[i]);
			break;
		}
	}
	if (i < FF_PINCTRL_STATE_MAXIMUM) {
		return (-ENODEV);
	}

	/* Initialize PWR/VDD pin. */
	if (g_context->vdd_use_pmic) {
		g_context->vreg = regulator_get(&pdev->dev, "vfp");
		if (IS_ERR(g_context->vreg)) {
			FF_LOGE("Unable to get vfp");
			return -EINVAL;
		}
	}

	if (g_context->vdd_use_gpio) {
		err = gpio_request(g_context->gpio_power_pin, "ff_gpio_power_pin");
		if (err) {
			FF_LOGE("gpio_request(%d) = %d.", g_context->gpio_power_pin, err);
			return err;
		}
		err = gpio_direction_output(g_context->gpio_power_pin, 0); // power off.
		if (err) {
			FF_LOGE("gpio_direction_output(%d, 0) = %d.", g_context->gpio_power_pin, err);
			return err;
		}
	}

	/* Initialize RST pin. */
	err = gpio_request(g_context->gpio_rst_pin, "ff_gpio_rst_pin");
	if (err) {
		FF_LOGE("gpio_request(%d) = %d.", g_context->gpio_rst_pin, err);
		return err;
	}
	err = gpio_direction_output(g_context->gpio_rst_pin, 1);
	if (err) {
		FF_LOGE("gpio_direction_output(%d, 1) = %d.", g_context->gpio_rst_pin, err);
		return err;
	}

	/* Initialize INT pin. */
	err = gpio_request(g_context->gpio_int_pin, "ff_gpio_int_pin");
	if (err) {
		FF_LOGE("gpio_request(%d) = %d.", g_context->gpio_int_pin, err);
		return err;
	}
	err = gpio_direction_input(g_context->gpio_int_pin);
	if (err) {
		FF_LOGE("gpio_direction_input(%d) = %d.", g_context->gpio_int_pin, err);
		return err;
	}
	/* Retrieve the IRQ number. */
	*irq_num = gpio_to_irq(g_context->gpio_int_pin);
	if (*irq_num < 0) {
		FF_LOGE("gpio_to_irq(%d) failed.", g_context->gpio_int_pin);
		return (-EIO);
	} else {
		FF_LOGD("gpio_to_irq(%d) = %d.", g_context->gpio_int_pin, *irq_num);
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

int ff_ctl_free_pins(void)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);
	
	if (unlikely(!g_context)) {
		return (-ENOSYS);
	}

	/* Release GPIO resources. */
	if (gpio_is_valid(g_context->gpio_int_pin)) {
		gpio_free(g_context->gpio_int_pin  );
	}

	if (gpio_is_valid(g_context->gpio_rst_pin)) {
		gpio_free(g_context->gpio_rst_pin  );
	}

	if (g_context->vdd_use_gpio && gpio_is_valid(g_context->gpio_power_pin)) {
		gpio_free(g_context->gpio_power_pin);
	}

	if (g_context->vdd_use_pmic) {
		regulator_put(g_context->vreg);
	}

	if (g_context->g_pinctrl) {
		devm_pinctrl_put(g_context->g_pinctrl);
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

#if 0
int ff_ctl_enable_spiclk(bool on)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);
	FF_LOGD("clock: '%s'.", on ? "enable" : "disabled");

	if (unlikely(!g_dev)) {
		return (-ENOSYS);
	}
	
	/* Control the clock source. */
	if (on && !g_context->b_spiclk_enabled) {
		mt_spi_enable_master_clk(g_dev);
		g_context->b_spiclk_enabled = true;
	} else if (!on && g_context->b_spiclk_enabled) {
		mt_spi_disable_master_clk(g_dev);
		g_context->b_spiclk_enabled = false;
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}
#else
int ff_ctl_enable_spiclk(bool on)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);
	FF_LOGD("clock: '%s'.", on ? "enable" : "disabled");
	vfp_spi_clk_enable(on);
	FF_LOGV("'%s' leave.", __func__);
	return err;
}
#endif

int ff_ctl_set_spi_status(bool on)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);
	FF_LOGD("spi status: '%s'.", on ? "spi mode" : "gpio mode");

	if (unlikely(!g_context)) {
		return (-ENOSYS);
	}

	if (on) {
		err = pinctrl_select_state(g_context->g_pinctrl, g_context->g_pin_states[FF_PINCTRL_STATE_CS_ACT]);
		err = pinctrl_select_state(g_context->g_pinctrl, g_context->g_pin_states[FF_PINCTRL_STATE_CLK_ACT]);
		err = pinctrl_select_state(g_context->g_pinctrl, g_context->g_pin_states[FF_PINCTRL_STATE_MOSI_ACT]);
		err = pinctrl_select_state(g_context->g_pinctrl, g_context->g_pin_states[FF_PINCTRL_STATE_MISO_ACT]);
	} else {
		err = pinctrl_select_state(g_context->g_pinctrl, g_context->g_pin_states[FF_PINCTRL_STATE_CS_CLR]);
		err = pinctrl_select_state(g_context->g_pinctrl, g_context->g_pin_states[FF_PINCTRL_STATE_CLK_CLR]);
		err = pinctrl_select_state(g_context->g_pinctrl, g_context->g_pin_states[FF_PINCTRL_STATE_MOSI_CLR]);
		err = pinctrl_select_state(g_context->g_pinctrl, g_context->g_pin_states[FF_PINCTRL_STATE_MISO_CLR]);
	}
	FF_LOGV("'%s' leave.", __func__);
	return 0;
}

int ff_ctl_enable_power(bool on)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);
	FF_LOGD("power: '%s'.", on ? "on" : "off");

	if (unlikely(!g_context)) {
		return (-ENOSYS);
	}
	
	if (g_context->vdd_use_pmic) {
		if (on) {
			if (0 == (regulator_is_enabled(g_context->vreg))) {
				err = regulator_set_load(g_context->vreg, 600000);
				if (err < 0) {
					FF_LOGE("regulator_set_load(..) = %d.", err);
				return err;
				}
				if (regulator_count_voltages(g_context->vreg) > 0) {
					err = regulator_set_voltage(g_context->vreg, 3000000, 3000000);
					if (err) {
					FF_LOGE("regulator_set_voltage(..) = %d.", err);
					return err;
					}
				}
				err = regulator_enable(g_context->vreg);
			} else if((regulator_is_enabled(g_context->vreg)) > 0){
				FF_LOGE("regulator_is_enabled(..) = %d.", err);
				return err;
			}else{
				FF_LOGE("regulator enabled fail = %d.");
				return (-ENOSYS);
			}
		}  else {
			if (g_context->vreg) {
				err = regulator_set_load(g_context->vreg, 0);
				if (err < 0) {
					FF_LOGE("regulator_set_load(..) = %d.", err);
					return err;
				}
				if (regulator_is_enabled(g_context->vreg)) {
					regulator_disable(g_context->vreg);
				}
			}
		}
	}

	if (g_context->vdd_use_gpio) {
		if (on) {
			err = gpio_direction_output(g_context->gpio_power_pin, 1);
			msleep(5);
		} else {
			msleep(5);
			err = gpio_direction_output(g_context->gpio_power_pin, 0);
		}
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

int ff_ctl_reset_device(void)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);

	if (unlikely(!g_context)) {
		return (-ENOSYS);
	}

	/* 3-1: Pull down RST pin. */
	err = gpio_direction_output(g_context->gpio_rst_pin, 0);

	/* 3-2: Delay for 10ms. */
	mdelay(10);

	/* Pull up RST pin. */
	err = gpio_direction_output(g_context->gpio_rst_pin, 1);

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

const char *ff_ctl_arch_str(void)
{
	return "mt6765";
}

