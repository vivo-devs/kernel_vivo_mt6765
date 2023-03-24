/*
 * Copyright (C) 2016 vivo Inc.
 * This head list is used for otg gpio pull 
 * pin ctrl
 * modify by wutianwen
 */
#include <linux/wakelock.h>

struct mtk_otg_gpio_pinctrl{
	bool id_pin_state;
	bool id_state_error;
	bool id_state_change;
	bool last_id_pin_state;
	bool otg_chip_compatible_mode;
	struct pinctrl *pinctrl;
	unsigned int usbid_gpio;
	unsigned int platform_id;
	struct wake_lock switchlock;
	struct delayed_work usbid_polling_work;
	struct pinctrl_state *otg_iddig_gpio;
	struct pinctrl_state *otg_iddig_gpio_pull_down;
	struct pinctrl_state *otg_pull_high;
	struct pinctrl_state *otg_pull_low;
	struct pinctrl_state *otg_pull_in;
	struct pinctrl_state *otg_pull1_high;
	struct pinctrl_state *otg_pull1_low;
	struct pinctrl_state *otg_pull1_in;
	struct pinctrl_state *otg_iddig_init;
	//struct pinctrl_state *otg_iddig2_gpio;
	//struct pinctrl_state *otg_iddig2_init;
	struct pinctrl_state *otg167_pull1_high;
	struct pinctrl_state *otg167_pull1_low;
	struct pinctrl_state *otg167_pull1_in;
	
};
