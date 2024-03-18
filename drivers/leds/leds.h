/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * LED Core
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 */
#ifndef __LEDS_H_INCLUDED
#define __LEDS_H_INCLUDED

#include <linux/rwsem.h>
#include <linux/leds.h>

static inline int led_get_brightness(struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

void led_init_core(struct led_classdev *led_cdev);
void led_stop_software_blink(struct led_classdev *led_cdev);
void led_set_brightness_nopm(struct led_classdev *led_cdev,
				enum led_brightness value);
void led_set_brightness_nosleep(struct led_classdev *led_cdev,
				enum led_brightness value);

extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;
extern struct list_head trigger_list;
extern const char * const led_colors[LED_COLOR_ID_MAX];

#ifdef CONFIG_LEDS_LGE_EMOTIONAL
struct pwm_setting {
	u64	pre_period_ns;
	u64	period_ns;
	u64	duty_ns;
};

struct led_setting {
	u64			on_ms;
	u64			off_ms;
	enum led_brightness	brightness;
	bool			blink;
	bool			breath;
};

struct qpnp_led_dev {
	struct led_classdev	cdev;
	struct pwm_device	*pwm_dev;
	struct pwm_setting	pwm_setting;
	struct led_setting	led_setting;
	struct qpnp_tri_led_chip	*chip;
	struct mutex		lock;
	const char		*label;
	const char		*default_trigger;
	u8			id;
	bool			blinking;
	bool			breathing;
};

struct qpnp_tri_led_chip {
	struct device		*dev;
	struct regmap		*regmap;
	struct qpnp_led_dev	*leds;
	struct nvmem_device	*pbs_nvmem;
	struct mutex		bus_lock;
	struct delayed_work     init_leds;;
	int			num_leds;
	u16			reg_base;
	u8			subtype;
	u8			bitmap;
};
#endif

#endif	/* __LEDS_H_INCLUDED */
