/* production_test.c
 *
 * Copyright (C) 2015 LGE.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

#define MODULE_MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MODULE_MAX_LOG_FILE_COUNT	5
#define MODULE_FILE_STR_LEN		(128)

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

int module_touch_check_boot_mode(struct device *dev)
{
	int ret = TOUCH_NORMAL_BOOT;
#ifdef CONFIG_LGE_USB_FACTORY
	bool factory_boot = false;
	struct touch_core_data *ts = plist->touch_core_data;
#endif
	TOUCH_TRACE();

#ifdef CONFIG_LGE_USB_FACTORY
	factory_boot = lge_get_factory_boot();

	if (factory_boot) {
		switch (atomic_read(&ts->state.mfts)) {
		case MFTS_NONE:
			ret = TOUCH_MINIOS_AAT;
			break;
		case MFTS_FOLDER:
			ret = TOUCH_MINIOS_MFTS_FOLDER;
			break;
		case MFTS_FLAT:
			ret = TOUCH_MINIOS_MFTS_FLAT;
			break;
		case MFTS_CURVED:
			ret = TOUCH_MINIOS_MFTS_CURVED;
			break;
		default:
			ret = TOUCH_MINIOS_AAT;
			break;
		}
	}
#endif

	return ret;
}
