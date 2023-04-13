/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <cam_sensor_cmn_header.h>
#include <cam_sensor_util.h>
#include <cam_sensor_io.h>
#include <cam_req_mgr_util.h>
#include "cam_lens_driver_soc.h"
#include "cam_soc_util.h"

int32_t cam_lens_driver_parse_dt(struct cam_lens_driver_ctrl_t *l_ctrl,
	struct device *dev)
{
	int32_t                         rc = 0;
	const char                      *lens_driver_name = NULL;
	struct cam_hw_soc_info          *soc_info = &l_ctrl->soc_info;
	struct cam_lens_driver_soc_private *soc_private =
		(struct cam_lens_driver_soc_private *)l_ctrl->soc_info.soc_private;
	struct device_node              *of_node = NULL;

	/* Initialize mutex */
	mutex_init(&(l_ctrl->lens_driver_mutex));
	mutex_init(&(l_ctrl->spi_sync_mutex));
	rc = cam_soc_util_get_dt_properties(soc_info);
	if (rc < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "parsing common soc dt(rc %d)", rc);
		return rc;
	}

	of_node = soc_info->dev->of_node;

	rc = of_property_read_string(dev->of_node, "lens-driver",
				&lens_driver_name);
	if (rc < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to read lens-driver(rc %d)", rc);
		goto end;
	} else {
		memcpy(&soc_private->cam_lens_driver_name, lens_driver_name,
			sizeof(soc_private->cam_lens_driver_name));
		CAM_DBG(CAM_LENS_DRIVER, "lens-driver: %s",
			soc_private->cam_lens_driver_name);
	}

	rc = of_get_named_gpio(of_node, "cs-gpio", 0);
	if (rc < 0)
		CAM_ERR(CAM_LENS_DRIVER, "cs-gpio not found (rc : %d)", rc);
	else
		soc_private->cam_lens_driver_gpio_num_info[LENS_DRIVER_CS] = rc;

	rc = of_get_named_gpio(of_node, "reset-gpio", 0);
	if (rc < 0)
		CAM_ERR(CAM_LENS_DRIVER, "reset-gpio not found (rc : %d)", rc);
	else
		soc_private->cam_lens_driver_gpio_num_info[LENS_DRIVER_RESET] = rc;


	rc = of_get_named_gpio(of_node, "spi-srdy-gpio", 0);
	if (rc < 0)
		CAM_ERR(CAM_LENS_DRIVER, "spi-srdy-gpio not found (rc : %d)", rc);
	else
		soc_private->cam_lens_driver_gpio_num_info[LENS_DRIVER_SPI_SRDY] = rc;


	rc = of_get_named_gpio(of_node, "drv-ext1-gpio", 0);
	if (rc < 0)
		CAM_ERR(CAM_LENS_DRIVER, "drv-ext1-gpio not found (rc : %d)", rc);
	else
		soc_private->cam_lens_driver_gpio_num_info[LENS_DRIVER_DRV_EX1] = rc;


	rc = of_get_named_gpio(of_node, "drv-ext2-gpio", 0);
	if (rc < 0)
		CAM_ERR(CAM_LENS_DRIVER, "drv-ext2-gpio not found (rc : %d)", rc);
	else
		soc_private->cam_lens_driver_gpio_num_info[LENS_DRIVER_DRV_EX2] = rc;

end:
	return rc;
}
