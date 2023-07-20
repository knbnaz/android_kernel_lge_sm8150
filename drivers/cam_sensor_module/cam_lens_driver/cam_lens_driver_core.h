/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_LENS_DRIVER_CORE_H_
#define _CAM_LENS_DRIVER_CORE_H_

#include "cam_lens_driver_dev.h"
#include "cam_req_mgr_workq.h"

#define LDM_WORKQ_NUM_TASK      15

/**
 * @a_ctrl: Lens Driver ctrl structure
 * @arg:    Camera control command argument
 *
 * This API handles the camera control argument reached to lens_driver
 */
int32_t cam_lens_driver_cmd(struct cam_lens_driver_ctrl_t *a_ctrl, void *arg);

/**
 * @a_ctrl: Lens Driver ctrl structure
 *
 * This API handles the shutdown ioctl/close
 */
void cam_lens_driver_shutdown(struct cam_lens_driver_ctrl_t *a_ctrl);

/*
 * @spi:        spi device handle
 * @txbuf:      Transmit data
 * @srxbuf:     Receive data
 * @num_byte:   Number of bytes
 *
 * This API will call spi_sync() byte by byte data communication.
 */
int32_t cam_lens_driver_spi_byte_txfr(struct spi_device *spi,
	const char *txbuf, char *rxbuf, int32_t num_byte);

/*
 * @spi:        spi device handle
 * @txbuf:      Transmit data
 * @srxbuf:     Receive data
 * @num_byte:   Number of bytes
 *
 * This API will call spi_sync() single time for num_byte  data communication.
 */
int32_t cam_lens_driver_spi_txfr(struct spi_device *spi, const char *txbuf,
	char *rxbuf, int32_t num_byte);

/*
 * This API will call create work queue to perform motor operation.
 */
int32_t cam_lens_driver_create_wq(struct cam_lens_driver_ctrl_t *l_ctrl);

#endif /* _CAM_LENS_DRIVER_CORE_H_ */
