/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_LENS_DRIVER_DEV_H_
#define _CAM_LENS_DRIVER_DEV_H_

#include <cam_sensor_io.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/irqreturn.h>
#include <linux/ion.h>
#include <linux/iommu.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <cam_cci_dev.h>
#include <cam_sensor_cmn_header.h>
#include <cam_subdev.h>
#include <cam_sensor_spi.h>
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_debug_util.h"
#include "cam_context.h"
#include "cam_req_mgr_workq.h"

#define CAMX_LENS_DRIVER_DEV_NAME "cam-lens-driver-dev"

enum cam_lens_driver_state {
	CAM_LENS_DRIVER_INIT,
	CAM_LENS_DRIVER_ACQUIRE,
	CAM_LENS_DRIVER_CONFIG,
	CAM_LENS_DRIVER_START,
};

enum cam_lens_driver_gpio_type {
	LENS_DRIVER_CS,
	LENS_DRIVER_RESET,
	LENS_DRIVER_SPI_SRDY,
	LENS_DRIVER_DRV_EX1,
	LENS_DRIVER_DRV_EX2,
	LENS_DRIVER_MAX_GPIO,
};

enum cam_lens_driver_motor_type {
	STM_MOTOR_PIRIS,
	STM_MOTOR_AF,
	STM_MOTOR_ZOOM,
	STM_MOTOR_DCIRIS,
	STM_MOTOR_MAX
};

enum cam_lens_driver_motor_power_level {
	POWER_LEVEL_OFF,
	POWER_LEVEL_WEAK,
	POWER_LEVEL_STRONG,
	POWER_LEVEL_MAX
};

enum cam_lens_driver_motor_move_direction {
	BACKWARD,
	FORWARD
};

enum cam_lens_driver_motor_driving_method {
	ABSOLUTE_DRV,
	RELATIVE_DRV
};

enum motor_driving_mode {
	DRV_MODE_22_PHASE,
	DRV_MODE_12_PHASE,
	DRV_MODE_16_MICROSTEP,
	DRV_MODE_64_MICROSTEP,
	DRV_MODE_512_MICROSTEP
};

struct cam_lens_driver_ctrl_t;

/**
 * struct cam_lens_driver_stm_motor_data
 * @channelNum: channel on which motor is connected
 * @is_moving: Motor running status
 * @motorId: MotorID used for getting moving status
 * @backLashStepCount: Back lash step value of motor
 * @is_backlash_step_added: It becomes true when motor
 *      run in backward direction.
 * @actual_position: Motor current position
 * @PI_position: PI position of motor
 * @speed: Speed of STM motor in PPS
 * @lensPowerLevel: Excitation level of motor
 * @last_drection: Motor driving direction
 * @drv_mode: Motor driving mode
 */
struct cam_lens_driver_stm_motor_data {
	uint8_t channelNum;
	uint8_t is_moving;
	uint8_t motorId;
	uint8_t backLashStepCount;
	uint8_t is_backlash_step_added;
	int32_t actual_position;
	uint32_t PI_position;
	uint32_t speed;
	enum cam_lens_driver_motor_power_level lensPowerLevel;
	enum cam_lens_driver_motor_move_direction last_direction;
	enum motor_driving_mode drv_mode;
};

/**
 * struct cam_lens_driver_soc_private
 * @cam_lens_driver_gpio_num_info: GPIO list
 * @cam_lens_driver_name: Lens Driver Name
 */
struct cam_lens_driver_soc_private {
	uint16_t cam_lens_driver_gpio_num_info[LENS_DRIVER_MAX_GPIO];
	char cam_lens_driver_name[CAM_CTX_DEV_NAME_MAX_LENGTH];
};

/**
 * struct intf_params
 * @device_hdl: Device Handle
 * @session_hdl: Session Handle
 * @ops: KMD operations
 * @crm_cb: Callback API pointers
 */
struct intf_params {
	int32_t device_hdl;
	int32_t session_hdl;
	int32_t link_hdl;
	struct cam_req_mgr_kmd_ops ops;
	struct cam_req_mgr_crm_cb *crm_cb;
};

typedef int32_t (*lens_driver_initialization) (
				struct cam_lens_driver_ctrl_t *);
typedef int32_t (*lens_driver_PI_calibration) (
				struct cam_lens_driver_ctrl_t *,
				struct cam_lens_driver_PI_calibration *,
				uint32_t *);
typedef int32_t (*lens_driver_set_motor_control) (
				struct cam_lens_driver_ctrl_t *,
				struct cam_lens_driver_set_motor_control *);
typedef int32_t (*lens_driver_absolute_motor_drive) (
				struct cam_lens_driver_ctrl_t *,
				struct cam_lens_driver_absolute_motor_move *);
typedef int32_t (*lens_driver_relative_motor_drive) (
				struct cam_lens_driver_ctrl_t *,
				struct cam_lens_driver_relative_motor_move *);
typedef int32_t (*lens_driver_get_current_motor_status) (
				struct cam_lens_driver_ctrl_t *,
				uint8_t,
				struct cam_lens_driver_motor_status *);
typedef int32_t (*lens_driver_get_motor_power_level) (
				struct cam_lens_driver_ctrl_t *,
				enum cam_lens_driver_motor_power_level *);
typedef int32_t (*lens_driver_set_motor_actual_position) (
				struct cam_lens_driver_ctrl_t *,
				uint8_t,
				int32_t);
typedef int32_t (*lens_driver_abort_motor_moving_ops) (
				struct cam_lens_driver_ctrl_t *,
				uint8_t,
				int32_t *);
typedef int32_t (*lens_driver_deinitialization) (
				struct cam_lens_driver_ctrl_t *);

struct lens_driver_ops {
	lens_driver_initialization lens_driver_init;
	lens_driver_PI_calibration PI_calibration;
	lens_driver_set_motor_control set_motor_control;
	lens_driver_absolute_motor_drive absolute_motor_drive;
	lens_driver_relative_motor_drive relative_motor_drive;
	lens_driver_get_current_motor_status get_motor_status;
	lens_driver_set_motor_actual_position set_motor_actual_position;
	lens_driver_abort_motor_moving_ops abort_motor_moving_ops;
	lens_driver_get_motor_power_level get_power_level;
	lens_driver_deinitialization lens_driver_deinit;
};

/**
 * struct cam_lens_driver_info
 * @cam_lens_driver_name    :   Lens Driver Name
 * @lens_driver_intf        :   Lens Driver function pointer
 */
struct cam_lens_driver_info {
	char cam_lens_driver_name[CAM_CTX_DEV_NAME_MAX_LENGTH];
	struct lens_driver_ops lens_driver_intf;
};

/**
 * struct lens_driver_cmd_work_data
 * @cmd_type        :   Motor operation command type
 * @payload         :   payload data for motor operation command
 */
struct lens_driver_cmd_work_data {
	enum cam_lens_driver_command_type cmd_type;
	void *payload;
};

/**
 * struct cam_lens_driver_ctrl_t
 * @device_name: Device name
 * @spi_client: SPI device handle
 * @soc_info: Soc Info of camera hardware driver module
 * @lens_driver_mutex: Lens Driver mutex
 * @spi_sync_mutex: SPI bus sync mutex
 * @cam_lens_driver_state: Lens driver state
 * @motor_drv_method: Motor driving method
 * @v4l2_dev_str: V4L2 device structure
 * @lens_driver_intf: Lens driver interface APIs
 * @stm_motor_info: Stepper motor info
 * @lens_capability: Lens driver Capability
 * @last_flush_req: Last request to flush
 * @cmd_work: Workqueue for motor driving operation
 * @cmd_work_data: Command data for motor operation
 */
struct cam_lens_driver_ctrl_t {
	char device_name[CAM_CTX_DEV_NAME_MAX_LENGTH];
	struct spi_device *spi_client;
	struct cam_hw_soc_info soc_info;
	struct mutex lens_driver_mutex;
	struct mutex spi_sync_mutex;
	uint32_t last_flush_req;
	enum cam_lens_driver_state cam_lens_drv_state;
	enum cam_lens_driver_motor_driving_method motor_drv_method;
	struct cam_subdev v4l2_dev_str;
	struct intf_params bridge_intf;
	struct lens_driver_ops *lens_driver_intf;
	struct cam_lens_driver_stm_motor_data stm_motor_info[STM_MOTOR_MAX];
	struct cam_lens_capability lens_capability;
	struct cam_req_mgr_core_workq *cmd_work;
	struct lens_driver_cmd_work_data *cmd_work_data;
};

/**
 * @brief : API to register Lens Driver hw to platform framework.
 * @return struct platform_device pointer on success, or ERR_PTR() on error.
 */
int cam_lens_driver_init(void);

/**
 * @brief : API to remove Lens Driver Hw from platform framework.
 */
void cam_lens_driver_exit(void);

#endif /* _CAM_LENS_DRIVER_DEV_H_ */
