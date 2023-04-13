/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <cam_sensor_cmn_header.h>
#include "cam_lens_driver_core.h"
#include "cam_sensor_util.h"
#include "cam_trace.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "r2j30516_lens_driver.h"

int32_t cam_lens_driver_spi_byte_txfr(struct spi_device *spi,
	const char *txbuf, char *rxbuf, int32_t num_byte)
{
	struct spi_transfer txfr;
	struct spi_message msg;
	int32_t i, ret;

	for (i = 0; i < num_byte; i++) {
		memset(&txfr, 0, sizeof(txfr));
		txfr.tx_buf = &txbuf[i];
		txfr.rx_buf = &rxbuf[i];
		txfr.len = 1;
		spi_message_init(&msg);
		spi_message_add_tail(&txfr, &msg);
		ret = spi_sync(spi, &msg);
		if (ret < 0)
			break;
	}

	return ret;
}

int32_t cam_lens_driver_spi_txfr(struct spi_device *spi,
	const char *txbuf, char *rxbuf, int32_t num_byte)
{
	struct spi_transfer txfr;
	struct spi_message msg;

	memset(&txfr, 0, sizeof(txfr));
	txfr.tx_buf = txbuf;
	txfr.rx_buf = rxbuf;
	txfr.len = num_byte;
	spi_message_init(&msg);
	spi_message_add_tail(&txfr, &msg);

	return spi_sync(spi, &msg);
}

static int32_t cam_lens_driver_power_down(
	struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int32_t rc = 0;

	if (l_ctrl->lens_driver_intf != NULL &&
		l_ctrl->lens_driver_intf->lens_driver_deinit != NULL) {
		mutex_lock(&(l_ctrl->spi_sync_mutex));
		rc = l_ctrl->lens_driver_intf->lens_driver_deinit(l_ctrl);
		mutex_unlock(&(l_ctrl->spi_sync_mutex));
	}

	return rc;
}

static void cam_req_mgr_process_workq_ldm_message_queue(struct work_struct *w)
{
	cam_req_mgr_process_workq(w);
}

int32_t cam_lens_driver_create_wq(struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int rc;
	int wq_flag = 0;

	wq_flag = CAM_WORKQ_FLAG_SERIAL;
	rc = cam_req_mgr_workq_create("ldm_command_queue", LDM_WORKQ_NUM_TASK,
			&l_ctrl->cmd_work, CRM_WORKQ_USAGE_NON_IRQ, wq_flag,
			cam_req_mgr_process_workq_ldm_message_queue);
	if (rc)
		CAM_ERR(CAM_LENS_DRIVER, "lens driver wq create fail");

	return rc;
}

static int32_t cam_lens_driver_adjust_backlash_step(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_motor_status *pMotor_status)
{
	int rc = 0;
	struct cam_lens_driver_set_motor_actual_position motor_act_pos_info;

	if (pMotor_status->motorId >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
		rc = -EINVAL;
		return rc;
	}

	if (l_ctrl->motor_drv_method == ABSOLUTE_DRV) {
		motor_act_pos_info.motorId = pMotor_status->motorId;
		motor_act_pos_info.motorPosition =
			pMotor_status->motorCurrentPosition +
			l_ctrl->stm_motor_info[pMotor_status->motorId].backLashStepCount;
	} else if (l_ctrl->motor_drv_method == RELATIVE_DRV) {
		motor_act_pos_info.motorId = pMotor_status->motorId;
		motor_act_pos_info.motorPosition =
			pMotor_status->motorCurrentPosition +
			l_ctrl->stm_motor_info[pMotor_status->motorId].backLashStepCount;
	}
	CAM_DBG(CAM_LENS_DRIVER, "set motor[%d] actual position[%d]",
			motor_act_pos_info.motorId,
			motor_act_pos_info.motorPosition);

	if (l_ctrl->lens_driver_intf != NULL &&
		l_ctrl->lens_driver_intf->set_motor_actual_position != NULL) {
		rc = l_ctrl->lens_driver_intf->set_motor_actual_position(
				l_ctrl,
				motor_act_pos_info.motorId,
				motor_act_pos_info.motorPosition);
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER,
				"Fail set motor actual position motorId:%d Act_Pos:%d",
				motor_act_pos_info.motorId,
				motor_act_pos_info.motorPosition);
		} else {
			l_ctrl->stm_motor_info[motor_act_pos_info.motorId].is_moving =
				pMotor_status->isMoving;
			l_ctrl->stm_motor_info[motor_act_pos_info.motorId].actual_position =
				pMotor_status->motorCurrentPosition;
		}
	}
	return rc;
}

static int32_t cam_lens_driver_process_cmd(void *priv, void *data)
{
	uint8_t motor_id;
	int32_t rc = 0;
	int32_t motor_stop_position = 0;
	uint32_t PI_position = 0;
	enum cam_lens_driver_motor_move_direction direction;
	struct cam_lens_driver_ctrl_t *l_ctrl;
	struct lens_driver_cmd_work_data *cmd_data;
	struct cam_lens_driver_absolute_motor_move *abs_motor_move_info = NULL;
	struct cam_lens_driver_relative_motor_move *rel_motor_move_info = NULL;
	struct cam_lens_driver_set_motor_control *motor_ctrl_info = NULL;
	struct cam_lens_driver_PI_calibration *PI_cal_info = NULL;
	struct cam_lens_driver_set_motor_actual_position *motor_act_pos_info = NULL;
	struct cam_lens_driver_abort_motor_moving *abort_motor_move_info = NULL;
	struct cam_lens_driver_motor_status motor_status;

	if (!data || !priv) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid data");
		return -EINVAL;
	}

	l_ctrl = (struct cam_lens_driver_ctrl_t *)priv;
	cmd_data = (struct lens_driver_cmd_work_data *)data;

	if (cmd_data->payload == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid payload data");
		return -EINVAL;
	}

	mutex_lock(&(l_ctrl->spi_sync_mutex));
	switch (cmd_data->cmd_type) {
	case CAM_LENS_DRIVER_ABS_MOTOR_MOVE:{
		int32_t actual_position;

		abs_motor_move_info =
			(struct cam_lens_driver_absolute_motor_move *)cmd_data->payload;
		if (abs_motor_move_info->motorId >= STM_MOTOR_MAX) {
			CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
			rc = -EINVAL;
			goto end;
		}
		motor_id = abs_motor_move_info->motorId;
		actual_position = l_ctrl->stm_motor_info[motor_id].actual_position;
		direction = ((abs_motor_move_info->usteps -
			actual_position) >= 0) ? FORWARD : BACKWARD;
		if (direction == BACKWARD &&
			l_ctrl->stm_motor_info[motor_id].last_direction
				== FORWARD) {
			abs_motor_move_info->usteps -=
				l_ctrl->stm_motor_info[motor_id].backLashStepCount;
			l_ctrl->stm_motor_info[motor_id].is_backlash_step_added = 1;
		}
		CAM_DBG(CAM_LENS_DRIVER, "ABS DRV motorID:%d usteps:%d",
				abs_motor_move_info->motorId,
				abs_motor_move_info->usteps);

		/* Call lens driver absolute motor drive API */
		if (l_ctrl->lens_driver_intf != NULL &&
				l_ctrl->lens_driver_intf->absolute_motor_drive != NULL) {
			rc = l_ctrl->lens_driver_intf->absolute_motor_drive(
					l_ctrl, abs_motor_move_info);
			if (rc != 0) {
				CAM_ERR(CAM_LENS_DRIVER, "ABS DRV fail motorId:%d usteps:%d",
						abs_motor_move_info->motorId,
						abs_motor_move_info->usteps);
			} else {
				l_ctrl->stm_motor_info[motor_id].last_direction = direction;
				l_ctrl->stm_motor_info[motor_id].is_moving = 1;
			}
		}
		break;
	}
	case CAM_LENS_DRIVER_REL_MOTOR_MOVE:
		rel_motor_move_info =
			(struct cam_lens_driver_relative_motor_move *)cmd_data->payload;

		if (rel_motor_move_info->motorId >= STM_MOTOR_MAX) {
			CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
			rc = -EINVAL;
			goto end;
		}

		motor_id = rel_motor_move_info->motorId;
		if (rel_motor_move_info->direction == BACKWARD &&
			l_ctrl->stm_motor_info[motor_id].last_direction
			== FORWARD) {
			rel_motor_move_info->usteps +=
				l_ctrl->stm_motor_info[motor_id].backLashStepCount;
			l_ctrl->stm_motor_info[motor_id].is_backlash_step_added = 1;
		}
		CAM_DBG(CAM_LENS_DRIVER, "REL DRV motorID:%d, usteps:%d, dir:%d",
				rel_motor_move_info->motorId,
				rel_motor_move_info->usteps,
				rel_motor_move_info->direction);

		/* Call lens driver relative motor drive API */
		if (l_ctrl->lens_driver_intf != NULL &&
				l_ctrl->lens_driver_intf->relative_motor_drive != NULL) {
			rc = l_ctrl->lens_driver_intf->relative_motor_drive(
					l_ctrl, rel_motor_move_info);
			if (rc != 0) {
				CAM_ERR(CAM_LENS_DRIVER, "REL DRV fail");
				CAM_ERR(CAM_LENS_DRIVER, "motorID:%d, usteps:%d, dir:%d",
						rel_motor_move_info->motorId,
						rel_motor_move_info->usteps,
						rel_motor_move_info->direction);
			} else {
				l_ctrl->stm_motor_info[motor_id].last_direction =
					rel_motor_move_info->direction;
				l_ctrl->stm_motor_info[motor_id].is_moving = 1;
			}
		}
		break;

	case CAM_LENS_DRIVER_PI_CALIBRATION:
		PI_cal_info =
			(struct cam_lens_driver_PI_calibration *)cmd_data->payload;
		CAM_DBG(CAM_LENS_DRIVER, "PI calibration of motorID : %d",
			PI_cal_info->motorId);
		/* Call lens driver PI calibration API */
		if (l_ctrl->lens_driver_intf != NULL &&
				l_ctrl->lens_driver_intf->PI_calibration != NULL) {
			rc = l_ctrl->lens_driver_intf->PI_calibration(l_ctrl,
					PI_cal_info, &PI_position);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER, "PI calibration Fail motor :%d",
					PI_cal_info->motorId);
			}
		}
		break;

	case CAM_LENS_DRIVER_SET_MOTOR_CONTROL:
		motor_ctrl_info =
			(struct cam_lens_driver_set_motor_control *)cmd_data->payload;

		if (motor_ctrl_info->motorId >= STM_MOTOR_MAX) {
			CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
			rc = -EINVAL;
			goto end;
		}

		motor_id = motor_ctrl_info->motorId;

		CAM_DBG(CAM_LENS_DRIVER, "SET MOTOR CONTROL");
		CAM_DBG(CAM_LENS_DRIVER, "motorID:%d lensPowerLevel:%d",
			motor_ctrl_info->motorId,
			motor_ctrl_info->lensPowerLevel);
		CAM_DBG(CAM_LENS_DRIVER, "speed:%d backlashStep:%d drivingMode:%d",
			motor_ctrl_info->speed,
			motor_ctrl_info->backLashStepCount,
			motor_ctrl_info->driving_mode);

		/* Call lens driver set motor control API */
		if (l_ctrl->lens_driver_intf != NULL &&
				l_ctrl->lens_driver_intf->set_motor_control != NULL) {
			rc = l_ctrl->lens_driver_intf->set_motor_control(l_ctrl,
					motor_ctrl_info);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER, "Failed to update motor ctrl data");
				CAM_ERR(CAM_LENS_DRIVER, "motor:%d lensPowerLevel:%d speed:%d",
					motor_ctrl_info->motorId,
					motor_ctrl_info->lensPowerLevel,
					motor_ctrl_info->speed);
			} else {
				l_ctrl->stm_motor_info[motor_id].speed =
					motor_ctrl_info->speed;
				l_ctrl->stm_motor_info[motor_id].lensPowerLevel =
					motor_ctrl_info->lensPowerLevel;
				l_ctrl->stm_motor_info[motor_id].backLashStepCount =
					motor_ctrl_info->backLashStepCount;
				l_ctrl->stm_motor_info[motor_id].drv_mode =
					motor_ctrl_info->driving_mode;
			}
		}
		break;

	case CAM_LENS_DRIVER_SET_MOTOR_ACTUAL_POSITION:
		motor_act_pos_info =
		(struct cam_lens_driver_set_motor_actual_position *)cmd_data->payload;

		if (motor_act_pos_info->motorId >= STM_MOTOR_MAX) {
			CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
			rc = -EINVAL;
			goto end;
		}
		motor_id = motor_act_pos_info->motorId;
		CAM_DBG(CAM_LENS_DRIVER, "motorID : %d motorPosition : %d",
			motor_act_pos_info->motorId,
			motor_act_pos_info->motorPosition);
		/* Call lens driver set motor actual position API */
		if (l_ctrl->lens_driver_intf != NULL &&
				l_ctrl->lens_driver_intf->set_motor_actual_position != NULL) {
			rc = l_ctrl->lens_driver_intf->set_motor_actual_position(
					l_ctrl,
					motor_act_pos_info->motorId,
					motor_act_pos_info->motorPosition);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER, "Failed to set motor actual position");
				CAM_ERR(CAM_LENS_DRIVER, "MotorId :%d Actual_Position : %d",
						motor_act_pos_info->motorId,
						motor_act_pos_info->motorPosition);
			} else {
				l_ctrl->stm_motor_info[motor_id].actual_position =
					motor_act_pos_info->motorPosition;
			}
		}
		break;

	case CAM_LENS_DRIVER_ABORT_MOTOR_MOVING:
		abort_motor_move_info =
			(struct cam_lens_driver_abort_motor_moving *)cmd_data->payload;

		if (abort_motor_move_info->motorId >= STM_MOTOR_MAX) {
			CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
			rc = -EINVAL;
			goto end;
		}
		motor_id = abort_motor_move_info->motorId;
		CAM_DBG(CAM_LENS_DRIVER, "Abort motorID : %d",
				abort_motor_move_info->motorId);
		/* Call lens driver motor abort API */
		if (l_ctrl->lens_driver_intf != NULL &&
				l_ctrl->lens_driver_intf->abort_motor_moving_ops != NULL) {
			rc = l_ctrl->lens_driver_intf->abort_motor_moving_ops(
					l_ctrl,
					abort_motor_move_info->motorId,
					&motor_stop_position);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER, "Failed to abort motor:%d",
						abort_motor_move_info->motorId);
			} else {
				l_ctrl->stm_motor_info[motor_id].is_moving = 0;
				l_ctrl->stm_motor_info[motor_id].actual_position =
					motor_stop_position;
			}
		}
		break;

	case CAM_LENS_DRIVER_GET_MOTOR_MOVING_STATUS:
		motor_id = *(uint8_t *)cmd_data->payload;

		if (motor_id >= STM_MOTOR_MAX) {
			CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
			rc = -EINVAL;
			goto end;
		}

		/* Call lens driver get motor status API */
		if (l_ctrl->lens_driver_intf != NULL &&
				l_ctrl->lens_driver_intf->get_motor_status != NULL) {
			rc = l_ctrl->lens_driver_intf->get_motor_status(
					l_ctrl, motor_id, &motor_status);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER, "Fail to motor[%d] status", motor_id);
			} else {
				CAM_DBG(CAM_LENS_DRIVER, "Read motor status of motorID :%d",
					motor_status.motorId);
				CAM_DBG(CAM_LENS_DRIVER, "isMoving :%d currentPosition :%d",
					motor_status.isMoving,
					motor_status.motorCurrentPosition);
				if (motor_status.isMoving == 0) {
					if (l_ctrl->stm_motor_info[motor_id].is_backlash_step_added
						== 1) {
						l_ctrl->stm_motor_info[motor_id].is_backlash_step_added
							= 0;
						CAM_DBG(CAM_LENS_DRIVER, "Adjust backlash step.");
						cam_lens_driver_adjust_backlash_step(l_ctrl,
							&motor_status);
					} else {
						l_ctrl->stm_motor_info[motor_id].is_moving =
							motor_status.isMoving;
						l_ctrl->stm_motor_info[motor_id].actual_position =
							motor_status.motorCurrentPosition;
					}
				}
			}
		}

		break;
	default:
		CAM_DBG(CAM_LENS_DRIVER,
			"Invalid motor operation cmd received");
		break;
	}
end:
	mutex_unlock(&(l_ctrl->spi_sync_mutex));
	return rc;
}

static int32_t cam_lens_driver_parse_get_motor_status_cmd(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_packet *csl_packet,
	uint32_t *cmd_buf)
{
	int32_t    rc = 0;
	struct     cam_buf_io_cfg *io_cfg;
	uint8_t    motor_flag;
	uint8_t    count = 0;
	uint8_t    j = 0;
	size_t     remain_len = 0;
	size_t     buf_size = 0;
	uint8_t    *read_buffer = NULL;
	uintptr_t  buf_addr;
	struct cam_lens_driver_get_motor_status *get_motor_cmd = NULL;
	struct cam_lens_driver_motor_status motor_status;

	if (cmd_buf == NULL) {
		rc = -EINVAL;
		goto end;
	}

	get_motor_cmd = (struct cam_lens_driver_get_motor_status *)cmd_buf;
	motor_flag = get_motor_cmd->motorId_flag;
	CAM_DBG(CAM_LENS_DRIVER, "Get motor status flag:%x, Num_cmd:%d",
			get_motor_cmd->motorId_flag, get_motor_cmd->num_of_cmd);
	/* Get IO config Buffer */
	io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

	CAM_DBG(CAM_LENS_DRIVER, "number of IO configs: %d:",
			csl_packet->num_io_configs);

	CAM_DBG(CAM_LENS_DRIVER, "Direction: %d:", io_cfg->direction);
	if (io_cfg->direction == CAM_BUF_OUTPUT) {
		rc = cam_mem_get_cpu_buf(io_cfg->mem_handle[0],
				&buf_addr, &buf_size);
		if (rc) {
			CAM_ERR(CAM_LENS_DRIVER, "Fail in get buffer: %d",
					rc);
			goto end;
		}
		if (buf_size <= io_cfg->offsets[0]) {
			CAM_ERR(CAM_LENS_DRIVER, "Not enough buffer");
			rc = -EINVAL;
			goto end;
		}

		remain_len = buf_size - io_cfg->offsets[0];
		CAM_DBG(CAM_LENS_DRIVER, "buf_addr : %pK, buf_size : %zu\n",
				(void *)buf_addr, buf_size);

		if (remain_len < (sizeof(struct cam_lens_driver_motor_status) *
					get_motor_cmd->num_of_cmd)) {
			CAM_ERR(CAM_LENS_DRIVER,
					"failed to copy, Invalid size");
			rc = -EINVAL;
			goto end;
		}
		read_buffer = (uint8_t *)buf_addr;
		if (!read_buffer) {
			CAM_ERR(CAM_LENS_DRIVER,
					"invalid buffer to copy data");
			rc = -EINVAL;
			goto end;
		}
		read_buffer += io_cfg->offsets[0];

		for (j = 0; j < STM_MOTOR_MAX &&
				count < get_motor_cmd->num_of_cmd; j++) {
			/* Call lens driver Get motor moving status API */
			if (motor_flag & (1 << j)) {
				struct crm_workq_task *task;
				struct lens_driver_cmd_work_data *cmd_data;

				task = cam_req_mgr_workq_get_task(l_ctrl->cmd_work);
				if (!task)
					CAM_ERR(CAM_LENS_DRIVER, "no empty task");
				else {
					if (j < STM_MOTOR_MAX) {
						CAM_DBG(CAM_LENS_DRIVER,
							"New task: read motor status");
						l_ctrl->stm_motor_info[j].motorId = j;
						cmd_data = (struct lens_driver_cmd_work_data *)
									task->payload;
						cmd_data->cmd_type =
							CAM_LENS_DRIVER_GET_MOTOR_MOVING_STATUS;
						cmd_data->payload =
						  (void *)&(l_ctrl->stm_motor_info[j].motorId);
						task->process_cb = cam_lens_driver_process_cmd;
						cam_req_mgr_workq_enqueue_task(task,
							l_ctrl, CRM_TASK_PRIORITY_0);
					}
				}
				count++;
				motor_status.motorId = j;
				motor_status.isMoving =
					l_ctrl->stm_motor_info[j].is_moving;
				motor_status.motorCurrentPosition =
					l_ctrl->stm_motor_info[j].actual_position;
				CAM_DBG(CAM_LENS_DRIVER, "computed motorID :%d ", j);
				CAM_DBG(CAM_LENS_DRIVER, "recv data for motorID :%d",
					motor_status.motorId);
				CAM_DBG(CAM_LENS_DRIVER, "isMoving:%d CurrPos:%d",
					motor_status.isMoving,
					motor_status.motorCurrentPosition);
				CAM_DBG(CAM_LENS_DRIVER, "copy the data, len:%d",
					sizeof(struct cam_lens_driver_motor_status));
				memcpy(read_buffer, &motor_status,
					sizeof(struct cam_lens_driver_motor_status));
				read_buffer +=
					sizeof(struct cam_lens_driver_motor_status);
			}
		}
	} else {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid direction");
		rc = -EINVAL;
	}
end:
	return rc;
}

static int32_t cam_lens_driver_motor_operation_parse(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_packet *csl_packet,
	struct cam_cmd_buf_desc *cmd_desc)
{
	uint8_t    motorId;
	int32_t    rc = 0;
	int32_t    i = 0;
	uint32_t   total_cmd_buf_in_bytes = 0;
	size_t     len_of_buff = 0;
	size_t     remain_len = 0;
	size_t     buf_size = 0;
	uint8_t    *cmd_type = NULL;
	uint8_t    *read_buffer = NULL;
	uint32_t   *cmd_buf = NULL;
	uintptr_t  generic_ptr;
	uintptr_t  buf_addr;
	struct cam_buf_io_cfg *io_cfg;
	struct cam_lens_driver_absolute_motor_move *abs_motor_move_info = NULL;
	struct cam_lens_driver_relative_motor_move *rel_motor_move_info = NULL;
	struct cam_lens_driver_get_motor_power_level *get_motor_power_level_cmd = NULL;
	struct cam_lens_driver_set_motor_control *motor_ctrl_info = NULL;
	struct cam_lens_driver_PI_calibration *PI_cal_info = NULL;
	struct cam_lens_driver_set_motor_actual_position *motor_act_pos_info = NULL;
	struct cam_lens_driver_abort_motor_moving *abort_motor_move_info = NULL;
	struct crm_workq_task *task;
	struct lens_driver_cmd_work_data *cmd_data;

	if (csl_packet == NULL || cmd_desc == NULL)
		return -EINVAL;

	/* Loop through multiple command buffers */
	for (i = 0; i < csl_packet->num_cmd_buf; i++) {
		total_cmd_buf_in_bytes = cmd_desc[i].length;
		if (!total_cmd_buf_in_bytes) {
			CAM_ERR(CAM_LENS_DRIVER, "total_cmd_buf_in_bytes is 0");
			continue;
		}
		rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buff);
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed to get cpu buf");
			goto end;
		}
		cmd_buf = (uint32_t *)generic_ptr;
		if (!cmd_buf) {
			CAM_ERR(CAM_LENS_DRIVER, "invalid cmd buf");
			rc = -EINVAL;
			goto end;
		}
		if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
			CAM_ERR(CAM_LENS_DRIVER, "Invalid length for sensor cmd");
			rc = -EINVAL;
			goto end;
		}

		remain_len = len_of_buff - cmd_desc[i].offset;
		cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
		cmd_type = (uint8_t *)cmd_buf;
		CAM_DBG(CAM_LENS_DRIVER, "cmd_type : %d", *cmd_type);

		switch (*cmd_type) {
		case CAM_LENS_DRIVER_ABS_MOTOR_MOVE:
			abs_motor_move_info =
				(struct cam_lens_driver_absolute_motor_move *)cmd_buf;
			CAM_DBG(CAM_LENS_DRIVER,
				"New task: ABS DRV: motorID=%d position=%d",
				abs_motor_move_info->motorId,
				abs_motor_move_info->usteps);
			motorId = abs_motor_move_info->motorId;
			if (motorId >= STM_MOTOR_MAX) {
				CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
				continue;
			}
			task = cam_req_mgr_workq_get_task(l_ctrl->cmd_work);
			if (!task)
				CAM_ERR(CAM_LENS_DRIVER, "no empty task");
			else {
				cmd_data = (struct lens_driver_cmd_work_data *)task->payload;
				cmd_data->cmd_type = *cmd_type;
				cmd_data->payload = (void *)abs_motor_move_info;
				task->process_cb = cam_lens_driver_process_cmd;
				cam_req_mgr_workq_enqueue_task(task, l_ctrl,
					CRM_TASK_PRIORITY_0);

				l_ctrl->stm_motor_info[motorId].is_moving = 1;
			}
			break;

		case CAM_LENS_DRIVER_REL_MOTOR_MOVE:
			rel_motor_move_info =
				(struct cam_lens_driver_relative_motor_move *)cmd_buf;
			CAM_DBG(CAM_LENS_DRIVER,
				"New task: REL DRV: motorID=%d, usteps=%d, direction=%d",
				rel_motor_move_info->motorId,
				rel_motor_move_info->usteps,
				rel_motor_move_info->direction);
			motorId = rel_motor_move_info->motorId;
			if (motorId >= STM_MOTOR_MAX) {
				CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
				continue;
			}
			task = cam_req_mgr_workq_get_task(l_ctrl->cmd_work);
			if (!task)
				CAM_ERR(CAM_LENS_DRIVER, "no empty task");
			else {
				cmd_data = (struct lens_driver_cmd_work_data *)task->payload;
				cmd_data->cmd_type = *cmd_type;
				cmd_data->payload = (void *)rel_motor_move_info;
				task->process_cb = cam_lens_driver_process_cmd;
				cam_req_mgr_workq_enqueue_task(task, l_ctrl,
					CRM_TASK_PRIORITY_0);

				l_ctrl->stm_motor_info[motorId].is_moving = 1;
			}

			break;

		case CAM_LENS_DRIVER_GET_MOTOR_MOVING_STATUS:{

			rc = cam_lens_driver_parse_get_motor_status_cmd(
				l_ctrl, csl_packet, cmd_buf);
			break;
		}
		case CAM_LENS_DRIVER_GET_POWER_LEVEL:{
			get_motor_power_level_cmd =
				(struct cam_lens_driver_get_motor_power_level *)cmd_buf;
			motorId = get_motor_power_level_cmd->motorId;
			CAM_DBG(CAM_LENS_DRIVER, "Get motor[%d] power level",
					get_motor_power_level_cmd->motorId);
			/* Get IO config Buffer */
			io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
					&csl_packet->payload +
					csl_packet->io_configs_offset);

			CAM_DBG(CAM_LENS_DRIVER, "number of IO configs: %d",
					csl_packet->num_io_configs);

			CAM_DBG(CAM_LENS_DRIVER, "Direction: %d:", io_cfg->direction);
			if (io_cfg->direction == CAM_BUF_OUTPUT) {
				rc = cam_mem_get_cpu_buf(io_cfg->mem_handle[0],
						&buf_addr, &buf_size);
				if (rc) {
					CAM_ERR(CAM_LENS_DRIVER, "Fail in get buffer: %d",
							rc);
					goto end;
				}
				if (buf_size <= io_cfg->offsets[0]) {
					CAM_ERR(CAM_LENS_DRIVER, "Not enough buffer");
					rc = -EINVAL;
					goto end;
				}

				remain_len = buf_size - io_cfg->offsets[0];
				CAM_DBG(CAM_LENS_DRIVER, "buf_addr : %pK, buf_size : %zu\n",
						(void *)buf_addr, buf_size);

				if (remain_len < (sizeof(uint8_t))) {
					CAM_ERR(CAM_LENS_DRIVER,
							"failed to copy, Invalid size");
					rc = -EINVAL;
					goto end;
				}
				read_buffer = (uint8_t *)buf_addr;
				if (!read_buffer) {
					CAM_ERR(CAM_LENS_DRIVER,
							"invalid buffer to copy data");
					rc = -EINVAL;
					goto end;
				}
				read_buffer += io_cfg->offsets[0];
				if (motorId < STM_MOTOR_MAX) {
					memcpy(read_buffer,
						&l_ctrl->stm_motor_info[motorId].lensPowerLevel,
						sizeof(uint8_t));
					CAM_ERR(CAM_LENS_DRIVER,
						"Motor[%d] Power level : %d", motorId,
						l_ctrl->stm_motor_info[motorId].lensPowerLevel);
				}
			} else {
				CAM_ERR(CAM_LENS_DRIVER, "Invalid direction");
				rc = -EINVAL;
			}

			break;
		}
		case CAM_LENS_DRIVER_PI_CALIBRATION:{
			PI_cal_info = (struct cam_lens_driver_PI_calibration *)cmd_buf;
			CAM_DBG(CAM_LENS_DRIVER,
				"New task : Do PI calibration of motorID[%d]",
				PI_cal_info->motorId);

			task = cam_req_mgr_workq_get_task(l_ctrl->cmd_work);
			if (!task)
				CAM_ERR(CAM_LENS_DRIVER, "no empty task");
			else {
				cmd_data = (struct lens_driver_cmd_work_data *)task->payload;
				cmd_data->cmd_type = *cmd_type;
				cmd_data->payload = (void *)PI_cal_info;
				task->process_cb = cam_lens_driver_process_cmd;
				cam_req_mgr_workq_enqueue_task(task, l_ctrl,
					CRM_TASK_PRIORITY_0);
			}

			break;
		}
		case CAM_LENS_DRIVER_SET_MOTOR_CONTROL:
			motor_ctrl_info =
				(struct cam_lens_driver_set_motor_control *)cmd_buf;

			CAM_DBG(CAM_LENS_DRIVER, "New Task : Set motor control para");
			CAM_DBG(CAM_LENS_DRIVER, "motorID:%d lensPowerLevel:%d speed:%d",
					motor_ctrl_info->motorId,
					motor_ctrl_info->lensPowerLevel,
					motor_ctrl_info->speed);

			task = cam_req_mgr_workq_get_task(l_ctrl->cmd_work);
			if (!task)
				CAM_ERR(CAM_LENS_DRIVER, "no empty task");
			else {
				cmd_data = (struct lens_driver_cmd_work_data *)task->payload;
				cmd_data->cmd_type = *cmd_type;
				cmd_data->payload = (void *)motor_ctrl_info;
				task->process_cb = cam_lens_driver_process_cmd;
				cam_req_mgr_workq_enqueue_task(task, l_ctrl,
					CRM_TASK_PRIORITY_0);
			}

			break;

		case CAM_LENS_DRIVER_SET_MOTOR_ACTUAL_POSITION:
			CAM_DBG(CAM_LENS_DRIVER, "Set motor actual position cmd");

			motor_act_pos_info =
				(struct cam_lens_driver_set_motor_actual_position *)cmd_buf;
			if (motor_act_pos_info != NULL) {
				CAM_DBG(CAM_LENS_DRIVER, "motorID : %d motorPosition : %d",
					motor_act_pos_info->motorId,
					motor_act_pos_info->motorPosition);

				task = cam_req_mgr_workq_get_task(l_ctrl->cmd_work);
				if (!task)
					CAM_ERR(CAM_LENS_DRIVER, "no empty task");
				else {
					cmd_data =
						(struct lens_driver_cmd_work_data *)task->payload;
					cmd_data->cmd_type = *cmd_type;
					cmd_data->payload = (void *)motor_act_pos_info;
					task->process_cb = cam_lens_driver_process_cmd;
					cam_req_mgr_workq_enqueue_task(task, l_ctrl,
						CRM_TASK_PRIORITY_0);
				}
			} else {
				CAM_ERR(CAM_LENS_DRIVER, "motor_act_pos_info is NULL");
			}

			break;

		case CAM_LENS_DRIVER_ABORT_MOTOR_MOVING:
			CAM_DBG(CAM_LENS_DRIVER, "Abort motor operation");
			abort_motor_move_info =
				(struct cam_lens_driver_abort_motor_moving *)cmd_buf;
			if (abort_motor_move_info != NULL) {
				CAM_DBG(CAM_LENS_DRIVER, "Abort motorID : %d",
					abort_motor_move_info->motorId);
				task = cam_req_mgr_workq_get_task(l_ctrl->cmd_work);
				if (!task)
					CAM_ERR(CAM_LENS_DRIVER, "no empty task");
				else {
					cmd_data =
						(struct lens_driver_cmd_work_data *)task->payload;
					cmd_data->cmd_type = *cmd_type;
					cmd_data->payload = (void *)abort_motor_move_info;
					task->process_cb = cam_lens_driver_process_cmd;
					cam_req_mgr_workq_enqueue_task(task, l_ctrl,
						CRM_TASK_PRIORITY_0);
				}
			} else {
				CAM_ERR(CAM_LENS_DRIVER, "motor_act_pos_info is NULL");
			}

			break;

		default:
			CAM_DBG(CAM_LENS_DRIVER,
					"Invalid motor operation cmd received");
			break;
		}
	}
end:
	return rc;
}

int32_t cam_lens_driver_config(struct cam_lens_driver_ctrl_t *l_ctrl,
	void *arg)
{
	int32_t    rc = 0;
	int32_t    i = 0;
	size_t     len_of_buff = 0;
	size_t     remain_len = 0;
	uint32_t   *offset = NULL;
	uint32_t   *cmd_buf = NULL;
	uintptr_t  generic_ptr;
	uintptr_t  generic_pkt_ptr;
	struct cam_config_dev_cmd config;
	struct cam_control    *ioctl_ctrl = NULL;
	struct cam_packet     *csl_packet = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;

	if (!l_ctrl || !arg) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Args");
		return -EINVAL;
	}

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&config, u64_to_user_ptr(ioctl_ctrl->handle),
			sizeof(config)))
		return -EFAULT;

	rc = cam_mem_get_cpu_buf(config.packet_handle,
		&generic_pkt_ptr, &len_of_buff);
	if (rc < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Error in converting command Handle %d",
			rc);
		return rc;
	}

	remain_len = len_of_buff;

	if ((sizeof(struct cam_packet) > len_of_buff) ||
		((size_t)config.offset >= len_of_buff -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), len_of_buff);
		rc = -EINVAL;
		goto end;
	}

	remain_len -= (size_t)config.offset;
	csl_packet = (struct cam_packet *)
			(generic_pkt_ptr + (uint32_t)config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
			remain_len)) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid packet params");
		rc = -EINVAL;
		goto end;
	}

	CAM_DBG(CAM_LENS_DRIVER, "Pkt opcode: %d", csl_packet->header.op_code);

	if (csl_packet->header.request_id > l_ctrl->last_flush_req)
		l_ctrl->last_flush_req = 0;

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_LENS_DRIVER_PACKET_OPCODE_INIT:{
		struct cam_lens_driver_init_param *init_param = NULL;

		offset = (uint32_t *)&csl_packet->payload;
		offset += csl_packet->cmd_buf_offset;
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		if (cmd_desc == NULL) {
			CAM_ERR(CAM_LENS_DRIVER, "Invalid cmd descreption");
			rc = -EFAULT;
			goto end;
		}

		rc = cam_mem_get_cpu_buf(cmd_desc->mem_handle,
				(uintptr_t *)&generic_ptr, &len_of_buff);
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed to get cpu buf");
			goto end;
		}

		cmd_buf = (uint32_t *)((uint8_t *)generic_ptr +
				cmd_desc->offset);

		init_param = (struct cam_lens_driver_init_param *)cmd_buf;
		l_ctrl->motor_drv_method = init_param->motorDrivingMethod;
		memcpy(&l_ctrl->lens_capability, &init_param->lensCapability,
				 sizeof(l_ctrl->lens_capability));
		CAM_DBG(CAM_LENS_DRIVER, "lens caps: PIRIS=%d, AF=%d, ZOOM=%d",
			l_ctrl->lens_capability.PIRIS,
			l_ctrl->lens_capability.AF,
			l_ctrl->lens_capability.ZOOM);
		CAM_DBG(CAM_LENS_DRIVER, "DCIRIS=%d, ICR=%d ",
			l_ctrl->lens_capability.DCIRIS,
			l_ctrl->lens_capability.ICR);
		for (i = 0; i < STM_MOTOR_MAX; i++) {
			l_ctrl->stm_motor_info[i].channelNum =
				init_param->motorChannelId[i];
			CAM_DBG(CAM_LENS_DRIVER, "init_param->motorChannelId[%d]: %d",
				i, init_param->motorChannelId[i]);
		}
		if (l_ctrl->lens_driver_intf != NULL &&
				l_ctrl->lens_driver_intf->lens_driver_init != NULL) {
			mutex_lock(&(l_ctrl->spi_sync_mutex));
			rc = l_ctrl->lens_driver_intf->lens_driver_init(l_ctrl);
			if (rc < 0)
				CAM_ERR(CAM_LENS_DRIVER, "Lens Driver Init failed");

			mutex_unlock(&(l_ctrl->spi_sync_mutex));
		}
		CAM_DBG(CAM_LENS_DRIVER, "Lens driving method %d",
			l_ctrl->motor_drv_method);

		break;
	}
	case CAM_LENS_DRIVER_PACKET_OPCODE_MOTOR_OPERATION:
		offset = (uint32_t *)&csl_packet->payload;
		offset += csl_packet->cmd_buf_offset;
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		CAM_DBG(CAM_LENS_DRIVER, "Num of cmd buf : %d",
				csl_packet->num_cmd_buf);
		rc = cam_lens_driver_motor_operation_parse(l_ctrl, csl_packet,
			cmd_desc);
		if (rc < 0)
			CAM_ERR(CAM_LENS_DRIVER, "motor ops command parsing fail");

		break;

	case CAM_PKT_NOP_OPCODE:
		CAM_DBG(CAM_LENS_DRIVER, "CAM_PKT_NOP_OPCODE");
		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Wrong Opcode: %d",
			csl_packet->header.op_code & 0xFFFFFF);

		rc = -EINVAL;
		goto end;
	}

end:
	return rc;
}

void cam_lens_driver_shutdown(struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int rc = 0;

	CAM_DBG(CAM_LENS_DRIVER, "calling lens driver shutdown ");

	if (l_ctrl->cam_lens_drv_state == CAM_LENS_DRIVER_INIT)
		return;

	if (l_ctrl->cam_lens_drv_state >= CAM_LENS_DRIVER_CONFIG) {
		rc = cam_lens_driver_power_down(l_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_LENS_DRIVER, "Lens Driver Power down failed");
		l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_ACQUIRE;
	}

	if (l_ctrl->cam_lens_drv_state >= CAM_LENS_DRIVER_ACQUIRE) {
		rc = cam_destroy_device_hdl(l_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_LENS_DRIVER, "destroying dhdl failed");
		l_ctrl->bridge_intf.device_hdl = -1;
		l_ctrl->bridge_intf.link_hdl = -1;
		l_ctrl->bridge_intf.session_hdl = -1;
	}

	l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_INIT;
}

int32_t cam_lens_driver_cmd(struct cam_lens_driver_ctrl_t *l_ctrl,
	void *arg)
{
	int rc = 0;
	struct cam_control *cmd = (struct cam_control *)arg;

	if (!l_ctrl || !cmd) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Args");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	CAM_DBG(CAM_LENS_DRIVER, "Opcode to Lens Driver: %d", cmd->op_code);

	mutex_lock(&(l_ctrl->lens_driver_mutex));
	switch (cmd->op_code) {
	case CAM_ACQUIRE_DEV: {
		struct cam_sensor_acquire_dev lens_driver_acq_dev;
		struct cam_create_dev_hdl bridge_params;

		if (l_ctrl->bridge_intf.device_hdl != -1) {
			CAM_ERR(CAM_LENS_DRIVER, "Device is already acquired");
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = copy_from_user(&lens_driver_acq_dev,
			u64_to_user_ptr(cmd->handle),
			sizeof(lens_driver_acq_dev));
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed Copying from user\n");
			goto release_mutex;
		}

		bridge_params.session_hdl = lens_driver_acq_dev.session_handle;
		bridge_params.ops = &l_ctrl->bridge_intf.ops;
		bridge_params.v4l2_sub_dev_flag = 0;
		bridge_params.media_entity_flag = 0;
		bridge_params.priv = l_ctrl;
		bridge_params.dev_id = CAM_LENS_DRIVER;
		lens_driver_acq_dev.device_handle =
			cam_create_device_hdl(&bridge_params);
		l_ctrl->bridge_intf.device_hdl = lens_driver_acq_dev.device_handle;
		l_ctrl->bridge_intf.session_hdl =
			lens_driver_acq_dev.session_handle;

		CAM_DBG(CAM_LENS_DRIVER, "Device Handle: %d",
			lens_driver_acq_dev.device_handle);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&lens_driver_acq_dev,
			sizeof(struct cam_sensor_acquire_dev))) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}

		l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_ACQUIRE;
	}
		break;
	case CAM_RELEASE_DEV: {
		if (l_ctrl->cam_lens_drv_state == CAM_LENS_DRIVER_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_LENS_DRIVER,
				"Cant release lens_driver: in start state");
			goto release_mutex;
		}

		if (l_ctrl->cam_lens_drv_state == CAM_LENS_DRIVER_CONFIG) {
			rc = cam_lens_driver_power_down(l_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER,
					"Lens Driver Power down failed");
				goto release_mutex;
			}
		}

		if (l_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_LENS_DRIVER, "link hdl: %d device hdl: %d",
				l_ctrl->bridge_intf.device_hdl,
				l_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}

		if (l_ctrl->bridge_intf.link_hdl != -1) {
			CAM_ERR(CAM_LENS_DRIVER,
				"Device [%d] still active on link 0x%x",
				l_ctrl->cam_lens_drv_state,
				l_ctrl->bridge_intf.link_hdl);
			rc = -EAGAIN;
			goto release_mutex;
		}

		rc = cam_destroy_device_hdl(l_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_LENS_DRIVER, "destroying the device hdl");

		l_ctrl->bridge_intf.device_hdl = -1;
		l_ctrl->bridge_intf.link_hdl = -1;
		l_ctrl->bridge_intf.session_hdl = -1;
		l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_INIT;
		l_ctrl->last_flush_req = 0;
	}
		break;
	case CAM_QUERY_CAP: {
		struct cam_ldm_query_cap lens_driver_cap = {0};

		lens_driver_cap.slot_info = l_ctrl->soc_info.index;
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&lens_driver_cap,
			sizeof(struct cam_ldm_query_cap))) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_ERR(CAM_LENS_DRIVER, "Query Cap success");
	}
		break;
	case CAM_START_DEV: {
		if (l_ctrl->cam_lens_drv_state != CAM_LENS_DRIVER_ACQUIRE) {
			rc = -EINVAL;
			CAM_WARN(CAM_LENS_DRIVER,
			"Not in right state to start : %d",
			l_ctrl->cam_lens_drv_state);
			goto release_mutex;
		}
		l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_START;
		l_ctrl->last_flush_req = 0;
	}
		break;
	case CAM_STOP_DEV: {
		if (l_ctrl->cam_lens_drv_state != CAM_LENS_DRIVER_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_LENS_DRIVER,
			"Not in right state to stop : %d",
			l_ctrl->cam_lens_drv_state);
			goto release_mutex;
		}
		l_ctrl->last_flush_req = 0;
		l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_CONFIG;
	}
		break;
	case CAM_CONFIG_DEV: {
		rc = cam_lens_driver_config(l_ctrl, arg);
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed in lens_driver Parsing");
			goto release_mutex;
		}
	}
		break;
	case CAM_FLUSH_REQ:
		//Add flush logic if required
		break;
	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Opcode %d", cmd->op_code);
	}

release_mutex:
	mutex_unlock(&(l_ctrl->lens_driver_mutex));

	return rc;
}
