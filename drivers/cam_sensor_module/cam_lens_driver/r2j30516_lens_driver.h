/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _r2j30516_DRIVER_H_
#define _r2j30516_DRIVER_H_

#include "cam_lens_driver_dev.h"

//ver 0.83
#define H8S_CODE_STARTADDR              0x00000200
#define H8S_CODE_ENDADDR                0x0000825F
#define H8S_CODE_SIZE                   0x00008060
#define H8S_CODE_CHECKSUM               0xB5770625

#define DSP_CODE_STARTADDR              0xFFFFE000
#define DSP_CODE_ENDADDR                0xFFFFE487
#define DSP_CODE_SIZE                   0x00000488
#define DSP_CODE_CHECKSUM               0xBF9B8238

#define DSP_DATA_STARTADDR              0xFFFFD000
#define DSP_DATA_ENDADDR                0xFFFFD287
#define DSP_DATA_SIZE                   0x00000288
#define DSP_DATA_CHECKSUM               0x01911A0B

#define STM_MIN_PPS                     20
#define STM_MAX_PPS                     8000
#define DUMMY_BYTE                      0x0
#define DRIVING_DUTY_OFF                0
#define DRIVING_DUTY_WEAK               34
#define DRIVING_DUTY_STRONG             100
#define MOTOR_STEADY_STATE              0x5

#define CH12                            0
#define CH34                            1
#define CH56                            2
#define CH78                            3

#define RE_CMD_SIZE                     12
#define WR_CMD_SIZE                     7
#define NUM_OF_DOWNLOAD_COMMAND         25
#define DOWNLOAD_CMD_SIZE               5
#define H8S_CODE_CHECKSUM_READ_CMD1     12
#define H8S_CODE_CHECKSUM_READ_CMD2     13
#define DSP_CODE_CHECKSUM_READ_CMD1     17
#define DSP_CODE_CHECKSUM_READ_CMD2     18
#define DSP_DATA_CHECKSUM_READ_CMD1     22
#define DSP_DATA_CHECKSUM_READ_CMD2     23

#define NUM_OF_INIT_CMD                 4
#define INIT_CMD_LENGTH                 5
#define CH12_MTD_EXT_ON_REG_VAL         0x0F0A
#define CH34_MTD_EXT_ON_REG_VAL         0x0F0A
#define CH12_MTD_STOP_OPERATION         0x0F02
#define CH34_MTD_STOP_OPERATION         0x0F02

#define CH12_STRONG_EXT_100_PER_AMP     0
#define CH34_STRONG_EXT_100_PER_AMP     0
#define CH12_WEAK_EXT_34_PER_AMP        992
#define CH34_WEAK_EXT_34_PER_AMP        992

#define SET_STM_TPosition_CH1           0x48
#define SET_STM_TPosition_CH2           0x49
#define SET_STM_TPosition_CH3           0x4a
#define SET_STM_TPosition_CH4           0x4b
#define SET_STM_TPPS                    0x4c
#define SET_STM_PC_Operation            0x4d
#define SET_STM_APosition               0x4e
#define SET_STM_DrivingDuty             0x4f

#define GET_STM_APosition               0x87
#define GET_STM_TPosition               0x88
#define GET_STM_TPPS                    0x89
#define GET_STM_PC_Operation            0x8a
#define GET_STM_DrivingDuty             0x8b

#define CH56_78_Set_PreExcitationTime   0x54
#define CH56_78_Set_PostExcitationTime  0x55
#define CH78_direction                  0x70
#define CH78_driving_mode               0x71
#define CH78_set_PPS_0                  0x72
#define CH78_steps                      0x73
#define CH78_PWM_duty                   0x74
#define CH78_excitation_on_off          0x75
#define CH78_run                        0x76
#define CH78_set_PPS_1                  0x77
#define CH78_AccelerationDeceleration   0x2a
#define CH78_pulse_count_rd             0xa4
#define CH78_pulse_count_wr             0x24
#define CH78_motor_running_status       0xcf
#define CH78_PreExcitationTime          0x0a
#define CH78_PostExcitationTime         0x0a

#define CH56_direction                  0x16
#define CH56_driving_mode               0x17
#define CH56_set_PPS_0                  0x18
#define CH56_steps                      0x19
#define CH56_PWM_duty                   0x1a
#define CH56_excitation_on_off          0x1b
#define CH56_run                        0x1c
#define CH56_set_PPS_1                  0x1d
#define CH56_AccelerationDeceleration   0x1f
#define CH56_pulse_count_rd             0xa3
#define CH56_pulse_count_wr             0x23
#define CH56_motor_running_status       0xac
#define CH56_PreExcitationTime          0x0a
#define CH56_PostExcitationTime         0x0a

#define MTD_REG_WRITE                   0x0c
#define MTD_REG_READ                    0x8c
#define CH12_MTD_STM_REG0               0x00
#define CH12_MTD_SET_PULSE1             0x02
#define CH12_MTD_SET_PULSE2             0x04
#define CH12_MTD_SET_PULSERATE0         0x06
#define CH12_MTD_SET_EXCITATION_TIME    0x12
#define CH12_MTD_SET_PWM_DUTY           0x18
#define CH12_EXCITATION_ON              0x1e
#define CH12_MOTOR_MOVING_STATUS        0x1c
#define CH12_MOTOR_12_PHASE_POS         0x14
#define CH12_MOTOR_MICRO_STEP_POS       0x16

#define CH34_MTD_STM_REG0               0x20
#define CH34_MTD_SET_PULSE1             0x22
#define CH34_MTD_SET_PULSE2             0x24
#define CH34_MTD_SET_PULSERATE0         0x26
#define CH34_MTD_SET_EXCITATION_TIME    0x32
#define CH34_MTD_SET_PWM_DUTY           0x38
#define CH34_EXCITATION_ON              0x3e
#define CH34_MOTOR_MOVING_STATUS        0x3c
#define CH34_MOTOR_12_PHASE_POS         0x34
#define CH34_MOTOR_MICRO_STEP_POS       0x36

#define PS_CONTROL_REG                  0xfc
#define CH1_to_6_PS_BIT                 0
#define CH78_PS_BIT                     1

enum r2j30516_fw_code_type {
	H8S_CODE_FW,
	DSP_CODE_FW,
	DSP_DATA_FW,
	INVALID
};

enum STM_excitation_type {
	PRE_EXCITATION,
	POST_EXCITATION
};

int32_t r2j30516_lens_driver_init(struct cam_lens_driver_ctrl_t *l_ctrl);
int32_t r2j30516_lens_driver_PI_calibration(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_PI_calibration *pi_cal_info,
	uint32_t *PI_position);
int32_t r2j30516_lens_driver_set_motor_control(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_set_motor_control *motor_ctrl);
int32_t r2j30516_lens_driver_absolute_motor_drive(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_absolute_motor_move *abs_motor_drv);
int32_t r2j30516_lens_driver_relative_motor_drive(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_relative_motor_move *rel_motor_data);
int32_t r2j30516_lens_driver_get_current_motor_status(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	struct cam_lens_driver_motor_status *motor_status);
int32_t r2j30516_lens_driver_set_motor_actual_position(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	int32_t motor_position);
int32_t r2j30516_lens_driver_abort_motor_moving_ops(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	int32_t *motor_position);
int32_t r2j30516_lens_driver_deinit(struct cam_lens_driver_ctrl_t *l_ctrl);

#endif /* _r2j30516_DRIVER_H_ */
