/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <cam_sensor_spi.h>
#include "cam_sensor_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "r2j30516_lens_driver.h"
#include "cam_lens_driver_core.h"

#define FW_VERSION 0x366
#define DEBUG   0

const char fw_cmd[NUM_OF_DOWNLOAD_COMMAND][DOWNLOAD_CMD_SIZE] = {
	{0xF5, 0x02, 0x00, 0x00, 0xa0},
	{0xF4, 0x01, 0x00, 0x00, 0xa0},
	{0xF0, 0x01, 0x13, 0x03, 0xa0},
	{0xF2, 0x33, 0x92, 0x00, 0xa0},
	{0xF3, 0x01, 0x00, 0x00, 0xa0},
	{0xF0, 0x01, 0x13, 0x03, 0xa0},
	{0xF1, 0x01, 0x00, 0x00, 0xa0},
	{0xF6, 0x12, 0x00, 0x00, 0xa0},
	{0xE3, 0x00, 0x02, 0x00, 0xa0},
	{0xE4, 0x60, 0x80, 0x00, 0xa0},
	{0xE1, 0x11, 0x21, 0x00, 0xa0},
	{0x66, 0x00, 0x00, 0x00, 0xa0},
	{0x67, 0x00, 0x00, 0x00, 0xa0},
	{0xE3, 0x00, 0xe0, 0xff, 0xa0},
	{0xE4, 0x88, 0x04, 0x00, 0xa0},
	{0xE1, 0x11, 0x22, 0x00, 0xa0},
	{0x66, 0x00, 0x00, 0x00, 0xa0},
	{0x67, 0x00, 0x00, 0x00, 0xa0},
	{0xE3, 0x00, 0xd0, 0xff, 0xa0},
	{0xE4, 0x88, 0x02, 0x00, 0xa0},
	{0xE1, 0x11, 0x23, 0x00, 0xa0},
	{0x66, 0x00, 0x00, 0x00, 0xa0},
	{0x67, 0x00, 0x00, 0x00, 0xa0},
	{0xE5, 0x00, 0x02, 0x00, 0xa0},
	{0xE2, 0x00, 0x00, 0x00, 0xa0}
};

static void cam_lens_driver_spi_setup(struct spi_device *spi,
	uint16_t mode, uint32_t speed)
{
	spi->bits_per_word = 8;
	spi->mode = mode;
	spi->max_speed_hz = speed;
	spi->chip_select = 0;
	spi_setup(spi);

	CAM_DBG(CAM_LENS_DRIVER, "irq[%d] cs[%x] CPHA[%x] CPOL[%x] CS_HIGH[%x]",
		spi->irq, spi->chip_select, (spi->mode & SPI_CPHA) ? 1 : 0,
		(spi->mode & SPI_CPOL) ? 1 : 0,
		(spi->mode & SPI_CS_HIGH) ? 1 : 0);
	CAM_DBG(CAM_LENS_DRIVER, "max_speed[%u]", spi->max_speed_hz);
}

static int32_t r2j30516_lens_driver_power_up(
	struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int rc = 0;
	struct cam_lens_driver_soc_private  *soc_private;

	soc_private =
		(struct cam_lens_driver_soc_private *)l_ctrl->soc_info.soc_private;

	if (soc_private == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "soc private data is NULL");
		rc = -EFAULT;
		goto end;
	}
	rc = gpio_direction_output(
			soc_private->cam_lens_driver_gpio_num_info[LENS_DRIVER_CS], 1);
	if (rc < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "SPI CS GPIO out direction fail.");
		goto end;
	}

	gpio_set_value(
		soc_private->cam_lens_driver_gpio_num_info[LENS_DRIVER_CS], 1);

	rc = gpio_direction_output(
			soc_private->cam_lens_driver_gpio_num_info[LENS_DRIVER_RESET], 1);
	if (rc < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "RESET GPIO out direction fail.");
		goto end;
	}
	msleep(1);

	gpio_set_value(
		soc_private->cam_lens_driver_gpio_num_info[LENS_DRIVER_RESET], 0);

	msleep(3);
end:
	return rc;
}

static int32_t r2j30516_lens_driver_send_command(struct spi_device *spi,
	int32_t cmd_num)
{
	char rx_buf[DOWNLOAD_CMD_SIZE] = {0};
	int32_t ret = 0;

	ret = cam_lens_driver_spi_byte_txfr(spi, &fw_cmd[cmd_num][0], rx_buf,
		DOWNLOAD_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi_sync failed. ret %d", ret);
		return ret;
	}
	if ((rx_buf[0] != 0x20) && (rx_buf[1] != fw_cmd[cmd_num][0])) {
		CAM_ERR(CAM_LENS_DRIVER, "Received response code is incorrect.");
		CAM_ERR(CAM_LENS_DRIVER, "Tx CMD : 0x%x 0x%x 0x%x 0x%x 0x%x",
			fw_cmd[cmd_num][0], fw_cmd[cmd_num][1], fw_cmd[cmd_num][2],
			fw_cmd[cmd_num][3], fw_cmd[cmd_num][4]);
		CAM_ERR(CAM_LENS_DRIVER, "Rx RES : 0x%x 0x%x 0x%x 0x%x 0x%x",
			rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
		ret = -EINVAL;
	}
#if DEBUG
	CAM_DBG(CAM_LENS_DRIVER, "Tx CMD : 0x%x 0x%x 0x%x 0x%x 0x%x",
				fw_cmd[cmd_num][0], fw_cmd[cmd_num][1], fw_cmd[cmd_num][2],
					fw_cmd[cmd_num][3], fw_cmd[cmd_num][4]);
	CAM_DBG(CAM_LENS_DRIVER, "Rx RES : 0x%x 0x%x 0x%x 0x%x 0x%x",
				rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
#endif
	return ret;
}

static int32_t r2j30516_lens_driver_send_user_code(struct spi_device *spi,
	enum r2j30516_fw_code_type fw_type, uint8_t *pFwCode)
{
	const char *pCode = (char *)pFwCode;
	char rx_byte = 0;
	uint32_t code_size = 0;
	int i, ret = 0;

	switch (fw_type) {
	case H8S_CODE_FW:
		code_size = H8S_CODE_SIZE;
		break;
	case DSP_CODE_FW:
		code_size = DSP_CODE_SIZE;
		break;
	case DSP_DATA_FW:
		code_size = DSP_DATA_SIZE;
		break;
	default:
		CAM_ERR(CAM_LENS_DRIVER, "Incalid fw type");
		return -EINVAL;
	}

	for (i = 0; i < code_size; i++)
		ret = cam_lens_driver_spi_byte_txfr(spi, &pCode[i], &rx_byte, 1);

	return ret;
}

static int32_t r2j30516_lens_driver_verify_checksum(struct spi_device *spi,
	int32_t cmd_num, uint32_t checksum)
{
	char rx_buf[DOWNLOAD_CMD_SIZE] = {0};
	int32_t ret = 0;
	int32_t i = cmd_num;

	for (i = cmd_num; i < cmd_num + 2 ; i++) {
		memset(&rx_buf, 0, sizeof(rx_buf));
		ret = cam_lens_driver_spi_byte_txfr(spi, &fw_cmd[i][0], rx_buf,
				DOWNLOAD_CMD_SIZE);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
			return ret;
		}
		if ((rx_buf[0] != 0x20) && (rx_buf[1] != fw_cmd[i][0])) {
			CAM_ERR(CAM_LENS_DRIVER, "Received response code is incorrect.");
			CAM_ERR(CAM_LENS_DRIVER, "Tx CMD : 0x%x 0x%x 0x%x 0x%x 0x%x",
				fw_cmd[i][0], fw_cmd[i][1], fw_cmd[i][2],
				fw_cmd[i][3], fw_cmd[i][4]);
			CAM_ERR(CAM_LENS_DRIVER, "Rx RES : 0x%x 0x%x 0x%x 0x%x 0x%x",
				rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
			ret = -1;
			goto end;
		}
		if (fw_cmd[i][0] == 0x66) {
			if ((rx_buf[2] != (checksum & 0xff)) ||
					(rx_buf[3] != ((checksum >> 8) & 0xff))) {
				CAM_ERR(CAM_LENS_DRIVER, "lower checksum mismatch");
				CAM_ERR(CAM_LENS_DRIVER, "lower 16bit checksum 0x%x 0x%x",
					rx_buf[3], rx_buf[2]);
				ret = -1;
			}
		} else if (fw_cmd[i][0] == 0x67) {
			if ((rx_buf[2] != ((checksum >> 16) & 0xff)) ||
					(rx_buf[3] != ((checksum >> 24) & 0xff))) {
				CAM_ERR(CAM_LENS_DRIVER, "higher checksum mismatch");
				CAM_ERR(CAM_LENS_DRIVER, "higher 16bit checksum 0x%x 0x%x",
						rx_buf[3], rx_buf[2]);
				ret = -1;
			}
		} else {
			CAM_ERR(CAM_LENS_DRIVER, "Invalid cheksum read cmd");
			ret = -1;
		}
#if DEBUG
		CAM_DBG(CAM_LENS_DRIVER, "Tx CMD : 0x%x 0x%x 0x%x 0x%x 0x%x",
				fw_cmd[i][0], fw_cmd[i][1], fw_cmd[i][2],
				fw_cmd[i][3], fw_cmd[i][4]);
		CAM_DBG(CAM_LENS_DRIVER, "Rx RES : 0x%x 0x%x 0x%x 0x%x 0x%x",
				rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
#endif
	}
end:
	return ret;
}

static int32_t r2j30516_lens_driver_read_fw_version(struct spi_device *spi,
	uint32_t *fw_version)
{
	const char fw_read_cmd[RE_CMD_SIZE] = {
	0x86, 0x00, 0x00, 0x00, 0x00, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	char rx_buf[RE_CMD_SIZE] = {0};
	uint8_t packet_checksum = 0;
	uint8_t i = 0;
	int32_t ret = 0;

	ret = cam_lens_driver_spi_byte_txfr(spi, fw_read_cmd, rx_buf, RE_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}

	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "Fw read [%d] = 0x%x ", i, rx_buf[i]);

	if (rx_buf[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buf[6]);
		ret = -1;
		return ret;
	}
	packet_checksum = rx_buf[7] + rx_buf[8] + rx_buf[9] + rx_buf[10];
	if (packet_checksum != rx_buf[11]) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid pkt checksum: calculated:%x recv:%x",
			packet_checksum, rx_buf[11]);
		ret = -1;
		return ret;
	}
	*fw_version = ((uint32_t) rx_buf[7]) << 24 |
		((uint32_t) rx_buf[8]) << 16 |
		((uint32_t) rx_buf[9]) <<  8 |
		((uint32_t) rx_buf[10]);
	return ret;
}

static int32_t r2j30516_lens_driver_set_normal(struct spi_device *spi)
{
	char tx_buff[WR_CMD_SIZE] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00};
	char rx_buff[WR_CMD_SIZE] = {0};
	int32_t ret = 0;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	uint8_t i = 0;

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);

#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
		return ret;
	}
	return ret;
}

static int32_t r2j30516_lens_driver_download_fw(
	struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int cmd_num, i, j, ret, value, copy_data_size;
	uint32_t speed = 1200000;
	uint32_t fw_version = 0;
	struct spi_device *spi = l_ctrl->spi_client;
	const struct firmware *firmware;
	uint8_t *pH8SCode = NULL;
	uint8_t *pDSPCode = NULL;
	uint8_t *pDSPData = NULL;
	const uint8_t *fw_start = NULL;
	char buff[5] = {0};
	bool is_hex_char = false;

	if (spi == NULL) {
		ret = -EFAULT;
		goto fw_download_fail;
	}

	ret = request_firmware(&firmware, "r2j30516_H8S_CODE.hex", &spi->dev);
	if (ret) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to locate fw: %d", ret);
		goto fw_download_fail;
	} else {
		CAM_DBG(CAM_LENS_DRIVER, "firmware data %p size %zu",
			firmware->data, firmware->size);

		pH8SCode = kzalloc(H8S_CODE_SIZE, GFP_KERNEL);
		if (pH8SCode == NULL) {
			CAM_ERR(CAM_LENS_DRIVER, "Can't allocate memory for H8S code");
			release_firmware(firmware);
			goto fw_download_fail;
		}
		fw_start = firmware->data;
		copy_data_size = sizeof(buff);
		j = 0;
		for (i = 0; i < firmware->size; i++) {
			is_hex_char = (fw_start[i] >= '0' && fw_start[i] <= '9') ||
				(fw_start[i] >= 'a' && fw_start[i] <= 'f') ||
				(fw_start[i] >= 'A' && fw_start[i] <= 'F');
			if (is_hex_char) {
				memcpy(buff, fw_start+i, copy_data_size);
				buff[4] = '\0';
				if (kstrtoint(buff, 16, &value) != 0) {
					CAM_ERR(CAM_LENS_DRIVER, "fail to convert [%s] to int",
						buff);
				} else {
					if (j < H8S_CODE_SIZE) {
						pH8SCode[j] = value;
						CAM_DBG(CAM_LENS_DRIVER, "buff %s : H8SCODE [%d]=%x",
							buff, j, pH8SCode[j]);
						j++;
						}
				}
				i = i + copy_data_size - 1;
			}
		}
		release_firmware(firmware);
	}

	ret = request_firmware(&firmware, "r2j30516_DSP_CODE.hex", &spi->dev);
	if (ret) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to locate fw: %d", ret);
		goto fw_download_fail;
	} else {
		CAM_DBG(CAM_LENS_DRIVER, "firmware data %p size %zu",
			firmware->data, firmware->size);

		pDSPCode = kzalloc(DSP_CODE_SIZE, GFP_KERNEL);
		if (pDSPCode == NULL) {
			CAM_ERR(CAM_LENS_DRIVER, "Can't allocate memory for DSP code");
			release_firmware(firmware);
			goto fw_download_fail;
		}
		fw_start = firmware->data;
		copy_data_size = sizeof(buff);
		j = 0;
		for (i = 0; i < firmware->size; i++) {
			is_hex_char = (fw_start[i] >= '0' && fw_start[i] <= '9') ||
				(fw_start[i] >= 'a' && fw_start[i] <= 'f') ||
				(fw_start[i] >= 'A' && fw_start[i] <= 'F');
			if (is_hex_char) {
				memcpy(buff, fw_start+i, copy_data_size);
				buff[4] = '\0';
				if (kstrtoint(buff, 16, &value) != 0) {
					CAM_ERR(CAM_LENS_DRIVER, "fail to convert [%s] to int",
						buff);
				} else {
					if (j < DSP_CODE_SIZE) {
						pDSPCode[j] = value;
						CAM_DBG(CAM_LENS_DRIVER, "buff %s : DSPCODE [%d]=%x",
							buff, j, pDSPCode[j]);
						j++;
					}
				}
				i = i + copy_data_size - 1;
			}
		}
		release_firmware(firmware);
	}

	ret = request_firmware(&firmware, "r2j30516_DSP_DATA.hex", &spi->dev);
	if (ret) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to locate fw: %d", ret);
		goto fw_download_fail;
	} else {
		CAM_DBG(CAM_LENS_DRIVER, "firmware data %p size %zu",
			firmware->data, firmware->size);

		pDSPData = kzalloc(DSP_DATA_SIZE, GFP_KERNEL);
		if (pDSPData == NULL) {
			CAM_ERR(CAM_LENS_DRIVER, "Can't allocate memory for DSP data");
			release_firmware(firmware);
			goto fw_download_fail;
		}
		fw_start = firmware->data;
		copy_data_size = sizeof(buff);
		j = 0;
		for (i = 0; i < firmware->size; i++) {
			is_hex_char = (fw_start[i] >= '0' && fw_start[i] <= '9') ||
				(fw_start[i] >= 'a' && fw_start[i] <= 'f') ||
				(fw_start[i] >= 'A' && fw_start[i] <= 'F');
			if (is_hex_char) {
				memcpy(buff, fw_start+i, copy_data_size);
				buff[4] = '\0';
				if (kstrtoint(buff, 16, &value) != 0) {
					CAM_ERR(CAM_LENS_DRIVER, "fail to convert [%s] to int",
						buff);
				} else {
					if (j < DSP_DATA_SIZE) {
						pDSPData[j] = value;
						CAM_DBG(CAM_LENS_DRIVER, "buff %s : DSPData [%d]=%x",
							buff, j, pDSPData[j]);
						j++;
					}
				}
				i = i + copy_data_size - 1;
			}
		}
		release_firmware(firmware);
	}

	ret =  r2j30516_lens_driver_power_up(l_ctrl);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "r2j30516 power up fail.");
		goto fw_download_fail;
	}

	cam_lens_driver_spi_setup(spi, SPI_MODE_3, speed);
	/* send PLL setting to r2j30516 lens driver. */
	for (cmd_num = 0; cmd_num < 7; cmd_num++) {
		ret = r2j30516_lens_driver_send_command(spi, cmd_num);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "fw flash command %d failed.", cmd_num);
			goto fw_download_fail;
		}
		if ((fw_cmd[cmd_num][0] == 0xF4) || (fw_cmd[cmd_num][0] == 0xF1))
			msleep(1);
	}
	/* Increase SPI communication speed for further firmware download. */
	speed = 12000000;
	/* set SPI master speed 12Mhz */
	cam_lens_driver_spi_setup(spi, SPI_MODE_3, speed);

	/* Download H8S core code */
	for (; cmd_num < 11; cmd_num++) {
		ret = r2j30516_lens_driver_send_command(spi, cmd_num);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "fw flash command %d failed.", cmd_num);
			goto fw_download_fail;
		}
	}

	r2j30516_lens_driver_send_user_code(spi, H8S_CODE_FW, pH8SCode);

	/* verify H8S checksum */
	ret = r2j30516_lens_driver_verify_checksum(spi,
			cmd_num, H8S_CODE_CHECKSUM);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "H8S_CODE checksum mismatch");
		goto fw_download_fail;
	}

	/* Download DSP code */
	for (cmd_num += 2; cmd_num < 16; cmd_num++) {
		ret = r2j30516_lens_driver_send_command(spi, cmd_num);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "fw flash command %d failed.", cmd_num);
			goto fw_download_fail;
		}
	}

	r2j30516_lens_driver_send_user_code(spi, DSP_CODE_FW, pDSPCode);

	/* verify DSP code checksum */
	ret = r2j30516_lens_driver_verify_checksum(spi,
			cmd_num, DSP_CODE_CHECKSUM);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "H8S_CODE checksum mismatch");
		goto fw_download_fail;
	}

	/* Download DSP Data */
	for (cmd_num += 2; cmd_num < 21; cmd_num++) {
		ret = r2j30516_lens_driver_send_command(spi, cmd_num);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "fw flash command %d failed.", cmd_num);
			goto fw_download_fail;
		}
	}

	r2j30516_lens_driver_send_user_code(spi, DSP_DATA_FW, pDSPData);

	/* read DSP Data checksum */
	ret = r2j30516_lens_driver_verify_checksum(spi,
			cmd_num, DSP_DATA_CHECKSUM);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "H8S_CODE checksum mismatch");
		goto fw_download_fail;
	}

	for (cmd_num += 2; cmd_num < 25; cmd_num++) {
		ret = r2j30516_lens_driver_send_command(spi, cmd_num);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "fw flash command %d failed.", cmd_num);
			goto fw_download_fail;
		}
	}

	msleep(100);

	r2j30516_lens_driver_read_fw_version(spi, &fw_version);
	if (fw_version != FW_VERSION) {
		CAM_ERR(CAM_LENS_DRIVER, "Fw_version mismatch", cmd_num);
		ret = -1;
	} else {
		r2j30516_lens_driver_set_normal(spi);
	}

fw_download_fail:
	if (pH8SCode != NULL)
		kfree(pH8SCode);
	if (pDSPCode != NULL)
		kfree(pDSPCode);
	if (pDSPData != NULL)
		kfree(pDSPData);

	return ret;
}

static int32_t r2j30516_lens_driver_STM_PC_operation(struct spi_device *spi,
	uint8_t channel, bool is_enable)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;

	tx_buff[i++] = SET_STM_PC_Operation;
	tx_buff[i++] = channel;
	tx_buff[i++] = 0;
	tx_buff[i++] = 0;
	tx_buff[i++] = is_enable;
	tx_buff[i++] = tx_buff[0] + tx_buff[1] +
				   tx_buff[2] + tx_buff[3] +
				   tx_buff[4];
	tx_buff[i++] = DUMMY_BYTE;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);

#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
		return ret;
	}
	return ret;
}

static int32_t init_relative_drive_params(struct spi_device *spi)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	char init_cmd[NUM_OF_INIT_CMD][INIT_CMD_LENGTH] = {
			{0x0c, 0x00, 0x86, 0x00, 0x01},     // CH12 EXT=During excitation, CH34 EXT=During excitation, EXT1=CH12, EXT2=EXT34
			{0x0c, 0x00, 0x88, 0x00, 0x04},     // MOB1=CH12, MOB2=CH34, PI1=OFF, PI2=OFF
			{0x0c, 0x00, 0x8e, 0x00, 0x49},     // CH12=STM, CH3=STM, CH4=STM
			{0x0c, 0x00, 0xfc, 0x01, 0xf3}      // All Power ON
		};

	uint8_t i, j;
	int32_t ret = 0;

	for (j = 0; j < NUM_OF_INIT_CMD; j++) {
		memset(tx_buff, 0, sizeof(tx_buff));
		for (i = 0; i < INIT_CMD_LENGTH; i++)
			tx_buff[i] = init_cmd[j][i];

		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
					   tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff,
					WR_CMD_SIZE);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d",
				ret);
			return ret;
		}
#if DEBUG
		for (i = 0 ; i < WR_CMD_SIZE; i++)
			CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

		for (i = 0 ; i < WR_CMD_SIZE; i++)
			CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);

#endif
		if (rx_buff[6] != 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Invalid response code:%x for command:%x",
				rx_buff[6], tx_buff[2]);
			ret = -1;
			return ret;
		}
	}
	return ret;
}

int32_t r2j30516_lens_driver_init(struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int32_t ret = 0;
	uint32_t fw_version = 0;
	struct spi_device *spi = l_ctrl->spi_client;
	uint32_t speed = 12000000;

	cam_lens_driver_spi_setup(spi, SPI_MODE_3, speed);

	ret = r2j30516_lens_driver_read_fw_version(spi, &fw_version);
	if (ret < 0)
		CAM_ERR(CAM_LENS_DRIVER, "FW read failed, ret:%d", ret);

	if (fw_version == FW_VERSION) {
		CAM_DBG(CAM_LENS_DRIVER, "No Need to flash firmware again", ret);
	} else {
		ret = r2j30516_lens_driver_download_fw(l_ctrl);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "FW download failed, ret:%d", ret);
			return ret;
		}
	}
	if (l_ctrl->motor_drv_method == ABSOLUTE_DRV) {
		if (l_ctrl->lens_capability.PIRIS == true) {
			ret = r2j30516_lens_driver_STM_PC_operation(spi,
					l_ctrl->stm_motor_info[STM_MOTOR_PIRIS].channelNum,
					true);
			if (ret < 0)
				CAM_ERR(CAM_LENS_DRIVER, "PIRIS ops enable failed.");

		}
		if (l_ctrl->lens_capability.AF == true) {
			ret = r2j30516_lens_driver_STM_PC_operation(spi,
				l_ctrl->stm_motor_info[STM_MOTOR_AF].channelNum,
				true);
			if (ret < 0)
				CAM_ERR(CAM_LENS_DRIVER, "PIRIS ops enable failed.");
		}
		if (l_ctrl->lens_capability.ZOOM == true) {
			ret = r2j30516_lens_driver_STM_PC_operation(spi,
					l_ctrl->stm_motor_info[STM_MOTOR_ZOOM].channelNum,
					true);
			if (ret < 0)
				CAM_ERR(CAM_LENS_DRIVER, "PIRIS ops enable failed.");
		}
		if (l_ctrl->lens_capability.DCIRIS == true) {
			ret = r2j30516_lens_driver_STM_PC_operation(spi,
					l_ctrl->stm_motor_info[STM_MOTOR_DCIRIS].channelNum,
					true);
			if (ret < 0)
				CAM_ERR(CAM_LENS_DRIVER, "PIRIS ops enable failed.");
		}
	} else if (l_ctrl->motor_drv_method == RELATIVE_DRV) {
		ret = init_relative_drive_params(spi);
		if (ret < 0)
			CAM_ERR(CAM_LENS_DRIVER, "init relative params failed.");
	} else {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor driving type.");
	}

	return ret;
}

int32_t r2j30516_lens_driver_PI_calibration(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_PI_calibration *pi_cal_info,
	uint32_t *PI_position)
{
/* TODO: Prepare command for PI calibration */
	int32_t ret = 0;

	if (l_ctrl == NULL || NULL == pi_cal_info) {
		CAM_ERR(CAM_LENS_DRIVER, "PIRIS ops enable failed.");
		return -EINVAL;
	}
		//TODO :add PI cal logic

	return ret;
}

static int32_t absolute_drive_set_motor_speed(struct spi_device *spi,
	uint32_t speed, uint8_t channel)
{
	char	rx_buff[WR_CMD_SIZE] = {0};
	char	tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;

	/* Multiplying speed by 2.
	 * From UMD,Speed unit is pps/22phase. But Absolute method
	 * expecting speed in pps/12phase
	 */
	speed = 2 * speed;

	if (speed > STM_MAX_PPS)
		speed = STM_MAX_PPS;
	if (speed < STM_MIN_PPS)
		speed = STM_MIN_PPS;

	tx_buff[i++] = SET_STM_TPPS;
	tx_buff[i++] = channel;
	tx_buff[i++] = 0;
	tx_buff[i++] = (speed >> 8) & 0xFF;
	tx_buff[i++] = speed & 0xFF;
	tx_buff[i++] = tx_buff[0] + tx_buff[1] +
				   tx_buff[2] + tx_buff[3] +
				   tx_buff[4];
	tx_buff[i] = DUMMY_BYTE;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -EINVAL;
		return ret;
	}
	return ret;
}

static int32_t absolute_drive_set_motor_driving_duty(
	struct spi_device *spi, uint8_t lensPowerLevel, uint8_t channel)
{
	char	rx_buff[WR_CMD_SIZE] = {0};
	char	tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	uint8_t driving_duty = 0;

	switch (lensPowerLevel) {
	case POWER_LEVEL_OFF:
		ret = r2j30516_lens_driver_STM_PC_operation(spi,
				channel, false);
		if (ret < 0)
			CAM_ERR(CAM_LENS_DRIVER, "PIRIS STM PC disable ops failed.");

		goto end;

	case POWER_LEVEL_WEAK:
		driving_duty = DRIVING_DUTY_WEAK;
		break;

	case POWER_LEVEL_STRONG:
		driving_duty = DRIVING_DUTY_STRONG;
		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid power level");
		ret = -1;
		return ret;
	}

	tx_buff[i++] = SET_STM_DrivingDuty;
	tx_buff[i++] = channel;
	tx_buff[i++] = 0;
	tx_buff[i++] = 0;
	tx_buff[i++] = driving_duty;
	tx_buff[i++] = tx_buff[0] + tx_buff[1] +
				   tx_buff[2] + tx_buff[3] +
				   tx_buff[4];
	tx_buff[i] = DUMMY_BYTE;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -EINVAL;
		return ret;
	}

	/* Add >1ms delay for driving pattern update */
	msleep(1);
end:
	return ret;
}

static uint16_t rel_drv_get_motor_driving_duty(uint8_t lensPowerLevel,
	uint8_t channel)
{
	uint16_t driving_duty;

	switch (lensPowerLevel) {
	case POWER_LEVEL_OFF:
		driving_duty = 0;
	case POWER_LEVEL_WEAK:
		if (channel == CH12)
			driving_duty = CH12_WEAK_EXT_34_PER_AMP;
		else if (channel == CH34)
			driving_duty = CH34_WEAK_EXT_34_PER_AMP;
		else
			driving_duty = DRIVING_DUTY_WEAK;
		break;

	case POWER_LEVEL_STRONG:
		if (channel == CH12)
			driving_duty = CH12_STRONG_EXT_100_PER_AMP;
		else if (channel == CH34)
			driving_duty = CH34_STRONG_EXT_100_PER_AMP;
		else
			driving_duty = DRIVING_DUTY_STRONG;
		break;

	default:
		if (channel == CH12)
			driving_duty = CH12_WEAK_EXT_34_PER_AMP;
		else if (channel == CH34)
			driving_duty = CH34_WEAK_EXT_34_PER_AMP;
		else
			driving_duty = DRIVING_DUTY_WEAK;
		break;
	}
	return driving_duty;
}

static int32_t relative_drive_set_pwm_duty(
	struct spi_device *spi,
	uint8_t channel,
	uint8_t lensPowerLevel)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	uint16_t driving_duty;
	int32_t ret = 0;

	if (spi == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	driving_duty = rel_drv_get_motor_driving_duty(lensPowerLevel, channel);

	switch (channel) {
	case CH12:
		if (lensPowerLevel == POWER_LEVEL_OFF) {
			tx_buff[i++] = MTD_REG_WRITE;
			tx_buff[i++] = 0;
			tx_buff[i++] = CH12_EXCITATION_ON;
			tx_buff[i++] = 0;
			tx_buff[i++] = 0;
		} else {
			tx_buff[i++] = MTD_REG_WRITE;
			tx_buff[i++] = 0;
			tx_buff[i++] = CH12_MTD_SET_PWM_DUTY;
			tx_buff[i++] = (driving_duty >> 8) & 0xFF;
			tx_buff[i++] = driving_duty & 0xFF;
		}
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH34:
		if (lensPowerLevel == POWER_LEVEL_OFF) {
			tx_buff[i++] = MTD_REG_WRITE;
			tx_buff[i++] = 0;
			tx_buff[i++] = CH34_EXCITATION_ON;
			tx_buff[i++] = 0;
			tx_buff[i++] = 0;
		} else {
			tx_buff[i++] = MTD_REG_WRITE;
			tx_buff[i++] = 0;
			tx_buff[i++] = CH34_MTD_SET_PWM_DUTY;
			tx_buff[i++] = (driving_duty >> 8) & 0xFF;
			tx_buff[i++] = driving_duty & 0xFF;
		}
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH56:
		if (lensPowerLevel == POWER_LEVEL_OFF)
			tx_buff[i++] = CH56_excitation_on_off;
		else
			tx_buff[i++] = CH56_PWM_duty;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = driving_duty & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH78:
		if (lensPowerLevel == POWER_LEVEL_OFF)
			tx_buff[i++] = CH78_excitation_on_off;
		else
			tx_buff[i++] = CH78_PWM_duty;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = driving_duty & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}

	return ret;
}

/*
 * @Function: step_transfer
 * @Input: step=total step/12phase, driving_mode: 0=22phase, 1=12phase, 2=512 micro-step
 * @Return: void
 * @Description: step transfer from total step to pulse 1 and pulse 2 for R2J30516 MTD register setting
 * @example: step_transfer(200, 0)		// 200step/12phase, 22phase driving
 */
static void step_transfer(uint32_t *pulse, uint32_t totalstep, enum motor_driving_mode drv_mode)
{
	switch (drv_mode) {
	case DRV_MODE_22_PHASE:								//22phase driving mode
		pulse[1] = totalstep/2/1024;
		pulse[0] = (totalstep/2)-pulse[1]*1024;
		break;
	case DRV_MODE_12_PHASE:								//12phase driving mode
		pulse[1] = totalstep/1024;
		pulse[0] = totalstep-pulse[1]*1024;
		break;
	case DRV_MODE_512_MICROSTEP:						//512 micro-step driving mode
		pulse[1] = totalstep/1024;
		pulse[0] = totalstep-pulse[1]*1024;
		break;
	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid driving mode: %x", drv_mode);
		break;
	}
}

/*
 * @Function: pps_transfer
 * @Input: pps (Pulse Per Second)/22phase, driving_mode: 0=22phase, 1=12phase, 2=512 micro-step
 * @Return: void
 * @Description: pps transfer from pps to pulse rate 0 for R2J30516 MTD register setting
 * @example: pps_transfer(200, 0)		// 200pps/12phase, 22phase driving
 */
static void pps_transfer(uint32_t *pulse_rate0, uint32_t pps_value, int drv_mode)
{
	switch (drv_mode) {
	case DRV_MODE_22_PHASE:								//22phase driving mode
		*pulse_rate0 = 10000000/(128*pps_value);		// calculate pulse rate 0
		*pulse_rate0 = *pulse_rate0|0x0c00;				// pulse rate range=25.6us
		break;
	case DRV_MODE_12_PHASE:								//12phase driving mode
		*pulse_rate0 = 10000000/(128*pps_value);		// calculate pulse rate 0
		*pulse_rate0 = *pulse_rate0|0x1000;				// pulse rate range=12.8us
		break;
	case DRV_MODE_512_MICROSTEP:						//512 micro-step driving mode
		/*
		 * 1 step/22phase = 128 step/512ustep
		 * pulse rate=1/(PPS*pulse rate range*128)
		 */
		*pulse_rate0 = 10000000/(pps_value*128*4);		// calculate pulse rate 0
		*pulse_rate0 = *pulse_rate0|0x2800;				// pulse rate range=0.4us
		break;
	default:
		break;
	}
}

static int32_t relative_drive_set_pps(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t channel,
	uint32_t speed)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;

	if (l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH12:
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_MTD_SET_PULSERATE0;
		tx_buff[i++] = (speed >> 8) & 0xFF;
		tx_buff[i++] = speed & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH34:
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_MTD_SET_PULSERATE0;
		tx_buff[i++] = (speed >> 8) & 0xFF;
		tx_buff[i++] = speed & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH56:
		tx_buff[i++] = CH56_set_PPS_0;
		tx_buff[i++] = (speed >> 24) & 0xFF;
		tx_buff[i++] = (speed >> 16) & 0xFF;
		tx_buff[i++] = (speed >> 8) & 0xFF;
		tx_buff[i++] = speed & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH78:
		tx_buff[i++] = CH78_set_PPS_0;
		tx_buff[i++] = (speed >> 24) & 0xFF;
		tx_buff[i++] = (speed >> 16) & 0xFF;
		tx_buff[i++] = (speed >> 8) & 0xFF;
		tx_buff[i++] = speed & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

int32_t r2j30516_lens_driver_set_motor_control(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_set_motor_control *motor_ctrl)
{
	uint8_t channel;
	int32_t ret = 0;
	uint32_t pulse_rate0 = 0;
	struct spi_device *spi = NULL;

	if (motor_ctrl == NULL || l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	if (motor_ctrl->motorId >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
		ret = -EINVAL;
		return ret;
	}

	spi = l_ctrl->spi_client;

	if (l_ctrl->motor_drv_method == ABSOLUTE_DRV) {
		ret = absolute_drive_set_motor_speed(spi,
				motor_ctrl->speed,
				l_ctrl->stm_motor_info[motor_ctrl->motorId].channelNum);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Set Motor Speed failed.");
		} else {
			l_ctrl->stm_motor_info[motor_ctrl->motorId].speed =
				motor_ctrl->speed;
		}

		ret = absolute_drive_set_motor_driving_duty(spi,
				motor_ctrl->lensPowerLevel,
				l_ctrl->stm_motor_info[motor_ctrl->motorId].channelNum);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Set Motor driving duty failed.");
		} else {
			l_ctrl->stm_motor_info[motor_ctrl->motorId].lensPowerLevel =
				motor_ctrl->lensPowerLevel;
		}
	} else if (l_ctrl->motor_drv_method == RELATIVE_DRV) {
		ret = relative_drive_set_pwm_duty(spi,
				l_ctrl->stm_motor_info[motor_ctrl->motorId].channelNum,
				motor_ctrl->lensPowerLevel);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Set Motor driving duty failed.");
		} else {
			l_ctrl->stm_motor_info[motor_ctrl->motorId].lensPowerLevel =
			motor_ctrl->lensPowerLevel;
		}

		channel = l_ctrl->stm_motor_info[motor_ctrl->motorId].channelNum;
		if (channel == CH12 || channel == CH34) {
			pps_transfer(&pulse_rate0, motor_ctrl->speed,
				motor_ctrl->driving_mode);
		} else if (channel == CH56 || channel == CH78) {
			pulse_rate0 = 16 * motor_ctrl->speed; // if speed is in 22phase
		}
		ret = relative_drive_set_pps(l_ctrl,
					l_ctrl->stm_motor_info[motor_ctrl->motorId].channelNum,
					pulse_rate0);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed to set pps. ret:%d", ret);
			goto end;
		} else {
			l_ctrl->stm_motor_info[motor_ctrl->motorId].speed =
				motor_ctrl->speed;
		}
	} else {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor driving type.");
		ret = -EINVAL;
	}
end:
	return ret;
}

int32_t r2j30516_lens_driver_absolute_motor_drive(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_absolute_motor_move *abs_motor_drv)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;
	uint8_t channel;

	if (abs_motor_drv == NULL || l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	if (abs_motor_drv->motorId >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
		ret = -EINVAL;
		return ret;
	}

	channel = l_ctrl->stm_motor_info[abs_motor_drv->motorId].channelNum;

	if (l_ctrl->stm_motor_info[abs_motor_drv->motorId].lensPowerLevel
			== POWER_LEVEL_OFF) {
		ret = r2j30516_lens_driver_STM_PC_operation(spi,
				channel, true);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "STM PC enable ops failed for motor:%d",
				abs_motor_drv->motorId);
			return ret;
		}
	}

	if (l_ctrl->stm_motor_info[abs_motor_drv->motorId].lensPowerLevel !=
			POWER_LEVEL_STRONG) {
		ret = absolute_drive_set_motor_driving_duty(spi,
				POWER_LEVEL_STRONG, channel);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Set strong motor driving duty failed.");
		} else {
			l_ctrl->stm_motor_info[abs_motor_drv->motorId].lensPowerLevel =
				POWER_LEVEL_STRONG;
		}
	}

	if (channel == CH12)
		tx_buff[i++] = SET_STM_TPosition_CH1;
	else if (channel == CH34)
		tx_buff[i++] = SET_STM_TPosition_CH2;
	else if (channel == CH56)
		tx_buff[i++] = SET_STM_TPosition_CH3;
	else if (channel == CH78)
		tx_buff[i++] = SET_STM_TPosition_CH4;

	tx_buff[i++] = (abs_motor_drv->usteps >> 24) & 0xFF;
	tx_buff[i++] = (abs_motor_drv->usteps >> 16) & 0xFF;
	tx_buff[i++] = (abs_motor_drv->usteps >> 8) & 0xFF;
	tx_buff[i++] = (abs_motor_drv->usteps) & 0xFF;
	tx_buff[i++] = tx_buff[0] + tx_buff[1] +
				   tx_buff[2] + tx_buff[3] +
				   tx_buff[4];
	tx_buff[i] = DUMMY_BYTE;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -EINVAL;
		return ret;
	}
	return ret;
}

static int32_t r2j30516_lens_driver_run_channel(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t channel)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;

	if (l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH56:
		tx_buff[i++] = CH56_run;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 1;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH78:
		tx_buff[i++] = CH78_run;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 1;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

static int32_t r2j30516_lens_driver_set_excitation(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t channel,
	bool flag)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	uint16_t mtd_ext_reg_value = 0;
	struct spi_device *spi = l_ctrl->spi_client;

	if (l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH12:
		mtd_ext_reg_value = CH12_MTD_EXT_ON_REG_VAL;
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_EXCITATION_ON;
		tx_buff[i++] = (mtd_ext_reg_value >> 8) & 0xFF;
		tx_buff[i++] = mtd_ext_reg_value & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH34:
		mtd_ext_reg_value = CH34_MTD_EXT_ON_REG_VAL;
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_EXCITATION_ON;
		tx_buff[i++] = (mtd_ext_reg_value >> 8) & 0xFF;
		tx_buff[i++] = mtd_ext_reg_value & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH56:
		tx_buff[i++] = CH56_excitation_on_off;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = flag;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH78:
		tx_buff[i++] = CH78_excitation_on_off;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = flag;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

static int32_t r2j30516_lens_driver_disable_acceleration_deceleration(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t channel)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;

	if (l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH56:
		tx_buff[i++] = CH56_AccelerationDeceleration;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH78:
		tx_buff[i++] = CH78_AccelerationDeceleration;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

static int32_t relative_drive_set_steps(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t channel,
	uint32_t steps)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;

	if (l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH56:
		tx_buff[i++] = CH56_steps;
		tx_buff[i++] = (steps >> 24) & 0xFF;
		tx_buff[i++] = (steps >> 16) & 0xFF;
		tx_buff[i++] = (steps >> 8) & 0xFF;
		tx_buff[i++] = steps & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH78:
		tx_buff[i++] = CH78_steps;
		tx_buff[i++] = (steps >> 24) & 0xFF;
		tx_buff[i++] = (steps >> 16) & 0xFF;
		tx_buff[i++] = (steps >> 8) & 0xFF;
		tx_buff[i++] = steps & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

static int32_t r2j30516_lens_driver_set_driving_mode(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t channel,
	uint8_t driving_mode)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;

	if (l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH56:
		tx_buff[i++] = CH56_driving_mode;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = driving_mode;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH78:
		tx_buff[i++] = CH78_driving_mode;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = driving_mode;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

static int32_t r2j30516_lens_driver_set_direction(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t channel,
	uint8_t direction)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;

	if (l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}
	/* Change direction as per renesas reg method */
	if (direction == FORWARD)
		direction = 0;
	else if (direction == BACKWARD)
		direction = 1;

	switch (channel) {
	case CH56:
		tx_buff[i++] = CH56_direction;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = direction;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH78:
		tx_buff[i++] = CH78_direction;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = direction;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

static int32_t set_MTD_driving_method_reg(
	struct spi_device *spi,
	uint8_t channel,
	uint16_t stm_drv_reg)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;

	if (spi == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH12:
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_MTD_STM_REG0;
		tx_buff[i++] = (stm_drv_reg >> 8) & 0xFF;
		tx_buff[i++] = stm_drv_reg & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH34:
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_MTD_STM_REG0;
		tx_buff[i++] = (stm_drv_reg >> 8) & 0xFF;
		tx_buff[i++] = stm_drv_reg & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

static int32_t set_pulse1_reg(
	struct spi_device *spi,
	uint32_t pulse1,
	uint8_t channel)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;

	if (spi == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH12:
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_MTD_SET_PULSE1;
		tx_buff[i++] = (pulse1 >> 8) & 0xFF;
		tx_buff[i++] = pulse1 & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH34:
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_MTD_SET_PULSE1;
		tx_buff[i++] = (pulse1 >> 8) & 0xFF;
		tx_buff[i++] = pulse1 & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

static int32_t set_pulse2_reg(
	struct spi_device *spi,
	uint32_t pulse2,
	uint8_t channel)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;

	if (spi == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH12:
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_MTD_SET_PULSE2;
		tx_buff[i++] = (pulse2 >> 8) & 0xFF;
		tx_buff[i++] = pulse2 & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH34:
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_MTD_SET_PULSE2;
		tx_buff[i++] = (pulse2 >> 8) & 0xFF;
		tx_buff[i++] = pulse2 & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

static int32_t set_ch12_34_excitation_time(
	struct spi_device *spi,
	uint8_t channel)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	uint16_t excitation_time = 0x0300;  // Pre-excitation=9.83ms, Post-excitation=0ms

	if (spi == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH12:
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_MTD_SET_EXCITATION_TIME;
		tx_buff[i++] = (excitation_time >> 8) & 0xFF;
		tx_buff[i++] = excitation_time & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH34:
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_MTD_SET_EXCITATION_TIME;
		tx_buff[i++] = (excitation_time >> 8) & 0xFF;
		tx_buff[i++] = excitation_time & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

static int32_t set_ch56_78_excitation_time(
	struct spi_device *spi,
	uint8_t channel,
	enum STM_excitation_type excitation_type)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	uint32_t excitation_time;

	if (spi == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH56:
		if (excitation_type == PRE_EXCITATION) {
			excitation_time = CH56_PreExcitationTime;
			tx_buff[i++] = CH56_78_Set_PreExcitationTime;
		} else {
			excitation_time = CH56_PostExcitationTime;
			tx_buff[i++] = CH56_78_Set_PostExcitationTime;
		}
		tx_buff[i++] = 0;
		tx_buff[i++] = (excitation_time >> 16) & 0xFF;
		tx_buff[i++] = (excitation_time >> 8) & 0xFF;
		tx_buff[i++] = excitation_time & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH78:
		if (excitation_type == PRE_EXCITATION) {
			excitation_time = CH78_PreExcitationTime;
			tx_buff[i++] = CH56_78_Set_PreExcitationTime;
		} else {
			excitation_time = CH78_PostExcitationTime;
			tx_buff[i++] = CH56_78_Set_PostExcitationTime;
		}
		tx_buff[i++] = 1;
		tx_buff[i++] = (excitation_time >> 16) & 0xFF;
		tx_buff[i++] = (excitation_time >> 8) & 0xFF;
		tx_buff[i++] = excitation_time & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

static int32_t relative_motor_drive_ch12(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_relative_motor_move *rel_motor_data)
{
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;
	uint32_t pulse[2] = {0};
	uint32_t pulse_rate0 = 0;
	uint16_t ch12_reg0 = 0x600; // Carrier1=390KHz, Carrier2=390K, 512u, Count=up, FW, No Acc&Dec, ST Exitation=A+B+, ST Excitation=OFF
	enum motor_driving_mode driving_mode;

	if (rel_motor_data == NULL || l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	if (rel_motor_data->motorId >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
		ret = -EINVAL;
		return ret;
	}

	if (rel_motor_data->direction == FORWARD)
		ch12_reg0 = ch12_reg0 & 0xffb0;
	else
		ch12_reg0 = ch12_reg0 | 0x0040;

	driving_mode = l_ctrl->stm_motor_info[rel_motor_data->motorId].drv_mode;
	if (driving_mode == DRV_MODE_22_PHASE)
		ch12_reg0 = (ch12_reg0 | 0x0500) & 0xfdff;
	else if (driving_mode == DRV_MODE_12_PHASE)
		ch12_reg0 = (ch12_reg0 | 0x0400) & 0xfcff;
	else if (driving_mode == DRV_MODE_512_MICROSTEP)
		ch12_reg0 = (ch12_reg0 | 0x0600) & 0xfeff;

	ret = set_MTD_driving_method_reg(spi, CH12, ch12_reg0);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Fail to set MTD STM drive reg. ret:%d", ret);
		return ret;
	}
	step_transfer(pulse, rel_motor_data->usteps, driving_mode);
	ret = set_pulse1_reg(spi, pulse[0], CH12);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to set pulse1 reg. ret:%d", ret);
		return ret;
	}
	ret = set_pulse2_reg(spi, pulse[1], CH12);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to set pulse2. ret:%d", ret);
		return ret;
	}
	pps_transfer(&pulse_rate0,
		l_ctrl->stm_motor_info[rel_motor_data->motorId].speed,
		driving_mode);
	ret = relative_drive_set_pps(l_ctrl, CH12, pulse_rate0);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to set pps. ret:%d", ret);
		return ret;
	}
	ret = set_ch12_34_excitation_time(spi, CH12);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to set excitation time. ret:%d", ret);
		return ret;
	}
	if (POWER_LEVEL_STRONG !=
		l_ctrl->stm_motor_info[rel_motor_data->motorId].lensPowerLevel) {
		ret = relative_drive_set_pwm_duty(spi, CH12, POWER_LEVEL_STRONG);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed to set amplitude. ret:%d", ret);
			goto end;
		} else {
			l_ctrl->stm_motor_info[rel_motor_data->motorId].lensPowerLevel =
				POWER_LEVEL_STRONG;
		}
	}
	ret = r2j30516_lens_driver_set_excitation(l_ctrl, CH12, true);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "excitation ON failed. ret:%d", ret);
		return ret;
	}
end:
	return ret;
}

static int32_t relative_motor_drive_ch34(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_relative_motor_move *rel_motor_data)
{
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;
	uint32_t pulse[2] = {0};
	uint32_t pulse_rate0 = 0;
	uint16_t ch34_reg0 = 0x600; // Carrier1=390KHz, Carrier2=390K, 512u, Count=up, FW, No Acc&Dec, ST Exitation=A+B+, ST Excitation=OFF
	enum motor_driving_mode driving_mode;

	if (rel_motor_data == NULL || l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		goto end;
	}

	if (rel_motor_data->motorId >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
		ret = -EINVAL;
		goto end;
	}

	if (rel_motor_data->direction == FORWARD)
		ch34_reg0 = ch34_reg0 & 0xffb0;
	else
		ch34_reg0 = ch34_reg0 | 0x0040;

	driving_mode = l_ctrl->stm_motor_info[rel_motor_data->motorId].drv_mode;
	if (driving_mode == DRV_MODE_22_PHASE)
		ch34_reg0 = (ch34_reg0 | 0x0500) & 0xfdff;
	else if (driving_mode == DRV_MODE_12_PHASE)
		ch34_reg0 = (ch34_reg0 | 0x0400) & 0xfcff;
	else if (driving_mode == DRV_MODE_512_MICROSTEP)
		ch34_reg0 = (ch34_reg0 | 0x0600) & 0xfeff;

	ret = set_MTD_driving_method_reg(spi, CH34, ch34_reg0);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Fail to set MTD STM drive reg. ret:%d", ret);
		goto end;
	}
	step_transfer(pulse, rel_motor_data->usteps, driving_mode);
	ret = set_pulse1_reg(spi, pulse[0], CH34);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to set pulse1 reg. ret:%d", ret);
		goto end;
	}
	ret = set_pulse2_reg(spi, pulse[1], CH34);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to set pulse2. ret:%d", ret);
		goto end;
	}
	pps_transfer(&pulse_rate0,
		l_ctrl->stm_motor_info[rel_motor_data->motorId].speed,
		driving_mode);
	ret = relative_drive_set_pps(l_ctrl, CH34, pulse_rate0);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to set pps. ret:%d", ret);
		goto end;
	}
	ret = set_ch12_34_excitation_time(spi, CH34);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to set excitation time. ret:%d", ret);
		goto end;
	}
	if (POWER_LEVEL_STRONG !=
		l_ctrl->stm_motor_info[rel_motor_data->motorId].lensPowerLevel) {
		ret = relative_drive_set_pwm_duty(spi, CH34, POWER_LEVEL_STRONG);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed to set amplitude. ret:%d", ret);
			goto end;
		} else {
			l_ctrl->stm_motor_info[rel_motor_data->motorId].lensPowerLevel =
				POWER_LEVEL_STRONG;
		}
	}
	ret = r2j30516_lens_driver_set_excitation(l_ctrl, CH34, true);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "excitation ON failed. ret %d", ret);
		goto end;
	}
end:
	return ret;
}

static int32_t relative_motor_drive_ch78(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_relative_motor_move *rel_motor_data)
{
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;
	enum motor_driving_mode driving_mode;

	if (rel_motor_data == NULL || l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		goto end;
	}

	if (rel_motor_data->motorId >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
		ret = -EINVAL;
		goto end;
	}

	driving_mode = l_ctrl->stm_motor_info[rel_motor_data->motorId].drv_mode;

	ret = r2j30516_lens_driver_set_driving_mode(l_ctrl, CH78,
			driving_mode);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "set driving mode failed. ret %d", ret);
		goto end;
	}
	ret = r2j30516_lens_driver_set_direction(l_ctrl, CH78,
			rel_motor_data->direction);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "set direction failed. ret %d", ret);
		goto end;
	}
	/* Speed is multiplied by 16 as speed unit is pps/22phase.
	 * 22phasePPS/22phase = 16*22phasePPS/64uSTEP
	 */
	ret = relative_drive_set_pps(l_ctrl, CH78,
			l_ctrl->stm_motor_info[rel_motor_data->motorId].speed * 16);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "set motor speed failed. ret %d", ret);
		goto end;
	}

	ret = relative_drive_set_steps(l_ctrl, CH78,
			rel_motor_data->usteps);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "set motor steps failed. ret %d", ret);
		goto end;
	}

	ret = set_ch56_78_excitation_time(spi, CH78, PRE_EXCITATION);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "set PreExcitationTime failed. ret %d", ret);
		goto end;
	}

	if (POWER_LEVEL_STRONG !=
		l_ctrl->stm_motor_info[rel_motor_data->motorId].lensPowerLevel) {
		ret = relative_drive_set_pwm_duty(spi, CH78, POWER_LEVEL_STRONG);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "set pwm duty failed. ret %d", ret);
			goto end;
		} else {
			l_ctrl->stm_motor_info[rel_motor_data->motorId].lensPowerLevel =
				POWER_LEVEL_STRONG;
		}
	}
	ret = r2j30516_lens_driver_disable_acceleration_deceleration(l_ctrl,
				 CH78);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "disable accel/decel failed. ret %d", ret);
		goto end;
	}

	ret = r2j30516_lens_driver_set_excitation(l_ctrl, CH78, true);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "excitation ON failed. ret %d", ret);
		goto end;
	}

	ret = r2j30516_lens_driver_run_channel(l_ctrl, CH78);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "run channel-78 failed. ret %d", ret);
		goto end;
	}
end:
	return ret;
}

static int32_t relative_motor_drive_ch56(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_relative_motor_move *rel_motor_data)
{
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;
	enum motor_driving_mode driving_mode;

	if (rel_motor_data == NULL || l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		goto end;
	}

	if (rel_motor_data->motorId >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
		ret = -EINVAL;
		goto end;
	}

	ret = r2j30516_lens_driver_set_direction(l_ctrl, CH56,
			rel_motor_data->direction);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "set direction failed. ret %d", ret);
		goto end;
	}

	driving_mode = l_ctrl->stm_motor_info[rel_motor_data->motorId].drv_mode;
	ret = r2j30516_lens_driver_set_driving_mode(l_ctrl, CH56,
			driving_mode);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "set driving mode failed. ret %d", ret);
		return ret;
	}
	/* Speed is multiplied by 16 as speed unit is pps/22phase.
	 * 22phasePPS/22phase = 16*22phasePPS/64uSTEP
	 */
	ret = relative_drive_set_pps(l_ctrl, CH56,
			l_ctrl->stm_motor_info[rel_motor_data->motorId].speed * 16);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "set motor speed failed. ret %d", ret);
		goto end;
	}

	ret = relative_drive_set_steps(l_ctrl, CH56,
			rel_motor_data->usteps);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "set motor steps failed. ret %d", ret);
		return ret;
	}

	ret = set_ch56_78_excitation_time(spi, CH56, PRE_EXCITATION);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "set PreExcitationTime failed. ret %d", ret);
		goto end;
	}

	if (POWER_LEVEL_STRONG !=
		l_ctrl->stm_motor_info[rel_motor_data->motorId].lensPowerLevel) {
		ret = relative_drive_set_pwm_duty(spi, CH56, POWER_LEVEL_STRONG);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "set pwm duty failed. ret %d", ret);
			goto end;
		} else {
			l_ctrl->stm_motor_info[rel_motor_data->motorId].lensPowerLevel =
				POWER_LEVEL_STRONG;
		}
	}
	ret = r2j30516_lens_driver_disable_acceleration_deceleration(l_ctrl,
				 CH56);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "disable accel/decel failed. ret %d", ret);
		goto end;
	}

	ret = r2j30516_lens_driver_set_excitation(l_ctrl, CH56, true);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "excitation ON failed. ret %d", ret);
		goto end;
	}

	ret = r2j30516_lens_driver_run_channel(l_ctrl, CH56);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "run channel-56 failed. ret %d", ret);
		goto end;
	}
end:
	return ret;
}

int32_t r2j30516_lens_driver_relative_motor_drive(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	struct cam_lens_driver_relative_motor_move *rel_motor_data)
{
	int32_t ret = 0;
	uint8_t channel;

	if (l_ctrl == NULL || rel_motor_data == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	if (rel_motor_data->motorId >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor Id.");
		ret = -EINVAL;
		return ret;
	}

	channel = l_ctrl->stm_motor_info[rel_motor_data->motorId].channelNum;

	switch (channel) {
	case CH12:
			ret = relative_motor_drive_ch12(l_ctrl, rel_motor_data);
		break;
	case CH34:
			ret = relative_motor_drive_ch34(l_ctrl, rel_motor_data);
		break;
	case CH56:
			ret = relative_motor_drive_ch56(l_ctrl, rel_motor_data);
		break;
	case CH78:
			ret = relative_motor_drive_ch78(l_ctrl, rel_motor_data);
		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel num.");
		ret = -1;
	}

	return ret;
}

static int32_t absolute_drive_get_actual_position(
	struct spi_device *spi,
	uint8_t channel,
	int32_t *curr_position)
{
	char	rx_buff[RE_CMD_SIZE] = {0};
	char	tx_buff[RE_CMD_SIZE] = {0};
	uint8_t i = 0;
	uint8_t packet_checksum = 0;
	int32_t ret = 0;

	if (curr_position == NULL || spi == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	tx_buff[i++] = GET_STM_APosition;
	tx_buff[i++] = channel;
	tx_buff[i++] = 0;
	tx_buff[i++] = 0;
	tx_buff[i++] = 0;
	tx_buff[i++] = tx_buff[0] + tx_buff[1] +
				   tx_buff[2] + tx_buff[3] +
				   tx_buff[4];
	tx_buff[i] = DUMMY_BYTE;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, RE_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
		return ret;
	}
	packet_checksum = rx_buff[7] + rx_buff[8] + rx_buff[9] + rx_buff[10];
	if (packet_checksum != rx_buff[11]) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Invalid pkt checksum: calculated : %x recv : %x",
			packet_checksum, rx_buff[11]);
		ret = -1;
		return ret;
	}

	*curr_position = ((uint32_t) rx_buff[7]) << 24 |
					 ((uint32_t) rx_buff[8]) << 16 |
					 ((uint32_t) rx_buff[9]) <<  8 |
					 ((uint32_t) rx_buff[10]);

	return ret;
}

static int32_t get_absolute_motor_drive_running_status(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	struct cam_lens_driver_motor_status *motor_status)
{
	char	rx_buff[RE_CMD_SIZE] = {0};
	char	tx_buff[RE_CMD_SIZE] = {0};
	uint8_t i = 0;
	uint8_t packet_checksum = 0;
	int32_t ret = 0;
	struct  spi_device *spi = l_ctrl->spi_client;
	uint8_t channel;

	if (motor_status == NULL || l_ctrl == NULL || motor_id >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	tx_buff[i++] = GET_STM_PC_Operation;
	channel = l_ctrl->stm_motor_info[motor_id].channelNum;

	tx_buff[i++] = channel;
	tx_buff[i++] = 0;
	tx_buff[i++] = 0;
	tx_buff[i++] = 0;
	tx_buff[i++] = tx_buff[0] + tx_buff[1] +
				   tx_buff[2] + tx_buff[3] +
				   tx_buff[4];
	tx_buff[i] = DUMMY_BYTE;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, RE_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -EINVAL;
		return ret;
	}
	packet_checksum = rx_buff[7] + rx_buff[8] + rx_buff[9] + rx_buff[10];
	if (packet_checksum != rx_buff[11]) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Invalid pkt checksum: calculated : %x recv : %x",
			packet_checksum, rx_buff[11]);
		ret = -EINVAL;
		return ret;
	}

	if (rx_buff[10] == MOTOR_STEADY_STATE) {
		motor_status->motorId = motor_id;
		motor_status->isMoving = false;
		ret = absolute_drive_get_actual_position(spi, channel,
			&motor_status->motorCurrentPosition);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Get abs motor position failed.");
		} else {
			l_ctrl->stm_motor_info[motor_id].actual_position =
				motor_status->motorCurrentPosition;
		}
	} else {
		motor_status->motorId = motor_id;
		motor_status->isMoving = true;
	}
	return ret;
}

static int32_t read_motor_running_status(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t channel,
	bool *is_moving)
{
	char rx_buff[RE_CMD_SIZE] = {0};
	char tx_buff[RE_CMD_SIZE] = {0};
	uint8_t i = 0;
	uint8_t packet_checksum = 0;
	int32_t ret = 0;
	int32_t moving_status = 0;
	struct  spi_device *spi = l_ctrl->spi_client;

	if (is_moving == NULL || l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH12:
		tx_buff[i++] = MTD_REG_READ;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_MOTOR_MOVING_STATUS;
		break;
	case CH34:
		tx_buff[i++] = MTD_REG_READ;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_MOTOR_MOVING_STATUS;
		break;
	case CH56:
		tx_buff[i++] = CH56_motor_running_status;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		break;
	case CH78:
		tx_buff[i++] = CH78_motor_running_status;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel num.");
		ret = -1;
	}

	tx_buff[i++] = 0;
	tx_buff[i++] = 0;
	tx_buff[i++] = tx_buff[0] + tx_buff[1] +
				   tx_buff[2] + tx_buff[3] +
				   tx_buff[4];
	tx_buff[i] = DUMMY_BYTE;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, RE_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -EINVAL;
		return ret;
	}
	packet_checksum = rx_buff[7] + rx_buff[8] + rx_buff[9] + rx_buff[10];
	if (packet_checksum != rx_buff[11]) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Invalid pkt checksum: calculated : %x recv : %x",
			packet_checksum, rx_buff[11]);
		ret = -EINVAL;
		return ret;
	}

	moving_status = ((uint32_t) rx_buff[7]) << 24 |
					((uint32_t) rx_buff[8]) << 16 |
					((uint32_t) rx_buff[9]) <<  8 |
					((uint32_t) rx_buff[10]);

	if (moving_status != 0)
		*is_moving = true;
	else
		*is_moving = false;

	return ret;
}

static int32_t get_micro_pulse_count(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t channel,
	int32_t *pMicroPulseCount)
{
	char	rx_buff[RE_CMD_SIZE] = {0};
	char	tx_buff[RE_CMD_SIZE] = {0};
	uint8_t i = 0;
	uint8_t packet_checksum = 0;
	int32_t ret = 0;
	struct  spi_device *spi = l_ctrl->spi_client;

	if (pMicroPulseCount == NULL || l_ctrl == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH12:
		tx_buff[i++] = MTD_REG_READ;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_MOTOR_MICRO_STEP_POS;
		break;
	case CH34:
		tx_buff[i++] = MTD_REG_READ;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_MOTOR_MICRO_STEP_POS;
		break;
	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel num.");
		ret = -1;
		return ret;
	}

	tx_buff[i++] = 0;
	tx_buff[i++] = 0;
	tx_buff[i++] = tx_buff[0] + tx_buff[1] +
				   tx_buff[2] + tx_buff[3] +
				   tx_buff[4];
	tx_buff[i] = DUMMY_BYTE;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, RE_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
		return ret;
	}
	packet_checksum = rx_buff[7] + rx_buff[8] + rx_buff[9] + rx_buff[10];
	if (packet_checksum != rx_buff[11]) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Invalid pkt checksum: calculated : %x recv : %x",
			packet_checksum, rx_buff[11]);
		ret = -1;
		return ret;
	}

	*pMicroPulseCount = ((uint32_t) rx_buff[7]) << 24 |
						((uint32_t) rx_buff[8]) << 16 |
						((uint32_t) rx_buff[9]) <<  8 |
						((uint32_t) rx_buff[10]);
	return ret;
}

static int32_t get_motor_current_position(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	int32_t *pCurrPosition)
{
	char	rx_buff[RE_CMD_SIZE] = {0};
	char	tx_buff[RE_CMD_SIZE] = {0};
	uint8_t i = 0;
	uint8_t packet_checksum = 0;
	uint8_t channel;
	int32_t ret = 0;
	int32_t micro_pulse_count = 0;
	struct  spi_device *spi = l_ctrl->spi_client;

	if (pCurrPosition == NULL || l_ctrl == NULL || motor_id >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	channel = l_ctrl->stm_motor_info[motor_id].channelNum;
	switch (channel) {
	case CH12:
		tx_buff[i++] = MTD_REG_READ;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_MOTOR_12_PHASE_POS;
		break;
	case CH34:
		tx_buff[i++] = MTD_REG_READ;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_MOTOR_12_PHASE_POS;
		break;
	case CH56:
		tx_buff[i++] = CH56_pulse_count_rd;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		break;
	case CH78:
		tx_buff[i++] = CH78_pulse_count_rd;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel num.");
		ret = -1;
	}

	tx_buff[i++] = 0;
	tx_buff[i++] = 0;
	tx_buff[i++] = tx_buff[0] + tx_buff[1] +
				   tx_buff[2] + tx_buff[3] +
				   tx_buff[4];
	tx_buff[i] = DUMMY_BYTE;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, RE_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -EINVAL;
		return ret;
	}
	packet_checksum = rx_buff[7] + rx_buff[8] + rx_buff[9] + rx_buff[10];
	if (packet_checksum != rx_buff[11]) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Invalid pkt checksum: calculated : %x recv : %x",
			packet_checksum, rx_buff[11]);
		ret = -EINVAL;
		return ret;
	}

	*pCurrPosition = ((uint32_t) rx_buff[7]) << 24 |
					 ((uint32_t) rx_buff[8]) << 16 |
					 ((uint32_t) rx_buff[9]) <<  8 |
					 ((uint32_t) rx_buff[10]);

	if (channel == CH12 || channel == CH34) {
		// Get Micro step pulse count
		ret = get_micro_pulse_count(l_ctrl, channel, &micro_pulse_count);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Fail to get micro pulse count. ret:%d",
				ret);
			return ret;
		}

		if (*pCurrPosition > 0x3FFF)
			*pCurrPosition = (*pCurrPosition & 0x7FFF) - 32768;
		else
			*pCurrPosition = *pCurrPosition & 0x7FFF;

		switch (l_ctrl->stm_motor_info[motor_id].drv_mode) {
		case DRV_MODE_22_PHASE:
			*pCurrPosition = (*pCurrPosition) << 1;
			break;
		case DRV_MODE_12_PHASE:
			*pCurrPosition = *pCurrPosition;
			break;
		case DRV_MODE_512_MICROSTEP:
			*pCurrPosition = (*pCurrPosition) << 6;
			break;
		default:
			CAM_DBG(CAM_LENS_DRIVER, "Default read 1-2 phase pulse count.");
		}
		*pCurrPosition = *pCurrPosition + ((micro_pulse_count & 0x007F) >> 1);
	}

	return ret;
}

static int32_t get_relative_motor_drive_running_status(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	struct cam_lens_driver_motor_status *motor_status)
{
	int32_t ret = 0;
	uint8_t channel;
	bool is_moving;

	if (motor_status == NULL || l_ctrl == NULL || motor_id >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	channel = l_ctrl->stm_motor_info[motor_id].channelNum;
	ret = read_motor_running_status(l_ctrl, channel, &is_moving);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Fail to get motor moving status. ret:%d",
			ret);
		return ret;
	}

	if (is_moving == true) {
		motor_status->motorId = motor_id;
		motor_status->isMoving = true;
		motor_status->motorCurrentPosition = 0;
	} else {
		motor_status->motorId = motor_id;
		motor_status->isMoving = false;
		ret = get_motor_current_position(l_ctrl, motor_id,
				&motor_status->motorCurrentPosition);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Fail to get motor position. ret:%d",
				ret);
		}
	}

	return ret;
}

int32_t r2j30516_lens_driver_get_current_motor_status(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	struct cam_lens_driver_motor_status *motor_status)
{
	int32_t ret = 0;

	if (motor_status == NULL || l_ctrl == NULL || motor_id >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	if (l_ctrl->motor_drv_method == ABSOLUTE_DRV) {
		ret = get_absolute_motor_drive_running_status(l_ctrl,
			motor_id, motor_status);
	} else if (l_ctrl->motor_drv_method == RELATIVE_DRV) {
		ret = get_relative_motor_drive_running_status(l_ctrl,
			motor_id, motor_status);
	} else {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor driving type.");
		ret = -EINVAL;
	}

	return ret;
}

static int32_t disable_operation_bit(
	struct spi_device *spi,
	uint8_t channel)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	uint16_t mtd_ext_reg_value = 0;

	if (spi == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH12:
		mtd_ext_reg_value = CH12_MTD_STOP_OPERATION;
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_EXCITATION_ON;
		tx_buff[i++] = (mtd_ext_reg_value >> 8) & 0xFF;
		tx_buff[i++] = mtd_ext_reg_value & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH34:
		mtd_ext_reg_value = CH34_MTD_STOP_OPERATION;
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_EXCITATION_ON;
		tx_buff[i++] = (mtd_ext_reg_value >> 8) & 0xFF;
		tx_buff[i++] = mtd_ext_reg_value & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);

#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}

	return ret;
}

static int32_t set_relative_motor_actual_position(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	int32_t motor_position)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;
	uint8_t channel;

	if (l_ctrl == NULL || motor_id >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	channel = l_ctrl->stm_motor_info[motor_id].channelNum;
	if (channel == CH12 || channel == CH34) {
		disable_operation_bit(spi, channel);
		do_div(motor_position, 64);
	}

	switch (channel) {
	case CH12:
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_MOTOR_12_PHASE_POS;
		tx_buff[i++] = (motor_position >> 8) & 0xFF;
		tx_buff[i++] = (motor_position) & 0xFF;
		break;

	case CH34:
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_MOTOR_12_PHASE_POS;
		tx_buff[i++] = (motor_position >> 8) & 0xFF;
		tx_buff[i++] = (motor_position) & 0xFF;
		break;

	case CH56:
		tx_buff[i++] = CH56_pulse_count_wr;
		tx_buff[i++] = (motor_position >> 24) & 0xFF;
		tx_buff[i++] = (motor_position >> 16) & 0xFF;
		tx_buff[i++] = (motor_position >> 8) & 0xFF;
		tx_buff[i++] = (motor_position) & 0xFF;
		break;

	case CH78:
		tx_buff[i++] = CH78_pulse_count_wr;
		tx_buff[i++] = (motor_position >> 24) & 0xFF;
		tx_buff[i++] = (motor_position >> 16) & 0xFF;
		tx_buff[i++] = (motor_position >> 8) & 0xFF;
		tx_buff[i++] = (motor_position) & 0xFF;
		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	tx_buff[i++] = tx_buff[0] + tx_buff[1] +
		tx_buff[2] + tx_buff[3] + tx_buff[4];
	tx_buff[i] = DUMMY_BYTE;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);

#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

static int32_t set_absolute_motor_actual_position(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	int32_t motor_position)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;
	uint8_t channel;

	if (l_ctrl == NULL || motor_id >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	channel = l_ctrl->stm_motor_info[motor_id].channelNum;
	tx_buff[i++] = SET_STM_APosition;
	tx_buff[i++] = channel;
	tx_buff[i++] = (motor_position >> 16) & 0xFF;
	tx_buff[i++] = (motor_position >> 8) & 0xFF;
	tx_buff[i++] = (motor_position) & 0xFF;
	tx_buff[i++] = tx_buff[0] + tx_buff[1] +
		tx_buff[2] + tx_buff[3] + tx_buff[4];
	tx_buff[i] = DUMMY_BYTE;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);

#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	return ret;
}

int32_t r2j30516_lens_driver_set_motor_actual_position(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	int32_t motor_position)
{
	int32_t ret = 0;

	if (l_ctrl == NULL || motor_id >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}
	if (l_ctrl->motor_drv_method == ABSOLUTE_DRV) {
		ret = set_absolute_motor_actual_position(l_ctrl,
			motor_id, motor_position);
	} else if (l_ctrl->motor_drv_method == RELATIVE_DRV) {
		ret = set_relative_motor_actual_position(l_ctrl,
			motor_id, motor_position);
	} else {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor driving type.");
		ret = -EINVAL;
	}
	return ret;
}

static int32_t read_operation_control_register(
	struct  spi_device *spi, uint8_t channel, int16_t *reg_value)
{
	char	rx_buff[RE_CMD_SIZE] = {0};
	char	tx_buff[RE_CMD_SIZE] = {0};
	uint8_t i = 0;
	uint8_t packet_checksum = 0;
	int32_t ret = 0;

	if (NULL == reg_value || NULL == spi) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	switch (channel) {
	case CH12:
		tx_buff[i++] = MTD_REG_READ;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_EXCITATION_ON;
		break;
	case CH34:
		tx_buff[i++] = MTD_REG_READ;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_EXCITATION_ON;
		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel num.");
		ret = -1;
		return ret;
	}

	tx_buff[i++] = 0;
	tx_buff[i++] = 0;
	tx_buff[i++] = tx_buff[0] + tx_buff[1] +
				   tx_buff[2] + tx_buff[3] +
				   tx_buff[4];
	tx_buff[i] = DUMMY_BYTE;

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, RE_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < RE_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);

#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -EINVAL;
		return ret;
	}
	packet_checksum = rx_buff[7] + rx_buff[8] + rx_buff[9] + rx_buff[10];
	if (packet_checksum != rx_buff[11]) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Invalid pkt checksum: calculated : %x recv : %x",
			packet_checksum, rx_buff[11]);
		ret = -EINVAL;
		return ret;
	}

	*reg_value = ((uint32_t) rx_buff[7]) << 24 |
				((uint32_t) rx_buff[8]) << 16 |
				((uint32_t) rx_buff[9]) <<  8 |
				((uint32_t) rx_buff[10]);

	return ret;
}

static int32_t abort_relative_motor_move(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	int32_t *motor_position)
{
	char rx_buff[WR_CMD_SIZE] = {0};
	char tx_buff[WR_CMD_SIZE] = {0};
	uint8_t i = 0;
	uint8_t channel;
	int32_t ret = 0;
	int32_t position = 0;
	uint16_t reg_ops_ctrl_data = 0;
	struct spi_device *spi = l_ctrl->spi_client;

	if (l_ctrl == NULL || motor_id >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}
	channel = l_ctrl->stm_motor_info[motor_id].channelNum;
	switch (channel) {
	case CH12:
		ret = read_operation_control_register(spi,
				channel, &reg_ops_ctrl_data);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Fail to read ops ctrl reg.");
			reg_ops_ctrl_data = CH12_MTD_STOP_OPERATION;
		} else {
			reg_ops_ctrl_data &= ~(1 << 3);
			CAM_DBG(CAM_LENS_DRIVER, "ops ctrl reg[%x] = %x",
				CH12_EXCITATION_ON, reg_ops_ctrl_data);
		}
		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH12_EXCITATION_ON;
		tx_buff[i++] = (reg_ops_ctrl_data >> 8) & 0xFF;
		tx_buff[i++] = reg_ops_ctrl_data & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH34:
		ret = read_operation_control_register(spi,
				channel, &reg_ops_ctrl_data);
		if (ret < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Fail to read ops ctrl reg.");
		reg_ops_ctrl_data = CH34_MTD_STOP_OPERATION;
		} else {
			reg_ops_ctrl_data &= ~(1 << 3);
			CAM_DBG(CAM_LENS_DRIVER, "ops ctrl reg[%x] = %x",
				CH34_EXCITATION_ON, reg_ops_ctrl_data);
		}

		tx_buff[i++] = MTD_REG_WRITE;
		tx_buff[i++] = 0;
		tx_buff[i++] = CH34_EXCITATION_ON;
		tx_buff[i++] = (reg_ops_ctrl_data >> 8) & 0xFF;
		tx_buff[i++] = reg_ops_ctrl_data & 0xFF;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH56:
		tx_buff[i++] = CH56_run;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	case CH78:
		tx_buff[i++] = CH78_run;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = 0;
		tx_buff[i++] = tx_buff[0] + tx_buff[1] +
			tx_buff[2] + tx_buff[3] + tx_buff[4];
		tx_buff[i] = DUMMY_BYTE;

		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid channel type");
		break;
	}

	ret = cam_lens_driver_spi_byte_txfr(spi, tx_buff, rx_buff, WR_CMD_SIZE);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi byte txfr failed. ret %d", ret);
		return ret;
	}
#if DEBUG
	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "TX[%d] = 0x%x ", i, tx_buff[i]);

	for (i = 0 ; i < WR_CMD_SIZE; i++)
		CAM_DBG(CAM_LENS_DRIVER, "RX[%d] = 0x%x ", i, rx_buff[i]);
#endif
	if (rx_buff[6] != 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid response code: %x", rx_buff[6]);
		ret = -1;
	}
	ret = get_motor_current_position(l_ctrl, motor_id, &position);
	if (ret < 0)
		CAM_ERR(CAM_LENS_DRIVER, "Fail to get motor position. ret:%d", ret);
	else
		*motor_position = position;

	return ret;
}

static int32_t abort_absolute_motor_move(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	int32_t *motor_position)
{
	int32_t ret = 0;
	int32_t position = 0;
	struct spi_device *spi = l_ctrl->spi_client;
	uint8_t channel;
	struct cam_lens_driver_absolute_motor_move abs_motor_drv;

	if (l_ctrl == NULL || motor_id >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}
	channel = l_ctrl->stm_motor_info[motor_id].channelNum;
	ret = absolute_drive_get_actual_position(spi, channel,
			&position);
	if (ret < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "ABORT: Get abs motor position failed.");
	} else {
		*motor_position = position;
		abs_motor_drv.motorId = motor_id;
		abs_motor_drv.usteps = position;
		ret = r2j30516_lens_driver_absolute_motor_drive(l_ctrl,
				&abs_motor_drv);
		if (ret < 0)
			CAM_ERR(CAM_LENS_DRIVER, "ABORT: abs motor drive fail.");
	}

	return ret;
}

int32_t r2j30516_lens_driver_abort_motor_moving_ops(
	struct cam_lens_driver_ctrl_t *l_ctrl,
	uint8_t motor_id,
	int32_t *motor_position)
{
	int32_t ret = 0;

	if (l_ctrl == NULL || NULL == motor_position || motor_id >= STM_MOTOR_MAX) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid input received.");
		ret = -EINVAL;
		return ret;
	}

	if (l_ctrl->motor_drv_method == ABSOLUTE_DRV) {
		ret = abort_absolute_motor_move(l_ctrl,
			motor_id, motor_position);
	} else if (l_ctrl->motor_drv_method == RELATIVE_DRV) {
		ret = abort_relative_motor_move(l_ctrl,
			motor_id, motor_position);
	} else {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid motor driving type.");
		ret = -EINVAL;
	}
	return ret;
}

static int32_t turn_off_motor_power_level(struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int32_t ret = 0;
	struct spi_device *spi = l_ctrl->spi_client;

	if (l_ctrl->lens_capability.PIRIS == true) {
		if (l_ctrl->motor_drv_method == ABSOLUTE_DRV) {
			ret = absolute_drive_set_motor_driving_duty(spi,
					POWER_LEVEL_OFF,
					l_ctrl->stm_motor_info[STM_MOTOR_PIRIS].channelNum);
			if (ret < 0) {
				CAM_ERR(CAM_LENS_DRIVER, "Set Motor driving duty failed.");
			} else {
				l_ctrl->stm_motor_info[STM_MOTOR_PIRIS].lensPowerLevel =
					POWER_LEVEL_OFF;
			}
		} else {
			ret = relative_drive_set_pwm_duty(spi,
					l_ctrl->stm_motor_info[STM_MOTOR_PIRIS].channelNum,
					POWER_LEVEL_OFF);
			if (ret < 0) {
				CAM_ERR(CAM_LENS_DRIVER, "Set Motor driving duty failed.");
			} else {
				l_ctrl->stm_motor_info[STM_MOTOR_PIRIS].lensPowerLevel =
					POWER_LEVEL_OFF;
			}
		}
	}
	if (l_ctrl->lens_capability.AF == true) {
		if (l_ctrl->motor_drv_method == ABSOLUTE_DRV) {
			ret = absolute_drive_set_motor_driving_duty(spi,
					POWER_LEVEL_OFF,
					l_ctrl->stm_motor_info[STM_MOTOR_AF].channelNum);
			if (ret < 0) {
				CAM_ERR(CAM_LENS_DRIVER, "Set Motor driving duty failed.");
			} else {
				l_ctrl->stm_motor_info[STM_MOTOR_AF].lensPowerLevel =
					POWER_LEVEL_OFF;
			}
		} else {
			ret = relative_drive_set_pwm_duty(spi,
					l_ctrl->stm_motor_info[STM_MOTOR_AF].channelNum,
					POWER_LEVEL_OFF);
			if (ret < 0) {
				CAM_ERR(CAM_LENS_DRIVER, "Set Motor driving duty failed.");
			} else {
				l_ctrl->stm_motor_info[STM_MOTOR_AF].lensPowerLevel =
					POWER_LEVEL_OFF;
			}
		}
	}
	if (l_ctrl->lens_capability.ZOOM == true) {
		if (l_ctrl->motor_drv_method == ABSOLUTE_DRV) {
			ret = absolute_drive_set_motor_driving_duty(spi,
					POWER_LEVEL_OFF,
					l_ctrl->stm_motor_info[STM_MOTOR_ZOOM].channelNum);
			if (ret < 0) {
				CAM_ERR(CAM_LENS_DRIVER, "Set Motor driving duty failed.");
			} else {
				l_ctrl->stm_motor_info[STM_MOTOR_ZOOM].lensPowerLevel =
					POWER_LEVEL_OFF;
			}
		} else {
			ret = relative_drive_set_pwm_duty(spi,
					l_ctrl->stm_motor_info[STM_MOTOR_ZOOM].channelNum,
					POWER_LEVEL_OFF);
			if (ret < 0) {
				CAM_ERR(CAM_LENS_DRIVER, "Set Motor driving duty failed.");
			} else {
				l_ctrl->stm_motor_info[STM_MOTOR_ZOOM].lensPowerLevel =
					POWER_LEVEL_OFF;
			}
		}
	}
	/* TODO : add logic for DCIRIS
	 *if (l_ctrl->lens_capability.DCIRIS == true)
	 */

	return ret;
}

int32_t r2j30516_lens_driver_deinit(struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int32_t ret = 0;

	ret = turn_off_motor_power_level(l_ctrl);
	if (ret < 0)
		CAM_ERR(CAM_LENS_DRIVER, "set motor weak power level failed.");

	return ret;
}
