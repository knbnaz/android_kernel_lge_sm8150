/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/spi/spi.h>
#include "cam_lens_driver_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_lens_driver_soc.h"
#include "cam_lens_driver_core.h"
#include "cam_trace.h"
#include "r2j30516_lens_driver.h"


static int cam_lens_driver_get_function_table(
	struct cam_lens_driver_ctrl_t *l_ctrl);

static struct cam_lens_driver_info r2j30516_lens_driver = {
	.cam_lens_driver_name = "R2J30516",
	.lens_driver_intf = {
		.lens_driver_init = r2j30516_lens_driver_init,
		.PI_calibration = r2j30516_lens_driver_PI_calibration,
		.set_motor_control = r2j30516_lens_driver_set_motor_control,
		.absolute_motor_drive = r2j30516_lens_driver_absolute_motor_drive,
		.relative_motor_drive = r2j30516_lens_driver_relative_motor_drive,
		.get_motor_status = r2j30516_lens_driver_get_current_motor_status,
		.set_motor_actual_position =
			r2j30516_lens_driver_set_motor_actual_position,
		.abort_motor_moving_ops = r2j30516_lens_driver_abort_motor_moving_ops,
		.lens_driver_deinit = r2j30516_lens_driver_deinit
	}
};

static struct cam_lens_driver_info *lens_driver_table[] = {
	&r2j30516_lens_driver,
};

static int cam_lens_driver_subdev_close_internal(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_lens_driver_ctrl_t *l_ctrl =
		v4l2_get_subdevdata(sd);

	if (!l_ctrl) {
		CAM_ERR(CAM_LENS_DRIVER, "l_ctrl ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&(l_ctrl->lens_driver_mutex));
	cam_lens_driver_shutdown(l_ctrl);
	mutex_unlock(&(l_ctrl->lens_driver_mutex));

	return 0;
}

static int cam_lens_driver_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	bool crm_active = cam_req_mgr_is_open(CAM_LENS_DRIVER);

	if (crm_active) {
		CAM_DBG(CAM_LENS_DRIVER, "CRM is ACTIVE, close should be from CRM");
		return 0;
	}
	return cam_lens_driver_subdev_close_internal(sd, fh);
}

static long cam_lens_driver_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int rc = 0;
	struct cam_lens_driver_ctrl_t *l_ctrl =
		v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_lens_driver_cmd(l_ctrl, arg);
		if (rc)
			CAM_ERR(CAM_LENS_DRIVER, "Failed for driver_cmd: %d", rc);

		break;
	case CAM_SD_SHUTDOWN:
		if (!cam_req_mgr_is_shutdown()) {
			CAM_ERR(CAM_CORE, "SD shouldn't come from user space");
			return 0;
		}
		rc = cam_lens_driver_subdev_close_internal(sd, NULL);
		break;
	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid ioctl cmd: %u", cmd);
		rc = -ENOIOCTLCMD;
		break;
	}
	return rc;
}

#ifdef CONFIG_COMPAT
static long cam_lens_driver_init_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		cmd = VIDIOC_CAM_CONTROL;
		rc = cam_lens_driver_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc) {
			CAM_ERR(CAM_LENS_DRIVER,
				"Failed in lens_driver subdev handling rc: %d",
				rc);
			return rc;
		}
	break;
	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid compat ioctl: %d", cmd);
		rc = -ENOIOCTLCMD;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_LENS_DRIVER,
				"Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}
	return rc;
}
#endif

static struct v4l2_subdev_core_ops cam_lens_driver_subdev_core_ops = {
	.ioctl = cam_lens_driver_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_lens_driver_init_subdev_do_ioctl,
#endif
};

static struct v4l2_subdev_ops cam_lens_driver_subdev_ops = {
	.core = &cam_lens_driver_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops cam_lens_driver_internal_ops = {
	.close = cam_lens_driver_subdev_close,
};

static int cam_lens_driver_init_subdev(struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int rc = 0;

	l_ctrl->v4l2_dev_str.internal_ops =
		&cam_lens_driver_internal_ops;
	l_ctrl->v4l2_dev_str.ops =
		&cam_lens_driver_subdev_ops;
	strlcpy(l_ctrl->device_name, CAMX_LENS_DRIVER_DEV_NAME,
		sizeof(l_ctrl->device_name));
	l_ctrl->v4l2_dev_str.name =
		l_ctrl->device_name;
	l_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	l_ctrl->v4l2_dev_str.ent_function =
		CAM_LDM_DEVICE_TYPE;
	l_ctrl->v4l2_dev_str.token = l_ctrl;

	rc = cam_register_subdev(&(l_ctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_LENS_DRIVER, "Fail with cam_register_subdev rc: %d", rc);

	return rc;
}

static int cam_lens_driver_get_function_table(
	struct cam_lens_driver_ctrl_t *l_ctrl)
{
	uint32_t i = 0;
	int32_t rc = -EFAULT;
	struct cam_lens_driver_soc_private *soc_private;

	soc_private =
		(struct cam_lens_driver_soc_private *)l_ctrl->soc_info.soc_private;

	l_ctrl->lens_driver_intf = NULL;

	for (i = 0;
		i < ARRAY_SIZE(lens_driver_table); i++) {
		if (strcmp(soc_private->cam_lens_driver_name,
			lens_driver_table[i]->cam_lens_driver_name) == 0) {
			l_ctrl->lens_driver_intf = &lens_driver_table[i]->lens_driver_intf;
			rc = 0;
			break;
		}
	}

	if (l_ctrl->lens_driver_intf == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Lens Driver Interface not found");
		return rc;
	}

	return rc;
}

static int cam_lens_driver_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int32_t                             rc = 0;
	int32_t                             i = 0;
	struct cam_lens_driver_ctrl_t      *l_ctrl = NULL;
	struct cam_lens_driver_soc_private *soc_private = NULL;
	struct spi_device *spi = to_spi_device(dev);

	if (spi == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "spi device is NULL");
		return -ENODEV;
	}

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3;
	spi->chip_select = 0;
	spi_setup(spi);

	CAM_DBG(CAM_LENS_DRIVER, "irq[%d] cs[%x] CPHA[%x] CPOL[%x] CS_HIGH[%x]",
		spi->irq, spi->chip_select, (spi->mode & SPI_CPHA) ? 1 : 0,
		(spi->mode & SPI_CPOL) ? 1 : 0,
		(spi->mode & SPI_CS_HIGH) ? 1 : 0);
	CAM_DBG(CAM_LENS_DRIVER, "max_speed[%u]", spi->max_speed_hz);

	/* Create lens_driver control structure */
	l_ctrl = kzalloc(sizeof(*l_ctrl), GFP_KERNEL);
	if (!l_ctrl)
		return -ENOMEM;

	/*fill in platform device*/
	l_ctrl->soc_info.dev = &spi->dev;
	l_ctrl->soc_info.dev_name = spi->modalias;
	l_ctrl->spi_client = spi;

	soc_private = kzalloc(sizeof(struct cam_lens_driver_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto free_ctrl;
	}

	l_ctrl->soc_info.soc_private = soc_private;

	rc = cam_lens_driver_parse_dt(l_ctrl, &(spi->dev));
	if (rc < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Parsing lens_driver dt failed rc %d", rc);
		goto free_soc;
	}

	rc = cam_lens_driver_init_subdev(l_ctrl);
	if (rc)
		goto free_soc;

	l_ctrl->bridge_intf.device_hdl = -1;
	l_ctrl->bridge_intf.link_hdl = -1;
	l_ctrl->bridge_intf.ops.get_dev_info = NULL;
	l_ctrl->bridge_intf.ops.link_setup = NULL;
	l_ctrl->bridge_intf.ops.apply_req = NULL;
	l_ctrl->bridge_intf.ops.flush_req = NULL;
	l_ctrl->last_flush_req = 0;

	spi_set_drvdata(spi, l_ctrl);
	l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_INIT;
	v4l2_set_subdevdata(&l_ctrl->v4l2_dev_str.sd, l_ctrl);

	rc = cam_lens_driver_get_function_table(l_ctrl);
	if (rc < 0) {
		l_ctrl->lens_driver_intf = NULL;
		goto free_soc;
	}

	rc = cam_lens_driver_create_wq(l_ctrl);
	if (rc) {
		CAM_ERR(CAM_LENS_DRIVER, "unable to create a workqueue");
		goto free_soc;
	}

	l_ctrl->cmd_work_data =
		kzalloc(sizeof(struct lens_driver_cmd_work_data) * LDM_WORKQ_NUM_TASK,
		GFP_KERNEL);
	if (!l_ctrl->cmd_work_data) {
		CAM_ERR(CAM_LENS_DRIVER, "Mem reservation fail for cmd_work_data");
		goto free_wq;
	}

	for (i = 0; i < LDM_WORKQ_NUM_TASK; i++)
		l_ctrl->cmd_work->task.pool[i].payload = &l_ctrl->cmd_work_data[i];

	return rc;

free_wq:
	cam_req_mgr_workq_destroy(&l_ctrl->cmd_work);
free_soc:
	kfree(soc_private);
free_ctrl:
	kfree(l_ctrl);

	return rc;
}

static void cam_lens_driver_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct cam_lens_driver_ctrl_t      *l_ctrl;
	struct spi_device *spi = to_spi_device(dev);

	l_ctrl = spi_get_drvdata(spi);
	if (!l_ctrl) {
		CAM_ERR(CAM_LENS_DRIVER, "Lens Driver device is NULL");
		return;
	}

	cam_req_mgr_workq_destroy(&l_ctrl->cmd_work);
	mutex_lock(&(l_ctrl->lens_driver_mutex));
	cam_lens_driver_shutdown(l_ctrl);
	mutex_unlock(&(l_ctrl->lens_driver_mutex));
	cam_unregister_subdev(&(l_ctrl->v4l2_dev_str));
	kfree(l_ctrl->soc_info.soc_private);
	l_ctrl->soc_info.soc_private = NULL;
	v4l2_set_subdevdata(&l_ctrl->v4l2_dev_str.sd, NULL);
	spi_set_drvdata(spi, NULL);
	kfree(l_ctrl->cmd_work_data);
	kfree(l_ctrl);
}
const static struct component_ops cam_lens_driver_component_ops = {
	.bind = cam_lens_driver_component_bind,
	.unbind = cam_lens_driver_component_unbind,
};

static int32_t cam_lens_driver_spi_driver_probe(struct spi_device *spi)
{
	int32_t rc = 0;

	rc = component_add(&spi->dev, &cam_lens_driver_component_ops);
	if (rc)
		CAM_ERR(CAM_LENS_DRIVER, "failed to add component rc: %d", rc);

	return rc;
}

static int32_t cam_lens_driver_spi_driver_remove(struct spi_device *spi)
{
	component_del(&spi->dev, &cam_lens_driver_component_ops);
	return 0;
}

static const struct of_device_id cam_lens_driver_dt_match[] = {
	{.compatible = "qcom,lens_driver"},
	{}
};
MODULE_DEVICE_TABLE(of, cam_lens_driver_dt_match);

struct spi_driver cam_lens_driver_spi_driver = {
	.driver = {
		.name = "qcom,lens_driver",
		.owner = THIS_MODULE,
		.of_match_table = cam_lens_driver_dt_match,
	},
	.probe = cam_lens_driver_spi_driver_probe,
	.remove = cam_lens_driver_spi_driver_remove,
};

int cam_lens_driver_init(void)
{
	int32_t rc = 0;

	rc = spi_register_driver(&cam_lens_driver_spi_driver);
	if (rc < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi_register_driver failed rc = %d", rc);
		return rc;
	}

	return rc;
}

void cam_lens_driver_exit(void)
{
	spi_unregister_driver(&cam_lens_driver_spi_driver);
}

MODULE_DESCRIPTION("cam_lens_driver");
MODULE_LICENSE("GPL v2");
