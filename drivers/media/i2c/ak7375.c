// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Intel Corporation

#include <linux/acpi.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/rk-camera-module.h>
#include <linux/rk_vcm_head.h>
#include <linux/timekeeping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define AK7375_NAME		"ak7375"
#define AK7375_MAX_FOCUS_POS	4095
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define AK7375_FOCUS_STEPS	1
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define AK7375_CTRL_STEPS	64
#define AK7375_CTRL_DELAY_US	1000

#define AK7375_REG_POSITION	0x0
#define AK7375_REG_CONT		0x2
#define AK7375_MODE_ACTIVE	0x0
#define AK7375_MODE_STANDBY	0x40

/* ak7375 device structure */
struct ak7375_device {
	struct v4l2_ctrl_handler ctrls_vcm;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	/* active or standby mode */
	bool active;

	/* Rockchip VCM timing support */
	unsigned short current_lens_pos;
	unsigned int vcm_movefull_t;
	struct __kernel_old_timeval start_move_tv;
	struct __kernel_old_timeval end_move_tv;
	unsigned long move_ms;
	u32 module_index;
	const char *module_facing;
};

static inline struct ak7375_device *to_ak7375_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct ak7375_device, ctrls_vcm);
}

static inline struct ak7375_device *sd_to_ak7375_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct ak7375_device, sd);
}

static int ak7375_i2c_write(struct ak7375_device *ak7375,
	u8 addr, u16 data, u8 size)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ak7375->sd);
	u8 buf[3];
	int ret;

	if (size != 1 && size != 2)
		return -EINVAL;
	buf[0] = addr;
	buf[size] = data & 0xff;
	if (size == 2)
		buf[1] = (data >> 8) & 0xff;
	ret = i2c_master_send(client, (const char *)buf, size + 1);
	if (ret < 0)
		return ret;
	if (ret != size + 1)
		return -EIO;

	return 0;
}

static int ak7375_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ak7375_device *dev_vcm = to_ak7375_vcm(ctrl);
	int move_pos;
	long mv_us;
	int ret;

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		move_pos = abs((int)dev_vcm->current_lens_pos - ctrl->val);
		ret = ak7375_i2c_write(dev_vcm, AK7375_REG_POSITION,
				       ctrl->val << 4, 2);
		if (ret)
			return ret;
		dev_vcm->current_lens_pos = ctrl->val;
		dev_vcm->move_ms = (unsigned long)(
			(u64)dev_vcm->vcm_movefull_t * move_pos /
			AK7375_MAX_FOCUS_POS);
		dev_vcm->start_move_tv = ns_to_kernel_old_timeval(ktime_get_ns());
		mv_us = dev_vcm->start_move_tv.tv_usec +
			dev_vcm->move_ms * 1000;
		if (mv_us >= 1000000) {
			dev_vcm->end_move_tv.tv_sec =
				dev_vcm->start_move_tv.tv_sec + 1;
			dev_vcm->end_move_tv.tv_usec = mv_us - 1000000;
		} else {
			dev_vcm->end_move_tv.tv_sec =
				dev_vcm->start_move_tv.tv_sec;
			dev_vcm->end_move_tv.tv_usec = mv_us;
		}
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_ctrl_ops ak7375_vcm_ctrl_ops = {
	.s_ctrl = ak7375_set_ctrl,
};

static int ak7375_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int ak7375_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops ak7375_int_ops = {
	.open = ak7375_open,
	.close = ak7375_close,
};

static long ak7375_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ak7375_device *dev_vcm = sd_to_ak7375_vcm(sd);
	struct rk_cam_vcm_tim *vcm_tim;

	if (cmd == RK_VIDIOC_VCM_TIMEINFO) {
		vcm_tim = (struct rk_cam_vcm_tim *)arg;
		vcm_tim->vcm_start_t.tv_sec = dev_vcm->start_move_tv.tv_sec;
		vcm_tim->vcm_start_t.tv_usec = dev_vcm->start_move_tv.tv_usec;
		vcm_tim->vcm_end_t.tv_sec = dev_vcm->end_move_tv.tv_sec;
		vcm_tim->vcm_end_t.tv_usec = dev_vcm->end_move_tv.tv_usec;
		return 0;
	}

	dev_err(sd->dev, "cmd 0x%x not supported\n", cmd);
	return -EINVAL;
}

#ifdef CONFIG_COMPAT
static long ak7375_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rk_cam_compat_vcm_tim compat_vcm_tim;
	struct rk_cam_vcm_tim vcm_tim;
	long ret;

	if (cmd == RK_VIDIOC_COMPAT_VCM_TIMEINFO) {
		struct rk_cam_compat_vcm_tim __user *p32 = up;

		ret = ak7375_ioctl(sd, RK_VIDIOC_VCM_TIMEINFO, &vcm_tim);
		compat_vcm_tim.vcm_start_t.tv_sec = vcm_tim.vcm_start_t.tv_sec;
		compat_vcm_tim.vcm_start_t.tv_usec = vcm_tim.vcm_start_t.tv_usec;
		compat_vcm_tim.vcm_end_t.tv_sec = vcm_tim.vcm_end_t.tv_sec;
		compat_vcm_tim.vcm_end_t.tv_usec = vcm_tim.vcm_end_t.tv_usec;

		put_user(compat_vcm_tim.vcm_start_t.tv_sec,
			 &p32->vcm_start_t.tv_sec);
		put_user(compat_vcm_tim.vcm_start_t.tv_usec,
			 &p32->vcm_start_t.tv_usec);
		put_user(compat_vcm_tim.vcm_end_t.tv_sec,
			 &p32->vcm_end_t.tv_sec);
		put_user(compat_vcm_tim.vcm_end_t.tv_usec,
			 &p32->vcm_end_t.tv_usec);
		return ret;
	}

	dev_err(sd->dev, "cmd 0x%x not supported\n", cmd);
	return -EINVAL;
}
#endif

static const struct v4l2_subdev_core_ops ak7375_core_ops = {
	.ioctl = ak7375_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ak7375_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_ops ak7375_ops = {
	.core = &ak7375_core_ops,
};

static void ak7375_subdev_cleanup(struct ak7375_device *ak7375_dev)
{
	v4l2_async_unregister_subdev(&ak7375_dev->sd);
	v4l2_ctrl_handler_free(&ak7375_dev->ctrls_vcm);
	media_entity_cleanup(&ak7375_dev->sd.entity);
}

static int ak7375_init_controls(struct ak7375_device *dev_vcm)
{
	struct v4l2_ctrl_handler *hdl = &dev_vcm->ctrls_vcm;
	const struct v4l2_ctrl_ops *ops = &ak7375_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	dev_vcm->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
		0, AK7375_MAX_FOCUS_POS, AK7375_FOCUS_STEPS, 0);

	if (hdl->error)
		dev_err(dev_vcm->sd.dev, "%s fail error: 0x%x\n",
			__func__, hdl->error);
	dev_vcm->sd.ctrl_handler = hdl;

	return hdl->error;
}

static int ak7375_parse_dt(struct i2c_client *client,
			  struct ak7375_device *ak7375_dev)
{
	struct device_node *np = client->dev.of_node;
	int ret = 0;

	ret = of_property_read_u32(np, "rockchip,camera-module-index",
				   &ak7375_dev->module_index);
	if (ret) {
		dev_info(&client->dev,
			 "could not get module index, use default 0\n");
		ak7375_dev->module_index = 0;
	}

	ret = of_property_read_string(np, "rockchip,camera-module-facing",
					&ak7375_dev->module_facing);
	if (ret) {
		dev_info(&client->dev,
			 "could not get module facing, use default back\n");
		ak7375_dev->module_facing = "back";
	}

	dev_info(&client->dev, "module_index=%u, module_facing=%s\n",
		 ak7375_dev->module_index, ak7375_dev->module_facing);

	return 0;
}

static int ak7375_i2c_read(struct i2c_client *client, u8 addr, u8 *val)
{
	int ret;

	ret = i2c_master_send(client, &addr, 1);
	if (ret < 0)
		return ret;
	ret = i2c_master_recv(client, val, 1);
	if (ret < 0)
		return ret;
	return 0;
}

static int ak7375_probe(struct i2c_client *client)
{
	struct ak7375_device *ak7375_dev;
	char facing[2];
	u8 reg_val;
	int ret;

	dev_info(&client->dev, "ak7375 probing at I2C addr 0x%02x\n",
		 client->addr);

	ak7375_dev = devm_kzalloc(&client->dev, sizeof(*ak7375_dev),
				  GFP_KERNEL);
	if (!ak7375_dev)
		return -ENOMEM;

	/* Verify device responds on I2C before registering subdev */
	ret = ak7375_i2c_read(client, AK7375_REG_CONT, &reg_val);
	if (ret < 0) {
		dev_err(&client->dev,
			"ak7375 not found or I2C error (ret=%d) — check wiring and I2C address\n",
			ret);
		return -ENODEV;
	}
	dev_info(&client->dev, "ak7375 CONT reg=0x%02x (0x40=standby, 0x00=active)\n",
		 reg_val);

	v4l2_i2c_subdev_init(&ak7375_dev->sd, client, &ak7375_ops);
	ak7375_dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ak7375_dev->sd.internal_ops = &ak7375_int_ops;
	ak7375_dev->sd.entity.function = MEDIA_ENT_F_LENS;
	dev_info(&client->dev, "ak7375 subdev initialized\n");

	/* Default full-range move time; overridable from DT */
	ak7375_dev->vcm_movefull_t = 200;
	of_property_read_u32(client->dev.of_node, "rockchip,vcm-move-time-ms",
			     &ak7375_dev->vcm_movefull_t);

	/* Optional Rockchip module index/facing for subdev naming */
	if (!of_property_read_u32(client->dev.of_node,
				  RKMODULE_CAMERA_MODULE_INDEX,
				  &ak7375_dev->module_index) &&
	    !of_property_read_string(client->dev.of_node,
				     RKMODULE_CAMERA_MODULE_FACING,
				     &ak7375_dev->module_facing)) {
		memset(facing, 0, sizeof(facing));
		facing[0] = (strcmp(ak7375_dev->module_facing, "back") == 0) ?
			     'b' : 'f';
		snprintf(ak7375_dev->sd.name, sizeof(ak7375_dev->sd.name),
			 "m%02d_%s_%s %s",
			 ak7375_dev->module_index, facing,
			 AK7375_NAME, dev_name(&client->dev));
	}

	ret = ak7375_init_controls(ak7375_dev);
	if (ret)
		goto err_cleanup;

	ret = media_entity_pads_init(&ak7375_dev->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	ret = v4l2_async_register_subdev(&ak7375_dev->sd);
	if (ret < 0)
		goto err_cleanup;

	/* Initialize move timestamps */
	ak7375_dev->start_move_tv = ns_to_kernel_old_timeval(ktime_get_ns());
	ak7375_dev->end_move_tv = ak7375_dev->start_move_tv;

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	dev_info(&client->dev, "ak7375 probe success, subdev: %s\n",
		 ak7375_dev->sd.name);
	return 0;

err_cleanup:
	v4l2_ctrl_handler_free(&ak7375_dev->ctrls_vcm);
	media_entity_cleanup(&ak7375_dev->sd.entity);

	dev_err(&client->dev, "ak7375 probe failed: %d\n", ret);
	return ret;
}

static void ak7375_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ak7375_device *ak7375_dev = sd_to_ak7375_vcm(sd);

	ak7375_subdev_cleanup(ak7375_dev);
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

/*
 * This function sets the vcm position, so it consumes least current
 * The lens position is gradually moved in units of AK7375_CTRL_STEPS,
 * to make the movements smoothly.
 */
static int __maybe_unused ak7375_vcm_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ak7375_device *ak7375_dev = sd_to_ak7375_vcm(sd);
	int ret, val;

	if (!ak7375_dev->active)
		return 0;

	for (val = ak7375_dev->focus->val & ~(AK7375_CTRL_STEPS - 1);
	     val >= 0; val -= AK7375_CTRL_STEPS) {
		ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_POSITION,
				       val << 4, 2);
		if (ret)
			dev_err_once(dev, "%s I2C failure: %d\n",
				     __func__, ret);
		usleep_range(AK7375_CTRL_DELAY_US, AK7375_CTRL_DELAY_US + 10);
	}

	ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_CONT,
			       AK7375_MODE_STANDBY, 1);
	if (ret)
		dev_err(dev, "%s I2C failure: %d\n", __func__, ret);

	ak7375_dev->active = false;

	return 0;
}

/*
 * This function sets the vcm position to the value set by the user
 * through v4l2_ctrl_ops s_ctrl handler
 * The lens position is gradually moved in units of AK7375_CTRL_STEPS,
 * to make the movements smoothly.
 */
static int __maybe_unused ak7375_vcm_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ak7375_device *ak7375_dev = sd_to_ak7375_vcm(sd);
	int ret, val;

	if (ak7375_dev->active)
		return 0;

	ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_CONT,
		AK7375_MODE_ACTIVE, 1);
	if (ret) {
		dev_err(dev, "%s I2C failure: %d\n", __func__, ret);
		return ret;
	}

	for (val = ak7375_dev->focus->val % AK7375_CTRL_STEPS;
	     val <= ak7375_dev->focus->val;
	     val += AK7375_CTRL_STEPS) {
		ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_POSITION,
				       val << 4, 2);
		if (ret)
			dev_err_ratelimited(dev, "%s I2C failure: %d\n",
						__func__, ret);
		usleep_range(AK7375_CTRL_DELAY_US, AK7375_CTRL_DELAY_US + 10);
	}

	ak7375_dev->active = true;

	return 0;
}

static const struct of_device_id ak7375_of_table[] = {
	{ .compatible = "asahi-kasei,ak7375" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ak7375_of_table);

static const struct dev_pm_ops ak7375_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ak7375_vcm_suspend, ak7375_vcm_resume)
	SET_RUNTIME_PM_OPS(ak7375_vcm_suspend, ak7375_vcm_resume, NULL)
};

static struct i2c_driver ak7375_i2c_driver = {
	.driver = {
		.name = "ak7375",
		.pm = &ak7375_pm_ops,
		.of_match_table = ak7375_of_table,
	},
	.probe_new = ak7375_probe,
	.remove = ak7375_remove,
};
module_i2c_driver(ak7375_i2c_driver);

MODULE_AUTHOR("Tianshu Qiu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_DESCRIPTION("AK7375 VCM driver");
MODULE_LICENSE("GPL v2");
