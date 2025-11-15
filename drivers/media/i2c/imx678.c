// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 Framos. All rights reserved.
 *
 * imx678.c - Framos imx678.c driver
 */

#include <linux/unaligned.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#include "imx678.h"

#define IMX678_K_FACTOR				1000LL
#define IMX678_M_FACTOR				1000000LL
#define IMX678_G_FACTOR				1000000000LL

#define IMX678_XCLK_FREQ			74250000

#define IMX678_LINK_FREQ_2376		(2376000000/2)
#define IMX678_LINK_FREQ_2079		(2079000000/2)
#define IMX678_LINK_FREQ_1782		(1782000000/2)
#define IMX678_LINK_FREQ_1440		(1440000000/2)
#define IMX678_LINK_FREQ_1188		(1188000000/2)
#define IMX678_LINK_FREQ_891		(891000000/2)
#define IMX678_LINK_FREQ_720		(720000000/2)
#define IMX678_LINK_FREQ_594		(594000000/2)

#define IMX678_MODE_STANDBY				0x01
#define IMX678_MODE_STREAMING			0x00

#define IMX678_MIN_SHR0_LENGTH				3
#define IMX678_MIN_INTEGRATION_LINES		1

#define IMX678_ANA_GAIN_MIN				0
#define IMX678_ANA_GAIN_MAX				240
#define IMX678_ANA_GAIN_STEP			1
#define IMX678_ANA_GAIN_DEFAULT			0

#define IMX678_EXP_GAIN_MIN 		0
#define IMX678_EXP_GAIN_MAX 		5
#define IMX678_EXP_GAIN_STEP 		1
#define IMX678_EXP_GAIN_DEFAULT 	2

#define IMX678_BLACK_LEVEL_MIN			0
#define IMX678_BLACK_LEVEL_STEP			1
#define IMX678_MAX_BLACK_LEVEL_10BPP		1023
#define IMX678_MAX_BLACK_LEVEL_12BPP		4095
#define IMX678_DEFAULT_BLACK_LEVEL_10BPP	50
#define IMX678_DEFAULT_BLACK_LEVEL_12BPP	200

enum pad_types {
	IMAGE_PAD,
	NUM_PADS
};

enum imx678_frame_mode {
	IMX678_HDR_MODE_ALL_PIXEL,
	IMX678_HDR_MODE_BINNING,
	IMX678_HDR_MODE_DOL_HDR,
	IMX678_HDR_MODE_DOL_HDR_BINNING,
	IMX678_HDR_MODE_CLEAR_HDR,
	IMX678_HDR_MODE_CLEAR_HDR_BINNING,
};

static const char * const imx678_frame_mode_menu[] = {
	[IMX678_HDR_MODE_ALL_PIXEL] = "All pixel (HDR off)",
	[IMX678_HDR_MODE_BINNING] = "H2V2 binning (HDR off)",
	[IMX678_HDR_MODE_DOL_HDR] = "DOL HDR",
	[IMX678_HDR_MODE_DOL_HDR_BINNING] = "DOL HDR + H2V2 binning",
	[IMX678_HDR_MODE_CLEAR_HDR] = "Clear HDR",
	[IMX678_HDR_MODE_CLEAR_HDR_BINNING] = "Clear HDR + H2V2 binning",
};

#define IMX678_NATIVE_WIDTH			3856U
#define IMX678_NATIVE_HEIGHT		2180U
#define IMX678_PIXEL_ARRAY_LEFT		0U
#define IMX678_PIXEL_ARRAY_TOP		0U
#define IMX678_PIXEL_ARRAY_WIDTH	3856U
#define IMX678_PIXEL_ARRAY_HEIGHT	2180U

#define V4L2_CID_FRAME_MODE 			(V4L2_CID_USER_IMX_BASE + 0)
#define V4L2_CID_FRAME_RATE				(V4L2_CID_USER_IMX_BASE + 1)
#define V4L2_CID_OPERATION_MODE			(V4L2_CID_USER_IMX_BASE + 2)
#define V4L2_CID_SYNC_MODE				(V4L2_CID_USER_IMX_BASE + 3)
#define V4L2_CID_EXPOSURE_SHORT			(V4L2_CID_USER_IMX_BASE + 4)
#define V4L2_CID_ANALOGUE_GAIN_SHORT	(V4L2_CID_USER_IMX_BASE + 5)
#define V4L2_CID_EXPONENTIAL_GAIN		(V4L2_CID_USER_IMX_BASE + 6)

struct imx678_reg_list {
	unsigned int num_of_regs;
	const struct imx678_reg *regs;
};

enum {
	IMX678_BIT_DEPTH_10,
	IMX678_BIT_DEPTH_12,
} bit_depth;

static const s64 imx678_link_freq_menu[] = {
	[IMX678_2376_MBPS] = IMX678_LINK_FREQ_2376,
	[IMX678_2079_MBPS] = IMX678_LINK_FREQ_2079,
	[IMX678_1782_MBPS] = IMX678_LINK_FREQ_1782,
	[IMX678_1440_MBPS] = IMX678_LINK_FREQ_1440,
	[IMX678_1188_MBPS] = IMX678_LINK_FREQ_1188,
	[IMX678_891_MBPS] = IMX678_LINK_FREQ_891,
	[IMX678_720_MBPS] = IMX678_LINK_FREQ_720,
};

struct imx678_link_mode {
	enum link_freq link_freq;
	enum bit_depth bit_depth;
	unsigned uint16_t hmax;
	unsigned uint16_t modes;
};

#define LINK_MODE_ALL_PIXEL 	(1 << 0)
#define LINK_MODE_ALL_BINNING	(1 << 1)
#define LINK_MODE_DOL_HDR		(1 << 2)
#define LINK_MODE_DOL_BINNING	(1 << 3)
#define LINK_MODE_CLEAR_HDR		(1 << 4)
#define LINK_MODE_CLEAR_BINNING	(1 << 5)

struct imx678_frame_mode {
	unsigned int width;
	unsigned int height;
	uint8_t frame_count;
	unsigned uint16_t flag;
};

static const struct imx678_frame_mode frame_mode_configs[] = {
	[IMX678_HDR_MODE_ALL_PIXEL] = {
		.width = IMX678_DEFAULT_WIDTH,
		.height = IMX678_DEFAULT_HEIGHT,
		.frame_count = 1,
		.flag = LINK_MODE_ALL_PIXEL,
	},
	[IMX678_HDR_MODE_BINNING] = {
		.width = IMX678_DEFAULT_WIDTH / 2,
		.height = IMX678_DEFAULT_HEIGHT / 2,
		.frame_count = 1,
		.flag = LINK_MODE_ALL_BINNING,
	},
	[IMX678_HDR_MODE_DOL_HDR] = {
		.width = IMX678_DEFAULT_WIDTH,
		.height = IMX678_DEFAULT_HEIGHT,
		.frame_count = 2,
		.flag = LINK_MODE_DOL_HDR,
	},
	[IMX678_HDR_MODE_DOL_HDR_BINNING] = {
		.width = IMX678_DEFAULT_WIDTH / 2,
		.height = IMX678_DEFAULT_HEIGHT / 2,
		.frame_count = 2,
		.flag = LINK_MODE_DOL_BINNING,
	},
	[IMX678_HDR_MODE_CLEAR_HDR] = {
		.width = IMX678_DEFAULT_WIDTH,
		.height = IMX678_DEFAULT_HEIGHT,
		.frame_count = 2,
		.flag = LINK_MODE_CLEAR_HDR,
	},
	[IMX678_HDR_MODE_CLEAR_HDR_BINNING] = {
		.width = IMX678_DEFAULT_WIDTH / 2,
		.height = IMX678_DEFAULT_HEIGHT / 2,
		.frame_count = 2,
		.flag = LINK_MODE_CLEAR_BINNING,
	},
};

static const struct imx678_link_mode all_pixel_link_modes[] = {
	// 25 fps, hmax 1320
	{
		.link_freq = IMX678_720_MBPS, 	.bit_depth = 10, .hmax = 1320,
		.modes = LINK_MODE_ALL_PIXEL | LINK_MODE_CLEAR_HDR | LINK_MODE_DOL_HDR,
	},
	{
		.link_freq = IMX678_720_MBPS, 	.bit_depth = 12, .hmax = 1320,
		.modes = LINK_MODE_ALL_BINNING | LINK_MODE_DOL_BINNING | LINK_MODE_CLEAR_BINNING,
	},
	{
		.link_freq = IMX678_891_MBPS, 	.bit_depth = 12, .hmax = 1320,
		.modes = LINK_MODE_ALL_PIXEL | LINK_MODE_CLEAR_HDR | LINK_MODE_DOL_HDR,
	},

	// 30 fps, hmax 1100
	{
		.link_freq = IMX678_720_MBPS, 	.bit_depth = 12, .hmax = 1100,
		.modes = LINK_MODE_ALL_BINNING | LINK_MODE_DOL_BINNING | LINK_MODE_CLEAR_BINNING,
	},
	{
		.link_freq = IMX678_891_MBPS, 	.bit_depth = 10, .hmax = 1100,
		.modes = LINK_MODE_ALL_PIXEL | LINK_MODE_CLEAR_HDR | LINK_MODE_DOL_HDR,
	},
	{
		.link_freq = IMX678_1188_MBPS,	.bit_depth = 12, .hmax = 1100,
		.modes = LINK_MODE_ALL_PIXEL | LINK_MODE_CLEAR_HDR | LINK_MODE_DOL_HDR,
	},

	// 50 fps, hmax 660
	{
		.link_freq = IMX678_1440_MBPS,	.bit_depth = 10, .hmax = 660,
		.modes = LINK_MODE_ALL_PIXEL | LINK_MODE_CLEAR_HDR | LINK_MODE_DOL_HDR,
	},
	{
		.link_freq = IMX678_1440_MBPS,	.bit_depth = 12, .hmax = 660,
		.modes = LINK_MODE_ALL_PIXEL | LINK_MODE_CLEAR_HDR |
			LINK_MODE_ALL_BINNING | LINK_MODE_DOL_HDR | LINK_MODE_DOL_BINNING |
			LINK_MODE_CLEAR_BINNING,
	}, 

	// 60 fps, hmax 550
	{
		.link_freq = IMX678_1440_MBPS,	.bit_depth = 10, .hmax = 550,
		.modes = LINK_MODE_ALL_PIXEL | LINK_MODE_CLEAR_HDR | LINK_MODE_DOL_HDR,
	},
	{
		.link_freq = IMX678_1440_MBPS,	.bit_depth = 12, .hmax = 550,
		.modes = LINK_MODE_ALL_BINNING | LINK_MODE_DOL_BINNING,
	},
	{
		.link_freq = IMX678_1782_MBPS,	.bit_depth = 12, .hmax = 550,
		.modes = LINK_MODE_ALL_PIXEL | LINK_MODE_DOL_HDR,
	},

	// 72 fps, hmax 458
	{
		.link_freq = IMX678_2079_MBPS,	.bit_depth = 10, .hmax = 458,
		.modes = LINK_MODE_ALL_PIXEL | LINK_MODE_DOL_HDR,
	},
	{
		.link_freq = IMX678_2376_MBPS,	.bit_depth = 12, .hmax = 458,
		.modes = LINK_MODE_ALL_BINNING,
	}
};

static const u32 bayer_formats[] = {
	MEDIA_BUS_FMT_SRGGB12_1X12, // no flip
	MEDIA_BUS_FMT_SGRBG12_1X12, // H flip
	MEDIA_BUS_FMT_SGBRG12_1X12, // V flip
	MEDIA_BUS_FMT_SBGGR12_1X12, // H+V flip

	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

struct imx678 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	unsigned int fmt_code;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *xmaster;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *framerate;
	struct v4l2_ctrl *operation_mode;
	struct v4l2_ctrl *sync_mode;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *blklvl;

	// DOL mode
	struct v4l2_ctrl *exposure_short;
	struct v4l2_ctrl *gain_short;

	// Clear mode
	struct v4l2_ctrl *exponential_gain;

	u64 line_time;
	u32 frame_length;

	const char *gmsl;

	const struct imx678_frame_mode *frame_mode;
	struct mutex mutex;
	bool streaming;
};

static inline struct imx678 *to_imx678(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx678, sd);
}

static const char * const imx678_test_pattern_menu[] = {
	[0] = "Disabled",
	[1] = "000h Pattern",
	[2] = "3FF(FFFh) Pattern",
	[3] = "155(555h) Pattern",
	[4] = "2AA(AAAh) Pattern",
	[5] = "555/AAAh Pattern",
	[6] = "AAA/555h Pattern",
	[7] = "000/555h Pattern",
	[8] = "555/000h Pattern",
	[9] = "000/FFFh Pattern",
	[10] = "FFF/000h Pattern",
	[11] = "H Color-bar",
	[12] = "V Color-bar"
};

static const char * const imx678_operation_mode_menu[] = {
	[MASTER_MODE] = "Master Mode",
	[SLAVE_MODE] = "Slave Mode",
};

static const char * const imx678_sync_mode_menu[] = {
	[NO_SYNC] = "No Sync",
	[INTERNAL_SYNC] = "Internal Sync",
	[EXTERNAL_SYNC] = "External Sync",
};

static int imx678_read_reg(struct imx678 *imx678, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

static int imx678_write_reg(struct imx678 *imx678, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_le32(val, buf + 2);

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int imx678_write_hold_reg(struct imx678 *imx678, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	int ret;

	ret = imx678_write_reg(imx678, REGHOLD, 1, 0x01);
	if (ret) {
		dev_err(dev, "%s failed to write reghold register\n",
								__func__);
		return ret;
	}

	ret = imx678_write_reg(imx678, reg, len, val);
	if (ret)
		goto reghold_off;

	ret = imx678_write_reg(imx678, REGHOLD, 1, 0x00);
	if (ret) {
		dev_err(dev, "%s failed to write reghold register\n",
								__func__);
		return ret;
	}

	return 0;

reghold_off:
	ret = imx678_write_reg(imx678, REGHOLD, 1, 0x00);
	if (ret) {
		dev_err(dev, "%s failed to write reghold register\n",
								__func__);
		return ret;
	}
	return ret;

}

static int imx678_write_table(struct imx678 *imx678,
				 const struct imx678_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx678_write_reg(imx678, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					"Failed to write reg 0x%4.4x. error = %d\n",
					regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

static u32 imx678_get_format_code(struct imx678 *imx678, u32 code)
{
	// Standard modes are 0 mod 4
	// H flip - 1 mod 4
	// V flip - 2 mod 4
	// H+V flip - 3 mod 4
	unsigned int i = (im678->hflip->val) ? 1 : 0
		+ (imx678->vflip->val) ? 2 : 0;

	for (i; i < ARRAY_SIZE(bayer_formats); i += 4) {
		if (bayer_formats[i] == code)
			return bayer_formats[i];
	}

	return bayer_formats[0];
}

static int imx678_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx678 *imx678 = to_imx678(sd);
	struct v4l2_mbus_framefmt *fmt_img =
		v4l2_subdev_state_get_format(fh->state, IMAGE_PAD);
	struct v4l2_rect *crop;

	mutex_lock(&imx678->mutex);

	fmt_img->width = frame_mode_configs[0].width;
	fmt_img->height = frame_mode_configs[0].height;
	fmt_img->code = MEDIA_BUS_FMT_SRGGB12_1X12;
	fmt_img->field = V4L2_FIELD_NONE;

	crop = v4l2_subdev_state_get_crop(fh->state, IMAGE_PAD);
	crop->left = IMX678_PIXEL_ARRAY_LEFT;
	crop->top = IMX678_PIXEL_ARRAY_TOP;
	crop->width = IMX678_PIXEL_ARRAY_WIDTH;
	crop->height = IMX678_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx678->mutex);

	return 0;
}

static int imx678_set_exposure(struct imx678 *imx678, u64 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	const struct imx678_frame_mode *mode = imx678->mode;
	u64 exposure;
	int ret;

	exposure = imx678->vblank->val + mode->height - val;

	ret = imx678_write_hold_reg(imx678, SHR0_LOW, 3, exposure);
	if (ret) {
		dev_err(dev, "%s failed to set exposure\n", __func__);
		return ret;
	}

	return ret;
}

static void imx678_adjust_exposure_range(struct imx678 *imx678)
{
	const struct imx678_frame_mode *mode = imx678->mode;
	u64 exposure_max;

	exposure_max = imx678->vblank->val + mode->height - IMX678_MIN_SHR0_LENGTH;

	__v4l2_ctrl_modify_range(imx678->exposure, IMX678_MIN_INTEGRATION_LINES,
				exposure_max, 1,
				exposure_max);
}

static int imx678_set_frame_rate(struct imx678 *imx678, u64 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	int ret;

	ret = imx678_write_hold_reg(imx678, VMAX_LOW, 3, imx678->frame_length);

	if (ret) {
		dev_err(dev, "%s failed to set frame rate\n", __func__);
		return ret;
	}

	return ret;
}

static void imx678_update_frame_rate(struct imx678 *imx678, u64 val)
{
	const struct imx678_frame_mode *mode = imx678->mode;
	u32 update_vblank;

	imx678->frame_length = (IMX678_M_FACTOR * IMX678_G_FACTOR) /
						(val * imx678->line_time);
	imx678->frame_length = (imx678->frame_length % 2) ?
				imx678->frame_length + 1 : imx678->frame_length;

	update_vblank = imx678->frame_length - mode->height;

	__v4l2_ctrl_modify_range(imx678->vblank, update_vblank,
				 update_vblank, 1, update_vblank);

	__v4l2_ctrl_s_ctrl(imx678->vblank, update_vblank);
}

static int imx678_set_hmax_register(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	const struct imx678_frame_mode *mode = imx678->mode;
	int ret;

	ret = imx678_write_hold_reg(imx678, HMAX_LOW, 2, mode->hmax);
	if (ret)
		dev_err(dev, "%s failed to write HMAX register\n", __func__);

	dev_dbg(dev, "%s: hmax: 0x%x\n", __func__, mode->hmax);

	return ret;

}

static int imx678_set_test_pattern(struct imx678 *imx678, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	int ret;

	if (val) {
		ret = imx678_write_table(imx678, mode_enable_pattern_generator,
				ARRAY_SIZE(mode_enable_pattern_generator));
		if (ret)
			goto fail;

		ret = imx678_write_reg(imx678, TPG_PATSEL_DUOUT, 1, val - 1);
		if (ret)
			goto fail;
	} else {
		ret = imx678_write_table(imx678, mode_disable_pattern_generator,
				ARRAY_SIZE(mode_disable_pattern_generator));
		if (ret)
			goto fail;
	}

	return 0;

fail:
	dev_err(dev, "%s: error setting test pattern\n", __func__);
	return ret;
}

static void imx678_update_blklvl_range(struct imx678 *imx678)
{
	switch (imx678->fmt_code) {
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		__v4l2_ctrl_modify_range(imx678->blklvl, IMX678_BLACK_LEVEL_MIN,
					IMX678_MAX_BLACK_LEVEL_12BPP,
					IMX678_BLACK_LEVEL_STEP,
					IMX678_DEFAULT_BLACK_LEVEL_12BPP);
		__v4l2_ctrl_s_ctrl(imx678->blklvl, IMX678_DEFAULT_BLACK_LEVEL_12BPP);
		break;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		__v4l2_ctrl_modify_range(imx678->blklvl, IMX678_BLACK_LEVEL_MIN,
					IMX678_MAX_BLACK_LEVEL_10BPP,
					IMX678_BLACK_LEVEL_STEP,
					IMX678_DEFAULT_BLACK_LEVEL_10BPP);
		__v4l2_ctrl_s_ctrl(imx678->blklvl, IMX678_DEFAULT_BLACK_LEVEL_10BPP);
		break;
	}

}

static int imx678_set_blklvl(struct imx678 *imx678, u64 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	u64 black_level_reg;
	int ret;

	if (imx678->fmt_code == MEDIA_BUS_FMT_SRGGB10_1X10)
		black_level_reg = val;
	else
		black_level_reg = val >> 2;

	ret = imx678_write_hold_reg(imx678, BLKLEVEL_LOW, 2, black_level_reg);

	if (ret) {
		dev_err(dev, "%s failed to adjust blklvl register\n",
								__func__);
	}

	dev_dbg(dev, "%s: blklvl value: %lld\n", __func__, black_level_reg);

	return ret;
}

static int imx678_set_operation_mode(struct imx678 *imx678, u32 val)
{
	gpiod_set_raw_value_cansleep(imx678->xmaster, val);

	return 0;
}

static int imx678_set_sync_mode(struct imx678 *imx678, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	int ret = 0;
	u8 extmode;

	if (val == EXTERNAL_SYNC)
		extmode = 1;
	else
		extmode = 0;

	ret = imx678_write_reg(imx678, EXTMODE, 1, extmode);
	if (ret)
		dev_err(dev, "%s: error setting sync mode\n", __func__);

	return ret;
}

static int imx678_configure_triggering_pins(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	int ret = 0;
	u8 xvs_xhs_drv = 0xF;

	switch (imx678->operation_mode->val) {
	case MASTER_MODE:
		if (imx678->sync_mode->val == INTERNAL_SYNC) {
			xvs_xhs_drv = 0x0;
			dev_dbg(dev,
				"%s: Sensor is in - Internal sync Master mode\n",
								__func__);
		} else {
			xvs_xhs_drv = 0xF;
			dev_dbg(dev,
				"%s: Sensor is in - No sync Master mode or External high-z mode\n",
								__func__);
		}
		break;
	case SLAVE_MODE:
		xvs_xhs_drv = 0xF;
		dev_dbg(dev, "%s: Sensor is in Slave mode\n", __func__);
		break;

	default:
		dev_err(dev, "%s: unknown synchronizing function.\n", __func__);
		return -EINVAL;
	}

	ret = imx678_write_reg(imx678, XVS_XHS_DRV, 1, xvs_xhs_drv);
	if (ret) {
		dev_err(dev, "%s: error setting Slave mode\n", __func__);
		return ret;
	}

	dev_dbg(dev, "%s: XVS_XHS driver register: 0x%x\n", __func__, xvs_xhs_drv);

	return 0;
}

static int imx678_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx678 *imx678 =
		container_of(ctrl->handler, struct imx678, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_FRAME_RATE:
		imx678_update_frame_rate(imx678, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		imx678_adjust_exposure_range(imx678);
		break;
	case V4L2_CID_FRAME_MODE:
		imx678->mode = frame_mode_configs[ctrl->val];
		break;
	}

	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx678_write_hold_reg(imx678, GAIN_0_LOW, 2, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx678_set_exposure(imx678, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx678_set_test_pattern(imx678, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = imx678_write_reg(imx678, HREVERSE, 1, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = imx678_write_reg(imx678, VREVERSE, 1, ctrl->val);
		break;
	case V4L2_CID_FRAME_RATE:
		ret = imx678_set_frame_rate(imx678, ctrl->val);
		break;
	case V4L2_CID_BLACK_LEVEL:
		ret = imx678_set_blklvl(imx678, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_SHORT:
		ret = imx678_set_exposure_short(imx678, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN_SHORT:
		ret = imx678_write_hold_reg(imx678, GAIN_1_LOW, 2, ctrl->val);
		break;
	case V4L2_CID_EXPONENTIAL_GAIN:
		ret = imx678_write_hold_reg(imx678, EXP_GAIN, 1, ctrl->val);
		break;
	case V4L2_CID_OPERATION_MODE:
		ret = imx678_set_operation_mode(imx678, ctrl->val);
		break;
	case V4L2_CID_SYNC_MODE:
		ret = imx678_set_sync_mode(imx678, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx678_ctrl_ops = {
	.s_ctrl = imx678_set_ctrl,
};

static int imx678_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx678 *imx678 = to_imx678(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= (ARRAY_SIZE(bayer_formats)))
			return -EINVAL;

		code->code = bayer_formats[code->index];
	}

	return 0;
}

static int imx678_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx678 *imx678 = to_imx678(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		const struct imx678_frame_mode *mode_list;
		unsigned int num_modes;

		get_mode_table(fse->code, &mode_list, &num_modes);

		if (fse->index >= num_modes)
			return -EINVAL;

		if (fse->code != imx678_get_format_code(imx678, fse->code))
			return -EINVAL;

		fse->min_width = mode_list[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].height;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx678_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx678_update_image_pad_format(struct imx678 *imx678,
						const struct imx678_frame_mode *mode,
						struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx678_reset_colorspace(&fmt->format);
}

static void imx678_set_limits(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	const struct imx678_frame_mode *mode = imx678->mode;
	u64 vblank, max_framerate;

	dev_dbg(dev, "%s: mode: %dx%d\n", __func__, mode->width, mode->height);

	vblank = IMX678_MIN_FRAME_LENGTH_DELTA;

	__v4l2_ctrl_modify_range(imx678->vblank, vblank,
				 vblank, 1, vblank);
	dev_dbg(dev, "%s: vblank: %lld\n", __func__, vblank);

	__v4l2_ctrl_modify_range(imx678->pixel_rate, mode->pixel_rate,
				 mode->pixel_rate, 1, mode->pixel_rate);
	dev_dbg(dev, "%s: pixel rate: %d\n", __func__, mode->pixel_rate);

	__v4l2_ctrl_s_ctrl(imx678->link_freq, mode->linkfreq);

	dev_dbg(dev, "%s: linkfreq: %lld\n", __func__,
					imx678_link_freq_menu[mode->linkfreq]);

	imx678->line_time = (mode->hmax*IMX678_G_FACTOR) / (IMX678_XCLK_FREQ);
	dev_dbg(dev, "%s: line time: %lld\n", __func__, imx678->line_time);

	imx678->frame_length = mode->height * mode->frame_count + vblank;
	dev_dbg(dev, "%s: frame length: %d\n", __func__, imx678->frame_length);

	max_framerate = (IMX678_G_FACTOR * IMX678_M_FACTOR) /
				(imx678->frame_length * imx678->line_time);

	__v4l2_ctrl_modify_range(imx678->framerate, mode->min_fps,
				 max_framerate, 1, max_framerate);
	dev_dbg(dev, "%s: max framerate: %lld\n", __func__, max_framerate);

	imx678_update_blklvl_range(imx678);

	__v4l2_ctrl_s_ctrl(imx678->framerate, max_framerate);
}

static int imx678_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	struct imx678 *imx678 = to_imx678(sd);
	const struct imx678_frame_mode *mode = imx678->mode;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx678->mutex);

	if (fmt->pad == IMAGE_PAD) {
		fmt->format.code = imx678_get_format_code(imx678, fmt->format.code);
		imx678_update_image_pad_format(imx678, mode, fmt);

		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state,
								fmt->pad);
			*framefmt = fmt->format;
		} else if (imx678->fmt_code != fmt->format.code) {
			imx678->fmt_code = fmt->format.code;
			imx678_set_limits(imx678);
		}
	}

	mutex_unlock(&imx678->mutex);

	return 0;
}

static const struct v4l2_rect *
__imx678_get_pad_crop(struct imx678 *imx678,
			struct v4l2_subdev_state *sd_state,
			unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx678->mode->crop;
	}

	return NULL;
}

static int imx678_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx678 *imx678 = to_imx678(sd);

		mutex_lock(&imx678->mutex);
		sel->r = *__imx678_get_pad_crop(imx678, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx678->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX678_NATIVE_WIDTH;
		sel->r.height = IMX678_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX678_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX678_PIXEL_ARRAY_TOP;
		sel->r.width = IMX678_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX678_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int imx678_set_mode(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	const struct imx678_reg_list *reg_list;
	int ret;

	ret = imx678_write_table(imx678, mode_common_regs,
					ARRAY_SIZE(mode_common_regs));

	if (ret) {
		dev_err(dev, "%s failed to set common settings\n", __func__);
		return ret;
	}

	reg_list = &imx678->mode->reg_list;
	ret = imx678_write_table(imx678, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	reg_list = &imx678->mode->reg_list_format;
	ret = imx678_write_table(imx678, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(dev, "%s failed to set frame format\n", __func__);
		return ret;
	}

	ret = imx678_set_hmax_register(imx678);
	if (ret) {
		dev_err(dev, "%s failed to write hmax register\n", __func__);
		return ret;
	}

	ret = imx678_write_reg(imx678, DATARATE_SEL, 1, imx678->linkfreq->val);
	if (ret) {
		dev_err(dev, "%s failed to set data rate\n", __func__);
		return ret;
	}

	ret = imx678_configure_triggering_pins(imx678);
	if (ret) {
		dev_err(dev, "%s failed to configure triggering pins\n",
								__func__);
		return ret;
	}

	return ret;
}

static int imx678_start_streaming(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	int ret;

	ret = imx678_set_mode(imx678);
	if (ret) {
		dev_err(dev, "%s failed to set mode start stream\n", __func__);
		return ret;
	}

	ret = __v4l2_ctrl_handler_setup(imx678->sd.ctrl_handler);
	if (ret)
		return ret;

	ret = imx678_write_reg(imx678, STANDBY, 1, IMX678_MODE_STREAMING);

	if (ret) {
		dev_err(dev, "%s failed to set STANDBY start stream\n",
								__func__);
		return ret;
	}

	usleep_range(29000, 30000);

	if (imx678->operation_mode->val == MASTER_MODE)
		ret = imx678_write_reg(imx678, XMSTA, 1, 0x00);
	else
		ret = imx678_write_reg(imx678, XMSTA, 1, 0x01);

	if (ret) {
		dev_err(dev, "%s failed to set XMSTA start stream\n", __func__);
		return ret;
	}

	return ret;
}

static void imx678_stop_streaming(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	int ret;

	ret = imx678_write_reg(imx678, XMSTA, 1, 0x01);
	if (ret)
		dev_err(dev, "%s failed to set XMSTA stop stream\n", __func__);

	ret = imx678_write_reg(imx678, STANDBY, 1, IMX678_MODE_STANDBY);
	if (ret)
		dev_err(dev, "%s failed to set stream\n", __func__);

	usleep_range(imx678->frame_length * imx678->line_time / IMX678_K_FACTOR,
		imx678->frame_length * imx678->line_time / IMX678_K_FACTOR + 1000);

}

static int imx678_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx678 *imx678 = to_imx678(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx678->mutex);
	if (imx678->streaming == enable) {
		mutex_unlock(&imx678->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		ret = imx678_start_streaming(imx678);
		if (ret)
			goto err_rpm_put;
	} else {
		imx678_stop_streaming(imx678);
		pm_runtime_put(&client->dev);
	}

	imx678->streaming = enable;

	__v4l2_ctrl_grab(imx678->vflip, enable);
	__v4l2_ctrl_grab(imx678->hflip, enable);
	__v4l2_ctrl_grab(imx678->operation_mode, enable);
	__v4l2_ctrl_grab(imx678->sync_mode, enable);

	mutex_unlock(&imx678->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx678->mutex);

	return ret;
}

static int imx678_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);

	if (strcmp(imx678->gmsl, "gmsl")) {
		gpiod_set_value_cansleep(imx678->reset_gpio, 1);
		usleep_range(25000, 30000);
	}

	return 0;
}

static int imx678_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);
	int ret;

	ret = imx678_write_reg(imx678, XVS_XHS_DRV, 1, 0xF);
	if (ret)
		dev_err(dev, "%s: error setting XVS XHS to Hi-Z\n", __func__);

	mutex_lock(&imx678->mutex);
	if (strcmp(imx678->gmsl, "gmsl")) {
		gpiod_set_value_cansleep(imx678->reset_gpio, 0);
	}
	mutex_unlock(&imx678->mutex);

	return 0;
}

static int __maybe_unused imx678_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);

	if (imx678->streaming)
		imx678_stop_streaming(imx678);

	return 0;
}

static int __maybe_unused imx678_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);
	int ret;

	if (imx678->streaming) {
		ret = imx678_start_streaming(imx678);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx678_stop_streaming(imx678);
	imx678->streaming = 0;
	return ret;
}

static int imx678_communication_verify(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	int ret;
	u32 val;

	ret = imx678_read_reg(imx678, VMAX_LOW, 3, &val);

	if (ret) {
		dev_err(dev, "%s unable to communicate with sensor\n", __func__);
		return ret;
	}

	dev_info(dev, "Detected imx678 sensor\n");

	return 0;
}

static const struct v4l2_subdev_core_ops imx678_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx678_video_ops = {
	.s_stream = imx678_set_stream,
};

static const struct v4l2_subdev_pad_ops imx678_pad_ops = {
	.enum_mbus_code = imx678_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx678_set_pad_format,
	.get_selection = imx678_get_selection,
	.enum_frame_size = imx678_enum_frame_size,
};

static const struct v4l2_subdev_ops imx678_subdev_ops = {
	.core = &imx678_core_ops,
	.video = &imx678_video_ops,
	.pad = &imx678_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx678_internal_ops = {
	.open = imx678_open,
};

static struct v4l2_ctrl_config imx678_ctrl_framerate[] = {
	{
		.ops = &imx678_ctrl_ops,
		.id = V4L2_CID_FRAME_RATE,
		.name = "Frame rate",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 0xFFFF,
		.def = 0xFFFF,
		.step = 1,
	},
};

static struct v4l2_ctrl_config imx678_ctrl_operation_mode[] = {
	{
		.ops = &imx678_ctrl_ops,
		.id = V4L2_CID_OPERATION_MODE,
		.name = "Operation mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = MASTER_MODE,
		.def = MASTER_MODE,
		.max = SLAVE_MODE,
		.qmenu = imx678_operation_mode_menu,
	},
};

static struct v4l2_ctrl_config imx678_ctrl_sync_mode[] = {
	{
		.ops = &imx678_ctrl_ops,
		.id = V4L2_CID_SYNC_MODE,
		.name = "Sync mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = NO_SYNC,
		.def = NO_SYNC,
		.max = EXTERNAL_SYNC,
		.qmenu = imx678_sync_mode_menu,
	},
};

static int imx678_init_controls(struct imx678 *imx678)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct device *dev = &client->dev;
	struct v4l2_fwnode_device_properties props;
	int ret;

	ctrl_hdlr = &imx678->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&imx678->mutex);
	ctrl_hdlr->lock = &imx678->mutex;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx678_ctrl_ops,
					V4L2_CID_FRAME_MODE,
					ARRAY_SIZE(imx678_frame_mode_menu) - 1,
					0, 0, imx678_hdr_mode_menu);

	imx678->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
						V4L2_CID_PIXEL_RATE, 0, 0, 1, 0);
	if (imx678->pixel_rate)
		imx678->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx678->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx678_ctrl_ops,
					V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(imx678_link_freq_menu) - 1, 0,
					imx678_link_freq_menu);
	if (imx678->link_freq)
		imx678->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx678->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
					V4L2_CID_VBLANK, 0, 0, 1, 0);

	imx678->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
					V4L2_CID_HBLANK, 0, 0, 1, 0);

	if (imx678->hblank)
		imx678->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx678->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
					V4L2_CID_EXPOSURE,
					IMX678_MIN_INTEGRATION_LINES,
					0xFF, 1, 0xFF);

	imx678->framerate = v4l2_ctrl_new_custom(ctrl_hdlr,
					imx678_ctrl_framerate, NULL);

	imx678->operation_mode = v4l2_ctrl_new_custom(ctrl_hdlr,
					imx678_ctrl_operation_mode, NULL);

	imx678->sync_mode = v4l2_ctrl_new_custom(ctrl_hdlr,
					imx678_ctrl_sync_mode, NULL);

	imx678->blklvl = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
					V4L2_CID_BLACK_LEVEL,
					IMX678_BLACK_LEVEL_MIN, 0xFF,
					IMX678_BLACK_LEVEL_STEP, 0xFF);

	imx678->gain = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
					IMX678_ANA_GAIN_MIN,
					IMX678_ANA_GAIN_MAX,
					IMX678_ANA_GAIN_STEP,
					IMX678_ANA_GAIN_DEFAULT);

	imx678->exposure_short = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
					V4L2_CID_EXPOSURE_SHORT,
					IMX678_MIN_INTEGRATION_LINES,
					0xFF, 1, IMX678_MIN_INTEGRATION_LINES);

	imx678->gain_short = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops, V4L2_CID_ANALOGUE_GAIN_SHORT,
					IMX678_ANA_GAIN_MIN,
					IMX678_ANA_GAIN_MAX,
					IMX678_ANA_GAIN_STEP,
					IMX678_ANA_GAIN_DEFAULT);

	imx678->exponential_gain = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
		V4L2_CID_EXPONENTIAL_GAIN,
		IMX678_EXP_GAIN_MIN,
		IMX678_EXP_GAIN_MAX,
		IMX678_EXP_GAIN_STEP,
		IMX678_EXP_GAIN_DEFAULT);

	imx678->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx678->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx678_ctrl_ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(imx678_test_pattern_menu) - 1,
					0, 0, imx678_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(dev, "%s control init failed (%d)\n", __func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx678_ctrl_ops, &props);
	if (ret)
		goto error;

	imx678->sd.ctrl_handler = ctrl_hdlr;

	mutex_lock(&imx678->mutex);

	imx678_set_limits(imx678);

	mutex_unlock(&imx678->mutex);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx678->mutex);

	return ret;
}

static void imx678_free_controls(struct imx678 *imx678)
{
	v4l2_ctrl_handler_free(imx678->sd.ctrl_handler);
	mutex_destroy(&imx678->mutex);
}

static int imx678_check_hwcfg(struct device *dev, struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	const char *gmsl;
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(dev, "only 4 data lanes are currently supported\n");
		goto error_out;
	}

	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != ARRAY_SIZE(imx678_link_freq_menu)) {
		dev_err(dev, "Link frequency missing in dtree\n");
		goto error_out;
	}

	for (int i = 0; i < ARRAY_SIZE(imx678_link_freq_menu); i++) {
		if (ep_cfg.link_frequencies[i] != imx678_link_freq_menu[i]) {
			dev_err(dev, "no supported link freq found\n");
			goto error_out;
		}
	}

	ret = of_property_read_string(node, "gmsl", &gmsl);
	if (ret) {
		dev_warn(dev, "initializing mipi...\n");
		imx678->gmsl = "mipi";
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static const struct of_device_id imx678_dt_ids[] = {
	{ .compatible = "sony,imx678" },
	{	}
};

static int imx678_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx678 *imx678;
	const struct of_device_id *match;
	int ret;

	imx678 = devm_kzalloc(&client->dev, sizeof(*imx678), GFP_KERNEL);
	if (!imx678)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx678->sd, client, &imx678_subdev_ops);

	match = of_match_device(imx678_dt_ids, dev);
	if (!match)
		return -ENODEV;

	if (imx678_check_hwcfg(dev, client))
		return -EINVAL;

	if (strcmp(imx678->gmsl, "gmsl")) {
		imx678->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
		if (IS_ERR(imx678->reset_gpio)) {
			dev_err(dev, "cannot get reset gpio\n");
			return PTR_ERR(imx678->reset_gpio);
		}
	}

	ret = imx678_power_on(dev);
	if (ret)
		return ret;

	ret = imx678_communication_verify(imx678);
	if (ret)
		goto error_power_off;


	imx678->xmaster = devm_gpiod_get(dev, "xmaster", GPIOD_OUT_HIGH);
	if (IS_ERR(imx678->xmaster)) {
		dev_err(dev, "cannot get xmaster gpio\n");
		return PTR_ERR(imx678->xmaster);
	}

	imx678->mode = &frame_mode_configs[0];
	imx678->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = imx678_init_controls(imx678);
	if (ret)
		goto error_power_off;

	imx678->sd.internal_ops = &imx678_internal_ops;
	imx678->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	imx678->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	imx678->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx678->sd.entity, NUM_PADS, imx678->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx678->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx678->sd.entity);

error_handler_free:
	imx678_free_controls(imx678);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	imx678_power_off(&client->dev);

	return ret;
}

static void imx678_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx678_free_controls(imx678);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx678_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

MODULE_DEVICE_TABLE(of, imx678_dt_ids);

static const struct dev_pm_ops imx678_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx678_suspend, imx678_resume)
	SET_RUNTIME_PM_OPS(imx678_power_off, imx678_power_on, NULL)
};

static struct i2c_driver imx678_i2c_driver = {
	.driver = {
		.name = "imx678",
		.of_match_table	= imx678_dt_ids,
		.pm = &imx678_pm_ops,
	},
	.probe = imx678_probe,
	.remove = imx678_remove,
};

module_i2c_driver(imx678_i2c_driver);

MODULE_AUTHOR("FRAMOS GmbH, Brady Law");
MODULE_DESCRIPTION("Sony IMX678 sensor driver");
MODULE_LICENSE("GPL v2");
