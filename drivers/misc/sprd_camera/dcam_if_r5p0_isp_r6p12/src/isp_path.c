/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>

#include "isp_path.h"
#include "isp_buf.h"
#include "cam_common.h"
#include "cam_buf.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_PATH: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define SHRINK_Y_UP_TH                 235
#define SHRINK_Y_DN_TH                 16
#define SHRINK_UV_UP_TH                240
#define SHRINK_UV_DN_TH                16
#define SHRINK_Y_OFFSET                16
#define SHRINK_Y_RANGE                 2
#define SHRINK_C_OFFSET                16
#define SHRINK_C_RANGE                 6

int isp_path_cfg(struct isp_path_desc *path)
{
	int rtn = 0;

	if (!path) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	path->src.w = path->coeff_latest.param.in_size.w;
	path->src.h = path->coeff_latest.param.in_size.h;
	path->trim0_info.start_x = path->coeff_latest.param.in_rect.x;
	path->trim0_info.start_y = path->coeff_latest.param.in_rect.y;
	path->trim0_info.size_x = path->coeff_latest.param.in_rect.w;
	path->trim0_info.size_y = path->coeff_latest.param.in_rect.h;
	path->dst.w = path->coeff_latest.param.out_size.w;
	path->dst.h = path->coeff_latest.param.out_size.h;
	path->trim1_info.start_x = 0;
	path->trim1_info.start_y = 0;
	path->trim1_info.size_x = path->dst.w;
	path->trim1_info.size_y = path->dst.h;
	path->path_sel = 1;

	pr_debug("path offline src %d %d dst %d %d trim0 %d %d %d %d trim1 %d %d %d %d\n",
		path->src.w, path->src.h,
		path->dst.w, path->dst.h,
		path->trim0_info.start_x,
		path->trim0_info.start_y,
		path->trim0_info.size_x,
		path->trim0_info.size_y,
		path->trim1_info.start_x,
		path->trim1_info.start_y,
		path->trim1_info.size_x,
		path->trim1_info.size_y);

	return rtn;
}

static enum isp_store_format isp_store_format(enum dcam_fmt in_format)
{
	enum isp_store_format format = ISP_STORE_FORMAT_MAX;

	switch (in_format) {
	case DCAM_YUV422:
		format = ISP_STORE_YUV422_2FRAME;
		break;
	case DCAM_YUV420:
		format = ISP_STORE_YVU420_2FRAME;
		break;
	case DCAM_YVU420:
		format = ISP_STORE_YUV420_2FRAME;
		break;
	case DCAM_YUV420_3FRAME:
		format = ISP_STORE_YUV420_3FRAME;
		break;
	case DCAM_RAWRGB:
		format = ISP_STORE_RAW10;
		break;
	case DCAM_RGB888:
		format = ISP_STORE_FULL_RGB8;
		break;
	default:
		format = ISP_STORE_FORMAT_MAX;
		pr_info("error, format not support!");
		break;
	}
	return format;
}

#if 0
static enum isp_fetchYUV_format isp_fetchyuv_format(enum dcam_fmt in_format)
{
	enum isp_fetchYUV_format format = ISP_FETCHYUV_FORMAT_MAX;

	switch (in_format) {
	case DCAM_YUV422:
		format = ISP_FETCHYUV_YUV422_2FRAME;
		break;
	case DCAM_YUV420:
		format = ISP_FETCHYUV_YVU420_2FRAME;
		break;
	case DCAM_YVU420:
		format = ISP_FETCHYUV_YUV420_2FRAME;
		break;
	default:
		format = ISP_FETCHYUV_FORMAT_MAX;
		pr_info("fail to get valid format\n!");
		break;
	}
	return format;
}

static void get_fetchyuv_pitch(struct slice_pitch *pitch_ptr,
	enum isp_fetchYUV_format format, uint32_t width)
{
	switch (format) {
	case ISP_FETCHYUV_YUV422_2FRAME:
	case ISP_FETCHYUV_YVU422_2FRAME:
	case ISP_FETCHYUV_YUV420_2FRAME:
	case ISP_FETCHYUV_YVU420_2FRAME:
		pitch_ptr->chn0 = width;
		pitch_ptr->chn1 = width;
		break;
	default:
		break;
	}
}
#endif

static void get_store_pitch(struct slice_pitch *pitch_ptr,
	enum isp_store_format format, uint32_t width)
{
	switch (format) {
	case ISP_STORE_YUV422_3FRAME:
	case ISP_STORE_YUV420_3FRAME:
		pitch_ptr->chn0 = width;
		pitch_ptr->chn1 = width >> 1;
		pitch_ptr->chn2 = width >> 1;
		break;
	case ISP_STORE_YUV422_2FRAME:
	case ISP_STORE_YVU422_2FRAME:
	case ISP_STORE_YUV420_2FRAME:
	case ISP_STORE_YVU420_2FRAME:
		pitch_ptr->chn0 = width;
		pitch_ptr->chn1 = width;
		break;
	case ISP_STORE_UYVY:
		pitch_ptr->chn0 = width << 1;
		break;
	case ISP_STORE_RAW10:
		pitch_ptr->chn0 = width << 1;
		break;
	case ISP_STORE_FULL_RGB8:
		pitch_ptr->chn0 = width * 4;
		break;
	default:
		break;
	}

}

static int isp_get_store_param(struct isp_path_desc *path)
{
	struct isp_store_info *store_info = NULL;

	if (!path) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	store_info = &path->store_info;
	store_info->bypass = 0;
	store_info->endian = path->data_endian.uv_endian;
	store_info->speed_2x = 1;
	store_info->mirror_en = 0;
	store_info->color_format = isp_store_format(path->output_format);
	store_info->max_len_sel = 0;
	store_info->shadow_clr_sel = 1;
	store_info->shadow_clr = 1;
	store_info->store_res = 1;
	store_info->rd_ctrl = 0;

	store_info->size.w = path->trim1_info.size_x;
	store_info->size.h = path->trim1_info.size_y;

	store_info->border.up_border = 0;
	store_info->border.down_border = 0;
	store_info->border.left_border = 0;
	store_info->border.right_border = 0;

	get_store_pitch((void *)&store_info->pitch,
			store_info->color_format, store_info->size.w);

	return 0;
}

static int isp_path_store_cfg(struct isp_path_desc *pre,
			struct isp_path_desc *vid,
			struct isp_path_desc *cap)
{
	int rtn = 0;

	if (!pre || !vid || !cap) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (pre->valid) {
		rtn = isp_get_store_param(pre);
		if (rtn) {
			pr_err("fail to get pre store param\n");
			return rtn;
		}
	}

	if (vid->valid) {
		rtn = isp_get_store_param(vid);
		if (rtn) {
			pr_err("fail to get vid store param error\n");
			return rtn;
		}
	}

	if (cap->valid) {
		rtn = isp_get_store_param(cap);
		if (rtn) {
			pr_err("fail to get cap store param error\n");
			return rtn;
		}
	}

	return 0;
}

int isp_start_pre_proc(struct isp_path_desc *pre,
		struct isp_path_desc *vid,
		struct isp_path_desc *cap)
{
	int rtn = 0;

	if (!pre || !vid || !cap) {
		pr_err("input ptr is NULL!\n");
		return -EFAULT;
	}

	pr_info("pre_valid %d, vid_valid %d, cap_valid %d\n",
		pre->valid, vid->valid, cap->valid);
	if (pre->valid)
		isp_path_cfg(pre);

	if (vid->valid)
		isp_path_cfg(vid);

	if (cap->valid)
		isp_path_cfg(cap);

	rtn = isp_path_store_cfg(pre, vid, cap);
	if (rtn) {
		pr_err("fail to get store param\n");
		return rtn;
	}

	return rtn;
}

static void isp_set_channel_info(uint32_t idx, struct isp_path_desc *path,
		enum isp_path_index path_index)
{
	uint32_t mipi_byte_info = 0;
	uint32_t mipi_word_info = 0;
	uint32_t start_col = 0;
	uint32_t end_col = path->src.w - 1;
	uint32_t mipi_word_num_start[16] = {
		0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5};
	uint32_t mipi_word_num_end[16] = {
		0, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5};

	uint32_t fetchraw_pitch = 0;
	uint32_t mod16_pixel = path->src.w & 0xF;
	uint32_t mod16_bytes = (mod16_pixel + 3) / 4 * 5;
	uint32_t mod16_words = (mod16_bytes + 3) / 4;

	mipi_byte_info = start_col & 0xF;
	mipi_word_info = ((end_col + 1) >> 4) * 5
				+ mipi_word_num_end[(end_col + 1) & 0xF]
				- ((start_col + 1) >> 4) * 5
				- mipi_word_num_start[(start_col + 1) & 0xF]
				+ 1;

	fetchraw_pitch = path->src.w / 16 * 20 + mod16_words * 4;
	pr_debug("path src %d, pitch %d, byte %d, word %d\n",
		path->src.w, fetchraw_pitch, mipi_byte_info, mipi_word_info);
	ISP_REG_WR(idx, ISP_FETCH_PARAM,  0x8 << 4);  /*csi raw10*/
	ISP_REG_WR(idx, ISP_FETCH_MEM_SLICE_SIZE,
		path->src.w | (path->src.h << 16));
	ISP_REG_WR(idx, ISP_FETCH_SLICE_Y_PITCH, fetchraw_pitch);
	ISP_REG_WR(idx, ISP_FETCH_MIPI_INFO,
		mipi_word_info | (mipi_byte_info << 16));

	if (ISP_PATH_IDX_CAP & path_index)
		ISP_REG_WR(idx, ISP_FETCH_LINE_DLY_CTRL, 0x800);
	else
		ISP_REG_WR(idx, ISP_FETCH_LINE_DLY_CTRL, 0x200);
	ISP_REG_WR(idx, ISP_DISPATCH_DLY,  0x1D3C);
	ISP_REG_WR(idx, ISP_DISPATCH_HW_CTRL_CH0,  0x080004);
	ISP_REG_WR(idx, ISP_DISPATCH_LINE_DLY1,  0x280001C);
	ISP_REG_WR(idx, ISP_DISPATCH_PIPE_BUF_CTRL_CH0,  0x64043C);
	ISP_REG_WR(idx, ISP_DISPATCH_CH0_SIZE,
		path->src.w | (path->src.h << 16));
	ISP_REG_WR(idx, ISP_DISPATCH_CH0_BAYER, 2);

}

static int isp_set_store(uint32_t idx, void *input_info, uint32_t addr)
{
	int ret = 0;
	uint32_t val = 0;
	struct isp_store_info *store_info =
		(struct isp_store_info *)input_info;

	pr_debug("isp set store in.  bypass %d\n", store_info->bypass);
	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		BIT_0, store_info->bypass);
	if (store_info->bypass)
		return 0;

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		BIT_1, (store_info->max_len_sel << 1));

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		BIT_2, (store_info->speed_2x << 2));

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		BIT_3, (store_info->mirror_en << 3));

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		0xF0, (store_info->color_format << 4));

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM,
		0x300, (store_info->endian << 8));

	val = ((store_info->size.h & 0xFFFF) << 16) |
		(store_info->size.w & 0xFFFF);
	ISP_REG_WR(idx, addr + ISP_STORE_SLICE_SIZE, val);

	val = ((store_info->border.right_border & 0xFF) << 24) |
		((store_info->border.left_border & 0xFF) << 16) |
		((store_info->border.down_border & 0xFF) << 8) |
		(store_info->border.up_border & 0xFF);
	ISP_REG_WR(idx, addr + ISP_STORE_BORDER, val);
	ISP_REG_WR(idx, addr + ISP_STORE_Y_PITCH, store_info->pitch.chn0);
	ISP_REG_WR(idx, addr + ISP_STORE_U_PITCH, store_info->pitch.chn1);
	ISP_REG_WR(idx, addr + ISP_STORE_V_PITCH, store_info->pitch.chn2);

	pr_debug("set_store size %d %d border %d %d %d %d\n",
		store_info->size.w, store_info->size.h,
		store_info->border.left_border,
		store_info->border.up_border,
		store_info->border.right_border,
		store_info->border.down_border);

	ISP_REG_MWR(idx, addr + ISP_STORE_READ_CTRL,
		0x3, store_info->rd_ctrl);
	ISP_REG_MWR(idx, addr + ISP_STORE_READ_CTRL,
		0xFFFFFFFC, store_info->store_res << 2);

	ISP_REG_MWR(idx, addr + ISP_STORE_SHADOW_CLR_SEL,
		BIT_1, store_info->shadow_clr_sel << 1);
	ISP_REG_MWR(idx, addr + ISP_STORE_SHADOW_CLR,
		BIT_0, store_info->shadow_clr);

	return ret;
}

static void cpp_print_coeff(enum isp_id idx,
			unsigned long h_coeff_addr,
			unsigned long v_chroma_coeff_addr,
			unsigned long v_coeff_addr)
{
#ifdef SCALE_DRV_DEBUG
	int i = 0, j = 0;
	unsigned long addr = 0;

	pr_info("SCAL: h_coeff_addr\n");
	for (i = 0; i < 16; i++) {
		pr_info("0x%lx: ", h_coeff_addr + 4 * (4 * i));
		for (j = 0; j < 4; j++)
			pr_info("0x%x ",
			ISP_REG_RD(idx, h_coeff_addr + 4 * (4 * i + j)));
		pr_info("\n");
	}

	pr_info("SCAL: v_chroma_coeff_addr\n");
	for (i = 0; i < 32; i++) {
		pr_info("0x%lx: ", v_chroma_coeff_addr + 4 * (4 * i));
		for (j = 0; j < 4; j++)
			pr_info("0x%x ",
				ISP_REG_RD(idx,
				v_chroma_coeff_addr + 4 * (4 * i + j)));
		pr_info("\n");
	}

	pr_info("SCAL: v_coeff_addr\n");
	for (addr = v_coeff_addr; addr <= (v_coeff_addr + 0x6FC); addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(idx, addr), ISP_REG_RD(idx, addr + 4),
			ISP_REG_RD(idx, addr + 8), ISP_REG_RD(idx, addr + 12));
	}
#endif
}

static int isp_set_sc_coeff_info(uint32_t idx, uint32_t addr,
	uint32_t *coeff_buf)
{
	int i = 0, rtn = 0;
	uint32_t h_coeff_addr = 0;
	uint32_t v_coeff_addr = 0;
	uint32_t h_chroma_coeff_addr = 0;
	uint32_t v_chroma_coeff_addr = 0;
	uint32_t *h_coeff = NULL;
	uint32_t *v_coeff = NULL;
	uint32_t *v_chroma_coeff = NULL;
	uint32_t reg_val = 0, work_mode = 0, isp_id = 0;

	if (coeff_buf == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	work_mode = ISP_GET_MODE_ID(idx);
	isp_id = ISP_GET_ISP_ID(idx);
	reg_val = (ISP_REG_RD(isp_id, addr +
			ISP_SCALER_CFG) & BIT_30) >> 30;

	pr_debug(" set sc coeff work_mode %d, reg_val 0x%x\n",
		work_mode, reg_val);
	h_coeff = coeff_buf;
	v_coeff = coeff_buf + (ISP_SC_COEFF_COEF_SIZE / 4);
	v_chroma_coeff = v_coeff + (ISP_SC_COEFF_COEF_SIZE / 4);
	if (work_mode == ISP_CFG_MODE) {
		if  (addr == ISP_SCALER_PRE_CAP_BASE) {
			h_coeff_addr =  ISP_SCALER_PRE_LUMA_HCOEFF_BUF0;
			h_chroma_coeff_addr = ISP_SCALER_PRE_CHROMA_HCOEFF_BUF0;
			v_coeff_addr =  ISP_SCALER_PRE_LUMA_VCOEFF_BUF0;
			v_chroma_coeff_addr = ISP_SCALER_PRE_CHROMA_VCOEFF_BUF0;
		} else if   (addr == ISP_SCALER_VID_BASE) {
			h_coeff_addr =  ISP_SCALER_VID_LUMA_HCOEFF_BUF0;
			h_chroma_coeff_addr = ISP_SCALER_VID_CHROMA_HCOEFF_BUF0;
			v_coeff_addr = ISP_SCALER_VID_LUMA_VCOEFF_BUF0;
			v_chroma_coeff_addr = ISP_SCALER_VID_CHROMA_VCOEFF_BUF0;
		}
	} else if  (work_mode == ISP_AP_MODE) {
		if  (addr == ISP_SCALER_PRE_CAP_BASE) {
			if  (reg_val == 1) {
				h_coeff_addr = ISP_SCALER_PRE_LUMA_HCOEFF_BUF0;
				h_chroma_coeff_addr =
					ISP_SCALER_PRE_CHROMA_HCOEFF_BUF0;
				v_coeff_addr =  ISP_SCALER_PRE_LUMA_VCOEFF_BUF0;
				v_chroma_coeff_addr =
					ISP_SCALER_PRE_CHROMA_VCOEFF_BUF0;
				ISP_REG_MWR(idx, addr+ISP_SCALER_CFG,
					BIT_30, 0 << 30);
			} else {
				h_coeff_addr = ISP_SCALER_PRE_LUMA_HCOEFF_BUF1;
				h_chroma_coeff_addr =
					ISP_SCALER_PRE_CHROMA_HCOEFF_BUF1;
				v_coeff_addr = ISP_SCALER_PRE_LUMA_VCOEFF_BUF1;
				v_chroma_coeff_addr =
					ISP_SCALER_PRE_CHROMA_VCOEFF_BUF1;
				ISP_REG_MWR(idx, addr+ISP_SCALER_CFG,
					BIT_30, 1 << 30);
			}
		} else if  (addr == ISP_SCALER_VID_BASE) {
			if (reg_val == 1) {
				h_coeff_addr =  ISP_SCALER_VID_LUMA_HCOEFF_BUF0;
				h_chroma_coeff_addr =
					ISP_SCALER_VID_CHROMA_HCOEFF_BUF0;
				v_coeff_addr =  ISP_SCALER_VID_LUMA_VCOEFF_BUF0;
				v_chroma_coeff_addr =
					ISP_SCALER_VID_CHROMA_VCOEFF_BUF0;
				ISP_REG_MWR(idx, addr+ISP_SCALER_CFG,
					BIT_30, 0 << 30);
			} else {
				h_coeff_addr =  ISP_SCALER_VID_LUMA_HCOEFF_BUF1;
				h_chroma_coeff_addr =
					ISP_SCALER_VID_CHROMA_HCOEFF_BUF1;
				v_coeff_addr =  ISP_SCALER_VID_LUMA_VCOEFF_BUF1;
				v_chroma_coeff_addr =
					ISP_SCALER_VID_CHROMA_VCOEFF_BUF1;
				ISP_REG_MWR(idx, addr+ISP_SCALER_CFG,
					BIT_30, 1 << 30);
			}
		}
	}
#if 0
	if (addr == ISP_SCALER_PRE_CAP_BASE)
		ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_30, 1 << 30);
	else
		reg_val = (ISP_REG_RD(idx, addr +
			ISP_SCALER_CFG) & BIT_30) >> 30;
#endif
	for (i = 0; i < ISP_SC_COEFF_H_NUM; i++) {
		ISP_REG_WR(idx, h_coeff_addr, *h_coeff);
		h_coeff_addr += 4;
		h_coeff++;
	}

	for (i = 0; i < ISP_SC_COEFF_H_CHROMA_NUM; i++) {
		ISP_REG_WR(idx, h_chroma_coeff_addr, *h_coeff);
		h_chroma_coeff_addr += 4;
		h_coeff++;
	}

	for (i = 0; i < ISP_SC_COEFF_V_NUM; i++) {
		ISP_REG_WR(idx, v_coeff_addr, *v_coeff);
		v_coeff_addr += 4;
		v_coeff++;
	}

	for (i = 0; i < ISP_SC_COEFF_V_CHROMA_NUM; i++) {
		ISP_REG_WR(idx, v_chroma_coeff_addr, *v_chroma_coeff);
		v_chroma_coeff_addr += 4;
		v_chroma_coeff++;
	}

	h_coeff_addr = ISP_SCALER_PRE_LUMA_HCOEFF_BUF0;
	h_chroma_coeff_addr =  ISP_SCALER_PRE_CHROMA_HCOEFF_BUF0;
	v_coeff_addr = ISP_SCALER_PRE_LUMA_VCOEFF_BUF0;
	v_chroma_coeff_addr =  ISP_SCALER_PRE_CHROMA_VCOEFF_BUF0;
	cpp_print_coeff(isp_id, h_coeff_addr,
		v_chroma_coeff_addr, v_coeff_addr);
#if 0
	if (addr == ISP_SCALER_PRE_CAP_BASE)
		ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_30, 0 << 30);
	else
		ISP_REG_MWR(idx, addr + ISP_SCALER_CFG,
			    BIT_30, (~reg_val) << 30);
#endif


	return rtn;
}


static void isp_set_shrink_info(void *input_info,
		uint32_t idx, uint32_t addr_base)
{
	unsigned long addr = 0;
	uint32_t reg_val = 0;
	struct isp_regular_info *regular_info = NULL;

	if (!input_info) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	regular_info = (struct isp_regular_info *)input_info;
	pr_debug("regular_mode %d\n", regular_info->regular_mode);
	addr = ISP_SCALER_CFG + addr_base;
	ISP_REG_MWR(idx, addr, (BIT_25 | BIT_26),
		regular_info->regular_mode << 25);

	/*TBD
	* the value need to update.
	*/
	if (regular_info->regular_mode == DCAM_REGULAR_SHRINK) {
		regular_info->shrink_y_up_th = SHRINK_Y_UP_TH;
		regular_info->shrink_y_dn_th = SHRINK_Y_DN_TH;
		regular_info->shrink_uv_up_th = SHRINK_UV_UP_TH;
		regular_info->shrink_uv_dn_th = SHRINK_UV_DN_TH;
		addr = ISP_SCALER_SHRINK_CFG + addr_base;
		reg_val = ((regular_info->shrink_uv_dn_th & 0xFF) << 24) |
			((regular_info->shrink_uv_up_th & 0xFF) << 16);
		reg_val |= ((regular_info->shrink_y_dn_th  & 0xFF) << 8) |
			((regular_info->shrink_y_up_th & 0xFF));
		ISP_REG_WR(idx, addr, reg_val);

		regular_info->shrink_y_offset = SHRINK_Y_OFFSET;
		regular_info->shrink_y_range = SHRINK_Y_RANGE;
		regular_info->shrink_c_offset = SHRINK_C_OFFSET;
		regular_info->shrink_c_range = SHRINK_C_RANGE;
		addr = ISP_SCALER_REGULAR_CFG + addr_base;
		reg_val = ((regular_info->shrink_c_range & 0xF) << 24) |
			((regular_info->shrink_c_offset & 0x1F) << 16);
		reg_val |= ((regular_info->shrink_y_range & 0xF) << 8) |
			(regular_info->shrink_y_offset & 0x1F);
		ISP_REG_WR(idx, addr, reg_val);
	} else if (regular_info->regular_mode == DCAM_REGULAR_CUT) {
		addr = ISP_SCALER_SHRINK_CFG + addr_base;
		reg_val = ((regular_info->shrink_uv_dn_th & 0xFF) << 24) |
			((regular_info->shrink_uv_up_th & 0xFF) << 16);
		reg_val |= ((regular_info->shrink_y_dn_th  & 0xFF) << 8) |
			((regular_info->shrink_y_up_th & 0xFF));
		ISP_REG_WR(idx, addr, reg_val);
	} else if (regular_info->regular_mode == DCAM_REGULAR_EFFECT) {
		addr = ISP_SCALER_EFFECT_CFG + addr_base;
		reg_val = ((regular_info->effect_v_th & 0xFF) << 16) |
				((regular_info->effect_u_th & 0xFF) << 8);
		reg_val |= (regular_info->effect_y_th & 0xFF);
		ISP_REG_WR(idx, addr, reg_val);
	} else
		pr_debug("regular_mode %d\n", regular_info->regular_mode);
}

static void isp_set_scaler_info(struct isp_path_desc *path,
	uint32_t idx, uint32_t addr_base)
{
	uint32_t reg_val = 0;
	struct isp_scaler_info *scalerInfo = NULL;

	scalerInfo = &path->scaler_info;

	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, BIT_20,
			scalerInfo->scaler_bypass << 20);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, 0xF0000,
			scalerInfo->scaler_y_ver_tap << 16);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, 0xF800,
			scalerInfo->scaler_uv_ver_tap << 11);

	reg_val = ((scalerInfo->scaler_ip_int & 0xF) << 16) |
			(scalerInfo->scaler_ip_rmd & 0x3FFF);
	ISP_REG_WR(idx, addr_base+ISP_SCALER_IP, reg_val);
	reg_val = ((scalerInfo->scaler_cip_int & 0xF) << 16) |
			(scalerInfo->scaler_cip_rmd & 0x3FFF);
	ISP_REG_WR(idx, addr_base+ISP_SCALER_CIP, reg_val);
	reg_val = ((scalerInfo->scaler_factor_in & 0x3FFF) << 16) |
			(scalerInfo->scaler_factor_out & 0x3FFF);
	ISP_REG_WR(idx, addr_base+ISP_SCALER_FACTOR, reg_val);

	reg_val = ((scalerInfo->scaler_ver_ip_int & 0xF) << 16) |
			 (scalerInfo->scaler_ver_ip_rmd & 0x3FFF);
	ISP_REG_WR(idx, addr_base+ISP_SCALER_VER_IP, reg_val);
	reg_val = ((scalerInfo->scaler_ver_cip_int & 0xF) << 16) |
			 (scalerInfo->scaler_ver_cip_rmd & 0x3FFF);
	ISP_REG_WR(idx, addr_base+ISP_SCALER_VER_CIP, reg_val);
	reg_val = ((scalerInfo->scaler_ver_factor_in & 0x3FFF) << 16) |
			 (scalerInfo->scaler_ver_factor_out & 0x3FFF);
	ISP_REG_WR(idx, addr_base+ISP_SCALER_VER_FACTOR, reg_val);

	pr_debug("set_scale_info in %d %d out %d %d\n",
		scalerInfo->scaler_factor_in,
		scalerInfo->scaler_ver_factor_in,
		scalerInfo->scaler_factor_out,
		scalerInfo->scaler_ver_factor_out);

	isp_set_sc_coeff_info(idx, addr_base, path->coeff_latest.coeff_buf);
}

static void isp_set_deci_info(void *input_info, uint32_t idx,
			uint32_t addr_base)
{
	struct isp_deci_info *deciInfo = (struct isp_deci_info *)input_info;

	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, BIT_2,
		deciInfo->deci_x_eb << 2);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, (BIT_0 | BIT_1),
		deciInfo->deci_x);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, BIT_5,
		deciInfo->deci_y_eb << 5);
	ISP_REG_MWR(idx, addr_base+ISP_SCALER_CFG, (BIT_3 | BIT_4),
		deciInfo->deci_y << 3);
}

void isp_path_set_scl(uint32_t idx, struct isp_path_desc *path,
	uint32_t addr)
{
	uint32_t reg_val = 0;

	if (!path) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	/*config path_eb*/
	ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_31, 1 << 31);
	ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_29, 0 << 29);
	ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_9, ~path->valid << 9);
	ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_8, 0 << 8);
	ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_10, path->uv_sync_v << 10);
	/*config frame deci*/
	if (path->frm_deci <= 3) {
		ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, (BIT_23 | BIT_24),
			path->frm_deci << 23);
	} else {
		pr_info("invalid frame_deci %d\n", path->frm_deci);
	}
	/*config output format*/
	if (path->output_format == DCAM_YUV422)
		path->odata_mode = 0x00;
	else if (path->output_format == DCAM_YUV420)
		path->odata_mode = 0x01;
	else if (path->output_format == DCAM_YUV420_3FRAME)
		path->odata_mode = 0x00;
	else
		pr_info("fail to get valid output format %d\n",
			path->output_format);

	ISP_REG_MWR(idx, addr+ISP_SCALER_CFG, BIT_6 | BIT_7,
			path->odata_mode << 6);

	/*config X/Y deci */
	isp_set_deci_info((void *)&path->deci_info, idx, addr);

	/*src size*/
	reg_val = ((path->src.h & 0x3FFF) << 16) | (path->src.w & 0x3FFF);
	ISP_REG_WR(idx, addr+ISP_SCALER_SRC_SIZE, reg_val);

	/* trim0 */
	reg_val = ((path->trim0_info.start_y & 0x3FFF) << 16) |
			(path->trim0_info.start_x & 0x3FFF);
	ISP_REG_WR(idx, addr+ISP_SCALER_TRIM0_START, reg_val);
	reg_val = ((path->trim0_info.size_y & 0x3FFF) << 16) |
			(path->trim0_info.size_x & 0x3FFF);
	ISP_REG_WR(idx, addr+ISP_SCALER_TRIM0_SIZE, reg_val);

	/* trim1 */
	reg_val = ((path->trim1_info.start_y & 0x3FFF) << 16) |
			(path->trim1_info.start_x & 0x3FFF);
	ISP_REG_WR(idx, addr+ISP_SCALER_TRIM1_START, reg_val);
	reg_val = ((path->trim1_info.size_y & 0x3FFF) << 16) |
			(path->trim1_info.size_x & 0x3FFF);
	ISP_REG_WR(idx, addr+ISP_SCALER_TRIM1_SIZE, reg_val);

	/* des size */
	reg_val = ((path->dst.h & 0x3FFF) << 16) | (path->dst.w & 0x3FFF);
	ISP_REG_WR(idx, addr+ISP_SCALER_DES_SIZE, reg_val);

	pr_debug("scl base addr %x\n", addr);
	pr_debug("set_scl src %d %d dst %d %d trim0 %d %d %d %d trim1 %d %d %d %d\n",
		path->src.w, path->src.h,
		path->dst.w, path->dst.h,
		path->trim0_info.start_x,
		path->trim0_info.start_y,
		path->trim0_info.size_x,
		path->trim0_info.size_y,
		path->trim1_info.start_x,
		path->trim1_info.start_y,
		path->trim1_info.size_x,
		path->trim1_info.size_y);
	/* scaler info*/
	isp_set_scaler_info(path, idx, addr);

	/* scaler_vid shrink */
	if (addr == ISP_SCALER_VID_BASE)
		isp_set_shrink_info((void *)&path->regular_info, idx, addr);
}

void isp_path_set(struct isp_module *module,
	struct isp_path_desc *path, enum isp_path_index path_index)
{
	uint32_t scl_addr = 0, store_addr = 0;
	uint32_t idx = 0;

	if (!module || !path) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	idx = module->com_idx;
	if (ISP_PATH_IDX_PRE & path_index) {
		scl_addr = ISP_SCALER_PRE_CAP_BASE;
		store_addr = ISP_STORE_PRE_CAP_BASE;
	} else if (ISP_PATH_IDX_VID & path_index) {
		scl_addr = ISP_SCALER_VID_BASE;
		store_addr = ISP_STORE_VID_BASE;
	} else if (ISP_PATH_IDX_CAP & path_index) {
		scl_addr = ISP_SCALER_PRE_CAP_BASE;
		store_addr = ISP_STORE_PRE_CAP_BASE;
	} else {
		pr_err("fail to get valid path index\n");
		return;
	}

	pr_debug("isp_path_set index %d\n", path_index);
	isp_set_channel_info(idx, path, path_index);
	isp_path_set_scl(idx, path, scl_addr);
	isp_set_store(idx, (void *)&path->store_info, store_addr);
}

int isp_get_scl_index(uint32_t channel_id)
{
	enum isp_scl_id  path_index;

	switch (channel_id) {
	case ISP_PATH_IDX_PRE:
		path_index = ISP_SCL_PRE;
		break;
	case ISP_PATH_IDX_VID:
		path_index = ISP_SCL_VID;
		break;
	case ISP_PATH_IDX_CAP:
		path_index = ISP_SCL_CAP;
		break;
	default:
		path_index = ISP_SCL_MAX;
		pr_info("fail to get path index, channel %d\n", channel_id);
	}

	return path_index;
}

int isp_path_set_next_frm(struct isp_module *module,
	enum isp_path_index path_index, struct slice_addr *addr,
	struct camera_frame *dcam_frame)
{
	int use_reserve_frame = 0;
	uint32_t idx = 0;
	unsigned long yuv_reg[3] = {0};
	unsigned long yuv_addr[3] = {0};
	enum isp_id id = ISP_ID_0;
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct camera_frame *reserved_frame = NULL;
	struct camera_frame frame;
	struct isp_path_desc *path = NULL;
	struct cam_frm_queue *p_heap = NULL;
	struct cam_buf_queue *p_buf_queue = NULL;

	if (module == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	memset((void *)&frame, 0, sizeof(frame));
	idx = module->com_idx;
	id = ISP_GET_ISP_ID(idx);
	if (path_index == ISP_PATH_IDX_PRE) {
		reserved_frame =
			 &module->isp_path[ISP_SCL_PRE].path_reserved_frame;
		path = &module->isp_path[ISP_SCL_PRE];
		yuv_reg[0] = ISP_STORE_PRE_CAP_BASE + ISP_STORE_SLICE_Y_ADDR;
		yuv_reg[1] = ISP_STORE_PRE_CAP_BASE + ISP_STORE_SLICE_U_ADDR;
		yuv_reg[2] = ISP_STORE_PRE_CAP_BASE + ISP_STORE_SLICE_V_ADDR;
		p_heap = &path->frame_queue;
		p_buf_queue = &path->buf_queue;
	} else if (path_index == ISP_PATH_IDX_VID) {
		reserved_frame =
			 &module->isp_path[ISP_SCL_VID].path_reserved_frame;
		path = &module->isp_path[ISP_SCL_VID];
		yuv_reg[0] = ISP_STORE_VID_BASE + ISP_STORE_SLICE_Y_ADDR;
		yuv_reg[1] = ISP_STORE_VID_BASE + ISP_STORE_SLICE_U_ADDR;
		yuv_reg[2] = ISP_STORE_VID_BASE + ISP_STORE_SLICE_V_ADDR;
		p_heap = &path->frame_queue;
		p_buf_queue = &path->buf_queue;
	} else if (path_index == ISP_PATH_IDX_CAP) {
		reserved_frame =
			 &module->isp_path[ISP_SCL_CAP].path_reserved_frame;
		path = &module->isp_path[ISP_SCL_CAP];
		yuv_reg[0] = ISP_STORE_PRE_CAP_BASE + ISP_STORE_SLICE_Y_ADDR;
		yuv_reg[1] = ISP_STORE_PRE_CAP_BASE + ISP_STORE_SLICE_U_ADDR;
		yuv_reg[2] = ISP_STORE_PRE_CAP_BASE + ISP_STORE_SLICE_V_ADDR;
		p_heap = &path->frame_queue;
		p_buf_queue = &path->buf_queue;
	} else {
		pr_err("fail to get error isp path index\n");
		return -EFAULT;
	}
	if (sprd_cam_buf_queue_read(p_buf_queue, &frame) == 0 &&
		(frame.buf_info.mfd[0] != 0)) {
		path->output_frame_count--;
	} else {
		use_reserve_frame = 1;
	}

	if (use_reserve_frame) {
		pr_debug("ISP%d: No free frame id %d path_index %d\n",
			id, frame.fid, path_index);
		if (reserved_frame->buf_info.mfd[0] == 0) {
			pr_info("ISP%d: No need to cfg frame buffer", id);
			return -1;
		}
		memcpy(&frame, reserved_frame, sizeof(struct camera_frame));
		use_reserve_frame = 1;
	}

	if (frame.buf_info.dev == NULL)
		pr_info("ISP%d next dev NULL %p\n", id, frame.buf_info.dev);

	if (dcam_frame != NULL) {
		frame.frame_id = dcam_frame->frame_id;
		frame.zoom_ratio = path->coeff_latest.param.in_size.w * 1000 /
			path->coeff_latest.param.in_rect.w;
	}

	frame.buf_info.type = CAM_BUF_USER_TYPE;
	rtn = cam_buf_map_addr(&frame.buf_info);
	if (rtn) {
		pr_err("fail to get dcam addr\n");
		return rtn;
	}
	yuv_addr[0] = frame.buf_info.iova[0] + frame.yaddr;
	yuv_addr[1] = frame.buf_info.iova[0] + frame.uaddr;
	yuv_addr[2] = frame.buf_info.iova[2];

	if (use_reserve_frame)
		memcpy(reserved_frame, &frame, sizeof(struct camera_frame));

	ISP_REG_WR(idx, yuv_reg[0], yuv_addr[0]);
	if (path->output_format < DCAM_YUV400) {
		ISP_REG_WR(idx, yuv_reg[1], yuv_addr[1]);
		if (path->output_format == DCAM_YUV420_3FRAME)
			ISP_REG_WR(idx, yuv_reg[2], yuv_addr[2]);
	}

	if (addr != NULL) {
		addr->chn0 = yuv_addr[0];
		addr->chn1 = yuv_addr[1];
		addr->chn2 = yuv_addr[2];
		pr_debug("isp%d path%d  mfd[0] 0x%x, addr->chn0 = 0x%x, yaddr = 0x%x, iova addr 0x%lx, uv addr 0x%x\n",
			id, path_index,
			frame.buf_info.mfd[0], addr->chn0, frame.yaddr,
			frame.buf_info.iova[0],  frame.uaddr);
	}

	if (sprd_cam_frm_enqueue(p_heap, &frame) == 0)
		pr_debug("success to enq frame buf\n");
	else {
		rtn = ISP_RTN_PATH_FRAME_LOCKED;
		pr_err("fail to enq frame buf\n");
	}

	return -rtn;
}

static void isp_pre_proc_dcam_frame(struct isp_pipe_dev *dev)
{
	uint32_t is_irq = 0x01;
	uint32_t skip_num = 0, max_zsl_num = 0;
	struct isp_pipe_dev *isp_handle = NULL;
	struct isp_module *module = NULL;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct isp_path_desc *cap = NULL;
	struct camera_frame frame;
	enum dcam_id idx = DCAM_ID_0;
	struct sprd_img_capture_param capture_param;

	isp_handle = (struct isp_pipe_dev *)dev;
	module = &isp_handle->module_info;
	fmcu_slice = &isp_handle->fmcu_slice;
	module->com_idx = isp_handle->com_idx;
	memset((void *)&frame, 0x00, sizeof(frame));
	memset(&capture_param, 0x00, sizeof(capture_param));

	cap = &dev->module_info.isp_path[ISP_SCL_CAP];

	skip_num = cap->skip_num;
	idx = ISP_GET_ISP_ID(dev->com_idx);
	if (dev->is_raw_capture == 1) {
		pr_info("is raw_capture\n");
		return;
	}

	if (dev->isp_offline_state == ISP_ST_START) {
		module->frm_cnt++;
		pr_debug("dev.cap_flag = %d",  dev->cap_flag);
		if (dev->cap_flag == DCAM_CAPTURE_START_HDR)
			max_zsl_num = ISP_HDR_NUM;
		else if (dev->cap_flag == DCAM_CAPTURE_START_3DNR)
			max_zsl_num = ISP_3DNR_NUM;
		else
			max_zsl_num = ISP_ZSL_BUF_NUM;
		/*skip frame*/
		pr_debug("cap_flag %d, valid_cnt %d, isp_busy %d, frm_cnt %d, skip_num %d\n",
			dev->cap_flag, module->full_zsl_queue.valid_cnt,
			dev->isp_busy, module->frm_cnt, skip_num);
		if (module->full_zsl_queue.valid_cnt > max_zsl_num
			|| (module->frm_cnt <= skip_num &&
			dev->is_raw_capture != 1)) {
			if (sprd_cam_frm_dequeue(&module->full_zsl_queue,
				&frame)) {
				pr_info("fail to dequeue full zsl dequeue\n");
				return;
			}
			/* this frame is skipped, set this buffer back to dcam*/
			dcam_set_frame_addr(isp_handle->isp_handle_addr,
				CAMERA_FULL_PATH, &frame);
		}

		pr_debug("capture_state %d, cap_cur_cnt = %d\n",
			fmcu_slice->capture_state, dev->cap_cur_cnt);
		if (fmcu_slice->capture_state == ISP_ST_START) {
			if (module->frm_cnt > skip_num)
				dev->cap_cur_cnt++;
			if ((dev->cap_flag == DCAM_CAPTURE_START_HDR
				|| dev->cap_flag == DCAM_CAPTURE_START_3DNR)
				&& dev->cap_cur_cnt < max_zsl_num)
				return;

			if (dev->cap_cur_cnt >= max_zsl_num) {
				dev->isp_offline_state = ISP_ST_STOP;
				if (s_isp_group.dual_cam) {
					if (s_isp_group.dual_fullpath_stop
						== 0) {
						s_isp_group.dual_fullpath_stop
							= 1;
						sprd_dcam_path_pause(DCAM_ID_0,
							CAMERA_FULL_PATH);
						sprd_dcam_path_pause(DCAM_ID_1,
							CAMERA_FULL_PATH);
					}
				} else {
					if (!dev->dcam_full_path_stop) {
						dev->dcam_full_path_stop = 1;
						sprd_dcam_path_pause(idx,
							CAMERA_FULL_PATH);
					}
				}
			}

			if (s_isp_group.dual_cam &&
				s_isp_group.dual_cap_cnt == 0
				&& s_isp_group.dual_cap_sts) {
				pr_info("wait dual first frame done.\n");
				s_isp_group.first_need_wait = 1;
			} else {
				pr_info("start isp%d fmcu\n", idx);
				capture_param.type = DCAM_CAPTURE_START;
				if (sprd_isp_start_fmcu(isp_handle,
					capture_param, is_irq,
					ISP_PATH_IDX_CAP)) {
					pr_info("fail to start slice capture\n");
					return;
				}
			}
		}
	} else {
		pr_info("Isp%d has been work, send buf back to dcam.\n", idx);
		/*revevie dcam full path done int,after stop dcam*/
		if (sprd_cam_frm_dequeue(&module->full_zsl_queue, &frame)) {
			pr_err("fail to dequeue full zsl frame\n");
			return;
		}
		dcam_set_frame_addr(dev->isp_handle_addr,
			CAMERA_FULL_PATH, &frame);
	}
}

/*
* dcam driver will call this func in tx_done,
* and send the client and handle of the buffer to isp driver,
* and isp driver will map the buffer again
*/
void isp_set_offline_frame(struct isp_pipe_dev *dev,
	enum camera_path_id path_index, struct camera_frame *frame)
{
	struct isp_pipe_dev *isp_handle = NULL;
	struct isp_module *module = NULL;

	isp_handle = (struct isp_pipe_dev *)dev;
	module = &isp_handle->module_info;

	if (path_index == CAMERA_BIN_PATH) {
		if (sprd_cam_buf_queue_write(&module->bin_buf_queue, frame)) {
			pr_err("fail to write buff frame\n");
			return;
		}

		complete(&dev->offline_thread_com);
	} else if (path_index == CAMERA_FULL_PATH) {
		if (sprd_cam_frm_enqueue(&module->full_zsl_queue, frame)) {
			pr_err("fail to enqueue full_zsl_queue\n");
			return;
		}
		if (dev->clr_queue == 1) {
			dev->clr_queue = 0;
			while (!sprd_cam_frm_dequeue(&module->full_zsl_queue,
				frame)) {
				pr_debug("capture clr queue\n");
				dcam_set_frame_addr(isp_handle->isp_handle_addr,
					CAMERA_FULL_PATH, frame);
			}
			return;
		}
		pr_debug("zsl queue %d\n", module->full_zsl_queue.valid_cnt);
		/*
		* when start capture too early, but the full_zsl_queue is empty,
		* we must wait the next full path tx_done,
		* and trigger the fmcu start.
		*/
		isp_pre_proc_dcam_frame(dev);
	}
}

void isp_start_raw_proc(struct isp_pipe_dev *dev,
	enum camera_path_id path_index, struct camera_frame *frame)
{
	struct isp_pipe_dev *isp_handle = NULL;
	struct isp_module *module = NULL;

	if (dev == NULL)
		pr_err("fail to check param\n");

	isp_handle = (struct isp_pipe_dev *)dev;
	module = &isp_handle->module_info;

	if (sprd_cam_frm_enqueue(&module->full_zsl_queue, frame)) {
		pr_err("fail to write buff frame\n");
		return;
	}
	pr_info("start raw proc\n");
	complete(&dev->offline_thread_com);
}
