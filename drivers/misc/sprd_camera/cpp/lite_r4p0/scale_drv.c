/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/semaphore.h>
#include <uapi/video/sprd_cpp.h>

#include "cpp_common.h"
#include "cpp_reg.h"
#include "cpp_core.h"
#include "scale_drv.h"

/* Macro Definition */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "SCALE_DRV: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define PATH0_ADDR_ALIGN 0x07
#define SCALE_LOWEST_ADDR		0x800
#define SCALE_ADDR_INVALID(addr) \
	((unsigned long)(addr) < SCALE_LOWEST_ADDR)
#define SCALE_YUV_ADDR_INVALID(y, u, v) \
	(SCALE_ADDR_INVALID(y) && \
	SCALE_ADDR_INVALID(u) && \
	SCALE_ADDR_INVALID(v))

#define SCALE_FRAME_WIDTH_MAX		8192
#define SCALE_FRAME_HEIGHT_MAX		8192
#define SCALE_FRAME_OUT_WIDTH_MAX	768
#define SCALE_SC_COEFF_MAX		8
#define SCALE_SC_COEFF_MID		4
#define SCALE_DECI_FAC_MAX		3
#define SCALE_PIXEL_ALIGNED		4
#define ALIGNED_DOWN_2(w) ((w) & ~(2 - 1))
#define ALIGNED_DOWN_4(w) ((w) & ~(4 - 1))
#define ALIGNED_DOWN_8(w) ((w) & ~(8 - 1))

/* Internal Function Implementation */
static void scale_dev_stop(struct scale_drv_private *p)
{
	unsigned long flags = 0;

	spin_lock_irqsave(p->hw_lock, flags);
	reg_awr(p, CPP_PATH_START, (~CPP_SCALE_START_BIT));
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static void scale_dev_start(struct scale_drv_private *p)
{
	unsigned long flags = 0;

	spin_lock_irqsave(p->hw_lock, flags);
	reg_owr(p, CPP_PATH_START, CPP_SCALE_START_BIT);
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static int scale_k_check_param(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_size.w > SCALE_FRAME_WIDTH_MAX ||
		cfg_parm->input_size.h > SCALE_FRAME_HEIGHT_MAX ||
		cfg_parm->output_size.w > SCALE_FRAME_OUT_WIDTH_MAX ||
		cfg_parm->output_size.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_err("fail to get valid size:%d %d %d %d\n",
			cfg_parm->input_size.w, cfg_parm->input_size.h,
			cfg_parm->output_size.w, cfg_parm->output_size.h);
		return -1;
	} else if (cfg_parm->input_size.w < cfg_parm->input_rect.w +
		cfg_parm->input_rect.x ||
		cfg_parm->input_size.h < cfg_parm->input_rect.h +
		cfg_parm->input_rect.y) {
		pr_err("fail to get valid rect %d %d %d %d %d %d\n",
			cfg_parm->input_size.w, cfg_parm->input_size.h,
			cfg_parm->input_rect.x, cfg_parm->input_rect.y,
			cfg_parm->input_rect.w, cfg_parm->input_rect.h);
		return -1;
	}
	if (cfg_parm->output_size.w % 4 != 0) {
		pr_info("fail to get dst width align 8: %d\n",
				cfg_parm->input_rect.x);
		return -1;
	}
	if (cfg_parm->output_format == SCALE_YUV420)
		if (cfg_parm->output_size.h % 2 != 0) {
			pr_info("fail to get dst height align 2: %d\n",
				cfg_parm->input_rect.x);
			return -1;
		}
	if (cfg_parm->input_size.w % 8 != 0) {
		pr_err("fail to get src scale pitch size %d\n",
			cfg_parm->input_size.w);
		return -1;
	}
	if (cfg_parm->input_format == SCALE_YUV420) {
		if (cfg_parm->input_rect.h % 2 != 0) {
			cfg_parm->input_rect.h =
				ALIGNED_DOWN_2(cfg_parm->input_rect.h);
			pr_info("adjust src height align 2: %d\n",
				cfg_parm->input_rect.y);
		}
		if (cfg_parm->input_rect.y % 2 != 0) {
			cfg_parm->input_rect.y =
				ALIGNED_DOWN_2(cfg_parm->input_rect.y);
			pr_info("adjust src offset y align 2: %d\n",
				cfg_parm->input_rect.y);
		}
	}
	if (cfg_parm->input_rect.w % 4 != 0) {
		cfg_parm->input_rect.w =
			ALIGNED_DOWN_4(cfg_parm->input_rect.w);
		pr_info("adjust src width align 4: %d\n",
				cfg_parm->input_rect.y);
	}
	if (cfg_parm->input_rect.x % 2 != 0) {
		cfg_parm->input_rect.x =
			ALIGNED_DOWN_2(cfg_parm->input_rect.x);
		pr_info("adjust src offset x align 2: %d\n",
				cfg_parm->input_rect.x);
	}

	return 0;
}

static void scale_k_set_src_pitch(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return;
	}
	cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_CFG3, CPP_SCALE_SRC_PITCH_MASK,
		cfg_parm->input_size.w);
}

static int scale_k_set_input_rect(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}
	cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_CFG1, CPP_SCALE_SRC_HEIGHT_MASK,
		cfg_parm->input_rect.h << 16);
	reg_mwr(p, CPP_PATH0_CFG1, CPP_SCALE_SRC_WIDTH_MASK,
		cfg_parm->input_rect.w);
	reg_mwr(p, CPP_PATH0_CFG4, CPP_SCALE_SRC_OFFSET_X_MASK,
		cfg_parm->input_rect.x << 16);
	reg_mwr(p, CPP_PATH0_CFG4, CPP_SCALE_SRC_OFFSET_Y_MASK,
		cfg_parm->input_rect.y);

	return 0;
}

static int scale_k_calc_sc_size(struct scale_drv_private *p)
{
	int i = 0;
	unsigned int deci_tmp_w = 0;
	unsigned int deci_tmp_h = 0;
	unsigned int div_factor = 1;
	unsigned int deci_val = 0;
	unsigned int pixel_aligned_num = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (unlikely(cfg_parm->input_rect.w >
	    (cfg_parm->output_size.w * SCALE_SC_COEFF_MAX *
	     (1 << SCALE_DECI_FAC_MAX))
	    || cfg_parm->input_rect.h >
	    (cfg_parm->output_size.h * SCALE_SC_COEFF_MAX *
	     (1 << SCALE_DECI_FAC_MAX))
	    || cfg_parm->input_rect.w <
	    cfg_parm->output_size.w
	    || cfg_parm->input_rect.h <
	    cfg_parm->output_size.h)) {
		pr_err("fail to get invalid in_rect %d %d, out_size %d %d",
			cfg_parm->input_rect.w, cfg_parm->input_rect.h,
			cfg_parm->output_size.w, cfg_parm->output_size.h);
		return -EINVAL;
	}

	/* check for w decimation */
	if (cfg_parm->input_rect.w <=
		(cfg_parm->output_size.w * SCALE_SC_COEFF_MID))
		deci_tmp_w = 0;
	else if (cfg_parm->input_rect.w >
		(cfg_parm->output_size.w * SCALE_SC_COEFF_MID) &&
		(cfg_parm->input_rect.w <=
		(cfg_parm->output_size.w * SCALE_SC_COEFF_MID *
		(1 << SCALE_DECI_FAC_MAX)))) {
		for (i = 1; i <= SCALE_DECI_FAC_MAX; i++) {
			div_factor =
			(unsigned int)(SCALE_SC_COEFF_MID * (1 << i));
			if (cfg_parm->input_rect.w <=
				(cfg_parm->output_size.w * div_factor)) {
				break;
			}
		}
		deci_tmp_w = i;
	} else
		deci_tmp_w = SCALE_DECI_FAC_MAX;

	deci_val = (1 << deci_tmp_w);
	pixel_aligned_num =
		(deci_val >= SCALE_PIXEL_ALIGNED)
		? deci_val : SCALE_PIXEL_ALIGNED;

	p->sc_input_size.w = cfg_parm->input_rect.w >> deci_tmp_w;

	if (pixel_aligned_num > 0 &&
		(p->sc_input_size.w % pixel_aligned_num)) {
		p->sc_input_size.w =
			p->sc_input_size.w / pixel_aligned_num
			* pixel_aligned_num;

		cfg_parm->input_rect.w = p->sc_input_size.w << deci_tmp_w;
	}
	p->sc_deci_val_w = deci_tmp_w;

	/* check for h decimation */
	if (cfg_parm->input_rect.h <=
		(cfg_parm->output_size.h * SCALE_SC_COEFF_MID))
		deci_tmp_h = 0;
	else if ((cfg_parm->input_rect.h >
		(cfg_parm->output_size.h * SCALE_SC_COEFF_MID)) &&
		(cfg_parm->input_rect.h <=
		(cfg_parm->output_size.h * SCALE_SC_COEFF_MID) *
		(1 << SCALE_DECI_FAC_MAX))) {
		for (i = 1; i <= SCALE_DECI_FAC_MAX; i++) {
			div_factor =
			(unsigned int)(SCALE_SC_COEFF_MID * (1 << i));
			if (cfg_parm->input_rect.h <=
				(cfg_parm->output_size.h * div_factor)) {
				break;
			}
		}
		deci_tmp_h = i;
	} else
		deci_tmp_h = SCALE_DECI_FAC_MAX;

	deci_val = (1 << deci_tmp_h);
	pixel_aligned_num =
		(deci_val >= SCALE_PIXEL_ALIGNED)
		? deci_val : SCALE_PIXEL_ALIGNED;

	p->sc_input_size.h = cfg_parm->input_rect.h >> deci_tmp_h;

	if (pixel_aligned_num > 0 &&
		(p->sc_input_size.w % pixel_aligned_num)) {
		p->sc_input_size.h =
			p->sc_input_size.h / pixel_aligned_num
			* pixel_aligned_num;

		cfg_parm->input_rect.h = p->sc_input_size.h << deci_tmp_h;
	}
	p->sc_deci_val_h = deci_tmp_h;

	reg_mwr(p, CPP_PATH0_CFG0,
		CPP_SCALE_DEC_H_MASK, deci_tmp_w << 4);
	reg_mwr(p, CPP_PATH0_CFG0,
		CPP_SCALE_DEC_V_MASK, deci_tmp_h << 6);

	pr_debug("sc_input_size %d %d, deci(w,h) %d, %d input_rect %d %d",
		p->sc_input_size.w,
		p->sc_input_size.h,
		deci_tmp_w, deci_tmp_h,
		cfg_parm->input_rect.w,
		cfg_parm->input_rect.h);

	return 0;
}

static int scale_k_set_input_size(struct scale_drv_private *p)
{
	int ret = 0;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	ret = scale_k_calc_sc_size(p);
	if (ret) {
		pr_err("fail to configure scaler\n");
		return ret;
	}
	ret = scale_k_set_input_rect(p);
	if (ret) {
		pr_err("fail to set input rect size\n");
		return ret;
	}

	return ret;
}

static int scale_k_set_input_format(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (!(cfg_parm->input_format == SCALE_YUV422 ||
		cfg_parm->input_format == SCALE_YUV420)) {
		pr_err("fail to get valid input format %d\n",
			cfg_parm->input_format);
		return -EINVAL;
	}

	reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_INPUT_FORMAT,
		(cfg_parm->input_format << 2));

	return 0;
}

static int scale_k_set_input_endian(struct scale_drv_private *p)
{
	unsigned int y_endian = 0;
	unsigned int uv_endian = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_endian.y_endian >= SCALE_ENDIAN_MAX
	 || cfg_parm->input_endian.uv_endian >= SCALE_ENDIAN_MAX) {
		pr_err("invalid input endian %d %d\n",
			cfg_parm->input_endian.y_endian,
			cfg_parm->input_endian.uv_endian);
		return -EINVAL;
	}

	if (cfg_parm->input_endian.y_endian == SCALE_ENDIAN_LITTLE)
		y_endian = 0;

	if (cfg_parm->input_endian.y_endian == SCALE_ENDIAN_LITTLE
	 && cfg_parm->input_endian.uv_endian == SCALE_ENDIAN_HALFBIG)
		uv_endian = 1;

	reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_DMA_INPUT_Y_ENDIAN),
		y_endian);
	reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_DMA_INPUT_UV_ENDIAN),
		uv_endian << 3);

	pr_debug("CPP:input endian y:%d, uv:%d\n", y_endian, uv_endian);


	return 0;
}

static int scale_k_set_des_pitch(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_CFG3, CPP_SCALE_DES_PITCH_MASK,
		cfg_parm->output_size.w << 16);

	return 0;
}

static int scale_k_set_output_size(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_CFG2, CPP_SCALE_DES_HEIGHT_MASK,
			cfg_parm->output_size.h << 16);
	reg_mwr(p, CPP_PATH0_CFG2, CPP_SCALE_DES_WIDTH_MASK,
			cfg_parm->output_size.w);

	reg_wr(p, CPP_PATH0_CFG5, 0);

	return 0;
}

static int scale_k_set_output_format(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (!(cfg_parm->output_format == SCALE_YUV422 ||
		cfg_parm->output_format == SCALE_YUV420)) {
		pr_err("fail to get valid output format %d\n",
			cfg_parm->output_format);
		return -EINVAL;
	}

	reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_OUTPUT_FORMAT,
		(cfg_parm->output_format << 4));

	return 0;
}

static int scale_k_set_output_endian(struct scale_drv_private *p)
{
	unsigned int y_endian = 0;
	unsigned int uv_endian = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->output_endian.y_endian >= SCALE_ENDIAN_MAX
	 || cfg_parm->output_endian.uv_endian >= SCALE_ENDIAN_MAX) {
		pr_err("fail to get valid output endian %d %d\n",
			cfg_parm->output_endian.y_endian,
			cfg_parm->output_endian.uv_endian);
		return -EINVAL;
	}
	if (cfg_parm->output_endian.y_endian == SCALE_ENDIAN_LITTLE)
		y_endian = 0;

	if (cfg_parm->output_endian.y_endian == SCALE_ENDIAN_LITTLE
	 && cfg_parm->output_endian.uv_endian == SCALE_ENDIAN_HALFBIG)
		uv_endian = 1;

	reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_DMA_OUTPUT_Y_ENDIAN),
		y_endian << 4);
	reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_DMA_OUTPUT_UV_ENDIAN),
		uv_endian << 7);

	pr_debug("CPP:output endian y:%d, uv:%d\n",
		y_endian, uv_endian);

	return 0;
}

static int scale_k_set_addr(struct scale_drv_private *p)
{
	int ret = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_addr.mfd[0] == 0 ||
		cfg_parm->input_addr.mfd[1] == 0 ||
		cfg_parm->output_addr.mfd[0] == 0 ||
		cfg_parm->output_addr.mfd[1] == 0) {
		pr_err("fail to get valid in or out mfd\n");
		return -1;
	}
	memcpy(p->iommu_src.mfd, cfg_parm->input_addr.mfd,
		3 * sizeof(unsigned int));
	memcpy(p->iommu_dst.mfd, cfg_parm->output_addr.mfd,
		3 * sizeof(unsigned int));

	ret = cpp_get_sg_table(&p->iommu_src);
	if (ret) {
		pr_err("fail to get cpp src sg table\n");
		return -1;
	}
	p->iommu_src.offset[0] = cfg_parm->input_addr.y;
	p->iommu_src.offset[1] = cfg_parm->input_addr.u;
	p->iommu_src.offset[2] = cfg_parm->input_addr.v;
	ret = cpp_get_addr(&p->iommu_src);
	if (ret) {
		pr_err("fail to get cpp src addr\n");
		return -1;
	}

	ret = cpp_get_sg_table(&p->iommu_dst);
	if (ret) {
		pr_err("fail to get cpp dst sg table\n");
		cpp_free_addr(&p->iommu_src);
		return ret;
	}
	p->iommu_dst.offset[0] = cfg_parm->output_addr.y;
	p->iommu_dst.offset[1] = cfg_parm->output_addr.u;
	p->iommu_dst.offset[2] = cfg_parm->output_addr.v;
	ret = cpp_get_addr(&p->iommu_dst);
	if (ret) {
		pr_err("fail to get cpp dst addr\n");
		cpp_free_addr(&p->iommu_src);
		return ret;
	}

	reg_wr(p, CPP_PATH0_SRC_ADDR_Y,
		p->iommu_src.iova[0]);
	reg_wr(p, CPP_PATH0_SRC_ADDR_UV,
		p->iommu_src.iova[1]);
	reg_wr(p, CPP_PATH0_DES_ADDR_Y,
	       p->iommu_dst.iova[0]);
	reg_wr(p, CPP_PATH0_DES_ADDR_UV,
	       p->iommu_dst.iova[1]);
	pr_debug("iommu_src y:0x%lx, uv:0x%lx\n",
		p->iommu_src.iova[0], p->iommu_src.iova[1]);
	pr_debug("iommu_dst y:0x%lx, uv:0x%lx\n",
		p->iommu_dst.iova[0], p->iommu_dst.iova[1]);

	return ret;
}


static void scale_dev_enable(struct scale_drv_private *p)
{
	unsigned long flags;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return;
	}
	spin_lock_irqsave(p->hw_lock, flags);
	reg_owr(p, CPP_PATH_EB, CPP_SCALE_PATH_EB_BIT);
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static void scale_dev_disable(struct scale_drv_private *p)
{
	unsigned long flags;

	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return;
	}
	spin_lock_irqsave(p->hw_lock, flags);
	reg_awr(p, CPP_PATH_EB, (~CPP_SCALE_PATH_EB_BIT));
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static void scale_k_set_qos(struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return;
	}
	reg_mwr(p, CPP_AXIM_CHN_SET, CPP_AXIM_CHN_SET_QOS_MASK, (0x1 << 28));
	reg_mwr(p, CPP_MMU_PT_UPDATE_QOS, CPP_MMU_PT_UPDATE_QOS_MASK, 0x1);
}

void get_cpp_max_size(unsigned int *max_width, unsigned int *max_height)
{
	*max_width = SCALE_FRAME_WIDTH_MAX;
	*max_height = SCALE_FRAME_HEIGHT_MAX;
}

int cpp_scale_start(struct sprd_cpp_scale_cfg_parm *parm,
			struct scale_drv_private *p)
{
	int ret = 0;

	if (!parm || !p) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	memset(&p->sc_input_size, 0x00, sizeof(p->sc_input_size));
	p->sc_deci_val_w = 0;
	p->sc_deci_val_h = 0;

	memcpy((void *)&p->cfg_parm, (void *)parm,
		sizeof(struct sprd_cpp_scale_cfg_parm));

	scale_dev_stop(p);
	scale_dev_enable(p);
	ret = scale_k_check_param(p);
	if (ret) {
		pr_err("fail to get valid param\n");
		return -1;
	}
	scale_k_set_src_pitch(p);
	scale_k_set_des_pitch(p);
	ret = scale_k_set_input_size(p);
	if (ret) {
		pr_err("fail to set input size\n");
		return -1;
	}
	scale_k_set_output_size(p);

	ret = scale_k_set_input_format(p);
	if (ret) {
		pr_err("fail to get valid input format\n");
		return -1;
	}
	ret = scale_k_set_output_format(p);
	if (ret) {
		pr_err("fail to get valid output format\n");
		return -1;
	}
	ret = scale_k_set_input_endian(p);
	if (ret) {
		pr_err("fail to get valid input endian\n");
		return -1;
	}
	ret = scale_k_set_output_endian(p);
	if (ret) {
		pr_err("fail to get valid output endian\n");
		return -1;
	}
	ret = scale_k_set_addr(p);
	if (ret) {
		pr_err("fail to get valid output format\n");
		return -1;
	}

	pr_info("in_size %d %d in_rect %d %d %d %d out_size %d %d deci(w,h) %d %d\n",
		p->cfg_parm.input_size.w, p->cfg_parm.input_size.h,
		p->cfg_parm.input_rect.x, p->cfg_parm.input_rect.y,
		p->cfg_parm.input_rect.w, p->cfg_parm.input_rect.h,
		p->cfg_parm.output_size.w, p->cfg_parm.output_size.h,
		p->sc_deci_val_w,
		p->sc_deci_val_h);

	scale_k_set_qos(p);
	scale_dev_start(p);

	return ret;
}

void cpp_scale_stop(struct scale_drv_private *p)
{
	if (!p) {
		pr_err("fail to get valid input ptr\n");
		return;
	}
	scale_dev_stop(p);
	scale_dev_disable(p);
	cpp_free_addr(&p->iommu_src);
	cpp_free_addr(&p->iommu_dst);
}

int cpp_scale_capability(struct sprd_cpp_scale_capability *scale_param)
{
	int ret = 0;

/* scale input size */
	if (scale_param->src_size.w > SCALE_FRAME_WIDTH_MAX ||
	    scale_param->src_size.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_err("invalid input size %d %d\n",
			scale_param->src_size.w, scale_param->src_size.h);
		return -1;
	}
	if (scale_param->src_size.w % 8 != 0) {
		pr_err("scale pitch size is error\n");
		return -1;
	}

/* scale output size */
	if (scale_param->dst_size.w > SCALE_FRAME_OUT_WIDTH_MAX
	 || scale_param->dst_size.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_err("invalid output size %d %d\n",
			scale_param->dst_size.w, scale_param->dst_size.h);
		return -1;
	}
	if (scale_param->dst_size.w % 8 != 0) {
		pr_err("scale pitch size is error\n");
		return -1;
	}

	if ((scale_param->dst_format == SCALE_YUV420)
		&& (scale_param->dst_size.h % 2 != 0)) {
		pr_err("scale output height is invalid:%u\n",
			scale_param->dst_size.h);
		return -1;
	}
	return ret;
}

