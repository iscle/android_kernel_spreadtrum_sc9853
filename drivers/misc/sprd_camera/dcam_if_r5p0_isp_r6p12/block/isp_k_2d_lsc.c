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

#include <linux/uaccess.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>
#include <video/sprd_mm.h>

#include "sprd_isp_hw.h"
#include "isp_drv.h"
#include "isp_buf.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "2D LSC: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define ISP_LSC_TIME_OUT_MAX        500
#define ISP_LSC_BUF0                0
#define ISP_LSC_BUF1                1
#define ISP_BYPASS_EB               1
#define  ISP_EVT_LSC_LOAD (1 << 2)

static int isp_k_2d_lsc_param_load
	(struct isp_k_block *isp_k_param, enum isp_id idx,
	struct isp_dev_2d_lsc_info *lens_info, uint32_t scene_id)
{
	int ret = 0;
	unsigned int val = 0;
	unsigned int time_out_cnt = 0;
	unsigned int reg_value = 0;
	enum camera_copy_id copy_id = BIN_COPY;
	enum dcam_id dcam_idx = DCAM_ID_0;

	dcam_idx = (enum dcam_id)idx;

	DCAM_REG_WR(idx, ISP_LENS_BASE_RADDR, isp_k_param->lsc_buf_phys_addr);

	DCAM_REG_MWR(idx, ISP_LENS_GRID_NUMBER, 0X7FF, lens_info->grid_num_t);

	DCAM_REG_MWR(idx, ISP_LENS_LOAD_CLR, BIT_0, 1);

	DCAM_REG_MWR(idx, ISP_LENS_LOAD_EB, BIT_0, 0);

	dcam_force_copy(dcam_idx, copy_id);

	val = ((lens_info->grid_width & 0x1FF) << 16) |
		((lens_info->grid_y_num & 0xFF) << 8) |
		(lens_info->grid_x_num & 0xFF);
	DCAM_REG_MWR(idx, ISP_LENS_GRID_SIZE, 0x1FFFFFF, val);

	if (scene_id == ISP_SCENE_PRE) {
		if (isp_k_param->lsc_load_buf_id_prv == ISP_LSC_BUF0)
			isp_k_param->lsc_load_buf_id_prv = ISP_LSC_BUF1;
		else
			isp_k_param->lsc_load_buf_id_prv = ISP_LSC_BUF0;
		DCAM_REG_MWR(idx, ISP_LENS_LOAD_EB, BIT_1,
				(!isp_k_param->lsc_load_buf_id_prv) << 1);
	} else {
		if (isp_k_param->lsc_load_buf_id_cap == ISP_LSC_BUF0)
			isp_k_param->lsc_load_buf_id_cap = ISP_LSC_BUF1;
		else
			isp_k_param->lsc_load_buf_id_cap = ISP_LSC_BUF0;
		DCAM_REG_MWR(idx, ISP_LENS_LOAD_EB, BIT_1,
				(!isp_k_param->lsc_load_buf_id_cap) << 1);
	}

	dcam_auto_copy(dcam_idx, copy_id);

	reg_value = DCAM_REG_RD(idx, ISP_LENS_LOAD_EB);
	while (((reg_value & ISP_EVT_LSC_LOAD) == 0x00)
		&& (time_out_cnt < ISP_LSC_TIME_OUT_MAX)) {
		udelay(1);
		reg_value = DCAM_REG_RD(idx, ISP_LENS_LOAD_EB);
		time_out_cnt++;
	}
	if (time_out_cnt >= ISP_LSC_TIME_OUT_MAX) {
		ret = -EPERM;
		pr_err("fail to load lsc param,lsc load time out\n");
	}

	DCAM_REG_MWR(idx, ISP_LENS_LOAD_CLR, BIT_1, 1);

	return ret;
}

static int isp_k_2d_lsc_block(struct isp_io_param *param,
	struct isp_k_block *isp_k_param,
	enum isp_id idx, uint32_t scene_id)
{
	int ret = 0;
	unsigned int i = 0;
	unsigned int dst_w_num = 0;
	unsigned long dst_addr = 0;
	void *data_ptr = NULL;
	unsigned short *w_buff = NULL;
	struct isp_dev_2d_lsc_info lens_info;

	memset(&lens_info, 0x00, sizeof(lens_info));

	if (isp_k_param->isp_status) {
		ret = 0;
		goto exit;
	}

	ret = copy_from_user((void *)&lens_info, param->property_param,
			sizeof(lens_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		ret = -EPERM;
		goto exit;
	}

	if (lens_info.bypass) {
		ret = 0;
		goto exit;
	}

	/*load weight tab*/
	if (!isp_k_param->lsc_2d_weight_en) {
#ifdef CONFIG_64BIT
		data_ptr = (void *)(((unsigned long)lens_info.data_ptr[1] << 32)
						| lens_info.data_ptr[0]);
#else
		data_ptr = (void *)(lens_info.data_ptr[0]);
#endif
		isp_k_param->lsc_2d_weight_en = 0;
		dst_addr = DCAM_BASE(idx) + ISP_LENS_WEIGHT_ADDR;
		w_buff = vzalloc(lens_info.weight_num);

		if (w_buff == NULL) {
			ret = -ENOMEM;
			goto exit;
		}

		ret = copy_from_user((void *)w_buff,  data_ptr,
			lens_info.weight_num);

		if (ret != 0) {
			pr_err("fail to copy from user, ret = %d\n",
				ret);
			vfree(w_buff);
			ret = -1;
			goto exit;
		}

		dst_w_num = (lens_info.grid_width >> 1) + 1;

		for (i = 0; i < dst_w_num; i++) {
			*((unsigned int *)dst_addr + i * 2) =
				(((unsigned int)(*(w_buff + i * 3))) & 0xFFFF) |
				((((unsigned int)(*(w_buff+i * 3 + 1))) &
				0xFFFF) << 16);

			*((unsigned int *)dst_addr + i * 2 + 1) =
				((unsigned int)(*(w_buff + i * 3 + 2))) &
				0xFFFF;
		}
		vfree(w_buff);
	}

	DCAM_REG_MWR(idx, DCAM_APB_SRAM_CTRL, BIT_0, 1);

	ret = isp_k_2d_lsc_param_load(isp_k_param, idx,
		&lens_info, scene_id);
	if (ret != 0) {
		pr_err("fail to load lsc param, ret = %d\n", ret);
		DCAM_REG_MWR(idx, ISP_LENS_LOAD_EB, BIT_0, ISP_BYPASS_EB);
		ret = -EPERM;
		goto exit;
	}

exit:
	return ret;
}

static int isp_k_2d_lsc_transaddr(struct isp_io_param *param,
			struct isp_k_block *isp_k_param)
{
	int ret = 0;
	struct isp_dev_block_addr lsc_buf;
	struct cam_buf_info *plsc_pfinfo = &isp_k_param->lsc_pfinfo;

	memset(&lsc_buf, 0x00, sizeof(lsc_buf));
	ret = copy_from_user(&lsc_buf, param->property_param,
			     sizeof(lsc_buf));

	plsc_pfinfo->dev = &s_dcam_pdev->dev;
	plsc_pfinfo->mfd[0] = lsc_buf.img_fd;
	plsc_pfinfo->offset[0] = lsc_buf.offset.x;
	ret = cam_buf_get_sg_table(plsc_pfinfo);
	ret = cam_buf_map_addr(plsc_pfinfo);

	isp_k_param->lsc_buf_phys_addr = plsc_pfinfo->iova[0]
		+ lsc_buf.offset.x;

	pr_debug("lsc addr 0x%lx, 0x%x\n",
		isp_k_param->lsc_pfinfo.iova[0],
		(uint32_t)isp_k_param->lsc_pfinfo.size[0]);

	return ret;
}

int isp_k_cfg_2d_lsc(struct isp_io_param *param,
			struct isp_k_block *isp_k_param, uint32_t com_idx)
{
	int ret = 0;
	enum isp_id idx = ISP_ID_0;
	uint32_t scene_id = 0;

	if (!param) {
		pr_err("fail to get param\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("fail to get property_param\n");
		return -EPERM;
	}

	idx = ISP_GET_ISP_ID(com_idx);
	scene_id = ISP_GET_SCENE_ID(com_idx);

	switch (param->property) {
	case ISP_PRO_2D_LSC_BLOCK:
		ret = isp_k_2d_lsc_block(param, isp_k_param, idx, scene_id);
		break;
	case ISP_PRO_2D_LSC_TRANSADDR:
		ret = isp_k_2d_lsc_transaddr(param, isp_k_param);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}
