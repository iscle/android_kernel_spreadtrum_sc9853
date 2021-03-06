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

#include "sprd_isp_hw.h"
#include "isp_block.h"

typedef int (*isp_cfg_fun_ptr)(struct isp_io_param *isp_param,
			struct isp_k_block *isp_k_param, uint32_t idx);

struct isp_cfg_fun {
	unsigned int sub_block;
	isp_cfg_fun_ptr cfg_fun;
};

static struct isp_cfg_fun isp_cfg_fun_tab[] = {
#if 0
	{ISP_BLOCK_ARBITER,            isp_k_cfg_arbiter},
	{ISP_BLOCK_DISPATCH,           isp_k_cfg_dispatch},
	{ISP_BLOCK_FETCH,              isp_k_cfg_fetch},
	{ISP_BLOCK_HIST,               isp_k_cfg_hist},
	{ISP_BLOCK_HIST2,              isp_k_cfg_hist2},
	{ISP_BLOCK_HUE,                isp_k_cfg_hue},
	{ISP_BLOCK_PDAF,               isp_k_cfg_pdaf},
	{ISP_BLOCK_PSTRZ,              isp_k_cfg_pstrz},
	{ISP_BLOCK_STORE,              isp_k_cfg_store},
	{ISP_BLOCK_YDELAY,             isp_k_cfg_ydelay},
	{ISP_BLOCK_YGAMMA,             isp_k_cfg_ygamma},
	{ISP_BLOCK_FETCH,              isp_k_cfg_fetch},
	{ISP_BLOCK_3DNR,               isp_k_cfg_3dnr},
#endif
	{ISP_BLOCK_CFA,			isp_k_cfg_cfa},
	{ISP_BLOCK_BRIGHTNESS,	isp_k_cfg_brightness},
	{ISP_BLOCK_CMC,			isp_k_cfg_cmc10},
	{ISP_BLOCK_CONTRAST,		isp_k_cfg_contrast},
	{ISP_BLOCK_CSA,			isp_k_cfg_csa},
	{ISP_BLOCK_EDGE,			isp_k_cfg_edge},
	{ISP_BLOCK_CCE,			isp_k_cfg_cce},
	{ISP_BLOCK_POST_CDN,		isp_k_cfg_post_cdn},
	{ISP_BLOCK_PRE_CDN,		isp_k_cfg_pre_cdn},
	{ISP_BLOCK_YUV_CDN,		isp_k_cfg_yuv_cdn},
	{ISP_BLOCK_UVD,			isp_k_cfg_uvd},
	{ISP_BLOCK_HSV,			isp_k_cfg_hsv},
	{ISP_BLOCK_YNR,			isp_k_cfg_ynr},
	{ISP_BLOCK_IIRCNR,		isp_k_cfg_iircnr},
	{ISP_BLOCK_NLM,			isp_k_cfg_nlm},
	{ISP_BLOCK_RGBG_DITHER,	isp_k_cfg_rgb_dither},
	{ISP_BLOCK_BPC,			isp_k_cfg_bpc},
	{ISP_BLOCK_GRGB,			isp_k_cfg_grgb},
	{ISP_BLOCK_ANTI_FLICKER_NEW, isp_k_cfg_anti_flicker_new},
	{ISP_BLOCK_PDAF,			isp_k_cfg_pdaf},
	{ISP_BLOCK_RAW_AFM,		isp_k_cfg_rgb_afm},
	{ISP_BLOCK_RGBG,			isp_k_cfg_rgb_gain},
	{ISP_BLOCK_BLC,			isp_k_cfg_blc},
	{ISP_BLOCK_AWBC,		isp_k_cfg_awbc},
	{ISP_BLOCK_2D_LSC,		isp_k_cfg_2d_lsc},
	{ISP_BLOCK_GAMMA,		isp_k_cfg_gamma},
	{ISP_BLOCK_COMMON,		isp_k_cfg_common},
	{ISP_BLOCK_RAW_AEM,		isp_k_cfg_raw_aem},
};

int isp_cfg_param(void *param,
		struct isp_k_block *isp_k_param, void *isp_handle)
{
	int ret = 0;
	unsigned int i = 0, cnt = 0;
	uint32_t idx = 0;
	isp_cfg_fun_ptr cfg_fun_ptr = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct isp_io_param isp_param = {0, 0, 0, 0, NULL};

	dev = (struct isp_pipe_dev *)isp_handle;
	idx = dev->com_idx;

	/*TBD
	* according to the size transmit from user,
	* to decide which cfg ctx buffer to use.
	*/

	if (!param) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}
	ret = copy_from_user((void *)&isp_param,
			(void *)param, sizeof(isp_param));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = 0x%x\n",
			(uint32_t)ret);
		return -1;
	}
	ISP_SET_SCENE_ID(idx, isp_param.scene_id);
	pr_debug("isp_cfg subblock %d, com_idx 0x%x, scene id %d\n",
		isp_param.sub_block, idx, isp_param.scene_id);

	cnt = ARRAY_SIZE(isp_cfg_fun_tab);
	for (i = 0; i < cnt; i++) {
		if (isp_param.sub_block ==
			isp_cfg_fun_tab[i].sub_block) {
			cfg_fun_ptr = isp_cfg_fun_tab[i].cfg_fun;
			break;
		}
	}

	if (cfg_fun_ptr != NULL)
		ret = cfg_fun_ptr(&isp_param, isp_k_param, idx);


	if (isp_param.sub_block == ISP_BLOCK_3DNR)
		ret = isp_k_cfg_3dnr(&isp_param, isp_k_param, idx, isp_handle);

	return ret;
}
